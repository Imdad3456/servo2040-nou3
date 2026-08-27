/**
 * Servo2040 <-> NOU3 "flash-once" firmware
 *
 * Designed for:
 *   - Pimoroni Servo 2040
 *   - NOU3 as the main brain
 *   - I2C slave on Servo2040 QW/ST connector
 *   - SDA = GP20, SCL = GP21
 *   - I2C address = 0x31
 *   - 18 hobby servos
 *   - 6 sensor / foot-switch inputs
 *
 * Safety:
 *   - Servo PWM is OFF at boot.
 *   - All 18 servo targets must be received before ENABLE=1 succeeds.
 *   - Servo commands are clamped to 500..2500 us.
 *   - A 500 ms command/heartbeat timeout disables all servo PWM.
 *   - After a failsafe, a fresh full set of 18 targets is required.
 *
 * LED states:
 *   GREEN  = firmware running, I2C ready, servos disabled
 *   BLUE   = servo PWM enabled and command link alive
 *   RED    = failsafe timeout occurred
 *   YELLOW = enable was rejected because all 18 targets were not valid
 *   PURPLE = malformed/unknown I2C command
 */

#include "main.h"
#include "pico/i2c_slave.h"
#include "hardware/i2c.h"
#include "hardware/sync.h"

using namespace plasma;
using namespace servo;

// -----------------------------------------------------------------------------
// Configuration
// -----------------------------------------------------------------------------

constexpr uint8_t PROTOCOL_VERSION = 3;

constexpr uint8_t I2C_ADDRESS = 0x31;
constexpr uint32_t I2C_BAUDRATE = 100000;

// Servo2040 QW/ST connector pins.
constexpr uint I2C_SDA_PIN = 20;
constexpr uint I2C_SCL_PIN = 21;

constexpr uint NUM_SERVOS = 18;
constexpr uint NUM_SENSORS = 6;

constexpr uint16_t SERVO_MIN_US = 500;
constexpr uint16_t SERVO_MAX_US = 2500;
constexpr uint16_t SERVO_DEFAULT_US = 1500;

constexpr uint32_t FAILSAFE_TIMEOUT_MS = 500;
constexpr uint32_t SENSOR_UPDATE_MS = 20;   // 50 Hz
constexpr uint32_t POWER_UPDATE_MS = 100;   // 10 Hz
constexpr uint32_t LED_UPDATE_MS = 50;      // 20 Hz
constexpr uint32_t ERROR_LED_HOLD_MS = 1000;

// I2C commands from the NOU3.
enum : uint8_t {
    CMD_SET_ALL         = 0x01, // [cmd][18 x uint16 little-endian]
    CMD_SET_RANGE       = 0x02, // [cmd][start 0..17][count][count x uint16 LE]
    CMD_ENABLE          = 0x03, // [cmd][0 or 1]
    CMD_HEARTBEAT       = 0x04, // [cmd]
    CMD_SELECT_RESPONSE = 0x10, // [cmd][response id]
};

// Response selections. The NOU3 writes CMD_SELECT_RESPONSE first, then performs
// an I2C read from 0x31.
enum : uint8_t {
    RESP_STATUS  = 0x80,
    RESP_SENSORS = 0x81,
    RESP_POWER   = 0x82,
    RESP_TARGETS = 0x83,
};

// Error/status codes.
enum : uint8_t {
    ERR_NONE           = 0,
    ERR_BAD_COMMAND    = 1,
    ERR_BAD_LENGTH     = 2,
    ERR_BAD_RANGE      = 3,
    ERR_ENABLE_DENIED  = 4,
    ERR_COMMAND_OVERRUN= 5,
};

// -----------------------------------------------------------------------------
// Hardware objects
// -----------------------------------------------------------------------------

constexpr int START_PIN = servo2040::SERVO_1;

ServoCluster servos(pio0, 0, START_PIN, NUM_SERVOS);

Analog sensor_adc(servo2040::SHARED_ADC);
Analog voltage_adc(servo2040::SHARED_ADC, servo2040::VOLTAGE_GAIN);
Analog current_adc(
    servo2040::SHARED_ADC,
    servo2040::CURRENT_GAIN,
    servo2040::SHUNT_RESISTOR,
    servo2040::CURRENT_OFFSET
);

AnalogMux mux(
    servo2040::ADC_ADDR_0,
    servo2040::ADC_ADDR_1,
    servo2040::ADC_ADDR_2,
    PIN_UNUSED,
    servo2040::SHARED_ADC
);

WS2812 led_bar(servo2040::NUM_LEDS, pio1, 0, servo2040::LED_DATA);

// -----------------------------------------------------------------------------
// Runtime state
// -----------------------------------------------------------------------------

uint16_t servo_targets[NUM_SERVOS];
uint32_t valid_target_mask = 0;

bool servo_enabled = false;
bool failsafe_latched = false;

uint8_t last_error = ERR_NONE;
uint32_t error_led_until_ms = 0;

uint32_t last_alive_ms = 0;
uint32_t last_sensor_update_ms = 0;
uint32_t last_power_update_ms = 0;
uint32_t last_led_update_ms = 0;

uint16_t sensor_mv[NUM_SENSORS] = {0};
uint8_t sensor_high_mask = 0;

uint16_t supply_mv = 0;
uint16_t current_ma = 0;

// I2C receive frame. Callback runs in ISR context, so it only buffers data.
// The main loop performs servo/sensor work.
constexpr uint8_t RX_BUFFER_SIZE = 64;
volatile uint8_t rx_work[RX_BUFFER_SIZE];
volatile uint8_t rx_work_len = 0;

volatile uint8_t pending_cmd[RX_BUFFER_SIZE];
volatile uint8_t pending_cmd_len = 0;
volatile bool pending_cmd_ready = false;

// Selected response and response byte index.
volatile uint8_t selected_response = RESP_STATUS;
volatile uint8_t response_index = 0;

// -----------------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------------

static inline uint32_t now_ms() {
    return (uint32_t)to_ms_since_boot(get_absolute_time());
}

static inline uint16_t clamp_servo_us(uint16_t value) {
    if (value < SERVO_MIN_US) return SERVO_MIN_US;
    if (value > SERVO_MAX_US) return SERVO_MAX_US;
    return value;
}

static inline bool all_targets_valid() {
    constexpr uint32_t ALL_18 = (1u << NUM_SERVOS) - 1u;
    return (valid_target_mask & ALL_18) == ALL_18;
}

static void set_error(uint8_t error) {
    last_error = error;
    error_led_until_ms = now_ms() + ERROR_LED_HOLD_MS;
}

static void disable_servos(bool because_failsafe) {
    servo_enabled = false;
    servos.disable_all();

    if (because_failsafe) {
        failsafe_latched = true;

        // Require a fresh complete pose before allowing torque again.
        valid_target_mask = 0;
    }
}

static void apply_target(uint index, uint16_t pulse_us) {
    if (index >= NUM_SERVOS) return;

    pulse_us = clamp_servo_us(pulse_us);
    servo_targets[index] = pulse_us;
    valid_target_mask |= (1u << index);

    // This updates the stored pulse even while disabled. If currently enabled,
    // the output is updated immediately.
    servos.pulse(START_PIN + index, pulse_us, servo_enabled);
}

static void set_all_leds(uint8_t r, uint8_t g, uint8_t b) {
    for (uint i = 0; i < servo2040::NUM_LEDS; ++i) {
        led_bar.set_rgb(i, r, g, b);
    }
}

static void update_leds() {
    uint32_t now = now_ms();
    if ((uint32_t)(now - last_led_update_ms) < LED_UPDATE_MS) return;
    last_led_update_ms = now;

    // Temporary command error indication has priority over normal idle/enabled.
    if ((int32_t)(error_led_until_ms - now) > 0) {
        if (last_error == ERR_ENABLE_DENIED) {
            set_all_leds(80, 55, 0);      // yellow
        } else {
            set_all_leds(55, 0, 80);      // purple
        }
        return;
    }

    if (failsafe_latched) {
        set_all_leds(90, 0, 0);           // red
    } else if (servo_enabled) {
        set_all_leds(0, 20, 100);         // blue
    } else {
        set_all_leds(0, 80, 0);           // green
    }
}

static float read_sensor_voltage(uint sensor_index) {
    mux.select(servo2040::SENSOR_1_ADDR + sensor_index);
    return sensor_adc.read_voltage();
}

static float read_supply_voltage() {
    mux.select(servo2040::VOLTAGE_SENSE_ADDR);
    return voltage_adc.read_voltage();
}

static float read_current_amps() {
    mux.select(servo2040::CURRENT_SENSE_ADDR);
    return current_adc.read_current();
}

static void update_sensors() {
    uint32_t now = now_ms();
    if ((uint32_t)(now - last_sensor_update_ms) < SENSOR_UPDATE_MS) return;
    last_sensor_update_ms = now;

    uint8_t mask = 0;

    for (uint i = 0; i < NUM_SENSORS; ++i) {
        float volts = read_sensor_voltage(i);
        if (volts < 0.0f) volts = 0.0f;
        if (volts > 3.3f) volts = 3.3f;

        uint16_t mv = (uint16_t)(volts * 1000.0f + 0.5f);
        sensor_mv[i] = mv;

        // Convenience digital interpretation. Raw millivolts are always
        // available, so the NOU3 can use a different threshold/inversion.
        if (mv >= 1650) {
            mask |= (1u << i);
        }
    }

    sensor_high_mask = mask;
}

static void update_power_readings() {
    uint32_t now = now_ms();
    if ((uint32_t)(now - last_power_update_ms) < POWER_UPDATE_MS) return;
    last_power_update_ms = now;

    float volts = read_supply_voltage();
    if (volts < 0.0f) volts = 0.0f;
    if (volts > 65.535f) volts = 65.535f;
    supply_mv = (uint16_t)(volts * 1000.0f + 0.5f);

    float amps = read_current_amps();
    if (amps < 0.0f) amps = 0.0f;
    if (amps > 65.535f) amps = 65.535f;
    current_ma = (uint16_t)(amps * 1000.0f + 0.5f);
}

static void process_command(const uint8_t *buf, uint8_t len) {
    if (len == 0) return;

    const uint8_t cmd = buf[0];

    switch (cmd) {
        case CMD_SET_ALL: {
            constexpr uint8_t EXPECTED = 1 + NUM_SERVOS * 2;
            if (len != EXPECTED) {
                set_error(ERR_BAD_LENGTH);
                return;
            }

            for (uint i = 0; i < NUM_SERVOS; ++i) {
                uint16_t value =
                    (uint16_t)buf[1 + i * 2] |
                    ((uint16_t)buf[2 + i * 2] << 8);
                apply_target(i, value);
            }

            last_alive_ms = now_ms();
            failsafe_latched = false;
            last_error = ERR_NONE;
            break;
        }

        case CMD_SET_RANGE: {
            if (len < 3) {
                set_error(ERR_BAD_LENGTH);
                return;
            }

            uint8_t start = buf[1];
            uint8_t count = buf[2];

            if (count == 0 || start >= NUM_SERVOS ||
                (uint)start + (uint)count > NUM_SERVOS) {
                set_error(ERR_BAD_RANGE);
                return;
            }

            uint8_t expected = 3 + count * 2;
            if (len != expected) {
                set_error(ERR_BAD_LENGTH);
                return;
            }

            for (uint i = 0; i < count; ++i) {
                uint16_t value =
                    (uint16_t)buf[3 + i * 2] |
                    ((uint16_t)buf[4 + i * 2] << 8);
                apply_target(start + i, value);
            }

            last_alive_ms = now_ms();
            last_error = ERR_NONE;
            break;
        }

        case CMD_ENABLE: {
            if (len != 2) {
                set_error(ERR_BAD_LENGTH);
                return;
            }

            if (buf[1] == 0) {
                disable_servos(false);
                failsafe_latched = false;
                last_error = ERR_NONE;
                return;
            }

            if (!all_targets_valid()) {
                disable_servos(false);
                set_error(ERR_ENABLE_DENIED);
                return;
            }

            // Make sure every channel holds its already-clamped target before
            // torque is enabled.
            for (uint i = 0; i < NUM_SERVOS; ++i) {
                servos.pulse(START_PIN + i, servo_targets[i], false);
            }

            servos.enable_all();
            servo_enabled = true;
            failsafe_latched = false;
            last_error = ERR_NONE;
            last_alive_ms = now_ms();
            break;
        }

        case CMD_HEARTBEAT: {
            if (len != 1) {
                set_error(ERR_BAD_LENGTH);
                return;
            }

            last_alive_ms = now_ms();
            last_error = ERR_NONE;
            break;
        }

        case CMD_SELECT_RESPONSE:
            // Normally handled immediately in ISR so a following read can use
            // the selection without waiting for the main loop.
            if (len != 2) {
                set_error(ERR_BAD_LENGTH);
            }
            break;

        default:
            set_error(ERR_BAD_COMMAND);
            break;
    }
}

// -----------------------------------------------------------------------------
// I2C response byte generator
// -----------------------------------------------------------------------------

static uint8_t response_byte(uint8_t response, uint8_t index) {
    switch (response) {
        case RESP_STATUS: {
            // 10 bytes total:
            // [0] protocol version
            // [1] flags: bit0 enabled, bit1 all-targets-valid,
            //            bit2 failsafe-latched, bit3 I2C-ready(always 1)
            // [2] last_error
            // [3] sensor high bitmask (bits 0..5)
            // [4..5] milliseconds since last motion/heartbeat, uint16 LE
            // [6..7] supply millivolts, uint16 LE
            // [8..9] current milliamps, uint16 LE
            uint8_t flags = 0x08;
            if (servo_enabled) flags |= 0x01;
            if (all_targets_valid()) flags |= 0x02;
            if (failsafe_latched) flags |= 0x04;

            uint32_t age32 = now_ms() - last_alive_ms;
            uint16_t age = age32 > 65535u ? 65535u : (uint16_t)age32;

            switch (index) {
                case 0: return PROTOCOL_VERSION;
                case 1: return flags;
                case 2: return last_error;
                case 3: return sensor_high_mask;
                case 4: return (uint8_t)(age & 0xFF);
                case 5: return (uint8_t)(age >> 8);
                case 6: return (uint8_t)(supply_mv & 0xFF);
                case 7: return (uint8_t)(supply_mv >> 8);
                case 8: return (uint8_t)(current_ma & 0xFF);
                case 9: return (uint8_t)(current_ma >> 8);
                default: return 0;
            }
        }

        case RESP_SENSORS: {
            // 13 bytes:
            // [0] digital-high bitmask
            // [1..12] six sensor values in millivolts, uint16 LE
            if (index == 0) return sensor_high_mask;

            uint8_t data_index = index - 1;
            uint8_t sensor = data_index / 2;
            if (sensor >= NUM_SENSORS) return 0;

            uint16_t value = sensor_mv[sensor];
            return (data_index & 1) ? (uint8_t)(value >> 8)
                                    : (uint8_t)(value & 0xFF);
        }

        case RESP_POWER: {
            // 4 bytes: supply mV LE, current mA LE
            switch (index) {
                case 0: return (uint8_t)(supply_mv & 0xFF);
                case 1: return (uint8_t)(supply_mv >> 8);
                case 2: return (uint8_t)(current_ma & 0xFF);
                case 3: return (uint8_t)(current_ma >> 8);
                default: return 0;
            }
        }

        case RESP_TARGETS: {
            // 36 bytes: 18 x target pulse width in microseconds, uint16 LE
            uint8_t servo = index / 2;
            if (servo >= NUM_SERVOS) return 0;

            uint16_t value = servo_targets[servo];
            return (index & 1) ? (uint8_t)(value >> 8)
                               : (uint8_t)(value & 0xFF);
        }

        default:
            return 0;
    }
}

// -----------------------------------------------------------------------------
// I2C ISR
// -----------------------------------------------------------------------------

static void i2c_slave_handler(i2c_inst_t *i2c, i2c_slave_event_t event) {
    switch (event) {
        case I2C_SLAVE_RECEIVE: {
            uint8_t value = i2c_read_byte_raw(i2c);

            if (rx_work_len < RX_BUFFER_SIZE) {
                rx_work[rx_work_len++] = value;
            }
            break;
        }

        case I2C_SLAVE_REQUEST: {
            uint8_t idx = response_index++;
            i2c_write_byte_raw(i2c, response_byte(selected_response, idx));
            break;
        }

        case I2C_SLAVE_FINISH: {
            // A repeated start also generates FINISH. Reset response index so
            // every master read begins from byte zero.
            response_index = 0;

            if (rx_work_len > 0) {
                // Handle response selection directly so a write+read sequence
                // works immediately.
                if (rx_work_len == 2 &&
                    rx_work[0] == CMD_SELECT_RESPONSE &&
                    rx_work[1] >= RESP_STATUS &&
                    rx_work[1] <= RESP_TARGETS) {

                    selected_response = rx_work[1];
                } else {
                    if (!pending_cmd_ready) {
                        uint8_t len = rx_work_len;
                        for (uint8_t i = 0; i < len; ++i) {
                            pending_cmd[i] = rx_work[i];
                        }
                        pending_cmd_len = len;
                        pending_cmd_ready = true;
                    } else {
                        last_error = ERR_COMMAND_OVERRUN;
                    }
                }
            }

            rx_work_len = 0;
            break;
        }

        default:
            break;
    }
}

// -----------------------------------------------------------------------------
// Initialization
// -----------------------------------------------------------------------------

static void setup_i2c_slave() {
    gpio_init(I2C_SDA_PIN);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);

    gpio_init(I2C_SCL_PIN);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SCL_PIN);

    i2c_init(i2c0, I2C_BAUDRATE);
    i2c_slave_init(i2c0, I2C_ADDRESS, i2c_slave_handler);
}

// -----------------------------------------------------------------------------
// Main
// -----------------------------------------------------------------------------

int main() {
    stdio_init_all();

    // Servo output setup. Never enable torque during boot.
    for (uint i = 0; i < NUM_SERVOS; ++i) {
        servo_targets[i] = SERVO_DEFAULT_US;
    }

    servos.init();
    servos.disable_all();

    // Sensor inputs default to pull-down, matching the original Servo2040
    // Chica driver and working well with switch signals that go high on press.
    for (uint i = 0; i < NUM_SENSORS; ++i) {
        mux.configure_pulls(servo2040::SENSOR_1_ADDR + i, false, true);
    }

    // LEDs are diagnostic and remain useful after the robot is fully assembled.
    led_bar.start();
    set_all_leds(20, 20, 20);  // brief boot indication

    // Start the QW/ST port as an I2C slave at 0x31.
    setup_i2c_slave();

    last_alive_ms = now_ms();
    last_sensor_update_ms = last_alive_ms;
    last_power_update_ms = last_alive_ms;
    last_led_update_ms = 0;

    // Ready: green, PWM disabled.
    set_all_leds(0, 80, 0);

    while (true) {
        // Copy one pending I2C command out of the ISR-owned buffer.
        if (pending_cmd_ready) {
            uint8_t local[RX_BUFFER_SIZE];
            uint8_t len;

            uint32_t irq_state = save_and_disable_interrupts();
            len = pending_cmd_len;
            for (uint8_t i = 0; i < len; ++i) {
                local[i] = pending_cmd[i];
            }
            pending_cmd_ready = false;
            restore_interrupts(irq_state);

            process_command(local, len);
        }

        // Link failsafe only matters while PWM is actively enabled.
        if (servo_enabled) {
            uint32_t age = now_ms() - last_alive_ms;
            if (age > FAILSAFE_TIMEOUT_MS) {
                disable_servos(true);
            }
        }

        update_sensors();
        update_power_readings();
        update_leds();

        tight_loop_contents();
    }
}
