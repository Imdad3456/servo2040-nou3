/**
 * Flash-once Servo 2040 firmware for a NOU3-controlled hexapod.
 *
 * Architecture:
 *   NOU3 (I2C master) -> QW/ST -> Servo 2040 (I2C slave @ 0x31)
 *   Servo 2040 -> 18 PWM servos + 6 sensor inputs
 *
 * Design goals:
 *   - NEVER enable servo PWM at boot.
 *   - Require valid targets for all 18 servos before ENABLE=1 is accepted.
 *   - Clamp every commanded pulse to 500..2500 us.
 *   - Disable all PWM after 500 ms without a valid motion/heartbeat command.
 *   - Require an explicit re-enable after a failsafe.
 *   - Keep calibration, IK, gait generation, IMU logic, and controller input on NOU3.
 *
 * This replaces the USB CDC command transport used by the Make Your Pet /
 * Chica Servo 2040 driver with an I2C slave transport. It intentionally does
 * not wait for USB and does not need a physical servo-power relay for software
 * torque-disable behavior.
 */

#include <cstdint>

#include "pico/stdlib.h"
#include "pico/i2c_slave.h"
#include "hardware/i2c.h"
#include "hardware/sync.h"

#include "servo2040.hpp"
#include "analogmux.hpp"
#include "analog.hpp"

using namespace servo;

namespace {

// -----------------------------------------------------------------------------
// Protocol / limits
// -----------------------------------------------------------------------------

constexpr uint8_t PROTOCOL_VERSION = 2;
constexpr uint8_t I2C_ADDRESS = 0x31;
constexpr uint I2C_BAUDRATE = 100000;

constexpr uint NUM_SERVOS = 18;
constexpr uint NUM_SENSORS = 6;
constexpr uint16_t SERVO_MIN_US = 500;
constexpr uint16_t SERVO_MAX_US = 2500;
constexpr uint32_t FAILSAFE_TIMEOUT_MS = 500;
constexpr uint32_t SENSOR_UPDATE_PERIOD_MS = 20;  // 50 Hz
constexpr float SENSOR_DIGITAL_THRESHOLD_V = 1.65f;
constexpr uint32_t ALL_TARGETS_MASK = (1u << NUM_SERVOS) - 1u;

// Servo 2040 QW/ST pins.
constexpr uint I2C_SDA_PIN = servo2040::I2C_SDA; // GP20
constexpr uint I2C_SCL_PIN = servo2040::I2C_SCL; // GP21

enum Command : uint8_t {
    CMD_SET_ALL         = 0x01, // [cmd][18 x uint16 little endian]
    CMD_SET_RANGE       = 0x02, // [cmd][start][count][count x uint16]
    CMD_ENABLE          = 0x03, // [cmd][0|1]
    CMD_HEARTBEAT       = 0x04, // [cmd]
    CMD_SELECT_RESPONSE = 0x10  // [cmd][Response]
};

enum Response : uint8_t {
    RESP_STATUS = 0x80, // 8 bytes
    RESP_SENSORS = 0x81, // 13 bytes
    RESP_POWER = 0x82, // 4 bytes
    RESP_TARGETS = 0x83 // 36 bytes
};

enum ErrorCode : uint8_t {
    ERR_NONE = 0,
    ERR_BAD_PACKET = 1,
    ERR_RX_OVERFLOW = 2,
    ERR_TARGETS_NOT_READY = 3,
    ERR_BAD_RESPONSE = 4
};

constexpr uint8_t STATUS_FLAG_ENABLED       = 1u << 0;
constexpr uint8_t STATUS_FLAG_FAILSAFE      = 1u << 1;
constexpr uint8_t STATUS_FLAG_TARGETS_READY = 1u << 2;

// -----------------------------------------------------------------------------
// Servo 2040 hardware
// -----------------------------------------------------------------------------

ServoCluster servos(pio0, 0, servo2040::SERVO_1, NUM_SERVOS);

Analog sensor_adc(servo2040::SHARED_ADC);
Analog voltage_adc(servo2040::SHARED_ADC, servo2040::VOLTAGE_GAIN);
Analog current_adc(servo2040::SHARED_ADC,
                   servo2040::CURRENT_GAIN,
                   servo2040::SHUNT_RESISTOR,
                   servo2040::CURRENT_OFFSET);
AnalogMux mux(servo2040::ADC_ADDR_0,
              servo2040::ADC_ADDR_1,
              servo2040::ADC_ADDR_2,
              PIN_UNUSED,
              servo2040::SHARED_ADC);

// -----------------------------------------------------------------------------
// State
// -----------------------------------------------------------------------------

uint16_t target_pulse_us[NUM_SERVOS] = {0};
uint32_t target_valid_mask = 0;
bool servo_enabled = false;
bool failsafe_active = false;
uint8_t last_error = ERR_NONE;
uint32_t last_valid_command_ms = 0;

// I2C transaction buffer, ISR-owned.
volatile uint8_t rx_buf[64] = {0};
volatile uint8_t rx_len = 0;
volatile bool rx_overflow = false;

// Master-commanded target image, ISR-owned. Keeping a complete image here makes
// multiple range writes accumulate safely even if the main loop has not yet
// consumed the previous write.
volatile uint16_t rx_target_pulse_us[NUM_SERVOS] = {0};
volatile uint32_t rx_target_valid_mask = 0;
volatile bool pending_target_update = false;
volatile bool pending_enable_valid = false;
volatile bool pending_enable = false;
volatile bool pending_liveness = false;
volatile uint8_t pending_protocol_error = ERR_NONE;

// Read selector/cursor, ISR-owned.
volatile uint8_t selected_response = RESP_STATUS;
volatile uint8_t tx_index = 0;

// Stable telemetry snapshots. Main loop updates atomically; ISR only reads.
// STATUS: version, flags, raw-high sensor mask, error, supply mV LE, current mA LE
volatile uint8_t status_response[8] = {0};
// SENSORS: raw-high mask + 6 x sensor mV LE
volatile uint8_t sensor_response[13] = {0};
// POWER: supply mV LE + current mA LE
volatile uint8_t power_response[4] = {0};
// TARGETS: 18 x target PWM us LE
volatile uint8_t target_response[36] = {0};

// -----------------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------------

inline uint16_t clamp_servo_us(uint16_t value) {
    if (value < SERVO_MIN_US) return SERVO_MIN_US;
    if (value > SERVO_MAX_US) return SERVO_MAX_US;
    return value;
}

inline uint16_t clamp_u16(int32_t value) {
    if (value <= 0) return 0;
    if (value >= 65535) return 65535;
    return static_cast<uint16_t>(value);
}

inline uint16_t volts_to_mv(float value) {
    return clamp_u16(static_cast<int32_t>(value * 1000.0f + 0.5f));
}

inline uint16_t amps_to_ma(float value) {
    if (value <= 0.0f) return 0;
    return clamp_u16(static_cast<int32_t>(value * 1000.0f + 0.5f));
}

inline uint16_t read_u16_le(const volatile uint8_t *src) {
    return static_cast<uint16_t>(src[0]) |
           static_cast<uint16_t>(static_cast<uint16_t>(src[1]) << 8);
}

inline void write_u16_le(volatile uint8_t *dst, uint16_t value) {
    dst[0] = static_cast<uint8_t>(value & 0xFFu);
    dst[1] = static_cast<uint8_t>((value >> 8) & 0xFFu);
}

inline bool all_targets_ready(uint32_t mask) {
    return (mask & ALL_TARGETS_MASK) == ALL_TARGETS_MASK;
}

void set_pending_error(uint8_t error) {
    // Keep the newest protocol error. Main loop copies and clears it.
    pending_protocol_error = error;
}

// -----------------------------------------------------------------------------
// I2C slave ISR
// -----------------------------------------------------------------------------

uint8_t response_byte(uint8_t response, uint8_t index) {
    switch (response) {
        case RESP_STATUS:
            return index < sizeof(status_response) ? status_response[index] : 0;
        case RESP_SENSORS:
            return index < sizeof(sensor_response) ? sensor_response[index] : 0;
        case RESP_POWER:
            return index < sizeof(power_response) ? power_response[index] : 0;
        case RESP_TARGETS:
            return index < sizeof(target_response) ? target_response[index] : 0;
        default:
            return 0;
    }
}

bool is_valid_response(uint8_t response) {
    return response == RESP_STATUS ||
           response == RESP_SENSORS ||
           response == RESP_POWER ||
           response == RESP_TARGETS;
}

void parse_finished_write() {
    if (rx_overflow) {
        set_pending_error(ERR_RX_OVERFLOW);
        return;
    }

    if (rx_len == 0) return;

    const uint8_t cmd = rx_buf[0];

    switch (cmd) {
        case CMD_SET_ALL: {
            constexpr uint8_t EXPECTED = 1 + NUM_SERVOS * 2;
            if (rx_len != EXPECTED) {
                set_pending_error(ERR_BAD_PACKET);
                break;
            }

            for (uint i = 0; i < NUM_SERVOS; ++i) {
                rx_target_pulse_us[i] =
                    clamp_servo_us(read_u16_le(&rx_buf[1 + i * 2]));
            }
            rx_target_valid_mask = ALL_TARGETS_MASK;
            pending_target_update = true;
            pending_liveness = true;
            break;
        }

        case CMD_SET_RANGE: {
            if (rx_len < 3) {
                set_pending_error(ERR_BAD_PACKET);
                break;
            }

            const uint8_t start = rx_buf[1];
            const uint8_t count = rx_buf[2];
            const uint16_t expected = static_cast<uint16_t>(3u + 2u * count);

            if (count == 0 ||
                start >= NUM_SERVOS ||
                static_cast<uint16_t>(start) + count > NUM_SERVOS ||
                rx_len != expected) {
                set_pending_error(ERR_BAD_PACKET);
                break;
            }

            for (uint i = 0; i < count; ++i) {
                const uint index = start + i;
                rx_target_pulse_us[index] =
                    clamp_servo_us(read_u16_le(&rx_buf[3 + i * 2]));
                rx_target_valid_mask |= (1u << index);
            }

            pending_target_update = true;
            pending_liveness = true;
            break;
        }

        case CMD_ENABLE:
            if (rx_len != 2 || rx_buf[1] > 1) {
                set_pending_error(ERR_BAD_PACKET);
                break;
            }
            pending_enable = rx_buf[1] != 0;
            pending_enable_valid = true;
            pending_liveness = true;
            break;

        case CMD_HEARTBEAT:
            if (rx_len != 1) {
                set_pending_error(ERR_BAD_PACKET);
                break;
            }
            pending_liveness = true;
            break;

        case CMD_SELECT_RESPONSE:
            if (rx_len != 2) {
                set_pending_error(ERR_BAD_PACKET);
                break;
            }
            if (!is_valid_response(rx_buf[1])) {
                set_pending_error(ERR_BAD_RESPONSE);
                break;
            }
            selected_response = rx_buf[1];
            tx_index = 0;
            // Telemetry selection intentionally does not refresh the servo
            // failsafe. The NOU3 must keep sending motion commands/heartbeats.
            break;

        default:
            set_pending_error(ERR_BAD_PACKET);
            break;
    }
}

void i2c_slave_handler(i2c_inst_t *i2c, i2c_slave_event_t event) {
    switch (event) {
        case I2C_SLAVE_RECEIVE: {
            const uint8_t value = i2c_read_byte_raw(i2c);
            if (rx_len < sizeof(rx_buf)) {
                rx_buf[rx_len++] = value;
            } else {
                rx_overflow = true;
            }
            break;
        }

        case I2C_SLAVE_REQUEST:
            i2c_write_byte_raw(i2c,
                               response_byte(selected_response, tx_index++));
            break;

        case I2C_SLAVE_FINISH:
            parse_finished_write();
            rx_len = 0;
            rx_overflow = false;
            tx_index = 0;
            break;

        default:
            break;
    }
}

void setup_i2c_slave() {
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
// Main-loop tasks
// -----------------------------------------------------------------------------

void disable_servos() {
    servos.disable_all();
    servo_enabled = false;
}

void apply_pending_commands() {
    bool got_target_update = false;
    bool got_enable = false;
    bool enable_value = false;
    bool got_liveness = false;
    uint8_t protocol_error = ERR_NONE;
    uint16_t new_targets[NUM_SERVOS] = {0};
    uint32_t new_valid_mask = 0;

    const uint32_t irq_state = save_and_disable_interrupts();

    if (pending_target_update) {
        for (uint i = 0; i < NUM_SERVOS; ++i) {
            new_targets[i] = rx_target_pulse_us[i];
        }
        new_valid_mask = rx_target_valid_mask;
        pending_target_update = false;
        got_target_update = true;
    }

    if (pending_enable_valid) {
        enable_value = pending_enable;
        pending_enable_valid = false;
        got_enable = true;
    }

    if (pending_liveness) {
        pending_liveness = false;
        got_liveness = true;
    }

    if (pending_protocol_error != ERR_NONE) {
        protocol_error = pending_protocol_error;
        pending_protocol_error = ERR_NONE;
    }

    restore_interrupts(irq_state);

    if (protocol_error != ERR_NONE) {
        last_error = protocol_error;
    }

    if (got_liveness) {
        last_valid_command_ms = to_ms_since_boot(get_absolute_time());
        failsafe_active = false;
        // A successful protocol command clears stale transport errors. An
        // enable-not-ready error remains until a later successful enable.
        if (last_error == ERR_BAD_PACKET ||
            last_error == ERR_RX_OVERFLOW ||
            last_error == ERR_BAD_RESPONSE) {
            last_error = ERR_NONE;
        }
    }

    if (got_target_update) {
        target_valid_mask = new_valid_mask;
        for (uint i = 0; i < NUM_SERVOS; ++i) {
            if ((target_valid_mask & (1u << i)) == 0) continue;
            target_pulse_us[i] = new_targets[i];

            // This mirrors the Make Your Pet driver behavior: when outputs are
            // disabled, ServoCluster stores the pulse but does not load it into
            // the live PIO state. On enable, all 18 stored targets are already
            // ready, avoiding an intermediate jump to a default center pulse.
            // Defer PIO loading until every changed target has been written so
            // all joints in a pose update on the same output frame.
            servos.pulse(servo2040::SERVO_1 + i,
                         target_pulse_us[i],
                         false);
        }
        if (servo_enabled) {
            servos.load();
        }
    }

    if (got_enable) {
        if (!enable_value) {
            disable_servos();
            last_error = ERR_NONE;
        } else if (all_targets_ready(target_valid_mask)) {
            servos.enable_all();
            servo_enabled = true;
            failsafe_active = false;
            last_error = ERR_NONE;
        } else {
            // Safety rule: never energize an incomplete 18-servo pose.
            disable_servos();
            last_error = ERR_TARGETS_NOT_READY;
        }
    }
}

void failsafe_task() {
    if (!servo_enabled) return;

    const uint32_t now_ms = to_ms_since_boot(get_absolute_time());
    if ((now_ms - last_valid_command_ms) > FAILSAFE_TIMEOUT_MS) {
        disable_servos();
        failsafe_active = true;
    }
}

void update_telemetry() {
    uint8_t raw_high_mask = 0;
    uint16_t sensor_mv[NUM_SENSORS] = {0};

    // Sensor polarity is deliberately NOT interpreted here. The NOU3 gets the
    // raw-high mask and the six analog voltages, so NO/NC switch orientation or
    // future FSR thresholds can be changed without reflashing the Servo 2040.
    for (uint i = 0; i < NUM_SENSORS; ++i) {
        mux.select(servo2040::SENSOR_1_ADDR + i);
        const float voltage = sensor_adc.read_voltage();
        sensor_mv[i] = volts_to_mv(voltage);
        if (voltage >= SENSOR_DIGITAL_THRESHOLD_V) {
            raw_high_mask |= static_cast<uint8_t>(1u << i);
        }
    }

    mux.select(servo2040::VOLTAGE_SENSE_ADDR);
    const uint16_t supply_mv = volts_to_mv(voltage_adc.read_voltage());

    mux.select(servo2040::CURRENT_SENSE_ADDR);
    const uint16_t current_ma = amps_to_ma(current_adc.read_current());

    uint8_t flags = 0;
    if (servo_enabled) flags |= STATUS_FLAG_ENABLED;
    if (failsafe_active) flags |= STATUS_FLAG_FAILSAFE;
    if (all_targets_ready(target_valid_mask)) flags |= STATUS_FLAG_TARGETS_READY;

    const uint32_t irq_state = save_and_disable_interrupts();

    status_response[0] = PROTOCOL_VERSION;
    status_response[1] = flags;
    status_response[2] = raw_high_mask;
    status_response[3] = last_error;
    write_u16_le(&status_response[4], supply_mv);
    write_u16_le(&status_response[6], current_ma);

    sensor_response[0] = raw_high_mask;
    for (uint i = 0; i < NUM_SENSORS; ++i) {
        write_u16_le(&sensor_response[1 + i * 2], sensor_mv[i]);
    }

    write_u16_le(&power_response[0], supply_mv);
    write_u16_le(&power_response[2], current_ma);

    for (uint i = 0; i < NUM_SERVOS; ++i) {
        write_u16_le(&target_response[i * 2], target_pulse_us[i]);
    }

    restore_interrupts(irq_state);
}

} // namespace

int main() {
    // Seed both target images at 1500 us, but DO NOT mark any target valid.
    // Therefore ENABLE=1 cannot energize the servos until the NOU3 has supplied
    // a complete 18-servo pose (in one SET_ALL or accumulated SET_RANGE writes).
    for (uint i = 0; i < NUM_SERVOS; ++i) {
        target_pulse_us[i] = 1500;
        rx_target_pulse_us[i] = 1500;
    }

    // Servo outputs are explicitly disabled immediately after initialization.
    servos.init();
    servos.disable_all();
    servo_enabled = false;

    // Pull disconnected sensor inputs low. We still expose raw values to NOU3,
    // so the high-level code can decide whether a particular switch is NO/NC.
    for (uint i = 0; i < servo2040::NUM_SENSORS; ++i) {
        mux.configure_pulls(servo2040::SENSOR_1_ADDR + i, false, true);
    }

    update_telemetry();
    setup_i2c_slave();

    last_valid_command_ms = to_ms_since_boot(get_absolute_time());
    uint32_t last_sensor_update_ms = 0;

    while (true) {
        apply_pending_commands();
        failsafe_task();

        const uint32_t now_ms = to_ms_since_boot(get_absolute_time());
        if ((now_ms - last_sensor_update_ms) >= SENSOR_UPDATE_PERIOD_MS) {
            last_sensor_update_ms = now_ms;
            update_telemetry();
        }

        tight_loop_contents();
    }
}
