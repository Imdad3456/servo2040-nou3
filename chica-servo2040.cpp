#include "pico/stdlib.h"
#include "pico/i2c_slave.h"
#include "hardware/i2c.h"

#include "servo2040.hpp"
#include "ws2812.hpp"

using namespace servo;
using namespace plasma;

constexpr uint8_t I2C_ADDRESS = 0x31;
constexpr uint SDA_PIN = 20;
constexpr uint SCL_PIN = 21;

// Servo2040's six onboard RGB LEDs
WS2812 led_bar(
    servo2040::NUM_LEDS,
    pio1,
    0,
    servo2040::LED_DATA
);

void i2c_handler(i2c_inst_t *i2c, i2c_slave_event_t event) {
    switch (event) {
        case I2C_SLAVE_RECEIVE:
            // Consume whatever the NOU3 sends.
            (void)i2c_read_byte_raw(i2c);
            break;

        case I2C_SLAVE_REQUEST:
            // Return a recognizable test byte.
            i2c_write_byte_raw(i2c, 0xA5);
            break;

        case I2C_SLAVE_FINISH:
            break;

        default:
            break;
    }
}

int main() {
    // Turn RGB LEDs on immediately.
    led_bar.start();

    for (uint i = 0; i < servo2040::NUM_LEDS; i++) {
        led_bar.set_rgb(i, 0, 80, 0);
    }

    // Configure Servo2040 QW/ST connector.
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);

    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    i2c_init(i2c0, 100000);
    i2c_slave_init(i2c0, I2C_ADDRESS, i2c_handler);

    while (true) {
        tight_loop_contents();
    }
}
