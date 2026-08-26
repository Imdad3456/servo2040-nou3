set(OUTPUT_NAME chica-servo2040)
add_executable(${OUTPUT_NAME} chica-servo2040.cpp)

target_link_libraries(${OUTPUT_NAME}
    pico_stdlib
    pico_i2c_slave
    hardware_i2c
    servo2040
    analogmux
    analog
)

# Runtime control is entirely I2C. USB CDC is deliberately not used, so the
# firmware never waits for or depends on a USB host. BOOTSEL UF2 flashing still
# works because that is provided by the RP2040 ROM bootloader.
pico_enable_stdio_usb(${OUTPUT_NAME} 0)
pico_enable_stdio_uart(${OUTPUT_NAME} 0)

pico_add_extra_outputs(${OUTPUT_NAME})
