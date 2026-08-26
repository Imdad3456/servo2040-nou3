# Validation status

## Completed here

- Reworked the first pass into protocol v2 with stricter startup safety.
- Verified the Servo2040 hardware assumptions against the Make Your Pet driver:
  18-channel `ServoCluster`, six muxed sensor inputs, and voltage/current ADCs.
- Verified QW/ST pin use: GP20 SDA and GP21 SCL.
- Verified the Raspberry Pi Pico SDK I2C slave event API shape used by the code.
- Host C++17 syntax check passes with `-Wall -Wextra -Wpedantic -Werror` against API stubs matching the methods used by the Make Your Pet/Pimoroni code.
- Removed the USB CDC runtime dependency.
- Added full-target arming protection, pulse clamps, synchronized ServoCluster loading, and a 500 ms communications failsafe.

## Not completed in the current execution environment

A real RP2040 `.uf2` was not produced locally because this runtime does not contain
the ARM embedded GCC/Pico SDK/Pimoroni build toolchain and outbound shell network
access is unavailable. A `.uf2` should not be fabricated or labeled verified without
that real cross-build.

The included GitHub Actions workflow performs the real cross-build against Eddie
Carrera's Make Your Pet Servo2040 repository plus Pico SDK 1.5.1 and emits
`Servo2040_NOU3_FlashOnce.uf2` as an artifact.
