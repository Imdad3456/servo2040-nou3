# Servo 2040 — NOU3 flash-once firmware

This is the Servo 2040 side of the NOU3 + Servo2040 hexapod architecture.
It is intended to be flashed once and then left alone while calibration, IK,
gaits, IMU/balance logic, controller input, and foot-contact interpretation are
implemented on the NOU3.

## What it does

- Drives all 18 Servo2040 PWM channels.
- Exposes all six Servo2040 sensor inputs to NOU3.
- Exposes Servo2040 voltage/current sensing.
- Acts as I2C slave `0x31` on the QW/ST connector.
- Boots with all servo PWM disabled.
- Refuses to enable until a complete 18-servo target pose has been received.
- Clamps servo pulses to 500–2500 µs.
- Disables all PWM if NOU3 commands stop for 500 ms.
- Requires explicit re-enable after a failsafe.
- Does not wait for USB and does not need USB during operation.

## Wiring assumed

Between NOU3 and Servo2040:

- SDA ↔ SDA
- SCL ↔ SCL
- GND ↔ GND
- Qwiic red/3.3 V conductor left disconnected in the wiring plan we established
- separate regulated NOU3 5 V → Servo2040 5 V logic rail

Servo power stays on the Servo2040 external/servo-power rail. If that rail is
above 5 V, follow Pimoroni's hardware instruction for separating USB/logic from
external servo power before applying the higher voltage.

## Calibration

No calibration table is compiled into this firmware. That is intentional.
The NOU3 will convert calibrated joint angles into final PWM microseconds and
send those 18 values to this board. Replacing/recalibrating a servo therefore
does not require reflashing the Servo2040.

## Build against the Make Your Pet driver repository

The supplied `chica-servo2040.cpp` and `chica-servo2040.cmake` are drop-in
replacements for the files with those names in the repository's
`chica-servo2040/` directory.

The CMake target additionally links `pico_i2c_slave` and `hardware_i2c`.
USB/UART stdio are disabled because runtime control is I2C-only.

See `PROTOCOL.md` for the NOU3-side packet format.

## Build/validation status

The source has passed a strict host-side syntax/API-shape check. This environment
cannot run the real ARM/Pico cross-build, so no unverified UF2 is included. The
included `.github/workflows/build-servo2040.yml` performs the real build and uploads
the resulting UF2. See `VALIDATION.md`.
