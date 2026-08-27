# Servo2040 ↔ NOU3 Flash-Once Firmware

I2C slave address: `0x31`
Servo2040 SDA: GP20
Servo2040 SCL: GP21
Bus speed: 100 kHz
Protocol version: 3

## Boot behavior

- All 18 servo PWM outputs are DISABLED.
- Default stored target for every servo is 1500 us.
- The NOU3 must send valid targets for all 18 channels before ENABLE=1 is accepted.
- Green LEDs mean the firmware is running and I2C is ready.

## Commands written from NOU3 to address 0x31

### 0x01 SET_ALL
37 bytes total:

`01 S1lo S1hi S2lo S2hi ... S18lo S18hi`

Every pulse is little-endian uint16 in microseconds.
Firmware clamps values to 500..2500 us.

This is the recommended normal motion command.

### 0x02 SET_RANGE
`02 start count value0_lo value0_hi ...`

- start = 0..17
- count >= 1
- start + count <= 18

### 0x03 ENABLE
`03 00` = disable all servo PWM
`03 01` = enable servo PWM

ENABLE=1 is rejected until all 18 targets have been supplied.

### 0x04 HEARTBEAT
`04`

Send this whenever the robot is intentionally holding still and SET_ALL is not
being sent frequently enough.

If servo PWM is enabled and neither a valid motion command nor heartbeat arrives
for more than 500 ms, all PWM is disabled.

After a failsafe, the firmware requires a new complete set of 18 targets and an
explicit ENABLE=1.

### 0x10 SELECT_RESPONSE
`10 response_id`

Then read the documented number of bytes from 0x31.

## Responses

### 0x80 STATUS — read 10 bytes
0: protocol version
1: flags
   bit0 = servo PWM enabled
   bit1 = all 18 targets valid
   bit2 = failsafe latched
   bit3 = I2C ready
2: last error
3: six-sensor digital-high bitmask
4..5: ms since last motion/heartbeat, uint16 LE
6..7: measured supply voltage in mV, uint16 LE
8..9: measured current in mA, uint16 LE

Errors:
0 = none
1 = unknown command
2 = bad command length
3 = bad servo range
4 = enable denied because not all 18 targets are valid
5 = command overrun

### 0x81 SENSORS — read 13 bytes
0: digital-high bitmask for sensors 1..6
1..12: six sensor input voltages in millivolts, uint16 LE

The digital bitmask uses 1.65 V as the threshold. The raw millivolt values let
the NOU3 invert the logic or choose a different threshold without reflashing the
Servo2040.

### 0x82 POWER — read 4 bytes
0..1: supply voltage mV, uint16 LE
2..3: current mA, uint16 LE

### 0x83 TARGETS — read 36 bytes
18 stored servo pulse widths in microseconds, uint16 LE.

## LED meanings

GREEN  = firmware running, I2C ready, servos disabled
BLUE   = servo PWM enabled
RED    = failsafe fired
YELLOW = enable rejected because not all 18 targets were supplied
PURPLE = malformed/unknown command

## Intended division of work

Servo2040 permanently handles:
- 18 PWM outputs
- 6 foot/switch sensor inputs
- power telemetry
- I2C communication
- failsafe
- diagnostic LEDs

NOU3 handles:
- calibration
- inverse kinematics
- gait generation
- IMU
- controller input
- foot-contact interpretation
- high-level robot behavior

That means calibration, gait changes, controller changes, and sensor logic can
be changed on the NOU3 without reflashing the Servo2040.
