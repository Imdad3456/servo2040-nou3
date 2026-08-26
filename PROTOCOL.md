# NOU3 ↔ Servo 2040 protocol v2

Servo 2040 I2C slave address: `0x31`  
Bus: QW/ST / Qwiic, 100 kHz  
Servo 2040 pins: GP20 = SDA, GP21 = SCL

All 16-bit values are little-endian.

## Safety behavior

- PWM is OFF at boot.
- `ENABLE=1` is rejected until all 18 servo targets have been supplied.
- Every servo target is clamped to 500–2500 µs.
- If enabled and no valid SET/ENABLE/HEARTBEAT command arrives for 500 ms, all PWM is disabled.
- After a failsafe the NOU3 must send `ENABLE=1` again; motion never re-arms automatically.
- Sensor polarity/threshold decisions stay on the NOU3. Servo 2040 reports raw sensor voltages plus a simple 1.65 V raw-high mask.

## Commands: NOU3 writes to 0x31

| Command | Bytes | Meaning |
|---|---:|---|
| `0x01` | 37 | Set all 18 servo targets: command + 18×uint16 pulse µs |
| `0x02` | 3 + 2×count | Set contiguous range: command, start, count, pulses |
| `0x03` | 2 | Enable/disable: command, `0` or `1` |
| `0x04` | 1 | Heartbeat |
| `0x10` | 2 | Select response for next read: command, response ID |

`SET_RANGE` can be used before the first enable; the firmware tracks which of the 18 targets have been supplied and only allows enable after all 18 are valid.

## Responses

First write `0x10 <response-id>`, then read the documented number of bytes.

### `0x80` STATUS — 8 bytes

| Byte | Meaning |
|---:|---|
| 0 | protocol version (`2`) |
| 1 | flags: bit0 enabled, bit1 failsafe, bit2 all targets ready |
| 2 | six-bit raw-high sensor mask |
| 3 | last error code |
| 4–5 | servo supply voltage, mV |
| 6–7 | servo current, mA |

Error codes: `0` none, `1` bad packet, `2` RX overflow, `3` enable rejected because targets are incomplete, `4` invalid response selector.

### `0x81` SENSORS — 13 bytes

- byte 0: raw-high mask
- bytes 1–12: six sensor voltages in mV (uint16 each)

### `0x82` POWER — 4 bytes

- bytes 0–1: supply mV
- bytes 2–3: current mA

### `0x83` TARGETS — 36 bytes

18 commanded pulse widths in µs, uint16 each.
