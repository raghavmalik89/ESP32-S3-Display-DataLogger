# Sensors

## Shared I²C bus

- GPIO47 SDA / GPIO48 SCL, port 0, **100 kHz**, ESP32 internal pull-ups.
- 100 kHz is deliberate (DEF-017 legacy lesson): the external LSM6DS
  breakout has no external pull-ups; internal ones are out of spec at
  400 kHz. Traffic (40 Hz baro + ~200 Hz IMU bursts) is far below budget.
- Bus helper (`i2c_bus.c`) provides reg-8 reads, address probing, and a
  bus-reset (SCL bit-bang) recovery for the MCU-reset slave-wedge fault.

## BMP581 barometer — address 0x47 (verified)

- CHIP_ID register 0x01 = 0x50 (probe-confirmed on the wired board).
- Config: OSR-P 16×, OSR-T 2×, ODR 40 Hz.
- Conversion (raw 24-bit): `Pa = raw / 64`, `°C = raw_t / 65536`.
- **Quirk**: the DRDY flag is *not* cleared by reading data registers.
  Firmware samples on a fixed 40 Hz cadence and never polls DRDY.
- Altitude uses a fixed 101325 Pa reference (from `baro_proc`); a NaN
  guard sits before the estimator bank. Running-mean p0 is forbidden
  by design.

## QMI8658C IMU — address 0x6B (onboard — currently unresponsive)

> **Status 2026-11-03: hardware-level fault.** The part ACKs at 0x6B and
> register 0x02 (CTRL1) reads its true power-on default 0x20 and latches
> written values, but **every other register reads 0x00** (WHO_AM_I,
> REVISION_ID, STATUS0, all data registers) and the sensor never starts.
> Ruled out by dedicated probe firmware (`probe.c`):
> new `i2c_master` driver @ 100 kHz **and** @ 300 kHz (Waveshare demo
> speed), software bit-bang I²C with fully controlled timing (perfect
> ACKs, still 0x00), soft reset (0x60←0xB0), CTRL9 calibrate (0xA2),
> CTRL1 sensor-enable writes (0x60/0x40), two-phase STOP+START reads,
> and a full USB power-cycle of the board. Conclusion: not fixable in
> firmware. Next steps are hardware-side (logic analyzer, scope, or
> board replacement). The logger runs baro-only in the meantime;
> `acq.c` already degrades gracefully (`imu=off` in the `#C` meta).

Historical notes (from when the part responded):

- WHO_AM_I register 0x00 = 0x05.
- Init (from Waveshare's working demo — required, do not skip):
  1. soft reset: write 0xB0 to register 0x60, 50 ms;
  2. on-demand calibration: CTRL9 (0x09) = 0xA2, wait 2200 ms, CTRL9 = 0;
  3. CTRL8 = 0xC0, CTRL1 = 0x60;
  4. CTRL2 = 0x25 (±8 g, 250 Hz), CTRL3 = 0x55 (±500 dps, 250 Hz);
  5. CTRL7 = 0x03 (accel + gyro enable), CTRL5 LPF off.
- **STATUS0 is register 0x2E** — 0x2F is STATUS1 (CMD_DONE/WAKEUP bits).
- Data: 12-byte burst from 0x35 (Ax…Gz), little-endian per axis.
- Scales: ±8 g → 4096 LSB/g; ±500 dps → 65.5 LSB/dps.

## LSM6DSV16X — external, address 0x6A/0x6B (deferred)

- ST official register-level driver + integration layer ported from the
  validated legacy codebase: ±8 g / ±500 dps, 240 Hz ODR (empirically
  better NACK behavior than 120 Hz on the old rig), WHO_AM_I 0x70,
  config readback for the `#C` meta sidecar, runtime link-recovery
  state machine (DEF-020).
- **Not currently fitted on the bench unit** (probe: not present at
  either address). Bring-up deferred; the logger must handle QMI-only.

## Status legend

- BMP581: ✅ verified live on COM6.
- QMI8658C: ❌ part unresponsive (hardware-level, see above); logger
  runs baro-only until hardware investigation.
- LSM6DSV16X: ⏸️ compiled, hardware deferred.
