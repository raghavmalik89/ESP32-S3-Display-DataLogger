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

## QMI8658C IMU — address 0x6B (onboard — recovered 2026-11-03)

> **Status 2026-11-03: ROOT CAUSE FOUND — software, fixed.** The part had
> been declared "hardware-level dead" after reading 0x00 on every register
> except CTRL1. The actual cause was a **swapped register pair in
> `qmi8658c.c`**: CTRL8 and CTRL9 were defined as 0x0A/0x09, but the
> datasheet (Rev 0.9) has **CTRL8 = 0x09 "Motion Detection Control"** and
> **CTRL9 = 0x0A "Host Commands"**. Every boot therefore wrote 0xC0 into
> the host-commands register (an undefined command) and the on-demand
> cali 0xA2 into motion-detection config — re-wedging the chip on every
> reset, which is why even a full USB power-cycle "failed" (the faulty
> init ran again immediately after each boot). With the defines corrected
> the part initializes cleanly: WHO_AM_I 0x05, REVISION_ID 0x7C, fresh
> accel+gyro 15/15 in the probe live-data loop (probe v2), and the
> production stream carries M frames at 200 Hz.
>
> Lesson recorded for future bring-ups: a chip that ACKs and latches
> writes on one register but reads 0x00 everywhere else is not proof of
> silicon death — audit *every* register address in the init sequence
> against the datasheet first (the Waveshare demo's own "CTRL9 (0x09)"
> comment was itself wrong and was inherited verbatim).

Historical notes (from when the part responded):

- WHO_AM_I register 0x00 = 0x05.
- **Register map correction (2026-11-03): CTRL8 = 0x09, CTRL9 = 0x0A**
  (datasheet Rev 0.9). Older notes/demo comments listing CTRL9 = 0x09
  were wrong and caused the wedge described above.
- Init (from Waveshare's working demo — required, do not skip):
  1. soft reset: write 0xB0 to register 0x60, 50 ms;
  2. on-demand calibration: CTRL9 (0x0A) = 0xA2, wait 2200 ms, CTRL9 = 0;
  3. CTRL8 (0x09) = 0xC0, CTRL1 = 0x60;
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
- **Not electrically present on the bench unit** (probe v2, 2026-11-03:
  bus scan ACKs only 0x47 + 0x6B; 0x6A NACKs; ST WHO_AM_I reg 0x0F reads
  0x00 at 0x6B). If the breakout is on the breadboard as intended, it is
  not wired correctly (no ACK on either address) — check power, SDA/SCL
  continuity and SA0. `acq.c` now identity-probes both addresses
  independently, so a correctly wired ST appears without code changes.
  No address conflict: the QMI owns 0x6B and the ST has no ACK anywhere.

## Status legend

- BMP581: ✅ verified live on COM6.
- QMI8658C: ✅ recovered and verified live (root cause was the swapped
  CTRL8/CTRL9 defines; see above). 200 Hz M frames in production stream.
- LSM6DSV16X: ⏸️ compiled, not electrically present (no ACK at 0x6A/0x6B);
  wiring check needed before bring-up can resume.
