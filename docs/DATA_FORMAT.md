# Data Format — USB Sensor Stream

> **Status: specified, not yet implemented.** The transmission module (`tx.c`)
> is part of the pending production firmware. This document is the authoritative
> spec it will be written against, reconciled with the proven legacy protocol
> (`docs/serial_protocol.md` in the Vario USB workbook, battle-tested on the
> MS5611/MPU6050 rig). Frame conventions below are inherited from that spec;
> payload fields are adapted to this project's BMP581 + QMI8658C hardware.

## Conventions (inherited from the legacy spec)

- Text protocol, one frame per line, `\n`-terminated (host strips `\r`).
  Human-readable, greppable, and fast enough at 921600 baud (~4 % of link
  utilization at planned rates).
- **Baud:** 921600, 8N1. **Clock:** `esp_timer_get_time()` (µs, monotonic).
- **`seq`** — global uint32 monotonic counter, +1 for **every** line emitted,
  of all types. Gaps in `seq` = dropped samples; this is how the host detects loss.
- **`t_us`** — uint64 timestamp taken at the moment the sample was **read**,
  not when the line was printed.
- No filtering, averaging or compensation on the MCU for raw frames — raw
  values only; estimates are emitted as separate, clearly-labelled frames.
- No `printf` debugging to the streaming port mid-capture (breaks the CSV).
- If USB TX blocks, the firmware counts the overflow and emits an error frame —
  it never blocks the sampling timer.

## Boot / meta frames (start with `#`, emitted at startup and on host attach)

| Frame | Fields | Meaning |
|-------|--------|---------|
| `#H` | `fw_name, fw_version` | Banner, e.g. `#H,s3-logger,0.1.0` |
| `#C` | `key=value, ...` | Free-form config. Keys: `p_hz`, `i_hz`, `imu=on/off`, `osr`, `backend=usb/sd/none` |
| `#D` | `key=value, ...` | Device inventory: `bmp581=ok@0x47`, `qmi8658c=ok@0x6B`, `st_imu=absent` |
| `#R` | `reason` | (Re)boot / attach reason: `poweron`, `reset`, `host_attach` |

**Host attach:** opening the CDC port does **not** reset the ESP32-S3. The
firmware re-emits `#H/#C/#D` ~2.5 s after a new host attaches, so a capture
tool that connects to an already-running board still receives meta. (This
behavior and its 2 s flush-window lesson come directly from the legacy rig.)

## Data frames

```
B,<seq>,<t_us>,<p_raw24>,<t_raw24>
M,<seq>,<t_us>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>
V,<seq>,<t_us>,<filter_id>,<vario_raw>,<alt_raw>
E,<seq>,<t_us>,<code>
```

| Field | Type | Notes |
|-------|------|-------|
| `B` | baro | BMP581 raw pressure and temperature, **24-bit raw** (0 … 16777215). Pressure in Pa = `p_raw24 / 64` (fixed-point, see `docs/SENSORS.md`). Raw is preserved so offline reprocessing is possible. |
| `M` | IMU | QMI8658C raw accel + gyro counts, int16. No scaling on the MCU — scale offline (±8 g → 4096 LSB/g; ±500 dps → 65.5 LSB/dps, see `docs/SENSORS.md`). External ST IMU frames (planned) use `sid` to disambiguate. |
| `V` | vario | Per-filter vertical-speed / altitude estimates. `filter_id` ∈ `C03, D01, S4, S5`. Estimates are **additive** — raw `B`/`M` frames continue regardless. |
| `E` | error | See table below. |

## Error codes (`E` frame)

| code | meaning |
|------|---------|
| 1 | I²C NAK / device missing on a scheduled read |
| 2 | I²C timeout / bus reset performed |
| 3 | buffer overflow — samples dropped; expect a `seq` gap after this |

## Session semantics

- Recording state is a **session**, not a file. The host is told the backend
  truthfully: `backend=usb` → **USB STREAM / VOLATILE**; never display "SAVED"
  for a streamed session.
- **Planned v2 frames:** `S` (session start/stop events with session id) and
  `F` (live filter-change events, so a capture is self-describing when the user
  cycles filters mid-recording).

## Worked example (startup → first data)

```
#H,s3-logger,0.1.0
#D,bmp581=ok@0x47,qmi8658c=ok@0x6B,st_imu=absent
#C,p_hz=50,i_hz=200,imu=on,backend=usb
#R,poweron
B,4,18423011,6432819,4194302
M,5,18426000,124,-36,15680,42,-17,9
V,6,18430000,C03,-121,1134280
```
