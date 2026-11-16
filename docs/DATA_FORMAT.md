# Data Format — USB Sensor Stream

> **Status: implemented** (firmware `tx.c`, host tool `tools/capture.py`).
> Conventions are inherited from the proven legacy protocol (`docs/serial_protocol.md`
> in the Vario USB workbook, battle-tested on the MS5611/MPU6050 rig);
> payload fields are adapted to this project's BMP581 (+QMI8658C) hardware.

## Conventions

- Text protocol, one frame per line, `\n`-terminated. **Baud:** 921600, 8N1.
  **Clock:** `esp_timer_get_time()` (µs, monotonic).
- **`seq`** — per-stream uint32 monotonic counter (+1 for every frame of that
  stream). Host detects loss per stream; counters reset to 0 on every `#R`
  meta re-emit (host treats a lower-than-last seq as a reset, not a gap).
- **`t_us`** — uint64 timestamp taken when the sample was **read**.
- Raw frames carry raw values only; estimates are separate, labelled frames.
- The streaming port emits **protocol frames only** while a host is attached
  (`usb_serial_jtag_is_connected()`); the firmware health log is suppressed
  so the CSV never sees stray lines.
- USB TX never blocks sampling: on overflow the frame is dropped and
  counted (`g_app.baro_dropped` / `imu_dropped`).

## Boot / meta frames (start with `#`, emitted at startup and on host attach)

| Frame | Fields | Meaning |
|-------|--------|---------|
| `#H` | `fw_name, fw_version` | Banner, e.g. `#H,s3-logger,0.1.0` |
| `#D` | `key=value, ...` | Device inventory, e.g. `bmp581=ok@0x47,qmi8658c=absent,st_imu=absent@0x6b,sd=absent` |
| `#C` | `key=value, ...` | Config: `p_hz`, `i_hz`, `imu=on/off`, `backend=usb/sd/none`, `filters=C03/D01/S4/S5` |
| `#R` | `reason` | (Re)boot / attach reason: `poweron`, `host_attach` |
| `#I` | see below | 1 Hz I²C timing instrumentation (bench), Firmware Reliability Rev 1 |

`#I` fields (per-second snapshot; durations include in-driver retries):
`#I,usec, b,n,fail,max_us,p99_us, q,..., s,..., stq,stale_qmi,sts,stale_st,
jm,jit_max_us,ja,jit_avg_us,qh,tx_q_high,nak,tmo,ovf,bd,baro_drops,id,imu_drops`
Device order is `b`=BMP581, `q`=QMI8658C, `s`=ST IMU.  Counters after the
histograms are cumulative; host tools diff consecutive `#I` frames.

**Host attach:** opening the CDC port does **not** reset the ESP32-S3. The
firmware re-emits meta ~2.5 s after a new host attaches, so a capture tool
that connects to an already-running board still receives the inventory.

**Host commands** (one line per command, `\n`-terminated; each is acked
with a `#K,...` line that bypasses the recorder):
`G,0/1` confirmed-capture flag · `C,0/1` stop/start a recording session
(same lifecycle as the 5 s BOOT hold) · `Q,0/1` QMI8658C reads on/off ·
`T,0/1` ST IMU reads on/off · `P,0/1` BMP581 reads on/off (bench only;
the buzzer is vetoed while the baro is off).

## Data frames

```
B,<seq>,<t_us>,<p_raw24>,<t_raw24>
M,<seq>,<t_us>,<sid>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>
V,<seq>,<t_us>,<filter_id>,<vs_mm_s>,<alt_mm>
E,<seq>,<t_us>,<code>
S,<seq>,<t_us>,<start>,<session_id>
F,<seq>,<t_us>,<filter_id>
```

| Frame | Meaning |
|-------|---------|
| `B` | BMP581 raw pressure/temperature, 24-bit raw. Pa = `p_raw24 / 64`; °C = `t_raw24 / 65536`. Emitted at 40 Hz. |
| `M` | IMU raw accel+gyro int16 counts, emitted at 200 Hz when an IMU is live (`sid` disambiguates onboard vs external). Currently absent (see `docs/SENSORS.md`). |
| `V` | Selected-filter estimate at 20 Hz. `filter_id` ∈ `C03, D01, S4, S5`; climb/sink in **mm/s**, altitude in **mm** (integer frames; divide by 1000 offline). |
| `E` | Error event — see table below. |
| `S` | Recording session start/stop (`start` = 1/0) with `session_id`; emitted by the record state machine. |
| `F` | Live filter-change event, so a capture is self-describing when the user cycles filters mid-recording. |

## Error codes (`E` frame)

| code | meaning |
|------|---------|
| 1 | I²C NAK / device missing on a scheduled read |
| 2 | I²C timeout / bus reset performed |
| 3 | buffer overflow — samples dropped; expect a seq gap after this |

## Session semantics

- Recording state is a **session**, not a file. The firmware reports the
  backend truthfully: `backend=usb` → **USB STREAM / VOLATILE**; "SAVED" is
  only shown for an SD session that was actually flushed to card.

## Worked example (startup → first data)

```
#H,s3-logger,0.1.0
#D,bmp581=ok@0x47,qmi8658c=absent,st_imu=absent@0x6b,sd=absent
#C,p_hz=40,i_hz=200,imu=off,backend=usb,filters=C03/D01/S4/S5
#R,poweron
B,1,18423011,6203808,4194302
V,1,18430000,C03,-12,386000
```
