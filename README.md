# ESP32-S3 Display DataLogger (Vario)

Portable variometer and sensor logger built on the Waveshare
**ESP32-S3-LCD-1.9** board. A BMP581 barometer and a QMI8658C IMU feed a
bank of four parallel variometer filters (C03, D01, S4, S5); results are
shown on a three-screen LCD UI, sounded on a vario buzzer, streamed over
USB to a host capture tool, and (in software) recorded to SD.

## Objectives

- **Vario core**: timestamped barometric acquisition at 40 Hz + IMU at
  200–250 Hz, four independent estimators evaluated in parallel on every
  sample, one selected to drive display and audio.
- **Field usability**: 320×170 landscape LCD with three screens, BOOT
  button navigation, passive-buzzer audio vario tone.
- **Data out**: loss-checked USB streaming to a Python host capture tool
  (session CSV + metadata sidecar); SD-card recording supported in
  software.
- **Honest engineering**: every feature is labeled verified-on-hardware
  vs. implemented-but-unverified vs. planned (see
  [Development status](#development-status)).

## Hardware

| Item | Detail |
|---|---|
| MCU board | Waveshare ESP32-S3-LCD-1.9 (non-touch), ESP32-S3R8 |
| Flash / PSRAM | 16 MB flash (W25Q128) / 8 MB octal PSRAM |
| LCD | 170×320 ST7789V2-class SPI panel, RGB565 |
| Barometer | Bosch BMP581 breakout (I²C 0x47) |
| IMU (onboard) | QST QMI8658C (I²C 0x6B) |
| IMU (external, deferred) | ST LSM6DSV16X breakout — driver kept, not fitted |
| Buzzer | Passive piezo on GPIO16 (LEDC tone) — implemented, not acoustically verified |
| User input | BOOT button on GPIO0 |
| SD slot | SDMMC 1-bit, onboard — software written, physical test deferred |
| Battery sense | GPIO4 divider (200k/100k, VBAT = VADC × 3) |

## Pinout

| Signal | GPIO | Notes |
|---|---|---|
| I²C SDA / SCL | 47 / 48 | Shared bus: onboard QMI + external BMP581, 100 kHz |
| LCD CLK / MOSI | 10 / 13 | SPI3, write-only |
| LCD DC / CS / RST | 11 / 12 / 9 | |
| LCD backlight | 14 | **Active low** (GPIO low = on) |
| Buzzer | 16 | LEDC PWM, 400–3500 Hz |
| BOOT button | 0 | Short click = screen cycle; 5 s hold = context action |
| WS2812 RGB ×2 | 15 | Reserved |
| SD CLK / CMD / D0 | 41 / 39 / 40 | SDMMC 1-bit |
| Battery ADC | 4 | ADC1_CH3 |

## Firmware architecture

ESP-IDF v6.1 project; two build personalities via `CONFIG_LOGGER_PROBE_MODE`
(set in `sdkconfig.defaults`):

- **Probe mode** (`=y`): bench bring-up app — I²C scan with WHO_AM_I/CHIP_ID
  disambiguation, LCD diagnostic frame, buzzer sweep, 1 Hz sensor heartbeat.
  Currently the flashed build.
- **Logger mode** (production, in progress): sensor acquisition task
  (core 1, prio 20), estimator bank, GUI task, audio, USB TX task
  (core 0, prio 8 — legacy-proven priorities), record/SD state machines.

Module map (all under `main/`):

| Module | Role | Status |
|---|---|---|
| `i2c_bus` | Shared I²C init, reg helpers, bus-reset recovery | Verified |
| `bmp581` | Pressure/temp raw24 driver (from validated legacy code) | Verified on bench |
| `qmi8658c` | Compact QMI driver: soft reset + cal, STATUS0 polling | Driver proven earlier; **part currently unresponsive (hardware)** |
| `imu_lsm6dsv16x` + `lsm6dsv16x_reg` | ST official driver, bus-wedge recovery | Compiled; hardware deferred |
| `lcd` + `font8x8_basic` | Panel driver + 8×8 font (see `docs/LCD.md`) | Verified (user sign-off) |
| `filters` | C03/D01/S4/S5 estimator bank, host-verified vs Python reference | Verified |
| `acq` | 40 Hz baro (+200 Hz IMU when present) acquisition, never blocks | Verified baro-only on bench |
| `tx` | USB text-protocol TX, per-stream seq, host handshake, SD mirror | Verified with `tools/capture.py` |
| `gui` | 10 fps three-screen render, REC chrome, hold-progress bar | Flashed; button bench pending |
| `btn` | BOOT gestures: click cycles screens, 5 s holds act once | Flashed; button bench pending |
| `audio` | LEDC vario tone, hysteresis latch | Implemented; not acoustically verified |
| `rec` | Recording state machine (IDLE / USB STREAM / USB+HOST / SD SAVED) | Flashed; bench pending |
| `sd_sink` | SDMMC 1-bit backend, graceful when no card | Compiled; physically untested |
| `app_state` | Shared application state | — |
| `probe` | Bench diagnostics (build-time `CONFIG_LOGGER_PROBE_MODE`) | Verified |

## GUI (three screens)

Design target, 320×170 landscape:

1. **Status / Recording** — filter name, battery, sensor health,
   record state; 5 s hold starts/stops recording.
2. **Graph** — scrolling signal trace (pressure / altitude / vario
   selectable); 5 s hold changes the graphed signal.
3. **Vario** — large numeric climb/sink readout; 5 s hold cycles the
   selected filter C03 → D01 → S4 → S5 → C03.

A record dot is persistent on **every** screen while recording.
Short BOOT click cycles screens; all interactions ignore the button until
released once after boot (see `docs/CONTROLS.md`).

## Vario filters (four parallel)

Every baro sample feeds all four estimators simultaneously (double
precision); the selected one drives display + audio, and host frames
report per-filter vario values:

| Filter | Lineage |
|---|---|
| C03 | Complementary filter, q=1.0 r=0.035 (legacy default) |
| D01 | Double-pole IIR variant (legacy validated) |
| S4 | State-space estimator (legacy validated) |
| S5 | LightGate estimator + audio hysteresis state machine + fault detector (`vario_core`) |

Audio policy (from the validated legacy baseline): lift-on 0.20 m/s,
lift-off 0.05 m/s hysteresis; tone = 500 + 450·v Hz, clamped 400–3500 Hz
in 25 Hz steps; muted on fault or stale data.

## Recording

- **USB streaming**: ring-buffered USB Serial/JTAG at 921600 baud;
  host handshake (`G`) + arm/ack; sequence counters let the capture tool
  detect gaps. Protocol: `docs/DATA_FORMAT.md`.
- **SD recording**: code-only in this phase (init/detect/write/flush);
  display shows `SAVED` only for actual SD writes, `USB STREAM · UNSAVED`
  for host-armed streaming without a card. **No card has been inserted
  yet — physically untested.**

## Build instructions

Windows (Git Bash) with ESP-IDF v6.1 — this machine's stock `export.bat`
is bypassed by a project wrapper (sets tools paths, `ESP_ROM_ELF_DIR`):

```bash
# from the project root
cmd //c "set MSYSTEM=&& tools/win/idfenv.bat build"      # build
cmd //c "set MSYSTEM=&& tools/win/idfenv.bat -p COM6 flash"
# one-shot serial capture (resets via esptool when NO_RESET is unset)
"C:/Espressif/python_env/idf6.1_py3.14_env/Scripts/python.exe" \
    tools/win/read_serial.py COM6 20
```

Probe vs. logger build: edit `CONFIG_LOGGER_PROBE_MODE` in
`sdkconfig.defaults`, delete `sdkconfig`, rebuild. Console is on
USB Serial/JTAG (INFO level); the host capture tool tolerates stray
console lines.

## Development status

| Feature | State |
|---|---|
| I²C bus @ 100 kHz, BMP581 @ 0x47 (CHIP_ID 0x50) | ✅ Verified on COM6 bench |
| LCD: landscape via software rotation, correct text/corners | ✅ Verified (user sign-off) |
| Production firmware (acq→filters→GUI→audio→USB) | ✅ Flashed; baro-only stream verified |
| USB streaming: B 40 Hz + V 20 Hz, 0 gaps, meta + handshake | ✅ Verified via `tools/capture.py` |
| QMI8658C @ 0x6B | ❌ Part unresponsive — reads 0x00 on all registers except CTRL1; proven **not** a firmware issue (driver, speed, bit-bang, reset, power-cycle all ruled out). Hardware investigation pending. Logger runs baro-only. See `docs/SENSORS.md` |
| GUI Rev 2 (mockup layouts, 300 s history + window cycling, storage dot, SD capacity bar, time-left estimate) | ✅ Flashed, streaming; visual sign-off + button bench pending |
| Vario audio (LEDC buzzer, hysteresis latch) | ⚠️ Implemented; **not acoustically verified** |
| SD backend | ✅ Card mounts (32 GB SDHC); write/flush/close paths instrumented, **session bench test pending** |
| External ST LSM6DSV16X | ⏸️ Driver compiled; bring-up deferred |
| Battery, enclosure, PCB | 📋 Deferred |

Legend: ✅ verified on hardware · ⚠️ implemented, verification pending ·
⏸️ deferred · 📋 planned.

## Documentation

- [`docs/LCD.md`](docs/LCD.md) — panel bring-up, why software rotation
- [`docs/SENSORS.md`](docs/SENSORS.md) — I²C bus, BMP581, QMI8658C
- [`docs/DATA_FORMAT.md`](docs/DATA_FORMAT.md) — USB protocol frames
- [`docs/CONTROLS.md`](docs/CONTROLS.md) — BOOT button interactions
