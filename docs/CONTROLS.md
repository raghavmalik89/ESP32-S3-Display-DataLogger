# Button Controls — BOOT / GPIO0

> **Status: implemented** (`btn.c`). Frozen interaction contract from the
> Rev 3.0 plan, updated by GUI Rev 2 (graph hold now cycles the time
> window instead of the signal — signal cycling was removed with the
> single-vertical-speed trace).

## Input

Single onboard **BOOT button (GPIO0)**. No external button exists. Active-low,
with debounce.

## Interactions

| Gesture | Effect |
|---------|--------|
| **Short click** (press + release < hold threshold) | Cycle the three GUI screens: Recording/Status → Graph → Vario → back to Recording/Status |
| **5-second hold** on Recording/Status screen | Start / stop recording |
| **5-second hold** on Graph screen | Cycle the history window: 10 s → 30 s → 60 s → 120 s → 300 s → 10 s… (default 60 s) |
| **5-second hold** on Vario screen | Cycle vario filter: C03 → D01 → S4 → S5 → C03 |
| **Release before 10 s** during a hold | Cancels the hold action — nothing happens |
| **Tap during startup** | Ignored until the system is initialized **and** the button has been released (prevents a held button at boot from triggering actions) |

## Hold semantics

- The 10-second threshold is measured from **press**, continuously held.
- A hold action fires **once** per hold; the button must be released and
  pressed again to rearm.
- If the hold continues past the 10 s trigger point, nothing further fires —
  one action per hold.

## Recording indicator

A storage dot is shown **persistently on every screen**, same position
(top right), driven by actual backend state (GUI Rev 2 honesty contract):

| Color | Meaning |
|-------|---------|
| **RED** | Confirmed persistent SD write active and succeeding (session file open, fwrite-confirmed bytes flowing, no SD error) |
| **GREEN** | Writable media available (SD mounted), no recording in progress |
| **YELLOW** | Warning: no card, volatile USB-only stream (incl. host-confirmed), or SD fault |

A REC command never alone turns the dot red: a failed SD session start
falls back to the volatile USB stream and stays yellow; red returns to
green only after a successful flush + close.

## Boot-time safety

- The BOOT button doubles as the download-mode strap. A **tap while the
  firmware is running is a normal UI gesture**; holding BOOT **during RESET**
  enters ROM download mode and is not a firmware concern.
- Firmware must not treat "button held at power-on" as a UI gesture — wait for
  release first.

## Screen ↔ hold-action matrix (summary)

| Screen | Short click | 5 s hold |
|--------|-------------|-----------|
| Recording/Status | → History | start/stop recording |
| History | → Vario | cycle window 10→30→60→120→300 s |
| Vario | → Recording/Status | cycle filter C03→D01→S4→S5 |
