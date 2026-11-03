# Button Controls — BOOT / GPIO0

> **Status: specified contract, not yet implemented.** `btn.c` is part of the
> pending production firmware. This document is the behavior it must implement;
> it is a frozen constraint from the Rev 3.0 plan and must not be redesigned.

## Input

Single onboard **BOOT button (GPIO0)**. No external button exists. Active-low,
with debounce.

## Interactions

| Gesture | Effect |
|---------|--------|
| **Short click** (press + release < hold threshold) | Cycle the three GUI screens: Recording/Status → Graph → Vario → back to Recording/Status |
| **10-second hold** on Recording/Status screen | Start / stop recording |
| **10-second hold** on Graph screen | Change the graphed signal |
| **10-second hold** on Vario screen | Cycle vario filter: C03 → D01 → S4 → S5 → C03 |
| **Release before 10 s** during a hold | Cancels the hold action — nothing happens |
| **Tap during startup** | Ignored until the system is initialized **and** the button has been released (prevents a held button at boot from triggering actions) |

## Hold semantics

- The 10-second threshold is measured from **press**, continuously held.
- A hold action fires **once** per hold; the button must be released and
  pressed again to rearm.
- If the hold continues past the 10 s trigger point, nothing further fires —
  one action per hold.

## Recording indicator

A recording indicator (dot/icon) is shown **persistently on every screen**
while a session is active — including after cycling away from the Recording
screen. Stopping the session clears it everywhere.

## Boot-time safety

- The BOOT button doubles as the download-mode strap. A **tap while the
  firmware is running is a normal UI gesture**; holding BOOT **during RESET**
  enters ROM download mode and is not a firmware concern.
- Firmware must not treat "button held at power-on" as a UI gesture — wait for
  release first.

## Screen ↔ hold-action matrix (summary)

| Screen | Short click | 10 s hold |
|--------|-------------|-----------|
| Recording/Status | → Graph | start/stop recording |
| Graph | → Vario | change graph signal |
| Vario | → Recording/Status | cycle filter C03→D01→S4→S5 |
