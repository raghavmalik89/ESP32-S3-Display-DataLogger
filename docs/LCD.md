# LCD Driver

Target: Waveshare ESP32-S3-LCD-1.9, 170×320 native portrait,
ST7789V2-class SPI panel, write-only (no MISO), RGB565.

## Why software rotation

The controller on this board is an ST7789V2-class part whose landscape
(MV=1) MADCTL modes all produce mirrored output on this glass — verified
on the bench with a labeled-corner test across `0x70`, `0x20`, `0x60`,
`0xA0`, `0x30`. The only manufacturer-validated orientation is the one
the factory firmware uses: **portrait 170×320, MADCTL never written**
(`Direction Normal` in Waveshare `08_FactoryProgram`).

Landscape is therefore done in software, exactly once, in one place:

```
logical (x, y) on 320×170 shadow framebuffer
    native_x = y + 35                    (short-axis GRAM offset)
    native_y = 319 − x                   (CW; flip LCD_ROT_CW for CCW)
```

`LCD_ROT_CW` selects the 90° direction; it is the only knob, and it
cannot introduce mirroring (pure rotation).

## Architecture

- All draw calls (`lcd_fill`, `lcd_blit`, `lcd_char`, `lcd_text`) render
  into a 320×170 RGB565 **shadow framebuffer** (PSRAM, falls back to
  internal DMA RAM) and only expand a dirty rectangle.
- `lcd_flush()` rotates the dirty rect into a DMA-capable bounce buffer
  (`FLUSH_SLICE_W = 24` logical px per slice), byte-swapping each pixel
  to panel order (RGB565 high byte first), then pushes it via
  `esp_lcd_panel_io_tx_color` with explicit `0x2A`/`0x2B` windows.

## Init sequence (factory-validated + one documented addition)

Order matches Waveshare's factory table: `COLMOD 0x3A=0x55`, `B2, B7,
BB, C0, C2, C3, C4, C6, D0, D6, E0, E1`, inversion on `0x21`, sleep out
`0x11` (+120 ms), display on `0x29`. **No MADCTL.**

- `COLMOD` is required: the companion driver Waveshare pairs with this
  table (esp_lcd_sh8601) always sends it before the vendor table; after
  a hardware reset the part is otherwise not in 16-bit mode and command
  bytes get eaten as pixel data (observed as short fills + stray blobs).
- Hardware reset: RST (GPIO9) low 20 ms, high 120 ms.
- Backlight GPIO14 is **active low**: driver holds it high during init,
  pulls low to illuminate (`lcd_backlight(true)` → GPIO 0).

## Gotchas discovered on the bench

1. Backlight polarity inverted vs. intuition — a "completely blank"
   screen can be a fully working panel with the backlight off.
2. RST must be configured as GPIO **output** before bit-banging; a
   toggle on an unconfigured pin is a silent no-op.
3. Fills must not byte-swap twice (colors red↔blue swapped otherwise).
4. Do not chase MADCTL landscape values on this clone — see above.

## Status

Driver implemented and flashed; rotated render awaiting final visual
sign-off. `lcd_flush()` slices full-screen updates (≈14 slices); partial
GUI updates touch far fewer pixels.
