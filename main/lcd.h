/* lcd.h — 320x170 landscape LCD over esp_lcd SPI panel-IO.
 *
 * The Waveshare ESP32-S3-LCD-1.9 panel is write-only SPI (no MISO), so the
 * controller cannot be ID-read from the MCU.  Two proven init tables are
 * compiled in (Waveshare's official ESP-IDF demo table and the classic
 * ST7789 table); LCD_PANEL_* selects which one is used.  Default follows the
 * official ESP-IDF factory demo for this exact board.
 */
#ifndef LCD_H
#define LCD_H

#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include <stdint.h>

esp_err_t lcd_init(void);
/* Push the dirty region of the shadow framebuffer to the panel (rotated).
 * All draw calls only mark dirty; one flush per frame. */
esp_err_t lcd_flush(void);esp_err_t lcd_fill(uint16_t rgb565);
/* Draw a bitmap at (x,y), size w*h, pixels row-major RGB565. */
esp_err_t lcd_blit(int x, int y, int w, int h, const uint16_t *pixels);
/* 8x8 text helpers (font8x8_basic).  Text does not wrap — caller keeps
 * strings within LCD_H_RES/8 columns.  Panel is write-only, so bg is
 * always a solid color (no transparency). */
esp_err_t lcd_char(int x, int y, char c, uint16_t fg, uint16_t bg);
esp_err_t lcd_text(int x, int y, const char *s, uint16_t fg, uint16_t bg);
void lcd_backlight(bool on);
void lcd_release(void);   /* tear down (probe mode switches tables) */

/* RGB565 helpers */
#define LCD_RGB565(r, g, b) \
    (uint16_t)((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | ((b) >> 3))
#define LCD_BLACK   0x0000
#define LCD_WHITE   0xFFFF
#define LCD_RED     0xF800
#define LCD_GREEN   0x07E0
#define LCD_BLUE    0x001F
#define LCD_YELLOW  0xFFE0
#define LCD_CYAN    0x07FF
#define LCD_GRAY    0x8410

/* software-rotation direction (see lcd.c) */
#ifndef LCD_ROT_CW
#define LCD_ROT_CW 1
#endif

extern esp_lcd_panel_handle_t g_lcd_panel;
extern esp_lcd_panel_io_handle_t g_lcd_io;

#endif
