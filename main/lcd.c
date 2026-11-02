/* lcd.c — SPI panel bring-up for the Waveshare 1.9" 170x320 LCD.
 *
 * Landscape UI: 320 wide x 170 tall.  The 170-column native panel is centered
 * in a 240-wide GRAM with a 35 px offset; in landscape that offset lands on
 * the Y axis (per the official Waveshare ESP-IDF demo flush callback).
 *
 * Panel type: the official ESP-IDF demo for this board drives the panel with
 * an SH8601-style init table; reseller docs say ST7789V2.  Both tables are
 * kept here; select with LCD_PANEL_TYPE.  Default: SH8601 table.
 */
#include "lcd.h"
#include "config.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_lcd_panel_commands.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdlib.h>
#include <string.h>

static const char *TAG = "lcd";

#define LCD_PANEL_SH8601 0
#define LCD_PANEL_ST7789 1
#define LCD_PANEL_TYPE LCD_PANEL_SH8601

esp_lcd_panel_handle_t g_lcd_panel;
esp_lcd_panel_io_handle_t g_lcd_io;

typedef struct { uint8_t cmd; uint8_t data[16]; uint8_t len; uint8_t delay_ms; } lcd_cmd_t;

/* SH8601-style table (Waveshare ESP-IDF 07_LVGL_Test / 08_FactoryProgram),
 * landscape MADCTL 0x70, Y gap 35. */
static const lcd_cmd_t s_init_sh8601[] = {
    {0x36, {0x70}, 1, 0},                                  /* MADCTL landscape */
    {0xB2, {0x0c, 0x0c, 0x00, 0x33, 0x33}, 5, 0},
    {0xB7, {0x35}, 1, 0},
    {0xBB, {0x13}, 1, 0},
    {0xC0, {0x2c}, 1, 0},
    {0xC2, {0x01}, 1, 0},
    {0xC3, {0x0b}, 1, 0},
    {0xC4, {0x20}, 1, 0},
    {0xC6, {0x0f}, 1, 0},
    {0xD0, {0xa4, 0xa1}, 2, 0},
    {0xD6, {0xa1}, 1, 0},
    {0xE0, {0x00, 0x03, 0x07, 0x08, 0x07, 0x15, 0x2A, 0x44,
            0x42, 0x0A, 0x17, 0x18, 0x25, 0x27}, 14, 0},
    {0xE1, {0x00, 0x03, 0x08, 0x07, 0x07, 0x23, 0x2A, 0x43,
            0x42, 0x09, 0x18, 0x17, 0x25, 0x27}, 14, 0},
    {0x21, {0}, 0, 0},                                     /* inversion on */
    {0x11, {0}, 0, 120},                                   /* sleep out */
    {0x29, {0}, 0, 0},                                     /* display on */
};

/* Classic ST7789V table; landscape MADCTL 0x60 (MV|MX), X gap 35 in
 * portrait terms -> kept here as fallback if the SH8601 table misbehaves. */
static const lcd_cmd_t s_init_st7789[] = {
    {0x36, {0x60}, 1, 0},
    {0x3A, {0x55}, 1, 0},                                  /* RGB565 */
    {0xB2, {0x0c, 0x0c, 0x00, 0x33, 0x33}, 5, 0},
    {0xB7, {0x35}, 1, 0},
    {0xBB, {0x13}, 1, 0},
    {0xC0, {0x2c}, 1, 0},
    {0xC2, {0x01}, 1, 0},
    {0xC3, {0x0b}, 1, 0},
    {0xC4, {0x20}, 1, 0},
    {0xC6, {0x0f}, 1, 0},
    {0xD0, {0xa4, 0xa1}, 2, 0},
    {0xE0, {0x00, 0x03, 0x07, 0x08, 0x07, 0x15, 0x2A, 0x44,
            0x42, 0x0A, 0x17, 0x18, 0x25, 0x27}, 14, 0},
    {0xE1, {0x00, 0x03, 0x08, 0x07, 0x07, 0x23, 0x2A, 0x43,
            0x42, 0x09, 0x18, 0x17, 0x25, 0x27}, 14, 0},
    {0x21, {0}, 0, 0},
    {0x11, {0}, 0, 120},
    {0x29, {0}, 0, 0},
};

#if LCD_PANEL_TYPE == LCD_PANEL_SH8601
#define GAP_X 0
#define GAP_Y 35
#else
#define GAP_X 35
#define GAP_Y 0
#endif

static esp_err_t tx_cmd(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    return esp_lcd_panel_io_tx_param(g_lcd_io, cmd, data, len);
}

esp_err_t lcd_init(void)
{
    /* backlight off during init */
    gpio_config_t bl = {
        .pin_bit_mask = 1ULL << LCD_PIN_BL,
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&bl);
    gpio_set_level(LCD_PIN_BL, 0);

    spi_bus_config_t buscfg = {
        .mosi_io_num = LCD_PIN_MOSI,
        .sclk_io_num = LCD_PIN_CLK,
        .miso_io_num = -1,            /* panel is write-only */
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_H_RES * LCD_V_RES * 2,
    };
    esp_err_t err = spi_bus_initialize(LCD_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) return err;

    esp_lcd_panel_io_spi_config_t io_cfg = {
        .dc_gpio_num = LCD_PIN_DC,
        .cs_gpio_num = LCD_PIN_CS,
        .pclk_hz = 20 * 1000 * 1000,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 4,
    };
    err = esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_SPI_HOST,
                                   &io_cfg, &g_lcd_io);
    if (err != ESP_OK) return err;

    /* hardware reset */
    gpio_set_level(LCD_PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(LCD_PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(120));

    const lcd_cmd_t *tbl = (LCD_PANEL_TYPE == LCD_PANEL_SH8601)
                               ? s_init_sh8601 : s_init_st7789;
    size_t n = (LCD_PANEL_TYPE == LCD_PANEL_SH8601)
                   ? (sizeof(s_init_sh8601) / sizeof(lcd_cmd_t))
                   : (sizeof(s_init_st7789) / sizeof(lcd_cmd_t));
    for (size_t i = 0; i < n; i++) {
        err = tx_cmd(tbl[i].cmd, tbl[i].len ? tbl[i].data : NULL, tbl[i].len);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "init cmd 0x%02x failed: %s", tbl[i].cmd,
                     esp_err_to_name(err));
            return err;
        }
        if (tbl[i].delay_ms) vTaskDelay(pdMS_TO_TICKS(tbl[i].delay_ms));
    }

    ESP_LOGW(TAG, "panel table=%s",
             LCD_PANEL_TYPE == LCD_PANEL_SH8601 ? "SH8601" : "ST7789");
    lcd_backlight(true);
    return ESP_OK;
}

/* SPI transmits memory order, MSB first; the panel wants RGB565 high byte
 * first, so every pixel passes through a byte swap on this little-endian
 * MCU.  lcd_blit bounces through a scratch buffer. */
esp_err_t lcd_blit(int x, int y, int w, int h, const uint16_t *pixels)
{
    size_t n = (size_t)w * h;
    uint16_t *swap = malloc(n * 2);
    if (!swap) return ESP_ERR_NO_MEM;
    for (size_t i = 0; i < n; i++) swap[i] = __builtin_bswap16(pixels[i]);

    uint8_t col[4] = { (uint8_t)((x + GAP_X) >> 8), (uint8_t)(x + GAP_X),
                       (uint8_t)((x + w - 1 + GAP_X) >> 8),
                       (uint8_t)(x + w - 1 + GAP_X) };
    uint8_t row[4] = { (uint8_t)((y + GAP_Y) >> 8), (uint8_t)(y + GAP_Y),
                       (uint8_t)((y + h - 1 + GAP_Y) >> 8),
                       (uint8_t)(y + h - 1 + GAP_Y) };
    esp_err_t err = tx_cmd(0x2A, col, 4);
    if (err == ESP_OK) err = tx_cmd(0x2B, row, 4);
    if (err == ESP_OK)
        err = esp_lcd_panel_io_tx_color(g_lcd_io, 0x2C, swap, n * 2);
    free(swap);
    return err;
}

esp_err_t lcd_fill(uint16_t rgb565)
{
    static uint16_t line[LCD_H_RES];
    uint16_t v = __builtin_bswap16(rgb565);
    for (int i = 0; i < LCD_H_RES; i++) line[i] = v;
    for (int y = 0; y < LCD_V_RES; y++) {
        esp_err_t err = lcd_blit(0, y, LCD_H_RES, 1, line);
        if (err != ESP_OK) return err;
    }
    return ESP_OK;
}

void lcd_backlight(bool on) { gpio_set_level(LCD_PIN_BL, on ? 1 : 0); }

void lcd_release(void) { /* kept for probe experiments; not used in app */ }
