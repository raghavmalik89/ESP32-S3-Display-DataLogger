/* lcd.c — SPI panel for the Waveshare ESP32-S3-LCD-1.9 (ST7789V2-class,
 * 170x320 native portrait glass).
 *
 * Validated reference: Waveshare 08_FactoryProgram ships with
 * `Direction Normal` = 170x320 PORTRAIT, NO MADCTL command sent, flush
 * offset x+35.  Every MADCTL landscape variant (0x70/0x20/0x60/0xA0/0x30)
 * produced mirrored output on this exact panel, so landscape is done in
 * SOFTWARE: the panel stays in the factory configuration and a single,
 * documented 90-degree transform maps the logical 320x170 landscape
 * framebuffer to the native portrait GRAM at flush time.
 *
 * Logical -> native transform (LCD_ROT_CW=1):
 *      native_x = logical_y + 35            (offset on the short axis)
 *      native_y = 319 - logical_x
 * Flip LCD_ROT_CW to 0 for the opposite rotation direction.
 *
 * Rendering path: lcd_blit()/lcd_fill()/lcd_text() draw into a full-screen
 * landscape shadow framebuffer and only mark a dirty rectangle; lcd_flush()
 * rotates the dirty region into a DMA-capable bounce buffer and sends it
 * through the esp_lcd SPI panel-IO.  The byte swap to panel-endian (RGB565
 * high byte first) happens while filling the bounce buffer.
 */
#include "lcd.h"
#include "config.h"
#include "font8x8_basic.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_commands.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <stdlib.h>
#include <string.h>

static const char *TAG = "lcd";

esp_lcd_panel_handle_t g_lcd_panel;
esp_lcd_panel_io_handle_t g_lcd_io;

#define LCD_NATIVE_W    170
#define LCD_NATIVE_H    320
#define LCD_NATIVE_XOFF 35

/* flush slice width (logical px); bounds the bounce buffer size */
#define FLUSH_SLICE_W 24
/* max dirty height is LCD_V_RES (170); bounce = W*H uint16 in DMA RAM.
 * TWO slices so the CPU can pack slice N+1 while DMA still reads slice N. */
static uint16_t s_bounce[2][FLUSH_SLICE_W * LCD_V_RES];

/* DMA completion bookkeeping: a counting semaphore owns the two bounce
 * buffers.  Take = buffer free, give (from the panel-IO color-done
 * callback) = its DMA transfer finished.  A buffer is never refilled
 * while its transfer is outstanding. */
static SemaphoreHandle_t s_buf_sem;
static int s_buf_idx;

static bool on_color_trans_done(esp_lcd_panel_io_handle_t panel_io,
                                esp_lcd_panel_io_event_data_t *edata,
                                void *user_ctx)
{
    BaseType_t hp = pdFALSE;
    xSemaphoreGiveFromISR(s_buf_sem, &hp);
    return hp == pdTRUE;
}

/* landscape shadow framebuffer (PSRAM preferred) */
static uint16_t *s_shadow;

/* dirty rectangle in logical coords, inclusive */
static int s_dx0, s_dy0, s_dx1, s_dy1;
static bool s_dirty;

typedef struct { uint8_t cmd; uint8_t data[16]; uint8_t len; uint8_t delay_ms; } lcd_cmd_t;

/* Init table = Waveshare factory table (08_FactoryProgram, Direction
 * Normal) with COLMOD 0x3A=0x55 re-inserted right after reset — the
 * esp_lcd_sh8601 driver that Waveshare pairs with this table always sends
 * COLMOD before the vendor commands; without it the part is not in
 * 16-bit mode after HW reset.  No MADCTL: factory-validated default. */
static const lcd_cmd_t s_init[] = {
    {0x3A, {0x55}, 1, 0},                                  /* COLMOD RGB565 */
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

static esp_err_t tx_cmd(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    return esp_lcd_panel_io_tx_param(g_lcd_io, cmd, data, len);
}

static void dirty_mark(int x, int y, int w, int h)
{
    if (w <= 0 || h <= 0) return;
    if (!s_dirty) {
        s_dx0 = x; s_dy0 = y; s_dx1 = x + w - 1; s_dy1 = y + h - 1;
        s_dirty = true;
    } else {
        if (x < s_dx0) s_dx0 = x;
        if (y < s_dy0) s_dy0 = y;
        if (x + w - 1 > s_dx1) s_dx1 = x + w - 1;
        if (y + h - 1 > s_dy1) s_dy1 = y + h - 1;
    }
}

esp_err_t lcd_init(void)
{
    /* backlight OFF during init — GPIO14 is active LOW on this board */
    gpio_config_t bl = {
        .pin_bit_mask = 1ULL << LCD_PIN_BL,
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&bl);
    gpio_set_level(LCD_PIN_BL, 1);

    gpio_config_t rst = {
        .pin_bit_mask = 1ULL << LCD_PIN_RST,
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&rst);

    s_shadow = heap_caps_malloc(LCD_H_RES * LCD_V_RES * sizeof(uint16_t),
                                MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_shadow)
        s_shadow = heap_caps_malloc(LCD_H_RES * LCD_V_RES * sizeof(uint16_t),
                                    MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    if (!s_shadow) {
        ESP_LOGE(TAG, "shadow framebuffer alloc failed");
        return ESP_ERR_NO_MEM;
    }
    memset(s_shadow, 0, LCD_H_RES * LCD_V_RES * sizeof(uint16_t));

    spi_bus_config_t buscfg = {
        .mosi_io_num = LCD_PIN_MOSI,
        .sclk_io_num = LCD_PIN_CLK,
        .miso_io_num = -1,            /* panel is write-only */
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = sizeof(s_bounce),
    };
    esp_err_t err = spi_bus_initialize(LCD_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) return err;

    esp_lcd_panel_io_spi_config_t io_cfg = {
        .dc_gpio_num = LCD_PIN_DC,
        .cs_gpio_num = LCD_PIN_CS,
        .pclk_hz = 40 * 1000 * 1000,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 4,
    };
    err = esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_SPI_HOST,
                                   &io_cfg, &g_lcd_io);
    if (err != ESP_OK) return err;

    /* bounce-buffer completion sync: two buffers, both start free */
    s_buf_sem = xSemaphoreCreateCounting(2, 2);
    s_buf_idx = 0;
    if (!s_buf_sem) return ESP_ERR_NO_MEM;
    const esp_lcd_panel_io_callbacks_t io_cbs = {
        .on_color_trans_done = on_color_trans_done,
    };
    err = esp_lcd_panel_io_register_event_callbacks(g_lcd_io, &io_cbs, NULL);
    if (err != ESP_OK) return err;

    /* hardware reset */
    gpio_set_level(LCD_PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(LCD_PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(120));

    for (size_t i = 0; i < sizeof(s_init) / sizeof(lcd_cmd_t); i++) {
        err = tx_cmd(s_init[i].cmd, s_init[i].len ? s_init[i].data : NULL,
                     s_init[i].len);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "init cmd 0x%02x failed: %s", s_init[i].cmd,
                     esp_err_to_name(err));
            return err;
        }
        if (s_init[i].delay_ms) vTaskDelay(pdMS_TO_TICKS(s_init[i].delay_ms));
    }

    ESP_LOGI(TAG, "panel init done (portrait native, %s software rotation)",
             LCD_ROT_CW ? "CW" : "CCW");
    lcd_backlight(true);
    return ESP_OK;
}

esp_err_t lcd_blit(int x, int y, int w, int h, const uint16_t *pixels)
{
    if (x < 0 || y < 0 || x + w > LCD_H_RES || y + h > LCD_V_RES)
        return ESP_ERR_INVALID_ARG;
    for (int r = 0; r < h; r++)
        memcpy(&s_shadow[(y + r) * LCD_H_RES + x], &pixels[r * w],
               (size_t)w * sizeof(uint16_t));
    dirty_mark(x, y, w, h);
    return ESP_OK;
}

esp_err_t lcd_flush(void)
{
    if (!s_dirty) return ESP_OK;
    int x0 = s_dx0, y0 = s_dy0, x1 = s_dx1, y1 = s_dy1;
    s_dirty = false;

    int H = y1 - y0 + 1;
    for (int sx0 = x0; sx0 <= x1; sx0 += FLUSH_SLICE_W) {
        int sx1 = sx0 + FLUSH_SLICE_W - 1;
        if (sx1 > x1) sx1 = x1;
        int W = sx1 - sx0 + 1;

        /* wait for a free bounce buffer: its previous DMA transfer, if
         * any, has fully completed (color-done callback gave the token) */
        if (xSemaphoreTake(s_buf_sem, portMAX_DELAY) != pdTRUE)
            return ESP_FAIL;
        uint16_t *bounce = s_bounce[s_buf_idx];
        s_buf_idx ^= 1;

        /* rotate slice into bounce, byte-swapping to panel endian.
         * Packing order must match the native window below AND the panel's
         * default (no-MADCTL) scan: address increments along native X
         * within a row, rows advance along native Y -> row-major with
         * bounce[r * H + c], r = native-row index, c = native-col index.
         *
         * CW  (LCD_ROT_CW=1): native_x = logical_y + 35,
         *                     native_y = 319 - logical_x
         * CCW (LCD_ROT_CW=0): native_x = 169 - logical_y + 35,
         *                     native_y = logical_x              */
#if LCD_ROT_CW
        for (int r = 0; r < W; r++) {          /* r -> native row 319-sx1+r */
            int lx = sx1 - r;                  /* native_y = 319 - lx */
            for (int c = 0; c < H; c++) {      /* c -> native col y0+35+c */
                uint16_t px = s_shadow[(y0 + c) * LCD_H_RES + lx];
                bounce[r * H + c] = __builtin_bswap16(px);
            }
        }
#else
        for (int r = 0; r < W; r++) {          /* r -> native row sx0+r */
            int lx = sx0 + r;                  /* native_y = lx */
            for (int c = 0; c < H; c++) {      /* c -> native col 169-y1+35+c */
                int ly = y1 - c;
                uint16_t px = s_shadow[ly * LCD_H_RES + lx];
                bounce[r * H + c] = __builtin_bswap16(px);
            }
        }
#endif

        /* native window for this slice */
        int nx0, nx1, ny0, ny1;
#if LCD_ROT_CW
        nx0 = y0 + LCD_NATIVE_XOFF;            /* native col = logical y */
        nx1 = y1 + LCD_NATIVE_XOFF;
        ny0 = LCD_NATIVE_H - 1 - sx1;          /* native row = 319 - log x */
        ny1 = LCD_NATIVE_H - 1 - sx0;
#else
        nx0 = LCD_NATIVE_W - 1 - y1 + LCD_NATIVE_XOFF;
        nx1 = LCD_NATIVE_W - 1 - y0 + LCD_NATIVE_XOFF;
        ny0 = sx0;
        ny1 = sx1;
#endif
        uint8_t col[4] = { (uint8_t)(nx0 >> 8), (uint8_t)nx0,
                           (uint8_t)(nx1 >> 8), (uint8_t)nx1 };
        uint8_t row[4] = { (uint8_t)(ny0 >> 8), (uint8_t)ny0,
                           (uint8_t)(ny1 >> 8), (uint8_t)ny1 };
        esp_err_t err = tx_cmd(0x2A, col, 4);
        if (err == ESP_OK) err = tx_cmd(0x2B, row, 4);
        if (err == ESP_OK)
            err = esp_lcd_panel_io_tx_color(g_lcd_io, 0x2C, bounce,
                                            (size_t)W * H * 2);
        if (err != ESP_OK) {
            xSemaphoreGive(s_buf_sem);         /* return the token we took */
            return err;
        }
    }

    /* drain: wait until every outstanding color transfer has completed so
     * lcd_flush() returns with all pixels committed (also guarantees the
     * shadow framebuffer can be safely rewritten by the next draw) */
    for (int i = 0; i < 2; i++)
        xSemaphoreTake(s_buf_sem, portMAX_DELAY);
    xSemaphoreGive(s_buf_sem);
    xSemaphoreGive(s_buf_sem);
    return ESP_OK;
}

esp_err_t lcd_fill(uint16_t rgb565)
{
    size_t n = LCD_H_RES * LCD_V_RES;
    for (size_t i = 0; i < n; i++) s_shadow[i] = rgb565;
    dirty_mark(0, 0, LCD_H_RES, LCD_V_RES);
    return lcd_flush();
}

void lcd_backlight(bool on) { gpio_set_level(LCD_PIN_BL, on ? 0 : 1); }

esp_err_t lcd_char(int x, int y, char c, uint16_t fg, uint16_t bg)
{
    uint16_t buf[8 * 8];
    const uint8_t *g = font8x8_basic[(uint8_t)c & 0x7F];
    for (int row = 0; row < 8; row++) {
        for (int col = 0; col < 8; col++) {
            bool on = (g[row] >> col) & 1;      /* table is LSB-first */
            buf[row * 8 + col] = on ? fg : bg;
        }
    }
    return lcd_blit(x, y, 8, 8, buf);
}

esp_err_t lcd_text(int x, int y, const char *s, uint16_t fg, uint16_t bg)
{
    esp_err_t err = ESP_OK;
    while (*s && err == ESP_OK) {
        err = lcd_char(x, y, *s++, fg, bg);
        x += 8;
    }
    return err;
}

void lcd_release(void) { /* kept for probe experiments; not used in app */ }
