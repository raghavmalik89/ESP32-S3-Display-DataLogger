/* gui.c — see gui.h.  Rendering writes into the LCD shadow framebuffer and
 * flushes once per frame; the acq task never waits on this.
 *
 * Layout constants assume 320x170 logical (landscape, software-rotated).
 */
#include "gui.h"

#include <stdio.h>
#include <string.h>

#include "acq.h"
#include "app_state.h"
#include "btn.h"
#include "config.h"
#include "font8x8_basic.h"
#include "lcd.h"
#include "rec.h"
#include "tx.h"

#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* ---------------- primitives ---------------- */

static uint16_t s_row[LCD_H_RES];

static void hline(int x, int y, int w, uint16_t c)
{
    for (int i = 0; i < w; i++) s_row[i] = c;
    lcd_blit(x, y, w, 1, s_row);
}

static void vline(int x, int y, int h, uint16_t c)
{
    uint16_t col[LCD_V_RES];
    for (int i = 0; i < h; i++) col[i] = c;
    lcd_blit(x, y, 1, h, col);
}

static void rect(int x, int y, int w, int h, uint16_t c)
{
    for (int r = 0; r < h; r++) hline(x, y + r, w, c);
}

/* scaled glyph: scale 1 or 2 (2 -> 16x16) */
static void glyph(int x, int y, char ch, int scale, uint16_t fg, uint16_t bg)
{
    const uint8_t *g = font8x8_basic[(uint8_t)ch & 0x7F];
    static uint16_t buf[16 * 16];
    for (int px = 0; px < 8 * scale; px++) {
        for (int py = 0; py < 8 * scale; py++) {
            int on = (g[py / scale] >> (px / scale)) & 1;  /* LSB-first */
            buf[py * 8 * scale + px] = on ? fg : bg;
        }
    }
    for (int r = 0; r < 8 * scale; r++)
        lcd_blit(x, y + r, 8 * scale, 1, buf + r * 8 * scale);
}

static void text(int x, int y, const char *s, uint16_t fg, uint16_t bg)
{
    while (*s) {
        glyph(x, y, *s++, 1, fg, bg);
        x += 8;
    }
}

static void text2(int x, int y, const char *s, uint16_t fg, uint16_t bg)
{
    while (*s) {
        glyph(x, y, *s++, 2, fg, bg);
        x += 16;
    }
}

/* right-aligned text: x is the RIGHT edge */
static void text_r(int xr, int y, const char *s, uint16_t fg, uint16_t bg)
{
    text(xr - 8 * (int)strlen(s), y, s, fg, bg);
}

static void fmt_v(char *out, float v)
{
    /* +/-X.XX m/s in 6 chars */
    snprintf(out, 8, "%+.2f", v);
}

/* ---------------- persistent chrome ---------------- */

/* recording indicator: red dot + truthful status, on EVERY screen */
static void draw_rec_chrome(void)
{
    if (g_app.rec_active) {
        rect(4, 4, 8, 8, LCD_RED);
        text(16, 4, rec_status_text(), LCD_RED, LCD_BLACK);
        char dur[12];
        snprintf(dur, sizeof(dur), "%lus", (unsigned long)rec_duration_s());
        text_r(LCD_H_RES - 4, 4, dur, LCD_GRAY, LCD_BLACK);
    } else {
        rect(4, 4, 8, 8, LCD_BLACK);
        text(16, 4, rec_status_text(), LCD_GRAY, LCD_BLACK);
    }
    /* filter id always visible */
    char f[8];
    snprintf(f, sizeof(f), "[%s]", filter_id(g_app.sel_filter));
    text_r(LCD_H_RES - 60, LCD_V_RES - 12, f, LCD_CYAN, LCD_BLACK);
}

/* hold progress bar across the bottom edge (all screens while holding) */
static void draw_hold_progress(float frac)
{
    if (frac < 0.f) return;
    int w = (int)(frac * (LCD_H_RES - 8));
    hline(4, LCD_V_RES - 6, LCD_H_RES - 8, 0x2104);        /* track */
    if (w > 0)
        hline(4, LCD_V_RES - 6, w, LCD_YELLOW);
}

/* ---------------- screen 0: status ---------------- */

static void draw_status(void)
{
    char ln[40];
    int y = 18;

    text(4, y, "SENSOR STATUS", LCD_WHITE, LCD_BLACK);
    y += 12;
    snprintf(ln, sizeof(ln), "BMP581 %s  QMI %s  ST %s",
             g_app.bmp_ok ? "OK" : "--",
             g_app.qmi_ok ? "OK" : "--",
             g_app.st_ok ? "OK" : "--");
    text(4, y, ln, LCD_GREEN, LCD_BLACK);
    y += 12;
    snprintf(ln, sizeof(ln), "BARO %4.1f Hz  IMU %4.0f Hz",
             g_app.p_hz, g_app.i_hz);
    text(4, y, ln, LCD_GREEN, LCD_BLACK);
    y += 12;
    if (g_app.bat_ok)
        snprintf(ln, sizeof(ln), "BAT %.2f V", g_app.bat_v);
    else
        snprintf(ln, sizeof(ln), "BAT n/a");
    text(4, y, ln, LCD_GREEN, LCD_BLACK);
    y += 12;
    snprintf(ln, sizeof(ln), "ALT %+.1f m  V %+.2f",
             g_app.alt_m, g_app.v[g_app.sel_filter]);
    text(4, y, ln, LCD_WHITE, LCD_BLACK);
    y += 12;
    snprintf(ln, sizeof(ln), "DROP B%lu I%lu  ERR n%lu t%lu o%lu",
             (unsigned long)g_app.baro_dropped, (unsigned long)g_app.imu_dropped,
             (unsigned long)g_app.err_nak, (unsigned long)g_app.err_timeout,
             (unsigned long)g_app.err_overflow);
    text(4, y, ln, g_app.err_overflow ? LCD_RED : LCD_GRAY, LCD_BLACK);
    y += 12;
    snprintf(ln, sizeof(ln), "SD %s  HOLD=REC",
             g_app.sd_present ? "READY" : "NO CARD");
    text(4, y, ln, g_app.sd_present ? LCD_GREEN : LCD_YELLOW, LCD_BLACK);
}

/* ---------------- screen 1: live graph ---------------- */

static float sig_value(int sig, uint32_t i)
{
    switch (sig) {
    case GS_ALT:   return g_hist_alt[i % HIST_N];
    case GS_PRESS: return g_hist_press[i % HIST_N];
    case GS_ACCZ:  return g_hist_accz[i % HIST_N];
    default:       return g_hist_vario[i % HIST_N];
    }
}

static void draw_graph(void)
{
    const int gx = 4, gy = 20, gw = LCD_H_RES - 8, gh = LCD_V_RES - 40;
    int sig = g_app.graph_signal;

    text(4, 16 - 8 + 4, g_graph_names[sig], LCD_WHITE, LCD_BLACK);

    rect(gx, gy, gw, gh, 0x2104);                     /* plot background */
    float lo, hi;
    int zero_y = -1;
    if (sig == GS_VARIO) { lo = -2.5f; hi = 2.5f; }
    else if (sig == GS_ACCZ) { lo = -2.f; hi = 2.f; }
    else {
        /* autorange altitude/pressure around the visible window */
        lo = 1e30f; hi = -1e30f;
        uint32_t head = g_hist_head;
        for (int x = 0; x < gw; x++) {
            float v = sig_value(sig, head - (gw - x));
            if (v < lo) lo = v;
            if (v > hi) hi = v;
        }
        float pad = (hi - lo) * 0.1f + 0.001f;
        lo -= pad; hi += pad;
    }
    if (lo < 0.f && hi > 0.f)
        zero_y = gy + gh - (int)((0.f - lo) / (hi - lo) * gh);

    uint32_t head = g_hist_head;
    for (int x = 0; x < gw; x++) {
        float v = sig_value(sig, head - (gw - x));
        if (v < lo) v = lo;
        if (v > hi) v = hi;
        int py = gy + gh - (int)((v - lo) / (hi - lo) * gh);
        if (py < gy) py = gy;
        if (py > gy + gh - 1) py = gy + gh - 1;
        vline(gx + x, py, 2, LCD_GREEN);
    }
    if (zero_y >= 0)
        hline(gx, zero_y, gw, LCD_RED);               /* fixed zero ref */
}

/* ---------------- screen 2: vario ---------------- */

static void draw_vario(void)
{
    char ln[24];
    int sel = g_app.sel_filter;
    float v = g_app.v[sel];

    /* big signed vario, 2x glyphs, centred-ish */
    fmt_v(ln, v);
    text2(60, 30, ln, v >= 0.f ? LCD_GREEN : LCD_CYAN, LCD_BLACK);
    text(200, 40, "m/s", LCD_GRAY, LCD_BLACK);

    /* climb/sink ladder: fixed +-2.5 m/s, zero at centre */
    int cx = 300, cy0 = 24, ch = 120;
    vline(cx, cy0, ch, 0x4208);
    int zy = cy0 + ch / 2;
    hline(cx - 4, zy, 8, LCD_RED);
    int vy = zy - (int)(v / 2.5f * (ch / 2));
    if (vy < cy0) vy = cy0;
    if (vy > cy0 + ch - 4) vy = cy0 + ch - 4;
    rect(cx - 3, vy, 6, 4, v >= 0.f ? LCD_GREEN : LCD_CYAN);

    snprintf(ln, sizeof(ln), "AVG10 %+.2f", app_vario_avg10s());
    text(4, 80, ln, LCD_WHITE, LCD_BLACK);
    snprintf(ln, sizeof(ln), "ALT %+.1f m", g_app.alt_m);
    text(4, 96, ln, LCD_WHITE, LCD_BLACK);
    snprintf(ln, sizeof(ln), "FIL %s", filter_id(sel));
    text(4, 112, ln, LCD_CYAN, LCD_BLACK);
    text(4, 128, "HOLD=FILT", LCD_GRAY, LCD_BLACK);
}

/* ---------------- event handling ---------------- */

static void handle_events(void)
{
    int ev;
    while ((ev = btn_get_event()) != BTN_NONE) {
        if (ev == BTN_CLICK) {
            g_app.screen = (g_app.screen + 1) % SCREEN_N;
        } else if (ev == BTN_HOLD) {
            switch (g_app.screen) {
            case SCREEN_STATUS: rec_toggle(); break;
            case SCREEN_GRAPH:
                g_app.graph_signal = (g_app.graph_signal + 1) % GS_N;
                break;
            case SCREEN_VARIO: {
                int f = (g_app.sel_filter + 1) % FILT_N;
                acq_select_filter(f);       /* no reset: spike-free */
                tx_event_filter(f);
                break;
            }
            }
        }
    }
}

static void gui_task(void *arg)
{
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(100));   /* 10 fps snapshots */
        handle_events();
        g_app.btn_held = btn_is_pressed();
        float prog = btn_hold_progress();

        lcd_clear(LCD_BLACK);
        switch (g_app.screen) {
        case SCREEN_GRAPH:  draw_graph();  break;
        case SCREEN_VARIO:  draw_vario();  break;
        default:            draw_status(); break;
        }
        draw_rec_chrome();
        draw_hold_progress(prog);
        lcd_flush();
    }
}

const char *g_graph_names[GS_N] = { "VARIO m/s", "ALT m", "P hPa", "ACCZ g" };
const char *g_screen_names[SCREEN_N] = { "STATUS", "GRAPH", "VARIO" };

esp_err_t gui_start(void)
{
    return xTaskCreate(gui_task, "gui", 4096, NULL, tskIDLE_PRIORITY + 4,
                       NULL) == pdPASS ? ESP_OK : ESP_FAIL;
}
