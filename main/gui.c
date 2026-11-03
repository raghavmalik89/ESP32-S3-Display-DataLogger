/* gui.c — GUI Rev 2 (see docs/mockups): dark instrument theme, three
 * screens, persistent storage dot, honest recording state.
 *
 * Rendering writes into the LCD shadow framebuffer and flushes once per
 * frame; the acq task never waits on this.  Layout assumes 320x170
 * logical (landscape, software-rotated).
 */
#include "gui.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "acq.h"
#include "app_state.h"
#include "btn.h"
#include "config.h"
#include "font8x8_basic.h"
#include "lcd.h"
#include "rec.h"
#include "sd_sink.h"
#include "tx.h"

#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* ---------------- palette (mockup-derived) ---------------- */

#define C_BG    LCD_RGB565(13, 22, 38)     /* page background        */
#define C_CARD  LCD_RGB565(24, 38, 58)     /* panel card             */
#define C_EDGE  LCD_RGB565(38, 56, 80)     /* outlines, grid, track  */
#define C_TEAL  LCD_RGB565(110, 220, 185)  /* accent                 */
#define C_TXT   LCD_WHITE
#define C_DIM   LCD_RGB565(130, 145, 165) /* secondary text         */
#define C_GOOD  LCD_RGB565(60, 230, 125)   /* green (brighter stock) */
#define C_WARN  LCD_RGB565(240, 200, 60)   /* amber                  */
#define C_BAD   LCD_RGB565(235, 80, 70)    /* red                    */
#define C_ZERO  LCD_RGB565(200, 210, 225)  /* zero reference line    */

const int g_graph_windows[GRAPH_WINDOW_N] = { 10, 30, 60, 120, 300 };
const char *g_screen_names[SCREEN_N] = { "STATUS", "HISTORY", "VARIO" };

/* ---------------- clipped primitives ---------------- */

static uint16_t s_row[LCD_H_RES];
static uint16_t s_glyph[24 * 24];
static float s_col_lo[LCD_H_RES], s_col_hi[LCD_H_RES];

static void hline(int x, int y, int w, uint16_t c)
{
    if (y < 0 || y >= LCD_V_RES || w <= 0) return;
    if (x < 0) { w += x; x = 0; }
    if (x + w > LCD_H_RES) w = LCD_H_RES - x;
    if (w <= 0) return;
    for (int i = 0; i < w; i++) s_row[i] = c;
    lcd_blit(x, y, w, 1, s_row);
}

static void vline(int x, int y, int h, uint16_t c)
{
    uint16_t col[LCD_V_RES];
    if (x < 0 || x >= LCD_H_RES || h <= 0) return;
    if (y < 0) { h += y; y = 0; }
    if (y + h > LCD_V_RES) h = LCD_V_RES - y;
    if (h <= 0) return;
    for (int i = 0; i < h; i++) col[i] = c;
    lcd_blit(x, y, 1, h, col);
}

static void rect(int x, int y, int w, int h, uint16_t c)
{
    for (int r = 0; r < h; r++) hline(x, y + r, w, c);
}

static void circle(int cx, int cy, int r, uint16_t c)
{
    for (int dy = -r; dy <= r; dy++) {
        int hw = (int)sqrtf((float)(r * r - dy * dy));
        hline(cx - hw, cy + dy, 2 * hw + 1, c);
    }
}

/* scaled glyph, scale 1..3 (8/16/24 px).  Out-of-screen glyphs skipped. */
static void glyph(int x, int y, char ch, int scale, uint16_t fg, uint16_t bg)
{
    if (x + 8 * scale > LCD_H_RES || x < 0 ||
        y + 8 * scale > LCD_V_RES || y < 0)
        return;
    const uint8_t *g = font8x8_basic[(uint8_t)ch & 0x7F];
    int px8 = 8 * scale;
    for (int px = 0; px < px8; px++)
        for (int py = 0; py < px8; py++)
            s_glyph[py * px8 + px] =
                ((g[py / scale] >> (px / scale)) & 1) ? fg : bg;  /* LSB-first */
    for (int r = 0; r < px8; r++)
        lcd_blit(x, y + r, px8, 1, s_glyph + r * px8);
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

static void text3(int x, int y, const char *s, uint16_t fg, uint16_t bg)
{
    while (*s) {
        glyph(x, y, *s++, 3, fg, bg);
        x += 24;
    }
}

/* right-aligned: x is the RIGHT edge */
static void text_r(int xr, int y, const char *s, uint16_t fg, uint16_t bg)
{
    text(xr - 8 * (int)strlen(s), y, s, fg, bg);
}
static void text2_r(int xr, int y, const char *s, uint16_t fg, uint16_t bg)
{
    text2(xr - 16 * (int)strlen(s), y, s, fg, bg);
}

/* ---------------- persistent chrome ---------------- */

/* storage dot: SAME position on every screen (top right) */
static void draw_dot(void)
{
    uint16_t c = rec_dot_color() == REC_DOT_RED ? C_BAD :
                 rec_dot_color() == REC_DOT_GREEN ? C_GOOD : C_WARN;
    circle(308, 10, 6, c);
}

/* header: big title + hairline.  Subtitle optional (one row under title). */
static void header(const char *title, const char *sub)
{
    text2(6, 2, title, C_TXT, C_BG);
    if (sub) text(6, 20, sub, C_DIM, C_BG);
    hline(0, 30, LCD_H_RES, C_EDGE);
}

/* hold progress bar across the bottom edge (all screens while holding) */
static void draw_hold_progress(float frac)
{
    if (frac < 0.f) return;
    int w = (int)(frac * (LCD_H_RES - 8));
    hline(4, LCD_V_RES - 5, LCD_H_RES - 8, C_EDGE);        /* track */
    if (w > 0)
        hline(4, LCD_V_RES - 5, w, C_WARN);
}

static void hint(const char *s)
{
    text(4, LCD_V_RES - 14, s, C_DIM, C_BG);
}

/* ---------------- screen 0: status + storage ---------------- */

static void fmt_gb(char *out, int n, uint64_t bytes)
{
    /* one-decimal GB, e.g. "28.4" */
    uint64_t tenths = (bytes * 10) >> 30;
    snprintf(out, n, "%llu.%llu", (unsigned long long)(tenths / 10),
             (unsigned long long)(tenths % 10));
}

static void draw_status(void)
{
    char ln[44];
    header("STATUS", "SENSOR & STORAGE");

    /* left card: sensors */
    rect(4, 36, 148, 60, C_CARD);
    text(12, 41, "SENSORS", C_TEAL, C_CARD);
    text(12, 53, "BMP581", C_TXT, C_CARD);
    if (g_app.bmp_ok) {
        snprintf(ln, sizeof(ln), "%3.0f Hz", g_app.p_hz);
        text_r(144, 53, ln, C_GOOD, C_CARD);
    } else
        text_r(144, 53, "OFF", C_WARN, C_CARD);
    text(12, 65, "QMI8658", C_TXT, C_CARD);
    if (g_app.qmi_ok) {
        snprintf(ln, sizeof(ln), "%3.0f Hz", g_app.i_hz);
        text_r(144, 65, ln, C_GOOD, C_CARD);
    } else
        text_r(144, 65, "FAULT", C_WARN, C_CARD);   /* soldered, dead (see SENSORS.md) */
    text(12, 77, "ST IMU", C_TXT, C_CARD);
    text_r(144, 77, g_app.st_ok ? "OK" : "NOT FITTED",
           g_app.st_ok ? C_GOOD : C_WARN, C_CARD);

    /* right card: microSD capacity + fill */
    rect(160, 36, 156, 60, C_CARD);
    text(168, 41, "microSD", C_TEAL, C_CARD);
    if (g_app.sd_present) {
        char gb[12];
        fmt_gb(gb, sizeof(gb), sd_free_now());
        text2(168, 51, gb, C_TXT, C_CARD);
        text(232, 57, "GB FREE", C_DIM, C_CARD);
        /* fill bar: fraction used */
        int bw = 132;
        uint64_t used = g_app.sd_total_bytes - sd_free_now();
        int fw = g_app.sd_total_bytes
                     ? (int)(used * bw / g_app.sd_total_bytes) : 0;
        if (fw > bw) fw = bw;
        rect(168, 72, bw, 6, C_EDGE);
        if (fw > 0) rect(168, 72, fw, 6, C_TEAL);
        uint64_t pct = g_app.sd_total_bytes
                           ? used * 100 / g_app.sd_total_bytes : 0;
        snprintf(ln, sizeof(ln), "%llu%% USED",
                 (unsigned long long)pct);
        text(168, 82, ln, C_DIM, C_CARD);
        fmt_gb(gb, sizeof(gb), g_app.sd_total_bytes);
        snprintf(ln, sizeof(ln), "%s GB CARD", gb);
        text_r(300, 82, ln, C_DIM, C_CARD);
    } else {
        text2(168, 51, "NO CARD", C_WARN, C_CARD);
        rect(168, 72, 132, 6, C_EDGE);
        text(168, 82, "USB-ONLY MODE", C_DIM, C_CARD);
    }

    /* bottom card: recording forecast + health */
    rect(4, 102, 312, 48, C_CARD);
    text(12, 107, "RECORDING TIME LEFT", C_DIM, C_CARD);
    double bps = g_app.stream_bps;
    if (g_app.sd_present && bps > 10.0) {
        uint64_t secs = (uint64_t)(sd_free_now() / bps);
        snprintf(ln, sizeof(ln), "%uh %02um",
                 (unsigned)(secs / 3600), (unsigned)((secs % 3600) / 60));
    } else
        snprintf(ln, sizeof(ln), "--");
    text2(12, 118, ln, C_TXT, C_CARD);

    uint32_t drops = g_app.baro_dropped + g_app.imu_dropped;
    text(208, 107, "DROPS", C_DIM, C_CARD);
    snprintf(ln, sizeof(ln), "%lu", (unsigned long)drops);
    text_r(308, 107, ln, drops ? C_WARN : C_GOOD, C_CARD);
    text(208, 119, "SD ERRORS", C_DIM, C_CARD);
    snprintf(ln, sizeof(ln), "%lu", (unsigned long)g_app.sd_err);
    text_r(308, 119, ln, g_app.sd_err ? C_BAD : C_GOOD, C_CARD);
    text(208, 131, "CAPTURE", C_DIM, C_CARD);
    uint16_t cc = rec_dot_color() == REC_DOT_RED ? C_BAD :
                  rec_dot_color() == REC_DOT_GREEN ? C_GOOD : C_WARN;
    text_r(308, 131, rec_status_text(), cc, C_CARD);

    hint("HOLD 10s: START/STOP REC");
}

/* ---------------- screen 1: history ---------------- */

#define GR_LO (-2.5f)
#define GR_HI (2.5f)

static int gr_map(float v, float lo, float hi, int gy, int gh)
{
    if (v < lo) v = lo;
    if (v > hi) v = hi;
    return gy + gh - (int)((v - lo) / (hi - lo) * gh);
}
static int gr_y(float v, int gy, int gh) { return gr_map(v, GR_LO, GR_HI, gy, gh); }
static int gr_y5(float v, int gy, int gh) { return gr_map(v, -5.f, 5.f, gy, gh); }

static void draw_history(void)
{
    char ln[44];
    const char *fid = filter_id(g_app.sel_filter);
    snprintf(ln, sizeof(ln), "%s | VERTICAL SPEED (m/s)", fid);
    header("HISTORY", ln);

    int win = g_graph_windows[g_app.graph_window_idx];
    /* window chip, top right */
    rect(220, 4, 96, 18, C_CARD);
    snprintf(ln, sizeof(ln), "%ds WINDOW", win);
    text_r(308, 9, ln, C_TEAL, C_CARD);

    const int gx = 32, gy = 40, gw = LCD_H_RES - gx - 6, gh = 96;

    /* grid + labels: fixed +-2.5 m/s scale, zero line distinct */
    for (int k = -2; k <= 2; k++) {
        float v = (float)k;
        int y = gr_y(v, gy, gh);
        uint16_t c = k == 0 ? C_ZERO : C_EDGE;
        hline(gx, y, gw, c);
        snprintf(ln, sizeof(ln), "%+d", k);
        text_r(gx - 4, y - 4, ln, k == 0 ? C_ZERO : C_DIM, C_BG);
    }

    /* aggregate min/max per column; left = old, right = now */
    uint32_t avail = acq_hist_available((uint32_t)win);
    for (int c = 0; c < gw; c++) { s_col_lo[c] = 1e30f; s_col_hi[c] = -1e30f; }
    if (avail > 1) {
        uint32_t t0 = g_hist_head - avail;      /* oldest usable sample */
        for (uint32_t i = t0; i < g_hist_head; i++) {
            float v = g_hist_vario[i % HIST_N];
            int c = (int)((i - t0) * (uint32_t)gw / avail);
            if (c >= gw) c = gw - 1;
            if (v < s_col_lo[c]) s_col_lo[c] = v;
            if (v > s_col_hi[c]) s_col_hi[c] = v;
        }
        for (int c = 0; c < gw; c++) {
            if (s_col_lo[c] > s_col_hi[c]) continue;
            int ya = gr_y(s_col_hi[c], gy, gh);
            int yb = gr_y(s_col_lo[c], gy, gh);
            vline(gx + c, ya, yb - ya + 1, C_TEAL);
        }
    }

    /* time labels: -win / -win/2 / NOW */
    snprintf(ln, sizeof(ln), "-%ds", win);
    text(gx, gy + gh + 4, ln, C_DIM, C_BG);
    snprintf(ln, sizeof(ln), "-%ds", win / 2);
    text(gx + gw / 2 - 8, gy + gh + 4, ln, C_DIM, C_BG);
    text_r(gx + gw, gy + gh + 4, "NOW", C_DIM, C_BG);

    hint("HOLD 10s: CHANGE WINDOW");
}

/* ---------------- screen 2: vario ---------------- */

static void draw_vario(void)
{
    char ln[24];
    int sel = g_app.sel_filter;
    float v = g_app.v[sel];
    int climbing = v >= 0.f;

    header("VARIO", "BAROMETRIC CLIMB / SINK");
    text(150, 6, filter_id(sel), C_TEAL, C_BG);   /* active filter, top centre */

    text(10, 38, climbing ? "CLIMB" : "SINK",
         climbing ? C_GOOD : C_BAD, C_BG);

    /* dominant number, 3x glyphs */
    snprintf(ln, sizeof(ln), "%+.2f", v);
    text3(8, 52, ln, C_TXT, C_BG);
    text(140, 60, "m/s", C_DIM, C_BG);

    /* right scale card: fixed +-5 m/s band with active range */
    const int sx = 246, sy = 36, sw = 70, sh = 112;
    rect(sx, sy, sw, sh, C_CARD);
    for (int k = -5; k <= 5; k += 2) {
        int y = gr_y5((float)k, sy, sh);
        snprintf(ln, sizeof(ln), "%+d", k);
        text(sx + 4, y - 4, ln, k == 0 ? C_ZERO : C_DIM, C_CARD);
        hline(sx + 26, y, sw - 30, k == 0 ? C_ZERO : C_EDGE);
    }
    {
        int zy = gr_y5(0.f, sy, sh);
        int vy = gr_y5(v, sy, sh);
        int y0 = climbing ? vy : zy;
        int hgt = abs(vy - zy);
        if (hgt > 0) rect(sx + 30, y0, sw - 38, hgt,
                          climbing ? C_GOOD : C_BAD);
    }

    /* bottom card: 10 s average + baro altitude */
    rect(4, 122, 234, 26, C_CARD);
    text(12, 125, "AVG 10s", C_DIM, C_CARD);
    snprintf(ln, sizeof(ln), "%+.2f", app_vario_avg10s());
    text2(12, 133, ln, C_TXT, C_CARD);
    vline(126, 126, 18, C_EDGE);
    text(136, 125, "ALT (ISA)", C_DIM, C_CARD);
    snprintf(ln, sizeof(ln), "%.0f m", g_app.alt_m);
    text2(136, 133, ln, C_TXT, C_CARD);

    hint("HOLD 10s: CHANGE FILTER");
}

/* ---------------- events + task ---------------- */

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
                /* cycle 10 > 30 > 60 > 120 > 300 s; history is preserved
                 * (the ring keeps 300 s regardless of window) */
                g_app.graph_window_idx =
                    (g_app.graph_window_idx + 1) % GRAPH_WINDOW_N;
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

        lcd_clear(C_BG);
        switch (g_app.screen) {
        case SCREEN_GRAPH:  draw_history(); break;
        case SCREEN_VARIO:  draw_vario();   break;
        default:            draw_status();  break;
        }
        draw_dot();
        draw_hold_progress(prog);
        lcd_flush();
    }
}

esp_err_t gui_start(void)
{
    return xTaskCreate(gui_task, "gui", 4096, NULL, tskIDLE_PRIORITY + 4,
                       NULL) == pdPASS ? ESP_OK : ESP_FAIL;
}
