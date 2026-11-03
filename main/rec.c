/* rec.c — see rec.h. */
#include "rec.h"

#include <stdio.h>

#include "app_state.h"
#include "sd_sink.h"
#include "tx.h"

#include "esp_timer.h"

void rec_toggle(void)
{
    if (!g_app.rec_active) {
        g_app.rec_session++;
        g_app.rec_start_ms = (uint32_t)(esp_timer_get_time() / 1000);
        g_app.rec_frames = 0;
        g_app.rec_active = 1;
        /* SD backend only when the session file is actually open and
         * accepting writes — a failed open falls back to the volatile
         * USB stream and stays yellow (never claims persistent save) */
        g_app.rec_backend = 1;
        if (g_app.sd_present &&
            sd_open_session(g_app.rec_session) == ESP_OK)
            g_app.rec_backend = 2;
        tx_event_session(1, g_app.rec_session);
    } else {
        g_app.rec_active = 0;
        tx_event_session(0, g_app.rec_session);
        if (g_app.rec_backend == 2)
            sd_close_session();
    }
}

const char *rec_status_text(void)
{
    static char buf[28];
    if (!g_app.rec_active)
        return "STANDBY";
    if (g_app.rec_backend == 2)
        snprintf(buf, sizeof(buf), "REC SD %lu.%lu KB",
                 (unsigned long)(sd_session_bytes() / 1024),
                 (unsigned long)((sd_session_bytes() % 1024) * 10 / 1024));
    else if (g_app.host_grab)
        snprintf(buf, sizeof(buf), "USB+HOST");
    else
        snprintf(buf, sizeof(buf), "USB STREAM");
    return buf;
}

int rec_dot_color(void)
{
    if (g_app.rec_active) {
        /* RED only while a confirmed persistent write is flowing */
        if (g_app.rec_backend == 2 && sd_session_open() &&
            g_app.sd_err == 0)
            return REC_DOT_RED;
        /* USB-only streaming is volatile: warning, not "recording" */
        return REC_DOT_YELLOW;
    }
    /* GREEN: writable media available, nothing recording */
    if (g_app.sd_present && g_app.sd_err == 0)
        return REC_DOT_GREEN;
    /* YELLOW: no card, or a storage fault on record */
    return REC_DOT_YELLOW;
}

uint32_t rec_duration_s(void)
{
    if (!g_app.rec_active) return 0;
    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
    return (now - g_app.rec_start_ms) / 1000;
}
