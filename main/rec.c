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
        g_app.rec_backend = g_app.sd_present ? 2 : 1;
        g_app.rec_active = 1;
        if (g_app.rec_backend == 2)
            sd_open_session(g_app.rec_session);
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
        return "IDLE";
    if (g_app.rec_backend == 2)
        snprintf(buf, sizeof(buf), "SD SAVED");
    else if (g_app.host_grab)
        snprintf(buf, sizeof(buf), "USB+HOST");
    else
        snprintf(buf, sizeof(buf), "USB STREAM");
    return buf;
}

uint32_t rec_duration_s(void)
{
    if (!g_app.rec_active) return 0;
    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
    return (now - g_app.rec_start_ms) / 1000;
}
