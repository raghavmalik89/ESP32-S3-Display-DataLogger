/* rec.c — recording session state machine.
 *
 * Firmware Reliability Rev 1: the GUI task no longer touches the SD
 * session file.  It only posts a start/stop REQUEST (rec_toggle /
 * rec_request); the TX task consumes it in rec_poll(), which is the
 * single context that opens, writes markers into, and closes the
 * session file — so the GUI can never close a file the TX task is
 * writing.  Startup sequence puts the START marker first in the file;
 * shutdown drains pending samples, writes the STOP marker last, then
 * flushes and closes (with read-back verification in sd_sink).
 *
 * Truthfulness rules (frozen constraint):
 *   - USB streaming alone is VOLATILE: the screen shows "USB STREAM", never
 *     "SAVED".  Only when the host tool confirms capture ("G,1") does the
 *     indicator add "+HOST".
 *   - SD recording claims "REC SD" only because frames are counted at the
 *     moment they are handed to the SD writer, and the red dot additionally
 *     requires the session file to be open with zero accumulated sd_err.
 */
#include "rec.h"

#include <stdio.h>

#include "app_state.h"
#include "sd_sink.h"
#include "tx.h"

#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "rec";

/* 0 = none, 1 = start, 2 = stop; written by GUI/host command context,
 * cleared and executed by the TX task in rec_poll() */
static volatile int s_req;

/* USB-only session numbering when no card is mounted (RAM-only, per boot) */
static uint32_t s_usb_seq;

void rec_toggle(void)
{
    s_req = g_app.rec_active ? 2 : 1;
}

void rec_request(int start)
{
    s_req = start ? 1 : 2;
}

void rec_poll(void)   /* TX task context ONLY */
{
    int req = s_req;
    if (!req) return;
    s_req = 0;

    if (req == 1 && !g_app.rec_active) {
        uint32_t id = 0;
        g_app.rec_backend = 1;
        if (g_app.sd_present &&
            sd_open_session_auto(&id) == ESP_OK) {
            g_app.rec_backend = 2;
        } else {
            id = ++s_usb_seq;   /* no card / open failed: volatile stream */
        }
        g_app.rec_session = id;
        g_app.rec_start_ms = (uint32_t)(esp_timer_get_time() / 1000);
        g_app.rec_frames = 0;
        g_app.rec_active = 1;               /* marker below mirrors to SD */
        tx_event_session(1, g_app.rec_session);
        ESP_LOGI(TAG, "session %lu start (%s)", (unsigned long)id,
                 g_app.rec_backend == 2 ? "SD" : "USB-only");
    } else if (req == 2 && g_app.rec_active) {
        tx_drain();                          /* pending samples -> file */
        tx_event_session(0, g_app.rec_session);  /* STOP marker, last line */
        g_app.rec_active = 0;
        if (g_app.rec_backend == 2)
            sd_close_session();              /* flush + close + verify */
        ESP_LOGI(TAG, "session %lu stop (%lu frames)",
                 (unsigned long)g_app.rec_session,
                 (unsigned long)g_app.rec_frames);
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
