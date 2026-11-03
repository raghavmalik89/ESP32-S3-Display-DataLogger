/* tx.c — see tx.h.
 *
 * The TX path never blocks acquisition: samples land in queues, this task
 * formats and emits them.  When no host is connected (or the TX fifo is
 * full) lines are dropped whole and counted — truncated lines must never
 * reach the host because they would corrupt its CSV.
 */
#include "tx.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "app_state.h"
#include "config.h"
#include "filters.h"
#include "sd_sink.h"

#include "driver/usb_serial_jtag.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

static const char *TAG = "tx";

static QueueHandle_t s_q_baro, s_q_imu;

/* per-stream monotonic counters (independent of the acq counters, which
 * count samples; these count EMITTED frames — a write drop creates a gap
 * here that the host can see even though the sample itself was kept) */
static uint32_t s_seq_b, s_seq_m, s_seq_v, s_seq_e, s_seq_meta;

static bool s_last_conn;
static int64_t s_attach_us;

bool tx_push_baro(const acq_baro_evt_t *evt)
{
    if (!s_q_baro) return false;
    if (xQueueSend(s_q_baro, evt, 0) == pdTRUE) return true;
    g_app.err_overflow++;
    return false;
}

bool tx_push_imu(const acq_imu_evt_t *evt)
{
    if (!s_q_imu) return false;
    if (xQueueSend(s_q_imu, evt, 0) == pdTRUE) return true;
    g_app.err_overflow++;
    return false;
}

/* write one complete line; drop it whole if it cannot be sent now.
 * When an SD session is active the line is mirrored to the card. */
static void emit(const char *line, int len)
{
    if (g_app.rec_active) {
        g_app.rec_frames++;
        if (g_app.rec_backend == 2)
            sd_write(line, (size_t)len);
    }
    if (!usb_serial_jtag_is_connected()) {
        g_app.err_overflow++;
        return;
    }
    int w = usb_serial_jtag_write_bytes(line, len, 0);
    if (w != len)
        g_app.err_overflow++;
}

static void emitf(const char *fmt, ...)
{
    char buf[160];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n > 0)
        emit(buf, n < (int)sizeof(buf) ? n : (int)sizeof(buf) - 1);
}

static void emit_meta(const char *reason)
{
    s_seq_meta++;
    emitf("#H,s3-logger,0.1.0\n");
    emitf("#D,bmp581=%s,qmi8658c=%s,st_imu=%s@0x%02x,sd=%s\n",
          g_app.bmp_ok ? "ok@0x47" : "absent",
          g_app.qmi_ok ? "ok@0x6B" : "absent",
          g_app.st_ok ? "ok" : "absent", g_app.st_addr,
          g_app.sd_present ? "present" : "absent");
    emitf("#C,p_hz=%d,i_hz=%d,imu=%s,backend=%s,filters=C03/D01/S4/S5\n",
          BMP581_P_HZ, 1000 / TICK_MS, g_app.qmi_ok ? "on" : "off",
          g_app.sd_present ? "sd|usb" : "usb");
    emitf("#R,%s\n", reason);
    s_seq_b = s_seq_m = 0;
}

void tx_event_session(int start, uint32_t session_id)
{
    emitf("S,%lu,%llu,%d,%lu\n", (unsigned long)++s_seq_e,
          (unsigned long long)(esp_timer_get_time()),
          start ? 1 : 0, (unsigned long)session_id);
}

void tx_event_filter(int filter)
{
    emitf("F,%lu,%llu,%s\n", (unsigned long)++s_seq_e,
          (unsigned long long)(esp_timer_get_time()), filter_id(filter));
}

/* host handshake: capture.py sends "G,1" (grabbing) / "G,0" (stopped) */
static void poll_host_cmd(void)
{
    char buf[16];
    int n = usb_serial_jtag_read_bytes(buf, sizeof(buf) - 1, 0);
    for (int i = 0; i < n; i++) {
        if (buf[i] == '1') g_app.host_grab = 1;
        else if (buf[i] == '0') g_app.host_grab = 0;
    }
}

static void tx_task(void *arg)
{
    char line[160];
    s_last_conn = usb_serial_jtag_is_connected();
    if (s_last_conn)
        emit_meta("poweron");
    ESP_LOGI(TAG, "tx task up; host %s", s_last_conn ? "connected" : "absent");

    TickType_t last_v = 0;
    for (;;) {
        acq_baro_evt_t b;
        acq_imu_evt_t m;

        while (s_q_baro && xQueueReceive(s_q_baro, &b, 0) == pdTRUE) {
            int n = snprintf(line, sizeof(line), "B,%lu,%llu,%lu,%lu\n",
                             (unsigned long)++s_seq_b,
                             (unsigned long long)b.t_us,
                             (unsigned long)b.p_raw, (unsigned long)b.t_raw);
            emit(line, n);
        }
        while (s_q_imu && xQueueReceive(s_q_imu, &m, 0) == pdTRUE) {
            int n = snprintf(line, sizeof(line),
                             "M,%lu,%llu,%d,%d,%d,%d,%d,%d,%d\n",
                             (unsigned long)++s_seq_m,
                             (unsigned long long)m.t_us, m.sid,
                             m.acc[0], m.acc[1], m.acc[2],
                             m.gyr[0], m.gyr[1], m.gyr[2]);
            emit(line, n);
        }

        /* V frame at 20 Hz: selected filter estimate (mm/s, mm) */
        TickType_t now = xTaskGetTickCount();
        if (now - last_v >= pdMS_TO_TICKS(50)) {
            last_v = now;
            int f = g_app.sel_filter;
            int vs = (int)(g_app.v[f] * 1000.f);
            int hs = (int)(g_app.alt_f[f] * 1000.f);
            emitf("V,%lu,%llu,%s,%d,%d\n", (unsigned long)++s_seq_v,
                  (unsigned long long)esp_timer_get_time(), filter_id(f),
                  vs, hs);
        }

        /* host attach: re-emit meta shortly after connect (legacy lesson:
         * opening the CDC port does not reset the board, so a late capture
         * tool must still receive boot meta) */
        int conn = usb_serial_jtag_is_connected();
        if (conn && !s_last_conn)
            s_attach_us = esp_timer_get_time();
        if (conn && s_attach_us != 0 &&
            esp_timer_get_time() - s_attach_us > 2500000) {
            emit_meta("host_attach");
            s_attach_us = 0;
        }
        s_last_conn = conn;

        poll_host_cmd();
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

esp_err_t tx_init(void)
{
    usb_serial_jtag_driver_config_t cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    esp_err_t err = usb_serial_jtag_driver_install(&cfg);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
        return err;                       /* console may already own it */
    s_q_baro = xQueueCreate(64, sizeof(acq_baro_evt_t));
    s_q_imu = xQueueCreate(256, sizeof(acq_imu_evt_t));
    return (s_q_baro && s_q_imu) ? ESP_OK : ESP_ERR_NO_MEM;
}

esp_err_t tx_start(void)
{
    return xTaskCreate(tx_task, "tx", 3072, NULL, tskIDLE_PRIORITY + 5,
                       NULL) == pdPASS ? ESP_OK : ESP_FAIL;
}
