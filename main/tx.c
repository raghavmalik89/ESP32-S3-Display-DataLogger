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
#include "rec.h"
#include "sd_sink.h"

#include <stdlib.h>

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
    if (xQueueSend(s_q_imu, evt, 0) == pdTRUE) {
        UBaseType_t depth = uxQueueMessagesWaiting(s_q_imu);
        if (depth > g_app.tx_q_high) g_app.tx_q_high = depth;
        return true;
    }
    g_app.err_overflow++;
    return false;
}

/* forward decl: shared line emitter below */
static void emit(const char *line, int len);

void tx_drain(void)
{
    char line[160];
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
}

/* measured protocol byte rate for the recording-time-left estimate */
static uint32_t s_bps_acc;
static int64_t s_bps_last_us;

/* write one complete line; drop it whole if it cannot be sent now.
 * When an SD session is active the line is mirrored to the card. */
static void emit(const char *line, int len)
{
    if (g_app.rec_active) {
        g_app.rec_frames++;
        if (g_app.rec_backend == 2)
            sd_write(line, (size_t)len);
    }
    s_bps_acc += (uint32_t)len;
    int64_t now = esp_timer_get_time();
    if (now - s_bps_last_us >= 1000000) {
        float inst = s_bps_acc * 1e6f / (float)(now - s_bps_last_us);
        /* 5 s EMA; first sample seeds it */
        g_app.stream_bps = (g_app.stream_bps <= 0.f)
                               ? inst
                               : g_app.stream_bps * 0.8f + inst * 0.2f;
        s_bps_acc = 0;
        s_bps_last_us = now;
    }
    /* No host attached: lines simply have nowhere to go.  When an SD
     * session owns the data this is not an error at all; when idle it is
     * expected behavior, not a buffer overflow — so only a failed write
     * to an ATTACHED host counts as err_overflow. */
    if (!usb_serial_jtag_is_connected())
        return;
    int w = usb_serial_jtag_write_bytes(line, len, 0);
    if (w != len)
        g_app.err_overflow++;
}

static void emitf(const char *fmt, ...)
{
    char buf[288];
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

/* ---- 1 Hz I2C instrumentation frame (#I), Task 2 bench -----------------
 * Per-second snapshot of the acq-side counters: per-sensor call count,
 * failures, worst and p99 transaction duration (incl. in-driver retries
 * and recovery), stale-sample counts, baro schedule jitter, TX queue
 * high-water and the global drop/NAK/timeout counters.  Meta-comment
 * frame: never mirrored into SD recordings... actually mirrored like any
 * other frame while recording, which is exactly what the recording-load
 * bench needs. */
static uint32_t s_hist_snap[3][8];
static uint32_t s_n_snap[3], s_fail_snap[3], s_max_snap[3];

static void i2c_snapshot(void)
{
    for (int d = 0; d < 3; d++) {
        s_n_snap[d] = g_app.i2c_n[d];
        g_app.i2c_n[d] = 0;
        s_fail_snap[d] = g_app.i2c_fail[d];
        g_app.i2c_fail[d] = 0;
        s_max_snap[d] = g_app.i2c_max_us[d];
        g_app.i2c_max_us[d] = 0;
        memcpy(s_hist_snap[d], g_app.i2c_hist[d], sizeof(s_hist_snap[d]));
        memset(g_app.i2c_hist[d], 0, sizeof(g_app.i2c_hist[d]));
    }
}

/* p99 = upper bound (us) of the bucket where the cumulative count
 * reaches ceil(99% of calls) */
static uint32_t i2c_p99(int d)
{
    static const uint32_t hi[8] =
        { 250, 500, 1000, 2000, 4000, 8000, 16000, 0xFFFFFFFFu };
    uint32_t n = s_n_snap[d];
    if (n == 0) return 0;
    uint32_t need = (n * 99 + 99) / 100;
    uint32_t acc = 0;
    for (int b = 0; b < 8; b++) {
        acc += s_hist_snap[d][b];
        if (acc >= need) return hi[b];
    }
    return 0xFFFFFFFFu;
}

static void emit_i2c_stat(void)
{
    i2c_snapshot();
    uint32_t javg = g_app.baro_jit_n
                        ? g_app.baro_jit_sum_us / g_app.baro_jit_n : 0;
    uint32_t jmax = g_app.baro_jit_max_us;
    g_app.baro_jit_n = 0;
    g_app.baro_jit_sum_us = 0;
    g_app.baro_jit_max_us = 0;
    emitf("#I,%llu,"
          "b,%lu,%lu,%lu,%lu,q,%lu,%lu,%lu,%lu,s,%lu,%lu,%lu,%lu,"
          "stq,%lu,sts,%lu,jm,%lu,ja,%lu,qh,%lu,nak,%lu,tmo,%lu,"
          "ovf,%lu,bd,%lu,id,%lu\n",
          (unsigned long long)esp_timer_get_time(),
          (unsigned long)s_n_snap[0], (unsigned long)s_fail_snap[0],
          (unsigned long)s_max_snap[0], (unsigned long)i2c_p99(0),
          (unsigned long)s_n_snap[1], (unsigned long)s_fail_snap[1],
          (unsigned long)s_max_snap[1], (unsigned long)i2c_p99(1),
          (unsigned long)s_n_snap[2], (unsigned long)s_fail_snap[2],
          (unsigned long)s_max_snap[2], (unsigned long)i2c_p99(2),
          (unsigned long)g_app.imu_stale[0], (unsigned long)g_app.imu_stale[1],
          (unsigned long)jmax, (unsigned long)javg,
          (unsigned long)g_app.tx_q_high,
          (unsigned long)g_app.err_nak, (unsigned long)g_app.err_timeout,
          (unsigned long)g_app.err_overflow, (unsigned long)g_app.baro_dropped,
          (unsigned long)g_app.imu_dropped);
}

/* host commands, one line per command, '\n' terminated:
 *   G,0 / G,1   stop/start confirmed-capture flag (legacy handshake)
 *   C,0 / C,1   stop/start a recording session (same lifecycle as the
 *               BOOT hold — request is executed by the TX task)
 *   Q,0 / Q,1   QMI8658C read scheduling off/on (bench)
 *   T,0 / T,1   external ST IMU read scheduling off/on (bench)
 *   P,0 / P,1   BMP581 read scheduling off/on (bench; buzzer vetoed)
 * Acks are written to the USB port directly (never emit(): an ack must
 * not land inside a recording file). */
static void poll_host_cmd(void)
{
    static char buf[24];
    static int n;
    char c;
    while (usb_serial_jtag_read_bytes(&c, 1, 0) == 1) {
        if (c == '\r') continue;
        if (c != '\n') {
            if (n < (int)sizeof(buf) - 1)
                buf[n++] = c;
            else
                n = 0;              /* oversize garbage: resync */
            continue;
        }
        buf[n] = 0;
        int len = n;
        n = 0;
        if (len < 3 || buf[1] != ',') continue;
        int v = atoi(buf + 2);
        char ack[24];
        int alen = 0;
        switch (buf[0]) {
        case 'G':
            g_app.host_grab = v != 0;
            alen = snprintf(ack, sizeof(ack), "#K,G,%d\n", v != 0);
            break;
        case 'C':
            rec_request(v != 0);
            alen = snprintf(ack, sizeof(ack), "#K,C,%d\n", v != 0);
            break;
        case 'Q':   /* QMI8658C reads on/off (bench) */
            acq_bench_qmi(v);
            alen = snprintf(ack, sizeof(ack), "#K,Q,%d\n", v != 0);
            break;
        case 'T':   /* external ST IMU reads on/off (bench) */
            acq_bench_st(v);
            alen = snprintf(ack, sizeof(ack), "#K,T,%d\n", v != 0);
            break;
        case 'P':   /* BMP581 reads on/off (bench; buzzer vetoed while off) */
            acq_bench_baro(v);
            alen = snprintf(ack, sizeof(ack), "#K,P,%d\n", v != 0);
            break;
        default:
            alen = snprintf(ack, sizeof(ack), "#K,?\n");
            break;
        }
        if (alen > 0)
            usb_serial_jtag_write_bytes(ack, alen, 0);
    }
}

static void tx_task(void *arg)
{
    s_last_conn = usb_serial_jtag_is_connected();
    if (s_last_conn)
        emit_meta("poweron");
    ESP_LOGI(TAG, "tx task up; host %s", s_last_conn ? "connected" : "absent");

    TickType_t last_v = 0;
    uint32_t last_i_ms = 0;
    for (;;) {
        tx_drain();

        /* serialized recording lifecycle (Firmware Reliability Rev 1):
         * this task is the only context that opens/closes the session
         * file, so file writes and the GUI can never race */
        rec_poll();

        /* 1 Hz I2C timing instrumentation (Task 2 bench) */
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
        if (now_ms - last_i_ms >= 1000) {
            last_i_ms = now_ms;
            emit_i2c_stat();
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
    s_bps_last_us = esp_timer_get_time();
    return xTaskCreate(tx_task, "tx", 3072, NULL, tskIDLE_PRIORITY + 5,
                       NULL) == pdPASS ? ESP_OK : ESP_FAIL;
}
