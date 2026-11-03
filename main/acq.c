/* acq.c — see acq.h. */
#include "acq.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "app_state.h"
#include "audio.h"
#include "baro_proc.h"
#include "bmp581.h"
#include "config.h"
#include "filters.h"
#include "i2c_bus.h"
#include "imu_lsm6dsv16x.h"
#include "qmi8658c.h"
#include "tx.h"
#include "vario_core.h"

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

static const char *TAG = "acq";

app_t g_app;
float g_hist_vario[HIST_N];
volatile uint32_t g_hist_head;

uint32_t acq_hist_available(uint32_t win_s)
{
    uint32_t want = win_s * BMP581_P_HZ;
    uint32_t have = g_hist_head;
    return have < want ? have : want;
}

static bmp581_t s_bmp;
static qmi_imu_t s_qmi;
static lsm_imu_t s_st;
static bool s_have_qmi, s_have_st;
static char s_fstate[FILT_N][256];
static vario_fault_t s_fault;
static adc_oneshot_unit_handle_t s_adc;
static adc_cali_handle_t s_adc_cali;
static bool s_bat_ok;

/* 10 s running average of the selected filter's vario */
#define AVG_N (10 * BMP581_P_HZ)
static float s_avg_ring[AVG_N];
static uint32_t s_avg_head, s_avg_count;

float app_vario_avg10s(void)
{
    if (s_avg_count == 0) return 0.f;
    float sum = 0.f;
    uint32_t n = s_avg_count;
    for (uint32_t i = 0; i < n; i++)
        sum += s_avg_ring[i];
    return sum / (float)n;
}

static void bat_init(void)
{
    adc_oneshot_unit_init_cfg_t ucfg = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    if (adc_oneshot_new_unit(&ucfg, &s_adc) != ESP_OK) return;
    adc_oneshot_chan_cfg_t ccfg = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    if (adc_oneshot_config_channel(s_adc, ADC_CHANNEL_3, &ccfg) != ESP_OK)
        return;
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    s_bat_ok = adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_adc_cali) == ESP_OK;
    g_app.bat_ok = s_bat_ok;
}

static void bat_read(void)
{
    if (!s_bat_ok) return;
    int raw, mv;
    if (adc_oneshot_read(s_adc, ADC_CHANNEL_3, &raw) == ESP_OK &&
        adc_cali_raw_to_voltage(s_adc_cali, raw, &mv) == ESP_OK)
        g_app.bat_v = mv / 1000.f * BAT_ADC_DIV;
}

esp_err_t acq_init(void)
{
    memset(&g_app, 0, sizeof(g_app));
    g_app.sel_filter = FILT_C03;          /* C03 is the default */
    g_app.screen = SCREEN_STATUS;
    g_app.graph_window_idx = 2;           /* 60 s default window */
    g_app.hold_frac = -1.f;
    g_app.boot_ms = (uint32_t)(esp_timer_get_time() / 1000);

    ESP_ERROR_CHECK(i2c_bus_init());

    /* BMP581 at 0x47 (fallback 0x46) */
    esp_err_t err = bmp581_init(&s_bmp, g_i2c_bus, 0x47);
    if (err != ESP_OK)
        err = bmp581_init(&s_bmp, g_i2c_bus, 0x46);
    if (err == ESP_OK) {
        g_app.bmp_ok = 1;
        bmp581_start(&s_bmp);
        ESP_LOGI(TAG, "BMP581 ok @0x%02x, %d Hz", s_bmp.addr, BMP581_P_HZ);
    } else {
        ESP_LOGE(TAG, "BMP581 missing: %s", esp_err_to_name(err));
        g_app.err_nak++;
    }

    /* onboard QMI8658C: 0x6B preferred, 0x6A fallback */
    s_have_qmi = qmi_imu_init(&s_qmi, g_i2c_bus, 0x6B) == ESP_OK;
    if (!s_have_qmi)
        s_have_qmi = qmi_imu_init(&s_qmi, g_i2c_bus, 0x6A) == ESP_OK;
    g_app.qmi_ok = s_have_qmi;
    ESP_LOGI(TAG, "QMI8658C %s", s_have_qmi ? "ok" : "MISSING");

    /* external ST IMU: only possible on the address the QMI does NOT own */
    uint8_t st_addr = s_have_qmi ? 0x6A : 0x6B;
    s_have_st = lsm_imu_init(&s_st, g_i2c_bus, st_addr) == ESP_OK;
    g_app.st_ok = s_have_st;
    g_app.st_addr = st_addr;
    if (s_have_st)
        ESP_LOGI(TAG, "external ST IMU ok @0x%02x", st_addr);
    else
        ESP_LOGI(TAG, "external ST IMU not present @0x%02x (ok: optional)",
                 st_addr);

    for (int f = 0; f < FILT_N; f++)
        g_filters[f].reset(s_fstate[f]);
    vario_fault_init(&s_fault, 8.0f, 0.3f, 20, 40);
    audio_init();
    bat_init();
    return ESP_OK;
}

void acq_select_filter(int f)
{
    if (f >= 0 && f < FILT_N)
        g_app.sel_filter = f;
}

static void acq_baro_read(void)
{
    uint32_t tr, pr;
    if (bmp581_read_raw(&s_bmp, &tr, &pr) != ESP_OK) {
        g_app.baro_dropped++;
        g_app.err_nak++;
        return;
    }
    int64_t t_us = esp_timer_get_time();
    double t_s = t_us / 1e6;
    float alt = (float)baro_proc_altitude_bmp581(pr);

    float v[FILT_N];
    for (int f = 0; f < FILT_N; f++) {
        v[f] = g_filters[f].update(s_fstate[f], alt, t_s);
        g_app.v[f] = v[f];
        g_app.alt_f[f] = g_filters[f].altitude(s_fstate[f]);
    }

    int sel = g_app.sel_filter;
    int fault = vario_fault_update(&s_fault, alt, g_app.alt_f[sel],
                                   (float)(1.0 / BMP581_P_HZ));
    audio_update(v[sel], t_s, fault);

    /* vertical-speed history ring (selected filter, baro rate) */
    uint32_t h = g_hist_head;
    g_hist_vario[h % HIST_N] = v[sel];
    g_hist_head = h + 1;

    /* 10 s average of the selected filter */
    s_avg_ring[s_avg_head % AVG_N] = v[sel];
    s_avg_head++;
    if (s_avg_count < AVG_N) s_avg_count++;

    g_app.p_raw = pr;
    g_app.t_raw = tr;
    g_app.alt_m = alt;
    g_app.p_t_us = t_us;
    g_app.baro_seq++;

    acq_baro_evt_t evt = { g_app.baro_seq, (uint64_t)t_us, pr, tr };
    tx_push_baro(&evt);
}

static void acq_imu_read(void)
{
    int16_t r[7];
    bool fresh;
    if (s_have_qmi) {
        if (qmi_imu_read(&s_qmi, r, &fresh) == ESP_OK) {
            memcpy(g_app.acc, r, sizeof(g_app.acc));
            memcpy(g_app.gyr, r + 3, sizeof(g_app.gyr));
            g_app.imu_t_us = esp_timer_get_time();
            g_app.imu_seq++;
            acq_imu_evt_t evt;
            evt.seq = g_app.imu_seq;
            evt.t_us = (uint64_t)g_app.imu_t_us;
            memcpy(evt.acc, r, 6);
            memcpy(evt.gyr, r + 3, 6);
            evt.sid = 0;
            tx_push_imu(&evt);
        } else {
            g_app.imu_dropped++;
            g_app.err_nak++;
        }
    }
    if (s_have_st) {
        int16_t sr[8];
        bool sfresh;
        if (lsm_imu_read(&s_st, sr, &sfresh) == ESP_OK) {
            acq_imu_evt_t evt;
            evt.seq = ++g_app.imu_seq;
            evt.t_us = (uint64_t)esp_timer_get_time();
            memcpy(evt.acc, sr, 6);
            memcpy(evt.gyr, sr + 4, 6);
            evt.sid = 1;
            tx_push_imu(&evt);
        }
    }
}

static void acq_task(void *arg)
{
    const TickType_t period = pdMS_TO_TICKS(TICK_MS);
    TickType_t last = xTaskGetTickCount();
    int div = 0;
    uint32_t p_cnt = 0, i_cnt = 0, last_report = g_app.boot_ms;

    for (;;) {
        vTaskDelayUntil(&last, period);
        acq_imu_read();
        if (++div >= (1000 / TICK_MS) / BMP581_P_HZ) {
            div = 0;
            acq_baro_read();
            p_cnt++;
        }
        i_cnt++;

        uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
        if (now - last_report >= 1000) {
            float el = (now - last_report) / 1000.f;
            g_app.p_hz = p_cnt / el;
            g_app.i_hz = i_cnt / el;
            p_cnt = i_cnt = 0;
            last_report = now;
            bat_read();
        }
    }
}

esp_err_t acq_start(void)
{
    BaseType_t ok = xTaskCreatePinnedToCore(acq_task, "acq", 4096, NULL,
                                            tskIDLE_PRIORITY + 10, NULL, 1);
    return ok == pdPASS ? ESP_OK : ESP_FAIL;
}
