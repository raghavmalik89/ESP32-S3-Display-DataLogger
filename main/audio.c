/* audio.c — see audio.h. */
#include "audio.h"

#include "config.h"
#include "vario_core.h"

#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"

#include <math.h>

static const char *TAG = "audio";

static vario_audio_state_t s_lat;
static vario_audio_config_t s_cfg = {
    .lift_on = AU_LIFT_ON,
    .lift_off = AU_LIFT_OFF,
    .hold_s = 0.25,
    .min_on_s = 0.10,
    .max_on_s = 30.0,
};
static int s_cur_hz;

/* ---- beep profile: climb m/s -> {freq Hz, ON ms, OFF ms} ---------------
 * Anchor table with linear interpolation between anchors.  These are
 * initial tuning values (config constants), NOT flight-validated. */
typedef struct {
    float v;            /* climb rate anchor (m/s) */
    uint16_t freq_hz;
    uint16_t on_ms;
    uint16_t off_ms;
} au_prof_t;

static const au_prof_t s_prof[] = {
    { 0.20f,  650, 100, 600 },
    { 0.50f,  750, 120, 430 },
    { 1.00f,  950, 140, 260 },
    { 2.00f, 1150, 140, 160 },
    { 3.00f, 1400, 160, 100 },
    { 5.00f, 1700, 180,  80 },
};
#define AU_PROF_N (sizeof(s_prof) / sizeof(s_prof[0]))

/* Nonblocking beep scheduler state.  t_s (monotonic sample time) drives
 * the ON/OFF phases; no new timer/task.  The schedule restarts whenever
 * the interpolated profile changes, so a filter switch never emits a
 * partial glitch period (the vario_core latch already survives it). */
static bool s_beep_active;
static double s_beep_t0;          /* schedule epoch (s) */
static uint16_t s_beep_on_ms, s_beep_off_ms;

static void au_profile(float v, int *hz, int *on_ms, int *off_ms)
{
    if (v <= s_prof[0].v) {
        *hz = s_prof[0].freq_hz; *on_ms = s_prof[0].on_ms;
        *off_ms = s_prof[0].off_ms;
        return;
    }
    const au_prof_t *hi = &s_prof[AU_PROF_N - 1];
    if (v >= hi->v) {
        *hz = hi->freq_hz; *on_ms = hi->on_ms; *off_ms = hi->off_ms;
        return;
    }
    for (size_t i = 0; i + 1 < AU_PROF_N; i++) {
        const au_prof_t *a = &s_prof[i], *b = &s_prof[i + 1];
        if (v < a->v || v >= b->v) continue;
        float f = (v - a->v) / (b->v - a->v);
        *hz     = (int)(a->freq_hz + f * (b->freq_hz - a->freq_hz));
        *on_ms  = (int)(a->on_ms  + f * (b->on_ms  - a->on_ms));
        *off_ms = (int)(a->off_ms + f * (b->off_ms - a->off_ms));
        return;
    }
}

esp_err_t audio_init(void)
{
    ledc_timer_config_t t = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = AU_TONE_BASE_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_channel_config_t c = {
        .gpio_num = BUZZER_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 512,
        .hpoint = 0,
    };
    esp_err_t err = ledc_timer_config(&t);
    if (err == ESP_OK) err = ledc_channel_config(&c);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "buzzer init failed: %s", esp_err_to_name(err));
        return err;
    }
    vario_audio_init(&s_lat);
    s_cur_hz = 0;
    ESP_LOGI(TAG, "buzzer on GPIO%d (tone %d..%d Hz, lift >= %.2f m/s)",
             BUZZER_GPIO, AU_TONE_MIN_HZ, AU_TONE_MAX_HZ, AU_LIFT_ON);
    return ESP_OK;
}

static void set_tone(int hz, int on)
{
    if (on) {
        /* quantize to the legacy step; avoid needless re-programming */
        hz = (hz / AU_TONE_STEP_HZ) * AU_TONE_STEP_HZ;
        if (hz != s_cur_hz) {
            ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0, hz);
            s_cur_hz = hz;
        }
        if (s_cur_hz > 0)
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 512);
    } else {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
        s_cur_hz = 0;
    }
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

void audio_update(float v, double t_s, int fault)
{
    int on = vario_audio_update(&s_lat, &s_cfg, v, t_s, fault);
    /* sinks stay silent; only climb beeps */
    if (!on || v <= 0.f) {
        set_tone(0, 0);
        s_beep_active = false;
        return;
    }

    int hz, on_ms, off_ms;
    au_profile(v, &hz, &on_ms, &off_ms);

    /* profile changed (or schedule not running): restart the phase at the
     * current tick so ON always begins cleanly */
    if (!s_beep_active || hz != s_cur_hz
        || on_ms != s_beep_on_ms || off_ms != s_beep_off_ms) {
        if (hz != s_cur_hz) {
            ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0,
                          (hz / AU_TONE_STEP_HZ) * AU_TONE_STEP_HZ);
            s_cur_hz = hz;
        }
        s_beep_on_ms = (uint16_t)on_ms;
        s_beep_off_ms = (uint16_t)off_ms;
        s_beep_t0 = t_s;
        s_beep_active = true;
    }

    /* ON while inside the ON window of the current period */
    double period_s = (s_beep_on_ms + s_beep_off_ms) / 1000.0;
    double ph = fmod(t_s - s_beep_t0, period_s);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0,
                  ph * 1000.0 < s_beep_on_ms ? 512 : 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}
