/* audio.c — see audio.h.
 *
 * Audio Rev 2: the beep profile table and the phase scheduler live in
 * audio_sched.h (pure C99, shared with the deterministic host test).
 * The scheduler preserves the beep phase across parameter changes —
 * pitch updates immediately, ON/OFF durations apply at phase boundaries
 * only, and a fresh cycle starts solely on climb-state entry.  This
 * removes the Rev-1 defect where per-sample interpolation changes reset
 * the cycle epoch and made the cadence irregular.
 *
 * PWM output: LEDC on BUZZER_GPIO (GPIO16), exact per-table frequency
 * (no coarse quantization), duty 512 during the ON window, 0 otherwise.
 * The vario_core latch keeps climb ON +0.20 / OFF +0.05 hysteresis,
 * sink silence and the fault/stale-data veto; this file stays
 * nonblocking — all timing derives from the monotonic sample timestamp
 * passed into audio_update(). */
#include "audio.h"

#include "audio_engine.h"
#include "config.h"

#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"

static const char *TAG = "audio";

static vario_audio_state_t s_lat;
static vario_audio_config_t s_cfg = {
    .lift_on = AU_LIFT_ON,
    .lift_off = AU_LIFT_OFF,
    .hold_s = 0.25,
    .min_on_s = 0.10,
    .max_on_s = 30.0,
};
static au_sched_t s_sched;
static int s_cur_hz;   /* frequency currently programmed in LEDC (0 = none) */

esp_err_t audio_init(void)
{
    ledc_timer_config_t t = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 500,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_channel_config_t c = {
        .gpio_num = BUZZER_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
    };
    esp_err_t err = ledc_timer_config(&t);
    if (err == ESP_OK) err = ledc_channel_config(&c);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "buzzer init failed: %s", esp_err_to_name(err));
        return err;
    }
    vario_audio_init(&s_lat);
    au_sched_init(&s_sched);
    s_cur_hz = 0;
    ESP_LOGI(TAG, "buzzer on GPIO%d (Audio Rev 2 profile, lift >= %.2f m/s)",
             BUZZER_GPIO, AU_LIFT_ON);
    return ESP_OK;
}

/* duty-only path: silence is immediate (fault/stale veto), frequency is
 * left programmed — it does not radiate at duty 0 */
static void duty(bool on)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, on ? 512 : 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

void audio_update(float v, double t_s, int fault)
{
    au_out_t o;
    au_engine_step(&s_lat, &s_cfg, &s_sched, v, t_s, fault, &o);

    if (o.silent) {
        duty(false);
        s_cur_hz = 0;
        return;
    }
    /* pitch: reprogram only when the interpolated value changes; never
     * touches the phase timeline */
    if (o.hz != s_cur_hz) {
        ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0, o.hz);
        s_cur_hz = o.hz;
    }
    duty(o.on);
}
