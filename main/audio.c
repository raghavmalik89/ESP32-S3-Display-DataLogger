/* audio.c — see audio.h. */
#include "audio.h"

#include "config.h"
#include "vario_core.h"

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
static int s_cur_hz;

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
        return;
    }
    int hz = (int)(AU_TONE_BASE_HZ + AU_TONE_SLOPE_HZ * v);
    if (hz < AU_TONE_MIN_HZ) hz = AU_TONE_MIN_HZ;
    if (hz > AU_TONE_MAX_HZ) hz = AU_TONE_MAX_HZ;
    set_tone(hz, 1);
}
