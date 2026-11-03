/* btn.c — see btn.h.  Polled at 50 Hz, 20 ms debounce. */
#include "btn.h"

#include <string.h>

#include "app_state.h"
#include "config.h"

#include "driver/gpio.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#define DEBOUNCE_MS      20
#define CLICK_MAX_MS     600
#define HOLD_MS          10000
#define POLL_MS          20

static QueueHandle_t s_ev;
static volatile float s_prog = -1.f;
static volatile int s_pressed;

static int raw_level(void)
{
    return gpio_get_level(BOOT_GPIO);   /* BOOT: active low */
}

esp_err_t btn_init(void)
{
    gpio_config_t c = {
        .pin_bit_mask = 1ULL << BOOT_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&c);
    if (err != ESP_OK) return err;
    s_ev = xQueueCreate(4, sizeof(uint8_t));
    return s_ev ? ESP_OK : ESP_FAIL;
}

static void btn_task(void *arg)
{
    /* boot-time safety: a button held at power-on is NOT a UI gesture */
    while (raw_level() == 0)
        vTaskDelay(pdMS_TO_TICKS(50));

    int pressed = 0;             /* debounced state */
    int fired = 0;               /* HOLD fired, waiting for release */
    uint32_t t_press = 0;
    TickType_t last = xTaskGetTickCount();

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(POLL_MS));
        int raw = raw_level();
        uint32_t now = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

        if (!pressed) {
            if (raw == 0) {
                /* candidate press: confirm after debounce window */
                vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_MS));
                if (raw_level() == 0) {
                    pressed = 1;
                    fired = 0;
                    t_press = now;
                    s_pressed = 1;
                }
            }
        } else {
            uint32_t held = now - t_press;
            if (raw != 0) {
                vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_MS));
                if (raw_level() != 0) {
                    /* released */
                    pressed = 0;
                    s_pressed = 0;
                    s_prog = -1.f;
                    if (!fired && held < CLICK_MAX_MS) {
                        uint8_t ev = BTN_CLICK;
                        xQueueSend(s_ev, &ev, 0);
                    }
                    continue;
                }
            }
            if (fired) {
                s_prog = -1.f;
            } else if (held >= HOLD_MS) {
                uint8_t ev = BTN_HOLD;
                xQueueSend(s_ev, &ev, 0);
                fired = 1;
                s_prog = -1.f;
            } else if (held >= 1000) {
                s_prog = held / (float)HOLD_MS;
            } else {
                s_prog = -1.f;
            }
        }
        (void)last;
    }
}

int btn_get_event(void)
{
    uint8_t ev = BTN_NONE;
    if (s_ev && xQueueReceive(s_ev, &ev, 0) == pdTRUE)
        return ev;
    return BTN_NONE;
}

float btn_hold_progress(void)
{
    float p = s_prog;
    g_app.hold_frac = p;
    return p;
}

int btn_is_pressed(void)
{
    return s_pressed;
}

esp_err_t btn_start(void)
{
    return xTaskCreate(btn_task, "btn", 2048, NULL, tskIDLE_PRIORITY + 3,
                       NULL) == pdPASS ? ESP_OK : ESP_FAIL;
}
