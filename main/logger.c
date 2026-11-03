/* logger.c — production application entry (CONFIG_LOGGER_PROBE_MODE=n).
 *
 * Task layout (priorities on a 240 MHz S3, acquisition never blocked):
 *   acq  (prio 10, CPU1) 200 Hz IMU / 40 Hz baro, filters, audio latch
 *   tx   (prio  5, CPU0) USB stream + host handshake
 *   gui  (prio  4, CPU0) 10 fps snapshot rendering, button actions
 *   btn  (prio  3, CPU0) BOOT gesture detection
 */
#include <stdio.h>

#include "acq.h"
#include "app_state.h"
#include "audio.h"
#include "btn.h"
#include "gui.h"
#include "lcd.h"
#include "rec.h"
#include "sd_sink.h"
#include "tx.h"

#include "driver/usb_serial_jtag.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "logger";

void logger_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(300));        /* let USB enumerate */

    esp_err_t err = lcd_init();
    if (err != ESP_OK)
        ESP_LOGE(TAG, "LCD init failed: %s", esp_err_to_name(err));

    ESP_ERROR_CHECK(acq_init());
    sd_init();                             /* absent card is not fatal */
    ESP_ERROR_CHECK(tx_init());

    btn_init();
    ESP_ERROR_CHECK(acq_start());
    ESP_ERROR_CHECK(tx_start());
    ESP_ERROR_CHECK(gui_start());
    ESP_ERROR_CHECK(btn_start());

    ESP_LOGI(TAG, "logger up: screens=%d filters=%s",
             SCREEN_N, "C03/D01/S4/S5");

    /* low-rate health line for the bench (production stream stays clean of
     * anything but protocol frames when the host tool is attached) */
    uint32_t last = 0;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
        if (now - last < 5000) continue;
        last = now;
        /* keep the CDC data stream pure protocol while a host tool is
         * attached; the health line only prints to a plain serial monitor */
        if (usb_serial_jtag_is_connected()) continue;
        ESP_LOGI(TAG,
                 "p=%.1fHz i=%.0fHz alt=%+.1f v=%+.2f/%+.2f/%+.2f/%+.2f "
                 "drop(b%lu i%lu) err(n%lu t%lu o%lu) rec=%d(%s,%lus) sd=%d",
                 g_app.p_hz, g_app.i_hz, g_app.alt_m,
                 g_app.v[0], g_app.v[1], g_app.v[2], g_app.v[3],
                 (unsigned long)g_app.baro_dropped,
                 (unsigned long)g_app.imu_dropped,
                 (unsigned long)g_app.err_nak,
                 (unsigned long)g_app.err_timeout,
                 (unsigned long)g_app.err_overflow,
                 g_app.rec_active, rec_status_text(),
                 (unsigned long)rec_duration_s(), g_app.sd_present);
    }
}
