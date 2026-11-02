/* main.c — application entry; dispatch between probe mode and the logger. */
#include <stdio.h>

#include "sdkconfig.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#if CONFIG_LOGGER_PROBE_MODE
void probe_main(void);
#else
void logger_main(void);
#endif

void app_main(void)
{
#if CONFIG_LOGGER_PROBE_MODE
    probe_main();
#else
    logger_main();
#endif
}
