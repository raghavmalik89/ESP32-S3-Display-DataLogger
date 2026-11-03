# Sources are selected by the LOGGER_PROBE_MODE kconfig (available here as a
# CMake variable through the sdkconfig mechanism).
set(COMMON_SRCS
    "i2c_bus.c"
    "lcd.c"
    "font8x8_basic.c"
    "qmi8658c.c"
    "bmp581.c"
    "lsm6dsv16x_reg.c"
    "imu_lsm6dsv16x.c"
)

if(CONFIG_LOGGER_PROBE_MODE)
    idf_component_register(
        SRCS "main.c" "probe.c" ${COMMON_SRCS}
        INCLUDE_DIRS "."
        REQUIRES esp_driver_i2c esp_driver_spi esp_driver_ledc esp_driver_gpio
                 esp_lcd
    )
else()
    idf_component_register(
        SRCS "main.c" "logger.c" "acq.c" "app_state.c" "audio.c" "btn.c" "gui.c"
             "tx.c" "rec.c" "sd_sink.c" "filters.c" "vario_core.c"
             "baro_proc.c" ${COMMON_SRCS}
        INCLUDE_DIRS "."
        REQUIRES esp_driver_i2c esp_driver_spi esp_driver_ledc esp_driver_gpio
                 esp_driver_sdmmc esp_driver_usb_serial_jtag esp_adc esp_lcd
                 fatfs sdmmc esp_timer freertos
    )
endif()
