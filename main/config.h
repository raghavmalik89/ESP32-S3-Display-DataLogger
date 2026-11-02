/* config.h — pin map, rates, protocol constants.
 *
 * Board: Waveshare ESP32-S3-LCD-1.9 (non-touch, ESP32-S3R8, 16MB flash,
 * 8MB octal PSRAM).  Pin assignments verified against the Waveshare ESP-IDF
 * demos and the board manual (see docs/PROJECT.md).
 */
#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

/* ---- Shared I2C bus (onboard QMI8658C + external BMP581 + external ST) -- */
#define I2C_SDA_GPIO        47
#define I2C_SCL_GPIO        48
#define I2C_PORT            I2C_NUM_0
/* DEF-017 legacy: the LSM6DS breakout has no external pull-ups; the ESP32's
 * weak internal ones are out of spec at 400 kHz.  100 kHz is comfortable and
 * far beyond sufficient for 40 Hz baro + 200 Hz IMU traffic. */
#define I2C_FREQ_HZ         100000
/* Legacy alias: the old repo ran a second, weak-pullup bus ("bus A") at
 * 100 kHz.  This design has a single shared bus, so bus A == the main bus. */
#define I2C_FREQ_HZ_A       I2C_FREQ_HZ
#define I2C_TIMEOUT_MS      20

/* ---- LCD (SPI3, ST7789V2-class panel; init table selectable in lcd.c) --- */
#define LCD_SPI_HOST        SPI3_HOST
#define LCD_PIN_CLK         10
#define LCD_PIN_MOSI        13
#define LCD_PIN_DC          11
#define LCD_PIN_CS          12
#define LCD_PIN_RST         9
#define LCD_PIN_BL          14
#define LCD_H_RES           320
#define LCD_V_RES           170

/* ---- Buzzer (passive piezo assumed; LEDC PWM tone) ----------------------- */
#define BUZZER_GPIO         16

/* ---- User input ---------------------------------------------------------- */
#define BOOT_GPIO           0

/* ---- Onboard RGB LED (WS2812 x2) ----------------------------------------- */
#define WS2812_GPIO         15

/* ---- Battery voltage divider (ADC1_CH3) ---------------------------------- */
#define BAT_ADC_GPIO        4
#define BAT_ADC_DIV         3.0f     /* VBAT = VADC * 3 (200k/100k divider) */

/* ---- SD card (SDMMC 1-bit, onboard slot; no card fitted in this phase) --- */
#define SD_PIN_CLK          41
#define SD_PIN_CMD          39
#define SD_PIN_D0           40

/* ---- BMP581 --------------------------------------------------------------- */
#define BMP581_OSR_P        0x04    /* 16X pressure oversampling              */
#define BMP581_OSR_T        0x01    /* 2X temperature oversampling            */
#define BMP581_ODR          0x11    /* 40 Hz ODR code                         */
#define BMP581_P_HZ         40

/* ---- IMU raw scales (both IMUs configured +-8 g / +-500 dps) ------------- */
#define IMU_A_LSB_PER_G     4096    /* 0.244 mg/LSB                           */
#define IMU_G_LSB_PER_DPS   65.5    /* 65.536 LSB/dps                         */

/* ---- Sampling schedule ---------------------------------------------------- */
#define TICK_MS             5       /* base acquisition tick                  */

/* ---- Audio policy (legacy validated baseline) ----------------------------- */
#define AU_LIFT_ON          0.20
#define AU_LIFT_OFF         0.05
#define AU_TONE_BASE_HZ     500
#define AU_TONE_SLOPE_HZ    450
#define AU_TONE_MIN_HZ      400
#define AU_TONE_MAX_HZ      3500
#define AU_TONE_STEP_HZ     25

/* ---- TX ring (USB Serial/JTAG) ------------------------------------------- */
#define TX_RING_BYTES       (64 * 1024)
#define TX_CHUNK_BYTES      2048

#endif
