/* app.h — shared application state, accessible from all tasks.
 *
 * Single global struct; individual scalar fields are written/read as
 * single words so brief tears are benign.  Compound reads (GUI composing a
 * screen) take g_app_mu.  See logger.c for task layout.
 */
#ifndef APP_H
#define APP_H

#include <stdbool.h>
#include <stdint.h>
#include "filters.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SCREEN_STATUS 0
#define SCREEN_GRAPH  1
#define SCREEN_VARIO  2
#define SCREEN_N      3
extern const char *g_screen_names[SCREEN_N];

/* graph time windows (Graph screen, 10 s hold cycles them; default 60 s) */
#define GRAPH_WINDOW_N 5
extern const int g_graph_windows[GRAPH_WINDOW_N];   /* 10/30/60/120/300 s */

typedef struct {
    /* ---- acquisition ---- */
    volatile uint32_t baro_seq;   /* global protocol sequence counter, baro */
    volatile uint32_t imu_seq;    /* global protocol sequence counter, imu */
    uint32_t err_nak;             /* I2C NAK / missing device */
    uint32_t err_timeout;         /* I2C timeout / bus reset */
    uint32_t baro_dropped;        /* baro reads that failed or were stale */
    uint32_t imu_dropped;
    uint32_t err_overflow;        /* samples/lines dropped: USB/queue full */
    float p_hz, i_hz;             /* measured sample rates */

    /* ---- latest baro sample ---- */
    uint32_t p_raw;               /* BMP581 raw24 pressure */
    uint32_t t_raw;               /* BMP581 raw24 temperature */
    float alt_m;                  /* altitude rel. fixed 101325 Pa reference */
    int64_t p_t_us;

    /* ---- filters (run in parallel on every baro sample) ---- */
    float v[FILT_N];              /* vertical-speed estimate per filter */
    float alt_f[FILT_N];          /* altitude state per filter */
    int sel_filter;               /* FILT_* — Vario screen + buzzer + V frame */

    /* ---- latest IMU sample (onboard QMI8658C) ---- */
    int16_t acc[3];               /* raw counts, +-8 g -> 4096 LSB/g */
    int16_t gyr[3];               /* raw counts, +-500 dps -> 65.5 LSB/dps */
    int64_t imu_t_us;

    /* ---- health / inventory ---- */
    int bmp_ok, qmi_ok, st_ok;
    int st_addr;                  /* 0x6A/0x6B when st_ok */

    /* ---- battery ---- */
    float bat_v;
    int bat_ok;

    /* ---- UI ---- */
    int screen;                   /* SCREEN_* */
    int btn_held;                 /* 1 while BOOT pressed (debounced) */
    float hold_frac;              /* 0..1 hold progress; <0 = not holding */
    int graph_window_idx;         /* index into g_graph_windows */

    /* ---- recording ---- */
    volatile int rec_active;
    int rec_backend;              /* 1 = USB stream, 2 = SD */
    int host_grab;                /* host tool confirmed it is capturing */
    uint32_t rec_session;         /* monotonically increasing session id */
    uint32_t rec_start_ms;
    uint32_t rec_frames;          /* data frames emitted this session */
    volatile uint32_t sd_err;     /* SD open/write/flush/close failures */

    /* ---- storage ---- */
    int sd_present;               /* microSD mounted at boot */
    uint64_t sd_total_bytes;      /* card capacity (cached at mount) */
    uint64_t sd_free_bytes;       /* free space (cached; mount/close refresh) */
    float stream_bps;             /* measured protocol bytes/s (EMA, tx.c) */
    uint32_t boot_ms;             /* boot timestamp (esp_timer ms domain) */
} app_t;

extern app_t g_app;

/* 10 s running-average vario of the selected filter (maintained by acq.c) */
float app_vario_avg10s(void);

#ifdef __cplusplus
}
#endif
#endif
