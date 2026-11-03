/* sd_sink.h — microSD recording backend (SDMMC 1-bit, onboard slot).
 *
 * No card is fitted in the current prototype phase: this module must
 * compile, mount cleanly when a card IS present, and fail gracefully
 * (sd_present = 0) when absent.  Physical recording verification is
 * DEFERRED — do not claim SD recording works until a card has been
 * bench-tested.
 */
#ifndef SD_SINK_H
#define SD_SINK_H

#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* mount the card at boot; sets g_app.sd_present */
esp_err_t sd_init(void);

/* open /sdcard/LOG%05u.CSV for a new session */
esp_err_t sd_open_session(uint32_t session_id);

/* append a formatted frame line; drops silently (and counts) when the
 * card stalls — acquisition must never block on storage */
void sd_write(const char *line, size_t len);

/* flush + close the current session file */
esp_err_t sd_close_session(void);

#ifdef __cplusplus
}
#endif
#endif
