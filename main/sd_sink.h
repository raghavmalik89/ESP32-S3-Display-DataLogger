/* sd_sink.h — microSD recording backend (SDMMC 1-bit, onboard slot).
 *
 * No card is fitted in the current prototype phase: this module must
 * compile, mount cleanly when a card IS present, and fail gracefully
 * (sd_present = 0) when absent.  Physical recording verification is
 * DEFERRED — do not claim SD recording works until a card has been
 * bench-tested.
 *
 * Honesty contract (GUI Rev 2):
 *   - the caller can distinguish "session file open and accepting
 *     writes" from every failure mode via sd_session_state();
 *   - sd_session_bytes() only counts bytes fwrite() confirmed;
 *   - every open/write/flush/close failure increments g_app.sd_err;
 *   - free-space stats are cached (f_getfree walks the whole FAT and is
 *     far too slow for the UI loop): refreshed at mount and after each
 *     session close, and adjusted by the confirmed session bytes while
 *     a session is open.
 */
#ifndef SD_SINK_H
#define SD_SINK_H

#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* mount the card at boot; sets g_app.sd_present and the capacity/free cache */
esp_err_t sd_init(void);

/* reopen the free-space cache (SLOW — call at mount and after close only) */
esp_err_t sd_refresh_stats(void);

/* open /sdcard/LOG%05u.CSV for a new session; every failure counted */
esp_err_t sd_open_session(uint32_t session_id);

/* session file state, for the red/green/yellow indicator */
int sd_session_open(void);
uint32_t sd_session_bytes(void);   /* fwrite-confirmed bytes this session */
uint32_t sd_session_drops(void);   /* short/failed writes this session */

/* current free-space estimate for the UI: cached f_getfree value minus
 * the bytes confirmed written into the open session (no FS calls) */
uint64_t sd_free_now(void);

/* append a formatted frame line; drops silently (and counts) when the
 * card stalls — acquisition must never block on storage */
void sd_write(const char *line, size_t len);

/* flush + close the current session file; failures counted */
esp_err_t sd_close_session(void);

#ifdef __cplusplus
}
#endif
#endif
