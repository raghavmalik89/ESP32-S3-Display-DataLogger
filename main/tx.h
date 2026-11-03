/* tx.h — USB sensor stream + host handshake (see docs/DATA_FORMAT.md).
 *
 * Text protocol, one frame per line at 921600 baud.  Per-stream monotonic
 * sequence counters: a gap in a stream's seq = dropped samples in that
 * stream (the firmware never renumbers).  Frames:
 *   #H/#D/#C/#R  boot meta (re-emitted ~2.5 s after a new host attaches)
 *   B  baro raw24      V  vario estimate (selected filter, 20 Hz)
 *   M  IMU raw         E  error
 *   S  session event   F  filter-change event
 * Host -> device: "G,1" / "G,0" toggles the confirmed-capture flag.
 */
#ifndef TX_H
#define TX_H

#include "acq.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t tx_init(void);
esp_err_t tx_start(void);

/* called from acq; never blocks.  Returns false (and counts an overflow)
 * when the consumer is behind and the sample must be dropped. */
bool tx_push_baro(const acq_baro_evt_t *evt);
bool tx_push_imu(const acq_imu_evt_t *evt);

/* frame emitters for session/filter events (rec.c / gui.c) */
void tx_event_session(int start, uint32_t session_id);
void tx_event_filter(int filter);

#ifdef __cplusplus
}
#endif
#endif
