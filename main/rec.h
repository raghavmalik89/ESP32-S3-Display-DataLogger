/* rec.h — recording session state machine.
 *
 * IDLE -> STREAMING (USB) or SAVING (SD, when a card is mounted).
 * Truthfulness rules (frozen constraint):
 *   - USB streaming alone is VOLATILE: the screen shows "USB STREAM", never
 *     "SAVED".  Only when the host tool confirms capture ("G,1") does the
 *     indicator add "+HOST".
 *   - SD recording claims "SAVED" only because frames are counted at the
 *     moment they are handed to the SD writer.
 */
#ifndef REC_H
#define REC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* toggle: start a new session or stop the active one */
void rec_toggle(void);

/* short status string for the GUI, always truthful */
const char *rec_status_text(void);

/* recording duration in seconds (0 when idle) */
uint32_t rec_duration_s(void);

#ifdef __cplusplus
}
#endif
#endif
