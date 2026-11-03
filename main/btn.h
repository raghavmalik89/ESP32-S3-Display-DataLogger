/* btn.h — BOOT button (GPIO0) gesture detection.
 *
 * Contract (docs/CONTROLS.md, frozen):
 *   short click (< 600 ms press)  -> BTN_CLICK  (screen cycle)
 *   hold to 5.0 s                 -> BTN_HOLD   (context action, fires once)
 *   release before 5 s            -> cancelled, no event
 *   after BTN_HOLD fires          -> button must be released to rearm
 *   press at boot                 -> ignored until released once
 *
 * Hold progress (0..1) is published continuously for the GUI progress bar.
 */
#ifndef BTN_H
#define BTN_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

enum { BTN_NONE = 0, BTN_CLICK = 1, BTN_HOLD = 2 };

/* call once at boot, before gui_start() */
esp_err_t btn_init(void);

/* start the button task (after LCD is up so early presses are safe) */
esp_err_t btn_start(void);

/* returns one of BTN_* ; BTN_CLICK/BTN_HOLD are delivered once per gesture */
int btn_get_event(void);

/* current hold progress 0..1, or <0 when not holding */
float btn_hold_progress(void);

/* 1 while the button is debounced-pressed (GUI feedback) */
int btn_is_pressed(void);

#ifdef __cplusplus
}
#endif
#endif
