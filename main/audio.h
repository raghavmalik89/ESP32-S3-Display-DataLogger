/* audio.h — GPIO16 buzzer driven by the selected filter's vario.
 *
 * Legacy-validated policy: two-state hysteresis latch (vario_core audio
 * state machine) gates the tone; the tone frequency maps climb rate with
 * quantized steps.  Sinks stay silent.  The latch state survives filter
 * switches (no audio glitches on filter change).
 */
#ifndef AUDIO_H
#define AUDIO_H

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t audio_init(void);

/* Called at baro rate with the selected filter's v [m/s].
 * fault != 0 forces the latch off (estimator fault veto). */
void audio_update(float v_m_s, double t_s, int fault);

#ifdef __cplusplus
}
#endif
#endif
