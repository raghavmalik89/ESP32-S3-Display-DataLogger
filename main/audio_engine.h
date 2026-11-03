/* audio_engine.h — one audio tick: vario_core latch -> beep scheduler.
 *
 * Shared verbatim between the firmware (audio.c, which only adds the
 * LEDC PWM backend on top of the result) and the deterministic host
 * test (tests/audio_sched_test.c), so the tested logic IS the shipped
 * logic, call for call. */
#ifndef AUDIO_ENGINE_H
#define AUDIO_ENGINE_H

#include <stdbool.h>

#include "audio_sched.h"
#include "vario_core.h"

static inline void au_engine_step(vario_audio_state_t *lat,
                                  const vario_audio_config_t *cfg,
                                  au_sched_t *sched,
                                  float v, double t_s, int fault,
                                  au_out_t *o)
{
    int latch = vario_audio_update(lat, cfg, v, t_s, fault);
    au_sched_step(sched, t_s, latch != 0, v, o);
}

#endif /* AUDIO_ENGINE_H */
