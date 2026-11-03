/* gui.h — three-screen LCD UI at ~10 fps, snapshot rendering.
 *
 * Screens (short BOOT click cycles): Status -> Graph -> Vario.
 * 10-second BOOT hold is context-sensitive (rec toggle / graph signal /
 * filter cycle) and its progress is drawn as a bar on every screen.
 * A truthful recording indicator is persistent on all screens.
 */
#ifndef GUI_H
#define GUI_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t gui_start(void);

#ifdef __cplusplus
}
#endif
#endif
