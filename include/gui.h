#ifndef GUI_H
#define GUI_H

#include "pros/apix.h"

#ifdef __cplusplus
extern "C" {
#endif

enum auton { none, skills_best, skills_real, match_best, match_real, test };

extern enum auton auton_id;

void gui_init();

#ifdef __cplusplus
}
#endif

#endif /* gui.h */