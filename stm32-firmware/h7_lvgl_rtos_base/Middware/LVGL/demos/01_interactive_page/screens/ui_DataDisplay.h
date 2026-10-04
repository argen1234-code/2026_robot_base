#ifndef UI_DATADISPLAY_H
#define UI_DATADISPLAY_H

#include "lvgl.h"

extern lv_obj_t *ui_DataDisplay;

void ui_DataDisplay_screen_init(void);
void ui_DataDisplay_screen_destroy(void);
void ui_event_DataDisplay(lv_event_t *e);

#endif
