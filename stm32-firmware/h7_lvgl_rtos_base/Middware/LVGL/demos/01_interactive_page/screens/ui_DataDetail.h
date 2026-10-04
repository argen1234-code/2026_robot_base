#ifndef UI_DATADETAIL_H
#define UI_DATADETAIL_H

#include "lvgl.h"

extern lv_obj_t *ui_DataDetail;

void ui_DataDetail_screen_init(void);
void ui_DataDetail_screen_destroy(void);

/* Open detail screen at a specific tab (0=GPS, 1=Speed, 2=IMU, 3=Mode) */
void dd_open_detail(int tab);

#endif
