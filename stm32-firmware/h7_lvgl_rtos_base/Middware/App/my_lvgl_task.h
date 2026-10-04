#ifndef MY_LVGL_TASK
#define MY_LVGL_TASK

#include "lvgl.h"

//#define LVGL_TASK 0
#define LVGL_TASK 1

#define LVGL_UI_ENABLE_ALL_BACKGROUND         1
#define LVGL_UI_ENABLE_LOGIN_IMAGE_BACKGROUND 0

extern void my_gui_task(void *argument);
extern void touch_reset_idle(lv_event_t *e);
















#endif 
