#ifndef UI_FILEMANAGER_H
#define UI_FILEMANAGER_H

#include "lvgl.h"

extern lv_obj_t *ui_FileManager;

void ui_FileManager_screen_init(void);
void ui_FileManager_screen_destroy(void);
void ui_event_FileManager(lv_event_t *e);

#endif /* UI_FILEMANAGER_H */
