#ifndef UI_LOGIN_H
#define UI_LOGIN_H

#include "lvgl.h"

extern lv_obj_t *ui_Login;
extern lv_obj_t *ui_Login_PwdTA;     /* password text area */

void ui_event_Login(lv_event_t *e);
void ui_Login_screen_init(void);
void ui_Login_screen_destroy(void);

#endif /* UI_LOGIN_H */
