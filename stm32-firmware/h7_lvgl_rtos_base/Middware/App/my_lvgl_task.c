/**
 * my_lvgl_task.c --- LVGL GUI task (FreeRTOS)
 * Init: LVGL -> SDMMC -> TF images -> UI -> 50Hz loop
 * Auto-logout: 60s idle -> Login
 */
#include "my_lvgl_task.h"
#include "lvgl.h"
#include "lv_port_disp_template.h"
#include "lv_port_indev_template.h"
#include "cmsis_os.h"
#include "ui.h"
#include "bsp_TF_card.h"
#include "bsp_tf_image_load.h"
#include <stdio.h>


#define AUTO_LOGOUT_MS  60000U

static void idle_timer_cb(lv_timer_t *t)
{
	(void)t;

	if (lv_disp_get_inactive_time(NULL) >= AUTO_LOGOUT_MS) {
		if (lv_scr_act() != ui_Login) {
			if (ui_Login_PwdTA) lv_textarea_set_text(ui_Login_PwdTA, "");
			if (_ui_screen_load(ui_Login, LV_SCR_LOAD_ANIM_MOVE_LEFT, 300, 0)) {
				lv_disp_trig_activity(NULL);
			}
		}
	}
}

void touch_reset_idle(lv_event_t *e)
{
	(void)e;
	lv_disp_trig_activity(NULL);
}




void my_gui_task(void *argument)
{
	lv_init();
	lv_port_disp_init();
	lv_port_indev_init();

	lv_timer_create(idle_timer_cb, 1000, NULL);

	int32_t tf_ret = BSP_TF_Init();
	printf("[TF] Init: %s (ret=%d)\r\n",
	       (tf_ret == BSP_TF_OK) ? "OK" : "FAIL", (int)tf_ret);
	if (tf_ret == BSP_TF_OK) {
		int32_t img_ret = bsp_tf_image_load_all();
		printf("[TF] Image load: %s (ret=%d)\r\n",
		       (img_ret == 0) ? "OK" : "FAIL", (int)img_ret);
	}

	ui_init();
	lv_timer_handler();

	while (1) {
		lv_timer_handler();
		osDelay(20);
	}
}
