/**
 * ui_Login.c --- Login / startup screen
 *
 * Password: "qmq"
 * On success -> transitions to ui_Screenmain (move left).
 */

#include "../ui.h"
#include <string.h>

lv_obj_t *ui_Login = NULL;
lv_obj_t *ui_Login_PwdTA = NULL;

/* internal widgets */
static lv_obj_t *msg_label = NULL;

static void kb_event_cb(lv_event_t *e)
{
	lv_event_code_t code = lv_event_get_code(e);
	lv_obj_t *kb = lv_event_get_target(e);

	if (code == LV_EVENT_READY || code == LV_EVENT_CANCEL) {
		lv_obj_del(kb);   /* dismiss keyboard */
	}
}


void ui_event_Login(lv_event_t *e)
{
	lv_event_code_t code = lv_event_get_code(e);
	lv_obj_t *btn  = lv_event_get_target(e);
	lv_obj_t *scr  = lv_obj_get_screen(btn);

	if (code == LV_EVENT_CLICKED) {
		const char *pwd = lv_textarea_get_text(ui_Login_PwdTA);
		if (strcmp(pwd, "qmq") == 0) {
			lv_label_set_text(msg_label, "\xE5\xAF\x86\xE7\xA0\x81\xE6\xAD\xA3\xE7\xA1\xAE\xEF\xBC\x8C\xE6\xAD\xA3\xE5\x9C\xA8\xE5\x8A\xA0\xE8\xBD\xBD...");
			lv_obj_clear_flag(msg_label, LV_OBJ_FLAG_HIDDEN);
			lv_obj_set_style_text_color(msg_label, lv_color_hex(UI_COLOR_CYAN), LV_PART_MAIN);
			_ui_screen_load(ui_Screenmain, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 500, 0);
		} else {
			lv_label_set_text(msg_label, "\xE5\xAF\x86\xE7\xA0\x81\xE9\x94\x99\xE8\xAF\xAF\xEF\xBC\x81");
			lv_obj_clear_flag(msg_label, LV_OBJ_FLAG_HIDDEN);
		}
	}
}


/* Called when password textarea is clicked: show keyboard */
static void pwd_ta_event_cb(lv_event_t *e)
{
	if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
		lv_obj_t *kb = lv_keyboard_create(lv_layer_top());
		lv_keyboard_set_textarea(kb, ui_Login_PwdTA);
		lv_obj_add_event_cb(kb, kb_event_cb, LV_EVENT_ALL, NULL);
	}
}


void ui_Login_screen_init(void)
{
	if (ui_Login) return;   /* already created */

	ui_Login = lv_obj_create(NULL);
	lv_obj_clear_flag(ui_Login, LV_OBJ_FLAG_SCROLLABLE);
	lv_obj_set_style_bg_color(ui_Login, lv_color_hex(UI_COLOR_BG), LV_PART_MAIN);
	lv_obj_set_style_bg_opa(ui_Login, 255, LV_PART_MAIN);

#if LVGL_UI_ENABLE_LOGIN_IMAGE_BACKGROUND
	lv_obj_t *bg = lv_img_create(ui_Login);
	lv_img_set_src(bg, &ui_img_1943878613);
	lv_obj_set_size(bg, 1024, 600);
	lv_obj_center(bg);
	lv_obj_clear_flag(bg, LV_OBJ_FLAG_SCROLLABLE);
	lv_obj_move_background(bg);
#endif

	/* ---- Title ---- */
	lv_obj_t *title = lv_label_create(ui_Login);
	lv_label_set_text(title, "\xE6\x88\x91\xE4\xBB\xAC\xE6\x98\xAF\xE4\xB8\x93\xE4\xB8\x9A\xE5\x9B\xA2\xE9\x98\x9F");
	lv_obj_set_style_text_color(title, lv_color_hex(UI_COLOR_CYAN_DARK), LV_PART_MAIN);
	lv_obj_set_style_text_font(title, &ui_font_CN64, LV_PART_MAIN);
	lv_obj_set_align(title, LV_ALIGN_TOP_MID);
	lv_obj_set_y(title, 80);

	/* ---- Username area ---- */
	lv_obj_t *user_ta = lv_textarea_create(ui_Login);
	lv_textarea_set_one_line(user_ta, true);
	lv_textarea_set_text(user_ta, "admin");
	lv_obj_set_width(user_ta, 300);
	lv_obj_set_align(user_ta, LV_ALIGN_CENTER);
	lv_obj_set_y(user_ta, -60);
	lv_obj_set_style_bg_color(user_ta, lv_color_hex(UI_COLOR_CARD), LV_PART_MAIN);
	lv_obj_set_style_border_color(user_ta, lv_color_hex(UI_COLOR_CYAN_SOFT), LV_PART_MAIN);
	lv_obj_set_style_text_color(user_ta, lv_color_hex(UI_COLOR_TEXT), LV_PART_MAIN);

	lv_obj_t *user_lbl = lv_label_create(ui_Login);
	lv_label_set_text(user_lbl, "\xE7\x94\xA8\xE6\x88\xB7\xE5\x90\x8D");
	lv_obj_set_style_text_font(user_lbl, &ui_font_CN14, LV_PART_MAIN);
	lv_obj_set_style_text_color(user_lbl, lv_color_hex(UI_COLOR_TEXT_MUTED), LV_PART_MAIN);
	lv_obj_align_to(user_lbl, user_ta, LV_ALIGN_OUT_TOP_LEFT, 0, -5);

	/* ---- Password area ---- */
	ui_Login_PwdTA = lv_textarea_create(ui_Login);
	lv_textarea_set_one_line(ui_Login_PwdTA, true);
	lv_textarea_set_password_mode(ui_Login_PwdTA, true);
	lv_textarea_set_text(ui_Login_PwdTA, "");
	lv_obj_set_width(ui_Login_PwdTA, 300);
	lv_obj_set_align(ui_Login_PwdTA, LV_ALIGN_CENTER);
	lv_obj_set_y(ui_Login_PwdTA, 10);
	lv_obj_set_style_bg_color(ui_Login_PwdTA, lv_color_hex(UI_COLOR_CARD), LV_PART_MAIN);
	lv_obj_set_style_border_color(ui_Login_PwdTA, lv_color_hex(UI_COLOR_CYAN_SOFT), LV_PART_MAIN);
	lv_obj_set_style_text_color(ui_Login_PwdTA, lv_color_hex(UI_COLOR_TEXT), LV_PART_MAIN);
	lv_obj_add_event_cb(ui_Login_PwdTA, pwd_ta_event_cb, LV_EVENT_CLICKED, NULL);

	lv_obj_t *pwd_lbl = lv_label_create(ui_Login);
	lv_label_set_text(pwd_lbl, "\xE5\xAF\x86\xE7\xA0\x81");
	lv_obj_set_style_text_font(pwd_lbl, &ui_font_CN14, LV_PART_MAIN);
	lv_obj_set_style_text_color(pwd_lbl, lv_color_hex(UI_COLOR_TEXT_MUTED), LV_PART_MAIN);
	lv_obj_align_to(pwd_lbl, ui_Login_PwdTA, LV_ALIGN_OUT_TOP_LEFT, 0, -5);

	/* ---- Login button ---- */
	lv_obj_t *btn = lv_btn_create(ui_Login);
	lv_obj_set_width(btn, 200);
	lv_obj_set_height(btn, 45);
	lv_obj_set_align(btn, LV_ALIGN_CENTER);
	lv_obj_set_y(btn, 90);
	lv_obj_set_style_bg_color(btn, lv_color_hex(UI_COLOR_BLUE_DARK), LV_PART_MAIN);
	lv_obj_set_style_shadow_width(btn, 0, LV_PART_MAIN);

	lv_obj_t *btn_lbl = lv_label_create(btn);
	lv_label_set_text(btn_lbl, "\xE7\x99\xBB\xE5\xBD\x95");
	lv_obj_set_style_text_font(btn_lbl, &ui_font_CN14, LV_PART_MAIN);
	lv_obj_set_style_text_color(btn_lbl, lv_color_hex(UI_COLOR_BG), LV_PART_MAIN);
	lv_obj_center(btn_lbl);

	lv_obj_add_event_cb(btn, ui_event_Login, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(ui_Login, touch_reset_idle, LV_EVENT_PRESSED, NULL);

	/* ---- Error message label (hidden) ---- */
	msg_label = lv_label_create(ui_Login);
	lv_label_set_text(msg_label, "");
	lv_obj_set_style_text_font(msg_label, &ui_font_CN14, LV_PART_MAIN);
	lv_obj_set_style_text_color(msg_label, lv_color_hex(UI_COLOR_DANGER), LV_PART_MAIN);
	lv_obj_set_align(msg_label, LV_ALIGN_CENTER);
	lv_obj_set_y(msg_label, 140);
	lv_obj_add_flag(msg_label, LV_OBJ_FLAG_HIDDEN);
}


void ui_Login_screen_destroy(void)
{
	if (ui_Login) {
		lv_obj_del(ui_Login);
		ui_Login = NULL;
		ui_Login_PwdTA = NULL;
		msg_label = NULL;
	}
}
