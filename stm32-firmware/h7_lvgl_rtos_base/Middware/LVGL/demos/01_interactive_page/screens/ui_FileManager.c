/**
 * ui_FileManager.c --- FatFs TF-card browser with text preview
 *
 * Uses LVGL FatFs drive 'S'. Directories can be entered and ASCII text files
 * such as GPS/GPSPTS.CSV can be opened in a read-only viewer.
 */
#include "../ui.h"
#include "../ui_helpers.h"
#include "bsp_tf_image_load.h"
#include <string.h>
#include <stdio.h>

#define FM_MAX_ITEMS       64U
#define FM_TEXT_BUFFER     4096U
#define FM_DOUBLE_CLICK_MS 450U
#define FM_IMAGE_MAX_ZOOM  1024U

typedef struct {
	char name[64];
	uint8_t is_dir;
	uint8_t is_up;
} fm_item_t;

lv_obj_t *ui_FileManager = NULL;
static lv_obj_t *g_list = NULL;
static lv_obj_t *g_title = NULL;
static lv_obj_t *g_viewer = NULL;
static char g_path[128] = "S:/";
static char g_text[FM_TEXT_BUFFER];
static fm_item_t g_items[FM_MAX_ITEMS];
static uint8_t g_item_count = 0U;
static char g_last_bin_name[64] = {0};
static uint32_t g_last_bin_click_tick = 0U;
static lv_obj_t *g_last_bin_button = NULL;

static void refresh_file_list(void);
static void item_click_cb(lv_event_t *e);

static void fm_reset_bin_click(void)
{
	if (g_last_bin_button != NULL) lv_obj_clear_state(g_last_bin_button, LV_STATE_CHECKED);
	g_last_bin_button = NULL;
	g_last_bin_name[0] = '\0';
	g_last_bin_click_tick = 0U;
}

static void file_manager_loaded_cb(lv_event_t *e)
{
	if (lv_event_get_code(e) == LV_EVENT_SCREEN_LOADED) {
		if (g_viewer != NULL) {
			lv_obj_del(g_viewer);
			g_viewer = NULL;
		}
		fm_reset_bin_click();
		refresh_file_list();
	}
}

static uint8_t fm_is_text_file(const char *name)
{
	const char *dot;
	char ext[5] = {0};
	uint8_t i;

	if (name == NULL) return 0U;
	dot = strrchr(name, '.');
	if (dot == NULL || dot[1] == '\0') return 0U;

	for (i = 0U; i < 4U && dot[i + 1U] != '\0'; i++) {
		char c = dot[i + 1U];
		ext[i] = (c >= 'a' && c <= 'z') ? (char)(c - 'a' + 'A') : c;
	}

	return (strcmp(ext, "TXT") == 0 || strcmp(ext, "CSV") == 0 ||
	        strcmp(ext, "LOG") == 0 || strcmp(ext, "MD") == 0) ? 1U : 0U;
}

static uint8_t fm_is_bin_file(const char *name)
{
	const char *dot;
	char b;
	char i;
	char n;

	if (name == NULL) return 0U;
	dot = strrchr(name, '.');
	if (dot == NULL || strlen(dot) != 4U) return 0U;
	b = dot[1]; i = dot[2]; n = dot[3];
	if (b >= 'a' && b <= 'z') b = (char)(b - 'a' + 'A');
	if (i >= 'a' && i <= 'z') i = (char)(i - 'a' + 'A');
	if (n >= 'a' && n <= 'z') n = (char)(n - 'a' + 'A');
	return (b == 'B' && i == 'I' && n == 'N') ? 1U : 0U;
}

static void viewer_close_cb(lv_event_t *e)
{
	if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
	if (g_viewer != NULL) {
		lv_obj_del(g_viewer);
		g_viewer = NULL;
	}
}

static void image_viewer_gesture_cb(lv_event_t *e)
{
	lv_indev_t *indev;
	lv_dir_t direction;

	if (lv_event_get_code(e) != LV_EVENT_GESTURE) return;
	indev = lv_indev_get_act();
	if (indev == NULL) return;
	direction = lv_indev_get_gesture_dir(indev);
	if (direction == LV_DIR_NONE) return;

	if (g_viewer != NULL) {
		lv_obj_del(g_viewer);
		g_viewer = NULL;
	}
}

static void show_bin_image(const char *name)
{
	const lv_img_dsc_t *descriptor;
	lv_obj_t *image;
	lv_obj_t *title;
	lv_obj_t *close;
	lv_obj_t *close_label;
	uint32_t zoom_w;
	uint32_t zoom_h;
	uint32_t zoom;

	descriptor = (const lv_img_dsc_t *)bsp_tf_image_find_descriptor(name);
	if (descriptor == NULL || descriptor->header.w == 0U || descriptor->header.h == 0U) return;

	if (g_viewer != NULL) lv_obj_del(g_viewer);
	g_viewer = lv_obj_create(ui_FileManager);
	lv_obj_set_size(g_viewer, 1000, 570);
	lv_obj_center(g_viewer);
	lv_obj_set_style_radius(g_viewer, 18, 0);
	lv_obj_set_style_bg_color(g_viewer, lv_color_hex(0x102A43), 0);
	lv_obj_set_style_bg_opa(g_viewer, LV_OPA_COVER, 0);
	lv_obj_set_style_border_color(g_viewer, lv_color_hex(UI_COLOR_CYAN), 0);
	lv_obj_set_style_border_width(g_viewer, 2, 0);
	ui_apply_raised_panel(g_viewer);
	lv_obj_clear_flag(g_viewer, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_GESTURE_BUBBLE);
	lv_obj_add_flag(g_viewer, LV_OBJ_FLAG_CLICKABLE);
	lv_obj_add_event_cb(g_viewer, image_viewer_gesture_cb, LV_EVENT_GESTURE, NULL);

	title = lv_label_create(g_viewer);
	lv_label_set_text_fmt(title, "FatFs  %s", name);
	lv_obj_set_width(title, 820);
	lv_label_set_long_mode(title, LV_LABEL_LONG_DOT);
	lv_obj_set_style_text_color(title, lv_color_hex(UI_COLOR_BG), 0);
	lv_obj_set_style_text_font(title, &lv_font_montserrat_14, 0);
	lv_obj_align(title, LV_ALIGN_TOP_LEFT, 12, 2);

	close = lv_btn_create(g_viewer);
	lv_obj_set_size(close, 72, 34);
	lv_obj_align(close, LV_ALIGN_TOP_RIGHT, -2, -7);
	ui_apply_raised_button(close);
	lv_obj_add_event_cb(close, viewer_close_cb, LV_EVENT_CLICKED, NULL);
	close_label = lv_label_create(close);
	lv_label_set_text(close_label, "\xE8\xBF\x94\xE5\x9B\x9E");
	lv_obj_set_style_text_font(close_label, &ui_font_CN14, 0);
	lv_obj_center(close_label);

	image = lv_img_create(g_viewer);
	lv_img_set_src(image, descriptor);
	zoom_w = (930U * 256U) / descriptor->header.w;
	zoom_h = (500U * 256U) / descriptor->header.h;
	zoom = (zoom_w < zoom_h) ? zoom_w : zoom_h;
	if (zoom > FM_IMAGE_MAX_ZOOM) zoom = FM_IMAGE_MAX_ZOOM;
	if (zoom == 0U) zoom = 1U;
	lv_img_set_zoom(image, (uint16_t)zoom);
	lv_obj_align(image, LV_ALIGN_CENTER, 0, 14);
	lv_obj_clear_flag(image, LV_OBJ_FLAG_GESTURE_BUBBLE);
	lv_obj_add_flag(image, LV_OBJ_FLAG_CLICKABLE);
	lv_obj_add_event_cb(image, image_viewer_gesture_cb, LV_EVENT_GESTURE, NULL);
	lv_obj_move_foreground(g_viewer);
}

static void show_text_file(const char *path)
{
	lv_fs_file_t file;
	lv_fs_res_t result;
	uint32_t bytes_read = 0U;
	lv_obj_t *title;
	lv_obj_t *content;
	lv_obj_t *text;
	lv_obj_t *close;
	lv_obj_t *close_label;

	if (path == NULL || !bsp_tf_fs_is_mounted()) return;
	if (!bsp_tf_fs_lock(BSP_TF_FS_WAIT_FOREVER)) return;

	result = lv_fs_open(&file, path, LV_FS_MODE_RD);
	if (result == LV_FS_RES_OK) {
		result = lv_fs_read(&file, g_text, FM_TEXT_BUFFER - 1U, &bytes_read);
		lv_fs_close(&file);
	}
	bsp_tf_fs_unlock();
	if (result != LV_FS_RES_OK) return;

	g_text[bytes_read] = '\0';
	if (bytes_read == FM_TEXT_BUFFER - 1U) {
		const char suffix[] = "\n...";
		memcpy(&g_text[FM_TEXT_BUFFER - sizeof(suffix)], suffix, sizeof(suffix));
	}

	if (g_viewer != NULL) lv_obj_del(g_viewer);
	g_viewer = lv_obj_create(ui_FileManager);
	lv_obj_set_size(g_viewer, 900, 520);
	lv_obj_center(g_viewer);
	lv_obj_set_style_radius(g_viewer, 18, 0);
	lv_obj_set_style_bg_color(g_viewer, lv_color_hex(UI_COLOR_CARD), 0);
	lv_obj_set_style_border_color(g_viewer, lv_color_hex(UI_COLOR_CYAN_SOFT), 0);
	lv_obj_set_style_border_width(g_viewer, 2, 0);
	ui_apply_raised_panel(g_viewer);
	lv_obj_clear_flag(g_viewer, LV_OBJ_FLAG_SCROLLABLE);

	title = lv_label_create(g_viewer);
	lv_label_set_text_fmt(title, "FatFs  %s", path);
	lv_obj_set_width(title, 760);
	lv_label_set_long_mode(title, LV_LABEL_LONG_DOT);
	lv_obj_set_style_text_color(title, lv_color_hex(UI_COLOR_BLUE_DARK), 0);
	lv_obj_set_style_text_font(title, &lv_font_montserrat_14, 0);
	lv_obj_align(title, LV_ALIGN_TOP_LEFT, 12, 4);

	close = lv_btn_create(g_viewer);
	lv_obj_set_size(close, 72, 34);
	lv_obj_align(close, LV_ALIGN_TOP_RIGHT, -4, -4);
	ui_apply_raised_button(close);
	lv_obj_add_event_cb(close, viewer_close_cb, LV_EVENT_CLICKED, NULL);
	close_label = lv_label_create(close);
	lv_label_set_text(close_label, "\xE8\xBF\x94\xE5\x9B\x9E");
	lv_obj_set_style_text_font(close_label, &ui_font_CN14, 0);
	lv_obj_center(close_label);

	content = lv_obj_create(g_viewer);
	lv_obj_set_size(content, 850, 440);
	lv_obj_align(content, LV_ALIGN_BOTTOM_MID, 0, -4);
	lv_obj_set_style_radius(content, 10, 0);
	lv_obj_set_style_bg_color(content, lv_color_hex(0xF7FBFC), 0);
	lv_obj_set_style_border_width(content, 0, 0);
	lv_obj_set_scroll_dir(content, LV_DIR_VER);

	text = lv_label_create(content);
	lv_obj_set_width(text, 810);
	lv_label_set_long_mode(text, LV_LABEL_LONG_WRAP);
	lv_label_set_text(text, g_text);
	lv_obj_set_style_text_color(text, lv_color_hex(UI_COLOR_TEXT), 0);
	lv_obj_set_style_text_font(text, &lv_font_montserrat_14, 0);
	lv_obj_align(text, LV_ALIGN_TOP_LEFT, 0, 0);
	lv_obj_move_foreground(g_viewer);
}

void ui_event_FileManager(lv_event_t *e)
{
	if (lv_event_get_code(e) == LV_EVENT_CLICKED ||
	    lv_event_get_code(e) == LV_EVENT_LONG_PRESSED ||
	    lv_event_get_code(e) == LV_EVENT_GESTURE) {
		_ui_screen_change(&ui_Screenmain, LV_SCR_LOAD_ANIM_MOVE_TOP, 500, 200,
		                  &ui_Screenmain_screen_init);
	}
}

static void item_click_cb(lv_event_t *e)
{
	fm_item_t *item = (fm_item_t *)lv_event_get_user_data(e);
	lv_obj_t *button = lv_event_get_target(e);
	char new_path[128];
	char *slash;
	uint32_t now;

	if (lv_event_get_code(e) != LV_EVENT_CLICKED || item == NULL) return;

	if (item->is_up) {
		fm_reset_bin_click();
		if (strlen(g_path) > 3U) {
			g_path[strlen(g_path) - 1U] = '\0';
			slash = strrchr(g_path, '/');
			if (slash != NULL) *(slash + 1) = '\0';
		}
		refresh_file_list();
		return;
	}

	if (item->is_dir) {
		fm_reset_bin_click();
		snprintf(new_path, sizeof(new_path), "%s%s/", g_path, item->name);
		strncpy(g_path, new_path, sizeof(g_path) - 1U);
		g_path[sizeof(g_path) - 1U] = '\0';
		refresh_file_list();
		return;
	}

	if (fm_is_text_file(item->name)) {
		fm_reset_bin_click();
		snprintf(new_path, sizeof(new_path), "%s%s", g_path, item->name);
		show_text_file(new_path);
		return;
	}

	if (fm_is_bin_file(item->name)) {
		now = lv_tick_get();
		if (strcmp(g_last_bin_name, item->name) == 0 &&
		    (now - g_last_bin_click_tick) <= FM_DOUBLE_CLICK_MS) {
			fm_reset_bin_click();
			show_bin_image(item->name);
		} else {
			fm_reset_bin_click();
			strncpy(g_last_bin_name, item->name, sizeof(g_last_bin_name) - 1U);
			g_last_bin_name[sizeof(g_last_bin_name) - 1U] = '\0';
			g_last_bin_click_tick = now;
			g_last_bin_button = button;
			lv_obj_add_state(button, LV_STATE_CHECKED);
		}
		return;
	}

	fm_reset_bin_click();
}

static void refresh_file_list(void)
{
	lv_fs_dir_t dd;
	lv_fs_res_t result;
	char fn[64];
	char label[128];
	char file_path[128];
	const char *raw_name;
	uint8_t is_dir;
	uint32_t size;
	lv_obj_t *button;

	if (g_list != NULL) lv_obj_del(g_list);
	g_last_bin_button = NULL;
	g_last_bin_name[0] = '\0';
	g_last_bin_click_tick = 0U;
	if (g_title != NULL) lv_label_set_text_fmt(g_title, "FatFs  %s", g_path);

	g_list = lv_list_create(ui_FileManager);
	lv_obj_set_size(g_list, 800, 440);
	lv_obj_set_align(g_list, LV_ALIGN_CENTER);
	lv_obj_set_y(g_list, 30);
	lv_obj_set_style_radius(g_list, 10, 0);
	lv_obj_set_style_bg_color(g_list, lv_color_hex(UI_COLOR_CARD), 0);
	ui_apply_raised_panel(g_list);
	g_item_count = 0U;

	if (strcmp(g_path, "S:/") != 0 && g_item_count < FM_MAX_ITEMS) {
		fm_item_t *up = &g_items[g_item_count++];
		memset(up, 0, sizeof(*up));
		strcpy(up->name, "..");
		up->is_up = 1U;
		button = lv_list_add_btn(g_list, NULL, "..");
		lv_obj_add_event_cb(button, item_click_cb, LV_EVENT_CLICKED, up);
	}

	if (!bsp_tf_fs_is_mounted()) {
		lv_list_add_text(g_list, "  FatFs: SD card not mounted");
		return;
	}
	if (!bsp_tf_fs_lock(BSP_TF_FS_WAIT_FOREVER)) {
		lv_list_add_text(g_list, "  FatFs: busy");
		return;
	}

	result = lv_fs_dir_open(&dd, g_path);
	if (result != LV_FS_RES_OK) {
		bsp_tf_fs_unlock();
		lv_list_add_text(g_list, "  FatFs: cannot open directory");
		return;
	}

	while (g_item_count < FM_MAX_ITEMS) {
		result = lv_fs_dir_read(&dd, fn);
		if (result != LV_FS_RES_OK || fn[0] == '\0') break;

		is_dir = (fn[0] == '/') ? 1U : 0U;
		raw_name = is_dir ? &fn[1] : fn;
		if (raw_name[0] == '\0') continue;

		fm_item_t *item = &g_items[g_item_count++];
		memset(item, 0, sizeof(*item));
		strncpy(item->name, raw_name, sizeof(item->name) - 1U);
		item->is_dir = is_dir;

		if (is_dir) {
			snprintf(label, sizeof(label), LV_SYMBOL_DIRECTORY "  %s", raw_name);
		} else {
			lv_fs_file_t file;
			size = 0U;
			snprintf(file_path, sizeof(file_path), "%s%s", g_path, raw_name);
			if (lv_fs_open(&file, file_path, LV_FS_MODE_RD) == LV_FS_RES_OK) {
				lv_fs_seek(&file, 0, LV_FS_SEEK_END);
				lv_fs_tell(&file, &size);
				lv_fs_close(&file);
			}
			if (size >= 1048576U)
				snprintf(label, sizeof(label), LV_SYMBOL_FILE "  %s  (%u MB)", raw_name, (unsigned)(size / 1048576U));
			else if (size >= 1024U)
				snprintf(label, sizeof(label), LV_SYMBOL_FILE "  %s  (%u KB)", raw_name, (unsigned)(size / 1024U));
			else
				snprintf(label, sizeof(label), LV_SYMBOL_FILE "  %s  (%u B)", raw_name, (unsigned)size);
		}

		button = lv_list_add_btn(g_list, NULL, label);
		lv_obj_set_style_border_side(button, LV_BORDER_SIDE_BOTTOM, 0);
		lv_obj_set_style_border_color(button, lv_color_hex(UI_COLOR_CYAN_SOFT), 0);
		lv_obj_set_style_border_width(button, 1, 0);
		lv_obj_add_event_cb(button, item_click_cb, LV_EVENT_CLICKED, item);
	}

	lv_fs_dir_close(&dd);
	bsp_tf_fs_unlock();
}

void ui_FileManager_screen_init(void)
{
	lv_obj_t *hint;

	if (ui_FileManager != NULL) return;

	ui_FileManager = lv_obj_create(NULL);
	lv_obj_clear_flag(ui_FileManager, LV_OBJ_FLAG_SCROLLABLE);
	ui_apply_gradient_background(ui_FileManager, 0xF8FBFF, 0xD7E6F2, LV_GRAD_DIR_VER);

	g_title = lv_label_create(ui_FileManager);
	lv_label_set_text(g_title, "FatFs  S:/");
	lv_obj_set_style_text_color(g_title, lv_color_hex(UI_COLOR_BLUE_DARK), LV_PART_MAIN);
	lv_obj_set_style_text_font(g_title, &lv_font_montserrat_14, LV_PART_MAIN);
	lv_obj_set_align(g_title, LV_ALIGN_TOP_MID);
	lv_obj_set_y(g_title, 5);

	hint = lv_label_create(ui_FileManager);
	lv_label_set_text(hint, "FatFs | \xE6\xBB\x91\xE5\x8A\xA8\xE6\xB5\x8F\xE8\xA7\x88 | \xE7\x82\xB9\xE5\x87\xBB\xE6\xB5\x8F\xE8\xA7\x88");
	lv_obj_set_style_text_color(hint, lv_color_hex(UI_COLOR_TEXT_MUTED), LV_PART_MAIN);
	lv_obj_set_style_text_font(hint, &ui_font_CN14, LV_PART_MAIN);
	lv_obj_set_align(hint, LV_ALIGN_BOTTOM_MID);
	lv_obj_set_y(hint, -10);

	strcpy(g_path, "S:/");
	refresh_file_list();
	lv_obj_add_event_cb(ui_FileManager, touch_reset_idle, LV_EVENT_PRESSED, NULL);
	lv_obj_add_event_cb(ui_FileManager, ui_event_FileManager, LV_EVENT_CLICKED, NULL);
	lv_obj_add_event_cb(ui_FileManager, file_manager_loaded_cb, LV_EVENT_SCREEN_LOADED, NULL);
}

void ui_FileManager_screen_destroy(void)
{
	if (ui_FileManager != NULL) {
		lv_obj_del(ui_FileManager);
		ui_FileManager = NULL;
		g_list = NULL;
		g_title = NULL;
		g_viewer = NULL;
		g_item_count = 0U;
		g_last_bin_button = NULL;
		g_last_bin_name[0] = '\0';
		g_last_bin_click_tick = 0U;
	}
}
