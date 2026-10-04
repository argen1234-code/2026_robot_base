/**
 * ui_DataDisplay.c --- Car status & sensor data dashboard
 *
 * Paginated display: GPS, Speed, IMU, Mode.
 * Click screen -> back to Screenmain.
 */
#include "../ui.h"
#include "../ui_helpers.h"
#include "ui_DataDetail.h"
#include "app_chassis_board.h"
#include "bsp_bluetooth.h"
#include "bsp_road_classification.h"
#include <stdio.h>

#define PANEL_W   360
#define PANEL_H   110
#define SENSOR_TIMEOUT_MS 1500U
#define GPS_TIMEOUT_MS    3000U
#define WHEEL_TARGET_ACTIVE_RPM   10.0f
#define WHEEL_FEEDBACK_ACTIVE_RPM 3.0f
#define WHEEL_ERROR_RPM           35.0f

/* ARMCC5 source strings use explicit UTF-8 bytes for the LVGL Chinese font. */
#define ROAD_TEXT_PREFIX       "\xE8\xB7\xAF\xE9\x9D\xA2: "
#define ROAD_TEXT_NOT_STARTED  "\xE6\x9C\xAA\xE8\xAF\x86\xE5\x88\xAB"
#define ROAD_TEXT_INDOORS      "\xE5\xAE\xA4\xE5\x86\x85\xE5\x9C\xB0\xE9\x9D\xA2"
#define ROAD_TEXT_CEMENT       "\xE6\xB0\xB4\xE6\xB3\xA5\xE8\xB7\xAF"
#define ROAD_TEXT_ASPHALT      "\xE6\x9F\x8F\xE6\xB2\xB9\xE8\xB7\xAF"
#define ROAD_TEXT_OUTDOORS     "\xE5\xAE\xA4\xE5\xA4\x96\xE5\x9C\xB0\xE9\x9D\xA2"

lv_obj_t *ui_DataDisplay = NULL;

static const char * const road_conditions[] = {
	ROAD_TEXT_NOT_STARTED,
	ROAD_TEXT_INDOORS,
	ROAD_TEXT_CEMENT,
	ROAD_TEXT_ASPHALT,
	ROAD_TEXT_OUTDOORS
};

static lv_obj_t *lb_gps = NULL, *lb_speed = NULL, *lb_imu = NULL, *lb_mode = NULL;
static lv_obj_t *lb_road = NULL;
static uint32_t road_last_classification_tick = 0U;
static uint8_t road_model_result_valid = 0U;

static uint8_t sensor_tick_online(uint32_t tick, uint32_t timeout)
{
	uint32_t now = HAL_GetTick();

	return (tick != 0U && (now - tick) <= timeout) ? 1U : 0U;
}

static uint8_t motor_speed_online(const ChassisTelemetry_t *data)
{
	uint8_t i;

	if (data == NULL) return 0U;

	for (i = 0U; i < 4U; i++) {
		if (sensor_tick_online(data->motor_last_update_tick[i], SENSOR_TIMEOUT_MS)) {
			return 1U;
		}
	}

	return 0U;
}

static float ui_absf(float value)
{
	return (value < 0.0f) ? -value : value;
}

static uint8_t wheel_online(const ChassisTelemetry_t *data, uint8_t index)
{
	if (data == NULL || index >= 4U) return 0U;

	return sensor_tick_online(data->motor_last_update_tick[index], SENSOR_TIMEOUT_MS);
}

static uint8_t wheel_fault(const ChassisTelemetry_t *data, uint8_t index)
{
	float target;
	float feedback;
	float err;

	if (!wheel_online(data, index)) return 1U;

	target = (float)data->motor_speed_set[index];
	feedback = (float)data->motor_speed[index];

	if (ui_absf(target) <= WHEEL_TARGET_ACTIVE_RPM) return 0U;
	if (ui_absf(feedback) < WHEEL_FEEDBACK_ACTIVE_RPM) return 1U;
	if ((target > 0.0f && feedback < -WHEEL_FEEDBACK_ACTIVE_RPM) ||
	    (target < 0.0f && feedback > WHEEL_FEEDBACK_ACTIVE_RPM)) return 1U;

	err = ui_absf(target - feedback);
	return (err > WHEEL_ERROR_RPM) ? 1U : 0U;
}

static const char *wheel_summary_text(const ChassisTelemetry_t *data)
{
	uint8_t i;

	if (data == NULL) return "\xE7\xA6\xBB\xE7\xBA\xBF";

	for (i = 0U; i < 4U; i++) {
		if (!wheel_online(data, i)) return "\xE8\xBD\xA6\xE8\xBD\xAE\xE7\xA6\xBB\xE7\xBA\xBF";
	}

	for (i = 0U; i < 4U; i++) {
		if (wheel_fault(data, i)) return "\xE8\xBD\xA6\xE8\xBD\xAE\xE6\x95\x85\xE9\x9A\x9C";
	}

	return "\xE8\xBD\xA6\xE8\xBD\xAE\xE6\xAD\xA3\xE5\xB8\xB8";
}


static const char *mode_name_from_chassis(CarMode_t mode)
{
	switch (mode) {
	case CAR_MODE_IDLE:    return "\xE6\x9C\xAA\xE4\xB8\x8A\xE7\x94\xB5";
	case CAR_MODE_GPS:     return "GPS ONLY";
	case CAR_MODE_GPS_ROS: return "GPS+ROS FUSION";
	case CAR_MODE_REMOTE:  return "\xE5\xBE\xAE\xE4\xBF\xA1\xE9\x81\xA5\xE6\x8E\xA7";
	case CAR_MODE_ROS_INDOOR: return "ROS\xE5\xAE\xA4\xE5\x86\x85";
	case CAR_MODE_INDOOR:  return "\xE8\x93\x9D\xE7\x89\x99\xE9\x81\xA5\xE6\x8E\xA7";
	case CAR_MODE_VOICE:   return "\xE8\xAF\xAD\xE9\x9F\xB3\xE6\x8E\xA7\xE5\x88\xB6";
	default:               return "--";
	}
}

static const char *road_condition_get(void)
{
	BSP_RoadClassificationResult_t result;
	BT_RoadDisplay_t display = BT_ROAD_DISPLAY_NOT_STARTED;
	uint8_t index;
	uint8_t count = (uint8_t)(sizeof(road_conditions) / sizeof(road_conditions[0]));

	BSP_RoadClassification_GetResult(&result);
	if (result.state == BSP_ROAD_CLASSIFICATION_READY && result.has_result) {
		switch (result.class_id) {
		case BSP_ROAD_CLASS_OUTDOOR: display = BT_ROAD_DISPLAY_OUTDOORS; break;
		case BSP_ROAD_CLASS_INDOOR:  display = BT_ROAD_DISPLAY_INDOORS;  break;
		default:                     display = BT_ROAD_DISPLAY_NOT_STARTED; break;
		}

		if (!road_model_result_valid ||
		    result.last_classification_tick != road_last_classification_tick) {
			BT_SetRoadDisplay(display);
			road_last_classification_tick = result.last_classification_tick;
			road_model_result_valid = 1U;
		}
	} else {
		BT_SetRoadDisplay(BT_ROAD_DISPLAY_NOT_STARTED);
		road_last_classification_tick = 0U;
		road_model_result_valid = 0U;
	}

	index = (uint8_t)BT_GetRoadDisplay();
	if (index >= count) index = 0U;
	return road_conditions[index];
}


void ui_event_DataDisplay(lv_event_t *e)
{
	if (lv_event_get_code(e) == LV_EVENT_CLICKED
	    || lv_event_get_code(e) == LV_EVENT_GESTURE
	    || lv_event_get_code(e) == LV_EVENT_LONG_PRESSED) {
		_ui_screen_change(&ui_Screenmain, LV_SCR_LOAD_ANIM_MOVE_LEFT, 500, 200,
		                  &ui_Screenmain_screen_init);
	}
}


/* ---- Refresh the value labels ---- */
static void refresh_values(void)
{
	char buf[64];
	ChassisTelemetry_t data;

	chassis_get_telemetry(&data);

	if (lb_gps)   {
		if (sensor_tick_online(data.gps_last_update_tick, GPS_TIMEOUT_MS)) {
			snprintf(buf,sizeof(buf),
			         "\xE7\xBA\xAC\xE5\xBA\xA6: %.5f\n\xE7\xBB\x8F\xE5\xBA\xA6: %.5f", data.gps_lat, data.gps_lon);
		} else {
			snprintf(buf,sizeof(buf),"\xE7\xA6\xBB\xE7\xBA\xBF");
		}
		lv_label_set_text(lb_gps, buf);
	}
	if (lb_speed) {
		if (motor_speed_online(&data)) {
			snprintf(buf,sizeof(buf),"Vx %.1f Vy %.1f\nWz %.1f %s",
			         data.vx_set,
			         data.vy_set,
			         data.wz_set,
			         wheel_summary_text(&data));
		} else {
			snprintf(buf,sizeof(buf),"\xE7\xA6\xBB\xE7\xBA\xBF");
		}
		lv_label_set_text(lb_speed,buf);
	}
	if (lb_imu)   {
		if (sensor_tick_online(data.ins_last_update_tick, SENSOR_TIMEOUT_MS)) {
			snprintf(buf,sizeof(buf),
			         "R=%.1f P=%.1f Y=%.1f",
			         data.ins_roll,
			         data.ins_pitch,
			         data.ins_yaw);
		} else {
			snprintf(buf,sizeof(buf),"\xE7\xA6\xBB\xE7\xBA\xBF");
		}
		lv_label_set_text(lb_imu, buf);
	}
	if (lb_mode)  { lv_label_set_text(lb_mode, mode_name_from_chassis(data.mode)); }
	if (lb_road)  { snprintf(buf,sizeof(buf),ROAD_TEXT_PREFIX "%s", road_condition_get()); lv_label_set_text(lb_road, buf); }
}

static void panel_click(lv_event_t *e)
{
	lv_obj_t *pnl = lv_event_get_target(e);
	int tab = (int)(uintptr_t)lv_obj_get_user_data(pnl);
	dd_open_detail(tab);
}

static void road_change_cb(lv_event_t *e)
{
	BT_RoadDisplay_t current;
	BT_RoadDisplay_t next;

	if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;

	current = BT_GetRoadDisplay();
	if (current < BT_ROAD_DISPLAY_INDOORS || current >= BT_ROAD_DISPLAY_ASPHALT) {
		next = BT_ROAD_DISPLAY_INDOORS;
	} else {
		next = (BT_RoadDisplay_t)(current + 1U);
	}
	BT_SetRoadDisplay(next);
	refresh_values();
}


/* ---- Helper: create a data panel ---- */
static lv_obj_t *add_panel(lv_obj_t *parent, const char *icon, const char *title,
                           lv_obj_t **val_out, lv_coord_t x, lv_coord_t y)
{
	lv_obj_t *pnl = lv_obj_create(parent);
	lv_obj_set_size(pnl, PANEL_W, PANEL_H);
	lv_obj_set_pos(pnl, x, y);
	lv_obj_set_style_radius(pnl, 12, 0);
	lv_obj_set_style_bg_color(pnl, lv_color_hex(UI_COLOR_CARD), 0);
	lv_obj_set_style_border_color(pnl, lv_color_hex(UI_COLOR_CYAN_SOFT), 0);
	lv_obj_set_style_border_width(pnl, 2, 0);
	ui_apply_raised_panel(pnl);
	lv_obj_clear_flag(pnl, LV_OBJ_FLAG_SCROLLABLE);

	lv_obj_t *ic = lv_label_create(pnl);
	lv_label_set_text(ic, icon);
	lv_obj_set_style_text_font(ic, &lv_font_montserrat_14, 0);
	lv_obj_align(ic, LV_ALIGN_TOP_LEFT, 10, 5);

	lv_obj_t *tl = lv_label_create(pnl);
	lv_label_set_text(tl, title);
	lv_obj_set_style_text_color(tl, lv_color_hex(UI_COLOR_TEXT_MUTED), 0);
	lv_obj_set_style_text_font(tl, &ui_font_CN14, 0);
	lv_obj_align_to(tl, ic, LV_ALIGN_OUT_RIGHT_MID, 8, 0);

	lv_obj_t *vl = lv_label_create(pnl);
	lv_label_set_text(vl, "--");
	lv_obj_set_style_text_color(vl, lv_color_hex(UI_COLOR_CYAN_DARK), 0);
	lv_obj_set_style_text_font(vl, &ui_font_CN14, 0);
	lv_obj_align(vl, LV_ALIGN_LEFT_MID, 20, 15);

	if (val_out) *val_out = vl;
	return pnl;
}

static void add_road_control(lv_obj_t *parent, lv_coord_t y)
{
	lv_obj_t *box = lv_obj_create(parent);
	lv_obj_set_size(box, 460, 94);
	lv_obj_align(box, LV_ALIGN_TOP_MID, 0, y);
	lv_obj_set_style_radius(box, 12, 0);
	lv_obj_set_style_bg_color(box, lv_color_hex(UI_COLOR_CYAN_SOFT), 0);
	lv_obj_set_style_border_width(box, 0, 0);
	ui_apply_raised_panel(box);
	lv_obj_clear_flag(box, LV_OBJ_FLAG_SCROLLABLE);

	lb_road = lv_label_create(box);
	lv_obj_set_size(lb_road, 440, 40);
	lv_label_set_text(lb_road, ROAD_TEXT_PREFIX ROAD_TEXT_NOT_STARTED);
	lv_obj_set_style_text_color(lb_road, lv_color_hex(UI_COLOR_CYAN_DARK), 0);
	lv_obj_set_style_text_font(lb_road, &ui_font_Road, 0);
	lv_obj_set_style_text_align(lb_road, LV_TEXT_ALIGN_CENTER, 0);
	lv_label_set_long_mode(lb_road, LV_LABEL_LONG_CLIP);
	lv_obj_align(lb_road, LV_ALIGN_TOP_MID, 0, 4);

	lv_obj_t *btn = lv_btn_create(box);
	lv_obj_set_size(btn, 120, 30);
	lv_obj_align(btn, LV_ALIGN_BOTTOM_MID, 0, -6);
	lv_obj_set_style_bg_opa(btn, LV_OPA_TRANSP, 0);
	lv_obj_set_style_border_width(btn, 0, 0);
	lv_obj_set_style_shadow_opa(btn, LV_OPA_TRANSP, 0);
	lv_obj_set_style_radius(btn, 8, 0);
	lv_obj_add_event_cb(btn, road_change_cb, LV_EVENT_CLICKED, NULL);

	lv_obj_t *btn_label = lv_label_create(btn);
	lv_label_set_text(btn_label, "");
	lv_obj_center(btn_label);
}


/* ---- Screen init ---- */
static void dd_refresh_timer(lv_timer_t *t)
{
	if (lv_scr_act() == ui_DataDisplay) refresh_values();
}


void ui_DataDisplay_screen_init(void)
{
	if (ui_DataDisplay) return;

	ui_DataDisplay = lv_obj_create(NULL);
	ui_apply_gradient_background(ui_DataDisplay, 0xF2FFFD, 0xD8ECFF, LV_GRAD_DIR_VER);

	/* Title */
	lv_obj_t *t = lv_label_create(ui_DataDisplay);
	lv_label_set_text(t, "\xE8\xBD\xA6\xE8\xBE\x86\xE6\x95\xB0\xE6\x8D\xAE\xE7\x9B\x91\xE6\x8E\xA7");
	lv_obj_set_style_text_color(t, lv_color_hex(UI_COLOR_BLUE_DARK), 0);
	lv_obj_set_style_text_font(t, &ui_font_CN64, 0);
	lv_obj_align(t, LV_ALIGN_TOP_MID, 0, 10);

	lv_coord_t x0 = (1024 - PANEL_W * 2 - 20) / 2;   /* center the 2-column grid */
	lv_coord_t y0 = 100;

	lv_obj_t *p0 = add_panel(ui_DataDisplay, LV_SYMBOL_GPS,   "GPS",      &lb_gps,   x0,             y0);
	lv_obj_t *p1 = add_panel(ui_DataDisplay, LV_SYMBOL_WIFI,  "\xE8\xBD\xA6\xE8\xBD\xAE",     &lb_speed, x0+PANEL_W+20,  y0);
	lv_obj_t *p2 = add_panel(ui_DataDisplay, LV_SYMBOL_SETTINGS, "IMU",   &lb_imu,   x0,             y0+PANEL_H+15);
	lv_obj_t *p3 = add_panel(ui_DataDisplay, LV_SYMBOL_HOME,  "\xE6\xA8\xA1\xE5\xBC\x8F",     &lb_mode,  x0+PANEL_W+20,  y0+PANEL_H+15);

	add_road_control(ui_DataDisplay, y0 + PANEL_H * 2 + 55);

	lv_obj_set_user_data(p0, (void *)0); lv_obj_add_event_cb(p0, panel_click, LV_EVENT_CLICKED, NULL);
	lv_obj_set_user_data(p1, (void *)1); lv_obj_add_event_cb(p1, panel_click, LV_EVENT_CLICKED, NULL);
	lv_obj_set_user_data(p2, (void *)2); lv_obj_add_event_cb(p2, panel_click, LV_EVENT_CLICKED, NULL);
	lv_obj_set_user_data(p3, (void *)3); lv_obj_add_event_cb(p3, panel_click, LV_EVENT_CLICKED, NULL);

	refresh_values();
	lv_obj_add_event_cb(ui_DataDisplay, touch_reset_idle, LV_EVENT_PRESSED, NULL);

	lv_obj_add_event_cb(ui_DataDisplay, ui_event_DataDisplay, LV_EVENT_ALL, NULL);

	/* Periodic refresh: update labels from globals when screen is visible */
	lv_timer_create(dd_refresh_timer, 500, NULL);
}


void ui_DataDisplay_screen_destroy(void)
{
	if (ui_DataDisplay) { lv_obj_del(ui_DataDisplay); ui_DataDisplay = NULL; }
	lb_gps = lb_speed = lb_imu = lb_mode = NULL;
	lb_road = NULL;
}
