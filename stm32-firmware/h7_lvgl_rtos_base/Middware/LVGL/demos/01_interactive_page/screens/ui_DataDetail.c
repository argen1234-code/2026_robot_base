/**
 * ui_DataDetail.c --- Detail views for GPS/Speed/IMU/Mode panels
 *
 * Uses tabview: 0=GPS, 1=Speed, 2=IMU, 3=Mode.
 * Mode tab has a dial to switch car mode.
 */
#include "../ui.h"
#include "../ui_helpers.h"
#include "app_chassis_board.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define COMPASS_CX          150
#define COMPASS_CY          145
#define COMPASS_RING_SIZE   230
#define COMPASS_LABEL_R     94
#define COMPASS_TRIGO_SCALE 32767L
#define SENSOR_TIMEOUT_MS   1500U
#define GPS_TIMEOUT_MS      3000U
#define WHEEL_TARGET_ACTIVE_RPM   10.0f
#define WHEEL_FEEDBACK_ACTIVE_RPM 3.0f
#define WHEEL_ERROR_RPM           35.0f
#define MODE_DIAL_COUNT           7U
#define GPS_ROUTE_W               610
#define GPS_ROUTE_H               385
#define GPS_ROUTE_PAD             38
#define ATTITUDE_CX               132
#define ATTITUDE_CY               142
#define ATTITUDE_RADIUS           100

lv_obj_t *ui_DataDetail = NULL;
static lv_obj_t *g_tv = NULL;

/* Detail labels for live update */
static lv_obj_t *dl_speed = NULL;
static lv_obj_t *dl_imu   = NULL;
static lv_obj_t *dl_mode  = NULL;
static lv_obj_t *compass_heading = NULL;
static lv_obj_t *compass_cardinal[4] = {NULL, NULL, NULL, NULL};
static lv_obj_t *compass_marker = NULL;
static lv_point_t compass_marker_points[2];
static lv_obj_t *gps_route_box = NULL;
static lv_obj_t *gps_status = NULL;
static lv_obj_t *gps_markers[MAX_WAYPOINTS] = {NULL};
static lv_obj_t *gps_marker_labels[MAX_WAYPOINTS] = {NULL};
static int gps_marker_indices[MAX_WAYPOINTS] = {0};
static int gps_selected_wp_index = -1;
static lv_obj_t *gps_route_lines[MAX_WAYPOINTS - 1] = {NULL};
static lv_point_t gps_route_line_points[MAX_WAYPOINTS - 1][2];
static lv_obj_t *gps_active_line = NULL;
static lv_point_t gps_active_line_points[2];
static lv_obj_t *gps_arrow_lines[2] = {NULL, NULL};
static lv_point_t gps_arrow_points[2][2];
static lv_obj_t *gps_vehicle_marker = NULL;
static lv_obj_t *wheel_cards[4] = {NULL};
static lv_obj_t *wheel_values[4] = {NULL};
static lv_obj_t *wheel_motion = NULL;
static lv_obj_t *attitude_horizon = NULL;
static lv_point_t attitude_horizon_points[2];
static lv_obj_t *attitude_yaw_pointer = NULL;
static lv_point_t attitude_yaw_points[2];
static lv_obj_t *attitude_yaw_marker = NULL;
static lv_obj_t *attitude_wing = NULL;
static lv_point_t attitude_wing_points[5];
static lv_point_t attitude_axis_points[2];
static lv_obj_t *attitude_data = NULL;
static lv_obj_t *attitude_state = NULL;
static lv_obj_t *mode_dial_buttons[MODE_DIAL_COUNT] = {NULL};
static lv_obj_t *mode_needle = NULL;
static lv_point_t mode_needle_points[2];
static lv_timer_t *detail_refresh_timer = NULL;


static const char * const mode_names[MODE_DIAL_COUNT] = {"\xE6\x9C\xAA\xE4\xB8\x8A\xE7\x94\xB5","GPS\xE5\xAF\xBC\xE8\x88\xAA","GPS+ROS","\xE5\xBE\xAE\xE4\xBF\xA1\xE9\x81\xA5\xE6\x8E\xA7","ROS\xE5\xAE\xA4\xE5\x86\x85","\xE8\x93\x9D\xE7\x89\x99\xE9\x81\xA5\xE6\x8E\xA7","\xE8\xAF\xAD\xE9\x9F\xB3\xE6\x8E\xA7\xE5\x88\xB6"};
static int mode_dial_indices[MODE_DIAL_COUNT] = {0, 1, 2, 3, 4, 5, 6};
static ChassisTelemetry_t detail_data;

static void mode_dial_refresh(void);

static void detail_update_data(void)
{
	chassis_get_telemetry(&detail_data);
}

static uint8_t sensor_tick_online(uint32_t tick, uint32_t timeout)
{
	uint32_t now = HAL_GetTick();

	return (tick != 0U && (now - tick) <= timeout) ? 1U : 0U;
}

static uint8_t motor_speed_online(void)
{
	uint8_t i;

	for (i = 0U; i < 4U; i++) {
		if (sensor_tick_online(detail_data.motor_last_update_tick[i], SENSOR_TIMEOUT_MS)) {
			return 1U;
		}
	}

	return 0U;
}

static float ui_absf(float value)
{
	return (value < 0.0f) ? -value : value;
}

static uint8_t wheel_online(uint8_t index)
{
	if (index >= 4U) return 0U;

	return sensor_tick_online(detail_data.motor_last_update_tick[index], SENSOR_TIMEOUT_MS);
}

static const char *wheel_status_text(uint8_t index)
{
	float target;
	float feedback;
	float err;

	if (!wheel_online(index)) return "\xE7\xA6\xBB\xE7\xBA\xBF";

	target = (float)detail_data.motor_speed_set[index];
	feedback = (float)detail_data.motor_speed[index];

	if (ui_absf(target) <= WHEEL_TARGET_ACTIVE_RPM) return "\xE7\xA9\xBA\xE9\x97\xB2";
	if (ui_absf(feedback) < WHEEL_FEEDBACK_ACTIVE_RPM) return "\xE5\xA0\xB5\xE8\xBD\xAC";
	if ((target > 0.0f && feedback < -WHEEL_FEEDBACK_ACTIVE_RPM) ||
	    (target < 0.0f && feedback > WHEEL_FEEDBACK_ACTIVE_RPM)) return "\xE5\x8F\x8D\xE8\xBD\xAC";

	err = ui_absf(target - feedback);
	return (err > WHEEL_ERROR_RPM) ? "\xE8\xAF\xAF\xE5\xB7\xAE" : "\xE6\xAD\xA3\xE5\xB8\xB8";
}

static void gps_project_point(double lat, double lon,
                              double min_lat, double max_lat,
                              double min_lon, double max_lon,
                              lv_coord_t *x, lv_coord_t *y)
{
	double lat_span = max_lat - min_lat;
	double lon_span = max_lon - min_lon;
	double sx;
	double sy;
	double scale;
	double draw_w;
	double draw_h;
	double x0;
	double y0;

	if (lat_span < 0.0000001) lat_span = 0.0000001;
	if (lon_span < 0.0000001) lon_span = 0.0000001;
	sx = (double)(GPS_ROUTE_W - GPS_ROUTE_PAD * 2) / lon_span;
	sy = (double)(GPS_ROUTE_H - GPS_ROUTE_PAD * 2) / lat_span;
	scale = (sx < sy) ? sx : sy;
	draw_w = lon_span * scale;
	draw_h = lat_span * scale;
	x0 = ((double)GPS_ROUTE_W - draw_w) * 0.5;
	y0 = ((double)GPS_ROUTE_H - draw_h) * 0.5;

	*x = (lv_coord_t)(x0 + (lon - min_lon) * scale);
	*y = (lv_coord_t)(y0 + (max_lat - lat) * scale);
	if (*x < 12) *x = 12;
	if (*x > GPS_ROUTE_W - 12) *x = GPS_ROUTE_W - 12;
	if (*y < 12) *y = 12;
	if (*y > GPS_ROUTE_H - 12) *y = GPS_ROUTE_H - 12;
}

static void gps_arrow_refresh(lv_coord_t x0, lv_coord_t y0, lv_coord_t x1, lv_coord_t y1, uint8_t visible)
{
	lv_coord_t dx;
	lv_coord_t dy;
	lv_coord_t norm;
	lv_coord_t tip_x;
	lv_coord_t tip_y;
	lv_coord_t back_x;
	lv_coord_t back_y;
	lv_coord_t side_x;
	lv_coord_t side_y;

	if (gps_arrow_lines[0] == NULL || gps_arrow_lines[1] == NULL) return;
	if (!visible) {
		lv_obj_add_flag(gps_arrow_lines[0], LV_OBJ_FLAG_HIDDEN);
		lv_obj_add_flag(gps_arrow_lines[1], LV_OBJ_FLAG_HIDDEN);
		return;
	}

	dx = x1 - x0;
	dy = y1 - y0;
	norm = (LV_ABS(dx) > LV_ABS(dy)) ? LV_ABS(dx) : LV_ABS(dy);
	if (norm < 2) norm = 2;
	tip_x = x0 + (dx * 7) / 10;
	tip_y = y0 + (dy * 7) / 10;
	back_x = tip_x - (dx * 14) / norm;
	back_y = tip_y - (dy * 14) / norm;
	side_x = (-dy * 8) / norm;
	side_y = ( dx * 8) / norm;

	gps_arrow_points[0][0].x = tip_x;
	gps_arrow_points[0][0].y = tip_y;
	gps_arrow_points[0][1].x = back_x + side_x;
	gps_arrow_points[0][1].y = back_y + side_y;
	gps_arrow_points[1][0].x = tip_x;
	gps_arrow_points[1][0].y = tip_y;
	gps_arrow_points[1][1].x = back_x - side_x;
	gps_arrow_points[1][1].y = back_y - side_y;
	lv_line_set_points(gps_arrow_lines[0], gps_arrow_points[0], 2);
	lv_line_set_points(gps_arrow_lines[1], gps_arrow_points[1], 2);
	lv_obj_clear_flag(gps_arrow_lines[0], LV_OBJ_FLAG_HIDDEN);
	lv_obj_clear_flag(gps_arrow_lines[1], LV_OBJ_FLAG_HIDDEN);
}

static void gps_route_refresh(void)
{
	double min_lat;
	double max_lat;
	double min_lon;
	double max_lon;
	lv_coord_t px[MAX_WAYPOINTS];
	lv_coord_t py[MAX_WAYPOINTS];
	lv_coord_t vehicle_x = 0;
	lv_coord_t vehicle_y = 0;
	uint8_t count = detail_data.gps_route_count;
	uint8_t active = detail_data.gps_current_wp_index;
	int arrived = -1;
	uint8_t i;
	char buf[512];
	const char *phase_text = "\xE8\xB7\xAF\xE7\xBA\xBF\xE5\xB0\xB1\xE7\xBB\xAA";

	if (count > MAX_WAYPOINTS) count = MAX_WAYPOINTS;
	if (gps_selected_wp_index >= (int)count) gps_selected_wp_index = -1;
	for (i = 0U; i < MAX_WAYPOINTS; i++) {
		if (gps_markers[i]) lv_obj_add_flag(gps_markers[i], LV_OBJ_FLAG_HIDDEN);
		if (i < MAX_WAYPOINTS - 1U && gps_route_lines[i]) lv_obj_add_flag(gps_route_lines[i], LV_OBJ_FLAG_HIDDEN);
	}
	if (gps_active_line) lv_obj_add_flag(gps_active_line, LV_OBJ_FLAG_HIDDEN);
	if (gps_vehicle_marker) lv_obj_add_flag(gps_vehicle_marker, LV_OBJ_FLAG_HIDDEN);
	gps_arrow_refresh(0, 0, 0, 0, 0U);

	if (count == 0U) {
		gps_selected_wp_index = -1;
		if (gps_status) {
			snprintf(buf, sizeof(buf),
			         "\xE8\xB7\xAF\xE5\xBE\x84\xE5\xBE\x85\xE5\x91\xBD\n\n\xE9\x87\x87\xE7\x82\xB9  0 / %u\nGPS   %s\n\xE5\x8D\xAB\xE6\x98\x9F  %u\n\n\xE7\xBA\xAC\xE5\xBA\xA6  %.5f\n\xE7\xBB\x8F\xE5\xBA\xA6  %.5f\n\n\xE9\x87\x87\xE9\x9B\x86\xE6\x96\xB0\xE7\x82\xB9\xE5\x90\x8E\xE5\xB0\x86\xE8\x87\xAA\xE5\x8A\xA8\n\xE6\x89\xA9\xE5\xB1\x95\xE5\xB7\xA1\xE8\x88\xAA\xE8\xB7\xAF\xE5\xBE\x84\xE3\x80\x82",
			         (unsigned)MAX_WAYPOINTS,
			         sensor_tick_online(detail_data.gps_last_update_tick, GPS_TIMEOUT_MS) ? "\xE5\x9C\xA8\xE7\xBA\xBF" : "\xE7\xA6\xBB\xE7\xBA\xBF",
			         (unsigned)detail_data.gps_sats,
			         detail_data.gps_lat,
			         detail_data.gps_lon);
			lv_label_set_text(gps_status, buf);
		}
		return;
	}

	min_lat = max_lat = detail_data.gps_route[0].lat;
	min_lon = max_lon = detail_data.gps_route[0].lon;
	for (i = 1U; i < count; i++) {
		if (detail_data.gps_route[i].lat < min_lat) min_lat = detail_data.gps_route[i].lat;
		if (detail_data.gps_route[i].lat > max_lat) max_lat = detail_data.gps_route[i].lat;
		if (detail_data.gps_route[i].lon < min_lon) min_lon = detail_data.gps_route[i].lon;
		if (detail_data.gps_route[i].lon > max_lon) max_lon = detail_data.gps_route[i].lon;
	}
	for (i = 0U; i < count; i++) {
		gps_project_point(detail_data.gps_route[i].lat, detail_data.gps_route[i].lon,
		                  min_lat, max_lat, min_lon, max_lon, &px[i], &py[i]);
	}

	if (detail_data.gps_nav_phase == NAV_PHASE_DWELLING && active < count) arrived = (int)active;
	else if (!detail_data.gps_is_navigating && active < count && detail_data.gps_distance_error < 2.0f) arrived = (int)active;
	else if (active > 0U && active <= count) arrived = (int)active - 1;

	for (i = 0U; i + 1U < count; i++) {
		gps_route_line_points[i][0].x = px[i];
		gps_route_line_points[i][0].y = py[i];
		gps_route_line_points[i][1].x = px[i + 1U];
		gps_route_line_points[i][1].y = py[i + 1U];
		lv_line_set_points(gps_route_lines[i], gps_route_line_points[i], 2);
		lv_obj_set_style_line_width(gps_route_lines[i], (i + 1U == active) ? 7 : 3, 0);
		lv_obj_set_style_line_color(gps_route_lines[i],
		                            lv_color_hex((i + 1U == active) ? UI_COLOR_ORANGE :
		                                         ((int)i < arrived ? UI_COLOR_CYAN_DARK : UI_COLOR_CYAN)), 0);
		lv_obj_clear_flag(gps_route_lines[i], LV_OBJ_FLAG_HIDDEN);
	}

	for (i = 0U; i < count; i++) {
		lv_coord_t size = ((int)i == arrived) ? 34 : ((i == active) ? 28 : 22);
		lv_obj_set_size(gps_markers[i], size, size);
		lv_obj_set_pos(gps_markers[i], px[i] - size / 2, py[i] - size / 2);
		lv_obj_set_style_bg_color(gps_markers[i],
		                          lv_color_hex(((int)i == arrived) ? UI_COLOR_ORANGE :
		                                       (i == active ? UI_COLOR_CYAN : UI_COLOR_CARD)), 0);
		lv_obj_set_style_border_width(gps_markers[i],
		                              ((int)i == gps_selected_wp_index) ? 5 : 3, 0);
		lv_obj_set_style_border_color(gps_markers[i],
		                              lv_color_hex(((int)i == gps_selected_wp_index) ?
		                                           UI_COLOR_BLUE_DARK : UI_COLOR_CYAN_DARK), 0);
		lv_obj_set_style_text_color(gps_marker_labels[i],
		                            lv_color_hex(((int)i == arrived || i == active) ? UI_COLOR_BG : UI_COLOR_TEXT), 0);
		lv_obj_clear_flag(gps_markers[i], LV_OBJ_FLAG_HIDDEN);
	}

	if (sensor_tick_online(detail_data.gps_last_update_tick, GPS_TIMEOUT_MS)) {
		gps_project_point(detail_data.gps_lat, detail_data.gps_lon,
		                  min_lat, max_lat, min_lon, max_lon, &vehicle_x, &vehicle_y);
		lv_obj_set_pos(gps_vehicle_marker, vehicle_x - 7, vehicle_y - 7);
		lv_obj_clear_flag(gps_vehicle_marker, LV_OBJ_FLAG_HIDDEN);
		if (detail_data.gps_is_navigating && active < count) {
			gps_active_line_points[0].x = vehicle_x;
			gps_active_line_points[0].y = vehicle_y;
			gps_active_line_points[1].x = px[active];
			gps_active_line_points[1].y = py[active];
			lv_line_set_points(gps_active_line, gps_active_line_points, 2);
			lv_obj_clear_flag(gps_active_line, LV_OBJ_FLAG_HIDDEN);
			gps_arrow_refresh(vehicle_x, vehicle_y, px[active], py[active], 1U);
		}
	}

	if (detail_data.gps_nav_phase == NAV_PHASE_DWELLING) phase_text = "\xE5\xB7\xB2\xE5\x88\xB0\xE8\xBE\xBE / \xE5\x81\x9C\xE7\x95\x99";
	else if (detail_data.gps_is_navigating) phase_text = "\xE5\xB7\xA1\xE8\x88\xAA\xE4\xB8\xAD";
	else phase_text = "\xE8\xB7\xAF\xE7\xBA\xBF\xE5\xB0\xB1\xE7\xBB\xAA";
	if (gps_status) {
		if (gps_selected_wp_index >= 0) {
			GPS_Point_t *selected = &detail_data.gps_route[gps_selected_wp_index];

			snprintf(buf, sizeof(buf),
			         "%s\n\n\xE9\x87\x87\xE7\x82\xB9  %u / %u\n\xE7\x9B\xAE\xE6\xA0\x87  P%u\n\xE5\xBE\xAA\xE7\x8E\xAF  %s\n\n\xE8\xB7\x9D\xE7\xA6\xBB  %.1f m\n\xE8\x88\xAA\xE5\x90\x91\xE5\xB7\xAE  %+.1f \xE5\xBA\xA6\nGPS   %s / %u \xE6\x98\x9F\n\n\xE7\xBA\xAC\xE5\xBA\xA6  %.5f\n\xE7\xBB\x8F\xE5\xBA\xA6  %.5f\n\n> \xE8\x88\xAA\xE7\x82\xB9 P%u\n\xE7\xBA\xAC\xE5\xBA\xA6 %.8f\n\xE7\xBB\x8F\xE5\xBA\xA6 %.8f",
			         phase_text,
			         (unsigned)count, (unsigned)MAX_WAYPOINTS,
			         (unsigned)((active < count) ? active + 1U : count),
			         detail_data.gps_loop_enable ? "\xE5\xBC\x80\xE5\x90\xAF" : "\xE5\x85\xB3\xE9\x97\xAD",
			         detail_data.gps_distance_error,
			         detail_data.gps_heading_error,
			         sensor_tick_online(detail_data.gps_last_update_tick, GPS_TIMEOUT_MS) ? "\xE5\x9C\xA8\xE7\xBA\xBF" : "\xE7\xA6\xBB\xE7\xBA\xBF",
			         (unsigned)detail_data.gps_sats,
			         detail_data.gps_lat,
			         detail_data.gps_lon,
			         (unsigned)(gps_selected_wp_index + 1),
			         selected->lat,
			         selected->lon);
		} else {
			snprintf(buf, sizeof(buf),
			         "%s\n\n\xE9\x87\x87\xE7\x82\xB9  %u / %u\n\xE7\x9B\xAE\xE6\xA0\x87  P%u\n\xE5\xBE\xAA\xE7\x8E\xAF  %s\n\n\xE8\xB7\x9D\xE7\xA6\xBB  %.1f m\n\xE8\x88\xAA\xE5\x90\x91\xE5\xB7\xAE  %+.1f \xE5\xBA\xA6\nGPS   %s / %u \xE6\x98\x9F\n\n\xE7\xBA\xAC\xE5\xBA\xA6  %.5f\n\xE7\xBB\x8F\xE5\xBA\xA6  %.5f",
			         phase_text,
			         (unsigned)count, (unsigned)MAX_WAYPOINTS,
			         (unsigned)((active < count) ? active + 1U : count),
			         detail_data.gps_loop_enable ? "\xE5\xBC\x80\xE5\x90\xAF" : "\xE5\x85\xB3\xE9\x97\xAD",
			         detail_data.gps_distance_error,
			         detail_data.gps_heading_error,
			         sensor_tick_online(detail_data.gps_last_update_tick, GPS_TIMEOUT_MS) ? "\xE5\x9C\xA8\xE7\xBA\xBF" : "\xE7\xA6\xBB\xE7\xBA\xBF",
			         (unsigned)detail_data.gps_sats,
			         detail_data.gps_lat,
			         detail_data.gps_lon);
		}
		lv_label_set_text(gps_status, buf);
	}
}

static void gps_marker_click_cb(lv_event_t *e)
{
	int *index = (int *)lv_event_get_user_data(e);

	if (lv_event_get_code(e) != LV_EVENT_CLICKED || index == NULL) return;
	if (*index < 0 || *index >= (int)detail_data.gps_route_count || *index >= MAX_WAYPOINTS) return;

	gps_selected_wp_index = *index;
	gps_route_refresh();
}

static void gps_route_create(lv_obj_t *parent)
{
	uint8_t i;
	char buf[8];

	lv_obj_clear_flag(parent, LV_OBJ_FLAG_SCROLLABLE);
	gps_route_box = lv_obj_create(parent);
	lv_obj_set_size(gps_route_box, GPS_ROUTE_W, GPS_ROUTE_H);
	lv_obj_align(gps_route_box, LV_ALIGN_LEFT_MID, 18, 5);
	lv_obj_set_style_radius(gps_route_box, 24, 0);
	lv_obj_set_style_bg_color(gps_route_box, lv_color_hex(UI_COLOR_CARD), 0);
	lv_obj_set_style_bg_opa(gps_route_box, LV_OPA_80, 0);
	lv_obj_set_style_border_width(gps_route_box, 2, 0);
	lv_obj_set_style_border_color(gps_route_box, lv_color_hex(UI_COLOR_CYAN_SOFT), 0);
	ui_apply_raised_panel(gps_route_box);
	lv_obj_clear_flag(gps_route_box, LV_OBJ_FLAG_SCROLLABLE);

	for (i = 0U; i < MAX_WAYPOINTS - 1U; i++) {
		gps_route_lines[i] = lv_line_create(gps_route_box);
		lv_obj_set_style_line_width(gps_route_lines[i], 3, 0);
		lv_obj_set_style_line_color(gps_route_lines[i], lv_color_hex(UI_COLOR_CYAN), 0);
		lv_obj_set_style_line_rounded(gps_route_lines[i], true, 0);
		lv_obj_add_flag(gps_route_lines[i], LV_OBJ_FLAG_HIDDEN);
	}

	gps_active_line = lv_line_create(gps_route_box);
	lv_obj_set_style_line_width(gps_active_line, 7, 0);
	lv_obj_set_style_line_color(gps_active_line, lv_color_hex(UI_COLOR_ORANGE), 0);
	lv_obj_set_style_line_rounded(gps_active_line, true, 0);
	lv_obj_add_flag(gps_active_line, LV_OBJ_FLAG_HIDDEN);

	for (i = 0U; i < 2U; i++) {
		gps_arrow_lines[i] = lv_line_create(gps_route_box);
		lv_obj_set_style_line_width(gps_arrow_lines[i], 5, 0);
		lv_obj_set_style_line_color(gps_arrow_lines[i], lv_color_hex(UI_COLOR_ORANGE), 0);
		lv_obj_set_style_line_rounded(gps_arrow_lines[i], true, 0);
		lv_obj_add_flag(gps_arrow_lines[i], LV_OBJ_FLAG_HIDDEN);
	}

	for (i = 0U; i < MAX_WAYPOINTS; i++) {
		gps_marker_indices[i] = (int)i;
		gps_markers[i] = lv_obj_create(gps_route_box);
		lv_obj_set_size(gps_markers[i], 22, 22);
		lv_obj_set_style_radius(gps_markers[i], LV_RADIUS_CIRCLE, 0);
		lv_obj_set_style_border_width(gps_markers[i], 3, 0);
		lv_obj_set_style_pad_all(gps_markers[i], 0, 0);
		lv_obj_clear_flag(gps_markers[i], LV_OBJ_FLAG_SCROLLABLE);
		lv_obj_add_flag(gps_markers[i], LV_OBJ_FLAG_CLICKABLE);
		lv_obj_add_event_cb(gps_markers[i], gps_marker_click_cb, LV_EVENT_CLICKED,
		                    &gps_marker_indices[i]);
		gps_marker_labels[i] = lv_label_create(gps_markers[i]);
		snprintf(buf, sizeof(buf), "%u", (unsigned)(i + 1U));
		lv_label_set_text(gps_marker_labels[i], buf);
		lv_obj_set_style_text_font(gps_marker_labels[i], &lv_font_montserrat_14, 0);
		lv_obj_center(gps_marker_labels[i]);
		lv_obj_add_flag(gps_markers[i], LV_OBJ_FLAG_HIDDEN);
	}

	gps_vehicle_marker = lv_obj_create(gps_route_box);
	lv_obj_set_size(gps_vehicle_marker, 14, 14);
	lv_obj_set_style_radius(gps_vehicle_marker, LV_RADIUS_CIRCLE, 0);
	lv_obj_set_style_bg_color(gps_vehicle_marker, lv_color_hex(UI_COLOR_BLUE_DARK), 0);
	lv_obj_set_style_border_width(gps_vehicle_marker, 3, 0);
	lv_obj_set_style_border_color(gps_vehicle_marker, lv_color_hex(UI_COLOR_BG), 0);
	lv_obj_clear_flag(gps_vehicle_marker, LV_OBJ_FLAG_SCROLLABLE);
	lv_obj_add_flag(gps_vehicle_marker, LV_OBJ_FLAG_HIDDEN);

	gps_status = lv_label_create(parent);
	lv_label_set_text(gps_status, "\xE8\xB7\xAF\xE5\xBE\x84\xE5\xBE\x85\xE5\x91\xBD");
	lv_obj_set_width(gps_status, 235);
	lv_obj_set_style_text_color(gps_status, lv_color_hex(UI_COLOR_CYAN_DARK), 0);
	lv_obj_set_style_text_font(gps_status, &ui_font_CN14, 0);
	lv_obj_align(gps_status, LV_ALIGN_RIGHT_MID, -10, 0);
	gps_route_refresh();
}

static void speed_detail_refresh(void)
{
	static const char * const wheel_name[4] = {"\xE5\x89\x8D\xE5\xB7\xA6", "\xE5\x89\x8D\xE5\x8F\xB3", "\xE5\x90\x8E\xE5\xB7\xA6", "\xE5\x90\x8E\xE5\x8F\xB3"};
	char buf[128];
	uint8_t i;

	if (dl_speed) {
		lv_label_set_text(dl_speed, motor_speed_online() ? "\xE9\xBA\xA6\xE5\x85\x8B\xE7\xBA\xB3\xE5\xA7\x86\xE9\xA9\xB1\xE5\x8A\xA8" : "\xE9\xA9\xB1\xE5\x8A\xA8\xE7\xA6\xBB\xE7\xBA\xBF");
		lv_obj_set_style_text_color(dl_speed,
		                            lv_color_hex(motor_speed_online() ? UI_COLOR_CYAN_DARK : UI_COLOR_ORANGE), 0);
	}
	if (wheel_motion) {
		snprintf(buf, sizeof(buf), "Vx %+.1f    Vy %+.1f    Wz %+.1f", detail_data.vx_set, detail_data.vy_set, detail_data.wz_set);
		lv_label_set_text(wheel_motion, buf);
	}

	for (i = 0U; i < 4U; i++) {
		const char *status = wheel_status_text(i);
		uint32_t color = UI_COLOR_CYAN;
		if (!wheel_online(i) || strcmp(status, "\xE5\xA0\xB5\xE8\xBD\xAC") == 0 ||
		    strcmp(status, "\xE5\x8F\x8D\xE8\xBD\xAC") == 0 || strcmp(status, "\xE8\xAF\xAF\xE5\xB7\xAE") == 0) color = UI_COLOR_ORANGE;
		else if (strcmp(status, "\xE7\xA9\xBA\xE9\x97\xB2") == 0) color = UI_COLOR_CYAN_SOFT;
		if (wheel_values[i]) {
			snprintf(buf, sizeof(buf), "%s   %s\n\xE7\x9B\xAE\xE6\xA0\x87  %+.0f rpm\n\xE5\x8F\x8D\xE9\xA6\x88  %+.0f rpm",
			         wheel_name[i], status,
			         (float)detail_data.motor_speed_set[i],
			         (float)detail_data.motor_speed[i]);
			lv_label_set_text(wheel_values[i], buf);
		}
		if (wheel_cards[i]) {
			lv_obj_set_style_border_color(wheel_cards[i], lv_color_hex(color), 0);
			lv_obj_set_style_bg_color(wheel_cards[i],
			                          lv_color_hex((color == UI_COLOR_ORANGE) ? 0xFFF1DA : UI_COLOR_CARD), 0);
		}
	}
}

static void wheel_detail_create(lv_obj_t *parent)
{
	static const lv_coord_t card_x[4] = {-270, 270, -270, 270};
	static const lv_coord_t card_y[4] = {-112, -112, 112, 112};
	lv_obj_t *body;
	lv_obj_t *cabin;
	lv_obj_t *nose;
	lv_obj_t *wheel;
	uint8_t i;

	lv_obj_clear_flag(parent, LV_OBJ_FLAG_SCROLLABLE);
	body = lv_obj_create(parent);
	lv_obj_set_size(body, 255, 330);
	lv_obj_center(body);
	lv_obj_set_style_radius(body, 55, 0);
	lv_obj_set_style_bg_color(body, lv_color_hex(UI_COLOR_BLUE_DARK), 0);
	lv_obj_set_style_border_width(body, 5, 0);
	lv_obj_set_style_border_color(body, lv_color_hex(UI_COLOR_CYAN), 0);
	ui_apply_raised_panel(body);
	lv_obj_clear_flag(body, LV_OBJ_FLAG_SCROLLABLE);

	cabin = lv_obj_create(body);
	lv_obj_set_size(cabin, 165, 145);
	lv_obj_align(cabin, LV_ALIGN_TOP_MID, 0, 58);
	lv_obj_set_style_radius(cabin, 35, 0);
	lv_obj_set_style_bg_color(cabin, lv_color_hex(UI_COLOR_CYAN_SOFT), 0);
	lv_obj_set_style_border_width(cabin, 2, 0);
	lv_obj_set_style_border_color(cabin, lv_color_hex(UI_COLOR_CYAN), 0);
	lv_obj_clear_flag(cabin, LV_OBJ_FLAG_SCROLLABLE);

	nose = lv_label_create(body);
	lv_label_set_text(nose, LV_SYMBOL_UP);
	lv_obj_set_style_text_color(nose, lv_color_hex(UI_COLOR_ORANGE), 0);
	lv_obj_set_style_text_font(nose, &ui_font_Font2, 0);
	lv_obj_align(nose, LV_ALIGN_TOP_MID, 0, 12);

	dl_speed = lv_label_create(body);
	lv_label_set_text(dl_speed, "\xE9\xBA\xA6\xE5\x85\x8B\xE7\xBA\xB3\xE5\xA7\x86\xE9\xA9\xB1\xE5\x8A\xA8");
	lv_obj_set_style_text_color(dl_speed, lv_color_hex(UI_COLOR_BG), 0);
	lv_obj_set_style_text_font(dl_speed, &ui_font_CN14, 0);
	lv_obj_align(dl_speed, LV_ALIGN_BOTTOM_MID, 0, -48);

	wheel_motion = lv_label_create(parent);
	lv_label_set_text(wheel_motion, "Vx --    Vy --    Wz --");
	lv_obj_set_style_text_color(wheel_motion, lv_color_hex(UI_COLOR_CYAN_DARK), 0);
	lv_obj_set_style_text_font(wheel_motion, &lv_font_montserrat_14, 0);
	lv_obj_align(wheel_motion, LV_ALIGN_BOTTOM_MID, 0, -12);

	for (i = 0U; i < 4U; i++) {
		wheel = lv_obj_create(parent);
		lv_obj_set_size(wheel, 178, 98);
		lv_obj_align(wheel, LV_ALIGN_CENTER, card_x[i], card_y[i]);
		lv_obj_set_style_radius(wheel, 18, 0);
		lv_obj_set_style_bg_color(wheel, lv_color_hex(UI_COLOR_CARD), 0);
		lv_obj_set_style_border_width(wheel, 3, 0);
		lv_obj_set_style_border_color(wheel, lv_color_hex(UI_COLOR_CYAN), 0);
		ui_apply_raised_panel(wheel);
		lv_obj_clear_flag(wheel, LV_OBJ_FLAG_SCROLLABLE);
		wheel_cards[i] = wheel;

		wheel_values[i] = lv_label_create(wheel);
		lv_label_set_text(wheel_values[i], "--");
		lv_obj_set_style_text_color(wheel_values[i], lv_color_hex(UI_COLOR_TEXT), 0);
		lv_obj_set_style_text_font(wheel_values[i], &ui_font_CN14, 0);
		lv_obj_center(wheel_values[i]);
	}
	speed_detail_refresh();
}

static int16_t compass_norm_angle(float angle)
{
	int32_t deg = (int32_t)(angle + 0.5f);

	while (deg < 0) deg += 360;
	while (deg >= 360) deg -= 360;

	return (int16_t)deg;
}

static lv_coord_t compass_sin(int16_t deg, lv_coord_t radius)
{
	return (lv_coord_t)(((int32_t)lv_trigo_sin(deg) * radius) / COMPASS_TRIGO_SCALE);
}

static lv_coord_t compass_cos(int16_t deg, lv_coord_t radius)
{
	return (lv_coord_t)(((int32_t)lv_trigo_cos(deg) * radius) / COMPASS_TRIGO_SCALE);
}

static void compass_center_obj(lv_obj_t *obj, lv_coord_t x, lv_coord_t y)
{
	if (obj == NULL) return;

	lv_obj_update_layout(obj);
	lv_obj_set_pos(obj, x - lv_obj_get_width(obj) / 2, y - lv_obj_get_height(obj) / 2);
}

static void compass_place_cardinal(lv_obj_t *label, int16_t base_deg, float heading)
{
	int16_t deg;
	lv_coord_t x;
	lv_coord_t y;

	if (label == NULL) return;

	deg = compass_norm_angle((float)base_deg - heading);
	x = COMPASS_CX + compass_sin(deg, COMPASS_LABEL_R);
	y = COMPASS_CY - compass_cos(deg, COMPASS_LABEL_R);
	compass_center_obj(label, x, y);
}

static void compass_refresh(void)
{
	char buf[160];
	float qmc_yaw = 0.0f;
	float qmc_pitch = 0.0f;
	float qmc_roll = 0.0f;
	uint8_t qmc_online = 0U;

	qmc_online = sensor_tick_online(detail_data.mag_last_update_tick, SENSOR_TIMEOUT_MS);
	qmc_yaw = detail_data.mag_yaw;
	qmc_pitch = detail_data.mag_pitch;
	qmc_roll = detail_data.mag_roll;

	if (compass_heading) {
		if (qmc_online) {
			snprintf(buf, sizeof(buf), "%.1f \xE5\xBA\xA6", qmc_yaw);
		} else {
			snprintf(buf, sizeof(buf), "\xE7\xA6\xBB\xE7\xBA\xBF");
		}
		lv_label_set_text(compass_heading, buf);
	}

	if (qmc_online) {
		compass_place_cardinal(compass_cardinal[0], 0, qmc_yaw);
		compass_place_cardinal(compass_cardinal[1], 90, qmc_yaw);
		compass_place_cardinal(compass_cardinal[2], 180, qmc_yaw);
		compass_place_cardinal(compass_cardinal[3], 270, qmc_yaw);
	}

	if (dl_imu) {
		if (qmc_online) snprintf(buf, sizeof(buf), "QMC5883\n\xE4\xBF\xAF\xE4\xBB\xB0  %+.1f\n\xE6\xA8\xAA\xE6\xBB\x9A  %+.1f", qmc_pitch, qmc_roll);
		else snprintf(buf, sizeof(buf), "QMC5883\n\xE7\xA6\xBB\xE7\xBA\xBF");
		lv_label_set_text(dl_imu, buf);
	}
}

static lv_obj_t *compass_label_create(lv_obj_t *parent, const char *text, lv_color_t color)
{
	lv_obj_t *label = lv_label_create(parent);

	lv_label_set_text(label, text);
	lv_obj_set_style_text_color(label, color, 0);
	lv_obj_set_style_text_font(label, &lv_font_montserrat_14, 0);

	return label;
}

static void compass_create(lv_obj_t *parent)
{
	lv_obj_t *box;
	lv_obj_t *ring;

	box = lv_obj_create(parent);
	lv_obj_set_size(box, 310, 330);
	lv_obj_align(box, LV_ALIGN_LEFT_MID, 40, 0);
	lv_obj_set_style_bg_opa(box, LV_OPA_TRANSP, 0);
	lv_obj_set_style_border_width(box, 0, 0);
	lv_obj_clear_flag(box, LV_OBJ_FLAG_SCROLLABLE);

	ring = lv_obj_create(box);
	lv_obj_set_size(ring, COMPASS_RING_SIZE, COMPASS_RING_SIZE);
	lv_obj_set_style_radius(ring, LV_RADIUS_CIRCLE, 0);
	lv_obj_set_style_bg_color(ring, lv_color_hex(UI_COLOR_CYAN_SOFT), 0);
	lv_obj_set_style_bg_opa(ring, LV_OPA_COVER, 0);
	lv_obj_set_style_border_width(ring, 3, 0);
	lv_obj_set_style_border_color(ring, lv_color_hex(UI_COLOR_CYAN), 0);
	ui_apply_raised_panel(ring);
	lv_obj_clear_flag(ring, LV_OBJ_FLAG_SCROLLABLE);
	lv_obj_align(ring, LV_ALIGN_TOP_LEFT, COMPASS_CX - COMPASS_RING_SIZE / 2, COMPASS_CY - COMPASS_RING_SIZE / 2);

	compass_marker = lv_line_create(box);
	compass_marker_points[0].x = COMPASS_CX;
	compass_marker_points[0].y = COMPASS_CY;
	compass_marker_points[1].x = COMPASS_CX;
	compass_marker_points[1].y = COMPASS_CY - 86;
	lv_line_set_points(compass_marker, compass_marker_points, 2);
	lv_obj_set_style_line_width(compass_marker, 4, 0);
	lv_obj_set_style_line_color(compass_marker, lv_color_hex(UI_COLOR_ORANGE), 0);
	lv_obj_set_style_line_rounded(compass_marker, true, 0);

	compass_cardinal[0] = compass_label_create(box, "N", lv_color_hex(UI_COLOR_ORANGE));
	compass_cardinal[1] = compass_label_create(box, "E", lv_color_hex(UI_COLOR_TEXT));
	compass_cardinal[2] = compass_label_create(box, "S", lv_color_hex(UI_COLOR_TEXT));
	compass_cardinal[3] = compass_label_create(box, "W", lv_color_hex(UI_COLOR_TEXT));

	compass_heading = compass_label_create(box, "--.- \xE5\xBA\xA6", lv_color_hex(UI_COLOR_CYAN_DARK));
	lv_obj_set_style_text_font(compass_heading, &ui_font_Road, 0);
	compass_center_obj(compass_heading, COMPASS_CX, COMPASS_CY + 138);
	compass_refresh();
}

static void attitude_refresh(void)
{
	char buf[240];
	float roll = detail_data.ins_roll;
	float pitch = detail_data.ins_pitch;
	float rad;
	float pitch_offset;
	float dx;
	float dy;
	float yaw_rad;
	lv_coord_t yaw_x;
	lv_coord_t yaw_y;
	uint8_t online;

	online = (detail_data.jy901s_online &&
	          sensor_tick_online(detail_data.jy901s_last_update_tick, SENSOR_TIMEOUT_MS)) ? 1U : 0U;
	if (attitude_state) {
		lv_label_set_text(attitude_state, online ? "JY901S  \xE5\x9C\xA8\xE7\xBA\xBF" : "JY901S  \xE7\xA6\xBB\xE7\xBA\xBF");
		lv_obj_set_style_text_color(attitude_state,
		                            lv_color_hex(online ? UI_COLOR_CYAN_DARK : UI_COLOR_ORANGE), 0);
	}
	if (!online) {
		if (attitude_data) lv_label_set_text(attitude_data, "\xE9\x99\x80\xE8\x9E\xBA X  --\n\xE9\x99\x80\xE8\x9E\xBA Y  --\n\xE9\x99\x80\xE8\x9E\xBA Z  --\n\n\xE5\x8A\xA0\xE9\x80\x9F\xE5\xBA\xA6  --");
		if (attitude_yaw_pointer) lv_obj_add_flag(attitude_yaw_pointer, LV_OBJ_FLAG_HIDDEN);
		if (attitude_yaw_marker) lv_obj_add_flag(attitude_yaw_marker, LV_OBJ_FLAG_HIDDEN);
		return;
	}
	if (attitude_yaw_pointer) lv_obj_clear_flag(attitude_yaw_pointer, LV_OBJ_FLAG_HIDDEN);
	if (attitude_yaw_marker) lv_obj_clear_flag(attitude_yaw_marker, LV_OBJ_FLAG_HIDDEN);

	rad = roll * 3.1415926f / 180.0f;
	pitch_offset = pitch * 1.35f;
	if (pitch_offset > 58.0f) pitch_offset = 58.0f;
	if (pitch_offset < -58.0f) pitch_offset = -58.0f;
	dx = cosf(rad) * (float)ATTITUDE_RADIUS;
	dy = sinf(rad) * (float)ATTITUDE_RADIUS;
	attitude_horizon_points[0].x = ATTITUDE_CX - (lv_coord_t)dx;
	attitude_horizon_points[0].y = ATTITUDE_CY + (lv_coord_t)pitch_offset - (lv_coord_t)dy;
	attitude_horizon_points[1].x = ATTITUDE_CX + (lv_coord_t)dx;
	attitude_horizon_points[1].y = ATTITUDE_CY + (lv_coord_t)pitch_offset + (lv_coord_t)dy;
	if (attitude_horizon) lv_line_set_points(attitude_horizon, attitude_horizon_points, 2);

	yaw_rad = detail_data.ins_yaw * 3.1415926f / 180.0f;
	yaw_x = ATTITUDE_CX + (lv_coord_t)(sinf(yaw_rad) * 76.0f);
	yaw_y = ATTITUDE_CY - (lv_coord_t)(cosf(yaw_rad) * 76.0f);
	attitude_yaw_points[0].x = ATTITUDE_CX;
	attitude_yaw_points[0].y = ATTITUDE_CY;
	attitude_yaw_points[1].x = yaw_x;
	attitude_yaw_points[1].y = yaw_y;
	if (attitude_yaw_pointer) lv_line_set_points(attitude_yaw_pointer, attitude_yaw_points, 2);
	if (attitude_yaw_marker) lv_obj_set_pos(attitude_yaw_marker, yaw_x - 6, yaw_y - 6);

	if (attitude_data) {
		snprintf(buf, sizeof(buf),
		         "\xE9\x99\x80\xE8\x9E\xBA X  %+.1f \xE5\xBA\xA6/s\n\xE9\x99\x80\xE8\x9E\xBA Y  %+.1f \xE5\xBA\xA6/s\n\xE9\x99\x80\xE8\x9E\xBA Z  %+.1f \xE5\xBA\xA6/s\n\n\xE5\x8A\xA0\xE9\x80\x9F\xE5\xBA\xA6  %+.2f  %+.2f  %+.2f g\n\n\xE6\xA8\xAA\xE6\xBB\x9A  %+.1f\n\xE4\xBF\xAF\xE4\xBB\xB0  %+.1f\n\xE8\x88\xAA\xE5\x90\x91  %+.1f",
		         detail_data.jy901s_gyro[0], detail_data.jy901s_gyro[1], detail_data.jy901s_gyro[2],
		         detail_data.jy901s_acc[0], detail_data.jy901s_acc[1], detail_data.jy901s_acc[2],
		         detail_data.ins_roll, detail_data.ins_pitch, detail_data.ins_yaw);
		lv_label_set_text(attitude_data, buf);
	}
}

static void attitude_create(lv_obj_t *parent)
{
	lv_obj_t *box;
	lv_obj_t *ring;
	lv_obj_t *axis;

	box = lv_obj_create(parent);
	lv_obj_set_size(box, 480, 330);
	lv_obj_align(box, LV_ALIGN_RIGHT_MID, -8, 0);
	lv_obj_set_style_bg_opa(box, LV_OPA_TRANSP, 0);
	lv_obj_set_style_border_width(box, 0, 0);
	lv_obj_clear_flag(box, LV_OBJ_FLAG_SCROLLABLE);

	ring = lv_obj_create(box);
	lv_obj_set_size(ring, ATTITUDE_RADIUS * 2 + 24, ATTITUDE_RADIUS * 2 + 24);
	lv_obj_set_pos(ring, ATTITUDE_CX - ATTITUDE_RADIUS - 12, ATTITUDE_CY - ATTITUDE_RADIUS - 12);
	lv_obj_set_style_radius(ring, LV_RADIUS_CIRCLE, 0);
	lv_obj_set_style_bg_color(ring, lv_color_hex(0xE8F8FA), 0);
	lv_obj_set_style_bg_opa(ring, LV_OPA_COVER, 0);
	lv_obj_set_style_border_width(ring, 4, 0);
	lv_obj_set_style_border_color(ring, lv_color_hex(UI_COLOR_CYAN), 0);
	ui_apply_raised_panel(ring);
	lv_obj_clear_flag(ring, LV_OBJ_FLAG_SCROLLABLE);

	attitude_horizon = lv_line_create(box);
	lv_obj_set_style_line_width(attitude_horizon, 7, 0);
	lv_obj_set_style_line_color(attitude_horizon, lv_color_hex(UI_COLOR_ORANGE), 0);
	lv_obj_set_style_line_rounded(attitude_horizon, true, 0);

	attitude_yaw_pointer = lv_line_create(box);
	lv_obj_set_style_line_width(attitude_yaw_pointer, 3, 0);
	lv_obj_set_style_line_color(attitude_yaw_pointer, lv_color_hex(UI_COLOR_CYAN_DARK), 0);
	lv_obj_set_style_line_rounded(attitude_yaw_pointer, true, 0);

	attitude_yaw_marker = lv_obj_create(box);
	lv_obj_set_size(attitude_yaw_marker, 12, 12);
	lv_obj_set_style_radius(attitude_yaw_marker, LV_RADIUS_CIRCLE, 0);
	lv_obj_set_style_bg_color(attitude_yaw_marker, lv_color_hex(UI_COLOR_CYAN_DARK), 0);
	lv_obj_set_style_border_width(attitude_yaw_marker, 2, 0);
	lv_obj_set_style_border_color(attitude_yaw_marker, lv_color_hex(UI_COLOR_BG), 0);
	lv_obj_set_style_pad_all(attitude_yaw_marker, 0, 0);
	lv_obj_clear_flag(attitude_yaw_marker, LV_OBJ_FLAG_SCROLLABLE);

	axis = lv_line_create(box);
	attitude_axis_points[0].x = ATTITUDE_CX;
	attitude_axis_points[0].y = ATTITUDE_CY - 84;
	attitude_axis_points[1].x = ATTITUDE_CX;
	attitude_axis_points[1].y = ATTITUDE_CY + 84;
	lv_line_set_points(axis, attitude_axis_points, 2);
	lv_obj_set_style_line_width(axis, 2, 0);
	lv_obj_set_style_line_color(axis, lv_color_hex(UI_COLOR_CYAN_SOFT), 0);

	attitude_wing = lv_line_create(box);
	attitude_wing_points[0].x = ATTITUDE_CX - 70;
	attitude_wing_points[0].y = ATTITUDE_CY;
	attitude_wing_points[1].x = ATTITUDE_CX - 20;
	attitude_wing_points[1].y = ATTITUDE_CY;
	attitude_wing_points[2].x = ATTITUDE_CX;
	attitude_wing_points[2].y = ATTITUDE_CY + 14;
	attitude_wing_points[3].x = ATTITUDE_CX + 20;
	attitude_wing_points[3].y = ATTITUDE_CY;
	attitude_wing_points[4].x = ATTITUDE_CX + 70;
	attitude_wing_points[4].y = ATTITUDE_CY;
	lv_line_set_points(attitude_wing, attitude_wing_points, 5);
	lv_obj_set_style_line_width(attitude_wing, 5, 0);
	lv_obj_set_style_line_color(attitude_wing, lv_color_hex(UI_COLOR_BLUE_DARK), 0);
	lv_obj_set_style_line_rounded(attitude_wing, true, 0);

	attitude_state = lv_label_create(box);
	lv_label_set_text(attitude_state, "JY901S  --");
	lv_obj_set_style_text_font(attitude_state, &ui_font_CN14, 0);
	lv_obj_align(attitude_state, LV_ALIGN_TOP_LEFT, 70, 8);

	attitude_data = lv_label_create(box);
	lv_label_set_text(attitude_data, "\xE9\x99\x80\xE8\x9E\xBA X  --\n\xE9\x99\x80\xE8\x9E\xBA Y  --\n\xE9\x99\x80\xE8\x9E\xBA Z  --");
	lv_obj_set_width(attitude_data, 205);
	lv_obj_set_style_text_color(attitude_data, lv_color_hex(UI_COLOR_CYAN_DARK), 0);
	lv_obj_set_style_text_font(attitude_data, &ui_font_CN14, 0);
	lv_obj_align(attitude_data, LV_ALIGN_RIGHT_MID, 0, 12);
	attitude_refresh();
}

static void detail_refresh_timer_cb(lv_timer_t *t)
{
	(void)t;

	if (lv_scr_act() == ui_DataDetail) {
		detail_update_data();
		gps_route_refresh();
		if (dl_speed) {
			speed_detail_refresh();
		}
		compass_refresh();
		attitude_refresh();
		mode_dial_refresh();
	}
}

static int mode_dial_current_index(void)
{
	int mode = (int)detail_data.mode;

	if (mode < 0 || mode >= (int)MODE_DIAL_COUNT) return 0;

	return mode;
}

static void mode_dial_refresh_to(int active)
{
	uint8_t i;
	static const lv_coord_t needle_x[MODE_DIAL_COUNT] = {0, 77, 105, 68, -68, -105, -77};
	static const lv_coord_t needle_y[MODE_DIAL_COUNT] = {-98, -61, 0, 80, 80, 0, -61};

	if (active < 0 || active >= (int)MODE_DIAL_COUNT) active = 0;

	if (dl_mode) {
		lv_label_set_text(dl_mode, mode_names[active]);
	}

	if (mode_needle) {
		mode_needle_points[0].x = 280;
		mode_needle_points[0].y = 210;
		mode_needle_points[1].x = 280 + needle_x[active];
		mode_needle_points[1].y = 210 + needle_y[active];
		lv_line_set_points(mode_needle, mode_needle_points, 2);
	}

	for (i = 0U; i < MODE_DIAL_COUNT; i++) {
		if (mode_dial_buttons[i] == NULL) continue;

		if ((int)i == active) {
			lv_obj_set_style_bg_color(mode_dial_buttons[i], lv_color_hex(UI_COLOR_CYAN), 0);
			lv_obj_set_style_bg_opa(mode_dial_buttons[i], LV_OPA_COVER, 0);
			lv_obj_set_style_border_color(mode_dial_buttons[i], lv_color_hex(UI_COLOR_CYAN_DARK), 0);
			lv_obj_set_style_text_color(mode_dial_buttons[i], lv_color_hex(UI_COLOR_BG), 0);
		} else {
			lv_obj_set_style_bg_color(mode_dial_buttons[i], lv_color_hex(UI_COLOR_CARD), 0);
			lv_obj_set_style_bg_opa(mode_dial_buttons[i], LV_OPA_COVER, 0);
			lv_obj_set_style_border_color(mode_dial_buttons[i], lv_color_hex(UI_COLOR_CYAN_SOFT), 0);
			lv_obj_set_style_text_color(mode_dial_buttons[i], lv_color_hex(UI_COLOR_TEXT), 0);
		}
	}
}

static void mode_dial_refresh(void)
{
	mode_dial_refresh_to(mode_dial_current_index());
}

static void mode_dial_cb(lv_event_t *e)
{
	int *sel_ptr = (int *)lv_event_get_user_data(e);
	int sel;

	if (sel_ptr == NULL) return;

	sel = *sel_ptr;
	chassis_request_mode((CarMode_t)sel);
	if (sel >= 0 && sel < (int)MODE_DIAL_COUNT) {
		mode_dial_refresh_to(sel);
	}
}

static lv_obj_t *mode_dial_button_create(lv_obj_t *parent, int index, lv_coord_t x, lv_coord_t y)
{
	lv_obj_t *btn = lv_btn_create(parent);
	lv_obj_t *label;

	lv_obj_set_size(btn, 128, 38);
	lv_obj_align(btn, LV_ALIGN_CENTER, x, y);
	lv_obj_set_style_radius(btn, 19, 0);
	lv_obj_set_style_border_width(btn, 2, 0);
	ui_apply_raised_button(btn);
	lv_obj_add_event_cb(btn, mode_dial_cb, LV_EVENT_CLICKED, &mode_dial_indices[index]);

	label = lv_label_create(btn);
	lv_label_set_text(label, mode_names[index]);
	lv_obj_set_style_text_font(label, &ui_font_CN14, 0);
	lv_obj_center(label);

	mode_dial_buttons[index] = btn;
	return btn;
}

static void mode_dial_create(lv_obj_t *parent)
{
	lv_obj_t *box;
	lv_obj_t *ring;
	lv_obj_t *center;

	lv_obj_clear_flag(parent, LV_OBJ_FLAG_SCROLLABLE);

	box = lv_obj_create(parent);
	lv_obj_set_size(box, 560, 420);
	lv_obj_center(box);
	lv_obj_set_style_bg_opa(box, LV_OPA_TRANSP, 0);
	lv_obj_set_style_border_width(box, 0, 0);
	lv_obj_clear_flag(box, LV_OBJ_FLAG_SCROLLABLE);

	ring = lv_obj_create(box);
	lv_obj_set_size(ring, 300, 300);
	lv_obj_center(ring);
	lv_obj_set_style_radius(ring, LV_RADIUS_CIRCLE, 0);
	lv_obj_set_style_bg_color(ring, lv_color_hex(UI_COLOR_CYAN_SOFT), 0);
	lv_obj_set_style_bg_opa(ring, LV_OPA_COVER, 0);
	lv_obj_set_style_border_width(ring, 4, 0);
	lv_obj_set_style_border_color(ring, lv_color_hex(UI_COLOR_CYAN), 0);
	ui_apply_raised_panel(ring);
	lv_obj_clear_flag(ring, LV_OBJ_FLAG_SCROLLABLE);

	mode_needle = lv_line_create(box);
	lv_obj_set_style_line_width(mode_needle, 5, 0);
	lv_obj_set_style_line_color(mode_needle, lv_color_hex(UI_COLOR_ORANGE), 0);
	lv_obj_set_style_line_rounded(mode_needle, true, 0);

	mode_dial_button_create(box, 0, 0, -158);
	mode_dial_button_create(box, 1, 130, -102);
	mode_dial_button_create(box, 2, 172, 0);
	mode_dial_button_create(box, 3, 108, 130);
	mode_dial_button_create(box, 4, -108, 130);
	mode_dial_button_create(box, 5, -172, 0);
	mode_dial_button_create(box, 6, -130, -102);

	center = lv_obj_create(box);
	lv_obj_set_size(center, 170, 74);
	lv_obj_center(center);
	lv_obj_set_style_radius(center, 16, 0);
	lv_obj_set_style_bg_color(center, lv_color_hex(UI_COLOR_BLUE_DARK), 0);
	lv_obj_set_style_bg_opa(center, LV_OPA_COVER, 0);
	lv_obj_set_style_border_width(center, 2, 0);
	lv_obj_set_style_border_color(center, lv_color_hex(UI_COLOR_CYAN), 0);
	ui_apply_raised_panel(center);
	lv_obj_clear_flag(center, LV_OBJ_FLAG_SCROLLABLE);

	dl_mode = lv_label_create(center);
	lv_label_set_text(dl_mode, "\xE6\x9C\xAA\xE4\xB8\x8A\xE7\x94\xB5");
	lv_obj_set_style_text_color(dl_mode, lv_color_hex(UI_COLOR_BG), 0);
	lv_obj_set_style_text_font(dl_mode, &ui_font_CN14, 0);
	lv_obj_center(dl_mode);

	mode_dial_refresh();
}

static void detail_back_cb(lv_event_t *e)
{
	if (lv_event_get_code(e) == LV_EVENT_CLICKED || lv_event_get_code(e) == LV_EVENT_LONG_PRESSED || lv_event_get_code(e) == LV_EVENT_GESTURE)
		_ui_screen_load(ui_DataDisplay, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 300, 200);
}


void ui_DataDetail_screen_init(void)
{
	if (ui_DataDetail) return;

	ui_DataDetail = lv_obj_create(NULL);
	ui_apply_gradient_background(ui_DataDetail, 0xF2FAFF, 0xD8F1F2, LV_GRAD_DIR_VER);

	g_tv = lv_tabview_create(ui_DataDetail, LV_DIR_TOP, 30);
	lv_obj_set_size(g_tv, 930, 560);
	lv_obj_align(g_tv, LV_ALIGN_TOP_LEFT, 5, 5);
	ui_apply_raised_panel(g_tv);
	lv_obj_set_style_text_font(lv_tabview_get_tab_btns(g_tv), &ui_font_CN14, 0);

	/* ---- Tab 0: GPS ---- */
	lv_obj_t *t0 = lv_tabview_add_tab(g_tv, "GPS");
	ui_apply_gradient_background(t0, 0xF3FFFA, 0xDCEFFF, LV_GRAD_DIR_VER);
	detail_update_data();
	gps_route_create(t0);

	/* ---- Tab 1: Speed ---- */
	lv_obj_t *t1 = lv_tabview_add_tab(g_tv, "\xE8\xBD\xA6\xE8\xBD\xAE");
	ui_apply_gradient_background(t1, 0xFFF9ED, 0xE1F2F3, LV_GRAD_DIR_VER);
	wheel_detail_create(t1);

	/* ---- Tab 2: IMU ---- */
	lv_obj_t *t2 = lv_tabview_add_tab(g_tv, "IMU");
	ui_apply_gradient_background(t2, 0xF1FAFF, 0xD8EEF0, LV_GRAD_DIR_VER);
	lv_obj_clear_flag(t2, LV_OBJ_FLAG_SCROLLABLE);
	compass_create(t2);
	attitude_create(t2);

	dl_imu = lv_label_create(t2);
	lv_label_set_text(dl_imu, "QMC5883\n\xE4\xBF\xAF\xE4\xBB\xB0 --\n\xE6\xA8\xAA\xE6\xBB\x9A --");
	lv_obj_set_style_text_color(dl_imu, lv_color_hex(UI_COLOR_CYAN_DARK), 0);
	lv_obj_set_style_text_font(dl_imu, &ui_font_CN14, 0);
	lv_obj_align(dl_imu, LV_ALIGN_BOTTOM_LEFT, 148, -4);
	compass_refresh();

	/* ---- Tab 3: Mode switcher ---- */
	lv_obj_t *t3 = lv_tabview_add_tab(g_tv, "\xE6\xA8\xA1\xE5\xBC\x8F");
	ui_apply_gradient_background(t3, 0xF7FAFC, 0xDFF4F5, LV_GRAD_DIR_VER);
	mode_dial_create(t3);

	/* Back button --- right edge, vertically centered */
	lv_obj_add_event_cb(ui_DataDetail, touch_reset_idle, LV_EVENT_PRESSED, NULL);
	lv_obj_t *back = lv_btn_create(ui_DataDetail);
	lv_obj_set_size(back, 70, 36);
	lv_obj_align(back, LV_ALIGN_RIGHT_MID, -5, 0);
	ui_apply_raised_button(back);
	lv_obj_t *bl = lv_label_create(back);
	lv_label_set_text(bl, "\xE8\xBF\x94\xE5\x9B\x9E");
	lv_obj_set_style_text_font(bl, &ui_font_CN14, 0);
	lv_obj_center(bl);
	lv_obj_add_event_cb(back, detail_back_cb, LV_EVENT_CLICKED, NULL);

	if (detail_refresh_timer == NULL) {
		detail_refresh_timer = lv_timer_create(detail_refresh_timer_cb, 200, NULL);
	}
}


void dd_open_detail(int tab)
{
	ui_DataDetail_screen_init();
	if (g_tv) lv_tabview_set_act(g_tv, tab, LV_ANIM_ON);
	_ui_screen_load(ui_DataDetail, LV_SCR_LOAD_ANIM_MOVE_LEFT, 300, 200);

	detail_update_data();
	gps_route_refresh();
	if (dl_speed) {
		speed_detail_refresh();
	}
	compass_refresh();
	attitude_refresh();
	mode_dial_refresh();
}


void ui_DataDetail_screen_destroy(void)
{
	uint8_t i;

	if (detail_refresh_timer) {
		lv_timer_del(detail_refresh_timer);
		detail_refresh_timer = NULL;
	}

	if (ui_DataDetail) { lv_obj_del(ui_DataDetail); ui_DataDetail = NULL; }
	g_tv = NULL;
	dl_speed = dl_imu = NULL;
	dl_mode = NULL;
	mode_dial_buttons[0] = NULL;
	mode_dial_buttons[1] = NULL;
	mode_dial_buttons[2] = NULL;
	mode_dial_buttons[3] = NULL;
	mode_dial_buttons[4] = NULL;
	mode_dial_buttons[5] = NULL;
	mode_dial_buttons[6] = NULL;
	mode_needle = NULL;
	compass_heading = NULL;
	compass_marker = NULL;
	compass_cardinal[0] = NULL;
	compass_cardinal[1] = NULL;
	compass_cardinal[2] = NULL;
	compass_cardinal[3] = NULL;
	gps_route_box = NULL;
	gps_status = NULL;
	gps_active_line = NULL;
	gps_vehicle_marker = NULL;
	gps_selected_wp_index = -1;
	gps_arrow_lines[0] = NULL;
	gps_arrow_lines[1] = NULL;
	wheel_motion = NULL;
	attitude_horizon = NULL;
	attitude_yaw_pointer = NULL;
	attitude_yaw_marker = NULL;
	attitude_wing = NULL;
	attitude_data = NULL;
	attitude_state = NULL;
	for (i = 0U; i < MAX_WAYPOINTS; i++) {
		gps_markers[i] = NULL;
		gps_marker_labels[i] = NULL;
		if (i < MAX_WAYPOINTS - 1U) gps_route_lines[i] = NULL;
	}
	for (i = 0U; i < 4U; i++) {
		wheel_cards[i] = NULL;
		wheel_values[i] = NULL;
	}
}
