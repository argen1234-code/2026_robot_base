#include "app_Navigation.h"
#include "bsp_GPS.h"
#include "app_Chassis_board.h"
#include <math.h>

#define PI 3.14159265358979323846f

/* ---- 距离PID参数 ---- */
static float Kp_dist  = 8.0f;     /* 米距 → 速度 单位转化比例 */
static float Max_speed = 100.0f;   /* [调试] 最大平移速度限制 (原100) */
static float Min_speed = 60.0f;   /* [调试] 克服底盘低速死区的最小平移速度 (原60) */
//static float Max_speed = 0.0f;   /* [调试] 最大平移速度限制 (原100) */
//static float Min_speed = 0.0f;   /* [调试] 克服底盘低速死区的最小平移速度 (原60) */

/* ---- 航向偏角PID参数 ---- */
static float Kp_yaw  = 0.5f;      /* 角度误差 → 角速度 比例 */
static float Max_wz  = 18.0f;     /* 最大旋转角速度限制 */
//static float Max_wz  = 0.0f;     /* 最大旋转角速度限制 */
/* Keil Watch-only navigation mirror; no control code reads it back. */

void Navigation_Set_Speed_Percent(uint8_t speed_percent)
{
    if (speed_percent < 20U) speed_percent = 20U;
    if (speed_percent > 100U) speed_percent = 100U;

    /* 纯GPS底盘低于Min_speed无法可靠起步，将20~100%映射到可用区间。 */
    Max_speed = Min_speed + (100.0f - Min_speed) * ((float)speed_percent / 100.0f);
}

static void Navigation_DebugSync(struct chassis_move_s *chassis, uint8_t navigation_mode)
{
    Navigation_State_t *nav;
    volatile GPS_NavigationDebug_t *debug;

    if (chassis == NULL) return;

    nav = &chassis->nav;
    debug = &g_gps_debug.navigation;

    debug->update_sequence++;
    debug->last_update_tick = HAL_GetTick();
    debug->dwell_start_tick = nav->dwell_start_tick;
    debug->is_navigating = nav->is_navigating;
    debug->loop_enable = nav->loop_enable;
    debug->phase = (uint8_t)nav->phase;
    debug->rtk_quality = nav->rtk_quality;
    debug->current_waypoint_index = nav->current_wp_index;
    debug->total_waypoints = nav->total_waypoints;
    debug->navigation_mode = navigation_mode;
    debug->current_latitude_deg = nav->current_pos.lat;
    debug->current_longitude_deg = nav->current_pos.lon;
    debug->target_latitude_deg = nav->target_pos.lat;
    debug->target_longitude_deg = nav->target_pos.lon;
    debug->current_heading_deg = nav->current_heading;
    debug->target_bearing_deg = nav->target_bearing;
    debug->distance_error_m = nav->distance_error;
    debug->heading_error_deg = nav->heading_error;
    debug->command_vx = chassis->Vx_set;
    debug->command_vy = chassis->Vy_set;
    debug->command_wz = chassis->Wz_set;
    debug->update_sequence++;
}

/* ============================================================
 *  内部: 加载航点路线
 * ============================================================ */
static void Navigation_Load_Route(Navigation_State_t *nav,
                                  GPS_Point_t *waypoints,
                                  uint8_t count, uint8_t loop_enable)
{
    if (count > MAX_WAYPOINTS) count = MAX_WAYPOINTS;
    if (count == 0) return;

    for (uint8_t i = 0; i < count; i++) {
        nav->route[i] = waypoints[i];
    }

    nav->total_waypoints  = count;
    nav->current_wp_index = 0;
    nav->loop_enable      = loop_enable;
    nav->target_pos       = nav->route[0];
    nav->phase            = NAV_PHASE_RUNNING;
    nav->is_navigating    = 1;
}

/* ============================================================
 *  工具: NMEA(ddmm.mmmm) → 十进制经纬度
 * ============================================================ */
static double NMEA_To_Degree(double nmea_cord)
{
    double degrees = floor(nmea_cord / 100.0);
    double minutes = nmea_cord - (degrees * 100.0);
    return degrees + (minutes / 60.0);
}

/* ============================================================
 *  工具: 两点经纬度 → 绝对方位角 (0=正北, 90=正东)
 * ============================================================ */
static float Calculate_Bearing(double lat1, double lon1,
                               double lat2, double lon2)
{
    double dLon   = (lon2 - lon1) * PI / 180.0;
    double lat1Rad = lat1 * PI / 180.0;
    double lat2Rad = lat2 * PI / 180.0;

    double y = sin(dLon) * cos(lat2Rad);
    double x = cos(lat1Rad) * sin(lat2Rad) -
               sin(lat1Rad) * cos(lat2Rad) * cos(dLon);

    double brng = atan2(y, x) * 180.0 / PI;
    if (brng < 0) brng += 360.0;
    return (float)brng;
}

/* ============================================================
 *  工具: 两点经纬度 → 球面距离(米)
 * ============================================================ */
static float Calculate_Distance(double lat1, double lon1,
                                double lat2, double lon2)
{
    double dLat = (lat2 - lat1) * 111320.0;
    double dLon = (lon2 - lon1) * 111320.0 * cos(lat1 * PI / 180.0);
    return (float)sqrt(dLat * dLat + dLon * dLon);
}

/* ============================================================
 *  公开: 单目标导航 (向后兼容)
 * ============================================================ */
void Navigation_Set_Target(Navigation_State_t *nav,
                           double target_lat, double target_lon)
{
    if (nav == NULL) return;

    nav->route[0].lat     = target_lat;
    nav->route[0].lon     = target_lon;
    nav->total_waypoints  = 1;
    nav->current_wp_index = 0;
    nav->loop_enable      = 0;
    nav->target_pos.lat   = target_lat;
    nav->target_pos.lon   = target_lon;
    nav->phase            = NAV_PHASE_RUNNING;
    nav->is_navigating    = 1;
}

/* ============================================================
 *  公开: 多目标巡航
 * ============================================================ */
void Navigation_Set_Route(Navigation_State_t *nav,
                          GPS_Point_t *waypoints, uint8_t count)
{
    if (nav != NULL) {
        Navigation_Load_Route(nav, waypoints, count, 0);
    }
}

/* ============================================================
 *  公开: 循环巡航
 * ============================================================ */
void Navigation_Set_Route_Loop(Navigation_State_t *nav,
                               GPS_Point_t *waypoints, uint8_t count)
{
    if (nav != NULL) {
        Navigation_Load_Route(nav, waypoints, count, 1);
    }
}

/* ============================================================
 *  公开: 停止导航
 * ============================================================ */
void Navigation_Stop(struct chassis_move_s *chassis)
{
    if (chassis == NULL) return;

    chassis->nav.is_navigating = 0;
    chassis->nav.loop_enable   = 0;
    chassis->nav.phase         = NAV_PHASE_IDLE;
    chassis->Vx_set = 0.0f;
    chassis->Vy_set = 0.0f;
    chassis->Wz_set = 0.0f;
    Navigation_DebugSync(chassis, GPS_DEBUG_NAV_MODE_NONE);
}

/* ============================================================
 *  公开: 导航核心更新循环 (每周期调用一次)
 *        从 chassis->imu.mag 读取磁力计航向
 *        向 chassis->Vx/Vy/Wz_set 写入全向移动目标速度
 * ============================================================ */
void Navigation_Update_Loop(struct chassis_move_s *chassis)
{
    if (chassis == NULL) return;

    Navigation_State_t *nav = &chassis->nav;

    if (!nav->is_navigating) {
        Navigation_DebugSync(chassis, GPS_DEBUG_NAV_MODE_PURE);
        return;
    }

    /* ---- 停留等待阶段 ---- */
    if (nav->phase == NAV_PHASE_DWELLING) {
        chassis->Vx_set = 0.0f;
        chassis->Vy_set = 0.0f;
        chassis->Wz_set = 0.0f;

        if (HAL_GetTick() - nav->dwell_start_tick >= WAYPOINT_DWELL_MS) {
            if (nav->current_wp_index + 1 >= nav->total_waypoints) {
                nav->current_wp_index = 0;
            } else {
                nav->current_wp_index++;
            }
            nav->target_pos = nav->route[nav->current_wp_index];
            nav->phase      = NAV_PHASE_RUNNING;
        }
        Navigation_DebugSync(chassis, GPS_DEBUG_NAV_MODE_PURE);
        return;
    }

    /* ---- 行驶阶段 ---- */

    /* 1. 读取 GPS 定位 */
    PT_GNGGA pGGA = GetGNGGA();
    PT_GPHPR pHPR = GetGPHPR();

    if (pGGA->qf < 1 || pGGA->lat < 1.0f ||
        pGGA->last_update_tick == 0U ||
        (HAL_GetTick() - pGGA->last_update_tick) > 3000U) {
        /* 定位无效 → 强制停车 */
        Navigation_Stop(chassis);
        return;
    }

    /* 磁力计是纯GPS航向唯一可信来源；掉线或数据过期时禁止驱动。 */
    if (chassis->imu.mag_last_update_tick == 0U ||
        (HAL_GetTick() - chassis->imu.mag_last_update_tick) > 1000U)
    {
        Navigation_Stop(chassis);
        return;
    }

    nav->rtk_quality  = pGGA->qf;
    nav->current_pos.lat = NMEA_To_Degree(pGGA->lat);
    nav->current_pos.lon = NMEA_To_Degree(pGGA->lon);

    /* 2. 读取磁力计航向 (数据已由 chassis_feedback_update 刷新) */
    nav->current_heading = chassis->imu.mag.yaw;

    double target_decimal_lat = nav->target_pos.lat;
    double target_decimal_lon = nav->target_pos.lon;

    /* 3. 计算绝对方位角与距离 */
    nav->target_bearing = Calculate_Bearing(
        nav->current_pos.lat, nav->current_pos.lon,
        target_decimal_lat,   target_decimal_lon);
    nav->distance_error = Calculate_Distance(
        nav->current_pos.lat, nav->current_pos.lon,
        target_decimal_lat,   target_decimal_lon);

    /* 4. 到达判断 (2m 精度) */
    if (nav->distance_error < 1.0f) {
        chassis->Vx_set = 0.0f;
        chassis->Vy_set = 0.0f;
        chassis->Wz_set = 0.0f;

        if (nav->current_wp_index + 1 >= nav->total_waypoints) {
            if (nav->loop_enable && nav->total_waypoints > 1) {
                nav->dwell_start_tick = HAL_GetTick();
                nav->phase = NAV_PHASE_DWELLING;
            } else {
                Navigation_Stop(chassis);
            }
        } else {
            nav->dwell_start_tick = HAL_GetTick();
            nav->phase = NAV_PHASE_DWELLING;
        }
        Navigation_DebugSync(chassis, GPS_DEBUG_NAV_MODE_PURE);
        return;
    }

    /* 5. 航向偏差 (归一化到 -180 ~ +180) */
    float angle_diff = nav->target_bearing - nav->current_heading;
    while (angle_diff >  180.0f) angle_diff -= 360.0f;
    while (angle_diff <= -180.0f) angle_diff += 360.0f;
    nav->heading_error = angle_diff;

    /* 6. 纯 GPS 蟹行 (Crab Walk) */

    /* a. 距离 → 平移速度 (P控) */
    float target_v = nav->distance_error * Kp_dist;
    if (target_v > Max_speed) target_v = Max_speed;
    if (target_v < Min_speed) target_v = Min_speed;

    float rad_err = angle_diff * PI / 180.0f;
    float out_vx = target_v * cosf(rad_err);
    float out_vy = target_v * sinf(rad_err);

    /* b. 航向 → 旋转速度 (P控) */
    float out_wz = angle_diff * Kp_yaw;
    if (out_wz >  Max_wz) out_wz =  Max_wz;
    if (out_wz < -Max_wz) out_wz = -Max_wz;

    /* 7. 输出到底盘结构体 */
    chassis->Vx_set = out_vx;
    chassis->Vy_set = out_vy;
    chassis->Wz_set = out_wz;
    Navigation_DebugSync(chassis, GPS_DEBUG_NAV_MODE_PURE);
}

/* ============================================================
 *  公开: GPS + ROS cmd_vel 融合导航 (每周期调用一次)
 *        ROS cmd_vel 来自 Jetson 融合了 heading_to_target_deg
 *        的路径规划输出 (虚拟目标点 3m 前推 + 避障),
 *        本函数保留 GPS 航点管理, 以 ROS 指令为主控,
 *        ROS 无指令时自动回退为纯 GPS 蟹行
 * ============================================================ */
void Navigation_Update_Loop_Fusion(struct chassis_move_s *chassis)
{
    if (chassis == NULL) return;

    Navigation_State_t *nav = &chassis->nav;

    if (!nav->is_navigating) {
        Navigation_DebugSync(chassis, GPS_DEBUG_NAV_MODE_FUSION);
        return;
    }

    /* ---- 停留等待阶段 (同纯GPS) ---- */
    if (nav->phase == NAV_PHASE_DWELLING) {
        chassis->Vx_set = 0.0f;
        chassis->Vy_set = 0.0f;
        chassis->Wz_set = 0.0f;

        if (HAL_GetTick() - nav->dwell_start_tick >= WAYPOINT_DWELL_MS) {
            if (nav->current_wp_index + 1 >= nav->total_waypoints) {
                nav->current_wp_index = 0;
            } else {
                nav->current_wp_index++;
            }
            nav->target_pos = nav->route[nav->current_wp_index];
            nav->phase      = NAV_PHASE_RUNNING;
        }
        Navigation_DebugSync(chassis, GPS_DEBUG_NAV_MODE_FUSION);
        return;
    }

    /* ---- 行驶阶段 ---- */

    /* 1. 读取 GPS 定位 */
    PT_GNGGA pGGA = GetGNGGA();
    PT_GPHPR pHPR = GetGPHPR();

		nav->gps_utc_time = pGGA->utc_time;//为了后续检查GPS模式下数据丢失是什么原因搞得
		
		
    if (pGGA->qf < 1 || pGGA->lat < 1.0f) {
        chassis->Vx_set = 0.0f;
        chassis->Vy_set = 0.0f;
        chassis->Wz_set = 0.0f;
        Navigation_DebugSync(chassis, GPS_DEBUG_NAV_MODE_FUSION);
        return;
    }

    nav->rtk_quality  = pGGA->qf;
    nav->current_pos.lat = NMEA_To_Degree(pGGA->lat);
    nav->current_pos.lon = NMEA_To_Degree(pGGA->lon);

    nav->current_heading = chassis->imu.mag.yaw;

    /* 2. 计算目标方位角与距离 (用于航点管理与遥测) */
    nav->target_bearing = Calculate_Bearing(
        nav->current_pos.lat, nav->current_pos.lon,
        nav->target_pos.lat,   nav->target_pos.lon);
    nav->distance_error = Calculate_Distance(
        nav->current_pos.lat, nav->current_pos.lon,
        nav->target_pos.lat,   nav->target_pos.lon);

    /* 航向偏差 (归一化到 -180 ~ +180) */
    float angle_diff = nav->target_bearing - nav->current_heading;
    while (angle_diff >  180.0f) angle_diff -= 360.0f;
    while (angle_diff <= -180.0f) angle_diff += 360.0f;
    nav->heading_error = angle_diff;

    /* 3. 到达判断 */
    if (nav->distance_error < 2.0f) {
        chassis->Vx_set = 0.0f;
        chassis->Vy_set = 0.0f;
        chassis->Wz_set = 0.0f;

        if (nav->current_wp_index + 1 >= nav->total_waypoints) {
            if (nav->loop_enable && nav->total_waypoints > 1) {
                nav->dwell_start_tick = HAL_GetTick();
                nav->phase = NAV_PHASE_DWELLING;
            } else {
                Navigation_Stop(chassis);
            }
        } else {
            nav->dwell_start_tick = HAL_GetTick();
            nav->phase = NAV_PHASE_DWELLING;
        }
        Navigation_DebugSync(chassis, GPS_DEBUG_NAV_MODE_FUSION);
        return;
    }

    /* 4. 融合: ROS cmd_vel 为主控, GPS 蟹行为回退 */

    float ros_vx = chassis->cmd_vel.vx;
    float ros_vz = chassis->cmd_vel.vz;
    float out_vx, out_wz;
    float vx_limit = Max_speed;
    float wz_limit = Max_wz;

    if (fabsf(ros_vx) > 0.01f)
    {
        /* 与室内ROS导航复用同一套可调缩放及限幅参数。 */
        out_vx = ros_vx * chassis->remote.ros_vx_scale;
        vx_limit = fabsf(chassis->remote.ros_max_speed);
    }
    else
    {
        /* ROS 无指令 → GPS 距离 P 控 */
        out_vx = nav->distance_error * Kp_dist;
    }
    if (out_vx >  vx_limit) out_vx =  vx_limit;
    if (out_vx < 0.0f)     out_vx = 0.0f;

    /* 横向: 融合模式不使用蟹行, 仅向前 */
    float out_vy = 0.0f;

    if (fabsf(ros_vz) > 0.01f)
    {
        /* 与室内ROS导航复用同一套可调缩放及限幅参数。 */
        out_wz = ros_vz * chassis->remote.ros_vz_scale;
        wz_limit = fabsf(chassis->remote.ros_max_wz);
    }
    else
    {
        /*
         * ROS 无旋转指令时使用磁力计/GPS回退。
         * angle_diff沿用罗盘定义：右侧为正(CW+)；底盘Wz/ROS angular.z
         * 则是左转为正(CCW+)，因此这里必须取反。
         */
        out_wz = -angle_diff * Kp_yaw;
    }
    if (out_wz >  wz_limit) out_wz =  wz_limit;
    if (out_wz < -wz_limit) out_wz = -wz_limit;

    /* 5. 输出到底盘结构体 */
    chassis->Vx_set = out_vx;
    chassis->Vy_set = out_vy;
    chassis->Wz_set = out_wz;
    Navigation_DebugSync(chassis, GPS_DEBUG_NAV_MODE_FUSION);
}
