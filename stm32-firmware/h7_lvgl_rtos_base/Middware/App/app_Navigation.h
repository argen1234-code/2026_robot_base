#ifndef __APP__NAVIGATION_H
#define __APP__NAVIGATION_H

#include "stm32h7xx_hal.h"

// ==================== 【可调参数，在这里修改】 ====================
// 最多支持的航点数量（可扩容，如改成20）
#define MAX_WAYPOINTS      10

// 每个中间航点的停留等待时间（毫秒）。最后一个航点到达后直接停车，不会再等
#define WAYPOINT_DWELL_MS  5000
// ================================================================

// 坐标结构体
typedef struct {
    double lat;     // Latitude
    double lon;     // Longitude
} GPS_Point_t;

// 导航阶段枚举
typedef enum {
    NAV_PHASE_IDLE,      // 空闲
    NAV_PHASE_RUNNING,   // 正在向当前航点行驶
    NAV_PHASE_DWELLING   // 已到达当前航点，正在原地停留等待
} Navigation_Phase_t;

// 导航控制器状态结构体
typedef struct {
    uint8_t is_navigating;    // 导航是否开启 1:开启 0:关闭
    uint8_t loop_enable;      // 是否循环巡航 1:循环 0:到终点停车
    Navigation_Phase_t phase; // 当前导航阶段
    GPS_Point_t current_pos;  // 当前坐标
    GPS_Point_t target_pos;   // 当前正在前往的目标坐标
    uint8_t rtk_quality;      // 定位质量 qf 值 (4是固定解,1是单点)
    
    float current_heading;    // 当前车头绝对角度 (0-360)
    float target_bearing;     // 目标绝对方位角 (0-360)
    
    float distance_error;     // 距离目标的误差 (米)
    float heading_error;      // 航向误差 (度)
    
    // ---- 多航点巡航相关 ----
    GPS_Point_t route[MAX_WAYPOINTS]; // 航点队列
    uint8_t total_waypoints;          // 本次任务总航点数
    uint8_t current_wp_index;         // 当前正在前往第几个航点 (从0开始)
    uint32_t dwell_start_tick;        // 到达航点时的系统时间戳 (用于计时停留)
	
		double gps_utc_time;
	
	
	
} Navigation_State_t;

/* 前向声明: 完整类型定义在 app_chassis_board.h */
struct chassis_move_s;

/* ---- 航点设置 (仅操作 Navigation_State_t) ---- */
void Navigation_Set_Target(Navigation_State_t *nav, double target_lat, double target_lon);
void Navigation_Set_Route(Navigation_State_t *nav, GPS_Point_t *waypoints, uint8_t count);
void Navigation_Set_Route_Loop(Navigation_State_t *nav, GPS_Point_t *waypoints, uint8_t count);
void Navigation_Set_Speed_Percent(uint8_t speed_percent);

/* ---- 导航控制 (需要底盘指针以读取IMU / 写入VxVyWz) ---- */
void Navigation_Stop(struct chassis_move_s *chassis);
void Navigation_Update_Loop(struct chassis_move_s *chassis);
void Navigation_Update_Loop_Fusion(struct chassis_move_s *chassis);

#endif
