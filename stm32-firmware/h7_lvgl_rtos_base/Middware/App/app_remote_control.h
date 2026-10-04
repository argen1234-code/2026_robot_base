#ifndef APP_REMOTE_CONTROL_H
#define APP_REMOTE_CONTROL_H

#include "main.h"

/* 前向声明: chassis_move_t 完整定义在 app_chassis_board.h */
typedef struct chassis_move_s chassis_move_t;

/* ---- 蓝牙遥控默认参数 ---- */
#define BT_REMOTE_SPEED       60.0f
#define BT_REMOTE_WZ          60.0f

/* ---- 微信遥控默认参数 ---- */
#define WECHAT_VX_SCALE       100.0f
#define WECHAT_VZ_SCALE       30.0f
#define WECHAT_SPEED_GAIN     1.5f
#define WECHAT_MAX_SPEED      75.0f
#define WECHAT_MAX_WZ         37.5f

/* ---- ROS 室内导航默认参数 ---- */
#define ROS_LINE_VX_SCALE     120.0f
#define ROS_LINE_VZ_SCALE     40.0f
#define ROS_LINE_MAX_SPEED    120.0f
#define ROS_LINE_MAX_WZ       25.0f

/* ============================================================
 *  遥控控制量参数结构体 (运行时可由 GUI / 调试器调整)
 * ============================================================ */
typedef struct {
    float bt_speed;           /* 蓝牙遥控线速度 */
    float bt_wz;              /* 蓝牙遥控角速度 */
    float wechat_vx_scale;    /* 微信 vx 缩放 */
    float wechat_vz_scale;    /* 微信 vz 缩放 */
    float wechat_speed_gain;  /* 微信总增益 */
    float wechat_max_speed;   /* 微信最大线速度 */
    float wechat_max_wz;      /* 微信最大角速度 */
    float ros_vx_scale;       /* ROS vx 缩放 */
    float ros_vz_scale;       /* ROS vz 缩放 */
    float ros_max_speed;      /* ROS 最大线速度 */
    float ros_max_wz;         /* ROS 最大角速度 */
} RemoteControl_t;

void Remote_Control_Update(chassis_move_t *chassis);
void Remote_WeChat_Update(chassis_move_t *chassis);
void Remote_ROS_Update(chassis_move_t *chassis);

#endif
