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
/* ⚠️ 40 -> 60：这 1.5 倍是【麦轮原地转的打滑补偿】，不是随手调的。推导如下：
     1) 原来的 40 本身是几何正确的：轮速计数与角速度的关系应为
        满量程 120 counts × (前轴到四轮中心 0.1716 + 半轮距 0.1402) = 37.4，
        固件取 40，差 7%。所以"指令 ω -> setpoint"这一步本来没有错。
     2) 但真机实测（tools/twist_sign_probe.py）：指令 0.45 rad/s -> setpoint 18
        -> 实际只转 0.13 rad/s，即指令的 29%。拆成两个相乘的损失：
          * 轮速环只能到设定值的 44%  <- 已由 MOTOR_SPEED_PID_MAX_IOUT 的修改修好
          * 麦轮原地转打滑，实际转角约为理想值的 0.66
     3) 轮速环修好后只剩打滑：只能到指令的 66%。要让车真的转到指令值，
        setpoint 必须放大 1/0.66 = 1.52 倍 -> 40 × 1.52 ≈ 61，取 60。
     独立佐证：蓝牙遥控的转向控制量是 BT_REMOTE_WZ = 60，而实测"蓝牙固定控制量下
     电机力足够"，两条路径算出来的 60 一致。

   ⚠️ 这仍是【估算】：打滑系数随地面与载重变化。请用 twist_sign_probe 看 ss_ratio，
      再用 chassis_set_remote_param(CHASSIS_REMOTE_PARAM_ROS_VZ_SCALE, x) 实时标定，
      目标是 ss_ratio ≈ 1.0（= 车真的转到指令角速度）。 */
#define ROS_LINE_VZ_SCALE     60.0f
#define ROS_LINE_MAX_SPEED    120.0f
/* ⚠️ 必须与 VZ_SCALE 同步抬高，否则 setpoint 会先在这里被夹掉：
   setpoint = clamp(ω * VZ_SCALE, ±MAX_WZ)。40/60 = 0.67 rad/s 的指令上限。 */
#define ROS_LINE_MAX_WZ       40.0f

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
