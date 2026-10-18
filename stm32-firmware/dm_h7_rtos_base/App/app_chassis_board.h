#ifndef APP_CHASSIS_BOARD_H
#define APP_CHASSIS_BOARD_H

#include <stdint.h>
#include "pid.h"
#include "app_remote_control.h"

/* 任务开关，沿用旧工程写法 */
#define chassis_board_task          1

/* ---------------- 运动学常量（先占位，装车后标定） ---------------- */
#define CHASSIS_WHEEL_RADIUS_M      0.034f
#define CHASSIS_HALF_WHEELBASE_M    0.150f
#define CHASSIS_HALF_TRACK_M        0.130f
#define CHASSIS_KIN_VY_SIGN         (+1.0f)
#define CHASSIS_KIN_WZ_SIGN         (+1.0f)
#define CHASSIS_MAX_WHEEL_RAD_S     30.0f

/* ---------------- 电机安装方向（装反单独改） ---------------- */
#define MOTOR_DIR_FL                (+1.0f)
#define MOTOR_DIR_FR                (+1.0f)
#define MOTOR_DIR_RL                (+1.0f)
#define MOTOR_DIR_RR                (+1.0f)

/* ---------------- 5 ms 轮速 PID 起步参数 ---------------- */
#define MOTOR_SPEED_PID_KP          0.35
#define MOTOR_SPEED_PID_KI          0.01
#define MOTOR_SPEED_PID_KD          0.0
#define MOTOR_SPEED_PID_MAX_OUT     2.0     /* N·m */
#define MOTOR_SPEED_PID_MAX_IOUT    200.0   /* = MAX_OUT / KI，夹的是原始积分量 */

/* 可选静摩擦前馈/死区，先关掉，落地后按需开启 */
#define CHASSIS_FRICTION_FF_NM      0.0f
#define CHASSIS_FRICTION_FF_MIN_RAD_S  0.5f
#define CHASSIS_SPEED_DEADZONE_RAD_S   0.0f

/* ---------------- 控制量斜坡 ---------------- */
#define CHASSIS_TRANSLATION_SLEW_MPS2   2.0f
#define CHASSIS_ROTATION_SLEW_RAD_S2    6.0f

/* ---------------- 故障位 ---------------- */
#define CHASSIS_FAULT_MOTOR_TIMEOUT     (1U << 0)
#define CHASSIS_FAULT_CAN_BUS_OFF       (1U << 1)
#define CHASSIS_FAULT_CAN_TX            (1U << 2)
#define CHASSIS_FAULT_BT_TIMEOUT        (1U << 3)
#define CHASSIS_FAULT_EMERGENCY         (1U << 4)
#define CHASSIS_FAULT_MOTORS_NOT_READY  (1U << 5)
#define CHASSIS_FAULT_ENABLE_FAILED     (1U << 6)
#define CHASSIS_FAULT_MOTOR_STATE       (1U << 7)

typedef enum
{
    CAR_MODE_IDLE = 0,
    CAR_MODE_INDOOR,
    CAR_MODE_CONFIG,
    CAR_MODE_FAULT
} CarMode_t;

typedef struct
{
    /* 单位：rad/s、N·m；speed 已做电机安装方向校正 */
    double speed;
    double speed_set;
    double torque_set;

    float t_mos_c;
    float t_rotor_c;

    uint8_t state;
    uint8_t online;
    uint32_t last_update_tick;

    PID_t speed_pid;
} chassis_motor_t;

typedef struct chassis_move_s
{
    CarMode_t mode;

    uint8_t fault_flags;
    uint8_t bt_online;
    uint8_t motors_ready;
    uint8_t emergency_latched;
    uint8_t bt_keys;

    float Vx_set;
    float Vy_set;
    float Wz_set;
    float Vx_cmd;
    float Vy_cmd;
    float Wz_cmd;

    chassis_motor_t motor[4];
    RemoteControl_t remote;

    uint32_t loop_count;
    uint32_t last_loop_tick;
    uint16_t loop_dt_ms;
} chassis_move_t;

extern chassis_move_t chassis_move;

void chassis_init(chassis_move_t *chassis);
void chassis_stop(chassis_move_t *chassis);

void chassis_feedback_update(chassis_move_t *chassis);
void chassis_mode_change(chassis_move_t *chassis);
void chassis_set_control(chassis_move_t *chassis);
void chassis_control_loop(chassis_move_t *chassis);
void chassis_send_cmd(chassis_move_t *chassis);

void chassis_task(void *argument);
chassis_move_t *chassis_get(void);

#endif
