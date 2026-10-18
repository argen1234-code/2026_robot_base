#ifndef APP_REMOTE_CONTROL_H
#define APP_REMOTE_CONTROL_H

#include <stdint.h>

typedef struct chassis_move_s chassis_move_t;

/* 蓝牙遥控默认参数（米制） */
#define BT_REMOTE_VX_MPS        0.30f
#define BT_REMOTE_VY_MPS        0.20f
#define BT_REMOTE_WZ_RAD_S      0.80f
#define BT_REMOTE_CMD_GAIN      1.0f

/* 请求值上限 */
#define CHASSIS_MAX_VX_MPS      0.60f
#define CHASSIS_MAX_VY_MPS      0.40f
#define CHASSIS_MAX_WZ_RAD_S    1.50f

typedef struct
{
    float bt_vx_mps;
    float bt_vy_mps;
    float bt_wz_rad_s;
    float bt_gain;
    float max_vx_mps;
    float max_vy_mps;
    float max_wz_rad_s;
} RemoteControl_t;

void Remote_Control_Update(chassis_move_t *chassis);

#endif
