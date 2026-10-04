#include "app_remote_control.h"
#include "app_chassis_board.h"
#include "bsp_bluetooth.h"

/* ============================================================
 *  蓝牙遥控 (最高优先级)
 *  前后 → Vx_set, 左/右/旋转 → Wz_set (差速转向, 不横移)
 * ============================================================ */
void Remote_Control_Update(chassis_move_t *chassis)
{
    uint8_t keys = BT_GetKeyState();
    float   speed = chassis->remote.bt_speed;
    float   wz    = chassis->remote.bt_wz;

    chassis->Vx_set = 0.0f;
    chassis->Vy_set = 0.0f;
    chassis->Wz_set = 0.0f;

    if (keys & BT_KEY_FORWARD)      chassis->Vx_set -= speed;
    if (keys & BT_KEY_BACKWARD)     chassis->Vx_set += speed;
    if (keys & BT_KEY_LEFT)         chassis->Wz_set += wz;      /* 左转 → 差速 */
    if (keys & BT_KEY_RIGHT)        chassis->Wz_set -= wz;      /* 右转 → 差速 */
    if (keys & BT_KEY_ROTATE_LEFT)  chassis->Wz_set += wz;
    if (keys & BT_KEY_ROTATE_RIGHT) chassis->Wz_set -= wz;
}

/* ============================================================
 *  微信小程序遥控 (第二优先级)
 *  Jetson 转发来的 cmd_vel, mode=2
 *  vx → 前向速度, vz → 旋转速度
 * ============================================================ */
void Remote_WeChat_Update(chassis_move_t *chassis)
{
    float vx_scale = chassis->remote.wechat_vx_scale;
    float vz_scale = chassis->remote.wechat_vz_scale;
    float gain     = chassis->remote.wechat_speed_gain;
    float max_vx   = chassis->remote.wechat_max_speed;
    float max_wz   = chassis->remote.wechat_max_wz;

    float out_vx = chassis->cmd_vel.vx * vx_scale * gain;
    float out_wz = chassis->cmd_vel.vz * vz_scale * gain;

    if (out_vx >  max_vx) out_vx =  max_vx;
    if (out_vx < -max_vx) out_vx = -max_vx;
    if (out_wz >  max_wz) out_wz =  max_wz;
    if (out_wz < -max_wz) out_wz = -max_wz;

    chassis->Vx_set = out_vx;
    chassis->Vy_set = 0.0f;
    chassis->Wz_set = out_wz;
}

/* ============================================================
 *  室内 ROS 自主导航 (Jetson mode=3)
 *  纯 ROS cmd_vel 控制, 无 GPS 参与
 *  vx → 前向速度, vz → 旋转速度
 * ============================================================ */
void Remote_ROS_Update(chassis_move_t *chassis)
{
    float vx_scale = chassis->remote.ros_vx_scale;
    float vz_scale = chassis->remote.ros_vz_scale;
    float max_vx   = chassis->remote.ros_max_speed;
    float max_wz   = chassis->remote.ros_max_wz;

    float out_vx = chassis->cmd_vel.vx * vx_scale;
    float out_wz = chassis->cmd_vel.vz * vz_scale;

    /*
     * ROS +X forward is converted by the Jetson serial bridge to the
     * chassis convention, where forward Vx is negative.
     * Indoor Nav2 is forward-only, so reject positive (reverse) commands.
     */
    if (out_vx < -max_vx) out_vx = -max_vx;
    if (out_vx > 0.0f)    out_vx = 0.0f;
    if (out_wz >  max_wz) out_wz =  max_wz;
    if (out_wz < -max_wz) out_wz = -max_wz;

    chassis->Vx_set = out_vx;
    chassis->Vy_set = 0.0f;
    chassis->Wz_set = out_wz;
}
