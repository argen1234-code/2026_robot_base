#include "app_remote_control.h"
#include "app_chassis_board.h"
#include "bsp_bluetooth.h"
#include <stddef.h>

/**
  * @brief  蓝牙按键 → 底盘请求速度
  * @note   f/b = Vx（m/s），l/r = Vy 横移（m/s），q/e = Wz（rad/s）
  *         正方向：x 前、y 左、ω 逆时针
  */
void Remote_Control_Update(chassis_move_t *chassis)
{
    uint8_t keys;
    float vx;
    float vy;
    float wz;

    if (chassis == NULL)
    {
        return;
    }

    keys = BT_GetKeyState();
    vx = chassis->remote.bt_vx_mps * chassis->remote.bt_gain;
    vy = chassis->remote.bt_vy_mps * chassis->remote.bt_gain;
    wz = chassis->remote.bt_wz_rad_s * chassis->remote.bt_gain;

    chassis->Vx_set = 0.0f;
    chassis->Vy_set = 0.0f;
    chassis->Wz_set = 0.0f;

    if ((keys & BT_KEY_FORWARD) != 0U)
    {
        chassis->Vx_set += vx;
    }
    if ((keys & BT_KEY_BACKWARD) != 0U)
    {
        chassis->Vx_set -= vx;
    }
    if ((keys & BT_KEY_LEFT) != 0U)
    {
        chassis->Vy_set += vy;
    }
    if ((keys & BT_KEY_RIGHT) != 0U)
    {
        chassis->Vy_set -= vy;
    }
    if ((keys & BT_KEY_ROTATE_LEFT) != 0U)
    {
        chassis->Wz_set += wz;
    }
    if ((keys & BT_KEY_ROTATE_RIGHT) != 0U)
    {
        chassis->Wz_set -= wz;
    }

    /* 限幅 */
    if (chassis->Vx_set > chassis->remote.max_vx_mps)
    {
        chassis->Vx_set = chassis->remote.max_vx_mps;
    }
    if (chassis->Vx_set < -chassis->remote.max_vx_mps)
    {
        chassis->Vx_set = -chassis->remote.max_vx_mps;
    }
    if (chassis->Vy_set > chassis->remote.max_vy_mps)
    {
        chassis->Vy_set = chassis->remote.max_vy_mps;
    }
    if (chassis->Vy_set < -chassis->remote.max_vy_mps)
    {
        chassis->Vy_set = -chassis->remote.max_vy_mps;
    }
    if (chassis->Wz_set > chassis->remote.max_wz_rad_s)
    {
        chassis->Wz_set = chassis->remote.max_wz_rad_s;
    }
    if (chassis->Wz_set < -chassis->remote.max_wz_rad_s)
    {
        chassis->Wz_set = -chassis->remote.max_wz_rad_s;
    }
}
