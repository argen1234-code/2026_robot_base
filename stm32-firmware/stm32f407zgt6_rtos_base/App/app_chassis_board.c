/*
 * 4×DM-H6215 麦轮底盘 —— 由 dm_h7_rtos_base 移植到 STM32F407ZGT6。
 *
 * 与 H7 版的差异：
 *   - CAN 从 FDCAN1 换成 bxCAN1（bsp_can，1 Mbps 经典 CAN）
 *   - 控制源从蓝牙改成 USB CDC 的 AA55 速度帧（app_usb_bridge）
 *   - 遥测从 USART10/VOFA+ 换成 USB CC55 v2（app_telemetry）
 *   - 去掉板级电源使能、急停键、@cfg 蓝牙链路（文本配置命令改走 USB）
 */
#include "app_chassis_board.h"
#include "app_motor_config.h"
#include "app_telemetry.h"
#include "app_usb_bridge.h"
#include "bsp_can.h"
#include "bsp_dm6215.h"
#include "main.h"
#include "cmsis_os2.h"

#include <math.h>
#include <string.h>

#define CHASSIS_TASK_PERIOD_MS      5U
#define CHASSIS_FEEDBACK_TIMEOUT_MS 100U
#define CHASSIS_CAN_TX_FAIL_LIMIT   20U

/* 启动文件清零 .bss；chassis_init() 还会再 memset 一次 */
chassis_move_t chassis_move;

static const float s_motor_dir[4] =
{
    MOTOR_DIR_FL,
    MOTOR_DIR_FR,
    MOTOR_DIR_RL,
    MOTOR_DIR_RR
};

static uint32_t s_last_control_tick = 0U;
static uint8_t  s_tx_fail_streak = 0U;
static uint8_t  s_prev_ready = 0U;

/* ------------------------------------------------------------------------- */
/* 工具                                                                       */
/* ------------------------------------------------------------------------- */

static float clampf(float value, float min, float max)
{
    if (value < min)
    {
        return min;
    }
    if (value > max)
    {
        return max;
    }
    return value;
}

static float slew_limit(float current, float target, float max_delta)
{
    float delta = target - current;

    if (delta > max_delta)
    {
        delta = max_delta;
    }
    else if (delta < -max_delta)
    {
        delta = -max_delta;
    }

    return current + delta;
}

static void chassis_reset_control(chassis_move_t *chassis)
{
    uint8_t i;

    chassis->Vx_set = 0.0f;
    chassis->Vy_set = 0.0f;
    chassis->Wz_set = 0.0f;
    chassis->Vx_cmd = 0.0f;
    chassis->Vy_cmd = 0.0f;
    chassis->Wz_cmd = 0.0f;

    for (i = 0U; i < 4U; i++)
    {
        chassis->motor[i].speed_set = 0.0;
        chassis->motor[i].torque_set = 0.0;
        PID_Reset(&chassis->motor[i].speed_pid);
    }
}

/* ------------------------------------------------------------------------- */
/* 初始化                                                                     */
/* ------------------------------------------------------------------------- */

void chassis_init(chassis_move_t *chassis)
{
    const double speed_pid_param[3] =
    {
        MOTOR_SPEED_PID_KP,
        MOTOR_SPEED_PID_KI,
        MOTOR_SPEED_PID_KD
    };
    uint8_t i;

    if (chassis == NULL)
    {
        return;
    }

    memset(chassis, 0, sizeof(*chassis));

    chassis->mode = CAR_MODE_IDLE;

    for (i = 0U; i < 4U; i++)
    {
        PID_init(&chassis->motor[i].speed_pid, PID_POSITION,
                 speed_pid_param,
                 MOTOR_SPEED_PID_MAX_OUT,
                 MOTOR_SPEED_PID_MAX_IOUT);
    }

    s_last_control_tick = HAL_GetTick();
    s_tx_fail_streak = 0U;
    s_prev_ready = 0U;

    USB_Bridge_Init();
    Telemetry_Init();
    App_MotorConfig_Init();

    chassis->motors_ready = App_MotorConfig_AllReady();
    chassis->usb_online = USB_Bridge_IsOnline();
    chassis->last_loop_tick = HAL_GetTick();

    chassis_stop(chassis);
}

void chassis_stop(chassis_move_t *chassis)
{
    uint8_t i;

    if (chassis == NULL)
    {
        return;
    }

    chassis_reset_control(chassis);

    if (App_MotorConfig_AllReady() != 0U)
    {
        for (i = 0U; i < 4U; i++)
        {
            (void)DM_SendTorque(i, 0.0f);
        }
    }
}

/* ------------------------------------------------------------------------- */
/* 1. 反馈                                                                    */
/* ------------------------------------------------------------------------- */

void chassis_feedback_update(chassis_move_t *chassis)
{
    uint32_t now;
    uint8_t faults;
    uint8_t i;

    if (chassis == NULL)
    {
        return;
    }

    now = HAL_GetTick();

    /* USB 文本配置命令（@cfg...）：只在任务里执行，命令内部可能阻塞 */
    {
        char line[USB_TEXT_LINE_MAX];

        if (USB_Bridge_GetTextRequest(line, sizeof(line)) != 0U)
        {
            App_MotorConfig_HandleText(line);
        }
    }

    App_MotorConfig_Poll(now);

    chassis->usb_online = USB_Bridge_IsOnline();
    chassis->jetson_mode = USB_Bridge_GetMode();
    chassis->motors_ready = App_MotorConfig_AllReady();

    faults = 0U;

    if (chassis->motors_ready == 0U)
    {
        if (App_MotorConfig_GetFault() == MOTOR_CFG_FAULT_ENABLE_FAILED)
        {
            faults |= CHASSIS_FAULT_ENABLE_FAILED;
        }
        else
        {
            faults |= CHASSIS_FAULT_MOTORS_NOT_READY;
        }
    }

    for (i = 0U; i < 4U; i++)
    {
        DM_Feedback_t feedback;

        if ((DM_GetFeedback(i, &feedback) != 0U) &&
            (feedback.rx_count > 0U))
        {
            chassis->motor[i].speed =
                (double)(s_motor_dir[i] * feedback.vel_rad_s);
            chassis->motor[i].t_mos_c = (float)feedback.t_mos_c;
            chassis->motor[i].t_rotor_c = (float)feedback.t_rotor_c;
            chassis->motor[i].state = feedback.state;
            chassis->motor[i].last_update_tick = feedback.last_rx_tick;
            chassis->motor[i].online =
                ((now - feedback.last_rx_tick) <= CHASSIS_FEEDBACK_TIMEOUT_MS)
                    ? 1U : 0U;

            if (chassis->motors_ready != 0U)
            {
                if (chassis->motor[i].online == 0U)
                {
                    faults |= CHASSIS_FAULT_MOTOR_TIMEOUT;
                }
                if (feedback.state >= 8U)
                {
                    faults |= CHASSIS_FAULT_MOTOR_STATE;
                }
            }
        }
        else
        {
            chassis->motor[i].online = 0U;
            chassis->motor[i].state = 0U;

            if (chassis->motors_ready != 0U)
            {
                faults |= CHASSIS_FAULT_MOTOR_TIMEOUT;
            }
        }
    }

    if (BSP_CAN_IsBusOff() != 0U)
    {
        faults |= CHASSIS_FAULT_CAN_BUS_OFF;
    }

    chassis->fault_flags = faults;

    if ((chassis->motors_ready != 0U) && (s_prev_ready == 0U))
    {
        /* 刚使能成功，积分保持清零 */
        for (i = 0U; i < 4U; i++)
        {
            PID_Reset(&chassis->motor[i].speed_pid);
        }
    }
    s_prev_ready = chassis->motors_ready;
}

/* ------------------------------------------------------------------------- */
/* 2. 模式仲裁                                                                */
/* ------------------------------------------------------------------------- */

void chassis_mode_change(chassis_move_t *chassis)
{
    CarMode_t old_mode;

    if (chassis == NULL)
    {
        return;
    }

    old_mode = chassis->mode;

    if (App_MotorConfig_IsConfigActive() != 0U)
    {
        chassis->mode = CAR_MODE_CONFIG;
    }
    else if (chassis->fault_flags != 0U)
    {
        chassis->mode = CAR_MODE_FAULT;
    }
    else if ((chassis->motors_ready != 0U) && (chassis->usb_online != 0U))
    {
        chassis->mode = CAR_MODE_INDOOR;
    }
    else
    {
        chassis->mode = CAR_MODE_IDLE;
    }

    if (chassis->mode != old_mode)
    {
        chassis_reset_control(chassis);
        s_last_control_tick = HAL_GetTick();
    }
}

/* ------------------------------------------------------------------------- */
/* 3. 控制量                                                                  */
/* ------------------------------------------------------------------------- */

void chassis_set_control(chassis_move_t *chassis)
{
    uint32_t now;
    uint32_t dt_ms;
    float dt;
    float max_dv;
    float max_dw;
    float vx;
    float wz;

    if (chassis == NULL)
    {
        return;
    }

    now = HAL_GetTick();
    dt_ms = now - s_last_control_tick;
    if (dt_ms < 1U)
    {
        dt_ms = 1U;
    }
    if (dt_ms > 20U)
    {
        dt_ms = 20U;
    }
    s_last_control_tick = now;
    dt = (float)dt_ms * 0.001f;

    if ((chassis->mode != CAR_MODE_INDOOR) || (chassis->usb_online == 0U))
    {
        chassis->Vx_set = 0.0f;
        chassis->Vy_set = 0.0f;
        chassis->Wz_set = 0.0f;
        return;
    }

    /*
     * 符号约定：
     *   下行帧 vx 是固件原始约定（负=前进），DM 底盘内部 Vx 正=前进 → 取反；
     *   下行帧 wz 已被上位机按固件约定取过反（见 chassis_bridge.py），
     *   DM 底盘内部 Wz 正=逆时针（左转）→ 再取反，最终物理方向与 ROS REP-103 一致。
     */
    vx = -USB_Bridge_GetVx();
    wz = -USB_Bridge_GetWz();

    chassis->Vx_set = clampf(vx, -0.60f, 0.60f);
    chassis->Vy_set = 0.0f;
    chassis->Wz_set = clampf(wz, -1.50f, 1.50f);

    max_dv = CHASSIS_TRANSLATION_SLEW_MPS2 * dt;
    max_dw = CHASSIS_ROTATION_SLEW_RAD_S2 * dt;

    chassis->Vx_cmd = slew_limit(chassis->Vx_cmd, chassis->Vx_set, max_dv);
    chassis->Vy_cmd = slew_limit(chassis->Vy_cmd, chassis->Vy_set, max_dv);
    chassis->Wz_cmd = slew_limit(chassis->Wz_cmd, chassis->Wz_set, max_dw);
}

/* ------------------------------------------------------------------------- */
/* 4. 运动学 + PID                                                            */
/* ------------------------------------------------------------------------- */

void chassis_control_loop(chassis_move_t *chassis)
{
    float kinematic_m;
    float vy;
    float wz;
    float target[4];
    float max_abs;
    float scale;
    uint8_t i;

    if (chassis == NULL)
    {
        return;
    }

    if ((chassis->mode != CAR_MODE_INDOOR) ||
        (chassis->motors_ready == 0U))
    {
        for (i = 0U; i < 4U; i++)
        {
            chassis->motor[i].speed_set = 0.0;
            chassis->motor[i].torque_set = 0.0;
            PID_Reset(&chassis->motor[i].speed_pid);
        }
        return;
    }

    kinematic_m = CHASSIS_HALF_WHEELBASE_M + CHASSIS_HALF_TRACK_M;
    vy = CHASSIS_KIN_VY_SIGN * chassis->Vy_cmd;
    wz = CHASSIS_KIN_WZ_SIGN * chassis->Wz_cmd;

    /* x 前 / y 左 / ω 逆时针，FL/FR/RL/RR */
    target[0] = (chassis->Vx_cmd - vy - kinematic_m * wz) /
                CHASSIS_WHEEL_RADIUS_M;
    target[1] = (chassis->Vx_cmd + vy + kinematic_m * wz) /
                CHASSIS_WHEEL_RADIUS_M;
    target[2] = (chassis->Vx_cmd + vy - kinematic_m * wz) /
                CHASSIS_WHEEL_RADIUS_M;
    target[3] = (chassis->Vx_cmd - vy + kinematic_m * wz) /
                CHASSIS_WHEEL_RADIUS_M;

    max_abs = 0.0f;
    for (i = 0U; i < 4U; i++)
    {
        float abs_value = fabsf(target[i]);
        if (abs_value > max_abs)
        {
            max_abs = abs_value;
        }
    }

    if (max_abs > CHASSIS_MAX_WHEEL_RAD_S)
    {
        scale = CHASSIS_MAX_WHEEL_RAD_S / max_abs;
        for (i = 0U; i < 4U; i++)
        {
            target[i] *= scale;
        }
    }

    for (i = 0U; i < 4U; i++)
    {
        double output;
        double feedforward = 0.0;

        if ((CHASSIS_SPEED_DEADZONE_RAD_S > 0.0f) &&
            (fabsf(target[i]) < CHASSIS_SPEED_DEADZONE_RAD_S))
        {
            target[i] = 0.0f;
        }

        chassis->motor[i].speed_set = (double)target[i];

        output = PID_Calculate(&chassis->motor[i].speed_pid,
                               chassis->motor[i].speed,
                               chassis->motor[i].speed_set);

        if ((CHASSIS_FRICTION_FF_NM > 0.0f) &&
            (fabsf(chassis->motor[i].speed_set) >
             CHASSIS_FRICTION_FF_MIN_RAD_S))
        {
            feedforward = (chassis->motor[i].speed_set > 0.0)
                              ? CHASSIS_FRICTION_FF_NM
                              : -CHASSIS_FRICTION_FF_NM;
        }

        output += feedforward;
        output = (double)clampf((float)output,
                                (float)(-MOTOR_SPEED_PID_MAX_OUT),
                                (float)(MOTOR_SPEED_PID_MAX_OUT));

        chassis->motor[i].torque_set = output;
    }
}

/* ------------------------------------------------------------------------- */
/* 5. 发送                                                                    */
/* ------------------------------------------------------------------------- */

void chassis_send_cmd(chassis_move_t *chassis)
{
    uint8_t ready;
    uint8_t config_active;

    if (chassis == NULL)
    {
        return;
    }

    ready = App_MotorConfig_AllReady();
    config_active = App_MotorConfig_IsConfigActive();

    if (BSP_CAN_IsBusOff() != 0U)
    {
        chassis->fault_flags |= CHASSIS_FAULT_CAN_BUS_OFF;
        App_MotorConfig_ForceFault(MOTOR_CFG_FAULT_BUS_OFF);
        (void)BSP_CAN_Recover();
    }
    else
    {
        uint8_t i;
        uint8_t fail_count = 0U;

        if ((ready != 0U) && (config_active == 0U))
        {
            for (i = 0U; i < 4U; i++)
            {
                float torque;
                float raw_torque;

                /* 已掉线的电机不再发帧，避免无 ACK 把整条总线打到 bus-off */
                if ((chassis->motor[i].last_update_tick != 0U) &&
                    (chassis->motor[i].online == 0U))
                {
                    continue;
                }

                torque = (chassis->mode == CAR_MODE_INDOOR)
                             ? (float)chassis->motor[i].torque_set
                             : 0.0f;
                raw_torque = s_motor_dir[i] * torque;

                if (DM_SendTorque(i, raw_torque) == 0U)
                {
                    fail_count++;
                }
            }

            if (fail_count > 0U)
            {
                if (s_tx_fail_streak < 255U)
                {
                    s_tx_fail_streak++;
                }
            }
            else
            {
                s_tx_fail_streak = 0U;
            }

            if (s_tx_fail_streak >= CHASSIS_CAN_TX_FAIL_LIMIT)
            {
                chassis->fault_flags |= CHASSIS_FAULT_CAN_TX;
            }
        }
    }

    Telemetry_Update(chassis);
}

/* ------------------------------------------------------------------------- */
/* 任务                                                                       */
/* ------------------------------------------------------------------------- */

void chassis_task(void *argument)
{
    uint32_t wake;
    uint32_t previous_start = 0U;

    (void)argument;

    chassis_init(&chassis_move);

    wake = osKernelGetTickCount();

    for (;;)
    {
        uint32_t now = HAL_GetTick();
        uint32_t now_ticks;

        if (previous_start != 0U)
        {
            uint32_t period = now - previous_start;
            chassis_move.loop_dt_ms =
                (period > 0xFFFFU) ? 0xFFFFU : (uint16_t)period;
        }
        previous_start = now;

        chassis_feedback_update(&chassis_move);
        chassis_mode_change(&chassis_move);
        chassis_set_control(&chassis_move);
        chassis_control_loop(&chassis_move);
        chassis_send_cmd(&chassis_move);

        chassis_move.loop_count++;
        chassis_move.last_loop_tick = now;

        now_ticks = osKernelGetTickCount();
        if ((int32_t)(now_ticks - wake) > (int32_t)CHASSIS_TASK_PERIOD_MS)
        {
            /* 配置命令等阻塞导致过周期，不要追赶 */
            wake = now_ticks;
        }

        wake += CHASSIS_TASK_PERIOD_MS;
        (void)osDelayUntil(wake);
    }
}

chassis_move_t *chassis_get(void)
{
    return &chassis_move;
}
