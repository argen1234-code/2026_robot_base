#include "app_chassis_board.h"
#include "app_motor_config.h"
#include "app_telemetry.h"
#include "bsp_bluetooth.h"
#include "bsp_dm6215.h"
#include "bsp_fdcan.h"
#include "main.h"
#include "cmsis_os2.h"

#include <math.h>
#include <string.h>

#define CHASSIS_TASK_PERIOD_MS      5U
#define CHASSIS_FEEDBACK_TIMEOUT_MS 100U
#define CHASSIS_CAN_TX_FAIL_LIMIT   20U

/* 启动文件清零 .bss；chassis_init() 还会再 memset 一次 */
chassis_move_t chassis_move;

/*
 * 单电机台架测试（GDB 写这些变量，默认关闭）：
 *   set g_bench_esc_id = 1
 *   set g_bench_torque = 0.1
 *   set g_bench_duration_ms = 1500
 *   set g_bench_enable = 1
 * 固件会对指定 ESC_ID 发清错+使能，然后每 5 ms 发一帧 MIT 力矩；
 * 到时自动置零、失能，并置 g_bench_done=1。
 */
volatile uint8_t  g_bench_enable = 0U;
volatile uint8_t  g_bench_done = 0U;
volatile uint16_t g_bench_esc_id = 1U;
volatile float    g_bench_torque = 0.0f;
volatile uint32_t g_bench_duration_ms = 1500U;

/* 运行中观测：最大 state / 最大 |轮速| / 最后轮速，GDB 读取 */
volatile uint8_t  g_bench_max_state = 0U;
volatile float    g_bench_max_speed = 0.0f;
volatile float    g_bench_last_speed = 0.0f;

static uint8_t  s_bench_was_enabled = 0U;
static uint32_t s_bench_start_tick = 0U;

static const float s_motor_dir[4] =
{
    MOTOR_DIR_FL,
    MOTOR_DIR_FR,
    MOTOR_DIR_RL,
    MOTOR_DIR_RR
};

static uint32_t s_last_control_tick = 0U;
static uint8_t s_tx_fail_streak = 0U;
static uint8_t s_disable_sent = 0U;
static uint8_t s_prev_ready = 0U;

/* ------------------------------------------------------------------------- */
/* 工具                                                                       */
/* ------------------------------------------------------------------------- */

static float clampf(float value, float min, float max)
{
    if (value > max)
    {
        return max;
    }
    if (value < min)
    {
        return min;
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
    chassis->remote.bt_vx_mps = BT_REMOTE_VX_MPS;
    chassis->remote.bt_vy_mps = BT_REMOTE_VY_MPS;
    chassis->remote.bt_wz_rad_s = BT_REMOTE_WZ_RAD_S;
    chassis->remote.bt_gain = BT_REMOTE_CMD_GAIN;
    chassis->remote.max_vx_mps = CHASSIS_MAX_VX_MPS;
    chassis->remote.max_vy_mps = CHASSIS_MAX_VY_MPS;
    chassis->remote.max_wz_rad_s = CHASSIS_MAX_WZ_RAD_S;

    for (i = 0U; i < 4U; i++)
    {
        PID_init(&chassis->motor[i].speed_pid, PID_POSITION,
                 speed_pid_param,
                 MOTOR_SPEED_PID_MAX_OUT,
                 MOTOR_SPEED_PID_MAX_IOUT);
    }

    s_last_control_tick = HAL_GetTick();
    s_tx_fail_streak = 0U;
    s_disable_sent = 0U;
    s_prev_ready = 0U;

    BT_Init();
    Telemetry_Init();
    App_MotorConfig_Init();

    chassis->motors_ready = App_MotorConfig_AllReady();
    chassis->bt_online = BT_IsOnline();
    chassis->bt_keys = BT_GetKeyState();
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
    uint8_t persistent_faults;
    uint8_t faults;
    uint8_t i;

    if (chassis == NULL)
    {
        return;
    }

    now = HAL_GetTick();

    BT_Update();

    /* @~：清故障并重新探测/使能，不恢复旧运动 */
    if (BT_GetAndClearResumeRequest() != 0U)
    {
        chassis->emergency_latched = 0U;
        chassis->fault_flags = 0U;
        s_tx_fail_streak = 0U;
        BT_ClearKeys();
        App_MotorConfig_RequestRestart();
    }

    {
        BT_CfgRequest_t request;

        if (BT_GetCfgRequest(&request) != 0U)
        {
            App_MotorConfig_HandleBtRequest(request.payload);
        }
    }

    App_MotorConfig_Poll(now);

    /* 急停/恢复沿检测 */
    if (BT_IsPowerless() != 0U)
    {
        if (chassis->emergency_latched == 0U)
        {
            chassis->emergency_latched = 1U;
            chassis->fault_flags |= CHASSIS_FAULT_EMERGENCY;
            BT_ClearKeys();
        }
    }
    else if (chassis->emergency_latched != 0U)
    {
        /* @~：清故障、清键，重新探测/使能，不恢复旧运动 */
        chassis->emergency_latched = 0U;
        chassis->fault_flags = 0U;
        s_tx_fail_streak = 0U;
        BT_ClearKeys();
        App_MotorConfig_RequestRestart();
    }
    else
    {
        /* 正常 */
    }

    chassis->bt_online = BT_IsOnline();
    chassis->bt_keys = BT_GetKeyState();
    chassis->motors_ready = App_MotorConfig_AllReady();

    persistent_faults = (uint8_t)(chassis->fault_flags &
        (CHASSIS_FAULT_MOTOR_TIMEOUT |
         CHASSIS_FAULT_MOTOR_STATE |
         CHASSIS_FAULT_CAN_BUS_OFF |
         CHASSIS_FAULT_CAN_TX));
    faults = persistent_faults;

    if (chassis->emergency_latched != 0U)
    {
        faults |= CHASSIS_FAULT_EMERGENCY;
    }

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

    if (BSP_FDCAN_IsBusOff() != 0U)
    {
        faults |= CHASSIS_FAULT_CAN_BUS_OFF;
    }

    chassis->fault_flags = faults;

    if ((chassis->motors_ready != 0U) && (s_prev_ready == 0U))
    {
        /* 刚使能成功，按键/积分保持清零 */
        BT_ClearKeys();
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
    uint8_t request;

    if (chassis == NULL)
    {
        return;
    }

    old_mode = chassis->mode;
    request = BT_GetAndClearModeReq();

    if (request == BT_MODE_REQ_IDLE)
    {
        BT_ClearKeys();
    }

    if ((chassis->emergency_latched != 0U) || (BT_IsPowerless() != 0U))
    {
        chassis->mode = CAR_MODE_IDLE;
    }
    else if (App_MotorConfig_IsConfigActive() != 0U)
    {
        chassis->mode = CAR_MODE_CONFIG;
    }
    else if (chassis->fault_flags != 0U)
    {
        chassis->mode = CAR_MODE_FAULT;
    }
    else if ((chassis->motors_ready != 0U) && (chassis->bt_online != 0U))
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

        if (chassis->mode != CAR_MODE_INDOOR)
        {
            BT_ClearKeys();
        }

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

    if (chassis->mode != CAR_MODE_INDOOR)
    {
        chassis->Vx_set = 0.0f;
        chassis->Vy_set = 0.0f;
        chassis->Wz_set = 0.0f;
        chassis->Vx_cmd = 0.0f;
        chassis->Vy_cmd = 0.0f;
        chassis->Wz_cmd = 0.0f;
        return;
    }

    Remote_Control_Update(chassis);

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
            (fabs(chassis->motor[i].speed_set) >
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

    /* ---------------- 单电机台架测试（默认关闭） ---------------- */
    if (g_bench_enable != 0U)
    {
        if (s_bench_was_enabled == 0U)
        {
            g_bench_done = 0U;
            g_bench_max_state = 0U;
            g_bench_max_speed = 0.0f;
            g_bench_last_speed = 0.0f;

            /* 台架强制 MIT 模式，再清错/使能（帧按顺序进 TX FIFO） */
            (void)DM_SendDisable(g_bench_esc_id);
            (void)DM_WriteParam(g_bench_esc_id, DM_RID_CTRL_MODE,
                                DM_CTRL_MODE_MIT);
            (void)DM_SendClearError(g_bench_esc_id);
            (void)DM_SendEnable(g_bench_esc_id);

            s_bench_was_enabled = 1U;
            s_bench_start_tick = HAL_GetTick();
        }

        if ((HAL_GetTick() - s_bench_start_tick) >= g_bench_duration_ms)
        {
            g_bench_torque = 0.0f;
            g_bench_enable = 0U;
            g_bench_done = 1U;
        }
        else
        {
            uint8_t slot = DM_GetSlotByEscId((uint8_t)g_bench_esc_id);

            (void)DM_SendMit(g_bench_esc_id, 0.0f, 0.0f,
                             0.0f, 0.0f, g_bench_torque);

            if (slot < 4U)
            {
                float speed_abs = fabsf((float)chassis->motor[slot].speed);

                g_bench_last_speed = (float)chassis->motor[slot].speed;
                if (chassis->motor[slot].state > g_bench_max_state)
                {
                    g_bench_max_state = chassis->motor[slot].state;
                }
                if (speed_abs > g_bench_max_speed)
                {
                    g_bench_max_speed = speed_abs;
                }
            }
        }
    }

    if ((s_bench_was_enabled != 0U) && (g_bench_enable == 0U))
    {
        (void)DM_SendDisable(g_bench_esc_id);
        s_bench_was_enabled = 0U;
    }

    if (g_bench_enable != 0U)
    {
        Telemetry_Update(chassis);
        BT_ServiceTx();
        return;
    }
    /* ---------------- 台架测试结束 ---------------- */

    ready = App_MotorConfig_AllReady();
    config_active = App_MotorConfig_IsConfigActive();

    if (BSP_FDCAN_IsBusOff() != 0U)
    {
        chassis->fault_flags |= CHASSIS_FAULT_CAN_BUS_OFF;
        App_MotorConfig_ForceFault(MOTOR_CFG_FAULT_BUS_OFF);
        (void)BSP_FDCAN_Recover();
    }
    else if (BT_IsPowerless() != 0U)
    {
        if (s_disable_sent == 0U)
        {
            uint8_t i;
            for (i = 0U; i < 4U; i++)
            {
                (void)DM_SendDisable(DM_GetEscId(i));
            }
            s_disable_sent = 1U;
        }
    }
    else
    {
        uint8_t i;
        uint8_t fail_count = 0U;

        s_disable_sent = 0U;

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
    BT_ServiceTx();
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
