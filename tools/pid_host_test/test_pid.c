/* 轮速环低速死区的算法验证（PC 上用 gcc 跑真实的 pid.c）。
 *
 * 为什么写这个：真机实测"原地转弯只有指令的 30%"、"指令 0.30 rad/s 三秒完全不动"、
 * "指令 0.45 rad/s 八秒只到约 8 counts"。怀疑是 PID 输出幅度本身不够（而不是
 * 电机没电/没力矩）。这份测试直接编译【真实的 pid.c】，把不同 setpoint 下能达到的
 * 输出上限算出来，并对比几组候选参数，看谁能越过静摩擦起步 duty。
 *
 * 摩擦模型（参数按真机两点现象标定，只用于判断"能不能起步/能到多快"）：
 *      |duty| < D_BREAK   -> 轮子完全不动（静摩擦锁死）
 *      |duty| >= D_BREAK  -> 有效转矩 = |duty| - D_COULOMB，向目标转速一阶趋近
 * 标定：setpoint=12 输出峰值 16.6 -> 不动，故 D_BREAK 取 18（>16.6）
 *       setpoint=18 输出峰值 22.9 -> 实测稳态约 8 counts
 *       => (22.9 - D_COULOMB)*GAIN = 8  -> D_COULOMB=8.5, GAIN=0.55
 *
 * ⚠️ dt 固定 10 ms（与固件 chassis_task 的 osDelay(10) 一致）。
 *    固件 PID 的 ErrorInt 是【不乘 dt 的累加】，Ki 单位是"每采样"，
 *    所以必须用同样的调用节奏，结果才有意义。
 *
 * 编译运行：
 *     gcc -std=gnu89 -Wall -Wextra -I. -I<pid.h 目录> test_pid.c <pid.c> -o test_pid -lm
 */

#include <stdio.h>
#include <math.h>

#include "pid.h"

/* ---- 真机常量（与 app_chassis_board.h / tim.c 一致） ---- */
#define PWM_MAX              99.0    /* TIM8 ARR=99 -> duty 0..99 */
#define DT_S                 0.010   /* 10 ms */
#define FULL_SCALE_COUNTS    120.0   /* 固件 ROS_LINE_MAX_SPEED */
#define SETPOINT_ROT_SLOW    12.0    /* 0.30 rad/s 原地转 */
#define SETPOINT_ROT_MAX     18.0    /* 0.45 rad/s 原地转 */

/* ---- 摩擦模型标定值（见文件头推导） ---- */
#define D_BREAK              18.0    /* 起步 duty */
#define D_COULOMB            8.5     /* 库仑摩擦 duty */
#define GAIN_COUNTS_PER_DUTY 0.55    /* 有效转矩 -> 稳态 counts */

typedef struct {
    double out_peak;        /* 观察到的输出峰值 */
    double speed_final;     /* 末速 (counts/10ms) */
    double breakout_ms;     /* 起步耗时，<0 表示从未起步 */
} result_t;

static void run_case(const double kp, const double ki, const double kd,
                     const double max_out, const double max_iout,
                     const double setpoint, const double sim_seconds,
                     result_t *r)
{
    PID_t pid;
    const double param[3] = { 0.0, 0.0, 0.0 };
    double motor_speed = 0.0;
    double t = 0.0;
    int n;
    int n_steps;

    /* PID_init 要求 const double[3]，这里复制一份（C89 下不能就地初始化数组） */
    {
        double p[3];
        p[0] = kp; p[1] = ki; p[2] = kd;
        PID_init(&pid, PID_POSITION, p, max_out, max_iout);
        (void)param;
    }

    r->out_peak = 0.0;
    r->breakout_ms = -1.0;

    n_steps = (int)(sim_seconds / DT_S + 0.5);
    for (n = 0; n < n_steps; ++n)
    {
        double duty;
        double applied;
        double eff_torque;
        double target_speed;

        /* 复刻固件顺序：PID_Calculate(&pid, speed, speed_set) */
        duty = PID_Calculate(&pid, motor_speed, setpoint);
        if (fabs(duty) > r->out_peak) r->out_peak = fabs(duty);

        /* ⚠️ 真机 Motor_SetPWM 里有 LimitMax(PWM, PWM_MAX=99)：
           实际加到电机上的 duty 被执行器夹住，PID 自己并不知道。
           模型必须复刻这一点，否则会误判"OutMax 对齐到 99 会降低最高速"。 */
        applied = duty;
        if (applied > PWM_MAX) applied = PWM_MAX;
        if (applied < -PWM_MAX) applied = -PWM_MAX;

        if (fabs(applied) < D_BREAK)
        {
            eff_torque = 0.0;
            motor_speed = 0.0;
        }
        else
        {
            eff_torque = (applied > 0.0 ? 1.0 : -1.0) * (fabs(applied) - D_COULOMB);
            if (r->breakout_ms < 0.0) r->breakout_ms = t * 1000.0;
        }

        target_speed = eff_torque * GAIN_COUNTS_PER_DUTY;
        motor_speed += (target_speed - motor_speed) * 0.35;

        /* 编码器量化到整数 counts（真机 Encoder_Rpm_Get 返回整数计数） */
        motor_speed = (double)((long)(motor_speed > 0 ? motor_speed + 0.5
                                                    : motor_speed - 0.5));
        if (fabs(applied) < D_BREAK) motor_speed = 0.0;

        t += DT_S;
    }

    r->speed_final = motor_speed;
}

static void print_row(const char *label, const result_t *r, const double setpoint)
{
    char boot[32];
    if (r->breakout_ms < 0.0) sprintf(boot, "从未起步");
    else sprintf(boot, "%.0f ms", r->breakout_ms);

    printf("%-22s sp=%5.1f | 输出峰值 %7.2f (%3.0f%% duty) | 末速 %5.1f counts | 起步 %s\n",
           label, setpoint, r->out_peak, r->out_peak / PWM_MAX * 100.0,
           r->speed_final, boot);
}

typedef struct {
    const char *name;
    double kp, ki, kd, max_out, max_iout;
} config_t;

int main(void)
{
    /* 现状 vs 部署后。部署后按已改的固件：MAX_OUT=99(=PWM_MAX)、MAX_IOUT=990(=PWM_MAX/Ki)。
     * VZ_SCALE 40->60 之后，0.45 rad/s 对应的 setpoint 也从 18 变成 27，一并测。 */
    static const config_t cfgs[] = {
        { "现状 OutMax=150 I=40  ", 1.05, 0.10, 0.10, 150.0,  40.0 },
        { "改后 OutMax=99  I=990 ", 1.05, 0.10, 0.10,  99.0, 990.0 },
    };
    const int n_cfg = (int)(sizeof(cfgs) / sizeof(cfgs[0]));
    int i;
    result_t r;

    printf("PWM_MAX=%.0f  满量程轮速=%.0f counts  D_BREAK=%.0f  D_COULOMB=%.1f\n",
           PWM_MAX, FULL_SCALE_COUNTS, D_BREAK, D_COULOMB);
    printf("模型校验：现状 sp=12 必须不动、sp=18 稳态约 8 counts（与真机实测吻合）\n\n");

    printf("=========== sp=12  (改前: 0.30 rad/s; 改后同一个 setpoint) ===========\n");
    for (i = 0; i < n_cfg; ++i)
    {
        run_case(cfgs[i].kp, cfgs[i].ki, cfgs[i].kd, cfgs[i].max_out,
                 cfgs[i].max_iout, SETPOINT_ROT_SLOW, 3.0, &r);
        print_row(cfgs[i].name, &r, SETPOINT_ROT_SLOW);
    }

    printf("\n=========== sp=18  (改前 0.45 rad/s 用到的 setpoint, 8 s) ===========\n");
    for (i = 0; i < n_cfg; ++i)
    {
        run_case(cfgs[i].kp, cfgs[i].ki, cfgs[i].kd, cfgs[i].max_out,
                 cfgs[i].max_iout, SETPOINT_ROT_MAX, 8.0, &r);
        print_row(cfgs[i].name, &r, SETPOINT_ROT_MAX);
    }

    printf("\n=========== sp=27  (改后 VZ_SCALE=60 时 0.45 rad/s 对应的 setpoint) =====\n");
    for (i = 0; i < n_cfg; ++i)
    {
        run_case(cfgs[i].kp, cfgs[i].ki, cfgs[i].kd, cfgs[i].max_out,
                 cfgs[i].max_iout, 27.0, 6.0, &r);
        print_row(cfgs[i].name, &r, 27.0);
    }

    printf("\n=========== sp=120 平移满速 (2 s) 看有没有副作用 ===========\n");
    for (i = 0; i < n_cfg; ++i)
    {
        run_case(cfgs[i].kp, cfgs[i].ki, cfgs[i].kd, cfgs[i].max_out,
                 cfgs[i].max_iout, FULL_SCALE_COUNTS, 2.0, &r);
        print_row(cfgs[i].name, &r, FULL_SCALE_COUNTS);
    }

    printf("\n--- 堵转时可达输出上限的解析式 ---\n");
    printf("  Out_max = Kp*setpoint + Ki*max_iout\n");
    printf("  现状: sp=12 -> 1.05*12 + 0.1*40  = %6.1f (%2.0f%% duty)  < D_BREAK=%.0f 起不来\n",
           1.05 * 12.0 + 0.1 * 40.0, (1.05 * 12.0 + 0.1 * 40.0) / PWM_MAX * 100.0, D_BREAK);
    printf("  现状: sp=18 -> 1.05*18 + 0.1*40  = %6.1f (%2.0f%% duty)\n",
           1.05 * 18.0 + 0.1 * 40.0, (1.05 * 18.0 + 0.1 * 40.0) / PWM_MAX * 100.0);
    printf("  现状: sp=27 -> 1.05*27 + 0.1*40  = %6.1f (%2.0f%% duty)\n",
           1.05 * 27.0 + 0.1 * 40.0, (1.05 * 27.0 + 0.1 * 40.0) / PWM_MAX * 100.0);
    printf("  改后: 积分上限从 4.0 个 duty 提到 99.0 个 duty（Ki*990），不再是瓶颈。\n");

    return 0;
}
