#include "pid.h"
#include <stddef.h>

/**
  * @brief  位置式 PID 初始化
  * @param  pid       PID 实例
  * @param  mode      目前只支持 PID_POSITION
  * @param  PID       增益数组 {Kp, Ki, Kd}
  * @param  max_out   输出限幅（对称）
  * @param  max_iout  积分限幅（夹的是原始误差累加量，不是输出量）
  */
void PID_init(PID_t *pid, uint8_t mode, const double PID[3],
              double max_out, double max_iout)
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }

    pid->mode = mode;
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];

    pid->OutMax = max_out;
    pid->max_iout = max_iout;

    pid->Target = 0.0;
    pid->Actual = 0.0;
    pid->Out = 0.0;

    pid->Error0 = 0.0;
    pid->Error1 = 0.0;
    pid->ErrorInt = 0.0;
    pid->D_error = 0.0;
}

/**
  * @brief  位置式 PID 计算
  * @param  pid  PID 实例
  * @param  ref  实测值
  * @param  set  目标值
  * @retval PID 输出（已限幅）
  * @note   无 dt 缩放，Ki/Kd 都是“每个控制周期”的系数；
  *         控制周期改变后需要重新整定。
  */
double PID_Calculate(PID_t *pid, double ref, double set)
{
    if (pid == NULL)
    {
        return 0.0;
    }

    if (pid->mode == PID_POSITION)
    {
        pid->Target = set;
        pid->Actual = ref;

        pid->Error0 = pid->Target - pid->Actual;

        /* 积分累加，先夹积分再参与输出，防止积分饱和 */
        pid->ErrorInt += pid->Error0;
        LimitMax(pid->ErrorInt, pid->max_iout);

        /* 微分作用于误差 */
        pid->D_error = pid->Error0 - pid->Error1;

        pid->Out = pid->Kp * pid->Error0 +
                   pid->Ki * pid->ErrorInt +
                   pid->Kd * pid->D_error;

        LimitMax(pid->Out, pid->OutMax);

        pid->Error1 = pid->Error0;
    }

    return pid->Out;
}

/**
  * @brief  清空 PID 历史和积分，模式切换/急停恢复时调用
  */
void PID_Reset(PID_t *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->Target = 0.0;
    pid->Actual = 0.0;
    pid->Out = 0.0;

    pid->Error0 = 0.0;
    pid->Error1 = 0.0;
    pid->ErrorInt = 0.0;
    pid->D_error = 0.0;
}
