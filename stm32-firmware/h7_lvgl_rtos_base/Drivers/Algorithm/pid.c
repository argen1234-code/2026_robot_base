#include "pid.h"
#include "math.h"



void PID_init(PID_t *pid, uint8_t mode, const double PID[3], double max_out, double max_iout)
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
	pid->D_error = 0.0f;
	pid->Error0 = pid->Error1 = pid->Out = pid->ErrorInt = 0.0f;
}

/**
  * @brief          pid计算
  * @param[out]        
  * @retval         
  */
double PID_Calculate(PID_t *pid, double ref, double set)  //位置式
{
	if (pid->mode == PID_POSITION)
	{
		pid->Target = set;
		pid->Actual = ref;
    // 计算当前误差
    pid->Error0 = pid->Target - pid->Actual;


    // 积分项累加（先尝试更新）
    pid->ErrorInt += pid->Error0;
		LimitMax(pid->ErrorInt, pid->max_iout);

    // 计算微分项（当前误差 - 上次误差）
    pid->D_error = pid->Error0 - pid->Error1;

    // 计算PID输出
    pid->Out = pid->Kp * pid->Error0 + 
               pid->Ki * pid->ErrorInt + 
               pid->Kd * pid->D_error;
		
		LimitMax(pid->Out, pid->OutMax);

    // 更新误差历史记录
    pid->Error1 = pid->Error0;  // 将当前误差保存为上次误差
		
		
	}
	
	return pid->Out;
}
