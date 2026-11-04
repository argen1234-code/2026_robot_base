#include "app_pid.h"
void AppPid_Reset(AppPid *pid){pid->integral=pid->previous=0;pid->initialized=0;}
float AppPid_Step(AppPid *pid,float error,float measurement,float kp,float ki,float kd,float dt,float limit)
{
 float derivative=pid->initialized?(measurement-pid->previous)/dt:0;
 float integral=pid->integral+ki*error*dt;
 float output=kp*error+integral-kd*derivative;
 if((output<=limit && output>= -limit) || (output>limit && error<0) || (output< -limit && error>0))pid->integral=integral;
 output=kp*error+pid->integral-kd*derivative;
 if(output>limit)output=limit;
 if(output< -limit)output= -limit;
 pid->previous=measurement;pid->initialized=1;return output;
}
