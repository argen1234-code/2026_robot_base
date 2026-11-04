#ifndef APP_PID_H
#define APP_PID_H
typedef struct {float integral,previous;unsigned initialized;} AppPid;
void AppPid_Reset(AppPid *pid);
/* Positional PID; derivative of measurement avoids target-step derivative kick.
 * Conditional integration prevents windup at the physical output limit. */
float AppPid_Step(AppPid *pid,float error,float measurement,float kp,float ki,float kd,float dt,float limit);
#endif
