#ifndef BSP_MOTOR_H
#define BSP_MOTOR_H

#include "main.h"

#define PWM_MAX     99


// 电机位置定义

enum{
	MOTOR_FRONT_LEFT = 0, MOTOR_FRONT_RIGHT, MOTOR_REAR_LEFT, MOTOR_REAR_RIGHT
};

extern void Motor_Init(void);
extern void Motor_SetPWM(int16_t PWM, uint8_t motor_position);

// 统一设置所有电机PWM
extern void Motor_SetAllPWM(int16_t front_left, int16_t front_right, int16_t rear_left, int16_t rear_right);

#endif
