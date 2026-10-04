#include "bsp_motor.h"
#include "pid.h"
#include "stdlib.h"
#include "tim.h"


void Motor_Init(void)
{
    // ��������PWMͨ��
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);  //A
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);  //B
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);  //C
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);  //D
}

void PWM_SetCompare(uint16_t Compare, uint8_t motor_position)
{
    switch(motor_position)
    {
        case MOTOR_FRONT_LEFT:
            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, Compare);
            break;
        case MOTOR_FRONT_RIGHT:
            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, Compare);
            break;
        case MOTOR_REAR_LEFT:
            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, Compare);
            break;
        case MOTOR_REAR_RIGHT:
            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, Compare);
            break;
        default:
            break;
    }
}

void Motor_SetDirection(int16_t PWM, uint8_t motor_position)
{
    switch(motor_position)
    {
        case MOTOR_FRONT_LEFT:
            if (PWM >= 0)
            {
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
            }
            else
            {
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
            }
            break;
            
        case MOTOR_FRONT_RIGHT:
            if (PWM >= 0)
            {
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);   // PC4=DIR1 (ex-PC11, moved for SDMMC1)
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET); // PC5=DIR2 (ex-PC12, moved for SDMMC1)
            }
            else
            {
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
            }
            break;
            
        case MOTOR_REAR_LEFT:
            if (PWM >= 0)
            {
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
            }
            else
            {
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
            }
            break;
            
        case MOTOR_REAR_RIGHT:
            if (PWM >= 0)
            {
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
            }
            else
            {
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
            }
            break;
            
        default:
            break;
    }
}

void Motor_SetPWM(int16_t PWM, uint8_t motor_position)
{
    LimitMax(PWM, PWM_MAX);  // ��PWM�޷�
    
    // ���õ������
    Motor_SetDirection(PWM, motor_position);
    
    // ����PWMֵ
    PWM_SetCompare(abs(PWM), motor_position);
}

// ͳһ�������е��PWM
void Motor_SetAllPWM(int16_t front_left, int16_t front_right, int16_t rear_left, int16_t rear_right)
{
    Motor_SetPWM(front_left, MOTOR_FRONT_LEFT);
    Motor_SetPWM(front_right, MOTOR_FRONT_RIGHT);
    Motor_SetPWM(rear_left, MOTOR_REAR_LEFT);
    Motor_SetPWM(rear_right, MOTOR_REAR_RIGHT);
}
	
