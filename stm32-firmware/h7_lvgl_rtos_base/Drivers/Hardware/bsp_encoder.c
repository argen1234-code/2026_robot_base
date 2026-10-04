#include "bsp_encoder.h"
#include "tim.h"


//PB3     ------> TIM2_CH2
//PA5     ------> TIM2_CH1
//PB5     ------> TIM3_CH2
//PB4 (NJTRST)     ------> TIM3_CH1
//PB6     ------> TIM4_CH1           ËÄ¸ö±àÂëÆ÷
//PB7     ------> TIM4_CH2
//PH10     ------> TIM5_CH1
//PH11     ------> TIM5_CH2

int16_t Encoder_Rpm_Get(uint8_t wheel_position)
{
    if (wheel_position == WHEEL_FRONT_LEFT)
    {
        int16_t Temp_FL;
        Temp_FL = __HAL_TIM_GET_COUNTER(&htim2);
        __HAL_TIM_SET_COUNTER(&htim2, 0);
        return -Temp_FL;
    }
    else if (wheel_position == WHEEL_FRONT_RIGHT)
    {
        int16_t Temp_FR;
        Temp_FR = __HAL_TIM_GET_COUNTER(&htim3);
        __HAL_TIM_SET_COUNTER(&htim3, 0);
        return -Temp_FR;
    }
    else if (wheel_position == WHEEL_REAR_LEFT)
    {
        int16_t Temp_RL;
        Temp_RL = __HAL_TIM_GET_COUNTER(&htim4);
        __HAL_TIM_SET_COUNTER(&htim4, 0);
        return -Temp_RL;
    }
    else if (wheel_position == WHEEL_REAR_RIGHT)
    {
        int16_t Temp_RR;
        Temp_RR = __HAL_TIM_GET_COUNTER(&htim5);
        __HAL_TIM_SET_COUNTER(&htim5, 0);
        return -Temp_RR;
    }
    return 0;
}

void Encoder_Init(void)
{
    // Æô¶¯Ç°×ó±àÂëÆ÷ (htim2)
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_2);

    // Æô¶¯Ç°ÓÒ±àÂëÆ÷ (htim3)
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_2);

    // Æô¶¯ºó×ó±àÂëÆ÷ (htim4)
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_2);

    // Æô¶¯ºóÓÒ±àÂëÆ÷ (htim5)
    HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_1);
    HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_2);
}

