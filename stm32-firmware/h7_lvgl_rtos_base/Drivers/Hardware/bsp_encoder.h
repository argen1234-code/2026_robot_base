#ifndef BSP_ENCODER_H
#define BSP_ENCODER_H

#include "main.h"

#define BUFFER_LENGTH 100 // 要采集的样本数

enum{
    WHEEL_FRONT_LEFT = 0, 
    WHEEL_FRONT_RIGHT, 
    WHEEL_REAR_LEFT, 
    WHEEL_REAR_RIGHT
};

extern void Encoder_Init(void);
extern int16_t Encoder_Rpm_Get(uint8_t wheel_position);

#endif

