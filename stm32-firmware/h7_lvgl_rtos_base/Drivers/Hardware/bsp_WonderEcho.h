#ifndef __BSP_WONDERECHO_H
#define __BSP_WONDERECHO_H

#include "stm32h7xx_hal.h"

/* WonderEcho(CI1302) 软件 I2C 协议参数 */
#define WONDERECHO_ADDR_7BIT            0x34U
#define WONDERECHO_WRITE_ADDR           ((uint8_t)(WONDERECHO_ADDR_7BIT << 1))
#define WONDERECHO_READ_ADDR            ((uint8_t)((WONDERECHO_ADDR_7BIT << 1) | 0x01U))

#define WONDERECHO_RESULT_REG           0x64U
#define WONDERECHO_SPEAK_REG            0x6EU

/* 语音识别结果 ID */
#define WONDERECHO_CMD_FORWARD          0x01U
#define WONDERECHO_CMD_BACKWARD         0x02U
#define WONDERECHO_CMD_TURN_LEFT        0x03U
#define WONDERECHO_CMD_TURN_RIGHT       0x04U
#define WONDERECHO_CMD_STOP             0x09U
#define WONDERECHO_CMD_SPEED_UP         0x0DU
#define WONDERECHO_CMD_SPEED_DOWN       0x0EU

typedef struct
{
    uint32_t read_count;
    uint32_t read_fail_count;
    uint32_t speak_count;
    uint32_t last_read_tick;
    uint32_t last_speak_tick;
    uint8_t  last_result;
    uint8_t  last_fail_step;
    uint8_t  last_speak_cmd;
    uint8_t  last_speak_id;
} WonderEcho_Debug_t;

extern volatile WonderEcho_Debug_t wonderecho_debug;

void WonderEcho_I2C_Init(void);
uint8_t WonderEcho_GetResult(void);
void WonderEcho_Speak(uint8_t cmd, uint8_t idNum);

#endif
