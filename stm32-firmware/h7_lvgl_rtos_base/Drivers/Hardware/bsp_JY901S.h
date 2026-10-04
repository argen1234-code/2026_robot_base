#ifndef BSP_JY901S_H
#define BSP_JY901S_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define JY901S_UPDATE_ACC    0x01U
#define JY901S_UPDATE_GYRO   0x02U
#define JY901S_UPDATE_ANGLE  0x04U
#define JY901S_UPDATE_MAG    0x08U
#define JY901S_UPDATE_TEMP   0x10U

#define JY901S_ONLINE_TIMEOUT_MS  250U

typedef struct
{
    float acc[3];       /* g */
    float gyro[3];      /* deg/s */
    float angle[3];     /* roll, pitch, yaw, deg */
    int16_t mag[3];     /* raw magnetic field */
    float temperature;  /* deg C */
    uint32_t update_flag;
    uint32_t last_update_tick;
    uint8_t online;

    /*
     * 两个轴组各自的单调递增序号。它们只记录“这一组物理量是否收到新帧”，
     * 不参与姿态解算、底盘控制或串口协议；路面分类 BSP 用它们把 ACC 与 GYRO
     * 配成一条与训练 logger 完全一致的六轴样本。
     *
     * 特意追加在结构体末尾，不插入原有字段之间：这样 acc/gyro/angle/mag、
     * temperature、update_flag、last_update_tick、online 的原地址偏移全部保持不变，
     * 原先 Keil Watch 中观察这些字段的表达式仍然有效。
     */
    uint32_t acc_update_sequence;
    uint32_t gyro_update_sequence;
} JY901S_Data_t;

void JY901S_Init(void);
void JY901S_RxPro_HAL(uint8_t *pBuf, uint16_t Size);
uint8_t JY901S_GetData(JY901S_Data_t *pData);
uint32_t JY901S_GetUpdateFlag(void);
void JY901S_ClearUpdateFlag(uint32_t flag);

#ifdef __cplusplus
}
#endif

#endif
