#include "bsp_JY901S.h"
#include "bsp_uart.h"
#include "bsp_wit_c_sdk.h"
#include "usart.h"
#include <string.h>

static JY901S_Data_t s_jy901s_data;
static volatile uint32_t s_jy901s_update_flag = 0;
static uint8_t s_jy901s_inited = 0;

static void JY901S_SerialWrite(uint8_t *p_data, uint32_t size)
{
    if (p_data == NULL || size == 0)
    {
        return;
    }

    HAL_UART_Transmit(&huart6, p_data, size, 100);
}

static void JY901S_DelayMs(uint16_t ms)
{
    HAL_Delay(ms);
}

static void JY901S_UpdateScaledData(uint32_t flag)
{
    if (flag & JY901S_UPDATE_ACC)
    {
        s_jy901s_data.acc[0] = sReg[AX] / 32768.0f * 16.0f;
        s_jy901s_data.acc[1] = sReg[AY] / 32768.0f * 16.0f;
        s_jy901s_data.acc[2] = sReg[AZ] / 32768.0f * 16.0f;

        /*
         * 这是一个纯观测序号：不影响原有 acc 数值、姿态、PID 或电机控制。
         * JY901S 的 ACC 与 GYRO 分属不同串口帧，分类 BSP 必须知道两者均已
         * 刷新，才能生成一条与上位机 logger 相同的六轴时间样本。
         */
        s_jy901s_data.acc_update_sequence++;
    }

    if (flag & JY901S_UPDATE_GYRO)
    {
        s_jy901s_data.gyro[0] = sReg[GX] / 32768.0f * 2000.0f;
        s_jy901s_data.gyro[1] = sReg[GY] / 32768.0f * 2000.0f;
        s_jy901s_data.gyro[2] = sReg[GZ] / 32768.0f * 2000.0f;

        /* 同上：仅为分类采样配对提供“新陀螺帧”标记，不改变任何已有功能。 */
        s_jy901s_data.gyro_update_sequence++;
    }

    if (flag & JY901S_UPDATE_ANGLE)
    {
        s_jy901s_data.angle[0] = sReg[Roll] / 32768.0f * 180.0f;
        s_jy901s_data.angle[1] = sReg[Pitch] / 32768.0f * 180.0f;
        s_jy901s_data.angle[2] = sReg[Yaw] / 32768.0f * 180.0f;
    }

    if (flag & JY901S_UPDATE_MAG)
    {
        s_jy901s_data.mag[0] = sReg[HX];
        s_jy901s_data.mag[1] = sReg[HY];
        s_jy901s_data.mag[2] = sReg[HZ];
    }

    if (flag & JY901S_UPDATE_TEMP)
    {
        s_jy901s_data.temperature = sReg[TEMP] / 100.0f;
    }

    s_jy901s_data.update_flag |= flag;
    s_jy901s_data.last_update_tick = HAL_GetTick();
    s_jy901s_data.online = 1;
    s_jy901s_update_flag |= flag;
}

static void JY901S_RegUpdateCallback(uint32_t uiReg, uint32_t uiRegNum)
{
    uint32_t flag = 0;

    for (uint32_t i = 0; i < uiRegNum; i++)
    {
        switch (uiReg)
        {
            case AZ:
                flag |= JY901S_UPDATE_ACC;
                break;
            case GZ:
                flag |= JY901S_UPDATE_GYRO;
                break;
            case HZ:
                flag |= JY901S_UPDATE_MAG;
                break;
            case Yaw:
                flag |= JY901S_UPDATE_ANGLE;
                break;
            case TEMP:
                flag |= JY901S_UPDATE_TEMP;
                break;
            default:
                break;
        }
        uiReg++;
    }

    if (flag != 0)
    {
        JY901S_UpdateScaledData(flag);
    }
}

void JY901S_Init(void)
{
    if (s_jy901s_inited)
    {
        return;
    }

    memset(&s_jy901s_data, 0, sizeof(s_jy901s_data));
    s_jy901s_update_flag = 0;

    WitInit(WIT_PROTOCOL_NORMAL, 0x50);
    WitSerialWriteRegister(JY901S_SerialWrite);
    WitRegisterCallBack(JY901S_RegUpdateCallback);
    WitDelayMsRegister(JY901S_DelayMs);

    uart_init(&huart6, UART_DMA_ToIdle_RX);

    s_jy901s_inited = 1;
}

void JY901S_RxPro_HAL(uint8_t *pBuf, uint16_t Size)
{
    if (pBuf == NULL || Size == 0)
    {
        return;
    }

    for (uint16_t i = 0; i < Size; i++)
    {
        WitSerialDataIn(pBuf[i]);
    }
}

uint8_t JY901S_GetData(JY901S_Data_t *pData)
{
    uint32_t primask;
    uint32_t now;

    if (pData == NULL)
    {
        return 0;
    }

    now = HAL_GetTick();
    primask = __get_PRIMASK();
    __disable_irq();
    if (s_jy901s_data.last_update_tick == 0U ||
        (uint32_t)(now - s_jy901s_data.last_update_tick) > JY901S_ONLINE_TIMEOUT_MS)
    {
        s_jy901s_data.online = 0U;
    }
    *pData = s_jy901s_data;
    if (primask == 0U)
    {
        __enable_irq();
    }

    return pData->online;
}

uint32_t JY901S_GetUpdateFlag(void)
{
    return s_jy901s_update_flag;
}

void JY901S_ClearUpdateFlag(uint32_t flag)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    s_jy901s_update_flag &= ~flag;
    s_jy901s_data.update_flag &= ~flag;
    if (primask == 0U)
    {
        __enable_irq();
    }
}
