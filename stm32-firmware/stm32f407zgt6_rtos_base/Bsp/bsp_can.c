#include "bsp_can.h"
#include "main.h"
#include "can.h"
#include <string.h>

#define BSP_CAN_INSTANCE    (&hcan1)

static BSP_CAN_RxCallback_t s_rx_callback = NULL;
static volatile uint8_t s_bus_off = 0U;
static BSP_CAN_Stats_t s_stats = {0};

/* 接收全部标准帧，软件按 ID 分发 */
static uint8_t can_config(void)
{
    CAN_FilterTypeDef filter = {0};

    filter.FilterBank = 0U;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0x0000U;
    filter.FilterIdLow = 0x0000U;
    filter.FilterMaskIdHigh = 0x0000U;
    filter.FilterMaskIdLow = 0x0000U;   /* mask = 0：不比较 ID */
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14U;

    if (HAL_CAN_ConfigFilter(BSP_CAN_INSTANCE, &filter) != HAL_OK)
    {
        return 0U;
    }

    if (HAL_CAN_ActivateNotification(BSP_CAN_INSTANCE,
                                     CAN_IT_RX_FIFO0_MSG_PENDING |
                                     CAN_IT_BUSOFF |
                                     CAN_IT_ERROR_PASSIVE |
                                     CAN_IT_ERROR_WARNING) != HAL_OK)
    {
        return 0U;
    }

    if (HAL_CAN_Start(BSP_CAN_INSTANCE) != HAL_OK)
    {
        return 0U;
    }

    return 1U;
}

void BSP_CAN_Init(void)
{
    memset(&s_stats, 0, sizeof(s_stats));
    s_bus_off = 0U;

    MX_CAN1_Init();

    if (can_config() == 0U)
    {
        Error_Handler();
    }
}

void BSP_CAN_SetRxCallback(BSP_CAN_RxCallback_t cb)
{
    s_rx_callback = cb;
}

uint8_t BSP_CAN_Send(uint16_t std_id, const uint8_t data[8])
{
    CAN_TxHeaderTypeDef tx = {0};
    uint32_t mailbox = 0U;

    if (s_bus_off != 0U)
    {
        s_stats.tx_fail++;
        return 0U;
    }

    if (HAL_CAN_GetTxMailboxesFreeLevel(BSP_CAN_INSTANCE) == 0U)
    {
        s_stats.tx_fail++;
        return 0U;
    }

    tx.StdId = (uint32_t)std_id;
    tx.ExtId = 0U;
    tx.IDE = CAN_ID_STD;
    tx.RTR = CAN_RTR_DATA;
    tx.DLC = 8U;
    tx.TransmitGlobalTime = DISABLE;

    if (HAL_CAN_AddTxMessage(BSP_CAN_INSTANCE, &tx, data, &mailbox) != HAL_OK)
    {
        s_stats.tx_fail++;
        return 0U;
    }

    s_stats.tx_ok++;
    return 1U;
}

uint8_t BSP_CAN_IsBusOff(void)
{
    return s_bus_off;
}

uint8_t BSP_CAN_Recover(void)
{
    (void)HAL_CAN_Stop(BSP_CAN_INSTANCE);
    (void)HAL_CAN_DeInit(BSP_CAN_INSTANCE);

    MX_CAN1_Init();

    if (can_config() == 0U)
    {
        return 0U;
    }

    s_bus_off = 0U;
    return 1U;
}

void BSP_CAN_GetStats(BSP_CAN_Stats_t *out)
{
    if (out == NULL)
    {
        return;
    }

    *out = s_stats;
}

/* ------------------------------------------------------------------------- */
/* HAL 回调                                                                   */
/* ------------------------------------------------------------------------- */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    BSP_CAN_Frame_t frame;
    CAN_RxHeaderTypeDef rx_header = {0};
    uint8_t rx_data[8] = {0};

    if (hcan->Instance != CAN1)
    {
        return;
    }

    /* 一次中断可能积压多帧，必须排空 FIFO */
    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U)
    {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0,
                                 &rx_header, rx_data) != HAL_OK)
        {
            break;
        }

        if ((rx_header.IDE != CAN_ID_STD) || (rx_header.DLC != 8U))
        {
            s_stats.unexpected_rx++;
            continue;
        }

        frame.id = (uint16_t)rx_header.StdId;
        frame.len = 8U;
        frame.tick = HAL_GetTick();
        memcpy(frame.data, rx_data, 8U);

        s_stats.rx_count++;

        if (s_rx_callback != NULL)
        {
            s_rx_callback(&frame);
        }
        else
        {
            s_stats.unexpected_rx++;
        }
    }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    uint32_t error;

    if (hcan->Instance != CAN1)
    {
        return;
    }

    error = HAL_CAN_GetError(hcan);

    if ((error & HAL_CAN_ERROR_BOF) != 0U)
    {
        s_bus_off = 1U;
        s_stats.bus_off_count++;
    }

    if ((error & HAL_CAN_ERROR_EPV) != 0U)
    {
        s_stats.error_passive_count++;
    }
}
