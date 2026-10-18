#include "bsp_fdcan.h"
#include "main.h"
#include "fdcan.h"
#include <string.h>

#define BSP_FDCAN_INSTANCE   (&hfdcan1)

static BSP_FDCAN_RxCallback_t s_rx_callback = NULL;
static volatile uint8_t s_bus_off = 0U;
static BSP_FDCAN_Stats_t s_stats = {0};

static uint8_t fdcan_config(void)
{
    FDCAN_FilterTypeDef filter = {0};

    /* 接收全部标准帧，软件按 ID 分发 */
    filter.IdType       = FDCAN_STANDARD_ID;
    filter.FilterIndex  = 0;
    filter.FilterType   = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1    = 0x000U;
    filter.FilterID2    = 0x000U;   /* mask = 0：不比较 ID */

    if (HAL_FDCAN_ConfigFilter(BSP_FDCAN_INSTANCE, &filter) != HAL_OK)
    {
        return 0U;
    }

    /* 非匹配帧拒绝；远程帧拒绝 */
    if (HAL_FDCAN_ConfigGlobalFilter(BSP_FDCAN_INSTANCE,
                                     FDCAN_REJECT, FDCAN_REJECT,
                                     FDCAN_REJECT_REMOTE,
                                     FDCAN_REJECT_REMOTE) != HAL_OK)
    {
        return 0U;
    }

    if (HAL_FDCAN_ActivateNotification(BSP_FDCAN_INSTANCE,
                                       FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
                                       FDCAN_IT_BUS_OFF |
                                       FDCAN_IT_ERROR_PASSIVE |
                                       FDCAN_IT_ERROR_WARNING,
                                       0U) != HAL_OK)
    {
        return 0U;
    }

    if (HAL_FDCAN_Start(BSP_FDCAN_INSTANCE) != HAL_OK)
    {
        return 0U;
    }

    return 1U;
}

void BSP_FDCAN_Init(void)
{
    s_stats.rx_count = 0U;
    s_stats.tx_ok = 0U;
    s_stats.tx_fail = 0U;
    s_stats.bus_off_count = 0U;
    s_stats.error_passive_count = 0U;
    s_stats.unexpected_rx = 0U;
    s_bus_off = 0U;

    if (fdcan_config() == 0U)
    {
        Error_Handler();
    }
}

void BSP_FDCAN_SetRxCallback(BSP_FDCAN_RxCallback_t cb)
{
    s_rx_callback = cb;
}

uint8_t BSP_FDCAN_Send(uint16_t std_id, const uint8_t data[8])
{
    FDCAN_TxHeaderTypeDef tx = {0};

    if (s_bus_off != 0U)
    {
        s_stats.tx_fail++;
        return 0U;
    }

    if (HAL_FDCAN_GetTxFifoFreeLevel(BSP_FDCAN_INSTANCE) == 0U)
    {
        s_stats.tx_fail++;
        return 0U;
    }

    tx.Identifier           = (uint32_t)std_id;
    tx.IdType               = FDCAN_STANDARD_ID;
    tx.TxFrameType          = FDCAN_DATA_FRAME;
    tx.DataLength           = FDCAN_DLC_BYTES_8;
    tx.ErrorStateIndicator  = FDCAN_ESI_ACTIVE;
    tx.BitRateSwitch        = FDCAN_BRS_OFF;
    tx.FDFormat             = FDCAN_CLASSIC_CAN;
    tx.TxEventFifoControl   = FDCAN_NO_TX_EVENTS;
    tx.MessageMarker        = 0U;

    if (HAL_FDCAN_AddMessageToTxFifoQ(BSP_FDCAN_INSTANCE, &tx, data) != HAL_OK)
    {
        s_stats.tx_fail++;
        return 0U;
    }

    s_stats.tx_ok++;
    return 1U;
}

uint8_t BSP_FDCAN_IsBusOff(void)
{
    return s_bus_off;
}

uint8_t BSP_FDCAN_Recover(void)
{
    (void)HAL_FDCAN_Stop(BSP_FDCAN_INSTANCE);

    if (fdcan_config() == 0U)
    {
        return 0U;
    }

    s_bus_off = 0U;
    return 1U;
}

void BSP_FDCAN_GetStats(BSP_FDCAN_Stats_t *out)
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

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    BSP_FDCAN_Frame_t frame;

    if (hfdcan->Instance != FDCAN1)
    {
        return;
    }

    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U)
    {
        return;
    }

    /* 一次中断可能积压多帧，必须排空 FIFO */
    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U)
    {
        FDCAN_RxHeaderTypeDef rx_header = {0};
        uint8_t rx_data[8] = {0};

        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0,
                                   &rx_header, rx_data) != HAL_OK)
        {
            break;
        }

        if (rx_header.IdType != FDCAN_STANDARD_ID)
        {
            s_stats.unexpected_rx++;
            continue;
        }

        if (rx_header.DataLength != FDCAN_DLC_BYTES_8)
        {
            s_stats.unexpected_rx++;
            continue;
        }

        frame.id = (uint16_t)rx_header.Identifier;
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

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
    if (hfdcan->Instance != FDCAN1)
    {
        return;
    }

    if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) != 0U)
    {
        s_bus_off = 1U;
        s_stats.bus_off_count++;
    }

    if ((ErrorStatusITs & FDCAN_IT_ERROR_PASSIVE) != 0U)
    {
        s_stats.error_passive_count++;
    }
}
