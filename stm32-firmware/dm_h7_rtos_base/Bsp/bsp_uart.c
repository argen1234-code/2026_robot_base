#include "bsp_uart.h"
#include "main.h"
#include "usart.h"
#include <string.h>

#define BSP_BT_RX_SIZE          512U
#define BSP_BT_TX_SIZE          160U
#define BSP_TELEMETRY_TX_SIZE   192U

/* DMA1 无法访问 DTCM，DMA 缓冲必须放在 RAM_D1 */
#if defined(__GNUC__)
#define BSP_DMA_RX_ATTR     __attribute__((section(".dma_buffer"), aligned(32)))
#define BSP_DMA_TX_ATTR     __attribute__((section(".dma_buffer"), aligned(32)))
#else
/* Keil AC5：没有 .dma_buffer 分散加载项时，直接放到 D1 SRAM 固定地址 */
#define BSP_DMA_RX_ATTR     __attribute__((at(0x24000000)))
#define BSP_DMA_TX_ATTR     __attribute__((at(0x24001000)))
#endif

static uint8_t s_bt_rx_dma[BSP_BT_RX_SIZE] BSP_DMA_RX_ATTR;
static uint8_t s_bt_tx_dma[BSP_BT_TX_SIZE] BSP_DMA_TX_ATTR;

static uint8_t s_telemetry_tx_buf[BSP_TELEMETRY_TX_SIZE];

static BSP_UART_RxCallback_t s_bt_rx_callback = NULL;
static volatile uint16_t s_bt_last_pos = 0U;
static volatile uint8_t s_bt_tx_busy = 0U;
static volatile uint8_t s_telemetry_tx_busy = 0U;
static uint8_t s_bt_rx_started = 0U;

static void bt_deliver(const uint8_t *data, uint16_t len)
{
    if ((s_bt_rx_callback != NULL) && (len > 0U))
    {
        s_bt_rx_callback(data, len);
    }
}

void BSP_UART_SetBtRxCallback(BSP_UART_RxCallback_t cb)
{
    s_bt_rx_callback = cb;
}

void BSP_UART_Init(void)
{
    if (s_bt_rx_started != 0U)
    {
        return;
    }

    s_bt_last_pos = 0U;
    s_bt_tx_busy = 0U;
    s_telemetry_tx_busy = 0U;

    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart1, s_bt_rx_dma,
                                     BSP_BT_RX_SIZE) == HAL_OK)
    {
        s_bt_rx_started = 1U;
    }
}

uint8_t BSP_UART_BtSend(const uint8_t *data, uint16_t len)
{
    if ((data == NULL) || (len == 0U) || (len > BSP_BT_TX_SIZE))
    {
        return 0U;
    }

    if ((s_bt_tx_busy != 0U) || (huart1.gState != HAL_UART_STATE_READY))
    {
        return 0U;
    }

    memcpy(s_bt_tx_dma, data, len);

    s_bt_tx_busy = 1U;
    if (HAL_UART_Transmit_DMA(&huart1, s_bt_tx_dma, len) != HAL_OK)
    {
        s_bt_tx_busy = 0U;
        return 0U;
    }

    return 1U;
}

uint8_t BSP_UART_BtTxBusy(void)
{
    return s_bt_tx_busy;
}

uint8_t BSP_UART_TelemetrySend(const uint8_t *data, uint16_t len)
{
    if ((data == NULL) || (len == 0U) || (len > BSP_TELEMETRY_TX_SIZE))
    {
        return 0U;
    }

    if ((s_telemetry_tx_busy != 0U) ||
        (huart10.gState != HAL_UART_STATE_READY))
    {
        return 0U;
    }

    memcpy(s_telemetry_tx_buf, data, len);

    s_telemetry_tx_busy = 1U;
    if (HAL_UART_Transmit_IT(&huart10, s_telemetry_tx_buf, len) != HAL_OK)
    {
        s_telemetry_tx_busy = 0U;
        return 0U;
    }

    return 1U;
}

uint8_t BSP_UART_TelemetryTxBusy(void)
{
    return s_telemetry_tx_busy;
}

/* ------------------------------------------------------------------------- */
/* HAL 回调                                                                   */
/* ------------------------------------------------------------------------- */

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    uint16_t pos;

    if (huart->Instance != USART1)
    {
        return;
    }

    pos = (Size > BSP_BT_RX_SIZE) ? BSP_BT_RX_SIZE : Size;

    if (pos == BSP_BT_RX_SIZE)
    {
        /* 刚好写满（DMA TC），下一字节从 0 开始 */
        if (pos > s_bt_last_pos)
        {
            bt_deliver(&s_bt_rx_dma[s_bt_last_pos],
                       (uint16_t)(pos - s_bt_last_pos));
        }
        s_bt_last_pos = 0U;
    }
    else if (pos > s_bt_last_pos)
    {
        bt_deliver(&s_bt_rx_dma[s_bt_last_pos],
                   (uint16_t)(pos - s_bt_last_pos));
        s_bt_last_pos = pos;
    }
    else if (pos < s_bt_last_pos)
    {
        /* 环形回卷 */
        bt_deliver(&s_bt_rx_dma[s_bt_last_pos],
                   (uint16_t)(BSP_BT_RX_SIZE - s_bt_last_pos));
        bt_deliver(&s_bt_rx_dma[0], pos);
        s_bt_last_pos = pos;
    }
    else
    {
        /* 没有新字节 */
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART10)
    {
        s_telemetry_tx_busy = 0U;
        return;
    }

    if (huart->Instance != USART1)
    {
        return;
    }

    s_bt_tx_busy = 0U;
    s_bt_last_pos = 0U;
    (void)HAL_UART_AbortReceive(&huart1);

    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart1, s_bt_rx_dma,
                                     BSP_BT_RX_SIZE) == HAL_OK)
    {
        s_bt_rx_started = 1U;
    }
    else
    {
        s_bt_rx_started = 0U;
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        s_bt_tx_busy = 0U;
    }
    else if (huart->Instance == USART10)
    {
        s_telemetry_tx_busy = 0U;
    }
}
