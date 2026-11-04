#include "bsp_jy901s.h"
#include "main.h"
/* Same USART6/PG9 RX/PG14 TX, 9600 8N1 as the reference project.
 * At 960 bytes/s an interrupt ring avoids DMA memory placement entirely. */
static UART_HandleTypeDef uart;
static uint8_t received,ring[256];
static volatile uint32_t head,tail;
void BspBluetooth_RxComplete(UART_HandleTypeDef *handle);
void BspBluetooth_RxError(UART_HandleTypeDef *handle);
void BspJy901s_Init(void)
{
 GPIO_InitTypeDef gpio={0};RCC_PeriphCLKInitTypeDef clock={0};
 clock.PeriphClockSelection=RCC_PERIPHCLK_USART6;
 clock.Usart16ClockSelection=RCC_USART16CLKSOURCE_D2PCLK2;
 if(HAL_RCCEx_PeriphCLKConfig(&clock)!=HAL_OK)Error_Handler();
 __HAL_RCC_USART6_CLK_ENABLE();__HAL_RCC_GPIOG_CLK_ENABLE();
 gpio.Pin=GPIO_PIN_9|GPIO_PIN_14;gpio.Mode=GPIO_MODE_AF_PP;
 gpio.Pull=GPIO_NOPULL;gpio.Speed=GPIO_SPEED_FREQ_LOW;gpio.Alternate=GPIO_AF7_USART6;
 HAL_GPIO_Init(GPIOG,&gpio);
 uart.Instance=USART6;uart.Init.BaudRate=9600;
 uart.Init.WordLength=UART_WORDLENGTH_8B;uart.Init.StopBits=UART_STOPBITS_1;
 uart.Init.Parity=UART_PARITY_NONE;uart.Init.Mode=UART_MODE_TX_RX;
 uart.Init.HwFlowCtl=UART_HWCONTROL_NONE;uart.Init.OverSampling=UART_OVERSAMPLING_16;
 uart.Init.ClockPrescaler=UART_PRESCALER_DIV1;
 if(HAL_UART_Init(&uart)!=HAL_OK)Error_Handler();
 HAL_NVIC_SetPriority(USART6_IRQn,5,0);HAL_NVIC_EnableIRQ(USART6_IRQn);
 if(HAL_UART_Receive_IT(&uart,&received,1)!=HAL_OK)Error_Handler();
}
bool BspJy901s_Read(uint8_t *byte)
{
 if(tail==head)return false;
 *byte=ring[tail&255U];tail++;return true;
}
void USART6_IRQHandler(void){HAL_UART_IRQHandler(&uart);}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *handle)
{
 if(handle!=&uart){BspBluetooth_RxComplete(handle);return;}
 if(head-tail<256U){ring[head&255U]=received;__DMB();head++;}
 (void)HAL_UART_Receive_IT(&uart,&received,1);
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *handle)
{
 if(handle==&uart)(void)HAL_UART_Receive_IT(&uart,&received,1);
 else BspBluetooth_RxError(handle);
}
