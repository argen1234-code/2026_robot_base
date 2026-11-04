#include "bsp_bluetooth.h"
#include "app_config.h"
#include "main.h"
#include <string.h>

/* JDY31 transparent UART, matching the reference's actual USART1 settings.
 * IRQs only move bytes; the MOTOR task parses and transmits without waiting.
 * No DMA or Cache maintenance is needed at 9600 baud. */
static UART_HandleTypeDef uart;
static uint8_t received,rx[256],tx[2048],chunk[192];
static volatile uint32_t rx_head,rx_tail,errors;
static uint32_t tx_head,tx_tail;

void BspBluetooth_Init(void)
{
 GPIO_InitTypeDef gpio={0};RCC_PeriphCLKInitTypeDef clock={0};
 rx_head=rx_tail=errors=tx_head=tx_tail=0;
 clock.PeriphClockSelection=RCC_PERIPHCLK_USART1;
 clock.Usart16ClockSelection=RCC_USART16CLKSOURCE_D2PCLK2;
 if(HAL_RCCEx_PeriphCLKConfig(&clock)!=HAL_OK)Error_Handler();
 __HAL_RCC_USART1_CLK_ENABLE();__HAL_RCC_GPIOA_CLK_ENABLE();
 gpio.Pin=GPIO_PIN_9|GPIO_PIN_10;gpio.Mode=GPIO_MODE_AF_PP;
 gpio.Pull=GPIO_NOPULL;gpio.Speed=GPIO_SPEED_FREQ_LOW;gpio.Alternate=GPIO_AF7_USART1;
 HAL_GPIO_Init(GPIOA,&gpio);
 uart.Instance=USART1;uart.Init.BaudRate=APP_BT_BAUD;
 uart.Init.WordLength=UART_WORDLENGTH_8B;uart.Init.StopBits=UART_STOPBITS_1;
 uart.Init.Parity=UART_PARITY_NONE;uart.Init.Mode=UART_MODE_TX_RX;
 uart.Init.HwFlowCtl=UART_HWCONTROL_NONE;uart.Init.OverSampling=UART_OVERSAMPLING_16;
 uart.Init.ClockPrescaler=UART_PRESCALER_DIV1;
 if(HAL_UART_Init(&uart)!=HAL_OK)Error_Handler();
 HAL_NVIC_SetPriority(USART1_IRQn,5,0);HAL_NVIC_EnableIRQ(USART1_IRQn);
 if(HAL_UART_Receive_IT(&uart,&received,1)!=HAL_OK)Error_Handler();
}
bool BspBluetooth_Read(uint8_t *byte)
{
 if(rx_head==rx_tail)return false;
 *byte=rx[rx_tail&255U];rx_tail++;return true;
}
void BspBluetooth_DiscardRx(void){rx_tail=rx_head;}
uint32_t BspBluetooth_Errors(void){return errors;}
bool BspBluetooth_Send(const char *text)
{
 uint32_t length=(uint32_t)strlen(text),i;
 if(length>sizeof(tx)-(tx_head-tx_tail))return false;
 for(i=0;i<length;i++)tx[(tx_head+i)&2047U]=(uint8_t)text[i];
 tx_head+=length;return true;
}
void BspBluetooth_Service(void)
{
 uint32_t length=tx_head-tx_tail,i;
 if(uart.gState!=HAL_UART_STATE_READY || !length)return;
 if(length>sizeof(chunk))length=sizeof(chunk);
 for(i=0;i<length;i++)chunk[i]=tx[(tx_tail+i)&2047U];
 if(HAL_UART_Transmit_IT(&uart,chunk,(uint16_t)length)==HAL_OK)tx_tail+=length;
}
void USART1_IRQHandler(void){HAL_UART_IRQHandler(&uart);}
/* Called by the shared HAL callbacks in bsp_jy901s.c. */
void BspBluetooth_RxComplete(UART_HandleTypeDef *handle)
{
 if(handle!=&uart)return;
 if(rx_head-rx_tail<sizeof(rx)){rx[rx_head&255U]=received;__DMB();rx_head++;}
 else errors++;
 if(HAL_UART_Receive_IT(&uart,&received,1)!=HAL_OK)errors++;
}
void BspBluetooth_RxError(UART_HandleTypeDef *handle)
{
 if(handle!=&uart)return;
 errors++;
 (void)HAL_UART_Receive_IT(&uart,&received,1);
}
