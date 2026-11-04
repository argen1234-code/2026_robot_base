#include "bsp_i2c.h"
#include "i2c.h"
#include <string.h>
static uint8_t tx[129]; /* Static lifetime; interrupt transfer, no DMA. */
static volatile uint32_t done,error;
static uint32_t started,last_error,recovery_phase,recovery_due;
static bool active,recovering;
static void quiesce(void)
{
 HAL_NVIC_DisableIRQ(I2C2_EV_IRQn);HAL_NVIC_DisableIRQ(I2C2_ER_IRQn);
 __HAL_I2C_DISABLE_IT(&hi2c2,I2C_IT_TXI|I2C_IT_RXI|I2C_IT_ADDRI|I2C_IT_NACKI|I2C_IT_STOPI|I2C_IT_TCI|I2C_IT_ERRI);
 __HAL_I2C_DISABLE(&hi2c2);
 __HAL_RCC_I2C2_FORCE_RESET();__HAL_RCC_I2C2_RELEASE_RESET();
 HAL_NVIC_ClearPendingIRQ(I2C2_EV_IRQn);HAL_NVIC_ClearPendingIRQ(I2C2_ER_IRQn);
 active=false;
}
bool BspI2c_TransmitBegin(uint8_t address,const uint8_t *data,uint16_t size)
{
 HAL_StatusTypeDef r;
 if(active || recovering || size>sizeof(tx) || !size)return false;
 memcpy(tx,data,size);done=error=0;
 started=HAL_GetTick();active=true;
 r=HAL_I2C_Master_Transmit_IT(&hi2c2,(uint16_t)(address<<1),tx,size);
 if(r==HAL_OK)return true;
 last_error=HAL_I2C_GetError(&hi2c2)|(r==HAL_BUSY?0x80000000UL:0x40000000UL);
 quiesce();return false;
}
uint32_t BspI2c_Poll(void)
{
 if(!active)return I2C_IDLE;
 if(done && !error){active=false;return I2C_OK;}
 if(!error && HAL_GetTick()-started<30U)return I2C_BUSY;
 last_error=HAL_I2C_GetError(&hi2c2)|(error?0:0x40000000UL);
 quiesce();return I2C_FAIL;
}
void BspI2c_RecoverBegin(void)
{
 GPIO_InitTypeDef g={0};
 quiesce();(void)HAL_I2C_DeInit(&hi2c2);
 __HAL_RCC_GPIOB_CLK_ENABLE();g.Pin=GPIO_PIN_10|GPIO_PIN_11;
 g.Mode=GPIO_MODE_OUTPUT_OD;g.Pull=GPIO_PULLUP;g.Speed=GPIO_SPEED_FREQ_LOW;
 HAL_GPIO_WritePin(GPIOB,g.Pin,GPIO_PIN_SET);HAL_GPIO_Init(GPIOB,&g);
 recovery_phase=0;recovery_due=HAL_GetTick()+1U;recovering=true;
}
uint32_t BspI2c_RecoverPoll(void)
{
 bool released;
 if(!recovering)return I2C_IDLE;
 if((int32_t)(HAL_GetTick()-recovery_due)<0)return I2C_BUSY;
 recovery_due=HAL_GetTick()+1U;
 if(recovery_phase<18) {
  /* At most nine clock pulses, no waiting for a stuck SCL. */
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,(recovery_phase&1U)?GPIO_PIN_SET:GPIO_PIN_RESET);
 }else if(recovery_phase==18)HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_RESET);
 else if(recovery_phase==19)HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_SET);
 else if(recovery_phase==20)HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_SET);
 else {
  released=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_10)==GPIO_PIN_SET && HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_11)==GPIO_PIN_SET;
  recovering=false;
  /* Init restores AF4 and IRQs via MSP. These HAL Init/filter calls do not
   * wait for a slave response. SCL/SDA stuck => FAIL, bounded backoff in app. */
  if(HAL_I2C_Init(&hi2c2)!=HAL_OK ||
     HAL_I2CEx_ConfigAnalogFilter(&hi2c2,I2C_ANALOGFILTER_ENABLE)!=HAL_OK ||
     HAL_I2CEx_ConfigDigitalFilter(&hi2c2,0)!=HAL_OK)return I2C_FAIL;
  return released?I2C_OK:I2C_FAIL;
 }
 recovery_phase++;return I2C_BUSY;
}
uint32_t BspI2c_LastError(void){return last_error;}
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *h){if(h==&hi2c2)done=1;}
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *h){if(h==&hi2c2)error=1;}
