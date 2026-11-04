#include "bsp_time.h"
#include "main.h"
static TIM_HandleTypeDef wall_timer;
bool BspTime_Init(void)
{
 RCC_ClkInitTypeDef clock;uint32_t latency,hz;
 HAL_RCC_GetClockConfig(&clock,&latency);
 hz=HAL_RCC_GetPCLK1Freq();
 if(clock.APB1CLKDivider!=RCC_APB1_DIV1)hz*=2U;
 if(hz<1000000U || hz%1000000U)return false;
 __HAL_RCC_TIM2_CLK_ENABLE();__HAL_DBGMCU_UnFreeze_TIM2();
 wall_timer.Instance=TIM2;wall_timer.Init.Prescaler=hz/1000000U-1U;
 wall_timer.Init.CounterMode=TIM_COUNTERMODE_UP;wall_timer.Init.Period=UINT32_MAX;
 wall_timer.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
 return HAL_TIM_Base_Init(&wall_timer)==HAL_OK && HAL_TIM_Base_Start(&wall_timer)==HAL_OK;
}
uint32_t BspTime_Us(void) {return TIM2->CNT;}
bool BspTime_DebugFrozen(void) {return (DBGMCU->APB1LFZ1 & DBGMCU_APB1LFZ1_DBG_TIM2)!=0;}
