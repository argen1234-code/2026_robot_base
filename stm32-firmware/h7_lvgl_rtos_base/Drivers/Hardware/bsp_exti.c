#include "bsp_exti.h"


uint16_t temp1 = 0;

//无消抖不准确
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch(GPIO_Pin)
	{
		case GPIO_PIN_15:
		{
			  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
			
		}break;
		
		
		default:
		{
			
		}break;
	}
}
