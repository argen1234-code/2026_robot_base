#include "touch_800x480.h"
#include "lcd_rgb.h"
#include "main.h"
#include "led.h"
#include "sdram.h"  
#include "tim.h"  
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "gpio.h"
#include "usart.h"
#include "dma.h"
#include "i2c.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "bsp_uart.h"
#include "bsp_bluetooth.h"

#include "lvgl.h"
#include "lv_port_disp_template.h"
#include "lv_port_indev_template.h"
#include "my_lvgl_task.h"


/********************************************** 函数声明 *******************************************/

void SystemClock_Config(void);		// 时钟初始化
void MPU_Config(void);					// MPU配置
void MX_FREERTOS_Init(void);
	
/***************************************************************************************************
*	函 数 名: main
*	入口参数: 无
*	返 回 值: 无
*	函数功能: LTDC驱动屏幕测试
*	说    明: 无
****************************************************************************************************/

/*已使用引脚
PC13(LED),PA15(KEY),PB10,PB11(OLED),PB12 13 ,14 15,  PC4,5(MOTOR_DIR_FR)

PB3     ------> TIM2_CH2
PA5     ------> TIM2_CH1
PB5     ------> TIM3_CH2
PB4 (NJTRST)     ------> TIM3_CH1
PB6     ------> TIM4_CH1           四个编码器
PB7     ------> TIM4_CH2
PH10     ------> TIM5_CH1
PH11     ------> TIM5_CH2

PI6     ------> TIM8_CH2
PI5     ------> TIM8_CH1
PI7     ------> TIM8_CH3          四路pwm
PI2     ------> TIM8_CH4

                  FMC GPIO Configuration
PF0   ------> FMC_A0			PD14   ------> FMC_D0			PC0    ------> FMC_SDNWE
PF1   ------> FMC_A1        PD15   ------> FMC_D1         PG15   ------> FMC_SDNCAS 
PF2   ------> FMC_A2        PD0    ------> FMC_D2         PF11   ------> FMC_SDNRAS
PF3   ------> FMC_A3        PD1    ------> FMC_D3         PH3    ------> FMC_SDNE0  
PF4   ------> FMC_A4        PE7    ------> FMC_D4         PG4    ------> FMC_BA0
PF5   ------> FMC_A5        PE8    ------> FMC_D5         PG5    ------> FMC_BA1
PF12  ------> FMC_A6       	PE9    ------> FMC_D6         PH2    ------> FMC_SDCKE0 
PF13  ------> FMC_A7       	PE10   ------> FMC_D7         PG8    ------> FMC_SDCLK
PF14  ------> FMC_A8       	PE11   ------> FMC_D8         PE1    ------> FMC_NBL1
PF15  ------> FMC_A9       	PE12   ------> FMC_D9         PE0    ------> FMC_NBL0
PG0   ------> FMC_A10       PE13   ------> FMC_D10
PG1   ------> FMC_A11       PE14   ------> FMC_D11
PG2   ------> FMC_A12       PE15   ------> FMC_D12
														PD8    ------> FMC_D13
														PD9    ------> FMC_D14
														PD10   ------> FMC_D15

                                 LTDC
PI15    ------> LTDC_R0		PJ7     ------> LTDC_G0			 PJ12    ------> LTDC_B0
PJ0     ------> LTDC_R1	   PJ8     ------> LTDC_G1        PJ13    ------> LTDC_B1
PJ1     ------> LTDC_R2      PJ9     ------> LTDC_G2	       PJ14    ------> LTDC_B2
PJ2     ------> LTDC_R3      PG10    ------> LTDC_G3	       PJ15    ------> LTDC_B3
PJ3     ------> LTDC_R4      PH15    ------> LTDC_G4	      PK4     ------> LTDC_B5
PJ5     ------> LTDC_R6	   PK1     ------> LTDC_G6	       PK5     ------> LTDC_B6
PJ6     ------> LTDC_R7	   PK2     ------> LTDC_G7	       PK6     ------> LTDC_B7

PI12     ------> LTDC_HSYNC
PI13     ------> LTDC_VSYNC
PK7      ------> LTDC_DE
PI14     ------> LTDC_CLK 
														
PG3(TOUCH SCL),PG7(TOUCH SDA),PI10, PI11,没在cubemx初始化 注意

//---SDMMC1 TF Card (4-bit, 20MHz, images loaded at boot)---
PC8      ------> SDMMC1_D0
PC9      ------> SDMMC1_D1
PC10     ------> SDMMC1_D2 (ex UART4_TX)
PC11     ------> SDMMC1_D3 (ex Motor DIR1, moved to PC4)
PC12     ------> SDMMC1_CK (ex Motor DIR2, moved to PC5)
PD2      ------> SDMMC1_CMD
//---QSPI W25Q64 Flash (quad I/O, 120MHz)---
//PF10    ------> QSPI_CLK
//PG6     ------> QSPI_NCS
//PF8     ------> QSPI_IO0
//PF9     ------> QSPI_IO1
//PF7     ------> QSPI_IO2
//PF6     ------> QSPI_IO3

// PH14 freed (ex UART4_RX)
// Images: FAT32 TF card → f_read → SDRAM → LVGL at boot

PA11     ------> USB_DM
PA12     ------> USB_DP

//---磁力计 QMC5883---
PB8     ------> I2C1_SCL
PB9     ------> I2C1_SDA

//---蓝牙---
PA9     ------> USART1_TX
PA10    ------> USART1_RX

//---GPS---
PA2     ------> USART2_TX
PD6     ------> USART2_RX

//---JY901S IMU---
PG9     ------> USART6_RX
PG14    ------> USART6_TX

*/
int main(void)
{ 	
	MPU_Config();				// MPU配置
	SCB_EnableICache();		// 使能ICache
	SCB_EnableDCache();		// 使能DCache
	HAL_Init();					// 初始化HAL库
	SystemClock_Config();	// 配置系统时钟，主频480MHz
	
		MX_GPIO_Init();           // GPIO pin init (LED, KEY, motor DIR)
	  MX_DMA_Init();
		MX_TIM2_Init();           // Encoder FL
		MX_TIM4_Init();           // Encoder RL
		MX_USART1_UART_Init();    // Bluetooth
		BT_Init();
		uart_init(&huart1, UART_DMA_ToIdle_RX); /* Start Bluetooth RX before RTOS startup. */
		MX_USART2_UART_Init();    // GPS
		MX_USART6_UART_Init();    // JY901S IMU
	//MX_UART4_Init();   // UART4 disabled: PC10 freed for SDMMC1_D2
		MX_TIM3_Init();           // Encoder FR																								
		MX_TIM5_Init();           // Encoder RR
		MX_TIM8_Init();           // 4-channel motor PWM
		MX_I2C1_Init();           // QMC5883 magnetometer
	
	LED_Init();					// 初始化LED引脚
	MX_FMC_Init();				// SDRAM初始化
	MX_USB_DEVICE_Init();		// USB设备初始化

	/* --- USB CDC 回环测试(阻塞RTOS启动) --- */
	/* 电脑通过虚拟串口发什么,32就原样返回 */
//	while (1)
//	{
//		if (usb_rx_flag)
//		{
//			CDC_Transmit_FS(UserRxBufferFS, (uint16_t)usb_rx_len);
//			usb_rx_flag = 0;
//		}
//	}
	/* --- 测试通过后注释掉上面的 while(1) --- */
	


  
	
	  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();
	
	while (1)
	{
		
	}
}
/****************************************************************************************************/
/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 480000000 (CPU Clock)
  *            HCLK(Hz)                       = 240000000 (AXI and AHBs Clock)
  *            AHB Prescaler                  = 2
  *            D1 APB3 Prescaler              = 2 (APB3 Clock  120MHz)
  *            D2 APB1 Prescaler              = 2 (APB1 Clock  120MHz)
  *            D2 APB2 Prescaler              = 2 (APB2 Clock  120MHz)
  *            D3 APB4 Prescaler              = 2 (APB4 Clock  120MHz)
  *            HSE Frequency(Hz)              = 25000000
  *            PLL_M                          = 5
  *            PLL_N                          = 192
  *            PLL_P                          = 2
  *            PLL_Q                          = 2
  *            PLL_R                          = 2
  *            VDD(V)                         = 3.3
  *            Flash Latency(WS)              = 4
  * @param  None
  * @retval None
  */
/****************************************************************************************************/  
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  
  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  
  /* 设置LTDC时钟，这里设置为33MHz，即刷新率在60帧左右，过高或者过低都会造成闪烁 */
  /* LCD clock configuration */
  /* PLL3_VCO Input = HSE_VALUE/PLL3M = 1 Mhz */
  /* PLL3_VCO Output = PLL3_VCO Input * PLL3N = 330 Mhz */
  /* PLLLCDCLK = PLL3_VCO Output/PLL3R = 330/10 = 33 Mhz */
  /* LTDC clock frequency = PLLLCDCLK = 33 Mhz */  
   
      
  PeriphClkInitStruct.PLL3.PLL3M = 25;
  PeriphClkInitStruct.PLL3.PLL3N = 330;
  PeriphClkInitStruct.PLL3.PLL3P = 2;
  PeriphClkInitStruct.PLL3.PLL3Q = 2;
  PeriphClkInitStruct.PLL3.PLL3R = 10;
  PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_0;
  PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOMEDIUM;
  PeriphClkInitStruct.PLL3.PLL3FRACN = 0;
  
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC|RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_FMC|RCC_PERIPHCLK_SDMMC;               
  PeriphClkInitStruct.FmcClockSelection = RCC_FMCCLKSOURCE_D1HCLK;
  PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
	PeriphClkInitStruct.SdmmcClockSelection = RCC_SDMMCCLKSOURCE_PLL;    // PLL1_Q=480MHz
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}


//	配置MPU
//
void MPU_Config(void)
{
	MPU_Region_InitTypeDef MPU_InitStruct;

	HAL_MPU_Disable();		// 先禁止MPU
	
	MPU_InitStruct.Enable 				= MPU_REGION_ENABLE;
	MPU_InitStruct.BaseAddress 		= LCD_MemoryAdd;	
	MPU_InitStruct.Size 					= MPU_REGION_SIZE_512KB;			// 片内SRAM
	MPU_InitStruct.AccessPermission 	= MPU_REGION_FULL_ACCESS;
	MPU_InitStruct.IsBufferable 		= MPU_ACCESS_NOT_BUFFERABLE;
	MPU_InitStruct.IsCacheable 		= MPU_ACCESS_NOT_CACHEABLE;
	MPU_InitStruct.IsShareable 		= MPU_ACCESS_NOT_SHAREABLE;
	MPU_InitStruct.Number 				= MPU_REGION_NUMBER0;
	MPU_InitStruct.TypeExtField 		= MPU_TEX_LEVEL0;
	MPU_InitStruct.SubRegionDisable 	= 0x00;
	MPU_InitStruct.DisableExec 		= MPU_INSTRUCTION_ACCESS_ENABLE;

	HAL_MPU_ConfigRegion(&MPU_InitStruct);
	
	MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
	MPU_InitStruct.BaseAddress      = SDRAM_BANK_ADDR;
	MPU_InitStruct.Size             = MPU_REGION_SIZE_32MB;			// SDRAM
	MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
	MPU_InitStruct.IsBufferable     = MPU_ACCESS_NOT_BUFFERABLE;
	MPU_InitStruct.IsCacheable      = MPU_ACCESS_CACHEABLE;
	MPU_InitStruct.IsShareable      = MPU_ACCESS_NOT_SHAREABLE;
	MPU_InitStruct.Number           = MPU_REGION_NUMBER1;
	MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL0;
	MPU_InitStruct.SubRegionDisable = 0x00;
	MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_ENABLE;

	HAL_MPU_ConfigRegion(&MPU_InitStruct);
	
	HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);	// 使能MPU
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}



/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM17 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM17)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

