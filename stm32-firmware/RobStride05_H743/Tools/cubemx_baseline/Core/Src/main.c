/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "fdcan.h"
#include "i2c.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_main.h"
#include "app_tasks.h"
#include "app_debug.h"
  /* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ============================================================================
 * 启动顺序总览（读代码时按这个顺序看）
 *
 *   上电/复位
 *     └ SystemInit()            ← 启动汇编调用，含比较器、FPU 使能
 *     └ main()
 *        1. 设 VTOR = __Vectors（必须在开中断之前）        boot_stage=1
 *        2. HAL_Init()            ← 时基改用 TIM6，1 ms     boot_stage=2
 *        3. SystemClock_Config()  ← 25 MHz HSE → 480 MHz    boot_stage=3
 *        4. MX_GPIO_Init / MX_FDCAN1_Init / MX_I2C2_Init    4 / 5 / 6
 *        5. App_Init()            ← TIM2、CAN 启动、灯带     8
 *        6. osKernelInitialize() → 创建 4 线程 → osKernelStart
 *        7. boot_stage=9、fatal=0 之后上位机才允许控制
 *
 * 任何一步失败都走 App_Fatal()：把原因写进 g_debug.system.fatal 并关中断停机。
 * 没有看门狗自动重启，也没有"失败了继续跑"的分支。
 *
 * 两个容易混的时基：
 *   HAL_GetTick()  ← TIM6 中断累加的 1 ms 计数，应用层（电机超时/斜坡）用它
 *   osKernelGetTickCount() ← SysTick 驱动的 RTOS 节拍，调度用
 * 它们互相独立、都从 0 开始，数值接近但不等，不要混用。
 * ==========================================================================*/

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* ======================================================================
   * 启动第 0 步：把向量表指针（VTOR）指到本固件
   *
   * H7 的地址 0 是 ITCM，不是内部 Flash 的别名（这点和 F4 不同）。
   * 复位后 VTOR=0，如果不在开中断之前改过来，第一个中断（HAL_Init 里
   * 打开的 TIM6 1 ms tick）就会去 ITCM 取向量 → 跳到全 0 的地址 → 跑飞。
   * CubeMX 生成的 system_stm32h7xx.c 只在定义了 USER_VECT_TAB_ADDRESS
   * 时才写 VTOR，本工程没定义，所以必须在这里显式设置。
   * ====================================================================== */
  extern const uint32_t __Vectors[];
  SCB->VTOR = (uint32_t)__Vectors;
  __DSB(); /* 等写入真正生效 */
  __ISB(); /* 清流水线，保证后面的取指用新向量表 */
  g_debug.system.boot_stage = 1U; /* 启动阶段标记：上位机/调试器靠它定位卡在哪 */
  g_debug.system.vtor = SCB->VTOR; /* 只读回读一份，现场证明 VTOR 正确 */
  /* ======================================================================
   * 关于 Cache：本镜像从头到尾都没有开启 I-Cache/D-Cache，也没有任何
   * clean/invalidate 调用。硬件复位后 CCR.IC/CCR.DC 本来就是 0，所以
   * 不需要"关缓存"，更不需要对 DTCM 地址做 cache 维护（那正是旧版一上电
   * 就 HardFault 的原因，见 Docs/08_Cache为什么会HardFault.md）。
   * 这里只把 CCR 抄一份到 g_debug，用来事后证明"运行时确实没开"。
   * ====================================================================== */
  g_debug.system.ccr = SCB->CCR;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the TIM6 time base. */
  g_debug.system.boot_stage = 2U;
  if (HAL_Init() != HAL_OK) App_Fatal(APP_FATAL_HAL);

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  g_debug.system.boot_stage = 3U;
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  g_debug.system.boot_stage = 4U; /* 4 = GPIO（灯带/板载 LED 等） */
  MX_GPIO_Init();
  g_debug.system.boot_stage = 5U; /* 5 = FDCAN1（PB8/PB9，1 Mbps 经典 CAN） */
  MX_FDCAN1_Init();
  g_debug.system.boot_stage = 6U; /* 6 = I2C2（PB10/PB11，SSD1306） */
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  /* App_Init()：BSP 时基 TIM2、启动 CAN（含初始 STOP 帧）、初始化灯带。
   * 它内部把 boot_stage 置 8。注意此刻调度器还没启动，所以这里不能有任何
   * 会阻塞或依赖 RTOS 的调用。 */
  App_Init(); /* BSP time base, CAN start and optional WS2812; sets stage 8. */
  g_debug.system.boot_stage = 9U; /* 9 = 就绪；上位机只接受 9 且 fatal=0 */
  /* 初始化内核对象池 → 创建四个线程 → 启动调度器。
   * 顺序不能换：osKernelInitialize 必须在 osThreadNew 之前，
   * osKernelStart 之后 main() 的栈就不再被使用（控制权交给调度器）。 */
  if (osKernelInitialize() != osOK) App_Fatal(APP_FATAL_KERNEL);
  MX_FREERTOS_Init(); /* Creates the application threads, see freertos.c. */
  (void)osKernelStart();
  /* 正常情况下 osKernelStart 永不返回；返回就说明调度器没接管（堆不足等），
   * 记 fatal 后停机，绝不"退回到 while(1) 凑合跑"。 */
  App_Fatal(APP_FATAL_KERNEL_START); /* Any return is a failed handoff. */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  if (HAL_GetREVID() != REV_ID_V) Error_Handler(); /* 480 MHz is Rev.V only. */
  if (HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY) != HAL_OK) Error_Handler();
  __HAL_RCC_SYSCFG_CLK_ENABLE();

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  { uint32_t start=HAL_GetTick();
    while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
      if(HAL_GetTick()-start>100U)Error_Handler();
    }
  }

  /* Rev.V VOS0 requires the LDO and SYSCFG overdrive. Settle at VOS1 first;
   * the HAL macro then enables ODEN. Raise clocks only after VOSRDY. */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
  { uint32_t start=HAL_GetTick();
    while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
      if(HAL_GetTick()-start>100U)Error_Handler();
    }
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  /* HSE 25 / M5 * N192 = 960 MHz VCO; P2 = CPU480, Q12 = CAN80.
   * HCLK240/APBx120 are the documented Rev.V VOS0 maxima (DS12110). */
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 12;
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
}

/* USER CODE BEGIN 4 */
/**
  * @brief  TIM6 的周期中断回调 —— HAL 的 1 ms 心跳就在这里累加
  *
  * 为什么要写这个函数：本工程的 HAL 时基被改成 TIM6（而不是 SysTick），
  * 因为 SysTick 已经被 FreeRTOS 端口占用（它靠 SysTick 做任务调度节拍）。
  * HAL 的 TIM6 中断服务函数最后会调用这个弱回调，如果不实现它，
  * uwTick 永远不增加 → HAL_GetTick() 恒为 0 → 所有超时判断全部失效。
  *
  * 注意：这里只做一次自增，不能调用任何 RTOS API（TIM6 中断优先级是 15，
  * 属于"可被临界区屏蔽"的范围，但也没必要在这里碰 RTOS）。
  *
  * @param  htim : 触发回调的定时器句柄
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  App_Fatal(APP_FATAL_HAL);
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  App_AssertFailed((const char *)file, line);
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
