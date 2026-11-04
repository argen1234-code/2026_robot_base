/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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

/* ============================================================================
 * freertos.c —— RTOS 的接线处（CubeMX 生成 + 少量 USER CODE）
 *
 * 这个文件只干三件事：
 *   1. MX_FREERTOS_Init()：调用 AppTasks_Start() 创建四个应用线程。
 *      调用时机在 main() 里：osKernelInitialize() 之后、osKernelStart() 之前。
 *   2. 栈溢出钩子：任何线程栈写穿 → App_Fatal(APP_FATAL_STACK) 停机。
 *      （configCHECK_FOR_STACK_OVERFLOW=2 时内核会在切换时检查栈边界）
 *   3. 堆耗尽钩子：pvPortMalloc 失败 → App_Fatal(APP_FATAL_HEAP) 停机。
 *
 * 为什么没有互斥量/消息队列：本工程的数据交换只有两处——
 *   · 电机快照：单写者（电机任务）+ seqlock 序号，读取方自己校验一致性；
 *   · CAN 接收：中断里写环形缓冲、任务里读，用短暂关中断保护。
 * 都不需要内核对象，少一层阻塞就少一类优先级反转/死锁风险。
 * ==========================================================================*/
/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_main.h"
#include "app_tasks.h"
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
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 4 */
/* Both hooks record the reason in g_debug.system.fatal and stop with
 * interrupts disabled. The console then refuses to control the board and the
 * value is readable over SWD; nothing is retried or silently recovered. */
void vApplicationStackOverflowHook(xTaskHandle xTask,char *pcTaskName)
{ (void)xTask; (void)pcTaskName; App_Fatal(APP_FATAL_STACK); }

void vApplicationMallocFailedHook(void) { App_Fatal(APP_FATAL_HEAP); }
  /* USER CODE END 4 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* No mutex: the motor snapshot is published with a seqlock, the CAN ring
   * buffer and the I2C state machine use interrupt masking only. */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* USER CODE BEGIN RTOS_THREADS */
  /* One periodic thread per application service, see app_tasks.c. */
  AppTasks_Start();
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}
