#ifndef APP_MAIN_H
#define APP_MAIN_H
#include <stdint.h>

/* Application entry points for the FreeRTOS build. The motor/OLED/LED work is
 * the same state machine as the polling HAL project; only the caller changed:
 * one periodic RTOS task per service, see app_tasks.c.
 *
 * App_Init()            main context, before osKernelStart().
 * AppMotor_Loop()       motor task,       APP_MOTOR_TASK_PERIOD_MS.
 * AppOled_Loop()        OLED task,        APP_OLED_TASK_PERIOD_MS.
 * AppLed_Loop()         WS2812 task,      APP_LED_TASK_PERIOD_MS.
 * AppHealth_Loop()      health task,      APP_HEALTH_TASK_PERIOD_MS.
 * None of these functions blocks, allocates or waits on a peripheral. */
void App_Init(void);
void AppMotor_Loop(void);
void AppOled_Loop(void);
void AppLed_Loop(void);
void AppHealth_Loop(void);

/* Recorded in g_debug.system.fatal; nonzero stops all application work.
 * The console refuses to connect while fatal is nonzero, so the value is a
 * diagnosis aid read over SWD, never a recovery mechanism. */
enum {
 APP_FATAL_TIMEBASE = 1,   /* BspTime_Init (TIM2) failed. */
 APP_FATAL_HAL = 2,        /* HAL_Init, clock tree or peripheral init failed. */
 APP_FATAL_KERNEL = 3,     /* osKernelInitialize failed. */
 APP_FATAL_TASK = 4,       /* osThreadNew returned NULL. */
 APP_FATAL_HEAP = 5,       /* vApplicationMallocFailedHook: FreeRTOS heap empty. */
 APP_FATAL_STACK = 6,      /* vApplicationStackOverflowHook: task stack exhausted. */
 APP_FATAL_ASSERT = 7,     /* FreeRTOS configASSERT or HAL assert_param. */
 APP_FATAL_KERNEL_START = 8,/* osKernelStart returned instead of scheduling. */
 APP_FATAL_HARDFAULT = 9,
 APP_FATAL_MEMMANAGE = 10,
 APP_FATAL_BUSFAULT = 11,
 APP_FATAL_USAGEFAULT = 12,
 APP_FATAL_NMI = 13
};
void App_Fatal(uint32_t reason);

/* Landing point for the FreeRTOS configASSERT and the HAL assert_param. The
 * source location is kept in two plain globals for the Keil Watch window only;
 * it is outside g_debug so the console ABI does not change. */
void App_AssertFailed(const char *file, uint32_t line);
extern const char *g_assert_file;
extern volatile uint32_t g_assert_line;
#endif
