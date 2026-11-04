/* ============================================================================
 * app_tasks.h —— 线程表与线程创建接口
 *
 * 线程表 g_app_task[] 是"应用层看到的 RTOS 任务清单"：每个周期任务一条，
 * 记着它的线程 id、名字、周期和心跳计数。OLED 页面直接按这张表打印，
 * 所以这里的顺序就是屏幕上的顺序：MOTOR、OLED、HEALTH、LED。
 *
 * 名字必须全大写：屏幕字形表里只有大写字母、数字和少数符号，没有小写。
 * ==========================================================================*/
#ifndef APP_TASKS_H
#define APP_TASKS_H
#include <stdint.h>
#include "cmsis_os2.h"

/* 线程表下标。用枚举而不是宏，写代码时不容易串行。 */
enum { APP_TASK_MOTOR = 0, APP_TASK_OLED, APP_TASK_HEALTH, APP_TASK_LED, APP_TASK_COUNT };

typedef struct {
 osThreadId_t id;             /* 线程句柄：AppTasks_Start() 成功后才有值，之前是 NULL */
 const char *name;            /* 线程名：同时作为 FreeRTOS 线程名（调试器里能看到） */
 uint32_t period_ms;          /* 周期，仅用于显示/查错，调度由 task_run 里的参数决定 */
 volatile uint32_t heartbeat; /* 心跳：任务自己每跑一圈 +1；volatile 因为会被别的任务读 */
} AppTaskInfo;

/* 全局线程表：定义在 app_tasks.c，OLED 页面与健康检查都会读它。 */
extern AppTaskInfo g_app_task[APP_TASK_COUNT];

/* 创建全部线程。调用时机很关键：
 *   osKernelInitialize() 之后、osKernelStart() 之前（见 Core/Src/freertos.c）。
 * 在这之前调用会因为没有内核对象而失败；在这之后调用则会和已启动的调度器抢资源。 */
void AppTasks_Start(void);
#endif
