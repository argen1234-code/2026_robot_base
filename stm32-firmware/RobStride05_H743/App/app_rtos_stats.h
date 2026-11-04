/* ============================================================================
 * app_rtos_stats.h —— 把"调度器到底怎么分 CPU"这件事读出来
 *
 * 屏幕上的 MOTO/OLED/HEALTH/LED 四行里的三个数字都来自这里：
 *   状态（RUN/BLK/...）  ← 调度器当前认为它在干什么
 *   剩余栈              ← FreeRTOS 记录的历史最小剩余（高水位）
 *   CPU 占用百分比       ← 本次采样窗口内它实际跑了多久
 * 另外还有一个全局的"上电以来上下文切换次数"，用来直观感受调度器有多忙。
 *
 * 这些数据全部来自 FreeRTOS 内核自己的账本（运行时统计 + uxTaskGetSystemState），
 * 不是应用自己估的，所以在排查"某个任务是不是没跑/跑太久"时是可信的。
 * ==========================================================================*/
#ifndef APP_RTOS_STATS_H
#define APP_RTOS_STATS_H
#include <stdint.h>
#include "cmsis_os2.h"

/* 内部缓存数组大小：必须 >= 系统里的任务总数（应用 4 个 + IDLE + Tmr Svc = 6），
 * 否则 FreeRTOS 的 uxTaskGetSystemState() 会命中 configASSERT。留 10 是余量。 */
#define APP_RTOS_STATS_TASKS 10U

typedef struct {
 const char *name;      /* 线程名（FreeRTOS 里注册的那个） */
 const char *state;     /* RUN 运行 / RDY 就绪 / BLK 阻塞 / SUS 挂起 / DEL 已删除 / ??? */
 uint32_t stack_free_b; /* 剩余栈字节数（历史最小值，越跑越小说明栈开小了） */
 uint32_t cpu_permille; /* 本采样窗口的 CPU 占用，千分数：123 表示 12.3% */
} AppRtosTask;

/* ---- 下面两个是 FreeRTOS 端口的钩子，在 FreeRTOSConfig.h 里被引用 -------------
 * 作用：给内核提供一个"高分辨率时间戳"，内核每次任务切换读一次，
 * 用来累计每个任务的运行时间。这里用 Cortex-M7 的 DWT 周期计数器（480 MHz）。 */
void App_RtosStatsTimerInit(void);
uint32_t App_RtosStatsCounter(void);

/* 上下文切换计数：由 FreeRTOSConfig.h 的 traceTASK_SWITCHED_IN() 宏在每次
 * 任务切换时 +1（发生在调度器内部、中断已关，所以只能是纯自增，不能调 RTOS API）。 */
extern volatile uint32_t g_rtos_switch_count;

/* 采样一次：按 ids[] 给出的线程填 out[]，并通过 switch_count 返回上电以来的
 * 切换总次数。只能在任务上下文调用（内部会短暂挂起调度器），耗时约 100 µs。
 * 第一次调用没有历史数据，CPU 占用会全是 0，第二次起才有效。 */
void App_RtosSample(const osThreadId_t *ids,AppRtosTask *out,uint32_t count,
                    uint32_t *switch_count);
#endif
