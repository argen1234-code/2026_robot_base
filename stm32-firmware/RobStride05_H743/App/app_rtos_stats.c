/* ============================================================================
 * app_rtos_stats.c —— 调度器统计（CPU 占用 / 状态 / 剩余栈 / 切换次数）
 *
 * 目的：让屏幕和上位机都能看到"谁在跑、跑了多少、栈够不够"。
 *
 * 两条数据来源，都是 FreeRTOS 内核自己维护的账本：
 *
 * 1) 运行时统计（run-time statistics）
 *    FreeRTOSConfig.h 里打开 configGENERATE_RUN_TIME_STATS 后，内核在**每次
 *    任务切换**时读一次 portGET_RUN_TIME_COUNTER_VALUE()，把两次读数的差累计
 *    到"刚被切出去的那个任务"的 ulRunTimeCounter 里。
 *    我们把这个计数器接到 Cortex-M7 的 DWT->CYCCNT（CPU 周期计数，480 MHz），
 *    所以每个任务的运行时间精度约 2.08 ns，而且几乎不占 CPU（读一个寄存器）。
 *
 * 2) uxTaskGetSystemState()
 *    一次性把"系统里所有任务的快照"取出来：名字、当前状态、优先级、栈高水位、
 *    ulRunTimeCounter。它是 FreeRTOS 的公开 API（需要 configUSE_TRACE_FACILITY=1）。
 *    它内部会短暂挂起调度器，耗时约 100 µs —— 在 5 Hz 的刷屏任务里调用没问题，
 *    绝不要放进电机任务的 2 ms 循环里。
 *
 * 为什么不用 vTaskGetRunTimeStats()？那个函数输出的是格式化字符串，还要自己
 * 算百分比、要额外缓冲区；我们只需要数字，自己算更省。
 * ==========================================================================*/

#include "app_rtos_stats.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include <string.h>

/* 上下文切换计数：由 FreeRTOSConfig.h 里的 traceTASK_SWITCHED_IN() 宏在每次
 * 任务切换时 +1。它运行在调度器内部（中断已关），所以这里只能是一个纯自增的
 * volatile 变量——不能调 RTOS API，也不能做任何 Cache 维护。 */
volatile uint32_t g_rtos_switch_count;

/* ---- DWT 周期计数器：给内核当"高分辨率时间戳"用 -----------------------------
 * CYCCNT 随 CPU 主频自增（480 MHz），32 位，约 8.95 秒回绕一圈。
 * 回绕不是问题：内核只做"两次读数相减"，无符号 32 位减法的结果在回绕时依然正确
 * （前提是两次读数间隔远小于 8.95 秒，我们这里是 200 ms 的刷屏窗口）。
 * TRCENA 是 DWT 的总开关，某些调试器复位后会清掉，所以这里显式置位。 */
void App_RtosStatsTimerInit(void)
{
 CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; /* 打开调试跟踪总开关（DWT 的前提） */
 DWT->CYCCNT = 0U;                               /* 从 0 开始计数 */
 DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;            /* 使能周期计数器 */
}
uint32_t App_RtosStatsCounter(void)
{
 return DWT->CYCCNT; /* 内核每次任务切换进来读一次 */
}

/* 上一次采样时每个任务的累计运行周期。按"任务句柄"匹配，
 * 因为 uxTaskGetSystemState 返回的顺序不保证稳定。
 * primed=0 表示这条记录刚建立、还没有上一次的值（首帧不算占用）。 */
#define STATS_CACHE APP_RTOS_STATS_TASKS
static struct {
 TaskHandle_t handle;
 uint32_t counter;
 uint32_t primed;
} s_previous[STATS_CACHE];
static uint32_t s_previous_count;

/* 把 FreeRTOS 的任务状态翻成 3 个字母，屏幕才放得下。
 * 说明：本工程四个周期任务平时都在 osDelayUntil 里等 → BLK（阻塞）是常态；
 * 正在画屏的那个（OLED）采样时一定是 RUN。 */
static const char *state3(eTaskState state)
{
 switch(state) {
 case eRunning:   return "RUN";
 case eReady:     return "RDY";
 case eBlocked:   return "BLK";
 case eSuspended: return "SUS";
 case eDeleted:   return "DEL";
 default:         return "???";
 }
}

/* ----------------------------------------------------------------------------
 * App_RtosSample —— 采样一次，填 out[0..count)，并返回切换总次数
 *
 * 算法：
 *   1. 取全系统任务快照（含 IDLE 和定时器服务任务）；
 *   2. 对每个任务，用本次 ulRunTimeCounter 减去上次的，得到"这个窗口内它跑了
 *      多少周期"；第一次见到的任务只登记、不计入（delta=0）；
 *   3. 把所有 delta 相加 = 这个窗口的真实流逝时间（因为总有任务在跑）；
 *   4. 应用任务的占用 = 自己的 delta / 总和；IDLE 的占用就是空闲比例，
 *      所以四个应用任务之和 ≈ 100% − 空闲。
 *
 * 注意：这个总和包含了 IDLE 和 Tmr Svc，所以百分比是"相对于全部 CPU 时间"，
 * 不会出现"四个任务加起来 100%"的误导。
 * --------------------------------------------------------------------------*/
void App_RtosSample(const osThreadId_t *ids,AppRtosTask *out,uint32_t count,
                    uint32_t *switch_count)
{
 TaskStatus_t status[STATS_CACHE];
 uint32_t delta[STATS_CACHE];
 uint32_t total=0,idle=0,i,j;
 UBaseType_t tasks;
 if(count>APP_RTOS_STATS_TASKS)count=APP_RTOS_STATS_TASKS;
 /* 取快照：数组必须够大，否则内核会 configASSERT（fatal=7）。 */
 tasks=uxTaskGetSystemState(status,(UBaseType_t)STATS_CACHE,NULL);
 for(i=0U;i<(uint32_t)tasks;i++) {
  delta[i]=0U;
  /* 在缓存里找这个任务上一次的记录（按句柄匹配）。 */
  for(j=0U;j<s_previous_count;j++) {
   if(s_previous[j].handle==status[i].xHandle)break;
  }
  /* 第一次见到它：登记一条新记录，本次不计时间。 */
  if(j==s_previous_count && s_previous_count<STATS_CACHE) {
   s_previous[j].handle=status[i].xHandle;
   s_previous[j].counter=status[i].ulRunTimeCounter;
   s_previous[j].primed=0U;
   s_previous_count++;
  }
  if(j<s_previous_count) {
   /* 有历史值才算增量；无符号减法天然处理 32 位回绕。 */
   if(s_previous[j].primed) {
    delta[i]=status[i].ulRunTimeCounter-s_previous[j].counter;
   }
   s_previous[j].counter=status[i].ulRunTimeCounter;
   s_previous[j].primed=1U;
  }
  total+=delta[i]; /* 窗口内的总 CPU 时间 */
  if(status[i].pcTaskName!=NULL && strcmp(status[i].pcTaskName,"IDLE")==0) {
   idle=delta[i]; /* 记下空闲时间（当前只用于统计，未显示） */
  }
 }
 /* 按调用者给的 id 列表填结果；找不到的（比如创建失败）填 "?"。 */
 for(i=0U;i<count;i++) {
  out[i].name="?";
  out[i].state="???";
  out[i].stack_free_b=0U;
  out[i].cpu_permille=0U;
  for(j=0U;j<(uint32_t)tasks;j++) {
   if((void *)status[j].xHandle==(void *)ids[i]) {
    out[i].name=status[j].pcTaskName;
    out[i].state=state3(status[j].eCurrentState);
    /* 高水位记录的是"历史最少剩余"，单位是字，乘 4 换成字节 */
    out[i].stack_free_b=(uint32_t)status[j].usStackHighWaterMark*sizeof(StackType_t);
    if(total!=0U)out[i].cpu_permille=(delta[j]*1000U)/total;
    break;
   }
  }
 }
 /* 上电以来的上下文切换总数：本工程负载下约 1800~2000 次/秒，
  * 屏幕取低 5 位显示，看起来每帧都在跳。 */
 if(switch_count!=NULL)*switch_count=g_rtos_switch_count;
 (void)idle; /* 空闲占比 = 1000 − 各任务占用之和，暂不单独显示 */
}
