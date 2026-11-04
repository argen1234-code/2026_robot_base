/* ============================================================================
 * app_tasks.c —— 四个周期任务的定义与创建
 *
 * 这个文件是整个 RTOS 版"骨架"：单线程 HAL 工程里是一个 while(1) 里顺序调用
 * AppMotor_Service()/oled_service()/AppLed_Service()/健康检查；RTOS 版把它们
 * 拆成四个独立线程，各自有自己的周期、优先级和栈。
 *
 * 线程表 g_app_task[] 同时被 OLED 页面用于显示（线程名/状态/CPU/栈），
 * 所以这里的顺序就是屏幕上的顺序：MOTOR、OLED、HEALTH、LED。
 *
 * 设计要点（改这里之前先看）：
 *  1. 每个服务函数都必须是"非阻塞"的：不许 osDelay 等外设、不许死等标志位。
 *     它们只做状态机推进并把耗时的 I/O 交给中断/DMA。
 *  2. 周期用 osDelayUntil 做"绝对时刻"调度，不是 osDelay(period)。
 *     osDelay(n) 是"从现在起睡 n 拍"，任务执行时间会把周期越拖越长；
 *     osDelayUntil(next) 是"睡到第 next 拍"，周期稳定。
 *  3. 一旦某次执行超过周期，不做"补跑"，而是把下一次对齐到当前时刻+1 拍。
 *     原因：电机任务里 20 ms 斜坡、100 ms 参数超时、150 ms 反馈超时都是按
 *     HAL_GetTick() 自己算的，如果这里为了"补上落后的节拍"连发几次，
 *     会让状态机在一瞬间连续推进，反而危险。
 * ==========================================================================*/

#include "app_tasks.h"
#include "app_main.h"
#include "app_config.h"

/* ---- 各任务的周期（毫秒）---------------------------------------------------
 * MOTOR 2 ms：电机任务承担 CAN 命令/安全状态机，20 ms 速度斜坡、100 ms 参数
 *   轮询、150 ms 反馈超时全部用 HAL_GetTick() 计时，2 ms 的节拍保证这些期限
 *   不会因为调度粒度而明显抖动，同时任务内不做任何阻塞等待。
 * OLED  2 ms：I²C 是中断传输，任务只负责"上一段发完了就启动下一段"。
 *   一屏 8 页 = 16 段、每段 129 字节，按 200 ms 刷一帧算，单段约 12 ms，
 *   所以轮询间隔必须远小于这个值，否则会白白空等。
 * LED   5 ms：灯带 20 ms 一帧（50 fps），5 ms 轮询足够发现 DMA 完成。
 * HEALTH 100 ms：10 Hz 健康检查，和 HAL 工程一致。
 * -------------------------------------------------------------------------*/
#define APP_MOTOR_TASK_PERIOD_MS   2U
#define APP_OLED_TASK_PERIOD_MS    2U
#define APP_LED_TASK_PERIOD_MS     5U
#define APP_HEALTH_TASK_PERIOD_MS  100U /* Same 10 Hz rate as the HAL project. */

/* ---- 各任务栈大小（字节）---------------------------------------------------
 * CMSIS-RTOS2 的 stack_size 单位是"字节"，内部会除以 sizeof(StackType_t) 换成
 * FreeRTOS 的"字"数。这里的余量是按最坏调用路径估的：
 *   MOTOR  2048：协议组帧、memcmp、几条浮点运算，实测高水位还剩 ~1.5 KB。
 *   OLED   3072：最深的坑是 snprintf 打浮点（内部要用几百字节），
 *                实测还剩 ~1.8 KB；这个不能省，否则会栈溢出（fatal=6）。
 *   LED    1536：灯效渲染是纯整数运算，实测还剩 ~1.2 KB。
 *   HEALTH 1024：几乎不做事，实测还剩 ~0.8 KB。
 * 加起来 7.7 KB，远小于 24 KB 的 FreeRTOS 堆，启动时一次性分配完。
 * -------------------------------------------------------------------------*/
#define APP_MOTOR_TASK_STACK    2048U
#define APP_OLED_TASK_STACK     3072U
#define APP_LED_TASK_STACK      1536U
#define APP_HEALTH_TASK_STACK   1024U

/* 线程表：id 由创建时填入，name 同时作为 FreeRTOS 线程名（调试器里可见），
 * heartbeat 由任务自己每跑一圈 +1（OLED 页面用它证明"这个线程真的在动"）。
 * 注意：名字全大写，因为 5x7 字形表里没有小写字母。 */
AppTaskInfo g_app_task[APP_TASK_COUNT]={
 {NULL,"MOTOR", APP_MOTOR_TASK_PERIOD_MS,  0U},
 {NULL,"OLED",  APP_OLED_TASK_PERIOD_MS,   0U},
 {NULL,"HEALTH",APP_HEALTH_TASK_PERIOD_MS, 0U},
 {NULL,"LED",   APP_LED_TASK_PERIOD_MS,    0U}
};

/* ----------------------------------------------------------------------------
 * task_run —— 所有任务共用的"周期执行器"
 *
 *   index      : 线程表下标（用来累加 heartbeat）
 *   service    : 每次要调用的服务函数（不许阻塞）
 *   period_ms  : 周期
 *
 * 流程：心跳+1 → 干活 → 算下一拍 → 睡到下一拍。
 * osKernelGetTickCount() 是 RTOS 的 1 kHz 系统节拍（SysTick 驱动），
 * 和 HAL_GetTick() 的 TIM6 时基是两套独立计数器，都从 0 开始但各自走。
 * --------------------------------------------------------------------------*/
static void task_run(uint32_t index,void (*service)(void),uint32_t period_ms)
{
 uint32_t next=osKernelGetTickCount(); /* 下一拍的绝对时刻 */
 for(;;) {
  uint32_t now;
  g_app_task[index].heartbeat++; /* the OLED page proves liveness with this */
  service();                     /* 只推进状态机，不阻塞 */
  now=osKernelGetTickCount();
  next+=period_ms;               /* 理想的下一次执行时刻 */
  /* Overrun: restart the schedule instead of running a catch-up burst. The
   * motor state machine measures its own deadlines, so a late cycle must not
   * be compensated by repeated fast cycles. */
  if((int32_t)(now-next)>=0) next=now+1U; /* 已经超期：放弃积压，下一拍再开始 */
  (void)osDelayUntil(next);               /* 睡到绝对时刻 next（阻塞态，让出 CPU） */
 }
}
/* 下面四个是 CMSIS-RTOS2 要求的线程入口，签名固定为 void (*)(void *)。 */
static void motor_task(void *argument)
{
 (void)argument;task_run(APP_TASK_MOTOR,AppMotor_Loop,APP_MOTOR_TASK_PERIOD_MS);
}
static void oled_task(void *argument)
{
 (void)argument;task_run(APP_TASK_OLED,AppOled_Loop,APP_OLED_TASK_PERIOD_MS);
}
static void health_task(void *argument)
{
 (void)argument;task_run(APP_TASK_HEALTH,AppHealth_Loop,APP_HEALTH_TASK_PERIOD_MS);
}
static void led_task(void *argument)
{
 (void)argument;task_run(APP_TASK_LED,AppLed_Loop,APP_LED_TASK_PERIOD_MS);
}

/* ----------------------------------------------------------------------------
 * AppTasks_Start —— 创建四个线程（在 osKernelStart 之前调用）
 *
 * 关于优先级：CMSIS-RTOS2 的数值直接映射到 FreeRTOS 优先级，数值越大越优先：
 *   AboveNormal=32 > Normal=24 > BelowNormal=16 > Low=8
 * 所以 MOTOR > OLED > HEALTH > LED。电机任务最高：它的 CAN 收发和超时判定
 * 最不能等；OLED/灯带晚几毫秒没有危害。
 *
 * 关于创建顺序：这里按表顺序创建，创建顺序不决定运行顺序（由优先级决定），
 * 而且此刻调度器还没启动，谁都不会跑。
 *
 * 失败处理：任何一个线程创建不出来（堆不够/参数非法）就 App_Fatal，
 * 把原因写进 g_debug.system.fatal 后停机，绝不"少一个任务继续跑"。
 * --------------------------------------------------------------------------*/
void AppTasks_Start(void)
{
 /* Priority follows the table order: motor > oled > health > led. */
 static const struct {
  void (*entry)(void *);
  uint32_t stack_size;
  osPriority_t priority;
 } spec[APP_TASK_COUNT]={
  {motor_task, APP_MOTOR_TASK_STACK,  osPriorityAboveNormal},
  {oled_task,  APP_OLED_TASK_STACK,   osPriorityNormal},
  {health_task,APP_HEALTH_TASK_STACK, osPriorityBelowNormal},
  {led_task,   APP_LED_TASK_STACK,    osPriorityLow}
 };
 uint32_t i;
 for(i=0U;i<APP_TASK_COUNT;i++) {
  osThreadAttr_t attr={0};
  attr.name=g_app_task[i].name;       /* 线程名（OLED 页面与调试器都用它） */
  attr.stack_size=spec[i].stack_size; /* 字节，从 FreeRTOS 堆里分配 */
  attr.priority=spec[i].priority;     /* CMSIS 优先级，数值越大越优先 */
  g_app_task[i].id=osThreadNew(spec[i].entry,NULL,&attr);
  if(g_app_task[i].id==NULL)App_Fatal(APP_FATAL_TASK); /* 创建失败=致命 */
 }
}
