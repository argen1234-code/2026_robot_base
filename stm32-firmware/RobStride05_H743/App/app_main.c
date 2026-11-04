#include "app_main.h"
#include "app_motor.h"
#include "app_config.h"
#include "app_led.h"
#include "app_tasks.h"
#include "app_display.h"
#include "app_rtos_stats.h"
#include "bsp_i2c.h"
#include "bsp_time.h"
#include "oled_ssd1306.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "main.h"

/* ============================================================================
 * app_main.c —— 应用层入口（RTOS 版）
 *
 * 与单线程 HAL 工程的关系：电机状态机（app_motor.c）、灯效（app_led.c）、
 * 协议（robstride05.c）、各 BSP 全部是同一份源码；唯一的差别是"谁来调用"：
 *   HAL 版：main 的 while(1) 里按顺序调用 AppMotor_Service/oled_service/...
 *   本  版：app_tasks.c 把每个服务放进自己的周期线程，各自 2/2/100/5 ms。
 * 所以这里的 App*_Loop() 就是"每个线程执行一次的入口"，它们必须非阻塞。
 *
 * 本文件负责四件事：
 *   1. App_Init()      —— 调度器启动之前的一次性初始化（TIM2、CAN、灯带）
 *   2. 四个 *_Loop()   —— 每个周期任务调用一次的服务函数
 *   3. OLED 状态机     —— oled_service()：初始化/刷屏/失败退避/总线恢复
 *   4. App_Fatal()     —— 统一故障落点：写 g_debug.system 后关中断停住
 *
 * 数据归属：g_debug 里 system/oled 两段由本文件写，motor/cmd 段由 app_motor.c
 * 写，两边互不越界；上位机与 OLED 只读。没有加锁——电机快照用 seqlock
 * （snapshot_seq 奇偶），其它段都是单写者、天然一致。
 * ==========================================================================*/

static uint32_t oled_phase,oled_due;
static bool oled_ready;

/* Seqlock read of the published motor snapshot. The motor task keeps each
 * publication stable for 20 ms, so a retry succeeds almost immediately. A
 * torn copy is used only after several attempts; it is display data and never
 * a command or safety decision. */
static void stable_snapshot(MotorStatus *out,volatile MotorStatus *source)
{
 uint32_t before,after,i;
 for(i=0U;i<8U;i++) {
  before=source->snapshot_seq;
  *out=*source;
  __DMB();
  after=source->snapshot_seq;
  if(before==after && !(before&1U))return;
 }
}
/* Row 0 of the page is the uptime; every other RTOS figure is read here so the
 * formatter stays a pure function that can be compiled and tested on the PC.
 * The per-thread numbers come from the scheduler itself (see app_rtos_stats.c):
 * state, stack headroom and the CPU share of the last sampling window. */
static void draw(void)
{
 MotorStatus s,s2;AppDisplayTask tasks[APP_DISPLAY_MAX_TASKS];
 AppRtosTask stats[APP_DISPLAY_MAX_TASKS];AppDisplayRtos rtos;
 osThreadId_t ids[APP_DISPLAY_MAX_TASKS];
 uint32_t i,count=APP_TASK_COUNT,switches;
 stable_snapshot(&s,&g_debug.motor);stable_snapshot(&s2,&g_debug.motor2);
 if(count>APP_DISPLAY_MAX_TASKS)count=APP_DISPLAY_MAX_TASKS;
 for(i=0U;i<count;i++)ids[i]=g_app_task[i].id;
 App_RtosSample(ids,stats,count,&switches);
 for(i=0U;i<count;i++) {
  tasks[i].name=stats[i].name;
  tasks[i].state=stats[i].state;
  tasks[i].stack_free_b=stats[i].stack_free_b;
  tasks[i].cpu_permille=stats[i].cpu_permille;
 }
 rtos.uptime_s=osKernelGetTickCount()/1000U;
 rtos.heap_free_b=(uint32_t)xPortGetFreeHeapSize();
 rtos.heap_min_b=(uint32_t)xPortGetMinimumEverFreeHeapSize();
 rtos.switch_count=switches;
 rtos.task=tasks;
 rtos.task_count=count;
 AppDisplay_Draw(&rtos,&s,&s2);
}
/* ----------------------------------------------------------------------------
 * OLED 状态机（oled_phase）
 *
 *   0 = 空闲：到点就决定下一步（没初始化过→初始化；已初始化→画一帧）
 *   1 = 正在发送初始化命令序列（4 字节那条），等 Oled_Poll() 报完成
 *   2 = 正在发送一帧（16 段 × 129 字节），每段完成后 Oled_Poll() 启动下一段
 *   3 = 需要恢复总线：等退避时间到，就调 BspI2c_RecoverBegin() 开始拉时钟
 *   4 = 正在恢复总线：每 1 ms 走一步，直到 SCL/SDA 释放并重新初始化 I²C
 *
 * 关键：整个过程没有一次"等 I²C 发完"的忙等。I²C 是中断传输，任务每 2 ms
 * 进来看一眼标志就走；一帧 16 段、每段约 12 ms，正好用 200 ms 的周期刷完。
 * 失败（NACK/超时/总线卡死）只影响显示，不会阻塞电机和灯带；
 * 连续失败会退避到 2 s 再试，避免把 CPU 耗在一个坏屏上。
 * --------------------------------------------------------------------------*/
static void oled_failed(uint32_t now)
{
 oled_ready=false;oled_phase=3;g_debug.oled.ready=0;
 g_debug.oled.refresh_fail++;g_debug.oled.consecutive_fail++;
 g_debug.oled.last_error=BspI2c_LastError(); /* 现场：HAL 错误码 + BSP 超时/忙标志 */
 oled_due=now+(g_debug.oled.consecutive_fail<3?500U:2000U); /* 前两次退避 0.5 s，之后 2 s */
 g_debug.oled.next_retry_ms=oled_due;
}
/* OLED 任务每 2 ms 调一次：只推进一步，绝不等待。 */
static void oled_service(uint32_t now)
{
 uint32_t r;g_debug.oled.heartbeat++; /* 心跳：即使屏坏了也一直在涨，证明任务没死 */
 if(oled_phase==1 || oled_phase==2) {   /* 有传输在进行：问它完成没有 */
  r=Oled_Poll();if(r==I2C_BUSY)return; /* 还没完，下一轮再来 */
  if(r!=I2C_OK){oled_failed(now);return;}
  if(oled_phase==1){oled_ready=true;oled_phase=0;oled_due=now;} /* 初始化完成 */
  else {                                /* 一帧的 16 段全部发完 */
   g_debug.oled.refresh_ok++;g_debug.oled.last_success_ms=now;
   g_debug.oled.consecutive_fail=0;g_debug.oled.ready=1;
   oled_phase=0;/* oled_due was set at frame start for ~5 Hz. */
  }
  return;
 }
 if(oled_phase==4) {                    /* 总线恢复进行中：每 1 ms 一步 */
  r=BspI2c_RecoverPoll();if(r==I2C_BUSY)return;
  if(r==I2C_OK){g_debug.oled.recovery_ok++;oled_phase=0;oled_due=now;}
  else oled_failed(now);
  return;
 }
 if((int32_t)(now-oled_due)<0)return;   /* 还没到下一个时间点（注意用有符号比较，处理回绕） */
 if(oled_phase==3) {                    /* 退避到期：开始一次有界总线恢复 */
  g_debug.oled.recovery_attempts++;BspI2c_RecoverBegin();oled_phase=4;return;
 }
 if(!oled_ready) {                      /* 还没成功初始化过：发初始化序列 */
  if(Oled_InitBegin())oled_phase=1;else oled_failed(now);
 }else {                                /* 正常一帧：先画到内存，再整帧发出去 */
  draw();oled_due=now+APP_OLED_PERIOD_MS;
  if(Oled_FlushBegin())oled_phase=2;else oled_failed(now);
 }
}
/* ----------------------------------------------------------------------------
 * App_Init —— 调度器启动之前的一次性初始化
 *
 * 顺序：TIM2 时基 → 启动 CAN（会先发一帧 STOP，保证上电不使能）→ 灯带。
 * 任何一步失败都直接 App_Fatal；CAN 启动失败则让电机模块进入故障态，
 * 而不是"先跑起来再说"。
 * 这里不能调用会阻塞的 RTOS API（调度器还没启动）。
 * --------------------------------------------------------------------------*/
void App_Init(void)
{
 if(!BspTime_Init())App_Fatal(APP_FATAL_TIMEBASE); /* TIM2：1 MHz 计时 + 服务间断检测 */
 g_debug.system.boot_stage=8U;
 AppMotor_Init();   /* 启动 FDCAN、发初始 STOP 帧、发布第一份快照 */
 oled_due=HAL_GetTick()+100U; /* 上电 100 ms 后再开始刷屏，给屏一个上电稳定时间 */
#if APP_LED_FEATURE_ENABLED
 AppLed_Init(); /* Optional decoration; failure must never stop motor/OLED. */
#endif
}
/* ----------------------------------------------------------------------------
 * AppMotor_Loop —— 电机任务每次醒来做的事（2 ms 周期）
 *
 * 顺序很重要：
 *   1. 先更新 g_debug.system.tick_ms / loop_heartbeat —— 上位机就是靠这两个
 *      数在 60 ms 内必须变化，来判断"H743 的调度器还活着"；所以必须放在
 *      最快的那个任务里（这里是电机任务）。
 *   2. 用 TIM2（1 MHz）量一次 AppMotor_Service() 的耗时，记进 last/max_service_us，
 *      用来发现"某次服务是不是偶尔变慢"。
 *   3. AppMotor_Service() 内部是纯状态机：收 CAN 帧 → 安全检查 → 取命令 →
 *      推进一笔有界事务（准备/使能）→ 20 ms 速度斜坡 → 发布快照。
 *      它不会等外设，也不 sleep，所以 2 ms 的节拍足够。
 * --------------------------------------------------------------------------*/
void AppMotor_Loop(void)
{
 uint32_t us,elapsed,now=HAL_GetTick();
 /* The console polls loop_heartbeat and tick_ms to prove the scheduler and
  * this service still run; keep them in the fastest application task. */
 g_debug.system.tick_ms=now;
 g_debug.system.loop_heartbeat++;
 us=BspTime_Us();         /* TIM2 自由计数，1 tick = 1 µs */
 AppMotor_Service();      /* CAN 安全/命令逻辑永远先于显示逻辑执行 */
 elapsed=BspTime_Us()-us;g_debug.system.last_service_us=elapsed;
 if(elapsed>g_debug.system.max_service_us)g_debug.system.max_service_us=elapsed;
}
/* ----------------------------------------------------------------------------
 * AppHealth_Loop —— 10 Hz 健康检查
 *
 * 目前只检查一件事：TIM2 有没有被别人冻结（BspTime_DebugFrozen）。
 * 如果 TIM2 被冻结，"服务间断"检测就失效（CPU 停了也看不出来），
 * 所以把健康位第 1 位置起来，电机状态机会据此拒绝新的运动命令。
 * （HAL 工程里还有"位 0 = D-Cache 被打开"的检查；本工程 Cache 恒关，
 *   这个位不再评估，CCR 的只读记录仍然留在 g_debug.system.ccr 里。）
 * --------------------------------------------------------------------------*/
void AppHealth_Loop(void)
{
 uint32_t fault=0;
 g_debug.system.health_heartbeat++;
 /* Bit1: the wall-clock TIM2 was frozen by something else, so a CPU halt
  * cannot be detected from the TIM2 gap. Bit0 of the HAL project (D-Cache
  * enabled) is not evaluated: this image never enables either cache. */
 if(BspTime_DebugFrozen())fault|=2U;
 AppMotor_SetHealth(fault);
}
/* OLED 任务每次醒来推进一步：初始化/发送下一段/判失败/总线恢复都在 oled_service 里。 */
void AppOled_Loop(void)
{
 oled_service(HAL_GetTick());
}
/* 灯带任务：最低优先级，只推进灯效状态机并启动 DMA，不等发送完成。 */
void AppLed_Loop(void)
{
#if APP_LED_FEATURE_ENABLED
 AppLed_Service(HAL_GetTick()); /* Lowest priority; PWM+DMA send asynchronously. */
#endif
}
/* ----------------------------------------------------------------------------
 * App_Fatal —— 所有致命错误的统一落点
 *
 * 调用者包括：HAL/时钟初始化失败、内核初始化失败、线程创建失败、堆耗尽、
 * 栈溢出、configASSERT、HAL 断言、以及 5 类异常（见 stm32h7xx_it.c）。
 * 做法：把现场（tick/vtor/ccr/cfsr/hfsr/IPSR）和原因码写进 g_debug.system，
 * 然后关中断死循环。这样 SWD 还能读、上位机会拒绝连接并在日志里报异常。
 *
 * 为什么不停机而是死循环：停机后 CPU 状态不可见，死循环至少能让调试器
 * 读到完整的现场；同时不喂狗、不自动复位，避免把真正的故障掩盖掉。
 * 注意：这里不能保证还能发出 CAN 停止帧（可能 ISR 已关/总线已坏），
 * 所以电机侧的超时是最后一道独立保护。
 * --------------------------------------------------------------------------*/
void App_Fatal(uint32_t reason)
{
 g_debug.system.tick_ms=HAL_GetTick();
 g_debug.system.vtor=SCB->VTOR;
 g_debug.system.ccr=SCB->CCR;
 g_debug.system.cfsr=SCB->CFSR;
 g_debug.system.hfsr=SCB->HFSR;
 g_debug.system.exception=__get_IPSR();
 g_debug.system.fatal=reason;
 /* Cannot guarantee a CAN send here. Motor-side timeout is the independent
  * protection. No unconditional watchdog feed/reset loop hides this fault. */
 __disable_irq();for(;;){__NOP();}
}
const char *g_assert_file;
volatile uint32_t g_assert_line;
void App_AssertFailed(const char *file,uint32_t line)
{
 g_assert_file=file;
 g_assert_line=line;
 App_Fatal(APP_FATAL_ASSERT);
}
