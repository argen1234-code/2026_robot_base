/* ============================================================================
 * app_display.c —— 0.96" OLED（SSD1306 128×64）的 RTOS 页面
 *
 * 这是一个"纯格式化"模块：只负责把一串数字拼成 8 行文字，然后交给
 * Oled_Line()，不碰任何 RTOS API、不碰硬件。好处是可以在 PC 上用假 OLED
 * 直接编译运行（Tools/test_display.c），改版面不用烧板子。
 *
 * 版面预算（很关键）：
 *   屏幕 128×64，字形 5×7 + 每字后 1 列空白 = 每字占 6 列、每行占 8 行像素，
 *   所以每行最多 128/6 = 21 个字符，共 8 行。所有行都按 21 字符精算，
 *   并用 22 字节缓冲 snprintf 兜底：数值再大也只会被截断，不会串行。
 *
 * 字形限制：Devices/oled_font.h 只有大写字母、数字和 + - . / : < = > ? 和空格，
 * 没有小写、没有 %。所以这里所有文字必须大写，也不能打印百分号
 * （CPU 占用因此写成 "1.2" 这种裸数字，含义在文档和行序里约定）。
 * ==========================================================================*/
#include "app_display.h"
#include "app_config.h"
#include "oled_ssd1306.h"
#include <stdio.h>

/* 每行可用字符数（见文件头的预算说明）。 */
#define APP_DISPLAY_CHARS 21U

/* 应用状态机 → 3 个字母。顺序必须和 App/app_motor.h 里的 MOTOR_* 枚举一致：
 * 0 DISABLED、1 PREPARED、2 ENABLED、3 RUNNING、4 STOPPING、5 FAULT、
 * 6 CONFIGURING、7 ENABLING。改状态机时这里要同步，否则屏幕上会显示错字。 */
static const char *state3(uint32_t state)
{
 static const char *names[]={"DIS","PRE","ENA","RUN","STP","FLT","CFG","ENG"};
 return state<8U?names[state]:"???";
}
/* 字节数 → "18.2K" 形式（一位小数）。屏幕上不需要更高精度，
 * 但一位小数能看出"堆在缓慢减少"这种趋势。 */
static void kilo(char *out,size_t size,uint32_t bytes)
{
 snprintf(out,size,"%lu.%luK",(unsigned long)(bytes/1024U),
          (unsigned long)(((bytes%1024U)*10U)/1024U));
}
/* 秒数 → "hh:mm:ss"。小时数超过 99 就钉在 99，
 * 目的是保证这一行永远不超过 8 个字符（否则整行会被挤掉尾巴）。 */
static void clock3(char *out,size_t size,uint32_t seconds)
{
 uint32_t hours=seconds/3600U,minutes=(seconds/60U)%60U,secs=seconds%60U;
 if(hours>99U)hours=99U;
 snprintf(out,size,"%02lu:%02lu:%02lu",(unsigned long)hours,
          (unsigned long)minutes,(unsigned long)secs);
}

/* ----------------------------------------------------------------------------
 * AppDisplay_Draw —— 画一整页（清除 → 逐行写入）
 *
 * 行 0：UP hh:mm:ss SW nnnnn   运行时长 + 上电以来上下文切换次数（低 5 位）
 *       SW 是"调度器干了多少活"的最直观指标：本工程约 1800~2000 次/秒。
 * 行 1：HEAP now MIN ever      当前剩余堆 / 历史最小剩余；两个数长期相等=无泄漏
 * 行 2-5：四个线程，各占一行    名字、状态(RUN/BLK/…)、CPU 占用%、剩余栈
 * 行 6：MOT ONLINE  V±x.xxx    电机是否在线 + 实测速度（离线时显示 ---）
 * 行 7：SET ±x.xxx RAD/S ST    设定速度 + 应用状态
 *
 * 注意：这里只做"取数→排版"，取数在 App/app_main.c 的 draw() 里完成，
 * 因为取数要用到 RTOS API，而本文件要保持可移植/可测。
 * --------------------------------------------------------------------------*/
void AppDisplay_Draw(const AppDisplayRtos *rtos,const MotorStatus *motor,const MotorStatus *motor2)
{
 char line[APP_DISPLAY_CHARS+1U];               /* 21 字符 + 结尾 0 */
 char uptime[9],now_heap[8],min_heap[8],stack[8];
 uint32_t i,count=rtos->task_count;
 if(count>APP_DISPLAY_MAX_TASKS)count=APP_DISPLAY_MAX_TASKS;
 Oled_Clear();                                  /* 先清整页，避免旧字符残留 */
 clock3(uptime,sizeof(uptime),rtos->uptime_s);
 snprintf(line,sizeof(line),"UP %s SW %05lu",uptime,
          (unsigned long)(rtos->switch_count%100000U)); /* 20 characters */
 Oled_Line(0,line);
 kilo(now_heap,sizeof(now_heap),rtos->heap_free_b);
 kilo(min_heap,sizeof(min_heap),rtos->heap_min_b);
 snprintf(line,sizeof(line),"HEAP %s MIN %s",now_heap,min_heap); /* 21 */
 Oled_Line(1,line);
 /* Two task rows alternate each second; both motors remain visible. */
 for(i=0U;i<2U && i<count;i++) {
  uint32_t at=(i+(rtos->uptime_s%2U)*2U)%count;
  /* 占用是千分数：123 → 12.3%。超过 999 钉住，保证"整数.小数"最多 4 字符。 */
  uint32_t share=rtos->task[at].cpu_permille;
  if(share>999U)share=999U; /* keeps the field at 4 characters */
  kilo(stack,sizeof(stack),rtos->task[at].stack_free_b);
  /* <线程名:6> <状态:3> <占用> S<剩余栈>：整行 20-21 字符正好放下 */
  snprintf(line,sizeof(line),"%-6s %s %lu.%lu S%s",rtos->task[at].name,
           rtos->task[at].state,(unsigned long)(share/10U),
           (unsigned long)(share%10U),stack);
  Oled_Line((uint8_t)(6U+i),line);
 }
 /* 速度只在反馈新鲜（<150 ms）时显示；显示一个陈旧的数字比显示 --- 更危险，
  * 因为看屏的人会以为电机还在按那个速度转。目标速度不受影响，始终显示。 */
 for(i=0U;i<2U;i++) {
  const MotorStatus *m=i?motor2:motor;
  if(m->online && m->feedback_age_ms<APP_FEEDBACK_TIMEOUT_MS)
   snprintf(line,sizeof(line),"M%lu %02lX %s V%+6.2f",(unsigned long)(i+1U),
    (unsigned long)m->configured_id,state3(m->state),(double)m->velocity_rad_s);
  else snprintf(line,sizeof(line),"M%lu %02lX OFFLINE %s",(unsigned long)(i+1U),
    (unsigned long)m->configured_id,state3(m->state));
  Oled_Line((uint8_t)(2U+2U*i),line);
  snprintf(line,sizeof(line),"SET %+6.2f RAD/S",(double)m->target_rad_s);
  Oled_Line((uint8_t)(3U+2U*i),line);
 }
}
