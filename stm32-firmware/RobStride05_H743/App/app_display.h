/* ============================================================================
 * app_display.h —— RTOS 屏幕页面的数据结构
 *
 * 屏幕（0.96" SSD1306，128×64）按 5×7 字形排 21 字符 × 8 行，本工程用满了：
 *
 *   行 0  UP hh:mm:ss SW nnnnn   运行时长 + 上电以来上下文切换次数
 *   行 1  HEAP <现在> MIN <最低>  FreeRTOS 堆余量 / 历史最小
 *   行 2-5  <线程名> <状态> <CPU%> S<剩余栈>
 *   行 6  MOT <ONLINE|OFFLINE> V<实测速度>
 *   行 7  SET <目标速度> RAD/S <应用状态>
 *
 * 这里刻意只放"数据 + 排版"，不放取数逻辑：取数要用 RTOS API，放在
 * App/app_main.c 的 draw()；本模块在 PC 上可以直接编译运行做版面测试。
 *
 * 为什么屏幕不显示"此刻哪个线程在跑"：采样/画屏本身就在 OLED 线程里，
 * 直接读当前线程永远只会得到 OLED 自己。真正有意义的是"调度器把时间分给了
 * 谁"（CPU 占用）和"它现在是运行还是阻塞"（状态），所以显示这两项。
 *
 * 字形限制：只有大写字母、数字和 + - . / : 等少数符号，没有小写、没有 %。
 * ==========================================================================*/
#ifndef APP_DISPLAY_H
#define APP_DISPLAY_H
#include <stdint.h>
#include "app_debug.h" /* MotorStatus */

/* 屏幕任务行最多显示 4 个线程（行 2~5），超出的不显示。 */
#define APP_DISPLAY_MAX_TASKS 4U

typedef struct {
 const char *name;      /* 线程名，按 6 字符左对齐打印 */
 const char *state;     /* RUN/RDY/BLK/SUS/DEL/??? */
 uint32_t stack_free_b; /* 剩余栈（高水位，字节） */
 uint32_t cpu_permille; /* 最近一个采样窗口的 CPU 占用，千分数（123 = 12.3%） */
} AppDisplayTask;

typedef struct {
 uint32_t uptime_s;     /* 运行时长（秒），来自 osKernelGetTickCount()/1000 */
 uint32_t heap_free_b;  /* 当前剩余堆字节数 */
 uint32_t heap_min_b;   /* 历史最小剩余堆字节数 */
 uint32_t switch_count; /* 上电以来上下文切换次数（屏幕只显示低 5 位） */
 const AppDisplayTask *task; /* 线程数组 */
 uint32_t task_count;   /* 数组元素个数（<= APP_DISPLAY_MAX_TASKS） */
} AppDisplayRtos;

/* 画一整页。调用前 pixels 缓冲由 Oled_Clear() 清理；函数内部完成 8 行写入。 */
void AppDisplay_Draw(const AppDisplayRtos *rtos,const MotorStatus *motor,const MotorStatus *motor2);
#endif
