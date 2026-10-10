/* Host-compile stub for pid.h's `#include "main.h"`.
 * 真机用 STM32 的 HAL main.h；这里只补 pid.h/pid.c 实际用到的类型，
 * 以便在 PC 上用 gcc 直接编译【真实的 pid.c】做算法验证。
 * 只放一个 <stdint.h> 是为了保证与真机完全一致的类型宽度。 */
#ifndef HOST_STUB_MAIN_H
#define HOST_STUB_MAIN_H

#include <stdint.h>
#include <stddef.h>

#endif
