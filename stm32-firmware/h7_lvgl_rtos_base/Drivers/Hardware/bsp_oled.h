#ifndef __OLED_H
#define __OLED_H

#include <stdint.h>
#include "main.h"

#define OLED_PRINTF 1   //oled打印开关

// OLED I2C地址
#define OLED_I2C_ADDRESS    0x78  // 或 0x7A，具体看OLED模块

#define I2C2_HARDWARE 0  //硬件i2c打开


#define I2C_SOFTWARE 1   //软件i2c打开
/*软件i2c引脚配置*/
#define OLED_W_SCL(x)		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, (GPIO_PinState)(x))
#define OLED_W_SDA(x)		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, (GPIO_PinState)(x))

extern int oled_printf(uint8_t Line, uint8_t Column, const char *format, ...);
extern void oled_clear_line_from(uint8_t Line, uint8_t Column);
extern void oled_clear_from(uint8_t Line, uint8_t Column);

extern void OLED_Init(void);
extern void OLED_Clear(void);
extern void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char);
extern void OLED_ShowString(uint8_t Line, uint8_t Column, char *String);
extern void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
extern void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length);
extern void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
extern void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);

#endif
