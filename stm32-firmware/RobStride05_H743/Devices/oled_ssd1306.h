#ifndef OLED_SSD1306_H
#define OLED_SSD1306_H
#include <stdbool.h>
#include <stdint.h>
/* Fixed SSD1306-compatible 128x64 profile; no controller auto-detection. */
void Oled_Clear(void);
void Oled_Line(uint8_t row,const char *text);
bool Oled_InitBegin(void);
bool Oled_FlushBegin(void);
uint32_t Oled_Poll(void); /* I2C_* result; one segment per call. */
#endif
