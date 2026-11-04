#ifndef BSP_JY901S_H
#define BSP_JY901S_H
#include <stdbool.h>
#include <stdint.h>
void BspJy901s_Init(void);
bool BspJy901s_Read(uint8_t *byte);
#endif
