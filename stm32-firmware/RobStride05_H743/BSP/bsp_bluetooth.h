#ifndef BSP_BLUETOOTH_H
#define BSP_BLUETOOTH_H
#include <stdbool.h>
#include <stdint.h>
void BspBluetooth_Init(void);
bool BspBluetooth_Read(uint8_t *byte);
void BspBluetooth_DiscardRx(void);
bool BspBluetooth_Send(const char *text);
void BspBluetooth_Service(void);
uint32_t BspBluetooth_Errors(void);
#endif
