/* Host-test mock for the USB CDC hardware interface.
 * Only the hardware boundary is mocked; App/app_usb.c is the real firmware
 * implementation. Values match usbd_def.h so no real USB stack is needed. */
#ifndef TEST_USBD_CDC_IF_H
#define TEST_USBD_CDC_IF_H

#include <stdint.h>

typedef struct {
  uint8_t dev_state;
  void *pClassData;
} USBD_HandleTypeDef;

#define USBD_STATE_CONFIGURED 0x03U
#define USBD_OK               0U
#define USBD_BUSY             1U
#define USBD_FAIL             2U

uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len);
uint16_t CDC_ReadRxData(uint8_t *Buf, uint16_t MaxLen);
uint32_t CDC_GetRxOverflowCount(void);

#endif /* TEST_USBD_CDC_IF_H */
