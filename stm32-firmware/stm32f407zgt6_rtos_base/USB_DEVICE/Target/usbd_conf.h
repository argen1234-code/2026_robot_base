#ifndef __USBD_CONF_H
#define __USBD_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*---------- USBD CONFIGURATION ----------*/
#define USBD_MAX_NUM_INTERFACES                 1U
#define USBD_MAX_NUM_CONFIGURATION              1U
#define USBD_MAX_STR_DESC_SIZ                   512U
#define USBD_DEBUG_LEVEL                        0U
#define USBD_LPM_ENABLED                        0U
#define USBD_SELF_POWERED                       1U

#define DEVICE_FS                               0

/*---------- Memory management ----------*/
void *USBD_static_malloc(uint32_t size);
void USBD_static_free(void *p);

#define USBD_malloc                             (void *)USBD_static_malloc
#define USBD_free                               USBD_static_free
#define USBD_memset                             memset
#define USBD_memcpy                             memcpy
#define USBD_Delay                              HAL_Delay

/*---------- DEBUG ----------*/
#if (USBD_DEBUG_LEVEL > 0U)
#define USBD_UsrLog(...)                        printf(__VA_ARGS__)
#define USBD_ErrLog(...)                        printf("ERROR: "); printf(__VA_ARGS__)
#define USBD_DbgLog(...)                        printf("DEBUG : "); printf(__VA_ARGS__)
#else
#define USBD_UsrLog(...)
#define USBD_ErrLog(...)
#define USBD_DbgLog(...)
#endif

#ifdef __cplusplus
}
#endif

#endif /* __USBD_CONF_H */
