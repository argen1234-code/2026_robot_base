/* USB CDC 设备描述符（VID/PID = STM32 虚拟串口 0483:5740） */
#include "usbd_core.h"
#include "usbd_ctlreq.h"
#include "usbd_desc.h"
#include "usbd_conf.h"

#define USBD_VID                        0x0483U
#define USBD_PID                        0x5740U
#define USBD_LANGID_STRING              0x409U
#define USBD_MANUFACTURER_STRING        "argen"
#define USBD_PRODUCT_FS_STRING          "DM6215 Chassis CDC"
#define USBD_CONFIGURATION_FS_STRING    "CDC Config"
#define USBD_INTERFACE_FS_STRING        "CDC Interface"

#define USB_SIZ_STRING_FS               64U

/* 设备描述符 */
static uint8_t USBD_FS_DeviceDesc[USB_LEN_DEV_DESC] =
{
    0x12,                       /* bLength */
    USB_DESC_TYPE_DEVICE,       /* bDescriptorType */
    0x00, 0x02,                 /* bcdUSB = 2.00 */
    0x02,                       /* bDeviceClass: CDC */
    0x02,                       /* bDeviceSubClass */
    0x00,                       /* bDeviceProtocol */
    USB_MAX_EP0_SIZE,           /* bMaxPacketSize */
    LOBYTE(USBD_VID), HIBYTE(USBD_VID),
    LOBYTE(USBD_PID), HIBYTE(USBD_PID),
    0x00, 0x02,                 /* bcdDevice = 2.00 */
    USBD_IDX_MFC_STR,           /* iManufacturer */
    USBD_IDX_PRODUCT_STR,       /* iProduct */
    USBD_IDX_SERIAL_STR,        /* iSerialNumber */
    0x01                        /* bNumConfigurations */
};

/* 配置描述符由 CDC 类驱动 usbd_cdc.c 提供（USBD_CDC_CfgDesc） */

static uint8_t USBD_FS_StrDesc[USB_SIZ_STRING_FS];
static uint8_t USBD_FS_LangIDDesc[USB_LEN_LANGID_STR_DESC] =
{
    USB_LEN_LANGID_STR_DESC,
    USB_DESC_TYPE_STRING,
    LOBYTE(USBD_LANGID_STRING),
    HIBYTE(USBD_LANGID_STRING)
};

static uint8_t *USBD_FS_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *USBD_FS_LangIDStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *USBD_FS_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *USBD_FS_ProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *USBD_FS_SerialStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *USBD_FS_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *USBD_FS_InterfaceStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);

USBD_DescriptorsTypeDef FS_Desc =
{
    USBD_FS_DeviceDescriptor,
    USBD_FS_LangIDStrDescriptor,
    USBD_FS_ManufacturerStrDescriptor,
    USBD_FS_ProductStrDescriptor,
    USBD_FS_SerialStrDescriptor,
    USBD_FS_ConfigStrDescriptor,
    USBD_FS_InterfaceStrDescriptor,
};

static uint8_t *USBD_FS_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    *length = sizeof(USBD_FS_DeviceDesc);
    return USBD_FS_DeviceDesc;
}

static uint8_t *USBD_FS_LangIDStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    *length = sizeof(USBD_FS_LangIDDesc);
    return USBD_FS_LangIDDesc;
}

static uint8_t *USBD_FS_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    USBD_GetString((uint8_t *)USBD_MANUFACTURER_STRING, USBD_FS_StrDesc, length);
    return USBD_FS_StrDesc;
}

static uint8_t *USBD_FS_ProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    USBD_GetString((uint8_t *)USBD_PRODUCT_FS_STRING, USBD_FS_StrDesc, length);
    return USBD_FS_StrDesc;
}

static uint8_t *USBD_FS_SerialStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    static uint8_t serial[USB_SIZ_STRING_FS];
    static const char hex[] = "0123456789ABCDEF";
    uint32_t uid[3];
    uint8_t idx = 0U;
    uint8_t i;

    (void)speed;

    uid[0] = *(uint32_t *)UID_BASE;
    uid[1] = *(uint32_t *)(UID_BASE + 4U);
    uid[2] = *(uint32_t *)(UID_BASE + 8U);

    serial[idx++] = 0x1AU;      /* 26 个字符 */
    serial[idx++] = USB_DESC_TYPE_STRING;

    for (i = 0U; i < 3U; i++)
    {
        uint32_t value = uid[i];
        uint8_t nibble;

        for (nibble = 0U; nibble < 8U; nibble++)
        {
            serial[idx++] = (uint8_t)hex[(value >> 28) & 0x0FU];
            serial[idx++] = 0U;
            value <<= 4;
        }
    }

    *length = idx;
    return serial;
}

static uint8_t *USBD_FS_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    USBD_GetString((uint8_t *)USBD_CONFIGURATION_FS_STRING, USBD_FS_StrDesc, length);
    return USBD_FS_StrDesc;
}

static uint8_t *USBD_FS_InterfaceStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    USBD_GetString((uint8_t *)USBD_INTERFACE_FS_STRING, USBD_FS_StrDesc, length);
    return USBD_FS_StrDesc;
}
