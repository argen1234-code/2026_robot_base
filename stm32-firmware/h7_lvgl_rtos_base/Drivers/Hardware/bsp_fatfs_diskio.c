/**
 * bsp_fatfs_diskio.c — FATFS disk I/O adapter (multi-volume)
 *
 * Device 0 = SDMMC1 TF card   (512B sectors, 4-bit, 20MHz)
 * Device 1 = W25Q64 QSPI Flash (quad I/O, 120MHz)
 */
#include "diskio.h"
#include "bsp_TF_card.h"
#include "sdmmc_sd.h"
#include "bsp_qspi_flash.h"
#include "lvgl.h"

extern QSPI_HandleTypeDef hqspi;   /* defined in bsp_qspi_flash.c */

/* ---- Device 0: SD card ---- */
static int sd_ok = 0;

void fatfs_diskio_set_init(void) { sd_ok = 1; }

/* ---- Device 1: QSPI Flash ---- */
static int qspi_ok = 0;
#define QSPI_SECTOR_COUNT  (8U * 1024U * 1024U / 512U)   /* 8MB / 512B = 16384 */


/* ==================== disk_initialize ==================== */
DSTATUS disk_initialize(BYTE pdrv)
{
	switch (pdrv) {
	case 0: /* SD card */
		if (sd_ok) return 0;
		if (BSP_SD_Init(SD_Instance) == BSP_ERROR_NONE) { sd_ok = 1; return 0; }
		return STA_NOINIT;
	case 1: /* QSPI Flash */
		if (qspi_ok) return 0;
		if (QSPI_W25Qxx_Init() == 0) { qspi_ok = 1; return 0; }
		return STA_NOINIT;
	}
	return STA_NOINIT;
}


/* ==================== disk_status ==================== */
DSTATUS disk_status(BYTE pdrv)
{
	switch (pdrv) {
	case 0: return sd_ok   ? 0 : STA_NOINIT;
	case 1: return qspi_ok ? 0 : STA_NOINIT;
	}
	return STA_NOINIT;
}


/* ==================== disk_read ==================== */
DRESULT disk_read(BYTE pdrv, BYTE *buff, DWORD sector, UINT count)
{
	if (!count) return RES_PARERR;

	switch (pdrv) {
	case 0: { /* SD card — protect with IRQ disable */
		__disable_irq();
		int32_t ret = BSP_SD_ReadBlocks(SD_Instance, (uint32_t *)buff, sector, count);
		__enable_irq();
		if (ret != BSP_ERROR_NONE) return RES_ERROR;
		if ((uint32_t)buff >= 0xC0000000U && (uint32_t)buff < 0xC2000000U)
			SCB_InvalidateDCache_by_Addr((uint32_t *)buff, (int32_t)(count * 512));
		return RES_OK;
	}
	case 1: /* QSPI Flash — quad I/O read */
		QSPI_W25Qxx_ReadBuffer(buff, sector * 512U, count * 512U);
		return RES_OK;
	}
	return RES_PARERR;
}


/* ==================== disk_write ==================== */
DRESULT disk_write(BYTE pdrv, const BYTE *buff, DWORD sector, UINT count)
{
	if (!count) return RES_PARERR;

	switch (pdrv) {
	case 0: /* SD card */
		__disable_irq();
		int32_t ret = BSP_SD_WriteBlocks(SD_Instance, (uint32_t *)buff, sector, count);
		__enable_irq();
		return (ret == BSP_ERROR_NONE) ? RES_OK : RES_ERROR;
	case 1: /* QSPI Flash — quad page program (auto page-boundary handling) */
		QSPI_W25Qxx_WriteBuffer((uint8_t *)buff, sector * 512U, count * 512U);
		return RES_OK;
	}
	return RES_PARERR;
}


/* ==================== disk_ioctl ==================== */
DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
	switch (pdrv) {
	case 0:
		switch (cmd) {
		case CTRL_SYNC:       return RES_OK;
		case GET_SECTOR_SIZE: *(DWORD *)buff = 512; return RES_OK;
		case GET_BLOCK_SIZE:  *(DWORD *)buff = 512; return RES_OK;
		case GET_SECTOR_COUNT: {
			BSP_SD_CardInfo info;
			if (BSP_SD_GetCardInfo(SD_Instance, &info) != BSP_ERROR_NONE) return RES_ERROR;
			*(DWORD *)buff = info.LogBlockNbr;
			return RES_OK;
		}
		}
		break;
	case 1:
		switch (cmd) {
		case CTRL_SYNC:       return RES_OK;
		case GET_SECTOR_SIZE: *(DWORD *)buff = 512; return RES_OK;
		case GET_BLOCK_SIZE:  *(DWORD *)buff = 4096; return RES_OK;
		case GET_SECTOR_COUNT: *(DWORD *)buff = QSPI_SECTOR_COUNT; return RES_OK;
		}
		break;
	}
	return RES_PARERR;
}


/* ==================== get_fattime ==================== */
DWORD get_fattime(void) { return 0; }


/* ==================== FATFS memory ==================== */
void *ff_memalloc(UINT size) { return lv_mem_alloc(size); }
void ff_memfree(void *mf)   { lv_mem_free(mf); }
