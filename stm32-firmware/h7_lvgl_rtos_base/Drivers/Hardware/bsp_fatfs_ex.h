/**
 * bsp_fatfs_ex.h — FATFS extended file management utilities
 * Ported from ALIENTEK exfuns, adapted for STM32H743 + LVGL memory pool.
 */
#ifndef BSP_FATFS_EX_H
#define BSP_FATFS_EX_H

#include "ff.h"       /* FATFS R0.12 */
#include <stdint.h>

/* FATFS work area (allocated on heap during bsp_fatfs_ex_init) */
extern FATFS *bsp_fs[FF_VOLUMES];

/* File type identifiers (extension-based) */
#define BSP_FTYPE_BIN    0x00    /* .bin */
#define BSP_FTYPE_LRC    0x10    /* .lrc */
#define BSP_FTYPE_TEXT   0x30    /* .txt / .c / .h */
#define BSP_FTYPE_AUDIO  0x40    /* .wav .mp3 .ogg .flac .aac .wma .mid */
#define BSP_FTYPE_IMAGE  0x50    /* .bmp .jpg .jpeg .gif */
#define BSP_FTYPE_VIDEO  0x60    /* .avi */

/* Initialize FATFS extended layer (allocates work buffers from LVGL pool).
 * Must be called after lv_init() and after BSP_TF_Init().
 * Returns 0 on success.
 */
uint8_t bsp_fatfs_ex_init(void);

/* Get disk space (KB). pdrv = "0:", total & free in KB. Returns 0 on success. */
uint8_t bsp_fatfs_get_free(const TCHAR *pdrv, uint32_t *total, uint32_t *free);

/* Identify file type from extension. Returns BSP_FTYPE_xxx or 0xFF if unknown. */
uint8_t bsp_fatfs_file_type(const TCHAR *fname);

/* Get total size of a directory (recursive). Returns size in bytes, 0 on error. */
uint32_t bsp_fatfs_dir_size(const TCHAR *dirname);

/* Progress callback for copy operations.
 * pname: current file name, pct: percentage (0-100),
 * mode bit0=update file name, bit1=update pct, bit2=copy start
 * Return 0 to continue, 1 to abort.
 */
typedef uint8_t (*bsp_fatfs_copy_cb)(const TCHAR *pname, uint8_t pct, uint8_t mode);

/* Copy a single file. totsize=0 means auto-detect from source file.
 * fwmode=0: fail if dst exists; 1: overwrite.
 * Returns 0 on success.
 */
uint8_t bsp_fatfs_file_copy(bsp_fatfs_copy_cb cb, const TCHAR *psrc, const TCHAR *pdst,
                            uint32_t totsize, uint32_t cpdsize, uint8_t fwmode);

/* Copy directory recursively. Returns 0 on success. */
uint8_t bsp_fatfs_dir_copy(bsp_fatfs_copy_cb cb, const TCHAR *psrc, const TCHAR *pdst,
                           uint32_t *totsize, uint32_t *cpdsize, uint8_t fwmode);

#endif /* BSP_FATFS_EX_H */
