#ifndef BSP_TF_IMAGE_LOAD_H
#define BSP_TF_IMAGE_LOAD_H

#include <stdint.h>

#define BSP_TF_FS_WAIT_FOREVER 0xFFFFFFFFU

/* Shared FatFs access guard. FatFs is configured with _FS_REENTRANT == 0,
 * therefore every task that accesses drive 0/S must use this guard. */
uint8_t bsp_tf_fs_lock_init(void);
uint8_t bsp_tf_fs_lock(uint32_t timeout_ticks);
void bsp_tf_fs_unlock(void);
uint8_t bsp_tf_fs_is_mounted(void);

/* Return the LVGL image descriptor for one of the known raw .bin assets.
 * The return value is NULL when the name is unknown or the asset is not loaded. */
const void *bsp_tf_image_find_descriptor(const char *filename);

/* Load LVGL image pixel data from TF card (FAT32 filesystem).
 * Image files on TF card:
 *   "icon.bin"  — 64x64 RGBA raw pixels  (16 KB)
 *   "bg.bin"    — 1024x600 RGBA raw pixels (2.4 MB)
 *
 * Allocates SDRAM via lv_mem_alloc(), patches lv_img_dsc_t descriptors.
 * Returns 0 on success, -1 on failure.
 */
int32_t bsp_tf_image_load_all(void);

#endif /* BSP_TF_IMAGE_LOAD_H */
