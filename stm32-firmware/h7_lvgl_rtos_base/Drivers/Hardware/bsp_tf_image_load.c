/**
 * bsp_tf_image_load.c — Load LVGL images from TF card via FAT32
 */
#include "bsp_tf_image_load.h"
#include "bsp_TF_card.h"
#include "main.h"
#include "ui.h"
#include "ff.h"
#include "lvgl.h"
#include "cmsis_os.h"
#include <string.h>
#include <stdio.h>

static FATFS g_fs;
static osMutexId_t g_tf_fs_mutex = NULL;
static volatile uint8_t g_tf_fs_mounted = 0U;

static const char *IMG_FILES[] = { "icon.bin", "bg.bin", "data.bin", "setbtn.bin", "lock.bin" };
static lv_img_dsc_t *IMG_DESC[] = { &ui_img_1301672413, &ui_img_1943878613, &ui_img_380527393, &ui_img_1565185032, &ui_img_1614311220 };

static uint8_t image_name_equal(const char *left, const char *right)
{
	char a;
	char b;

	if (left == NULL || right == NULL) return 0U;
	while (*left != '\0' && *right != '\0') {
		a = *left++;
		b = *right++;
		if (a >= 'A' && a <= 'Z') a = (char)(a - 'A' + 'a');
		if (b >= 'A' && b <= 'Z') b = (char)(b - 'A' + 'a');
		if (a != b) return 0U;
	}
	return (*left == '\0' && *right == '\0') ? 1U : 0U;
}

const void *bsp_tf_image_find_descriptor(const char *filename)
{
	const char *name = filename;
	const char *cursor;
	uint32_t i;

	if (filename == NULL) return NULL;
	for (cursor = filename; *cursor != '\0'; cursor++) {
		if (*cursor == '/' || *cursor == '\\') name = cursor + 1;
	}

	for (i = 0U; i < (uint32_t)(sizeof(IMG_FILES) / sizeof(IMG_FILES[0])); i++) {
		if (image_name_equal(name, IMG_FILES[i])) {
			return (IMG_DESC[i]->data != NULL && IMG_DESC[i]->data_size > 0U) ?
			       (const void *)IMG_DESC[i] : NULL;
		}
	}
	return NULL;
}

uint8_t bsp_tf_fs_lock_init(void)
{
	if (g_tf_fs_mutex == NULL) {
		g_tf_fs_mutex = osMutexNew(NULL);
	}
	return (g_tf_fs_mutex != NULL) ? 1U : 0U;
}

uint8_t bsp_tf_fs_lock(uint32_t timeout_ticks)
{
	if (g_tf_fs_mutex == NULL || __get_IPSR() != 0U) return 0U;
	return (osMutexAcquire(g_tf_fs_mutex, timeout_ticks) == osOK) ? 1U : 0U;
}

void bsp_tf_fs_unlock(void)
{
	if (g_tf_fs_mutex != NULL && __get_IPSR() == 0U) {
		(void)osMutexRelease(g_tf_fs_mutex);
	}
}

uint8_t bsp_tf_fs_is_mounted(void)
{
	return g_tf_fs_mounted;
}


int32_t bsp_tf_image_load_all(void)
{
	FRESULT fr;
	FIL     fp;
	UINT    br;
	FSIZE_t fsz;
	uint8_t *buf;
	int32_t result = -1;
	uint8_t file_open = 0U;

	if (!bsp_tf_fs_lock(BSP_TF_FS_WAIT_FOREVER)) return -1;

	fr = f_mount(&g_fs, "0:", 1);
	if (fr != FR_OK) {
		g_tf_fs_mounted = 0U;
		printf("[IMG] mount fail\n");
		goto cleanup;
	}
	g_tf_fs_mounted = 1U;

	for (int i = 0; i < (int)(sizeof(IMG_FILES)/sizeof(IMG_FILES[0])); i++)
	{
		fr = f_open(&fp, IMG_FILES[i], FA_READ);
		if (fr != FR_OK) {
			printf("[IMG] %s open fail\n", IMG_FILES[i]);
			goto cleanup;
		}
		file_open = 1U;

		fsz = f_size(&fp);
		buf = lv_mem_alloc((uint32_t)fsz);
		if (!buf) {
			printf("[IMG] %s alloc fail (%uB)\n", IMG_FILES[i], (unsigned)fsz);
			f_close(&fp);
			file_open = 0U;
			goto cleanup;
		}

		fr = f_read(&fp, buf, (UINT)fsz, &br);
		f_close(&fp);
		file_open = 0U;
		if (fr != FR_OK || br != (UINT)fsz) {
			printf("[IMG] %s read fail fr=%d %u/%u\n",
			       IMG_FILES[i], (int)fr, (unsigned)br, (unsigned)fsz);
			lv_mem_free(buf);
			goto cleanup;
		}

		SCB_InvalidateDCache_by_Addr((uint32_t *)buf, (int32_t)fsz);

		IMG_DESC[i]->data      = buf;
		IMG_DESC[i]->data_size = (uint32_t)fsz;
		printf("[IMG] %s %uB\n", IMG_FILES[i], (unsigned)fsz);
	}

	printf("[IMG] all OK\n");
	result = 0;

cleanup:
	if (file_open) f_close(&fp);
	bsp_tf_fs_unlock();
	return result;
}
