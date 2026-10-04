/**
 * bsp_fatfs_ex.c — FATFS extended file management utilities
 * Ported from ALIENTEK exfuns, adapted: mymalloc → lv_mem_alloc, SDRAM pool.
 */
#include "bsp_fatfs_ex.h"
#include "lvgl.h"
#include <string.h>
#include <stdio.h>

/* FATFS work area (per-volume FS objects, allocated on heap) */
FATFS *bsp_fs[FF_VOLUMES];

/* File type lookup table [major][subtype] */
#define FT_MAJOR 7
#define FT_MINOR 7
static const char *const g_ft_tbl[FT_MAJOR][FT_MINOR] = {
    {"BIN"},
    {"LRC"},
    {"NES", "SMS"},
    {"TXT", "C", "H"},
    {"WAV", "MP3", "OGG", "FLAC", "AAC", "WMA", "MID"},
    {"BMP", "JPG", "JPEG", "GIF"},
    {"AVI"},
};


uint8_t bsp_fatfs_ex_init(void)
{
    uint8_t i;
    for (i = 0; i < FF_VOLUMES; i++) {
        bsp_fs[i] = (FATFS *)lv_mem_alloc(sizeof(FATFS));
        if (!bsp_fs[i]) break;
    }
    return (i == FF_VOLUMES) ? 0 : 1;
}


static uint8_t char_upper(uint8_t c)
{
    if (c >= 'a' && c <= 'z') return c - 0x20;
    return c;
}


uint8_t bsp_fatfs_file_type(const TCHAR *fname)
{
    const TCHAR *attr = NULL;
    uint8_t tbuf[5];
    uint8_t i, j;
    int len;

    len = (int)strlen((const char *)fname);
    if (len < 2) return 0xFF;

    /* find last '.' */
    for (i = 0; i < (uint8_t)len; i++) {
        if (fname[i] == '.') attr = &fname[i + 1];
    }
    if (!attr) return 0xFF;

    strncpy((char *)tbuf, (const char *)attr, 4);
    tbuf[4] = '\0';
    for (i = 0; i < 4; i++) tbuf[i] = char_upper(tbuf[i]);

    for (i = 0; i < FT_MAJOR; i++) {
        for (j = 0; j < FT_MINOR; j++) {
            if (!g_ft_tbl[i][j] || g_ft_tbl[i][j][0] == '\0') break;
            if (strcmp((const char *)g_ft_tbl[i][j], (const char *)tbuf) == 0)
                return (i << 4) | j;
        }
    }
    return 0xFF;
}


uint8_t bsp_fatfs_get_free(const TCHAR *pdrv, uint32_t *total, uint32_t *free)
{
    FATFS *fs1;
    DWORD fre_clust = 0;
    uint32_t tot_sect, fre_sect;

    FRESULT res = f_getfree(pdrv, &fre_clust, &fs1);
    if (res != FR_OK) return 1;

    tot_sect = (fs1->n_fatent - 2) * fs1->csize;
    fre_sect = fre_clust * fs1->csize;
    *total = tot_sect >> 1;   /* sectors → KB (512B/sector) */
    *free  = fre_sect >> 1;
    return 0;
}


uint32_t bsp_fatfs_dir_size(const TCHAR *dirname)
{
    DIR     dd;
    FILINFO fi;
    TCHAR   path[256];
    uint32_t size = 0;

    if (f_opendir(&dd, dirname) != FR_OK) return 0;

    while (f_readdir(&dd, &fi) == FR_OK && fi.fname[0]) {
        if (fi.fname[0] == '.') continue;

        if (fi.fattrib & AM_DIR) {
            snprintf((char *)path, sizeof(path), "%s/%s", (const char *)dirname, fi.fname);
            size += bsp_fatfs_dir_size(path);
        } else {
            size += fi.fsize;
        }
    }
    return size;
}


uint8_t bsp_fatfs_file_copy(bsp_fatfs_copy_cb cb, const TCHAR *psrc, const TCHAR *pdst,
                            uint32_t totsize, uint32_t cpdsize, uint8_t fwmode)
{
    FIL     fsrc, fdst;
    uint8_t *buf;
    UINT    br, bw;
    FRESULT res;
    uint8_t curpct = 0;
    uint32_t lcpdsize = cpdsize;

    buf = (uint8_t *)lv_mem_alloc(8192);
    if (!buf) return 100;

    BYTE mode = (fwmode == 0) ? FA_CREATE_NEW : FA_CREATE_ALWAYS;

    res = f_open(&fsrc, psrc, FA_READ | FA_OPEN_EXISTING);
    if (res == FR_OK) res = f_open(&fdst, pdst, FA_WRITE | mode);

    if (res != FR_OK) { lv_mem_free(buf); return (uint8_t)res; }

    if (totsize == 0) {
        totsize = (uint32_t)fsize(&fsrc);
        lcpdsize = 0;
    }

    if (cb) cb(psrc, 0, 2);   /* notify copy start */

    while (1) {
        res = f_read(&fsrc, buf, 8192, &br);
        if (res != FR_OK || br == 0) break;
        res = f_write(&fdst, buf, br, &bw);
        lcpdsize += bw;

        if (totsize > 0) {
            uint8_t pct = (uint8_t)((lcpdsize * 100) / totsize);
            if (pct != curpct) {
                curpct = pct;
                if (cb && cb(psrc, curpct, 2)) { res = 0xFF; break; }
            }
        }
        if (res != FR_OK || bw < br) break;
    }

    f_close(&fsrc);
    f_close(&fdst);
    lv_mem_free(buf);
    return (uint8_t)res;
}


uint8_t bsp_fatfs_dir_copy(bsp_fatfs_copy_cb cb, const TCHAR *psrc, const TCHAR *pdst,
                           uint32_t *totsize, uint32_t *cpdsize, uint8_t fwmode)
{
    DIR     dd;
    FILINFO fi;
    TCHAR   spath[256], dpath[256];
    const TCHAR *fn;

    if (f_opendir(&dd, psrc) != FR_OK) return 1;

    /* extract dest folder name */
    fn = psrc;
    while (*fn) fn++;
    while (fn > psrc && *(fn - 1) != '/' && *(fn - 1) != '\\') fn--;

    snprintf((char *)dpath, sizeof(dpath), "%s/%s", (const char *)pdst, fn);
    f_mkdir(dpath);   /* create folder, ignore FR_EXIST */

    if (cb) cb(fn, 0, 4);

    while (f_readdir(&dd, &fi) == FR_OK && fi.fname[0]) {
        if (fi.fname[0] == '.') continue;

        snprintf((char *)spath, sizeof(spath), "%s/%s", (const char *)psrc, fi.fname);

        if (fi.fattrib & AM_DIR) {
            bsp_fatfs_dir_copy(cb, spath, dpath, totsize, cpdsize, fwmode);
        } else {
            snprintf((char *)dpath, sizeof(dpath), "%s/%s", (const char *)pdst, fn);
            snprintf((char *)dpath, sizeof(dpath), "%s/%s/%s",
                     (const char *)pdst, fn, fi.fname);
            if (cb) cb(fi.fname, 0, 1);
            bsp_fatfs_file_copy(cb, spath, dpath, 0, 0, fwmode);
            if (cpdsize) *cpdsize += fi.fsize;
        }
    }
    return 0;
}
