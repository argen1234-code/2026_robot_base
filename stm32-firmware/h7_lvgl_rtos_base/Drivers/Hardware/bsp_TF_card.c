/**
 * bsp_TF_card.c — TF card (microSD) driver wrapper
 *
 * Uses STM32H743 SDMMC1 peripheral in 4-bit mode @ 20MHz.
 * Wraps the ST BSP SD driver (sdmmc_sd.c).
 *
 * Pin mapping:
 *   PC8  → SDMMC1_D0    PC12 → SDMMC1_CK
 *   PC9  → SDMMC1_D1    PD2  → SDMMC1_CMD
 *   PC10 → SDMMC1_D2
 *   PC11 → SDMMC1_D3
 */

#include "bsp_TF_card.h"
#include "sdmmc_sd.h"

/* SD Instance index (only one on this board) */
#define TF_INSTANCE  0

int32_t BSP_TF_Init(void)
{
    int32_t ret = BSP_SD_Init(TF_INSTANCE);
    return (ret == BSP_ERROR_NONE) ? BSP_TF_OK : BSP_TF_ERROR;
}

int32_t BSP_TF_ReadBlocks(uint32_t *pData, uint32_t BlockIdx, uint32_t BlocksNbr)
{
    int32_t ret = BSP_SD_ReadBlocks(TF_INSTANCE, pData, BlockIdx, BlocksNbr);
    return (ret == BSP_ERROR_NONE) ? BSP_TF_OK : BSP_TF_ERROR;
}

int32_t BSP_TF_WriteBlocks(uint32_t *pData, uint32_t BlockIdx, uint32_t BlocksNbr)
{
    int32_t ret = BSP_SD_WriteBlocks(TF_INSTANCE, pData, BlockIdx, BlocksNbr);
    return (ret == BSP_ERROR_NONE) ? BSP_TF_OK : BSP_TF_ERROR;
}

int32_t BSP_TF_GetState(void)
{
    return (int32_t)BSP_SD_GetCardState(TF_INSTANCE);
}

int32_t BSP_TF_Erase(uint32_t BlockIdx, uint32_t BlocksNbr)
{
    int32_t ret = BSP_SD_Erase(TF_INSTANCE, BlockIdx, BlocksNbr);
    return (ret == BSP_ERROR_NONE) ? BSP_TF_OK : BSP_TF_ERROR;
}

int32_t BSP_TF_GetCardState(void)
{
    int32_t state = BSP_SD_GetCardState(TF_INSTANCE);
    return (state == 0) ? BSP_TF_OK : BSP_TF_ERROR;
}
