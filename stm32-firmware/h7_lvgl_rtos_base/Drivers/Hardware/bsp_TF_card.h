#ifndef BSP_TF_CARD_H
#define BSP_TF_CARD_H

#include <stdint.h>

/* TF card initialization result codes */
#define BSP_TF_OK       0
#define BSP_TF_ERROR   -1

/* Block size for SD cards */
#define BSP_TF_BLOCK_SIZE  512

/* Initialize TF card via SDMMC1.
 * Returns BSP_TF_OK on success, BSP_TF_ERROR on failure.
 */
int32_t BSP_TF_Init(void);

/* Read blocks from TF card (polling mode).
 * pData:    pointer to buffer (uint32_t aligned)
 * BlockIdx: starting block number
 * BlocksNbr: number of blocks to read
 * Returns BSP_TF_OK on success.
 */
int32_t BSP_TF_ReadBlocks(uint32_t *pData, uint32_t BlockIdx, uint32_t BlocksNbr);

/* Write blocks to TF card (polling mode).
 * Returns BSP_TF_OK on success.
 */
int32_t BSP_TF_WriteBlocks(uint32_t *pData, uint32_t BlockIdx, uint32_t BlocksNbr);

/* Get TF card transfer state.
 * Returns 0 if idle, non-zero if busy.
 */
int32_t BSP_TF_GetState(void);

/* Erase blocks on TF card.
 * Returns BSP_TF_OK on success.
 */
int32_t BSP_TF_Erase(uint32_t BlockIdx, uint32_t BlocksNbr);

/* Get raw card state (returns BSP_ERROR_NONE on idle).
 * Used for polling: while(BSP_TF_GetCardState() != BSP_TF_OK) ;
 */
int32_t BSP_TF_GetCardState(void);

/* Expose BSP_ERROR_NONE for comparison */
#ifndef BSP_ERROR_NONE
#define BSP_ERROR_NONE  0
#endif

#endif /* BSP_TF_CARD_H */
