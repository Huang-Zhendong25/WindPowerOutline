#ifndef __FLASH_DRV_H
#define __FLASH_DRV_H

#include "main.h"

//#define FLASH_PAGE_SIZE     1024

bool FlashErasePage(uint32_t page_start_addr);
bool Flash_WriteWord(uint32_t addr, uint32_t data);
bool Flash_WriteBuffer(uint32_t addr, uint32_t *pData, uint32_t word_num, bool erase);
bool Flash_ReadBuffer(uint32_t start_addr, uint32_t *pData, uint32_t word_num);

#endif // !__FLASH_DRV_H
