#include "flash_drv.h"
#include <string.h>

static uint32_t FlashGetPageAddr(uint32_t addr)
{
    return (addr / FLASH_PAGE_SIZE) * FLASH_PAGE_SIZE;
}

bool FlashErasePage(uint32_t page_start_addr)
{
    uint32_t page_error = 0;

    page_start_addr = FlashGetPageAddr(page_start_addr);

    FLASH_EraseInitTypeDef erase_init = {
        .TypeErase = FLASH_TYPEERASE_PAGES,
        .PageAddress = page_start_addr,
        .NbPages = 1
    };

    HAL_FLASH_Unlock();
    HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase_init, &page_error);
    HAL_FLASH_Lock();

    return status == HAL_OK ? true : false;
}


/*Write a word*/
bool Flash_WriteWord(uint32_t addr, uint32_t data)
{
    HAL_FLASH_Unlock();
    HAL_StatusTypeDef status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, data);
    HAL_FLASH_Lock();

    return status == HAL_OK ? true : false;
}

/*Write multiple words once*/
bool Flash_WriteBuffer(uint32_t addr, uint32_t *pData, uint32_t word_num)
{
    if (word_num == 0)
        return false;
    bool status = FlashErasePage(addr);
    if (FlashErasePage(addr) == false)
        return false;
    HAL_FLASH_Unlock();
    for (uint32_t i = 0; i < word_num; i++)
    {
        if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + i * 4, pData[i]) != HAL_OK)
        {
            status = false;
            break;
        }
    }
    HAL_FLASH_Lock();
    
    return status;
}

bool Flash_ReadBuffer(uint32_t start_addr, uint32_t *pData, uint32_t word_num)
{
    if (word_num == 0)
        return false;
        
    memcpy(pData, (const void*)start_addr, word_num * sizeof(uint32_t));
    
    return true;
}
