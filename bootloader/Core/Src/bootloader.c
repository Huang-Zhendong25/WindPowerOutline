#include "bootloader.h"
#include <string.h>
#include "usart.h"
#include "iwdg.h"

static uint32_t cur_write_addr = APP_FLASH_STARTADDR;
static uint32_t cur_total_len = 0;
//static uint8_t rx_buffer[RX_BUFFER_SIZE + 2];

static void Flash_EraseApp(void)
{
    uint32_t page_start = (APP_FLASH_STARTADDR / 1024) * 1024;
    uint32_t page_end = ((APP_FLASH_STARTADDR + APP_MAX_SIZE - 1) / 1024) * 1024;

    FLASH_EraseInitTypeDef erase_init = {
        .TypeErase = FLASH_TYPEERASE_PAGES,
        .PageAddress = page_start,
        .NbPages = (page_end - page_start) / 1024 + 1
    };
    uint32_t erase_error = 0;
    HAL_FLASH_Unlock();
    HAL_FLASHEx_Erase(&erase_init, &erase_error);
    HAL_FLASH_Lock();
}

static bool Flash_WriteBuffer(uint32_t addr, uint8_t *data, uint32_t len)
{
    uint32_t word_count = len / 4, word_count_remain = len % 4;

    HAL_FLASH_Unlock();
    for (uint32_t i = 0; i < word_count; i++)
    {
        uint32_t word = *(uint32_t *)(data + i * 4);
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + i * 4, word) != HAL_OK)
        {
            HAL_FLASH_Lock();
            return false;
        }
    }
    if (word_count_remain > 0)
    {
        uint32_t last_word = 0xffffffff;
        memcpy(&last_word, data + word_count * 4, word_count_remain);
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + word_count * 4, last_word) != HAL_OK)
        {
            HAL_FLASH_Lock();
            return false;
        }
    }
    HAL_FLASH_Lock();
    return true;
}

static bool Flash_WriteUpgradeInfo(uint32_t addr, uint32_t page_start_addr, uint32_t page_num, UpgradeInfo_t *data)
{
    uint32_t erase_erro = 0;
    //uint32_t page_addr_end = ((page_start_addr + size - 1) / 1024) * 1024;

    FLASH_EraseInitTypeDef erase_init = {
        .TypeErase = FLASH_TYPEERASE_PAGES,
        .PageAddress = page_start_addr,
        .NbPages = page_num,
    };

    HAL_FLASH_Unlock();
    
    if (HAL_FLASHEx_Erase(&erase_init, &erase_erro) != HAL_OK)
    {
        HAL_FLASH_Lock();
        return false;
    }

    uint32_t data_len = sizeof(UpgradeInfo_t);
    uint32_t word_num = data_len / 4;

    for (uint32_t i = 0; i < word_num; i++)
    {
        uint32_t word = ((uint32_t *)data)[i];
        
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + i * 4, word) != HAL_OK)
        {
            HAL_FLASH_Lock();
            return false;
        }
    }

    HAL_FLASH_Lock();

    return true;
}

static void UpgradeState_Set(uint32_t state)
{
    UpgradeInfo_t UpgradeInfo;

    UpgradeInfo.upgrade_flag = *(__IO uint32_t *)UPGRADE_FLAG_ADDR;
    UpgradeInfo.upgrade_state = state;
    //Flash_WriteBuffer(UPGRADE_STATE_FLASH_ADDR, (uint8_t *)&state, 4);
    Flash_WriteUpgradeInfo(UPGRADE_FLAG_ADDR, UPGRADE_FLAG_ADDR, 1, &UpgradeInfo);
}

static uint32_t UpgradeState_Get(void)
{
    return *(__IO uint32_t *)UPGRADE_STATE_FLASH_ADDR;    //UPGRADE_FLAG_ADDR must be 4-byte aligned
}

static void UpgradeFlag_Clear(void)
{
    UpgradeInfo_t UpgradeInfo;

    UpgradeInfo.upgrade_flag = UPGRADE_FLAG_CLEAR;
    //UpgradeInfo.upgrade_state = *(__IO uint32_t *)UPGRADE_STATE_FLASH_ADDR;
    UpgradeInfo.upgrade_state = UPGRADE_STATE_IDLE;
    //uint32_t clear = UPGRADE_FLAG_CLEAR;
    //Flash_WriteBuffer(UPGRADE_FLAG_ADDR, (uint8_t *)&clear, 4);    //clear upgrade flag
    Flash_WriteUpgradeInfo(UPGRADE_FLAG_ADDR, UPGRADE_FLAG_ADDR, 1, &UpgradeInfo);
    //HAL_Delay(100);
    //UpgradeState_Set(UPGRADE_STATE_IDLE);     //upgrade idle after clear upgrade flag
}

void Bootloader_SendResponse(char *respText)
{
    HAL_GPIO_WritePin(RS485_EN1_GPIO_Port, RS485_EN1_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart2, (uint8_t *)respText, strlen(respText), 500);
    HAL_GPIO_WritePin(RS485_EN1_GPIO_Port, RS485_EN1_Pin, GPIO_PIN_RESET);
}

//static bool Bootloader_ReceiveData(uint8_t *data, uint16_t len, uint32_t timeout)
//{
//    /* uint32_t tick_start = HAL_GetTick();
//    uint16_t received_len = 0; */
//
//    /* while (received_len < len)
//    {
//        //HAL_IWDG_Refresh(&hiwdg);
//        if (HAL_GetTick() - tick_start > timeout)
//        {
//            return false;
//        }
//        if (HAL_UART_Receive(&huart2, &data[received_len], 1, 10) == HAL_OK)
//        {
//            received_len += 1;
//            tick_start = HAL_GetTick();
//        }
//    } */
//   HAL_GPIO_WritePin(RS485_EN1_GPIO_Port, RS485_EN1_Pin, GPIO_PIN_RESET);
//    if (HAL_UART_Receive(&huart2, data, len, timeout) == HAL_OK)
//    {
//        return true;
//    }
//    return false;
//}

static bool Bootloader_ReceiveFrame(uint8_t *buffer, uint16_t *out_len, uint32_t timeout_ms)
{
    uint32_t start_tick = HAL_GetTick();
    uint16_t idx = 0;
    uint8_t cmd = 0;
    uint8_t data_len = 0;
    //uint8_t expected_len = 0;

    HAL_GPIO_WritePin(RS485_EN1_GPIO_Port, RS485_EN1_Pin, GPIO_PIN_RESET);

    while (1)
    {
        if (HAL_GetTick() - start_tick > timeout_ms)    //a complete frame timeout
        {
            return false;
        }
        if (HAL_UART_Receive(&huart2, &buffer[idx], 1, 10) == HAL_OK)    //one byte timeout
        {
            idx += 1;

            if (idx == 1)
            {
                cmd = buffer[0];
                if (cmd == CMD_ABORT)
                {
                    *out_len = idx;
                    return true;
                }
            }
            /* else if (idx == 2)
            {
                data_len = buffer[1];
            } */
            else 
            {
                if (cmd == CMD_DATA)
                {
                    data_len = buffer[1];

                    if (idx >= data_len + 2)   //cmd + len + firmware_data
                    {
                        *out_len = idx;
                        return true;
                    }
                }
                else if (cmd == CMD_END)
                {
                    if (idx >= 9)
                    {
                        *out_len = idx;
                        return true;
                    }
                }
                else
                {}
            }
        }
    }
}

static uint32_t Bootloader_CRC32(uint8_t *data, uint32_t len) {
    uint32_t crc = 0xFFFFFFFF;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xEDB88320;
            } else {
                crc >>= 1;
            }
        }
    }
    return ~crc;
}

static bool isValid(void)
{
    if (UpgradeState_Get() != UPGRADE_STATE_IDLE)
        return false;

    uint32_t app_stack_top = *(__IO uint32_t *)APP_FLASH_STARTADDR;
    uint32_t app_resetHandler = *(__IO uint32_t *)(APP_FLASH_STARTADDR + 4);

    if ((app_stack_top & 0x2ffe0000u) != 0x20000000u)
    {
        return false;
    }
    if ((app_resetHandler < 0x08000000u) || (app_resetHandler > 0x080fffffu))
    {
        return false;
    }
    return true;
}

void Bootloader_JumpToApp(void)
{
    if (!isValid())
        return;
    
    uint32_t app_resetHandler = *(__IO uint32_t *)(APP_FLASH_STARTADDR + 4);
    uint32_t app_stack_top = *(__IO uint32_t *)APP_FLASH_STARTADDR;

    __disable_irq();
    //__set_PRIMASK(1);
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;
    for (uint8_t i = 0; i < 8; i++)
    {
        NVIC->ICER[i] = 0xffffffff;
        NVIC->ICPR[i] = 0xffffffff;
    }

    __enable_irq();
    //__set_PRIMASK(0);

    HAL_UART_DeInit(&huart2);
    SCB->VTOR = APP_FLASH_STARTADDR;
    __HAL_RCC_USART2_FORCE_RESET();     //Reset all Registers of USART2
    __HAL_RCC_USART2_RELEASE_RESET();

    __set_MSP(app_stack_top);
    __set_CONTROL(0);
    /* __ISB(); */

    typedef void (*pFunction)(void);
    pFunction jump_func = (pFunction)app_resetHandler;
    jump_func();

    while (1)
    {
        NVIC_SystemReset();
    }
}

//static bool Bootloader_Upgrade(void)
//{
//    uint8_t cmd;
//    uint8_t data_len;    //maxlen = 256bytes
//    uint32_t host_crc, host_size;
//    uint32_t calc_crc;
//
//    bool data_received = false;
//    uint32_t start_tick;
//
//    LED_ON;
//    cur_write_addr = APP_FLASH_STARTADDR;
//    cur_total_len = 0;
//
//    Flash_EraseApp();
//    /* start_tick = HAL_GetTick(); */
//
//    while (1)
//    {
//        //HAL_IWDG_Refresh(&hiwdg);
//
//        /* if (!data_received && (HAL_GetTick() - start_tick > UPGRADE_WAIT_TIMEOUT_MS))
//        {
//            LED_OFF;
//            return false;
//        } */
//
//        if (!Bootloader_ReceiveData(rx_buffer, 128 + 2, 500))
//        {
//            continue;
//        }
//        //LED_ON;
//        //Bootloader_ReceiveData(rx_buffer, 128 + 2, 500);
//        data_received = true;
//        cmd = rx_buffer[0];
//        data_len = rx_buffer[1];
//
//        if (cmd == CMD_DATA)
//        {
//            /* if (!Bootloader_ReceiveData(&data_len, 1, 100))
//            {
//                Bootloader_SendResponse(RESP_LEN_FAIL);
//                continue;
//            } */
//            if (data_len == 0 || data_len > RX_BUFFER_SIZE || (data_len % 4))
//            {
//                Bootloader_SendResponse(RESP_LEN_ERR);
//                continue;
//            }
//            /* if (!Bootloader_ReceiveData(rx_buffer, data_len, 500))
//            {
//                Bootloader_SendResponse(RESP_DATA_FAIL);
//                continue;
//            } */
//            if (!Flash_WriteBuffer(cur_write_addr, rx_buffer + 2, data_len))
//            {
//                Bootloader_SendResponse(RESP_WRITE_FAIL);
//                continue;
//            }
//            cur_write_addr += data_len;
//            cur_total_len += data_len;
//
//            /* __HAL_UART_CLEAR_FLAG(&huart2, UART_FLAG_RXNE);
//            __HAL_UART_CLEAR_FLAG(&huart2, UART_FLAG_IDLE); */
//            Bootloader_SendResponse(RESP_OK);
//        }
//        else if (cmd == CMD_END)
//        {
//            /* if (!Bootloader_ReceiveData((uint8_t *)&host_crc, 4, 100))
//            {
//                Bootloader_SendResponse(RESP_CRC_FAIL);
//                return false;
//            }
//            if (!Bootloader_ReceiveData((uint8_t *)&host_size, 4, 100))
//            {
//                Bootloader_SendResponse(RESP_SIZE_FAIL);
//                return false;
//            } */
//           host_crc = *(uint32_t *)&rx_buffer[1];
//           host_size  = *(uint32_t *)&rx_buffer[5];
//           if (cur_total_len != host_size)
//            {
//                Bootloader_SendResponse(RESP_SIZE_MISMATCH);
//                return false;
//            }
//            calc_crc = Bootloader_CRC32((uint8_t *)APP_FLASH_STARTADDR, host_size);
//            if (calc_crc == host_crc)
//            {
//                Bootloader_SendResponse(RESP_DONE);
//                LED_OFF;
//
//                uint32_t clear = 0xffffffff;
//                Flash_WriteBuffer(UPGRADE_FLAG_ADDR, (uint8_t *)&clear, 4);
//                
//                HAL_Delay(100);
//                //Bootloader_JumpToApp();
//
//                return true;
//            }
//            else
//            {
//                Bootloader_SendResponse(RESP_FAIL);
//                return false;
//            }
//        }
//        else if (cmd == CMD_ABORT)
//        {
//            //Bootloader_SendResponse(RESP_OK);
//            LED_OFF;
//            return false;
//        }
//    }
//}

static bool Bootloader_Upgrade(void)
{
    uint16_t frame_len;
    uint8_t rx_buffer[RX_BUFFER_SIZE + 2];
    uint8_t timeout_count = 0;
    bool first_transmit = true;

    cur_write_addr = APP_FLASH_STARTADDR;
    cur_total_len = 0;

    Flash_EraseApp();
    UpgradeState_Set(UPGRADE_STATE_RECEIVING);    //state: receiving data frames

    //LED_OFF;

    while (1)
    {
        HAL_IWDG_Refresh(&hiwdg);
        if (UpgradeState_Get() == UPGRADE_STATE_IDLE)    //if RECEIVING state was cleared
        {
            LED_ON;
            return false;
        }

        memset(rx_buffer, 0, RX_BUFFER_SIZE + 2);
        
        if (!Bootloader_ReceiveFrame(rx_buffer, &frame_len, 500))
        {
            timeout_count += 1;
            if (timeout_count >= 10)    //return after 500ms * 10 if no data is received
                return false;
            // 超时或出错，继续等待
            continue;
        }

        if (first_transmit)
        {
            LED_OFF;
            first_transmit = false;
        }

        uint8_t cmd = rx_buffer[0];
        uint8_t data_len;

        if (cmd == CMD_DATA)
        {
            data_len = rx_buffer[1];
            
            if (data_len == 0 || data_len > RX_BUFFER_SIZE || (data_len % 4) != 0)
            {
                Bootloader_SendResponse(RESP_LEN_ERR);
                continue;
            }

            if (!Flash_WriteBuffer(cur_write_addr, rx_buffer + 2, data_len))
            {
                Bootloader_SendResponse(RESP_WRITE_FAIL);
                continue;
            }

            cur_write_addr += data_len;
            cur_total_len += data_len;

            Bootloader_SendResponse(RESP_OK);
        }
        else if (cmd == CMD_END)
        {
            uint32_t host_crc = *(uint32_t *)&rx_buffer[1];
            uint32_t host_size = *(uint32_t *)&rx_buffer[5];

            if (cur_total_len != host_size)
            {
                Bootloader_SendResponse(RESP_SIZE_MISMATCH);
                UpgradeFlag_Clear();

                return false;
            }

            uint32_t calc_crc = Bootloader_CRC32((uint8_t *)APP_FLASH_STARTADDR, host_size);
            if (calc_crc == host_crc)
            {
                Bootloader_SendResponse(RESP_DONE);
                LED_ON;
                UpgradeFlag_Clear();
                HAL_Delay(100);
                
                if (isValid())
                {
                    return true;
                }
                return false;
            }
            else
            {
                Bootloader_SendResponse(RESP_CRC_FAIL);
                UpgradeFlag_Clear();

                return false;
            }
        }
        else if (cmd == CMD_ABORT)
        {
            Bootloader_SendResponse(RESP_OK);
            LED_OFF;
            UpgradeFlag_Clear();
            UpgradeState_Set(UPGRADE_STATE_ABORT);

            return false;
        }
    }
}

void Bootloader_MainLoop(void)
{
    LED_OFF;
    uint32_t upgrade_flag = *(__IO uint32_t *)UPGRADE_FLAG_ADDR;
    uint32_t upgrade_state = UpgradeState_Get();
    
    //the process of receiving firmware data was interrupted unexpectly
    if (upgrade_state == UPGRADE_STATE_RECEIVING && upgrade_flag != UPGRADE_FLAG_MAGIC)
    {
        UpgradeFlag_Clear();
        UpgradeState_Set(UPGRADE_STATE_TRANS_INTERRUPTED);   //upgrade state: interrupted unexpectedly
    }

    //Firmware upgrade initiated by the APP
    if (upgrade_flag == UPGRADE_FLAG_MAGIC)
    {
        UpgradeFlag_Clear();

        if (Bootloader_Upgrade())
        {
            if (isValid())
            {
                Bootloader_SendResponse(RESP_JUMP_TO_APP);
                HAL_Delay(100);
                Bootloader_JumpToApp();
            }
        }
    }

    //the APP is functioning correctly, the bootloader jumps directly to the APP upon power-up
    if (upgrade_flag == UPGRADE_FLAG_CLEAR && upgrade_state == UPGRADE_STATE_IDLE)
    {
        uint32_t start_tick = HAL_GetTick();
        uint8_t start_upgrade_frame[UPGRADE_START_FRAME_LEN];
        bool jump_to_app = true;

        while (HAL_GetTick() - start_tick < UPGRADE_COMFIRM_JUMPTOAPP_TIMOEOUT_MS)   //break after 2s
        {
            HAL_IWDG_Refresh(&hiwdg);
            Bootloader_SendResponse(RESP_COMFIRM_JUMP_TO_APP);
            if (HAL_UART_Receive(&huart2, start_upgrade_frame, UPGRADE_START_FRAME_LEN, 100) == HAL_OK)
            {
                uint8_t cmd = start_upgrade_frame[UPGRADE_START_FRAME_CMD_INDEX];
                //there may be an error int the app, so it need to confirm manually 
                //whether to jump to app or wait for the firmware upgrade
                if (cmd == UPGRADE_START_FRAME_CMD)
                {
                    //UpgradeState_Set(UPGRADE_STATE_WATING_UPGRADE);
                    jump_to_app = false;
                    break;
                }
            }
        }
        if (upgrade_flag == UPGRADE_FLAG_CLEAR && upgrade_state == UPGRADE_STATE_IDLE && jump_to_app)
        {
            if (isValid())
            {
                Bootloader_SendResponse(RESP_JUMP_TO_APP);
                HAL_Delay(100);
                Bootloader_JumpToApp();
            }
        }
    }

    LED_ON;
    uint32_t last_led_toggle = HAL_GetTick();
    Bootloader_SendResponse(RESP_WAIT_FOR_BOOT);

    while (1)
    {
        HAL_IWDG_Refresh(&hiwdg);

        if (HAL_GetTick() - last_led_toggle >= 200)
        {
            HAL_GPIO_TogglePin(LED_STATE_GPIO_Port, LED_STATE_Pin);
            last_led_toggle = HAL_GetTick();
        }

        uint8_t start_upgrade_frame[UPGRADE_START_FRAME_LEN];
        if (HAL_UART_Receive(&huart2, start_upgrade_frame, UPGRADE_START_FRAME_LEN, 100) == HAL_OK)
        {
            uint8_t cmd = start_upgrade_frame[UPGRADE_START_FRAME_CMD_INDEX];
            if (cmd == UPGRADE_START_FRAME_CMD)
            {
                if (Bootloader_Upgrade())
                {
                    if (isValid())
                    {
                        HAL_Delay(100);
                        Bootloader_JumpToApp();
                    }
                }
                LED_ON;
            }
        }
        HAL_Delay(10);
    }
}
