#ifndef __BOOTLOADER_H
#define __BOOTLOADER_H

#include "main.h"
#include <stdint.h>

#define APP_FLASH_STARTADDR     0x08004000
#define APP_MAX_SIZE            0x0000a000    //40kb -> 0x0800e000
#define APP_MAX_PAGE_SIZE       32    //1kb * 32

#define UPGRADE_FLAG_ADDR       0x0800f000
#define UPGRADE_FLAG_MAGIC     0x5a5a5a5a
#define UPGRADE_FLAG_CLEAR     0xffffffff

#define UPGRADE_STATE_FLASH_ADDR    (UPGRADE_FLAG_ADDR + 4)
//#define UPGRADE_STATE_FLASH_ADDR    0x0800f400
#define UPGRADE_STATE_IDLE              0xffffffff
#define UPGRADE_STATE_RECEIVING         0xaaaaaaaa
#define UPGRADE_STATE_ABORT             0xbbbbbbbb
#define UPGRADE_STATE_TRANS_INTERRUPTED 0xcccccccc
#define UPGRADE_STATE_WATING_UPGRADE    0xdddddddd
//#define UPGRADE_STATE_CRC       0x55555555

//#define UPGRADE_WAIT_TIMEOUT_MS             3000
#define UPGRADE_RECEIVE_FRAME_TIMEOUT_MS    500
#define UPGRADE_COMFIRM_JUMPTOAPP_TIMOEOUT_MS   2000

#define CMD_DATA                0x01
#define CMD_END                 0x02
#define CMD_ABORT               0x03

//#define UPGRADE_WAIT_TIMEOUT_MS 2000

#define RESP_OK                 "OK"
#define RESP_LEN_FAIL           "LEN_FAIL"
#define RESP_LEN_ERR            "LEN_ERR"
#define RESP_DATA_FAIL          "DATA_FAIL"
#define RESP_WRITE_FAIL         "WRITE_FAIL"
#define RESP_CRC_FAIL           "CRC_FAIL"
#define RESP_SIZE_FAIL          "SIZE_FAIL"
#define RESP_SIZE_MISMATCH      "SIZE_MISMATCH"
#define RESP_DONE               "DONE"
#define RESP_FAIL               "FAIL"

#define RESP_BOOT_START             "BOOTSTART"
#define RESP_WAIT_FOR_BOOT          "WAITFORBOOT"
#define RESP_JUMP_TO_APP            "JUMPTOAPP"
#define RESP_COMFIRM_JUMP_TO_APP    "COMFIRM"

#define UPGRADE_START_FRAME_LEN         5
#define UPGRADE_START_FRAME_CMD_INDEX   2
#define UPGRADE_START_FRAME_CMD         0x45

#define RX_BUFFER_SIZE      128

#define LED_ON      HAL_GPIO_WritePin(LED_STATE_GPIO_Port, LED_STATE_Pin, GPIO_PIN_SET)
#define LED_OFF     HAL_GPIO_WritePin(LED_STATE_GPIO_Port, LED_STATE_Pin, GPIO_PIN_RESET)

typedef struct
{
    uint32_t upgrade_flag;
    uint32_t upgrade_state;
} UpgradeInfo_t;

void Bootloader_SendResponse(char *respText);
void Bootloader_MainLoop(void);

#endif // !__BOOTLOADER_H
