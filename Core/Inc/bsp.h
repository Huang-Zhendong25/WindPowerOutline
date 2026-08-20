#ifndef __BSP_H
#define __BSP_H

#include "main.h"

#define LASER_NUMS  3
#define BLADE_NUMS  3

#define LASER_NUM1  GPIO_PIN_9
#define LASER_NUM2  GPIO_PIN_8
#define LASER_NUM3  GPIO_PIN_7

#define LASER_BRIGHTNESS_LEVEL0     0.0f
#define LASER_BRIGHTNESS_LEVEL1     0.5f
#define LASER_BRIGHTNESS_LEVEL2     1.0f
#define LASER_BRIGHTNESS_LEVEL3     1.5f

#define BLADE_NUM1          0x01
#define BLADE_NUM2          0x02
#define BLADE_NUM3          0x04
#define BLADE_NUM_ALL       0x07

#define RS485_NUMS          2
#define RS485_NUM1          0x01
#define RS485_NUM2          0x02

#define RS485_FRAME_MAX_LEN     60

#define CONTROL_MODE_MANUAL      0x00
#define CONTROL_MODE_SERIAL      0x01

#define SYS_INFO_SERIAL_NUMBER   "FD202606001"
#define SYS_INFO_FIRMWARE_VERSION "V1.0.0"

#define MAX_DAC_VALUE 757
#define MAX_POWER_LEVEL 4095
#define MAX_CURRENT 3.5f
#define RSNS_VALUE    0.068f
#define RIPPLE_CURRENT 0.05f

typedef struct
{
    uint8_t laser_status;      // Laser status for each blade
    uint8_t control_mode;      // Control mode (manual or serial)
    uint8_t blade1_power_level[2]; // Power level for blade 1
    uint8_t blade2_power_level[2]; // Power level for blade 2
    uint8_t blade3_power_level[2]; // Power level for blade 3
    char serial_number[11]; // Serial number of the device
    char firmware_version[6]; // Firmware version of the device
} sys_info;


void bsp_init(void);
void Laser_Enable(uint8_t bladenums);
void Laser_Disable(uint8_t bladenums);
bool Laser_Set_Brightness(uint8_t bladenums, float Brightness);
bool Laser_Set_PowerLevel(uint8_t bladenums, uint16_t powerlevel);
void RS485_Receive2Transmit(uint8_t Rs485Num);
void RS485_Transmit2Receive(uint8_t Rs485Num);
bool RS485_RespondFrame(uint8_t rs485_num, uint8_t cmd, uint8_t datasize, uint8_t *data);

#endif
