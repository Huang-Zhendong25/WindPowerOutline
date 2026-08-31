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

#define LASER_ON            0x01
#define LASER_OFF           0x00

#define SYS_INFO_SERIAL_NUMBER   "FD202606001"
#define SYS_INFO_FIRMWARE_VERSION "V1.0.0"

#define MAX_DAC_VALUE 757
#define MAX_POWER_LEVEL 4095
#define MAX_CURRENT 3.5f
#define RSNS_VALUE    0.068f
#define RIPPLE_CURRENT 0.05f

#define CONFIG_INFO_START_ADDR      0x0800f400     //flash address of configuration information
#define CONFIG_INFO_WORD_SIZE       (sizeof(saved_config_info) / 4)

#define FIRMWIRE_UPGRADE_FLAG_ADDR      0x0800f000
#define FIRMWIRE_UPGRADE_FLAGE_MAGIC    0x5a5a5a5a
#define APP_FLASH_STARTADDR             0x08004000
#define APP_MAX_SIZE                    0x0000a000

typedef struct
{
    uint8_t all_laser_status;      // Laser status for all blades
    uint8_t each_laser_status[3];   // Laser status for each blade
    uint8_t control_mode;      // Control mode (manual or serial)
    uint8_t blade_power_level[3][2]; // Power level for each blade
    float laser_temperature[3]; // Temperature for each laser
    char serial_number[11]; // Serial number of the device
    char firmware_version[6]; // Firmware version of the device
} sys_info;

typedef struct 
{
    uint8_t each_laser_status[3];
    uint8_t control_mode;
    uint8_t blade_power_level[3][2];
    
    uint8_t reverse[2];    //4-byte alignment
} saved_config_info;



void bsp_init(void);
void Laser_Enable(uint8_t bladenums);
void Laser_Disable(uint8_t bladenums);
bool Laser_Set_Brightness(uint8_t bladenums, float Brightness);
bool Laser_Set_PowerLevel(uint8_t bladenums, uint16_t powerlevel);
void RS485_Receive2Transmit(uint8_t Rs485Num);
void RS485_Transmit2Receive(uint8_t Rs485Num);
bool RS485_RespondFrame(uint8_t rs485_num, uint8_t cmd, uint8_t datasize, uint8_t *data);
bool ConfigInfo_Load(saved_config_info *info);
bool ConfigInfo_Save(const saved_config_info *info);
bool ConfigInfo_ResetToDefault(void);
bool ConfigInfo_ConfigureSys(void);

#endif
