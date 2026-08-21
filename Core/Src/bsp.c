#include <string.h>
#include "bsp.h"
#include "MS5314.h"
#include "LM3409.h"
#include "RS485.h"
#include "adc_dma.h"
#include "dma.h"
#include "flash_drv.h"


extern sys_info system_info;
extern uint8_t ms5314_channels[];
uint8_t blade_numbers[BLADE_NUMS] = {BLADE_NUM1, BLADE_NUM2, BLADE_NUM3};
uint16_t laser_numbers[LASER_NUMS] = {LASER_NUM1, LASER_NUM2, LASER_NUM3};
uint8_t rs485_numbers[RS485_NUMS] = {RS485_NUM1, RS485_NUM2};
uint8_t rs485_en_pins[RS485_NUMS] = {RS485_EN1_Pin, RS485_EN2_Pin};
uint16_t maxDACvalue = MAX_DAC_VALUE;
static const config_info default_config_info = {
    .blade_power_levels = {{0}},
};


void bsp_init(void)
{
    MS5314_Init(&hspi2);
    ADC_DMA_Init(&hadc1);
    //MS5314_Set_Voltage(MS5314_CHANNEL_ALL, 0.0f, true);
    //Laser_Disable(BLADE_NUM_ALL);
    memcpy(system_info.serial_number, SYS_INFO_SERIAL_NUMBER, sizeof(system_info.serial_number));
    memcpy(system_info.firmware_version, SYS_INFO_FIRMWARE_VERSION, sizeof(system_info.firmware_version));

    //Laser_PowerLevel_Calculate(MAX_CURRENT, RIPPLE_CURRENT, RSNS_VALUE);
}

void Laser_PowerLevel_Calculate(float maxCurrent, float CurrentRipple, float RSNS)
{
    float VoltageDAC = 5.0 * RSNS * (maxCurrent + CurrentRipple / 2.0f);
    maxDACvalue = (uint16_t)(VoltageDAC * 1024.0f / MS5314_REF_VOLTAGE);
}

void Laser_Enable(uint8_t bladenums)
{
    // Implementation for enabling laser
    uint16_t LaserEnablePins = 0;

    for (uint8_t i = 0; i < LASER_NUMS; i++)
    {
        if (bladenums & blade_numbers[i])
        {
            LaserEnablePins |= laser_numbers[i];
        }
    }
    LM3409_Enable(LaserEnablePins);
}

void Laser_Disable(uint8_t bladenums)
{
    // Implementation for disabling laser
    uint16_t LaserDisablePins = 0;

    for (uint8_t i = 0; i < LASER_NUMS; i++)
    {
        if (bladenums & blade_numbers[i])
        {
            LaserDisablePins |= laser_numbers[i];
        }
    }
    LM3409_Disable(LaserDisablePins);
}

bool Laser_Set_Brightness(uint8_t bladenums, float Brightness)
{
    uint8_t channel = 0;

    for (uint8_t i = 0; i < BLADE_NUMS; i++)
    {
        if (bladenums & blade_numbers[i])
        {
            channel |= ms5314_channels[BLADE_NUMS - 1 - i];
        }
    }
    if (!MS5314_Set_Voltage(channel, Brightness, true))
    {
        return false;
    }

    return true;
}

bool Laser_Set_PowerLevel(uint8_t bladenums, uint16_t powerlevel)
{
    uint8_t channel = 0;
    powerlevel = (powerlevel > MAX_POWER_LEVEL) ? MAX_POWER_LEVEL : powerlevel; // Clamp power level to MAX_POWER_LEVEL
    uint16_t dac_decval = (uint16_t)((powerlevel * maxDACvalue) / MAX_POWER_LEVEL); // Convert power level to DAC value

    for (uint8_t i = 0; i < BLADE_NUMS; i++)
    {
        if (bladenums & blade_numbers[i])
        {
            channel |= ms5314_channels[BLADE_NUMS - 1 - i];
        }
    }
    if (!MS5314_Set_DECValue(channel, dac_decval, true))
    {
        return false;
    }

    return true;
}

void RS485_Receive2Transmit(uint8_t Rs485Num)
{
    for (uint8_t rs485_idx = 0; rs485_idx < RS485_NUMS; rs485_idx++)
    {
        if (Rs485Num & rs485_numbers[rs485_idx])
        {
            RS485_Receive_To_Transmit(rs485_en_pins[rs485_idx]);
        }
    }
}

void RS485_Transmit2Receive(uint8_t Rs485Num)
{
    for (uint8_t rs485_idx = 0; rs485_idx < RS485_NUMS; rs485_idx++)
    {
        if (Rs485Num & rs485_numbers[rs485_idx])
        {
            RS485_Transmit_To_Receive(rs485_en_pins[rs485_idx]);
        }
    }
}

bool RS485_RespondFrame(uint8_t rs485_num, uint8_t cmd, uint8_t datasize, uint8_t *data)
{
    uint8_t frame[RS485_FRAME_MAX_LEN];

    if (datasize + RS485_FRAME_OVERHEAD_LEN > RS485_FRAME_MAX_LEN)
    {
        return false;
    }
    frame[0] = RS485_FRAME_START;
    frame[1] = RS485_FRAME_TYPE;
    frame[2] = cmd;
    frame[3] = datasize;
    memcpy(&frame[4], data, datasize);
    frame[4 + datasize] = RS485_CheckFrameSum(frame, datasize + RS485_FRAME_OVERHEAD_LEN);

    RS485_Receive2Transmit(rs485_num);
    if (!RS485_TransmitFrame(frame, datasize + RS485_FRAME_OVERHEAD_LEN))
    {
        RS485_Transmit2Receive(rs485_num);
        return false;
    }
    RS485_Transmit2Receive(rs485_num);
    return true;
}

bool ConfigInfo_Load(config_info *info)
{
    uint32_t flash_data[CONFIG_INFO_WORD_SIZE];
    
    bool status = Flash_ReadBuffer(CONFIG_INFO_START_ADDR, flash_data, CONFIG_INFO_WORD_SIZE);
    memcpy(info, flash_data, sizeof(config_info));

    return status;
}

bool ConfigInfo_Save(const config_info *info)
{
    uint32_t flash_data[CONFIG_INFO_WORD_SIZE];

    memcpy(flash_data, info, sizeof(config_info));

    return Flash_WriteBuffer(CONFIG_INFO_START_ADDR, flash_data, CONFIG_INFO_WORD_SIZE);
}

bool ConfigInfo_ResetToDefault(void)
{
    return ConfigInfo_Save(&default_config_info);
}
