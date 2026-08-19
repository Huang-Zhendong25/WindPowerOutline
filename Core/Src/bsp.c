#include "bsp.h"
#include "MS5314.h"
#include "LM3409.h"
#include "RS485.h"
#include <string.h>

extern sys_info system_info;
extern uint8_t ms5314_channels[];
uint8_t blade_numbers[BLADE_NUMS] = {BLADE_NUM1, BLADE_NUM2, BLADE_NUM3};
uint16_t laser_numbers[LASER_NUMS] = {LASER_NUM1, LASER_NUM2, LASER_NUM3};
uint8_t rs485_numbers[RS485_NUMS] = {RS485_NUM1, RS485_NUM2};
uint8_t rs485_en_pins[RS485_NUMS] = {RS485_EN1_Pin, RS485_EN2_Pin};

void bsp_init(void)
{
    MS5314_Init(&hspi2);
    MS5314_Set_Voltage(MS5314_CHANNEL_ALL, 0.0f, true);
    memcpy(system_info.serial_number, SYS_INFO_SERIAL_NUMBER, sizeof(system_info.serial_number));
    memcpy(system_info.firmware_version, SYS_INFO_FIRMWARE_VERSION, sizeof(system_info.firmware_version));
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
    MS5314_Set_Voltage(channel, Brightness, true);

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
