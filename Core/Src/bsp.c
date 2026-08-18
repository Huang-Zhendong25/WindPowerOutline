#include "bsp.h"
#include "MS5314.h"
#include "LM3409.h"

extern uint8_t ms5314_channels[];
uint8_t blade_numbers[BLADE_NUMS] = {BLADE_NUM1, BLADE_NUM2, BLADE_NUM3};
uint16_t laser_numbers[LASER_NUMS] = {LASER_NUM1, LASER_NUM2, LASER_NUM3};

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

void Laser_Set_Brightness(uint8_t bladenums, uint8_t Brightness)
{
    // Implementation for setting laser brightness
    uint8_t channel = 0;

    for (uint8_t i = 0; i < BLADE_NUMS; i++)
    {
        if (bladenums & blade_numbers[i])
        {
            channel |= ms5314_channels[BLADE_NUMS - 1 - i];
        }
    }
    MS5314_Set_Voltage(channel, Brightness, MS5314_UPDATE_OUTPUT);
}

