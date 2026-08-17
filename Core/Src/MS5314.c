#include "MS5314.h"

static SPI_HandleTypeDef *hspi_ms5314;
uint8_t ms5314_channels[] = {MS5314_CHANNEL_A, MS5314_CHANNEL_B, MS5314_CHANNEL_C, MS5314_CHANNEL_D};

void MS5314_Init(SPI_HandleTypeDef *hspi)
{
    hspi_ms5314 = hspi;
}

void MS5314_Write(uint8_t channel, uint8_t mode, uint8_t sync, uint16_t voltage)
{
    uint8_t update = sync ? MS5314_UPDATE_OUTPUT : MS5314_UPDATE_REG;
    uint16_t ms5314_frame = MS5314_FRAME(channel, mode, update, voltage);
    HAL_SPI_Transmit(hspi_ms5314, (uint8_t *)&ms5314_frame, 2, HAL_MAX_DELAY);
}

void MS5314_Set_Voltage(uint8_t channel, uint16_t voltage, uint8_t sync)
{
    for (uint8_t i = 0; i < MS5314_CHANNEL_NUM; i++)
    {
        if (channel & ms5314_channels[i])
        {
            MS5314_Write(ms5314_channels[i], MS5314_MODE_NORMAL, sync, voltage);
        }
    }
}

