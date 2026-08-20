#include "MS5314.h"

static SPI_HandleTypeDef *hspi_ms5314;
uint8_t ms5314_channels[] = {MS5314_CHANNEL_A, MS5314_CHANNEL_B, MS5314_CHANNEL_C, MS5314_CHANNEL_D};
uint8_t ms5314_channel_addresses[] = {MS5314_CHANNEL_ADDR_A, MS5314_CHANNEL_ADDR_B, MS5314_CHANNEL_ADDR_C, MS5314_CHANNEL_ADDR_D};

void MS5314_Init(SPI_HandleTypeDef *hspi)
{
    hspi_ms5314 = hspi;

    //MS5314_Set_Voltage(MS5314_CHANNEL_ALL, 0.0f, true);
}

bool MS5314_Write(uint8_t channel_addr, uint8_t mode, const float *voltage, const uint16_t *dec_value, const bool sync)
{
    uint8_t update = sync ? MS5314_UPDATE_OUTPUT : MS5314_UPDATE_REG;
    uint16_t ms5314_frame;
    uint8_t tx_buffer[2];

    if (voltage)
    {
        ms5314_frame = MS5314_FRAME(channel_addr, mode, update, *voltage);
    }
    else
    {
        ms5314_frame = MS5314_FRAME_BY_DEC(channel_addr, mode, update, *dec_value);
    }

    tx_buffer[0] = (ms5314_frame >> 8) & 0xFF; // High byte
    tx_buffer[1] = ms5314_frame & 0xFF;        // Low byte
    HAL_GPIO_WritePin(SPI_NSS_GPIO_Port, SPI_NSS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SPI_NSS_GPIO_Port, SPI_NSS_Pin, GPIO_PIN_RESET);
    if (HAL_SPI_Transmit(hspi_ms5314, tx_buffer, 2, HAL_MAX_DELAY) != HAL_OK)
    {
        return false;
    }
    HAL_GPIO_WritePin(SPI_NSS_GPIO_Port, SPI_NSS_Pin, GPIO_PIN_SET);
    return true;
}

bool MS5314_Set_Voltage(uint8_t channel, float voltage, bool sync)
{
    voltage = voltage > 1.5f ? 1.5f : (voltage < 0.0f ? 0.0f : voltage); // Clamp voltage to [0.0, 1.5]
    for (uint8_t channel_idx = 0; channel_idx < MS5314_CHANNEL_NUM; channel_idx++)
    {
        if (channel & ms5314_channels[channel_idx])
        {
            if (!MS5314_Write(ms5314_channel_addresses[channel_idx], MS5314_MODE_NORMAL, &voltage, NULL, sync))
            {
                return false;
            }
        }
    }
    return true;
}

bool MS5314_Set_DECValue(uint8_t channel, uint16_t dec_value, bool sync)
{
    if (dec_value > 1023)
    {
        return false; // Invalid DEC value
    }
    for (uint8_t channel_idx = 0; channel_idx < MS5314_CHANNEL_NUM; channel_idx++)
    {
        if (channel & ms5314_channels[channel_idx])
        {
            if (!MS5314_Write(ms5314_channel_addresses[channel_idx], MS5314_MODE_NORMAL, NULL, &dec_value, sync))
            {
                return false;
            }
        }
    }
    return true;
}
