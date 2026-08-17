#ifndef __MS5314_H
#define __MS5314_H

#include "main.h"

#define MS5314_CHANNEL_NUM          4
#define MS5314_CHANNEL_A			0x00
#define MS5314_CHANNEL_B			0x01
#define MS5314_CHANNEL_C			0x10
#define MS5314_CHANNEL_D			0x11

#define MS5314_MODE_NORMAL		    0x01
#define MS5314_MODE_PWEDOWN		    0x00

#define MS5314_UPDATE_REG			0x01
#define MS5314_UPDATE_OUTPUT	    0x00

#define MS5314_REF_VOLTAGE		    1.65f

#define MS5314_VOLTAGE_TO_DATA(voltage)	((uint16_t)(voltage * 1024.0f / MS5314_REF_VOLTAGE))

#define MS5314_FRAME(channel, mode, update, voltage)	((uint16_t)((((channel) & 0x03) << 14) | \
                                                            (((mode) & 0x01) << 13) | \
                                                            (((update) & 0x01) << 12) | \
                                                            ((MS5314_VOLTAGE_TO_DATA(voltage) & 0x03FF) << 2)))

void MS5314_Init(SPI_HandleTypeDef *hspi);
void MS5314_Write(uint8_t channel, uint8_t mode, uint8_t sync, uint16_t voltage);
void MS5314_Set_Voltage(uint8_t channel, uint16_t voltage, uint8_t sync);

#endif
