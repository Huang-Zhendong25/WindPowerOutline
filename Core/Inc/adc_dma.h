#ifndef __ADC_DMA_H
#define __ADC_DMA_H

#include "main.h"

#define ADC_DMA_BUFFER_SIZE     3

extern uint16_t adc_dma_buffer[ADC_DMA_BUFFER_SIZE];  //the adc of stm32f103 is 12bit
extern volatile uint8_t adc_dma_complete;

void ADC_DMA_Init(ADC_HandleTypeDef *hadc);
void ADC_DMA_Start(void);
uint8_t ADC_DMA_IsComplete(void);
void ADC_DMA_GetValues(uint16_t *buffer);

#endif // !__ADC_DMA_H

