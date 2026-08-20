#include "adc_dma.h"
#include "dma.h"

static ADC_HandleTypeDef *hadc_dma;
uint16_t adc_dma_buffer[ADC_DMA_BUFFER_SIZE] = {0};  //the adc of stm32f103 is 12bit
volatile uint8_t adc_dma_complete = 0;

void ADC_DMA_Init(ADC_HandleTypeDef *hadc)
{
    hadc_dma = hadc;
    for (uint8_t i = 0; i < ADC_DMA_BUFFER_SIZE; i++)
    {
        adc_dma_buffer[i] = 0;
    }
    adc_dma_complete = 0;
}

void ADC_DMA_Start(void)
{
    adc_dma_complete = 0;
    HAL_ADC_Start_DMA(hadc_dma, (uint32_t*)adc_dma_buffer, ADC_DMA_BUFFER_SIZE);
}

uint8_t ADC_DMA_IsComplete(void)
{
    return adc_dma_complete;
}

void ADC_DMA_GetValues(uint16_t *buffer)
{
    for (uint8_t i = 0; i < ADC_DMA_BUFFER_SIZE; i++)
    {
        buffer[i] = adc_dma_buffer[i];
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1)
    {
        adc_dma_complete = 1;
    }
}

