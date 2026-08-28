#include "ntc.h"
#include "adc_dma.h"
#include <math.h>

static float temperature_filter_buffer[ADC_DMA_BUFFER_SIZE][NTC_FILTER_LEN];
static uint8_t filter_idx = 0;
static uint8_t filter_count = 0;

void NTC_Init(void)
{
    for (uint8_t adc_channel = 0; adc_channel < ADC_DMA_BUFFER_SIZE; adc_channel++)
    {
        for (uint8_t i = 0; i < NTC_FILTER_LEN; i++)
        {
            temperature_filter_buffer[adc_channel][i] = 0.0f;
        }
    }
    
    filter_idx = 0;
    filter_count = 0;
}

static float ADC_To_Resistance(uint32_t adc_value)
{
    if (adc_value >= 4095)
    {
        return NTC_FIXED_RESISTENCE * 100.0f;
    }
    return (adc_value / (4095.0f - adc_value)) * NTC_FIXED_RESISTENCE;
}

static float Resistance_To_Temperature(float resistance)
{
    float temperature = (1.0 / ((1.0 / NTC_B_VALUE) * log(resistance / NTC_REF_TEMP_RESISTENCE) +
     (1.0 / (NTC_REF_TEMPERATURE + 273.15))) - 273.15);

    return temperature;
}

static void NTC_SampleOnce(float *temperature_values)
{
    uint16_t adc_values[ADC_DMA_BUFFER_SIZE];
    uint32_t adc_raw[ADC_DMA_BUFFER_SIZE];
    float resistance[ADC_DMA_BUFFER_SIZE];
    
    ADC_DMA_GetValues(adc_values);

    for (uint8_t adc_channel = 0; adc_channel < ADC_DMA_BUFFER_SIZE; adc_channel++)
    {
        adc_raw[adc_channel] = adc_values[adc_channel];
        resistance[adc_channel] = ADC_To_Resistance(adc_raw[adc_channel]);
        temperature_values[adc_channel] = Resistance_To_Temperature(resistance[adc_channel]);
    }
}

static void NTC_Filter(float *new_sample, float *filtered_temperatures)
{
    if (filter_count < NTC_FILTER_LEN)
    {
        filter_count += 1;
    }

    for (uint8_t adc_channel = 0; adc_channel < ADC_DMA_BUFFER_SIZE; adc_channel++)
    {
        float sum = 0.0f;

        temperature_filter_buffer[adc_channel][filter_idx] = new_sample[adc_channel];
        
        for (uint8_t i = 0; i < filter_count; i++)
        {
            sum += temperature_filter_buffer[adc_channel][i];
        }
        filtered_temperatures[adc_channel] = sum / filter_count;
    }

    filter_idx = (filter_idx + 1) % NTC_FILTER_LEN;
}

void NTC_GetTemperature(float *filtered_temperatures)
{
    float temperatures[ADC_DMA_BUFFER_SIZE];
    //float filtered_temperatures[ADC_DMA_BUFFER_SIZE];

    if (!ADC_DMA_IsComplete())
    {
        /* filtered_temperatures[0] = 0.0f;
        filtered_temperatures[1] = 0.0f;
        filtered_temperatures[2] = 0.0f; */
        return;
    }
    NTC_SampleOnce(temperatures);
    NTC_Filter(temperatures, filtered_temperatures);
}
