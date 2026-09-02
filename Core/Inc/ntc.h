#ifndef __NTC_H
#define __NTC_H

#include "main.h"

#define NTC_FIXED_RESISTENCE    10000.0f
#define NTC_B_VALUE             3950.0f
#define NTC_REF_TEMPERATURE     25.0f
#define NTC_REF_TEMP_RESISTENCE 10000.0f

#define NTC_FILTER_LEN          5

void NTC_Init(void);
void NTC_GetTemperature(float *filtered_temperatures);


#endif // !__NTC_H
