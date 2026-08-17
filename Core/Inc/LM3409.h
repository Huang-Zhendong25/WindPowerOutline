#ifndef __LM3409_H
#define __LM3409_H

#include "main.h"

#define LM3409_GPIOX    GPIOB   

void LM3409_Enable(uint16_t enable_pins);
void LM3409_Disable(uint16_t disable_pins);

#endif
