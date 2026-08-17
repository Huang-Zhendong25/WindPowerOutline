#include "LM3409.h"

void LM3409_Enable(uint16_t enable_pins)
{
    // Implementation for enabling LM3409
    HAL_GPIO_WritePin(LM3409_GPIOX, enable_pins, GPIO_PIN_SET);
}

void LM3409_Disable(uint16_t disable_pins)
{
    // Implementation for disabling LM3409
    HAL_GPIO_WritePin(LM3409_GPIOX, disable_pins, GPIO_PIN_RESET);
}
