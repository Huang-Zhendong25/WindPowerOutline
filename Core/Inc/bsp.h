#ifndef __BSP_H
#define __BSP_H

#include "main.h"

#define LASER_NUMS  3
#define BLADE_NUMS  3

#define LASER_NUM1  GPIO_PIN_9
#define LASER_NUM2  GPIO_PIN_8
#define LASER_NUM3  GPIO_PIN_7

#define LASER_BRIGHTNESS_LEVEL0     0.0f
#define LASER_BRIGHTNESS_LEVEL1     0.5f
#define LASER_BRIGHTNESS_LEVEL2     1.0f
#define LASER_BRIGHTNESS_LEVEL3     1.5f

#define BLADE_NUM1          0x01
#define BLADE_NUM2          0x02
#define BLADE_NUM3          0x04
#define BLADE_NUM_ALL       0x07

void bsp_init(void);
void Laser_Enable(uint8_t bladenums);
void Laser_Disable(uint8_t bladenums);
void Laser_Set_Brightness(uint8_t bladenums, float Brightness);

#endif
