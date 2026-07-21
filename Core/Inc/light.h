#ifndef __LIGHT_H
#define __LIGHT_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

/*
 * 3 fill lights on TIM3 (1kHz PWM):
 *   Light 1 = TIM3_CH1 = PB4
 *   Light 2 = TIM3_CH2 = PB5
 *   Light 3 = TIM3_CH3 = PB0
 *
 * TIM3 must be configured in CubeMX:
 *   Prescaler = 83, Period (ARR) = 999  ->  84MHz / 84 / 1000 = 1kHz
 */

void Light_Init(void);
void Light_SetBright(uint8_t light_id, uint8_t bright);  /* light_id: 1-3, bright: 0-100 */
void Light_SetAll(uint8_t bright);                       /* set all 3 lights */

#endif
