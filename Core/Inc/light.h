#ifndef __LIGHT_H
#define __LIGHT_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

/*
 * 4 fill lights, 1kHz PWM:
 *   Light 1 = TIM3_CH1 = PB4
 *   Light 2 = TIM3_CH2 = PB5
 *   Light 3 = TIM3_CH3 = PB0
 *   Light 4 = TIM5_CH4 = PA3   (USER CODE 初始化)
 *
 * TIM3 / TIM5: Prescaler=83, ARR=999 → 84MHz/84/1000 = 1kHz
 */

void Light_Init(void);
void Light_SetBright(uint8_t light_id, uint8_t bright);  /* light_id: 1-4, bright: 0-100 */
void Light_SetAll(uint8_t bright);                       /* set all 4 lights */

#endif
