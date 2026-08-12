#ifndef __LIGHT_H
#define __LIGHT_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

/*
 * 4 fill lights, 1kHz PWM:
 *   Light 1 = TIM3_CH1 = PB4   ← 常亮 (上电100%, 通信命令不影响)
 *   Light 2 = TIM3_CH2 = PB5   ← 初始灭, 通信控制, 只支持0/100
 *   Light 3 = TIM3_CH3 = PB0   ← 初始灭, 通信控制, 只支持0/100
 *   Light 4 = TIM5_CH4 = PA3   ← 初始灭, 通信控制, 只支持0/100
 *
 * TIM3 / TIM5: Prescaler=83, ARR=999 → 84MHz/84/1000 = 1kHz
 */

void Light_Init(void);
void Light_SetBright(uint8_t light_id, uint8_t bright);  /* light_id: 1-4, bright: 0-100 */
void Light_SetAll(uint8_t bright);                       /* set all 4 lights */
void Light_SetDefault(void);                             /* PB4=100常亮, 其余3个灭 */
void Light_SetCommLights(uint8_t on);                    /* 通信可控灯(2,3,4)整体开/关, 1=100% 0=灭 */

#endif
