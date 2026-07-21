#include "light.h"
#include "tim.h"

/* ============================================================
 * 3 fill lights on TIM3 (1kHz PWM)
 *
 *   Light 1 = TIM3_CH1 = PB4
 *   Light 2 = TIM3_CH2 = PB5
 *   Light 3 = TIM3_CH3 = PB0
 *
 * CubeMX TIM3 config required:
 *   Prescaler = 83, Period (ARR) = 999
 *   -> 84MHz / (83+1) / (999+1) = 1kHz PWM
 *   -> duty resolution = 0.1% (compare 0-1000)
 *
 * Usage:
 *   Light_Init();              // start 3-channel PWM
 *   Light_SetBright(1, 80);   // light 1 -> 80%
 *   Light_SetAll(100);        // all lights -> 100%
 * ============================================================
 */

static const uint32_t s_light_ch[3] = {
    TIM_CHANNEL_1,   /* PB4 - Light 1 */
    TIM_CHANNEL_2,   /* PB5 - Light 2 */
    TIM_CHANNEL_3    /* PB0 - Light 3 */
};

/**
 * @brief  Start TIM3 CH1/CH2/CH3 PWM output
 */
void Light_Init(void)
{
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
}

/**
 * @brief  Set brightness of one fill light
 * @param  light_id  1, 2, or 3
 * @param  bright    0 (off) to 100 (full)
 */
void Light_SetBright(uint8_t light_id, uint8_t bright)
{
    if (light_id < 1 || light_id > 3) return;
    if (bright > 100) bright = 100;
    /* ARR=999: compare = bright * 10  (0-1000 duty) */
    __HAL_TIM_SET_COMPARE(&htim3, s_light_ch[light_id - 1],
                          (uint32_t)bright * 10);
}

/**
 * @brief  Set brightness of all 3 fill lights
 * @param  bright  0 (off) to 100 (full)
 */
void Light_SetAll(uint8_t bright)
{
    for (uint8_t i = 1; i <= 3; i++) {
        Light_SetBright(i, bright);
    }
}
