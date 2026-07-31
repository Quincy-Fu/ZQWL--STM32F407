#include "light.h"
#include "tim.h"

/* ============================================================
 * 4 fill lights, 1kHz PWM
 *
 *   Light 1 = TIM3_CH1 = PB4
 *   Light 2 = TIM3_CH2 = PB5
 *   Light 3 = TIM3_CH3 = PB0
 *   Light 4 = TIM5_CH4 = PA3   (USER CODE 初始化, 不受 CubeMX 重生成)
 *
 * TIM3 / TIM5 config:
 *   Prescaler = 83, Period (ARR) = 999
 *   -> 84MHz / 84 / 1000 = 1kHz PWM
 *   -> duty resolution = 0.1% (compare 0-1000)
 *
 * Usage:
 *   Light_Init();              // start 4-channel PWM
 *   Light_SetBright(4, 80);   // light 4 (PA3) -> 80%
 *   Light_SetAll(100);        // all 4 lights -> 100%
 * ============================================================
 */

void Light_Init(void)
{
    /* TIM5 CH4 (PA3) 初始化 — 在 USER CODE 区, 不依赖 CubeMX */
    MX_TIM5_Init();

    /* TIM3 CH1/CH2/CH3 (PB4/PB5/PB0) */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
}

/**
 * @brief  Set brightness of one fill light
 * @param  light_id  1, 2, 3 (TIM3) or 4 (TIM5/PA3)
 * @param  bright    0 (off) to 100 (full)
 */
void Light_SetBright(uint8_t light_id, uint8_t bright)
{
    if (light_id < 1 || light_id > 4) return;
    if (bright > 100) bright = 100;
    /* ARR=999: compare = bright * 10  (0-1000 duty) */
    uint32_t cmp = (uint32_t)bright * 10;
    if (light_id <= 3) {
        static const uint32_t ch[3] = {
            TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3
        };
        __HAL_TIM_SET_COMPARE(&htim3, ch[light_id - 1], cmp);
    } else {
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, cmp);
    }
}

/**
 * @brief  Set brightness of all 4 fill lights
 * @param  bright  0 (off) to 100 (full)
 */
void Light_SetAll(uint8_t bright)
{
    for (uint8_t i = 1; i <= 4; i++) {
        Light_SetBright(i, bright);
    }
}
