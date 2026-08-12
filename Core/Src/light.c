#include "light.h"
#include "tim.h"

/* ============================================================
 * 4 fill lights, 1kHz PWM
 *
 *   Light 1 = TIM3_CH1 = PB4   ← 常亮 (上电即100%, 通信不可控)
 *   Light 2 = TIM3_CH2 = PB5   ← 初始灭, 通信控制, 只支持0/100
 *   Light 3 = TIM3_CH3 = PB0   ← 初始灭, 通信控制, 只支持0/100
 *   Light 4 = TIM5_CH4 = PA3   ← 初始灭, 通信控制, 只支持0/100
 *
 * TIM3 / TIM5 config:
 *   Prescaler = 83, Period (ARR) = 999
 *   -> 84MHz / 84 / 1000 = 1kHz PWM
 *   -> duty resolution = 0.1% (compare 0-1000)
 *
 * Usage:
 *   Light_Init();                 // start 4-channel PWM
 *   Light_SetDefault();           // PB4=100%常亮, 2/3/4灭 (上电默认)
 *   Light_SetCommLights(1);       // 通信命令: 2/3/4全亮100%
 *   Light_SetBright(4, 80);       // light 4 (PA3) -> 80% (仅内部用)
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

    /* 上电默认状态: PB4常亮100%, 其余3个灭 (compare默认0) */
    Light_SetDefault();
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

/**
 * @brief  Default startup state:
 *         PB4 (light 1) always-on 100%, others (2,3,4) off
 */
void Light_SetDefault(void)
{
    Light_SetBright(1, 100);   /* PB4 常亮 */
    Light_SetBright(2, 0);     /* PB5 初始灭 */
    Light_SetBright(3, 0);     /* PB0 初始灭 */
    Light_SetBright(4, 0);     /* PA3 初始灭 */
}

/**
 * @brief  Communication-controlled lights (2,3,4) all on/off.
 *         Light 1 (PB4) is always-on and not touched.
 * @param  on  1 = 100%, 0 = off
 */
void Light_SetCommLights(uint8_t on)
{
    uint8_t bright = on ? 100 : 0;
    Light_SetBright(2, bright);
    Light_SetBright(3, bright);
    Light_SetBright(4, bright);
}
