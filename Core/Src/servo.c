#include "servo.h"
#include "tim.h"

/* ============================================================
 * 2-servo angle control with quintic polynomial easing
 *
 * Servo 1: TIM2_CH3 (PA2)
 * Servo 2: TIM2_CH4 (PA3)
 *
 * Angle input: 0-2700 (0.1 deg units) -> pulse 500-2500 us
 * Easing: f(t) = 6t^5 - 15t^4 + 10t^3  (zero vel & accel at endpoints)
 *
 * Usage:
 *   Servo_Init();            // call once (starts PWM, centers servos)
 *   Servo_SetAngle(1, 900);  // servo 1 -> 90.0 deg (non-blocking)
 *   Servo_Update();          // call every 10ms in task loop
 * ============================================================
 */

/* Per-servo easing state */
typedef struct {
    bool     easing;
    uint16_t cur_pulse;
    uint16_t start_pulse;
    uint16_t target_pulse;
    uint32_t start_tick;
    uint32_t ease_dur_ms;
} ServoEase_t;

static ServoEase_t s_servo[2];  /* index 0 = servo1, 1 = servo2 */

/* ---- Quintic easing core ---- */
/* f(t) = t^3 * (10 + t * (-15 + 6t)),  t in [0,1] */
static float quintic_ease(float t)
{
    float t3 = t * t * t;
    return t3 * (10.0f + t * (-15.0f + 6.0f * t));
}

/* angle_deg10 (0-2700) -> pulse (500-2500 us) */
static uint16_t angle_to_pulse(uint16_t angle_deg10)
{
    if (angle_deg10 > SERVO_ANGLE_MAX) angle_deg10 = SERVO_ANGLE_MAX;
    return (uint16_t)(SERVO_PULSE_MIN
        + ((uint32_t)angle_deg10 * (SERVO_PULSE_MAX - SERVO_PULSE_MIN))
          / SERVO_ANGLE_MAX);
}

/**
 * @brief  Start TIM2 CH3+CH4 PWM, init easing state to min pulse
 */
void Servo_Init(void)
{
    HAL_TIM_PWM_Start(&htim2, SERVO1_CHANNEL);
    HAL_TIM_PWM_Start(&htim2, SERVO2_CHANNEL);

    for (int i = 0; i < 2; i++) {
        s_servo[i].easing       = false;
        s_servo[i].cur_pulse    = SERVO_PULSE_MIN;
        s_servo[i].start_pulse  = SERVO_PULSE_MIN;
        s_servo[i].target_pulse = SERVO_PULSE_MIN;
        s_servo[i].start_tick   = 0;
        s_servo[i].ease_dur_ms  = 600;
    }

    __HAL_TIM_SET_COMPARE(&htim2, SERVO1_CHANNEL, SERVO_PULSE_MIN);
    __HAL_TIM_SET_COMPARE(&htim2, SERVO2_CHANNEL, SERVO_PULSE_MIN);
}

/**
 * @brief  Set target angle for a servo (non-blocking, starts easing)
 * @param  servo_id  1 or 2
 * @param  angle_deg10  target angle in 0.1 deg units (0-2700)
 *
 * If a previous easing is still running, re-targets from the
 * current actual pulse position so motion stays continuous.
 */
void Servo_SetAngle(uint8_t servo_id, uint16_t angle_deg10)
{
    if (servo_id < 1 || servo_id > 2) return;
    ServoEase_t *s = &s_servo[servo_id - 1];

    uint16_t target = angle_to_pulse(angle_deg10);

    /* Dynamic easing duration proportional to pulse difference */
    int32_t diff = (int32_t)target - (int32_t)s->cur_pulse;
    if (diff < 0) diff = -diff;
    s->ease_dur_ms = (uint32_t)((float)diff * SERVO_EASE_MS_PER_US);
    if (s->ease_dur_ms < SERVO_EASE_MIN_MS)
        s->ease_dur_ms = SERVO_EASE_MIN_MS;

    /* Start from current actual position (no jump on re-target) */
    s->start_pulse  = s->cur_pulse;
    s->target_pulse = target;
    s->start_tick   = HAL_GetTick();
    s->easing       = true;
}

/**
 * @brief  Easing state machine, call every ~10ms from ServoTask
 *
 * For each servo currently easing:
 *   t = elapsed / duration, clamped to [0,1]
 *   pulse = start + (target - start) * f(t)
 * When t >= 1, snap to target and stop easing.
 */
void Servo_Update(void)
{
    const uint32_t ch[2] = { SERVO1_CHANNEL, SERVO2_CHANNEL };

    for (int i = 0; i < 2; i++) {
        ServoEase_t *s = &s_servo[i];
        if (!s->easing) continue;

        uint32_t elapsed = HAL_GetTick() - s->start_tick;

        if (elapsed >= s->ease_dur_ms) {
            s->cur_pulse = s->target_pulse;
            s->easing    = false;
        } else {
            float t      = (float)elapsed / (float)s->ease_dur_ms;
            float factor = quintic_ease(t);
            int32_t delta = (int32_t)s->target_pulse
                          - (int32_t)s->start_pulse;
            s->cur_pulse = (uint16_t)((float)s->start_pulse
                                      + (float)delta * factor);
        }

        __HAL_TIM_SET_COMPARE(&htim2, ch[i], s->cur_pulse);
    }
}
