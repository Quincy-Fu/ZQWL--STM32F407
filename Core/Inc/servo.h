#ifndef __SERVO_H
#define __SERVO_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* ---- PWM channels (TIM2, 50Hz servo) ---- */
#define SERVO1_CHANNEL    TIM_CHANNEL_2   /* PA1 */
#define SERVO2_CHANNEL    TIM_CHANNEL_3   /* PA2 */

/* ---- Pulse range (us) ---- */
#define SERVO_PULSE_MIN   500
#define SERVO_PULSE_MAX   2500

/* ---- Angle range (0.1 deg units): 0-1800 = 0.0-180.0 deg ---- */
/* Change to 2700 if your servos are 270-degree type */
#define SERVO_ANGLE_MAX   1800

/* ---- Quintic easing parameters ---- */
#define SERVO_EASE_MS_PER_US  0.5f   /* ms per us pulse diff (full range ~1s) */
#define SERVO_EASE_MIN_MS     200    /* minimum easing duration (ms) */

void Servo_Init(void);
void Servo_SetAngle(uint8_t servo_id, uint16_t angle_deg10);
void Servo_Update(void);

#endif
