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

/* ---- Angle range (0.1 deg units): 0-2700 = 0.0-270.0 deg ---- */
/* Change to 1800 if your servos are 180-degree type */
#define SERVO_ANGLE_MAX   2700

/* ---- Quintic easing parameters ---- */
#define SERVO_EASE_MS_PER_US  0.3f   /* ms of easing per us pulse diff */
#define SERVO_EASE_MIN_MS     100    /* minimum easing duration (ms)   */

void Servo_Init(void);
void Servo_SetAngle(uint8_t servo_id, uint16_t angle_deg10);
void Servo_Update(void);

#endif
