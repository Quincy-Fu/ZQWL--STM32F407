/**
 * @file    xpt2046.h
 * @brief   XPT2046 resistive touch screen driver (SPI3, 2.625MHz)
 *
 * Hardware:
 *   SPI3: PC10=SCK, PC11=MISO, PC12=MOSI (AF6)
 *   CS:   PC9  (active low)
 *   IRQ:  PG8  (active low, pull-up)
 *
 * XPT2046 protocol:
 *   Command byte: [S A2 A1 A0 MOD SER1 SER0]
 *   S=1 (start), A2-0=channel, MOD=0(12-bit), SER=1(single-ended)
 *   Response: 12-bit ADC value, MSB first, clocked on falling edge
 */

#ifndef __XPT2046_H
#define __XPT2046_H

#include "main.h"
#include <stdint.h>

/* ── Pin macros ── */
#define T_CS_PORT    GPIOC
#define T_CS_PIN     GPIO_PIN_9
#define T_IRQ_PORT   GPIOG
#define T_IRQ_PIN    GPIO_PIN_8

#define T_CS_LOW()   HAL_GPIO_WritePin(T_CS_PORT, T_CS_PIN, GPIO_PIN_RESET)
#define T_CS_HIGH()  HAL_GPIO_WritePin(T_CS_PORT, T_CS_PIN, GPIO_PIN_SET)
#define T_PRESSED()  (HAL_GPIO_ReadPin(T_IRQ_PORT, T_IRQ_PIN) == GPIO_PIN_RESET)

/* ── XPT2046 command bytes (S=1, MOD=0/12bit, SER=1/single-ended) ── */
#define XPT_CMD_X    0xD0   /* 1 101 0 000 = read X position */
#define XPT_CMD_Y    0x90   /* 1 001 0 000 = read Y position */
#define XPT_CMD_Z1   0xB0   /* 1 011 0 000 = pressure Z1 */
#define XPT_CMD_Z2   0xC0   /* 1 100 0 000 = pressure Z2 */

/* ── Screen dimensions (ILI9488 portrait) ── */
#define TOUCH_SCREEN_W   320
#define TOUCH_SCREEN_H   480

/* ── Calibration defaults (replace with actual calibration values) ── */
/* These are approximate; run XPT2046_Calibrate() to get real values */
#define XPT_RAW_X_MIN    200
#define XPT_RAW_X_MAX    3900
#define XPT_RAW_Y_MIN    200
#define XPT_RAW_Y_MAX    3900

/* ── Sample count for noise reduction ── */
#define XPT_SAMPLE_COUNT   8

/* ── Pressure threshold (Z1 value below which = touched) ── */
#define XPT_PRESS_THRESHOLD  50

/* ── Data types ── */
typedef struct {
    uint16_t x;          /* screen X (0 ~ TOUCH_SCREEN_W-1) */
    uint16_t y;          /* screen Y (0 ~ TOUCH_SCREEN_H-1) */
    uint8_t  pressed;    /* 1 = touching, 0 = not */
} TouchPoint_t;

typedef struct {
    int32_t  x_offset;
    int32_t  y_offset;
    float    x_scale;
    float    y_scale;
    uint8_t  calibrated;
} TouchCalib_t;

/* ── API ── */

/**
 * @brief  Initialize XPT2046 (CS high, verify communication)
 * @return 1 = OK, 0 = no response
 */
uint8_t XPT2046_Init(void);

/**
 * @brief  Read raw 12-bit ADC value for given command
 * @param  cmd  XPT_CMD_X / XPT_CMD_Y / XPT_CMD_Z1 / XPT_CMD_Z2
 * @return 12-bit value (0~4095)
 */
uint16_t XPT2046_ReadRaw(uint8_t cmd);

/**
 * @brief  Read touch state and coordinates (with noise filtering)
 * @param  pt  [out] touch point
 * @return 1 = pressed, 0 = not pressed
 */
uint8_t XPT2046_ReadTouch(TouchPoint_t *pt);

/**
 * @brief  Simple calibration: touch 2 corners, compute mapping
 *         Call this from a task, it blocks waiting for touches.
 * @param  cal  [out] calibration data
 */
void XPT2046_Calibrate(TouchCalib_t *cal);

/**
 * @brief  Get current calibration data
 */
TouchCalib_t* XPT2046_GetCalib(void);

/**
 * @brief  Apply calibration to raw values -> screen coords
 * @param  raw_x  raw ADC X
 * @param  raw_y  raw ADC Y
 * @param  sx     [out] screen X
 * @param  sy     [out] screen Y
 */
void XPT2046_RawToScreen(uint16_t raw_x, uint16_t raw_y,
                          uint16_t *sx, uint16_t *sy);

#endif /* __XPT2046_H */
