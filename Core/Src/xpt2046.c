/**
 * @file    xpt2046.c
 * @brief   XPT2046 resistive touch screen driver
 *
 * Uses SPI3 (hspi3) for communication. The XPT2046 shares SPI3 bus
 * with no other device; CS (PC9) is manually controlled.
 *
 * Coordinate mapping (ILI9488 portrait 320x480, MADCTL=0x08):
 *   Touch X axis -> screen horizontal (0=left, 319=right)
 *   Touch Y axis -> screen vertical   (0=top, 479=bottom)
 *   These may need swapping/flipping after calibration.
 */

#include "xpt2046.h"
#include "spi.h"
#include <string.h>

/* ── Calibration state ── */
static TouchCalib_t g_cal = {
    .x_offset = 0,
    .y_offset = 0,
    .x_scale  = 1.0f,
    .y_scale  = 1.0f,
    .calibrated = 0
};

/* ── SPI helpers ── */

static uint8_t spi_tx, spi_rx;

static uint8_t XPT_TransferByte(uint8_t tx)
{
    spi_tx = tx;
    HAL_SPI_TransmitReceive(&hspi3, &spi_tx, &spi_rx, 1, 50);
    return spi_rx;
}

/* ── API ── */

uint8_t XPT2046_Init(void)
{
    T_CS_HIGH();
    HAL_Delay(10);

    /* Verify: read X with CS low, expect non-0xFF response */
    T_CS_LOW();
    HAL_Delay(1);
    XPT_TransferByte(XPT_CMD_X);     /* send command */
    uint8_t hi = XPT_TransferByte(0);/* read high byte */
    T_CS_HIGH();

    /* 0xFF means no device responding */
    if (hi == 0xFF) return 0;

    /* Set default calibration (center of raw range -> center of screen) */
    g_cal.x_offset = XPT_RAW_X_MIN;
    g_cal.y_offset = XPT_RAW_Y_MIN;
    g_cal.x_scale  = (float)TOUCH_SCREEN_W / (XPT_RAW_X_MAX - XPT_RAW_X_MIN);
    g_cal.y_scale  = (float)TOUCH_SCREEN_H / (XPT_RAW_Y_MAX - XPT_RAW_Y_MIN);
    g_cal.calibrated = 1;

    return 1;
}

uint16_t XPT2046_ReadRaw(uint8_t cmd)
{
    T_CS_LOW();
    HAL_Delay(1);

    XPT_TransferByte(cmd);              /* send command byte */
    uint8_t hi = XPT_TransferByte(0);   /* clock out 12-bit result */
    uint8_t lo = XPT_TransferByte(0);

    T_CS_HIGH();

    /* 12-bit result: hi[7:0] + lo[7:4] -> shift right 4 */
    uint16_t val = ((uint16_t)hi << 8) | lo;
    val >>= 4;  /* align to 12-bit (0~4095) */

    return val;
}

/**
 * Read X or Y with multi-sample median filter.
 * Sorts samples and takes middle value to reject outliers.
 */
static uint16_t read_filtered(uint8_t cmd, uint8_t count)
{
    uint16_t samples[XPT_SAMPLE_COUNT];
    if (count > XPT_SAMPLE_COUNT) count = XPT_SAMPLE_COUNT;

    for (uint8_t i = 0; i < count; i++) {
        samples[i] = XPT2046_ReadRaw(cmd);
    }

    /* Simple insertion sort */
    for (uint8_t i = 1; i < count; i++) {
        uint16_t key = samples[i];
        int8_t j = i - 1;
        while (j >= 0 && samples[j] > key) {
            samples[j + 1] = samples[j];
            j--;
        }
        samples[j + 1] = key;
    }

    /* Return median (middle sample) */
    return samples[count / 2];
}

uint8_t XPT2046_ReadTouch(TouchPoint_t *pt)
{
    if (!pt) return 0;

    /* Quick check: is screen touched? */
    if (!T_PRESSED()) {
        pt->pressed = 0;
        return 0;
    }

    /* Read raw coordinates with filtering */
    uint16_t rx = read_filtered(XPT_CMD_X, XPT_SAMPLE_COUNT);
    uint16_t ry = read_filtered(XPT_CMD_Y, XPT_SAMPLE_COUNT);

    /* Verify still pressed after reading */
    if (!T_PRESSED()) {
        pt->pressed = 0;
        return 0;
    }

    /* Apply calibration -> screen coords */
    XPT2046_RawToScreen(rx, ry, &pt->x, &pt->y);
    pt->pressed = 1;
    return 1;
}

void XPT2046_RawToScreen(uint16_t raw_x, uint16_t raw_y,
                          uint16_t *sx, uint16_t *sy)
{
    if (!g_cal.calibrated) {
        *sx = 0;
        *sy = 0;
        return;
    }

    int32_t x = (int32_t)((raw_x - g_cal.x_offset) * g_cal.x_scale);
    int32_t y = (int32_t)((raw_y - g_cal.y_offset) * g_cal.y_scale);

    /* Clamp to screen bounds */
    if (x < 0) x = 0;
    if (x >= TOUCH_SCREEN_W) x = TOUCH_SCREEN_W - 1;
    if (y < 0) y = 0;
    if (y >= TOUCH_SCREEN_H) y = TOUCH_SCREEN_H - 1;

    /* Invert for 180° display rotation (MADCTL MY|MX) */
    x = TOUCH_SCREEN_W - 1 - x;
    y = TOUCH_SCREEN_H - 1 - y;

    *sx = (uint16_t)x;
    *sy = (uint16_t)y;
}

void XPT2046_Calibrate(TouchCalib_t *cal)
{
    /*
     * Simple 2-point calibration:
     * 1. Show crosshair at (20, 20), wait for touch, record raw
     * 2. Show crosshair at (300, 460), wait for touch, record raw
     * 3. Compute scale and offset
     *
     * For now, just use default values. Full interactive calibration
     * requires LCD drawing which couples with DisplayTask.
     */
    if (cal) {
        *cal = g_cal;
    }
}

TouchCalib_t* XPT2046_GetCalib(void)
{
    return &g_cal;
}
