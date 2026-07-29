/**
 * @file    oflow.c
 * @brief   光流处理模块 — OptFlowTask 主循环实现
 *
 * 10ms 周期读 PMW3901 Motion Burst，做偏心补偿后积分到光流坐标。
 * 独立于编码器里程计运行，输出 oflow_x/y 供融合或监测。
 *
 * 坐标映射 (PMW3901 安装: X朝车前, Y朝车左, 输出取反):
 *   dx_body (右) = -dy_raw * pix_to_m
 *   dy_body (前) = -dx_raw * pix_to_m
 *
 * 偏心补偿 (传感器偏移 Lx=右正, Ly=前正):
 *   dx_center = dx_sensor + d_theta * Ly
 *   dy_center = dy_sensor - d_theta * Lx
 *   d_theta = IMU 角度变化量 (弧度, CCW正)
 */

#include "oflow.h"
#include "pmw3901.h"
#include "move.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include <math.h>

/* 外部变量 */
extern volatile float g_imu_yaw;   /* IMU 航向 ° (CCW正), freertos.c */

/* ================================================================
 *  全局变量定义
 * ================================================================ */
volatile float    oflow_x             = 0.0f;
volatile float    oflow_y             = 0.0f;
volatile float    oflow_vx            = 0.0f;
volatile float    oflow_vy            = 0.0f;
volatile float    oflow_squal_avg     = 0.0f;
volatile uint32_t oflow_valid_count   = 0;
volatile uint32_t oflow_invalid_count = 0;
volatile int32_t  oflow_accum_dx_raw  = 0;
volatile int32_t  oflow_accum_dy_raw  = 0;
volatile uint8_t  oflow_sensor_ok     = 0;

/* ================================================================
 *  内部辅助
 * ================================================================ */

#define DEG_TO_RAD  0.01745329f

/**
 * @brief  清零光流坐标和累计值
 */
void OFlow_Reset(void)
{
    __disable_irq();
    oflow_x = 0.0f;
    oflow_y = 0.0f;
    oflow_vx = 0.0f;
    oflow_vy = 0.0f;
    oflow_accum_dx_raw = 0;
    oflow_accum_dy_raw = 0;
    __enable_irq();
}

/**
 * @brief  获取光流坐标快照
 */
void OFlow_GetPose(float *x, float *y)
{
    __disable_irq();
    *x = oflow_x;
    *y = oflow_y;
    __enable_irq();
}

/* ================================================================
 *  OptFlowTask 主循环
 * ================================================================ */

void OFlow_TaskLoop(void)
{
    /* ---- 初始化 ---- */
    osDelay(OFLOW_INIT_DELAY_MS);

    if (!pmw3901_init()) {
        /* 传感器初始化失败, 标记并挂起 */
        oflow_sensor_ok = 0;
        vTaskSuspend(NULL);
        return;  /* 不会执行到这里 */
    }
    oflow_sensor_ok = 1;

    /* 清零累计 */
    OFlow_Reset();

    /* 上一帧 IMU yaw (用于差分角速度) */
    float prev_yaw_deg = g_imu_yaw;
    float omega_filtered = 0.0f;   /* 滤波后角速度 rad/s */
    float squal_sum = 0.0f;
    uint32_t squal_cnt = 0;

    const float dt = OFLOW_SAMPLE_MS * 0.001f;  /* 采样间隔 s */

    /* ---- 主循环 ---- */
    TickType_t last_tick = xTaskGetTickCount();

    for (;;) {
        /* 1. 读 Motion Burst */
        int16_t dx_raw = 0, dy_raw = 0;
        uint8_t squal = 0, obs = 0;
        pmw3901_read_motion(&dx_raw, &dy_raw, &squal, &obs);

        /* 2. 有效性检查 */
        bool valid = (obs == PMW_OBSERVATION_OK) && (squal >= OFLOW_SQUAL_MIN);

        if (valid) {
            oflow_valid_count++;

            /* 累计原始像素 (调试) */
            oflow_accum_dx_raw += dx_raw;
            oflow_accum_dy_raw += dy_raw;

            /* squal 滑动平均 */
            squal_sum += squal;
            squal_cnt++;
            oflow_squal_avg = squal_sum / (float)squal_cnt;

            /* 3. 像素→体坐标位移 (m)
             * PMW3901: X朝前, Y朝左, 输出取反
             * dx_body(右) = -dy_raw * pix_to_m
             * dy_body(前) = -dx_raw * pix_to_m */
            float dx_sensor = -(float)dy_raw * pmw_pix_to_m;  /* 右移 */
            float dy_sensor = -(float)dx_raw * pmw_pix_to_m;  /* 前进 */

            /* 4. IMU 角速度 (用于偏心补偿) */
            float cur_yaw_deg = g_imu_yaw;  /* CCW正 */
            float d_yaw_deg = cur_yaw_deg - prev_yaw_deg;
            prev_yaw_deg = cur_yaw_deg;

            /* 处理 ±360 跳变 (不太可能出现, 但安全) */
            if (d_yaw_deg > 180.0f)  d_yaw_deg -= 360.0f;
            if (d_yaw_deg < -180.0f) d_yaw_deg += 360.0f;

            float d_theta = d_yaw_deg * DEG_TO_RAD;  /* 弧度, CCW正 */

            /* 角速度 rad/s + 低通滤波 */
            float omega_raw = d_theta / dt;
            omega_filtered = OFLOW_OMEGA_LPFA * omega_raw
                           + (1.0f - OFLOW_OMEGA_LPFA) * omega_filtered;

            /* 5. 偏心补偿
             * dx_center = dx_sensor + d_theta * offset_y
             * dy_center = dy_sensor - d_theta * offset_x */
            float dx_center = dx_sensor + d_theta * OFLOW_OFFSET_Y_M;
            float dy_center = dy_sensor - d_theta * OFLOW_OFFSET_X_M;

            /* 6. 体坐标→场坐标旋转
             * field_x(右) =  dx*cos(imu) + dy*sin(imu)
             * field_y(前) = -dx*sin(imu) + dy*cos(imu)
             * imu_yaw 是 CCW正 */
            float imu_rad = cur_yaw_deg * DEG_TO_RAD;
            float cy = cosf(imu_rad);
            float sy = sinf(imu_rad);

            float dX_field =  dx_center * cy + dy_center * sy;
            float dY_field = -dx_center * sy + dy_center * cy;

            /* 7. 累加坐标 */
            __disable_irq();
            oflow_x += dX_field;
            oflow_y += dY_field;
            oflow_vx = dX_field / dt;
            oflow_vy = dY_field / dt;
            __enable_irq();

        } else {
            /* 无效帧 */
            oflow_invalid_count++;

            /* 仍更新 IMU yaw 基准, 避免恢复时跳变 */
            prev_yaw_deg = g_imu_yaw;
        }

        /* 8. 精确周期延时 */
        vTaskDelayUntil(&last_tick, pdMS_TO_TICKS(OFLOW_SAMPLE_MS));
    }
}
