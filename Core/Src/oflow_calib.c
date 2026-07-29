/**
 * @file    oflow_calib.c
 * @brief   光流标定模块 — 高度标定 + 偏心偏移标定实现
 */

#include "oflow_calib.h"
#include "oflow.h"
#include "pmw3901.h"
#include "move.h"
#include "Emm_V5.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include <math.h>
#include <stdlib.h>

/* 外部: Emm_V5 电机地址 */
static const uint8_t cal_wheel_addr[4] = {MOTOR_FL, MOTOR_FR, MOTOR_RL, MOTOR_RR};

/* 外部: Move 层函数 */
extern volatile uint8_t g_move_active;

/* ================================================================
 *  高度标定
 * ================================================================ */

/**
 * @brief  发位置命令给 4 轮 (位置模式, 同步触发)
 *
 * @param dir  各轮方向数组 [4]: 0=CW, 1=CCW
 * @param clk  脉冲数
 */
static void cal_send_pos(const uint8_t dir[4], uint32_t clk)
{
    for (uint8_t i = 0; i < 4; i++) {
        Emm_V5_Pos_Control(cal_wheel_addr[i], dir[i],
                           OFLOW_CAL_VEL_RPM, OFLOW_CAL_ACC,
                           clk, false /*相对位置*/, true /*snF*/);
        osDelay(15);  /* Emm_V5 Pos_Control 13字节→2帧, 需更长间隔 */
    }
    Emm_V5_Synchronous_motion(0x00);  /* 广播同步触发 */
}

/**
 * @brief  等待运动结束 (光流连续 N 帧无位移)
 *
 * @param accum_dx  [out] 累计像素 X
 * @param accum_dy  [out] 累计像素 Y
 * @param valid     [out] 有效采样数
 * @param invalid   [out] 无效采样数
 * @return 1=正常结束, 0=超时
 */
static uint8_t cal_wait_done(int32_t *accum_dx, int32_t *accum_dy,
                              uint32_t *valid, uint32_t *invalid)
{
    *accum_dx = 0;
    *accum_dy = 0;
    *valid = 0;
    *invalid = 0;

    uint32_t t0 = xTaskGetTickCount() * portTICK_PERIOD_MS;
    uint32_t still_count = 0;
    #define STILL_THRESHOLD  5   /* 连续 5 帧位移 < 5 像素视为停止 */
    #define STILL_REQUIRED   20  /* 连续 20 次 (200ms) 确认停止 */

    for (;;) {
        /* 超时检查 */
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (now - t0 >= OFLOW_CAL_TIMEOUT_MS) return 0;

        /* 读光流 */
        int16_t dx = 0, dy = 0;
        uint8_t squal = 0, obs = 0;
        pmw3901_read_motion(&dx, &dy, &squal, &obs);

        if (obs == PMW_OBSERVATION_OK && squal >= OFLOW_SQUAL_MIN) {
            (*valid)++;
            *accum_dx += dx;
            *accum_dy += dy;

            /* 停止检测 */
            if (abs(dx) < STILL_THRESHOLD && abs(dy) < STILL_THRESHOLD) {
                still_count++;
            } else {
                still_count = 0;
            }
        } else {
            (*invalid)++;
        }

        /* 运动结束: 连续 200ms 无显著位移 */
        if (still_count >= STILL_REQUIRED) break;

        osDelay(OFLOW_CAL_SAMPLE_MS);
    }
    return 1;
}

uint8_t OFlowCalib_Height(uint8_t axis, float num_revolutions,
                           OFlowCalibResult_t *result)
{
    if (!oflow_sensor_ok || num_revolutions <= 0.0f) return 0;

    /* 锁定 CAN 总线 */
    g_move_active = 1;

    /* 清零光流累计 */
    OFlow_Reset();
    osDelay(OFLOW_CAL_SETTLE_MS);

    /* 计算脉冲数 */
    uint32_t clk = (uint32_t)(num_revolutions * OFLOW_CAL_PULSES_PER_REV);

    /* 确定各轮方向 */
    uint8_t dir[4];
    if (axis == 0) {
        /* 前进: 左CW, 右CCW(镜像) */
        dir[0] = 0;  dir[1] = 1;  dir[2] = 0;  dir[3] = 1;
    } else {
        /* 右移: FL前(CW), FR后(CW镜像反转→CCW反→CW), RL后(CW反→CCW), RR前(CCW镜像→CCW)
         * 实际: 麦轮右移 = FL前进, FR后退, RL后退, RR前进
         * 前进=左CW/右CCW, 后退=左CCW/右CW */
        dir[0] = 0;  /* FL 前进 → CW */
        dir[1] = 0;  /* FR 后退 → 正常前进是CCW, 后退=CW */
        dir[2] = 1;  /* RL 后退 → 正常前进是CW, 后退=CCW */
        dir[3] = 1;  /* RR 前进 → CCW (镜像) */
    }

    /* 发位置命令 */
    cal_send_pos(dir, clk);

    /* 等待运动完成 */
    uint8_t ok = cal_wait_done(&result->accum_dx_pixels,
                                &result->accum_dy_pixels,
                                &result->valid_samples,
                                &result->invalid_samples);

    /* 停稳后等待 */
    osDelay(OFLOW_CAL_SETTLE_MS);
    g_move_active = 0;

    if (!ok) return 0;

    /* 计算结果 */
    result->num_revolutions = num_revolutions;
    result->actual_distance_m = 3.14159265f * OFLOW_CAL_WHEEL_D_M * num_revolutions;

    /* 取运动轴的累计像素 (前进=Y轴对应 dx_raw, 侧移=X轴对应 dy_raw)
     * 传感器: dx_raw=前后方向, dy_raw=左右方向
     * 前进时主要位移在 dx_raw, 侧移时主要位移在 dy_raw */
    int32_t travel_pixels;
    if (axis == 0) {
        travel_pixels = abs(result->accum_dx_pixels);
    } else {
        travel_pixels = abs(result->accum_dy_pixels);
    }

    if (travel_pixels > 0) {
        result->pix_to_m_result = result->actual_distance_m / (float)travel_pixels;
        result->estimated_height_m = result->pix_to_m_result / PMW_RESOLUTION_M;
    } else {
        result->pix_to_m_result = 0.0f;
        result->estimated_height_m = 0.0f;
    }

    /* 更新全局比例系数 */
    if (result->pix_to_m_result > 0.0f) {
        pmw_pix_to_m = result->pix_to_m_result;
    }

    return 1;
}

/* ================================================================
 *  偏心偏移标定
 * ================================================================ */

uint8_t OFlowCalib_Offset(float *offset_x_out, float *offset_y_out)
{
    if (!oflow_sensor_ok) return 0;

    /* 清零光流坐标 */
    OFlow_Reset();
    osDelay(OFLOW_CAL_SETTLE_MS);

    /* 原地转 360° (CW 正, 即 +360°)
     * RotateTo 会设置 g_move_active=1, 阻塞等待完成 */
    uint8_t ok = RotateTo(360.0f, MOVE_YAW_TURN_LIMIT);

    osDelay(OFLOW_CAL_SETTLE_MS);

    if (!ok) return 0;

    /* 读取光流累计位移 (场坐标, 理论应为零) */
    float fx, fy;
    OFlow_GetPose(&fx, &fy);

    /* 反推偏移量:
     * 转一圈 (2π 弧度) 的光流位移:
     *   oflow_x ≈ -2π * Ly  → Ly = -oflow_x / (2π)
     *   oflow_y ≈  2π * Lx  → Lx =  oflow_y / (2π)
     *
     * 推导: 偏心补偿公式 dx_center = dx_sensor + dθ * Ly
     *   如果 Lx/Ly 设错, 转一圈后:
     *   Δoflow_x = ∫(补偿误差_x) = -2π * (Ly_true - Ly_set)
     *   Δoflow_y = ∫(补偿误差_y) =  2π * (Lx_true - Lx_set)
     *   这里 Ly_set=0, Lx_set=0 (初始), 所以直接反推 */
    const float TWO_PI = 6.28318530f;
    *offset_x_out =  fy / TWO_PI;   /* 右偏量 m */
    *offset_y_out = -fx / TWO_PI;   /* 前偏量 m */

    return 1;
}

/* ================================================================
 *  辅助函数
 * ================================================================ */

float OFlowCalib_GetPixToM(void)
{
    return pmw_pix_to_m;
}

void OFlowCalib_SetPixToM(float val)
{
    if (val > 0.0f) {
        pmw_pix_to_m = val;
    }
}
