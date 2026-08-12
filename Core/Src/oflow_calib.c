/**
 * @file    oflow_calib.c
 * @brief   光流标定模块 — 高度标定 + 偏心偏移标定实现
 *
 * 当前状态: 已停用 (oflow.h 中 OFLOW_ENABLE = 0).
 * 代码保留完整可编译; 启用光流后, 可经上位机标定命令或
 * Keil 调试器直接调用本模块函数. 不控制补光灯 (与 light 模块解耦).
 */

#include "oflow_calib.h"
#include "oflow.h"
#include "pmw3901.h"
#include "move.h"
#include "Emm_V5.h"
#include "shared_vars.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

/* 偏心标定诊断 (Keil Watch 窗口查看) */
volatile int32_t  calib_dbg_dx;
volatile int32_t  calib_dbg_dy;
volatile uint8_t  calib_dbg_squal;
volatile uint8_t  calib_dbg_obs;
volatile float    calib_dbg_total_deg;

/* 外部: Emm_V5 电机地址 */
static const uint8_t cal_wheel_addr[4] = {MOTOR_FL, MOTOR_FR, MOTOR_RL, MOTOR_RR};

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
 * @brief  等待运动结束 (时间法: 电机参数已知, 精确计算运动时长)
 *
 * 不依赖PMW3901噪声检测: 8cm高度下PMW3901噪声不可预测,
 * 固定阈值和EMA都无法可靠区分运动/停止.
 * 电机位置模式: 200RPM, 3200脉冲/圈, 运动时间可精确算出.
 *
 * @param num_revolutions  电机转动圈数
 * @param accum_dx  [out] 累计像素 X
 * @param accum_dy  [out] 累计像素 Y
 * @param valid     [out] 有效采样数
 * @param invalid   [out] 无效采样数
 * @return 1=正常结束
 */
static uint8_t cal_wait_done(int32_t *accum_dx, int32_t *accum_dy,
                              uint32_t *valid, uint32_t *invalid,
                              float num_revolutions)
{
    *accum_dx = 0;
    *accum_dy = 0;
    *valid = 0;
    *invalid = 0;

    /* 时间法: 200RPM=3.33rev/s, 每圈0.3s
     * 总等待 = 运动时间 + 加速余量(1s) + 停止settle(0.5s) */
    uint32_t motion_ms = (uint32_t)(num_revolutions * 60.0f
                                   / (float)OFLOW_CAL_VEL_RPM * 1000.0f);
    uint32_t wait_ms = motion_ms + 1000 + OFLOW_CAL_SETTLE_MS;

    uint32_t t0 = xTaskGetTickCount() * portTICK_PERIOD_MS;

    for (;;) {
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (now - t0 >= wait_ms) break;

        int16_t dx = 0, dy = 0;
        uint8_t squal = 0, obs = 0;
        pmw3901_read_motion(&dx, &dy, &squal, &obs);

        if (obs == PMW_OBSERVATION_OK && squal >= OFLOW_SQUAL_MIN) {
            (*valid)++;
            *accum_dx += dx;
            *accum_dy += dy;
        } else {
            (*invalid)++;
        }

        osDelay(OFLOW_CAL_SAMPLE_MS);
    }
    return 1;
}

uint8_t OFlowCalib_Height(uint8_t axis, float num_revolutions,
                           OFlowCalibResult_t *result)
{
    if (!oflow_sensor_ok || num_revolutions <= 0.0f) return 0;

    /* 挂起 OptFlowTask: 它会读 PMW3901 并清零 delta 寄存器,
     * 与 cal_wait_done 竞争, 导致像素被偷 + 提前判定停止 */
    if (OptFlowTaskHandle) vTaskSuspend(OptFlowTaskHandle);
    PMW_CS_HIGH();
    HAL_Delay(1);

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
                                &result->invalid_samples,
                                num_revolutions);

    /* 停稳后等待 */
    osDelay(OFLOW_CAL_SETTLE_MS);
    g_move_active = 0;

    if (!ok) {
        if (OptFlowTaskHandle) vTaskResume(OptFlowTaskHandle);
        return 0;
    }

    /* 计算结果 */
    result->num_revolutions = num_revolutions;
    /* 麦轮 45° 滚柱: 轮子转一圈, 机器人前进 π×D×cos45°, 不是 π×D */
    result->actual_distance_m = 3.14159265f * OFLOW_CAL_WHEEL_D_M
                                * 0.70710678f * num_revolutions;

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

    if (OptFlowTaskHandle) vTaskResume(OptFlowTaskHandle);
    return 1;
}

/* ================================================================
 *  偏心偏移标定
 * ================================================================ */

uint8_t OFlowCalib_Offset(float *offset_x_out, float *offset_y_out)
{
    if (!oflow_sensor_ok) return 0;

    if (OptFlowTaskHandle) vTaskSuspend(OptFlowTaskHandle);
    PMW_CS_HIGH();
    HAL_Delay(1);

    OFlow_Reset();

    /* dummy read: 清 PMW3901 delta 寄存器 (OFlow_Reset 只清软件) */
    {
        int16_t ddx, ddy; uint8_t dsq, dob;
        pmw3901_read_motion(&ddx, &ddy, &dsq, &dob);
    }

    osDelay(OFLOW_CAL_SETTLE_MS);
    g_move_active = 1;

    /* ── 慢速转 360° + 循环读 PMW3901 + 软件累加 ──
     *
     * PMW3901 是鼠标传感器, delta 寄存器必须频繁读取.
     *
     * 刚体运动学推导:
     *   传感器在车体系位置 (Lx, Ly) 固定, 角速度 ω 时
     *   v_body = ω × r = (-ω·Ly, ω·Lx)  — 常量! 不随 θ 变化
     *   360° 积分 = v_body × T = v_body × 2π/|ω| → 积分结果是 2π, 不是 π
     *
     *   accum_dx =  2π·Lx / pix_to_m  (CW)
     *   accum_dy = -2π·Ly / pix_to_m  (CW)
     *
     *   → Lx = accum_dx × pix_to_m / (2π)
     *   → Ly = -accum_dy × pix_to_m / (2π)
     */
    const float TARGET_DEG  = 360.0f;
    const float DECEL_DEG   = 60.0f;
    const float CAL_SPEED   = 0.05f;
    const float TWO_PI      = 6.28318531f;

    uint32_t t0 = xTaskGetTickCount() * portTICK_PERIOD_MS;

    Move_SetRobotVelocity(0.0f, 0.0f, CAL_SPEED);
    osDelay(50);

    float prev_yaw = g_imu_yaw;
    float total_rotation = 0.0f;
    /* 双轨累加: filtered (和高度标定一致) + unfiltered (fallback) */
    int32_t accum_dx_filt = 0, accum_dy_filt = 0;  /* 过滤后 (仅 valid) */
    int32_t accum_dx_raw  = 0, accum_dy_raw  = 0;  /* 不过滤 (全部) */
    uint32_t valid_cnt = 0, invalid_cnt = 0;
    uint8_t last_squal = 0, last_obs = 0;

    for (;;) {
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (now - t0 >= OFLOW_CAL_TIMEOUT_MS) goto fail;

        /* 减速区: 只在剩余 < 60° 时更新速度 */
        float remaining = TARGET_DEG - fabsf(total_rotation);
        if (remaining < DECEL_DEG) {
            float spd;
            if (remaining <= 0.0f) {
                spd = MOVE_YAW_FINE_SPEED;
            } else {
                spd = MOVE_YAW_FINE_SPEED
                      + (CAL_SPEED - MOVE_YAW_FINE_SPEED)
                        * (remaining / DECEL_DEG);
            }
            Move_SetRobotVelocity(0.0f, 0.0f, spd);
        }

        /* 读 PMW3901 */
        int16_t dx = 0, dy = 0;
        uint8_t squal = 0, obs = 0;
        pmw3901_read_motion(&dx, &dy, &squal, &obs);
        last_squal = squal;
        last_obs = obs;

        /* 双轨累加 */
        accum_dx_raw += dx;
        accum_dy_raw += dy;

        /* 过滤累加: 和高度标定 cal_wait_done 用同一条件
         * pix_to_m 是用过滤后的像素算的, 偏心标定也必须用过滤后的,
         * 否则噪声样本膨胀累计, 偏移结果放大数十倍 */
        if (obs == PMW_OBSERVATION_OK && squal >= OFLOW_SQUAL_MIN) {
            accum_dx_filt += dx;
            accum_dy_filt += dy;
            valid_cnt++;
        } else {
            invalid_cnt++;
        }

        /* IMU 角度累计 */
        float cur_yaw = g_imu_yaw;
        float dyaw = cur_yaw - prev_yaw;
        while (dyaw > 180.0f)  dyaw -= 360.0f;
        while (dyaw < -180.0f) dyaw += 360.0f;
        total_rotation += dyaw;
        prev_yaw = cur_yaw;

        if (fabsf(total_rotation) >= TARGET_DEG) break;

        osDelay(OFLOW_CAL_SAMPLE_MS);
    }

    Move_Stop();

    /* ── 等待惯性停止 ──
     * 电机已急停锁死, 但IMU有LPF滞后, 继续读PMW3901+IMU直到稳定,
     * 捕获真实总转角(含残余转动/IMU追赶), 用实际角度算偏移公式 */
    {
        uint32_t stop_t0 = xTaskGetTickCount() * portTICK_PERIOD_MS;
        uint32_t still_cnt = 0;

        for (;;) {
            uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
            if (now - stop_t0 >= 1000) break;  /* 最多等 1 秒 */

            int16_t dx2 = 0, dy2 = 0;
            uint8_t sq2 = 0, ob2 = 0;
            pmw3901_read_motion(&dx2, &dy2, &sq2, &ob2);
            accum_dx_raw += dx2;
            accum_dy_raw += dy2;
            if (ob2 == PMW_OBSERVATION_OK && sq2 >= OFLOW_SQUAL_MIN) {
                accum_dx_filt += dx2;
                accum_dy_filt += dy2;
                valid_cnt++;
            } else {
                invalid_cnt++;
            }
            last_squal = sq2;
            last_obs = ob2;

            float cur_yaw = g_imu_yaw;
            float dyaw = cur_yaw - prev_yaw;
            while (dyaw > 180.0f)  dyaw -= 360.0f;
            while (dyaw < -180.0f) dyaw += 360.0f;
            total_rotation += dyaw;
            prev_yaw = cur_yaw;

            if (fabsf(dyaw) < 0.1f) {
                still_cnt++;
                if (still_cnt >= 20) break;  /* 200ms 稳定 */
            } else {
                still_cnt = 0;
            }

            osDelay(OFLOW_CAL_SAMPLE_MS);
        }
    }

    g_move_active = 0;
    if (OptFlowTaskHandle) vTaskResume(OptFlowTaskHandle);

    /* 选择累加源: 优先用过滤后的 (和高度标定一致), fallback 到不过滤 */
    int32_t use_dx, use_dy;
    if (valid_cnt > 0) {
        use_dx = accum_dx_filt;
        use_dy = accum_dy_filt;
    } else {
        use_dx = accum_dx_raw;
        use_dy = accum_dy_raw;
    }

    /* 诊断存入全局变量 */
    calib_dbg_dx        = use_dx;
    calib_dbg_dy        = use_dy;
    calib_dbg_squal     = last_squal;
    calib_dbg_obs       = last_obs;
    calib_dbg_total_deg = total_rotation;

    /* 公式: v_body=ω×r 常量, 积分 = r × actual_angle
     * actual_angle 含惯性过冲 (如 375° 而非 360°), 不是假设值
     * offset_x = accum_dx × pix_to_m / actual_rad
     * offset_y = -accum_dy × pix_to_m / actual_rad */
    float actual_rad = fabsf(total_rotation) * 0.01745329f;
    if (actual_rad < 0.1f) actual_rad = TWO_PI;  /* fallback */
    *offset_x_out =  (float)use_dx * pmw_pix_to_m / actual_rad;
    *offset_y_out = -(float)use_dy * pmw_pix_to_m / actual_rad;

    return 1;

fail:
    Move_Stop();

    /* 超时: 同样等待惯性停止 */
    {
        uint32_t stop_t0 = xTaskGetTickCount() * portTICK_PERIOD_MS;
        uint32_t still_cnt = 0;
        for (;;) {
            uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
            if (now - stop_t0 >= 1000) break;

            int16_t dx2 = 0, dy2 = 0;
            uint8_t sq2 = 0, ob2 = 0;
            pmw3901_read_motion(&dx2, &dy2, &sq2, &ob2);
            accum_dx_raw += dx2; accum_dy_raw += dy2;
            if (ob2 == PMW_OBSERVATION_OK && sq2 >= OFLOW_SQUAL_MIN) {
                accum_dx_filt += dx2; accum_dy_filt += dy2; valid_cnt++;
            } else { invalid_cnt++; }
            last_squal = sq2; last_obs = ob2;

            float cur_yaw = g_imu_yaw;
            float dyaw = cur_yaw - prev_yaw;
            while (dyaw > 180.0f)  dyaw -= 360.0f;
            while (dyaw < -180.0f) dyaw += 360.0f;
            total_rotation += dyaw;
            prev_yaw = cur_yaw;

            if (fabsf(dyaw) < 0.1f) {
                still_cnt++;
                if (still_cnt >= 20) break;
            } else {
                still_cnt = 0;
            }
            osDelay(OFLOW_CAL_SAMPLE_MS);
        }
    }

    g_move_active = 0;
    if (OptFlowTaskHandle) vTaskResume(OptFlowTaskHandle);

    /* 诊断也要存 (超时也能看到转了多少、squal 多少) */
    int32_t fail_dx = (valid_cnt > 0) ? accum_dx_filt : accum_dx_raw;
    int32_t fail_dy = (valid_cnt > 0) ? accum_dy_filt : accum_dy_raw;
    calib_dbg_dx        = fail_dx;
    calib_dbg_dy        = fail_dy;
    calib_dbg_squal     = last_squal;
    calib_dbg_obs       = last_obs;
    calib_dbg_total_deg = total_rotation;

    /* 超时但有数据: 按实际转角计算 (含过冲) */
    if (fabsf(total_rotation) > 10.0f) {
        float actual_rad = fabsf(total_rotation) * 0.01745329f;
        *offset_x_out =  (float)fail_dx * pmw_pix_to_m / actual_rad;
        *offset_y_out = -(float)fail_dy * pmw_pix_to_m / actual_rad;
        return 1;
    }

    *offset_x_out = 0.0f;
    *offset_y_out = 0.0f;
    return 0;
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
