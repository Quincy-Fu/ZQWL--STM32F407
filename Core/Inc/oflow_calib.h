/**
 * @file    oflow_calib.h
 * @brief   光流标定模块 — 高度标定 + 偏心偏移标定
 *
 * 当前状态: 随光流模块停用 (oflow.h 中 OFLOW_ENABLE = 0).
 * 代码保留完整编译; 上位机标定命令 (NAV_CMD_CALIB_HEIGHT/OFFSET)
 * 仅在 OFLOW_ENABLE=1 时接入 NavTask, 停用期间也可在 Keil
 * 调试器中手动调用本模块函数. 标定不控制补光灯 (已解耦).
 *
 * 1. 高度标定 (pix_to_m):
 *    用电机位置模式驱动 4 轮走固定圈数 (实际位移已知),
 *    同时累计光流像素, 算出 pix_to_m 比例系数.
 *
 * 2. 偏心偏移标定 (offset_x/y):
 *    慢速原地转 360° (Move_SetRobotVelocity + IMU 累计角度),
 *    PMW3901 delta 寄存器自然累计整圈像素, 转完一次读出.
 *    刚体运动学: v_body = ω × r = (-ω·Ly, ω·Lx) 常量,
 *    360° 积分系数 = 2π (不是 π), 实际用 actual_rad (含过冲):
 *      offset_x =  accum_dx × pix_to_m / actual_rad
 *      offset_y = -accum_dy × pix_to_m / actual_rad
 *
 * 所有标定函数阻塞执行, 在 NavTask 或 Keil 调试器中调用.
 */

#ifndef __OFLOW_CALIB_H
#define __OFLOW_CALIB_H

#include <stdint.h>

/* ================================================================
 *  Emm_V5 位置模式参数 (底盘电机标定用)
 * ================================================================ */
#define OFLOW_CAL_PULSES_PER_REV  3200u    /* 16细分: 3200 脉冲 = 1 圈 */
#define OFLOW_CAL_VEL_RPM         200      /* 标定运动速度 RPM (慢速, 减小打滑) */
#define OFLOW_CAL_ACC             5        /* 加速度 (0=直接启动, 偏慢) */
#define OFLOW_CAL_WHEEL_D_M      0.065f   /* 轮径 m */
#define OFLOW_CAL_SAMPLE_MS      10       /* 标定期间光流采样周期 ms */
#define OFLOW_CAL_SETTLE_MS      500      /* 启动/停止后等待稳定 ms */
#define OFLOW_CAL_TIMEOUT_MS     40000    /* 偏心标定超时: 360°@17°/s+减速+settle≈28s */

/* ================================================================
 *  标定结果结构体
 * ================================================================ */
typedef struct {
    /* 输入参数 */
    float    num_revolutions;      /* 转数 (输入) */
    /* 计算值 */
    float    actual_distance_m;    /* 理论位移 = π × D × N */
    /* 光流累计 */
    int32_t  accum_dx_pixels;      /* X方向累计像素 (传感器原始) */
    int32_t  accum_dy_pixels;      /* Y方向累计像素 (传感器原始) */
    /* 采样统计 */
    uint32_t valid_samples;        /* 有效采样次数 */
    uint32_t invalid_samples;      /* 无效采样次数 */
    /* 结果 */
    float    pix_to_m_result;      /* 标定得到的比例系数 */
    float    estimated_height_m;   /* 估计安装高度 m */
} OFlowCalibResult_t;

/* ================================================================
 *  API
 * ================================================================ */

/**
 * @brief  高度标定: 4 轮位置模式同步行走 + 光流累计 + 比例系数计算
 *
 * 步骤:
 *   1. 清零光流累计, 等待稳定
 *   2. 4 轮同时发 Pos_Control (snF=true) + Synchronous_motion 同步触发
 *   3. 轮询读光流, 累计像素, 直到运动结束或超时
 *   4. 计算 pix_to_m = actual_distance / 运动轴累计像素
 *   5. 更新全局 pmw_pix_to_m
 *
 * @param  axis            0=前进(Y)标定, 1=侧移(X)标定
 * @param  num_revolutions 转数 (如 5.0 = 5 圈)
 * @param  result          [out] 标定结果
 * @return 1=成功, 0=超时或失败
 */
uint8_t OFlowCalib_Height(uint8_t axis, float num_revolutions,
                           OFlowCalibResult_t *result);

/**
 * @brief  偏心偏移标定: 慢速原地转 360°, 反推 offset_x/y
 *
 * 步骤:
 *   1. 开补光灯 + 挂起 OptFlowTask + PMW_CS_HIGH 防 SPI 竞争
 *   2. OFlow_Reset 清软件累计 + dummy read 清 PMW3901 delta 寄存器
 *   3. Move_SetRobotVelocity 慢速 (0.05m/s) 持续旋转,
 *      循环内每 10ms 读 PMW3901 + 软件累加 int32_t (不过滤!)
 *   4. IMU dyaw 逐样本归一化累计, |total|>=360° 停
 *   5. 最后 60° 线性减速 (防惯性过冲)
 *
 * 刚体运动学: 传感器在车体系位置 (Lx,Ly) 固定, 角速度 ω 时
 *   v_body = ω × r = (-ω·Ly, ω·Lx) — 常量! 不随 θ 变化
 *   360° 积分 = v_body × 2π/|ω| → 系数是 2π 不是 π
 *   accum_dx =  2π·Lx / pix_to_m  → Lx = accum_dx·pix_to_m / (2π)
 *   accum_dy = -2π·Ly / pix_to_m  → Ly = -accum_dy·pix_to_m / (2π)
 *
 * 超时容错: 即使超时, 若已转 >10° 且有数据, 按比例折算后返回 1.
 * 诊断数据在成功/失败两条路径都存入全局变量.
 *
 * @param  offset_x_out  [out] X 偏移量 m (右正, 传感器相对旋转中心)
 * @param  offset_y_out  [out] Y 偏移量 m (前正)
 * @return 1=成功(含超时但有数据), 0=完全失败
 */
uint8_t OFlowCalib_Offset(float *offset_x_out, float *offset_y_out);

/**
 * @brief  获取当前生效的 pix_to_m 值
 */
float OFlowCalib_GetPixToM(void);

/**
 * @brief  手动设置 pix_to_m (覆盖标定值)
 */
void OFlowCalib_SetPixToM(float val);

#endif /* __OFLOW_CALIB_H */
