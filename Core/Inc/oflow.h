/**
 * @file    oflow.h
 * @brief   光流处理模块 — PMW3901 采样 + 偏心补偿 + 独立里程计
 *
 * 10ms 周期读 PMW3901 Motion Burst，做偏心补偿后积分到光流坐标。
 * 输出 oflow_x/y (场坐标, 米) 供融合或监测使用。
 *
 * 坐标系: +X=右, +Y=前, +θ=CW (与 move.h 一致)
 * 光流传感器安装: X朝车前, Y朝车左 (pmw3901.h), 输出取反
 *
 * 偏心补偿: 传感器不在底盘中心时，旋转会产生额外位移。
 *   dx_center = dx_sensor + d_theta * offset_y
 *   dy_center = dy_sensor - d_theta * offset_x
 *   offset_x/y 为用户配置 (体坐标: x=右正, y=前正)
 */

#ifndef __OFLOW_H
#define __OFLOW_H

#include <stdint.h>
#include <stdbool.h>

/* ================================================================
 *  配置参数 (用户填入实测值)
 * ================================================================ */
#define OFLOW_OFFSET_X_M     0.000f   /* 传感器中心相对底盘中心的右偏量 m (待填入) */
#define OFLOW_OFFSET_Y_M     0.000f   /* 传感器中心相对底盘中心的前偏量 m (待填入) */
#define OFLOW_SAMPLE_MS      10       /* 采样周期 ms (100Hz) */
#define OFLOW_SQUAL_MIN      0x19     /* 最低可信 squal (同 pmw3901.h) */
#define OFLOW_OMEGA_LPFA     0.3f     /* 角速度低通滤波系数 (0~1, 小=更平滑) */
#define OFLOW_INIT_DELAY_MS  2500     /* 启动等待 ms (等系统就绪) */

/* ================================================================
 *  全局状态 (extern, 定义在 oflow.c)
 * ================================================================ */
extern volatile float oflow_x;         /* 光流场坐标 X m (右正) */
extern volatile float oflow_y;         /* 光流场坐标 Y m (前进) */
extern volatile float oflow_vx;        /* 光流速度估计 m/s (右) */
extern volatile float oflow_vy;        /* 光流速度估计 m/s (前) */
extern volatile float oflow_squal_avg; /* 平均表面质量 */
extern volatile uint32_t oflow_valid_count;    /* 有效采样计数 */
extern volatile uint32_t oflow_invalid_count;  /* 无效采样计数 */
extern volatile int32_t oflow_accum_dx_raw;    /* 调试: 累计原始像素 X */
extern volatile int32_t oflow_accum_dy_raw;    /* 调试: 累计原始像素 Y */
extern volatile uint8_t oflow_sensor_ok;       /* 传感器状态: 1=正常 */

/* ================================================================
 *  API
 * ================================================================ */

/**
 * @brief  光流任务主循环 (由 OptFlowTask 调用, 不返回)
 *         初始化传感器 → 10ms 循环采样 → 偏心补偿 → 积分
 */
void OFlow_TaskLoop(void);

/**
 * @brief  清零光流坐标 (归零 oflow_x/y 和累计像素)
 */
void OFlow_Reset(void);

/**
 * @brief  获取光流坐标快照 (关中断保护)
 * @param  x  [out] 场坐标 X m
 * @param  y  [out] 场坐标 Y m
 */
void OFlow_GetPose(float *x, float *y);

#endif /* __OFLOW_H */
