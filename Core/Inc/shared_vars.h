/**
 * @file    shared_vars.h
 * @brief   跨模块共享变量的 extern 声明 (集中管理)
 *
 * 所有在 freertos.c / main.c / oflow.c / move.c 等定义的全局变量,
 * 只要被 2 个以上 .c 文件引用, 都必须在此声明.
 * 任何 .c 文件需要访问这些变量时, #include "shared_vars.h" 即可.
 */

#ifndef __SHARED_VARS_H
#define __SHARED_VARS_H

#include "cmsis_os.h"
#include <stdint.h>

/* ================================================================
 *  freertos.c 定义
 * ================================================================ */

/* IMU 航向 (度, 上电=0°; 原始正方向由安装决定, move.c统一映射到CW正) */
extern volatile float g_imu_yaw;       /* LPF滤波后, 诊断/校准用; 普通航向不依赖它 */
extern volatile float g_imu_yaw_raw;  /* 无LPF, 诊断/校准用; 运动角度闭环不依赖它 */
extern volatile uint8_t g_imu_verified;

/* 里程计 (场坐标: x=右m, y=前m, theta=CW+ rad, =move_yaw×π/180) */
extern volatile float g_odom_x;
extern volatile float g_odom_y;
extern volatile float g_odom_theta;

/* 速度指令 (m/s, rad/s) */
extern volatile float g_tgt_vx;
extern volatile float g_tgt_vy;
extern volatile float g_tgt_omega;

/* 转盘 */
extern volatile uint8_t g_target_gear;
extern volatile uint32_t g_pos_cmd_count;
extern volatile uint8_t g_pos_homed;

/* 灯光/舵机 ISR 派发 */
extern volatile uint8_t g_light_pending_id;
extern volatile uint8_t g_light_pending_on;
extern volatile uint8_t g_light_pending;
extern volatile uint8_t g_arm_state;

/* FreeRTOS 句柄 (跨任务引用时需要) */
extern osThreadId OptFlowTaskHandle;
extern osMessageQId DataQueueHandle;
extern osMessageQId NavQueueHandle;
extern osMutexId    Uart6MutexHandle;
extern osMutexId    CanTxMutexHandle;
extern osThreadId   CommTaskHandle;

/* ================================================================
 *  move.c 定义
 * ================================================================ */
extern volatile uint8_t g_move_active;

#endif /* __SHARED_VARS_H */
