/**
 * @file    move.h
 * @brief   底层运动控制模块 — 移植自Blu3 Move层
 *
 * 提供：位置控制(MoveTo)、轴锁定(MoveToAxisLock)、原地旋转(RotateTo)、
 *       圆弧跟踪(MoveArc)、里程计积分、急停。
 *
 * 控制环30ms周期：P环位置控制 + 编码器回读里程计 + IMU航向保持。
 * 单位：米(m)、米/秒(m/s)、度(deg)。
 *
 * 坐标系 (与Blu3场坐标一致):
 *   +X = 右   (right)
 *   +Y = 前进 (forward)
 *   +θ = 顺时针 (CW)
 *   move_x/y = 场坐标 m, move_yaw = 偏航角(度, CW正, 上电=0°)
 *
 * 内部实现:
 *   运动学/电机层使用Blu3体坐标(dy=前进, dx=右移, CCW正),
 *   IMU(g_imu_yaw)也是CCW正.
 *   仅在场坐标边界做yaw符号映射: 外部θ(CW)=-内部θ(CCW).
 *   X(右)/Y(前)与Blu3场坐标dx(右)/dy(前)方向一致, 无需取反.
 */

#ifndef __MOVE_H
#define __MOVE_H

#include <stdint.h>
#include <stdbool.h>

/* ================================================================
 *  几何参数
 * ================================================================ */
#define MOVE_WHEEL_D        0.065f     /* 轮径 m */
#define MOVE_WHEEL_R        (MOVE_WHEEL_D * 0.5f)
#define MOVE_HALF_WB        0.085f     /* 半轮距(前后) m */
#define MOVE_HALF_TW        0.085f     /* 半轮距(左右) m */
#define MOVE_L_SUM          (MOVE_HALF_WB + MOVE_HALF_TW)  /* = 0.170 */

/* ================================================================
 *  单位换算
 * ================================================================ */
#define MOVE_RPM_PER_MPS    (60.0f / (6.28318530f * MOVE_WHEEL_R))
                                     /* RPM per m/s ≈ 587.7 */
#define MOVE_MPS_PER_RPM    (1.0f / MOVE_RPM_PER_MPS)
#define MOVE_ENC_PER_REV    65536.0f  /* S_CPOS 分辨率 */
#define MOVE_ENC_TO_M       (3.14159265f * MOVE_WHEEL_D / MOVE_ENC_PER_REV)
                                     /* ≈ 3.117e-6 m/count */

/* ================================================================
 *  控制参数 (实车标定)
 * ================================================================ */
#define MOVE_POS_KP             2.0f   /* 位置误差(m) → 速度(m/s) */
#define MOVE_MIN_SPEED          0.02f  /* 最低运动速度 m/s (克服静摩擦) */
#define MOVE_MAX_SPEED          0.5f   /* 默认最大移动速度 m/s */
#define MOVE_DEFAULT_TOL        0.005f /* 默认到位容差 m (5mm) */
#define MOVE_DEFAULT_TIMEOUT_MS 30000  /* 默认移动超时 ms */

/* 旋转 */
#define MOVE_YAW_KP             1.5f   /* 航向误差(°) → 角速度(m/s轮速) */
#define MOVE_YAW_HOLD_LIMIT     0.10f  /* 移动中航向修正限幅 m/s */
#define MOVE_YAW_TURN_LIMIT     0.30f  /* 原地旋转限幅 m/s */
#define MOVE_YAW_TOL_DEG        1.0f   /* 旋转到位容差 ° */
#define MOVE_YAW_TURN_TIMEOUT   15000  /* 旋转超时 ms */

/* 电机 */
#define MOVE_ACC_DEFAULT        10     /* Emm_V5加速度参数 */
#define MOVE_MOTOR_VEL_LIMIT    5000   /* RPM 上限 */
#define MOVE_CMD_DELAY_MS       2      /* 电机间CAN发帧间隔 */
#define MOVE_READ_TIMEOUT_MS    20     /* S_CPOS回读超时 */

/* 控制环 */
#define MOVE_CTRL_PERIOD_MS     30     /* 控制环周期 ms */

/* 圆弧控制 */
#define MOVE_ARC_SPEED          0.10f  /* 默认圆弧速度 m/s */
#define MOVE_ARC_KP_RADIAL      3.0f   /* 径向偏差修正增益 */
#define MOVE_ARC_TOL            0.010f /* 圆弧完成容差 m */
#define MOVE_ARC_TIMEOUT_MS     60000  /* 圆弧超时 ms */

/* ================================================================
 *  轴锁定选择
 * ================================================================ */
#define MOVE_AXIS_X  0   /* 主轴=X(右), 副轴=Y(前)锁定纠正 */
#define MOVE_AXIS_Y  1   /* 主轴=Y(前), 副轴=X(右)锁定纠正 */

/* ================================================================
 *  全局位置变量 (extern, 定义在move.c)
 * ================================================================ */
extern volatile float move_x;      /* 全局X坐标 m (右正) */
extern volatile float move_y;      /* 全局Y坐标 m (前进) */
extern volatile float move_yaw;    /* 全局航向 ° (CW正, =-g_imu_yaw) */
extern volatile float move_target_yaw; /* 运动开始时锁定的目标航向 ° (CW正) */

/* 活跃标志: 1=Move模块正在控制电机, OdomTask应跳过CAN读取 */
extern volatile uint8_t g_move_active;

/* ================================================================
 *  公共API
 * ================================================================ */

/* 位姿管理 */
void Move_InitPose(float x, float y, float yaw_deg);
void Move_ResetPose(void);

/* 速度设置 (内部+外部可用, 直接发CAN) */
void Move_SetRobotVelocity(float vx, float vy, float wz);
void Move_SetFieldVelocity(float vx_f, float vy_f, float wz);

/* 停止 */
void Move_Stop(void);

/* 阻塞式运动 (在NavTask中调用) */
uint8_t MoveToAccurateTimed(float tx, float ty, float max_speed,
                            float tol, uint32_t timeout_ms);
uint8_t MoveTo(float tx, float ty, float max_speed);
uint8_t RotateToTimed(float target_yaw_deg, float max_speed,
                      uint32_t timeout_ms);
uint8_t RotateTo(float target_yaw_deg, float max_speed);
uint8_t MoveToAxisLockTimed(float tx, float ty,
                            float main_speed, float lock_speed,
                            float main_tol, float lock_tol,
                            uint8_t axis, uint32_t timeout_ms);
uint8_t MoveToAxisLock(float tx, float ty,
                       float main_speed, float lock_speed,
                       float main_tol, float lock_tol, uint8_t axis);

/* 圆弧运动 */
uint8_t MoveArc(float cx, float cy, float radius,
                float start_angle_deg, float end_angle_deg,
                float speed);

#endif /* __MOVE_H */
