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
 *   运动学/电机层使用Blu3体坐标(dy=前进, dx=右移, CW正),
 *   IMU(g_imu_yaw)是CCW正.
 *   场坐标边界做yaw符号映射: 外部θ(CW)=-内部θ(CCW).
 *   X(右)/Y(前)与Blu3场坐标dx(右)/dy(前)方向一致, 无需取反.
 */

#ifndef __MOVE_H
#define __MOVE_H

#include <stdint.h>
#include <stdbool.h>

/* ================================================================
 *  路径点类型 (Pure Pursuit)
 * ================================================================ */
typedef struct {
    float x;        /* 场坐标 X (右) m */
    float y;        /* 场坐标 Y (前) m */
    float heading;  /* 目标航向 ° (CW正); =MOVE_PP_FREE_HEADING 表示自由点(不约束航向) */
} PathPt_t;

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
                                     /* RPM per m/s ≈ 293.9 (标准麦轮运动学 V=ωR) */
#define MOVE_MPS_PER_RPM    (1.0f / MOVE_RPM_PER_MPS)
#define MOVE_ENC_PER_REV    65536.0f  /* S_CPOS 分辨率 */
#define MOVE_ENC_TO_M       (3.14159265f * MOVE_WHEEL_D / MOVE_ENC_PER_REV)
                                     /* ≈ 3.117e-6 m/count */
#define MOVE_ENC_MAX_DELTA  50000    /* 合理性阈值: 正常88RPM×200ms≈191counts,
                                      * 50000=260倍余量, 拦截符号位损坏(200000+) */

/* ================================================================
 *  控制参数 (实车标定)
 * ================================================================ */
#define MOVE_POS_KP             2.0f   /* 位置误差(m) → 速度(m/s) [KP=3.0震荡过冲,退回] */
#define MOVE_POS_KD             2.0f   /* 超速阻尼增益 [仅作用于v_actual-P命令的超速量, 不影响正常巡航] */
#define MOVE_MIN_SPEED          0.02f  /* 最低运动速度 m/s (克服静摩擦) */
#define MOVE_MAX_SPEED          0.8f   /* 默认最大移动速度 m/s [0.7→0.8: 配合0.70巡航, 留余量] */
#define MOVE_DEFAULT_TOL        0.005f /* 默认到位容差 m (5mm) */
#define MOVE_STOP_LATENCY_S     0.025f /* 停止延迟 s (CAN急停20ms+电机响应5ms), 预测制动用 */
#define MOVE_DEFAULT_TIMEOUT_MS 30000  /* 默认移动超时 ms */
#define MOVE_DECEL_DIST         1.20f  /* 减速区距离 m [1.00→1.20: a=0.204, 回到验证值~0.21附近, 电机跟踪极限] */
#define MOVE_CREEP_DIST         0.04f  /* 蠕变区距离 m: 进入后硬限速 */
#define MOVE_CREEP_SPEED        0.04f  /* 蠕变区最高速度 m/s (40mm/s, Move_Stop无冲击) */

/* 旋转 (按Blu3比例带设计: KP×比例带≈限幅, P控制自然减速) */
#define MOVE_YAW_KP             0.004f /* 航向误差(°) → 角速度(m/s轮速) [Blu3=1.2°→RPM÷293.9] */
#define MOVE_YAW_HOLD_LIMIT     0.12f  /* 移动中航向修正限幅 m/s [第二档提速同步提高] */
#define MOVE_YAW_HOLD_DEADZONE  2.0f   /* 航向保持死区 ° (误差<此值不修正, 防直线摇摆) */
#define MOVE_YAW_TURN_LIMIT     0.35f  /* 原地旋转限幅 m/s [0.40→0.35: 180°仍过冲, 再降一档] */
#define MOVE_YAW_DECEL_DEG      35.0f  /* 旋转减速区 ° [30→35: 增加减速余量] */
#define MOVE_YAW_FINE_SPEED     0.015f /* 近距离(<5°)最低旋转速度 m/s [0.008→0.015: 解决差一点不到位] */
#define MOVE_YAW_TOL_DEG        0.5f   /* 旋转到位容差 ° [原1.0] */
#define MOVE_YAW_TURN_TIMEOUT   15000  /* 旋转超时 ms */

/* 电机 */
#define MOVE_ACC_DEFAULT        30     /* Emm_V5加速度参数 [10→30: 电机更快跟踪减速命令] */
#define MOVE_MOTOR_VEL_LIMIT    5000   /* RPM 上限 */
#define MOVE_CMD_DELAY_MS       5      /* 电机间CAN发帧间隔 [10→5: 2ms曾丢帧, 5ms=38×帧时间裕量] */
#define MOVE_READ_TIMEOUT_MS    20     /* S_CPOS回读超时 */
#define MOVE_RAMP_TIME_MS       200    /* 软启动时间 ms: 0→满速线性加速, 减少起步打滑 */

/* 控制环 */
#define MOVE_CTRL_PERIOD_MS     5      /* 控制环周期 ms [30→5: I/O本身~28ms已提供节拍, 末尾延时纯浪费] */
#define MOVE_IMU_TIMEOUT_MS     200    /* IMU通信超时 ms (5×40ms帧间隔, 防接触不良疯转) */

/* 圆弧控制 */
#define MOVE_ARC_SPEED          0.10f  /* 默认圆弧速度 m/s */
#define MOVE_ARC_KP_RADIAL      3.0f   /* 径向偏差修正增益 */
#define MOVE_ARC_TOL            0.010f /* 圆弧完成容差 m */
#define MOVE_ARC_TIMEOUT_MS     60000  /* 圆弧超时 ms */

/* Pure Pursuit 路径跟踪 (上位机预插值密集点, MCU端自主前视点跟踪) */
#define MOVE_PP_MAX_PTS         256    /* 路径点缓冲上限 (256×12B=3KB .bss) */
#define MOVE_PP_FREE_HEADING    999.0f /* 航向哨兵: 点heading=此值表示自由点(不约束航向) */
/* 自适应前视: L = clamp(GAIN×R_local, MIN, MAX), R_local=局部曲率半径
 * 切弯公式 r=√(R²-L²): 固定L对紧弯切太多(R=0.5,L=0.25→r=0.43切7cm);
 * 按曲率缩放后, 急弯自动缩短前视约束切弯误差, 直道顶到MAX保平滑 */
#define MOVE_PP_LOOKAHEAD_GAIN  0.25f  /* 前视距离 = 局部曲率半径 × 此系数 */
#define MOVE_PP_LOOKAHEAD_MIN   0.08f  /* 前视下限 m (急弯, 防切弯过大) */
#define MOVE_PP_LOOKAHEAD_MAX   0.25f  /* 前视上限 m (直道/缓弯, 保平滑) */
#define MOVE_PP_SPEED           0.30f  /* 默认路径速度 m/s (曲线比直线GOTO慢, 弯更稳) */
#define MOVE_PP_TOL             0.010f /* 终点到位容差 m */
#define MOVE_PP_TIMEOUT_MS      60000  /* 路径跟踪超时 ms */
#define MOVE_PP_YAW_LIMIT       0.25f  /* 路径航向通道限幅 m/s [≈84°/s; 航向是主目标,强于HOLD(0.12)弱于TURN(0.35)] */

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

/* Pure Pursuit 路径跟踪
 * 上位机预插值密集点 → Move_PathBegin + 多次Move_PathAddPoint 装载 →
 * NavTask调用MovePathPurePursuit阻塞跟踪。航向在约束点间按弧长线性插值。 */
void    Move_PathBegin(uint8_t count, float speed);          /* ISR: 预告点数+速度, 清零缓冲 */
void    Move_PathAddPoint(float x, float y, float heading);  /* ISR: 追加一个路径点 */
uint8_t MovePathPurePursuit(void);                           /* NavTask: 阻塞跟踪, 1=完成 0=超时/中止 */

#endif /* __MOVE_H */
