/**
 * @file    move.h
 * @brief   底层运动控制模块 — 移植自Blu3 Move层
 *
 * 提供：位置控制(MoveTo)、轴锁定(MoveToAxisLock)、原地旋转(RotateTo)、
 *       圆弧跟踪(MoveArc)、里程计积分、急停。
 *
 * 控制环30ms周期：P环位置控制 + 编码器回读XY + yaw反馈。
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
 *   yaw反馈由MOVE_YAW_USE_IMU选择: 0=编码器, 1=IMU优先且编码器掉线兜底。
 *   场坐标计算内部仍用数学CCW正, 入口处由move_yaw转换.
 *   X(右)/Y(前)与Blu3场坐标dx(右)/dy(前)方向一致, 无需取反.
 */

#ifndef __MOVE_H
#define __MOVE_H

#include <stdint.h>
#include <stdbool.h>

/* ================================================================
 *  路径点类型 (路径点跟踪器)
 * ================================================================ */
#define PATH_MODE_NORMAL  0   /* 普通通过点: 不停, 进入通过半径后切下一段 */
#define PATH_MODE_KEY     1   /* 关键点: 带目标姿态角, 进关键半径且姿态基本到位后切段 */

typedef struct {
    float   x;            /* 场坐标 X (右) m */
    float   y;            /* 场坐标 Y (前) m */
    float   target_theta; /* 目标姿态角 ° (CW+); mode=1时作为分段yaw目标 */
    uint8_t mode;         /* 0=普通通过点, 1=关键点/姿态目标点 */
    uint8_t _pad[3];      /* 对齐填充 → 16B/点 */
} PathPt_t;

/* ================================================================
 *  几何参数
 * ================================================================ */
#define MOVE_WHEEL_D        0.065f     /* 轮径 m */
#define MOVE_WHEEL_R        (MOVE_WHEEL_D * 0.5f)
#define MOVE_HALF_WB        0.085f     /* 半轴距(前后) m: 实测85mm */
#define MOVE_HALF_TW        0.083f     /* 半轮距(左右) m: 轮距微调为83mm */
#define MOVE_L_SUM          (MOVE_HALF_WB + MOVE_HALF_TW)  /* = 0.168 */
#define MOVE_YAW_L_SUM      MOVE_L_SUM /* 普通移动/原地旋转yaw有效L_SUM: 使用物理168mm */
#define MOVE_ARC_YAW_L_SUM  0.164f     /* 圆弧段yaw有效L_SUM: 标定测试值164mm */

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
#define MOVE_POS_KD             3.0f   /* 超速阻尼增益 [加重后过冲, 从2.0→3.0增强刹车] */
#define MOVE_MIN_SPEED          0.02f  /* 最低运动速度 m/s (克服静摩擦) */
#define MOVE_MAX_SPEED          0.8f   /* 默认最大移动速度 m/s [0.7→0.8: 配合0.70巡航, 留余量] */
#define MOVE_DEFAULT_TOL        0.005f /* 默认到位容差 m (5mm) */
#define MOVE_STOP_LATENCY_S     0.025f /* 停止延迟 s (CAN急停20ms+电机响应5ms), 预测制动用 */
#define MOVE_STOP_SETTLE_MS     80u    /* 平移动作停电机后复检等待: 防刚进容差但仍在滑行就返回 */
#define MOVE_STOP_SYNC_REPEATS  2u     /* 急停同步帧重复次数: 降低单个电机漏收导致偏航的概率 */
#define MOVE_DEFAULT_TIMEOUT_MS 30000  /* 默认移动超时 ms */
#define MOVE_DECEL_DIST         1.60f  /* 减速区距离 m [1.20→1.60: 加重后惯性大, 给更长刹车距离, a≈0.20] */
#define MOVE_CREEP_DIST         0.04f  /* 蠕变区距离 m: 进入后硬限速 */
#define MOVE_CREEP_SPEED        0.03f  /* 蠕变区最高速度 m/s [0.04→0.03: 加重后末端更柔, 减小小过冲] */

/* 旋转 (按Blu3比例带设计: KP×比例带≈限幅, P控制自然减速) */
#define MOVE_YAW_KP             0.004f /* 航向误差(°) → 角速度(m/s轮速) [Blu3=1.2°→RPM÷293.9] */
#define MOVE_YAW_HOLD_LIMIT     0.12f  /* 移动中航向修正限幅 m/s [第二档提速同步提高] */
#define MOVE_YAW_HOLD_DEADZONE  1.0f   /* 航向保持死区 ° */
#define MOVE_YAW_TURN_LIMIT     0.25f  /* 原地旋转限幅 m/s [0.35→0.25: IMU安装/悬挂不稳, 降低转动冲击] */
#define MOVE_YAW_DECEL_DEG      45.0f  /* 旋转减速区 ° [50→45: 恢复P环+减速区, 减小过冲] */
#define MOVE_YAW_FINE_SPEED     0.015f /* 近距离(<5°)最低旋转速度 m/s [0.008→0.015: 解决差一点不到位] */
#define MOVE_YAW_TOL_DEG        0.2f   /* 旋转到位容差 ° */
#define MOVE_YAW_ACCEPT_DEG     0.4f   /* 首停后接受阈值 ° (比TOL宽, 容留松手后0.1~0.3°惯性漂移, 防极限环微调) */
#define MOVE_YAW_STOP_LATENCY_MS 50    /* 旋转停止延迟 ms [100→50: 编码器无IMU通信延迟, 仅CAN+电机响应+惯性] */
#define MOVE_YAW_SETTLE_WAIT_MS 120    /* 首停后最短等待 ms: 等机械滑行后再判定是否真的到位 */
#define MOVE_YAW_SETTLE_FRAMES  2      /* 等待后连续控制周期在容差内才返回完成, 防止预测停止后提前返回 */
#define MOVE_YAW_TURN_TIMEOUT   15000  /* 旋转超时 ms */
#define MOVE_IMU_ROTATE_TOL_DEG 0.5f   /* IMU原地旋转容差: IMU噪声/安装误差下不能用编码器级0.2° */
#define MOVE_IMU_ACCEPT_DEG     0.8f   /* IMU停稳后接受阈值: 防止0.5°附近反复小幅拉回形成震荡 */
#define MOVE_IMU_STOP_LATENCY_S 0.03f  /* IMU旋转预测停止延迟: 旧闭环经验值, 避免停后反复拉回 */

/* 移动中转角: 用于圆弧前短段从直线姿态平滑切到圆弧切线姿态 */
#define MOVE_GOTO_YAW_DONE_RATIO 0.75f /* 路径前75%完成目标yaw变化, 留后25%稳定进圆弧 */
#define MOVE_GOTO_YAW_ACCEPT_DEG 1.0f  /* 移动+转角完成接受阈值, 防止末端为小角度长时间微调 */

/* 电机 */
#define MOVE_ACC_DEFAULT        5      /* Emm_V5加速度参数 [30→5: 旧值致换向6s, Blu3=5] */
#define MOVE_MOTOR_VEL_LIMIT    5000   /* RPM 上限 */
#define MOVE_CMD_DELAY_MS       3      /* 电机间CAN发帧间隔 [5→3→2→3: 2ms丢帧不稳定,3ms=23×帧时间裕量稳定] */
#define MOVE_READ_TIMEOUT_MS    20     /* S_CPOS回读超时 */
#define MOVE_RAMP_TIME_MS       200    /* 软启动时间 ms: 0→满速线性加速, 减少起步打滑 */

/* 控制环 */
#define MOVE_CTRL_PERIOD_MS     1      /* 控制环末尾延时 ms [5→1: I/O本身已提供节拍,省4ms/循环] */
#define MOVE_IMU_TIMEOUT_MS     200    /* IMU通信超时 ms (5×40ms帧间隔, 防接触不良疯转) */
#define MOVE_YAW_USE_IMU        1      /* 1=编译IMU yaw支持; 实际使用由运行期yaw源切换命令决定 */
#define MOVE_YAW_SOURCE_ENCODER 0u     /* 运行期yaw反馈源: 编码器 */
#define MOVE_YAW_SOURCE_IMU     1u     /* 运行期yaw反馈源: IMU优先, 异常时编码器兜底 */
#define MOVE_YAW_SOURCE_DEFAULT MOVE_YAW_SOURCE_ENCODER
#define MOVE_IMU_YAW_SIGN       (1.0f)  /* 当前IMU接线/安装: 原始yaw已与系统CW正方向一致 */
#define MOVE_IMU_YAW_MAX_STEP_DEG 25.0f /* 单次yaw反馈跳变上限, 超过则认为IMU瞬态异常并用编码器兜底 */

/* 圆弧控制 */
#define MOVE_ARC_SPEED          0.25f  /* 默认圆弧速度 m/s */
#define MOVE_ARC_CREEP_SPEED    0.025f /* 圆弧末端最低切向速度 m/s, 独立于GOTO蠕变速度 */
#define MOVE_ARC_KP_RADIAL      5.0f   /* 径向偏差修正增益 [验证甜点: 3.0→5.0(0.6稳),5.0+CMD2丢帧≠振荡,3.5太低不稳→退回5.0] */
#define MOVE_ARC_TOL            0.010f /* 圆弧完成容差 m */
#define MOVE_ARC_TIMEOUT_MS     60000  /* 圆弧超时 ms */
#define MOVE_ARC_DECEL_DEG      30.0f  /* 末端减速区: [15→30: 电机实际速度3x命令,需更长减速时间] */
#define MOVE_ARC_ACCEL          0.15f  /* 圆弧切向最大加/减速度 m/s² [启动限加速度,末端v=sqrt(2*a*s)] */
#define MOVE_ARC_STOP_LATENCY_MS 130   /* 停止延迟ms: 原120ms对应旧重量, 结构增重后惯性增大, 130ms补偿过冲 */
#define MOVE_ARC_SETTLE_KP      2.0f   /* 弧线settle位置P增益 (修正速度=误差m×此值, 2.0: 3cm→6cm/s) */
#define MOVE_ARC_SETTLE_MAX_SPEED 0.06f /* 弧线settle修正最高速度 m/s (6cm/s, gentle, 防位置修正冲击) */
#define MOVE_ARC_POS_TOL        0.010f /* 弧线到位位置容差 m (1cm, 配合KP=2时P输出=MIN_SPEED临界, 静摩擦刚好能克服) */
#define MOVE_ARC_SWEEP_TOL_DEG  1.0f   /* 圆弧位置扫角完成容差: 防yaw已到但弧长/半径少走时提前结束 */
#define MOVE_ARC_XY_GAIN        1.04f  /* 圆弧XY切向速度补偿: >1用于修正实车弧长/半径少走 */
#define MOVE_ARC_OUTWARD_COMP_K 0.06f  /* 圆弧速度相关径向外推补偿: comp=K*v²/R, 抵消高速切内圈 */
#define MOVE_ARC_OUTWARD_COMP_MAX 0.025f /* 圆弧径向外推补偿限幅 m/s, 防止小半径/高速时过补偿 */

/* 路径执行 (MCU内部顺序执行位置点/转角点, 稳定优先) */
#define MOVE_WP_MAX_PTS         256    /* 路径点缓冲上限 (256×16B=4KB .bss) */
#define MOVE_WP_END_TOL         0.010f /* 末端到位容差 m */
#define MOVE_WP_SPEED           0.30f  /* 默认路径速度 m/s */
#define MOVE_WP_TIMEOUT_MS      15000  /* 路径跟踪超时 ms */
#define MOVE_WP_MOVE_DECEL_DIST 0.35f  /* path点到点减速区 m: 短于普通MoveTo的1.60m, 提高短段速度 */
#define MOVE_WP_PASS_RADIUS     0.080f /* 普通通过点提前切段半径 m: 减少折线点停车/硬拐 */
#define MOVE_WP_KEY_PASS_RADIUS 0.030f /* 关键点通过半径 m: 保证关键点精度, 过大等于切角 */
#define MOVE_WP_YAW_KP          0.012f /* 圆弧/路径内部航向P增益 (°→m/s轮速差) */
#define MOVE_WP_YAW_MAX         0.30f  /* 圆弧/路径内部航向修正限幅 m/s */
#define MOVE_WP_ZERO_YAW_LIMIT  0.22f /* path重复坐标转角段限速: 略低于普通TURNTO, 兼顾速度和震荡 */

/* 视觉微调 (到位后视觉闭环方向微调, 体坐标系) */
#define MOVE_VISION_NUDGE_SPEED     0.05f  /* 微调速度 m/s (慢于Blu3 GOTO_CORRECT_SPEED 0.25, 略高于蠕动区0.04, 供视觉闭环跟踪) */
#define MOVE_VISION_NUDGE_TIMEOUT_MS 2000 /* 微调超时 ms: 无新命令自动停止+锁死 (视觉帧率5-10Hz, 2s=10-20倍裕量) */

/* 视觉 dx/dy 一次性修正: 复用位置环, 车体坐标系(+X右,+Y前)
 * 内部会拆成单轴小步，并在停车后做编码器复检补偿。 */
#define MOVE_FINE_LOOP_SPEED       0.07f   /* 视觉微调位置环最高速度 m/s, 降低末端冲过 */
#define MOVE_FINE_LOOP_TOL         0.0015f /* 视觉微调内部到位容差 m, 上位机最终容差可略放宽 */
#define MOVE_FINE_LOOP_DECEL_DIST  0.035f  /* 小距离专用减速区, 缩小后减少全程蠕动 */
#define MOVE_FINE_LOOP_MIN_SPEED   0.008f  /* FINE专用最低速度, 降低小步末端残差 */
#define MOVE_FINE_LOOP_CREEP_SPEED 0.012f  /* FINE专用蠕动限速 */
#define MOVE_FINE_AXIS_EPS         0.0005f /* 小于0.5mm的单轴分量直接忽略 */
#define MOVE_FINE_RECHECK_SETTLE_MS 150u   /* 停车后等待机械/编码器残余更新 */
#define MOVE_FINE_RECHECK_MAX      3u      /* 每个单轴段最多追加三次补偿 */
#define MOVE_FINE_LOOP_TIMEOUT_MS  12000  /* FINE_MOVE最大等待时间 ms，避免200mm回退实际已动但闭环超时误报 */

/* 开环车体位移: 用 Emm_V5 位置模式直接给四轮相对脉冲。
 * 用于 ring 对准后的固定前推/后退，不替代视觉 dx/dy 闭环微调。 */
#define MOVE_BODY_POS_PULSES_PER_REV 3200u  /* Emm位置模式 16细分: 3200脉冲/圈 */
#define MOVE_BODY_POS_MECANUM_FACTOR 1.0f /* 与编码器里程计一致: 纯前进/横移按完整轮周πD换算 */
#define MOVE_BODY_POS_VEL_RPM        300u   /* 固定推送/回退速度, 比位置环快但保守 */
#define MOVE_BODY_POS_ACC            5u     /* 与现有底盘加速度档一致 */
#define MOVE_BODY_POS_CMD_GAP_MS     30u    /* 13字节位置命令发给4个电机的间隔, 降低单电机漏收概率 */
#define MOVE_BODY_POS_SETTLE_MS      250u   /* 位置模式估算完成后的停稳余量 */
#define MOVE_BODY_POS_MARGIN_MS      350u   /* 加减速和驱动器处理余量 */
#define MOVE_BODY_POS_MIN_WAIT_MS    2000u  /* 按厂家位置模式例程, 发命令后至少等待2秒 */
#define MOVE_BODY_POS_RELEASE_MS     80u    /* 返回上位机前同步停四轮并释放位置模式残留 */
#define MOVE_BODY_POS_EXIT_CMD_GAP_MS 10u   /* BODY_POS后切回速度模式0速时的电机间隔, 防个别电机漏收 */
#define MOVE_BODY_POS_EXIT_SYNC_MS    30u   /* BODY_POS后切回速度模式0速前的同步等待 */
#define MOVE_BODY_POS_MAX_DIST       0.50f  /* 防误发大距离开环位移 */
#define MOVE_BODY_POS_VERIFY_MIN_RATIO 0.50f /* 回读校验: 单轮实际位移至少达到期望的50% */
#define MOVE_BODY_POS_VERIFY_MIN_COUNTS 1000 /* 期望位移过小时不做单轮校验, 避免量化误判 */

/* C/D专用固定连续段开关与速度表。
 * 仅由 TYPE_CMD_CD_FIXED_ARC 触发；宏置0时命令直接返回失败, 不影响原GOTO/ARC流程。 */
#define MOVE_CD_FIXED_ARC_ENABLE       1u
#define MOVE_CD_FIXED_TRACK_ARC        1u      /* 1=过渡段后用闭环圆弧; 0=全程固定四轮速度表 */
#define MOVE_CD_FIXED_LOOP_MS          5u
#define MOVE_CD_FIXED_PRE_MS           1200u
#define MOVE_CD_FIXED_PRE_VX          (-0.0236f) /* 车体+X右, 负值=左移；过渡段终点超点时优先调小平移量 */
#define MOVE_CD_FIXED_PRE_VY           0.1275f  /* 车体+Y前, 正值=向前走；当前为原平移速度约65% */
#define MOVE_CD_FIXED_PRE_WZ           0.0513f  /* 过渡段转到-69°用, 不用于调圆弧半径 */
#define MOVE_CD_FIXED_START_X         (-0.900f)
#define MOVE_CD_FIXED_START_Y          0.250f
#define MOVE_CD_FIXED_START_YAW       (-69.0f)
#define MOVE_CD_FIXED_ARC_RADIUS       0.869f
#define MOVE_CD_FIXED_ARC_SWEEP_DEG    130.0f
#define MOVE_CD_FIXED_ARC_DIR          1
#define MOVE_CD_FIXED_ARC_MS           6570u
#define MOVE_CD_FIXED_ARC_VY           0.3000f
#define MOVE_CD_FIXED_ARC_WZ           0.0620f  /* 固定速度表兜底值; 闭环圆弧模式下不使用 */

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
extern volatile float move_yaw;    /* 全局航向 ° (CW正; 由当前yaw反馈源维护) */
extern volatile float move_target_yaw; /* 运动开始时锁定的目标航向 ° (CW正) */
extern volatile uint8_t g_move_yaw_source; /* MOVE_YAW_SOURCE_* */

/* 活跃标志: 1=Move模块正在控制电机, OdomTask应跳过CAN读取 */
extern volatile uint8_t g_move_active;

/* ================================================================
 *  公共API
 * ================================================================ */

/* 位姿管理 */
void Move_InitPose(float x, float y, float yaw_deg);
void Move_ResetPose(void);
float Move_GetYaw(void);
void Move_UpdateYawFeedback(float encoder_delta_cw_deg);
uint8_t Move_SetYawSource(uint8_t source);
uint8_t Move_GetYawSource(void);

/* 速度设置 (内部+外部可用, 直接发CAN) */
void Move_SetRobotVelocity(float vx, float vy, float wz);
void Move_SetFieldVelocity(float vx_f, float vy_f, float wz);

/* 停止 */
void Move_Stop(void);

/* 阻塞式运动 (在NavTask中调用) */
uint8_t MoveToAccurateTimed(float tx, float ty, float max_speed,
                            float tol, uint32_t timeout_ms);
uint8_t MoveTo(float tx, float ty, float max_speed);
uint8_t MoveToYawTimed(float tx, float ty, float target_yaw_deg,
                       float max_speed, uint32_t timeout_ms,
                       uint8_t stop_on_done);
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

/* 视觉微调: dx/dy为车体坐标相对位移, 内部转换到场地坐标后复用位置环 */
uint8_t MoveFinePositionBody(float dx_body_m, float dy_body_m,
                             uint32_t timeout_ms);

/* 开环车体相对位移: dx/dy为车体坐标, 使用四轮位置模式执行 */
uint8_t MoveBodyPositionOpenLoop(float dx_body_m, float dy_body_m);

/* C/D专用固定连续段: 写死速度表, 用于验证连贯过渡+圆弧 */
uint8_t MoveCDFixedArcTrack(void);

/* 圆弧运动 */
uint8_t MoveArc(float cx, float cy, float radius,
                float start_angle_deg, float end_angle_deg,
                float speed);

/* 圆弧轨迹跟踪 (弧长参数化, 前馈v/R + P反馈)
 * 从当前位置/航向自动计算圆心, 沿弧线运动sweep_deg度.
 * 前馈wz=(v/R)×L_SUM消除曲线上稳态滞后, P反馈修正 transient 误差.
 * 径向P修正保持机器人在弧线上.
 * dir: +1=CW(右转), -1=CCW(左转)
 * sweep_deg: 扫过角度° (180=半圆, 360=整圆)
 */
uint8_t MoveArcTrack(float radius, float speed, int dir,
                     float sweep_deg, uint32_t timeout_ms);

/* 圆弧轨迹跟踪，并在实际弧进度达到触发角度时下发转盘槽位切换。 */
uint8_t MoveArcTrackWithTurntable(float radius, float speed, int dir,
                                  float sweep_deg,
                                  float trigger1_deg, uint8_t slot1,
                                  float trigger2_deg, uint8_t slot2,
                                  float trigger3_deg, uint8_t slot3,
                                  uint32_t timeout_ms);

/* 路径执行 (MCU内部顺序执行位置点/转角点)
 * 上位机发路径点(x,y,target_theta,mode) → Move_PathBegin + 多次Move_PathAddPoint 装载 →
 * NavTask调用MovePathTrack阻塞跟踪:
 *   坐标不同: 调用 MoveToAccurateTimed 到目标点
 *   坐标相同且mode=1: 调用 RotateTo 到 target_theta */
void    Move_PathBegin(uint8_t count, float speed);                      /* ISR: 预告点数+速度, 清零缓冲 */
void    Move_PathAddPoint(float x, float y, float target_theta, uint8_t mode); /* ISR: 追加一个路径点 */
uint8_t MovePathTrack(void);                                             /* NavTask: 阻塞跟踪, 1=完成 0=超时/中止 */

#endif /* __MOVE_H */
