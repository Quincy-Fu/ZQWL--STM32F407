/**
 * @file    goto_pos.h
 * @brief   导航规划层 — 移植自Blu3 Goto层
 *
 * 在Move层之上, 提供:
 *   - ToPoint: 两阶段移动(全速→低速纠偏)
 *   - ToX/ToY: 轴锁定移动(防麦轮漂移)
 *   - ToBlackPoint: 折线路径(先X后Y, 段间Yaw校准)
 *   - StabilizeYaw: 段间原地航向校准
 *
 * 所有函数阻塞式, 在NavTask中调用。
 * 坐标系: +X=右, +Y=前, CW正 (与move.h一致)
 */

#ifndef __GOTO_POS_H
#define __GOTO_POS_H

#include <stdint.h>

/* ================================================================
 *  Goto参数 (实车标定)
 * ================================================================ */

/* 第一次主移动 */
#define GOTO_DEFAULT_SPEED      0.70f   /* m/s [第四档提速, 0.60→0.70] */

/* 二次纠偏 (低速精修) */
#define GOTO_CORRECT_SPEED      0.25f   /* m/s [第二档提速, 配合减速区快速接近+平滑制动] */
#define GOTO_CORRECT_TOL        0.005f  /* m (5mm) */
#define GOTO_CORRECT_MAX_TIMES  3       /* 最多纠偏轮数 (斜线残余误差需多轮修正) */
#define GOTO_CORRECT_TIMEOUT_MS 2000    /* 每轮超时 ms */

/* 轴锁定 */
#define GOTO_AXIS_LOCK_SPEED    0.030f  /* m/s 副轴纠正速度 [0.015→0.03: 到位判定加副轴检查后需更快收尾] */
#define GOTO_AXIS_LOCK_TOL      0.008f  /* m 副轴允许漂移量 */
#define GOTO_AXIS_LOCK_MAX      1       /* 轴锁定后追加补偿轮数 */

/* 段间Yaw校准 */
#define GOTO_YAW_STAB_ENABLE    1       /* 1=启用段间Yaw校准 */
#define GOTO_YAW_SKIP_DEG       1.5f    /* 误差小于此值跳过校准 ° */
#define GOTO_YAW_STAB_SPEED     0.06f   /* m/s 校准旋转速度 */
#define GOTO_YAW_STAB_TIMEOUT   800     /* ms 校准超时 */

/* 二次纠偏开关 */
#define GOTO_POINT_CORRECT_EN   1       /* ToPoint到点后启用二次纠偏 */
#define GOTO_TOX_POST_CORRECT   1       /* ToX结束后追加补偿 */
#define GOTO_TOY_POST_CORRECT   0       /* ToY结束后追加补偿 (默认关,防X被带偏) */

/* 到位判定 */
#define GOTO_CLOSE_TOL          0.008f  /* IsClose判定容差 m */

/* ================================================================
 *  公共API
 * ================================================================ */

/**
 * @brief  两阶段点到点: 全速MoveTo + 低速纠偏
 * @return 1=到位(close), 0=未到位
 */
uint8_t ToPointClose(float tx, float ty);

/**
 * @brief  同上但不返回结果
 */
void ToPoint(float tx, float ty);

/**
 * @brief  轴锁定移X(右): 主轴X, 副轴Y(前)锁定
 */
void ToX(float aimX);

/**
 * @brief  轴锁定移Y(前): 主轴Y, 副轴X(右)锁定
 */
void ToY(float aimY);

/**
 * @brief  折线路径: 先ToX再ToY, 段间Yaw校准
 */
void ToBlackPoint(float tx, float ty);

/**
 * @brief  同上, 返回是否到位
 */
uint8_t ToBlackPointClose(float tx, float ty);

/**
 * @brief  段间原地Yaw校准 (误差<SKIP_DEG跳过)
 */
void Goto_StabilizeYaw(void);

#endif /* __GOTO_POS_H */
