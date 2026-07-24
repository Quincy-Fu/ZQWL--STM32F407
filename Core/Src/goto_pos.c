/**
 * @file    goto_pos.c
 * @brief   导航规划层 — 移植自Blu3 Goto层
 *
 * 在Move层之上的导航增强:
 *   - 两阶段移动 (全速MoveTo → 低速精修)
 *   - 轴锁定移动 (ToX/ToY防麦轮漂移)
 *   - 段间Yaw校准 (折线路径每段原地校准航向)
 *   - 折线路径 (ToBlackPoint: 先X后Y)
 *
 * 所有函数阻塞式, 直接调用Move层API。
 * 坐标系: +X=右, +Y=前, CW正
 */

#include "goto_pos.h"
#include "move.h"
#include "imu_protocol.h"
#include <math.h>

/* (move_yaw / move_target_yaw 已在 move.h 声明, 无需额外 extern) */

/* ================================================================
 *  内部辅助
 * ================================================================ */

static float goto_abs(float v) { return v >= 0.0f ? v : -v; }

/**
 * @brief  判断当前是否在目标容差内
 */
static uint8_t goto_is_close(float tx, float ty)
{
    float dx = tx - move_x;
    float dy = ty - move_y;
    return (goto_abs(dx) <= GOTO_CLOSE_TOL &&
            goto_abs(dy) <= GOTO_CLOSE_TOL) ? 1 : 0;
}

/**
 * @brief  低速二次纠偏 (最多N轮, 每轮带超时)
 *
 * 在全速MoveTo之后, 如果还没到位, 用低速再跑一轮MoveToAccurateTimed。
 * 直接调用Move层, 不递归, 避免栈溢出。
 *
 * @return 1=纠偏后到位, 0=仍然偏
 */
static uint8_t goto_correct_to_point(float tx, float ty)
{
    for (uint8_t i = 0; i < GOTO_CORRECT_MAX_TIMES; i++) {
        if (goto_is_close(tx, ty)) return 1;

        MoveToAccurateTimed(tx, ty, GOTO_CORRECT_SPEED,
                            GOTO_CORRECT_TOL, GOTO_CORRECT_TIMEOUT_MS);
    }
    return goto_is_close(tx, ty);
}

/* ================================================================
 *  段间Yaw校准
 * ================================================================ */

/**
 * @brief  原地Yaw校准 (折线路径段间使用)
 *
 * 如果当前航向与move_target_yaw偏差 > SKIP_DEG,
 * 用RotateToTimed低速校准。超时不阻塞, 直接继续。
 */
void Goto_StabilizeYaw(void)
{
#if GOTO_YAW_STAB_ENABLE
    float err = move_yaw - move_target_yaw;
    if (goto_abs(err) > GOTO_YAW_SKIP_DEG) {
        RotateToTimed(move_target_yaw, GOTO_YAW_STAB_SPEED,
                      GOTO_YAW_STAB_TIMEOUT);
    }
#endif
}

/* ================================================================
 *  ToPoint — 两阶段点到点
 * ================================================================ */

/**
 * @brief  两阶段移动: 全速MoveTo + 低速纠偏
 *
 * 阶段1: 全速MoveTo(GOTO_DEFAULT_SPEED)
 * 阶段2: 如果GOTO_POINT_CORRECT_EN=1且阶段1未到位或不在容差内,
 *        用GOTO_CORRECT_SPEED低速再修一轮。
 *
 * @return 1=到位(close), 0=未到位
 */
uint8_t ToPointClose(float tx, float ty)
{
    /* 阶段1: 全速移动 */
    uint8_t ok = MoveTo(tx, ty, GOTO_DEFAULT_SPEED);

    /* 阶段2: 低速纠偏 */
#if GOTO_POINT_CORRECT_EN
    if (!ok || !goto_is_close(tx, ty)) {
        return goto_correct_to_point(tx, ty);
    }
#endif

    return ok;
}

void ToPoint(float tx, float ty)
{
    ToPointClose(tx, ty);
}

/* ================================================================
 *  ToX / ToY — 轴锁定移动
 * ================================================================ */

/**
 * @brief  轴锁定移X: 前进方向保持不变, 横移到目标X坐标
 *
 * 使用MoveToAxisLock(MOVE_AXIS_X):
 *   主轴=X(右), 副轴=Y(前)锁定
 *   主轴全速, 副轴低速纠正漂移
 *
 * 到点后如果GOTO_TOX_POST_CORRECT=1, 追加补偿轮。
 */
void ToX(float aimX)
{
    /* 锁定当前Y */
    float lock_y = move_y;

    MoveToAxisLock(aimX, lock_y,
                   GOTO_DEFAULT_SPEED, GOTO_AXIS_LOCK_SPEED,
                   GOTO_CLOSE_TOL, GOTO_AXIS_LOCK_TOL,
                   MOVE_AXIS_X);

#if GOTO_TOX_POST_CORRECT
    /* 追加补偿 */
    for (uint8_t i = 0; i < GOTO_AXIS_LOCK_MAX; i++) {
        if (goto_is_close(aimX, lock_y)) break;
        MoveToAxisLockTimed(aimX, lock_y,
                            GOTO_CORRECT_SPEED, GOTO_AXIS_LOCK_SPEED,
                            GOTO_CORRECT_TOL, GOTO_AXIS_LOCK_TOL,
                            MOVE_AXIS_X, GOTO_CORRECT_TIMEOUT_MS);
    }
#endif
}

/**
 * @brief  轴锁定移Y: 横移方向保持不变, 纵移到目标Y坐标
 *
 * 使用MoveToAxisLock(MOVE_AXIS_Y):
 *   主轴=Y(前), 副轴=X(右)锁定
 *
 * 到点后如果GOTO_TOY_POST_CORRECT=1, 追加补偿轮。
 */
void ToY(float aimY)
{
    float lock_x = move_x;

    MoveToAxisLock(lock_x, aimY,
                   GOTO_DEFAULT_SPEED, GOTO_AXIS_LOCK_SPEED,
                   GOTO_CLOSE_TOL, GOTO_AXIS_LOCK_TOL,
                   MOVE_AXIS_Y);

#if GOTO_TOY_POST_CORRECT
    for (uint8_t i = 0; i < GOTO_AXIS_LOCK_MAX; i++) {
        if (goto_is_close(lock_x, aimY)) break;
        MoveToAxisLockTimed(lock_x, aimY,
                            GOTO_CORRECT_SPEED, GOTO_AXIS_LOCK_SPEED,
                            GOTO_CORRECT_TOL, GOTO_AXIS_LOCK_TOL,
                            MOVE_AXIS_Y, GOTO_CORRECT_TIMEOUT_MS);
    }
#endif
}

/* ================================================================
 *  ToBlackPoint — 折线路径 (先X后Y, 段间Yaw校准)
 * ================================================================ */

/**
 * @brief  L形折线: StabilizeYaw → ToX → StabilizeYaw → ToY → StabilizeYaw
 *
 * 保持车头方向不变, 先沿X轴移动到位, 再沿Y轴移动到位。
 * 每段前后做Yaw校准, 防止斜切。
 *
 * 注: "BlackPoint"名字来自Blu3的黑线网格概念,
 *     这里只是先X后Y的折线, 不涉及实际黑线循迹。
 */
void ToBlackPoint(float tx, float ty)
{
    Goto_StabilizeYaw();
    ToX(tx);
    Goto_StabilizeYaw();
    ToY(ty);
    Goto_StabilizeYaw();
}

uint8_t ToBlackPointClose(float tx, float ty)
{
    ToBlackPoint(tx, ty);
    return goto_is_close(tx, ty);
}
