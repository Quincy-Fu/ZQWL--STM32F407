/**
 * @file    move.c
 * @brief   底层运动控制模块 — 移植自Blu3 Move层
 *
 * 30ms控制环: P环位置控制 + 编码器回读里程计 + IMU航向保持。
 * 阻塞式API, 在NavTask中调用。
 *
 * 坐标系: vy>0=前进, vx>0=右移, wz>0=CCW
 * 电机地址: FL=0x01, FR=0x02, RL=0x03, RR=0x04
 * 右轮(FR/RR)镜像安装, motor_emit方向反转。
 */

#include "move.h"
#include "Emm_V5.h"
#include "can.h"
#include "imu_protocol.h"
#include <math.h>

/* FreeRTOS */
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* ================================================================
 *  全局变量定义
 * ================================================================ */
volatile float move_x = 0.0f;
volatile float move_y = 0.0f;
volatile float move_yaw = 0.0f;
volatile float move_target_yaw = 0.0f;
volatile uint8_t g_move_active = 0;

/* ================================================================
 *  外部引用 (freertos.c / imu_protocol.c)
 * ================================================================ */
extern volatile float g_imu_yaw;       /* IMU偏航角, 度, 上电归零 */
extern volatile float g_odom_x;        /* 里程计X, 同步用 */
extern volatile float g_odom_y;        /* 里程计Y, 同步用 */
extern volatile float g_odom_theta;    /* 里程计theta, 同步用 */

/* ================================================================
 *  静态变量
 * ================================================================ */
static const uint8_t wheel_addr[4] = {
    MOTOR_FL, MOTOR_FR, MOTOR_RL, MOTOR_RR
};
/* 右轮(FR=1, RR=3)镜像安装: true表示方向需反转 */
static const uint8_t wheel_mirror[4] = {0, 1, 0, 1};

/* 编码器上一次读数 (S_CPOS累计脉冲) */
static int32_t enc_last[4] = {0, 0, 0, 0};
static bool    enc_has_last = false;

/* 每轮命令速度 (回读失败时的fallback) */
static float cmd_wheel_rpm[4] = {0, 0, 0, 0};

/* ================================================================
 *  静态辅助函数
 * ================================================================ */

static float move_abs(float v) { return v >= 0.0f ? v : -v; }

static float move_clamp(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static float move_sqrt(float v) { return sqrtf(v); }
static float move_sin(float deg) { return sinf(deg * 0.01745329f); }
static float move_cos(float deg) { return cosf(deg * 0.01745329f); }
static float move_atan2(float y, float x) {
    return atan2f(y, x) * 57.2957795f;  /* → 度 */
}

/**
 * @brief  FreeRTOS延时, 同时可用于超时计时
 */
static void move_delay(uint32_t ms) {
    osDelay((uint32_t)ms);
}

/**
 * @brief  获取当前tick (ms)
 */
static uint32_t move_tick(void) {
    return xTaskGetTickCount() * portTICK_PERIOD_MS;
}

/* ================================================================
 *  电机驱动层
 * ================================================================ */

/**
 * @brief  单电机速度发送 (signed RPM → dir + abs_vel)
 *
 * 与freertos.c中motor_emit逻辑完全一致。
 * @param idx    轮索引 0=FL, 1=FR, 2=RL, 3=RR
 * @param rpm    有符号RPM, 正=前进方向
 */
static void move_motor_emit(uint8_t idx, float rpm)
{
    int16_t r = (int16_t)rpm;
    uint8_t dir;

    if (wheel_mirror[idx]) {
        /* 右轮镜像: 正RPM → dir=1(CCW) */
        dir = (r >= 0) ? 1 : 0;
    } else {
        /* 左轮正常: 正RPM → dir=0(CW) */
        dir = (r >= 0) ? 0 : 1;
    }

    uint16_t vel = (uint16_t)(r >= 0 ? r : -r);
    if (vel > MOVE_MOTOR_VEL_LIMIT) vel = MOVE_MOTOR_VEL_LIMIT;

    Emm_V5_Vel_Control(wheel_addr[idx], dir, vel, MOVE_ACC_DEFAULT, false);
}

/**
 * @brief  设置单轮速度 (m/s → RPM → CAN)
 * @param idx    轮索引 0~3
 * @param mps    有符号速度 m/s (正=前进)
 */
static void move_set_one_wheel(uint8_t idx, float mps)
{
    float rpm = mps * MOVE_RPM_PER_MPS;
    cmd_wheel_rpm[idx] = rpm;
    move_motor_emit(idx, rpm);
    move_delay(MOVE_CMD_DELAY_MS);
}

/**
 * @brief  设置4轮速度并同步启动
 *
 * Blu3模式: 逐轮snF=true + 广播同步触发。
 * 但当前motor_emit用snF=false逐轮立即启动, 间隔CMD_DELAY。
 * 两种方式都可用; 这里用同步模式以获得更好的多轮一致性。
 *
 * @param w 4轮速度数组 m/s, 正=前进
 */
static void move_set_wheels(const float w[4])
{
    /* 逐轮发送 (snF=false, 立即执行, 与MotorTask一致) */
    for (uint8_t i = 0; i < 4; i++) {
        float rpm = w[i] * MOVE_RPM_PER_MPS;
        cmd_wheel_rpm[i] = rpm;

        int16_t r = (int16_t)rpm;
        uint8_t dir = wheel_mirror[i] ?
                      ((r >= 0) ? 1 : 0) :
                      ((r >= 0) ? 0 : 1);
        uint16_t vel = (uint16_t)(r >= 0 ? r : -r);
        if (vel > MOVE_MOTOR_VEL_LIMIT) vel = MOVE_MOTOR_VEL_LIMIT;

        Emm_V5_Vel_Control(wheel_addr[i], dir, vel, MOVE_ACC_DEFAULT, false);
        move_delay(MOVE_CMD_DELAY_MS);
    }
}

/* ================================================================
 *  编码器回读
 * ================================================================ */

/**
 * @brief  读取单电机S_CPOS (累计位置脉冲)
 * @param  idx  轮索引 0~3
 * @param  out  输出: 有符号累计脉冲
 * @return 1=成功, 0=超时
 */
static uint8_t move_read_encoder(uint8_t idx, int32_t *out)
{
    Emm_V5_Read_Sys_Params(wheel_addr[idx], S_CPOS);

    uint32_t t0 = move_tick();
    while (move_tick() - t0 < MOVE_READ_TIMEOUT_MS) {
        if (can.rxFrameFlag) {
            uint8_t rx_addr = (uint8_t)(can.CAN_RxMsg.ExtId >> 8);
            if (rx_addr == wheel_addr[idx] &&
                can.rxData[0] == 0x36 &&
                can.CAN_RxMsg.DLC == 7) {

                uint32_t pos_u = ((uint32_t)can.rxData[2] << 24) |
                                 ((uint32_t)can.rxData[3] << 16) |
                                 ((uint32_t)can.rxData[4] << 8)  |
                                 ((uint32_t)can.rxData[5]);
                int32_t pos = (int32_t)pos_u;
                if (can.rxData[1]) pos = -pos;   /* sign bit */

                *out = pos;
                can.rxFrameFlag = false;
                return 1;
            }
            can.rxFrameFlag = false;
        }
        move_delay(1);
    }
    return 0;  /* 超时 */
}

/**
 * @brief  读取4轮编码器
 * @param  cur_pos  输出数组[4]
 * @return 成功读取的轮数 (0~4)
 */
static uint8_t move_read_all_encoders(int32_t cur_pos[4])
{
    uint8_t ok = 0;
    for (uint8_t i = 0; i < 4; i++) {
        ok += move_read_encoder(i, &cur_pos[i]);
    }
    return ok;
}

/* ================================================================
 *  里程计积分
 * ================================================================ */

/**
 * @brief  从编码器读数更新里程计 (在控制环内调用)
 *
 * 正运动学 (内部体坐标):
 *   dy_body = (w1+w2+w3+w4) / 4     前进
 *   dx_body = (w1-w2-w3+w4) / 4     右移
 *
 * 场坐标系 (+X=右, +Y=前, CW正, 与Blu3场坐标一致):
 *   move_x +=  dx_body*cos(yaw) + dy_body*sin(yaw)   右方向
 *   move_y += -dx_body*sin(yaw) + dy_body*cos(yaw)   前进方向
 *   move_yaw = -g_imu_yaw                            CW正
 *
 * 注: 内部dx_body/dy_body用Blu3体坐标(right/forward),
 *     X(右)/Y(前)与dx(右)/dy(前)方向一致, 无需轴交换.
 */
static void move_update_odom(const int32_t cur_pos[4])
{
    if (!enc_has_last) {
        for (uint8_t i = 0; i < 4; i++) enc_last[i] = cur_pos[i];
        enc_has_last = true;
        return;
    }

    /* 各轮位移 (米), 右轮镜像取反 */
    float d0 =  (float)(cur_pos[0] - enc_last[0]) * MOVE_ENC_TO_M;  /* FL */
    float d1 = -(float)(cur_pos[1] - enc_last[1]) * MOVE_ENC_TO_M;  /* FR mirror */
    float d2 =  (float)(cur_pos[2] - enc_last[2]) * MOVE_ENC_TO_M;  /* RL */
    float d3 = -(float)(cur_pos[3] - enc_last[3]) * MOVE_ENC_TO_M;  /* RR mirror */

    /* 麦轮正运动学 → 体坐标系位移 */
    float dx_body = (d0 - d1 - d2 + d3) * 0.25f;   /* 右移 */
    float dy_body = (d0 + d1 + d2 + d3) * 0.25f;   /* 前进 */

    /* IMU航向 */
    float yaw_deg = g_imu_yaw;
    float cy = move_cos(yaw_deg);
    float sy = move_sin(yaw_deg);

    /* 体坐标 → 场坐标 (+X=右, +Y=前, CW正)
     * 内部dx_body=右移, dy_body=前进
     * field_X(右) =  dx_body*cos(yaw) + dy_body*sin(yaw)
     * field_Y(前) = -dx_body*sin(yaw) + dy_body*cos(yaw)
     */
    float dX_field = dx_body * cy + dy_body * sy;
    float dY_field = -dx_body * sy + dy_body * cy;

    /* 累加 */
    move_x += dX_field;
    move_y += dY_field;
    move_yaw = -yaw_deg;   /* CW正 = -IMU(CCW正) */

    /* 保存当前读数 */
    for (uint8_t i = 0; i < 4; i++) enc_last[i] = cur_pos[i];
}

/* ================================================================
 *  公共API — 位姿管理
 * ================================================================ */

void Move_InitPose(float x, float y, float yaw_deg)
{
    move_x = x;
    move_y = y;
    move_yaw = -g_imu_yaw;          /* CW正 = -IMU(CCW正) */
    move_target_yaw = move_yaw;

    /* 同步到全局里程计 (CommTask读取, 内部Blu3惯例: dx=右, dy=前, CCW+) */
    __disable_irq();
    g_odom_x = x;                      /* 右 = dx (同向) */
    g_odom_y = y;                      /* 前 = dy (同向) */
    g_odom_theta = g_imu_yaw * 0.01745329f;  /* 内部CCW正弧度 */
    __enable_irq();

    /* 重置编码器基准, 避免下次odom跳变 */
    enc_has_last = false;
}

void Move_ResetPose(void)
{
    Move_InitPose(0.0f, 0.0f, 0.0f);  /* yaw_deg unused, move_yaw = -g_imu_yaw */
}

/* ================================================================
 *  公共API — 速度设置
 * ================================================================ */

/**
 * @brief  体坐标系速度 → 4轮 → CAN
 *
 * 麦轮逆运动学 (Blu3一致):
 *   w1(FL) = vy + vx + wz
 *   w2(FR) = vy - vx - wz
 *   w3(RL) = vy - vx + wz
 *   w4(RR) = vy + vx - wz
 *
 * @param vx  右移速度 m/s
 * @param vy  前进速度 m/s
 * @param wz  旋转分量 m/s (CW正, 正=顺时针旋转)
 */
void Move_SetRobotVelocity(float vx, float vy, float wz)
{
    float w[4];
    w[0] = vy + vx + wz;   /* FL */
    w[1] = vy - vx - wz;   /* FR */
    w[2] = vy - vx + wz;   /* RL */
    w[3] = vy + vx - wz;   /* RR */

    /* 归一化: 任一轮超限则等比缩小 */
    float max_abs = 0.0f;
    for (uint8_t i = 0; i < 4; i++) {
        float a = move_abs(w[i]);
        if (a > max_abs) max_abs = a;
    }
    float limit = MOVE_MAX_SPEED * 2.0f;  /* 留余量 */
    if (max_abs > limit) {
        float scale = limit / max_abs;
        for (uint8_t i = 0; i < 4; i++) w[i] *= scale;
    }

    move_set_wheels(w);
}

/**
 * @brief  场坐标系速度 → 体坐标系 → 4轮 → CAN
 *
 * 外部约定: +X=右, +Y=前, wz>0=CW
 * 运动学层wz也是CW正, 直接传入.

 * @param vx_f  场坐标右移速度 m/s (+X=右)
 * @param vy_f  场坐标前进速度 m/s (+Y=前)
 * @param wz    旋转分量 m/s (CW正)
 */
void Move_SetFieldVelocity(float vx_f, float vy_f, float wz)
{
    /* 外部X(右)/Y(前)直接对应体坐标方向, 仅做场→体旋转 */
    float blu3_vx_f = vx_f;   /* 右 → 右 */
    float blu3_vy_f = vy_f;   /* 前 → 前 */

    /* 旋转: 场→体 (用内部yaw, CCW正; move_yaw是CW正=-g_imu_yaw) */
    float internal_yaw = -move_yaw;   /* = g_imu_yaw, CCW正 */
    float ci = move_cos(internal_yaw);
    float si = move_sin(internal_yaw);
    float vx_body =  blu3_vx_f * ci + blu3_vy_f * si;
    float vy_body = -blu3_vx_f * si + blu3_vy_f * ci;

    Move_SetRobotVelocity(vx_body, vy_body, wz);  /* wz已是CW+, 运动学CW+直接传 */
}

/* ================================================================
 *  公共API — 停止
 * ================================================================ */

void Move_Stop(void)
{
    for (uint8_t i = 0; i < 4; i++) {
        Emm_V5_Stop_Now(wheel_addr[i], true);
        move_delay(MOVE_CMD_DELAY_MS);
    }
    Emm_V5_Synchronous_motion(0x00);

    for (uint8_t i = 0; i < 4; i++) cmd_wheel_rpm[i] = 0.0f;
}

/* ================================================================
 *  同步: move_* → g_odom_* (控制环结束时调用)
 * ================================================================ */
static void move_sync_to_odom(void)
{
    __disable_irq();
    g_odom_x = move_x;                  /* 右 = dx (同向) */
    g_odom_y = move_y;                  /* 前 = dy (同向) */
    g_odom_theta = g_imu_yaw * 0.01745329f;  /* 内部CCW正弧度 */
    __enable_irq();
}

/* ================================================================
 *  控制函数 — MoveTo (点到点直线移动)
 * ================================================================ */

/**
 * @brief  P环位置控制: 移动到目标坐标, 带容差和超时
 *
 * 30ms控制环:
 *   1. 读4轮编码器 → 里程计积分
 *   2. 算距离误差 → P控制算速度
 *   3. 场坐标系分解vx/vy
 *   4. Yaw保持: (target_yaw - imu_yaw) * YAW_KP → wz
 *   5. 发送4轮速度
 *   6. 检查到位/超时
 *
 * @return 1=到位, 0=超时
 */
uint8_t MoveToAccurateTimed(float tx, float ty, float max_speed,
                            float tol, uint32_t timeout_ms)
{
    g_move_active = 1;

    /* 锁定目标航向 (外部CW+ = move_yaw = -g_imu_yaw) */
    move_target_yaw = move_yaw;

    /* 重置编码器基准 */
    enc_has_last = false;

    uint32_t t0 = move_tick();

    for (;;) {
        /* 超时检查 */
        if (move_tick() - t0 >= timeout_ms) {
            Move_Stop();
            move_sync_to_odom();
            g_move_active = 0;
            return 0;
        }

        /* 1. 读编码器 → 里程计 */
        int32_t cur_pos[4];
        move_read_all_encoders(cur_pos);
        move_update_odom(cur_pos);

        /* 2. 距离误差 */
        float dx = tx - move_x;
        float dy = ty - move_y;
        float dist = move_sqrt(dx * dx + dy * dy);

        /* 3. 到位检查 */
        if (dist <= tol) {
            Move_Stop();
            move_sync_to_odom();
            g_move_active = 0;
            return 1;
        }

        /* 4. P控制: 误差 → 速度 */
        float speed = dist * MOVE_POS_KP;
        if (speed < MOVE_MIN_SPEED) speed = MOVE_MIN_SPEED;
        if (speed > max_speed)      speed = max_speed;

        /* 5. 场坐标系分解 (+X=右, +Y=前) */
        float vx_f = (dx / dist) * speed;   /* 右移分量 */
        float vy_f = (dy / dist) * speed;   /* 前进分量 */

        /* 6. Yaw保持: err是CCW+, 取反→CW+给SetFieldVelocity */
        float yaw_err = (-move_target_yaw) - g_imu_yaw;
        float wz = yaw_err * MOVE_YAW_KP;
        wz = move_clamp(wz, -MOVE_YAW_HOLD_LIMIT, MOVE_YAW_HOLD_LIMIT);

        /* 7. 发送速度 (-wz: CCW→CW) */
        Move_SetFieldVelocity(vx_f, vy_f, -wz);

        /* 8. 同步里程计到全局 (CommTask用) */
        move_sync_to_odom();

        /* 9. 控制环延时 */
        move_delay(MOVE_CTRL_PERIOD_MS);
    }
}

uint8_t MoveTo(float tx, float ty, float max_speed)
{
    return MoveToAccurateTimed(tx, ty, max_speed,
                               MOVE_DEFAULT_TOL, MOVE_DEFAULT_TIMEOUT_MS);
}

/* ================================================================
 *  控制函数 — RotateTo (原地旋转)
 * ================================================================ */

/**
 * @brief  原地旋转到目标航向
 *
 * 30ms控制环:
 *   1. 读编码器 → 里程计 (保持位置更新)
 *   2. 算航向误差
 *   3. P控制 → 纯旋转wz
 *   4. 到位/超时检查
 *
 * @return 1=到位, 0=超时
 */
uint8_t RotateToTimed(float target_yaw_deg, float max_speed,
                      uint32_t timeout_ms)
{
    g_move_active = 1;
    move_target_yaw = target_yaw_deg;
    enc_has_last = false;

    uint32_t t0 = move_tick();

    for (;;) {
        if (move_tick() - t0 >= timeout_ms) {
            Move_Stop();
            move_sync_to_odom();
            g_move_active = 0;
            return 0;
        }

        /* 读编码器 → 里程计 */
        int32_t cur_pos[4];
        move_read_all_encoders(cur_pos);
        move_update_odom(cur_pos);

        /* 航向误差: CCW正, 正→左转(CCW), 负→右转(CW)
         * err = imu(CCW+) - target(CCW+) = g_imu_yaw - (-target_yaw_deg) */
        float err = g_imu_yaw - (-target_yaw_deg);
        if (move_abs(err) <= MOVE_YAW_TOL_DEG) {
            Move_Stop();
            move_yaw = -g_imu_yaw;   /* CW正 = -IMU */
            move_sync_to_odom();
            g_move_active = 0;
            return 1;
        }

        /* P控制 → 旋转速度 */
        float wz = err * MOVE_YAW_KP;
        if (wz > 0.0f && wz < MOVE_MIN_SPEED)  wz =  MOVE_MIN_SPEED;
        if (wz < 0.0f && wz > -MOVE_MIN_SPEED) wz = -MOVE_MIN_SPEED;
        wz = move_clamp(wz, -max_speed, max_speed);

        /* 纯旋转: vx=0, vy=0 */
        Move_SetRobotVelocity(0.0f, 0.0f, wz);

        move_sync_to_odom();
        move_delay(MOVE_CTRL_PERIOD_MS);
    }
}

uint8_t RotateTo(float target_yaw_deg, float max_speed)
{
    return RotateToTimed(target_yaw_deg, max_speed, MOVE_YAW_TURN_TIMEOUT);
}

/* ================================================================
 *  控制函数 — MoveToAxisLock (轴锁定移动)
 *
 *  移X时锁Y (主轴X, 副轴Y): 副轴误差超阈值则纠正
 *  移Y时锁X (主轴Y, 副轴X): 同上
 * ================================================================ */

/**
 * @brief  轴锁定移动: 主轴全速, 副轴低速纠正漂移
 *
 * @param tx, ty       目标坐标 m
 * @param main_speed   主轴最大速度 m/s
 * @param lock_speed   副轴纠正速度 m/s (比主轴慢)
 * @param main_tol     主轴到位容差 m
 * @param lock_tol     副轴锁定容差 m (漂移超过此值才纠正)
 * @param axis         MOVE_AXIS_X(移X锁Y) 或 MOVE_AXIS_Y(移Y锁X)
 * @param timeout_ms   超时 ms
 * @return 1=到位, 0=超时
 */
uint8_t MoveToAxisLockTimed(float tx, float ty,
                            float main_speed, float lock_speed,
                            float main_tol, float lock_tol,
                            uint8_t axis, uint32_t timeout_ms)
{
    g_move_active = 1;
    move_target_yaw = move_yaw;   /* 外部CW+ */
    enc_has_last = false;

    /* 副轴锁定值 (运动开始时的副轴坐标) */
    float lock_val = (axis == MOVE_AXIS_X) ? move_y : move_x;

    uint32_t t0 = move_tick();

    for (;;) {
        if (move_tick() - t0 >= timeout_ms) {
            Move_Stop();
            move_sync_to_odom();
            g_move_active = 0;
            return 0;
        }

        /* 读编码器 → 里程计 */
        int32_t cur_pos[4];
        move_read_all_encoders(cur_pos);
        move_update_odom(cur_pos);

        /* 主轴误差 */
        float main_err = (axis == MOVE_AXIS_X) ? (tx - move_x) : (ty - move_y);
        float main_dist = move_abs(main_err);

        /* 主轴到位检查 */
        if (main_dist <= main_tol) {
            Move_Stop();
            move_sync_to_odom();
            g_move_active = 0;
            return 1;
        }

        /* 主轴P控制 */
        float main_spd = main_dist * MOVE_POS_KP;
        if (main_spd < MOVE_MIN_SPEED) main_spd = MOVE_MIN_SPEED;
        if (main_spd > main_speed)     main_spd = main_speed;
        float main_sign = (main_err >= 0.0f) ? 1.0f : -1.0f;

        /* 副轴误差 → 纠正 */
        float lock_err = (axis == MOVE_AXIS_X) ? (lock_val - move_y) : (lock_val - move_x);
        float lock_spd = 0.0f;
        if (move_abs(lock_err) > lock_tol) {
            lock_spd = move_abs(lock_err) * MOVE_POS_KP;
            if (lock_spd < MOVE_MIN_SPEED) lock_spd = MOVE_MIN_SPEED;
            if (lock_spd > lock_speed)     lock_spd = lock_speed;
            lock_spd *= (lock_err >= 0.0f) ? 1.0f : -1.0f;
        }

        /* 组合场坐标系速度 (+X=右, +Y=前) */
        float vx_f, vy_f;
        if (axis == MOVE_AXIS_X) {
            vx_f = main_sign * main_spd;  /* 主轴=X(右) */
            vy_f = lock_spd;               /* 副轴=Y(前)纠正 */
        } else {
            vx_f = lock_spd;               /* 副轴=X(右)纠正 */
            vy_f = main_sign * main_spd;  /* 主轴=Y(前) */
        }

        /* Yaw保持: err是CCW+, 取反→CW+给SetFieldVelocity */
        float yaw_err = (-move_target_yaw) - g_imu_yaw;
        float wz = yaw_err * MOVE_YAW_KP;
        wz = move_clamp(wz, -MOVE_YAW_HOLD_LIMIT, MOVE_YAW_HOLD_LIMIT);

        Move_SetFieldVelocity(vx_f, vy_f, -wz);
        move_sync_to_odom();
        move_delay(MOVE_CTRL_PERIOD_MS);
    }
}

uint8_t MoveToAxisLock(float tx, float ty,
                       float main_speed, float lock_speed,
                       float main_tol, float lock_tol, uint8_t axis)
{
    return MoveToAxisLockTimed(tx, ty, main_speed, lock_speed,
                               main_tol, lock_tol, axis,
                               MOVE_DEFAULT_TIMEOUT_MS);
}

/* ================================================================
 *  控制函数 — MoveArc (圆弧路径移动)
 *
 *  速度 = 切线方向 + 径向修正
 *  航向 = 跟随切线方向
 * ================================================================ */

/**
 * @brief  沿圆弧路径移动
 *
 * 控制环:
 *   1. 计算车相对圆心的当前角度和半径
 *   2. 切线方向速度 (沿弧前进)
 *   3. 径向修正速度 (纠正半径偏差)
 *   4. 航向跟随切线方向
 *   5. 角度完成 + 半径容差 → 结束
 *
 * @param cx, cy          圆心坐标 m
 * @param radius          弧半径 m
 * @param start_angle_deg 起始角 ° (atan2坐标系)
 * @param end_angle_deg   终止角 °
 * @param speed           切线速度 m/s
 * @return 1=完成, 0=超时
 */
uint8_t MoveArc(float cx, float cy, float radius,
                float start_angle_deg, float end_angle_deg,
                float speed)
{
    g_move_active = 1;
    enc_has_last = false;

    /* 外部CW+角度 → 内部CCW+ (与atan2/三角函数一致) */
    start_angle_deg = -start_angle_deg;
    end_angle_deg   = -end_angle_deg;

    /* 判断方向: 从start到end是逆时针还是顺时针 */
    float sweep = end_angle_deg - start_angle_deg;
    /* 归一化到[-180, +180] */
    while (sweep >  180.0f) sweep -= 360.0f;
    while (sweep < -180.0f) sweep += 360.0f;
    float dir = (sweep >= 0.0f) ? 1.0f : -1.0f;  /* +1=CCW, -1=CW */

    /* 累计扫过角度 */
    float total_sweep = move_abs(sweep);
    float swept = 0.0f;
    float prev_angle = start_angle_deg;

    uint32_t t0 = move_tick();

    for (;;) {
        if (move_tick() - t0 >= MOVE_ARC_TIMEOUT_MS) {
            Move_Stop();
            move_sync_to_odom();
            g_move_active = 0;
            return 0;
        }

        /* 读编码器 → 里程计 */
        int32_t cur_pos[4];
        move_read_all_encoders(cur_pos);
        move_update_odom(cur_pos);

        /* 当前相对圆心的极坐标 (X=右, Y=前 → 标准atan2) */
        float dx = move_x - cx;       /* 右差 */
        float dy = move_y - cy;       /* 前差 */
        float cur_r = move_sqrt(dx * dx + dy * dy);
        float cur_angle = move_atan2(dy, dx);  /* 度, CCW+从右 */

        /* 累计扫过角度 (增量法, 避免±180跳变) */
        float delta = cur_angle - prev_angle;
        while (delta >  180.0f) delta -= 360.0f;
        while (delta < -180.0f) delta += 360.0f;
        swept += move_abs(delta);
        prev_angle = cur_angle;

        /* 完成检查: 扫过角度够了 + 半径偏差在容差内 */
        float r_err = cur_r - radius;
        if (swept >= total_sweep && move_abs(r_err) <= MOVE_ARC_TOL) {
            Move_Stop();
            move_sync_to_odom();
            g_move_active = 0;
            return 1;
        }

        /* 切线速度 (垂直于径向, 内部frame: tangent_vx=右, tangent_vy=前) */
        float ca = move_cos(cur_angle);
        float sa = move_sin(cur_angle);
        float tangent_vx = -sa * speed * dir;   /* 切线右分量 */
        float tangent_vy =  ca * speed * dir;   /* 切线前分量 */

        /* 径向修正 (拉回目标半径) */
        float radial_spd = r_err * MOVE_ARC_KP_RADIAL;
        radial_spd = move_clamp(radial_spd, -speed, speed);
        float radial_vx = -ca * radial_spd;     /* 径向右分量 */
        float radial_vy = -sa * radial_spd;     /* 径向前分量 */

        /* 合成场坐标系速度 (+X=右, +Y=前)
         * tangent_vx=右分量, tangent_vy=前分量 → 直接对应 */
        float vx_f = tangent_vx + radial_vx;   /* 右   */
        float vy_f = tangent_vy + radial_vy;   /* 前进 */

        /* 航向跟随切线方向 (外部CW+)
         * 内部atan2(右,前)=CCW+; 取反→CW+ */
        float target_yaw = -move_atan2(tangent_vx, tangent_vy);
        float yaw_err = target_yaw - move_yaw;
        /* 归一化yaw_err到[-180,+180] */
        while (yaw_err >  180.0f) yaw_err -= 360.0f;
        while (yaw_err < -180.0f) yaw_err += 360.0f;
        float wz = yaw_err * MOVE_YAW_KP;
        wz = move_clamp(wz, -MOVE_YAW_HOLD_LIMIT, MOVE_YAW_HOLD_LIMIT);

        /* MoveArc中wz已经是CW+(target_yaw和move_yaw都是CW+),
         * SetFieldVelocity现在直接传CW+给SetRobotVelocity, 无需取反 */
        Move_SetFieldVelocity(vx_f, vy_f, wz);
        move_sync_to_odom();
        move_delay(MOVE_CTRL_PERIOD_MS);
    }
}
