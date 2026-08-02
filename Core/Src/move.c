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

/* ── 编码器诊断 (LCD显示用) ── */
volatile int32_t dbg_enc_raw[4]  = {0, 0, 0, 0};   /* 最近一次原始S_CPOS */
volatile int32_t dbg_enc_delta[4] = {0, 0, 0, 0};   /* 最近一次delta (counts) */
volatile int32_t dbg_enc_ok = 0;                     /* 成功读到的轮数 */
volatile uint8_t dbg_enc_fail = 0;                   /* 最近失败原因: 0=成功 1=超时 2=错地址 3=错功能码 4=错DLC */
volatile uint32_t dbg_enc_bad_delta = 0;             /* 合理性检查失败次数 (delta超限) */
volatile int16_t dbg_cmd_rpm[4] = {0, 0, 0, 0};     /* 最近一次发给各电机的RPM命令 */

/* ================================================================
 *  外部引用 (freertos.c / imu_protocol.c)
 * ================================================================ */
extern volatile float g_imu_yaw;       /* IMU偏航角, 度, 上电归零 */
extern volatile uint32_t g_imu_last_tick; /* IMU最近一次更新的tick */
extern volatile float g_odom_x;        /* 里程计X, 同步用 */
extern volatile float g_odom_y;        /* 里程计Y, 同步用 */
extern volatile float g_odom_theta;    /* 里程计theta, 同步用 */
/* [调试用,定位后删除] PP内部状态遥测, 定义在freertos.c */
extern void SendPathDebugToPC(float mx, float my, int16_t closest, int16_t la,
                              float vx_f, float vy_f, float wz, float target_yaw,
                              uint16_t loop_ms, uint8_t enc_ok);

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

/* ── Pure Pursuit 路径缓冲 (ISR装载, NavTask读取) ──
 * 装载流程: Move_PathBegin(预告count+speed) → 多次Move_PathAddPoint → EXEC触发执行。
 * 执行期间上位机等待RESP, 不会再发POINT帧, 故无并发写读竞争。 */
static PathPt_t g_path_pts[MOVE_PP_MAX_PTS];
static volatile uint8_t g_path_count    = 0;   /* 已装载点数 */
static volatile uint8_t g_path_expected = 0;   /* BEGIN预告的点数 (校验用) */
static volatile float   g_path_speed    = MOVE_PP_SPEED;

/* ================================================================
 *  静态辅助函数
 * ================================================================ */

static float move_abs(float v) { return v >= 0.0f ? v : -v; }

/**
 * @brief  安全的编码器差值 (处理32位回绕)
 *
 * S_CPOS 是 32 位有符号计数器, uint32 减法天然处理回绕:
 *   cur=0xFFFFFFFF(-1), last=100 → uint32差=0xFFFFFF63 → (int32_t)=-157
 * 正常运动每步几千counts, 不会超过int32范围, 所以uint32减法安全.
 */
static int32_t enc_safe_delta(int32_t cur, int32_t last)
{
    return (int32_t)((uint32_t)cur - (uint32_t)last);
}

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
        dbg_cmd_rpm[i] = (int16_t)rpm;   /* 诊断: 记录命令RPM */

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
    can.rxFrameFlag = false;   /* 清除残留帧, 防上轮旧响应被误判为本轮回复 */
    Emm_V5_Read_Sys_Params(wheel_addr[idx], S_CPOS);

    uint8_t fail = 1;  /* default: timeout */
    uint32_t t0 = move_tick();
    while (move_tick() - t0 < MOVE_READ_TIMEOUT_MS) {
        if (can.rxFrameFlag) {
            /* 从ISR快照读取 (避免共享rxData被ISR覆盖导致部分读取) */
            uint8_t rx_addr = (uint8_t)(can.rxSnap.ExtId >> 8);
            uint8_t sd[8];
            for (uint8_t k = 0; k < 8; k++) sd[k] = can.rxSnapData[k];

            if (rx_addr != wheel_addr[idx]) {
                fail = 2;  /* 错误电机地址 */
            } else if (sd[0] != 0x36) {
                fail = 3;  /* 错误功能码 */
            } else if (can.rxSnap.DLC != 7) {
                fail = 4;  /* 错误DLC */
            } else {
                /* 成功 */
                uint32_t pos_u = ((uint32_t)sd[2] << 24) |
                                 ((uint32_t)sd[3] << 16) |
                                 ((uint32_t)sd[4] << 8)  |
                                 ((uint32_t)sd[5]);
                int32_t pos = (int32_t)pos_u;
                if (sd[1]) pos = -pos;   /* sign-magnitude */

                *out = pos;
                can.rxFrameFlag = false;
                dbg_enc_fail = 0;
                return 1;
            }
            can.rxFrameFlag = false;
        }
        move_delay(1);
    }
    dbg_enc_fail = fail;
    return 0;
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
        cur_pos[i] = enc_last[i];   /* 失败时保持上次值, delta=0 (安全默认) */
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
    /* 诊断: 保存原始值 */
    for (uint8_t i = 0; i < 4; i++) dbg_enc_raw[i] = cur_pos[i];

    if (!enc_has_last) {
        for (uint8_t i = 0; i < 4; i++) enc_last[i] = cur_pos[i];
        enc_has_last = true;
        return;
    }

    /* 诊断: 保存delta (counts) */
    for (uint8_t i = 0; i < 4; i++)
        dbg_enc_delta[i] = enc_safe_delta(cur_pos[i], enc_last[i]);

    /* 合理性检查: 正常88RPM×200ms≈191counts, 阈值50000=260倍余量.
     * 超限说明符号位损坏或基线错误 → 跳过积分, 重置基线让下次重建. */
    for (uint8_t i = 0; i < 4; i++) {
        int32_t ad = dbg_enc_delta[i];
        if (ad < 0) ad = -ad;
        if (ad > MOVE_ENC_MAX_DELTA) {
            dbg_enc_bad_delta++;
            enc_has_last = false;   /* 下次调用重建基线, 不用损坏的旧基线 */
            return;
        }
    }

    /* 各轮位移 (米), 右轮镜像取反 */
    float d0 =  (float)dbg_enc_delta[0] * MOVE_ENC_TO_M;  /* FL */
    float d1 = -(float)dbg_enc_delta[1] * MOVE_ENC_TO_M;  /* FR mirror */
    float d2 =  (float)dbg_enc_delta[2] * MOVE_ENC_TO_M;  /* RL */
    float d3 = -(float)dbg_enc_delta[3] * MOVE_ENC_TO_M;  /* RR mirror */

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

    /* PD速度阻尼: 追踪实际接近速度 */
    float prev_dist = -1.0f;       /* -1=首帧无历史 */
    uint32_t prev_tick = t0;
    float v_approach_lp = 0.0f;    /* 低通滤波后的接近速度 m/s */

    for (;;) {
        /* 超时检查 */
        if (move_tick() - t0 >= timeout_ms) {
            Move_Stop();
            move_sync_to_odom();
            g_move_active = 0;
            return 0;
        }

        /* IMU通信看门狗: 200ms无更新→急停 */
        if (move_tick() - g_imu_last_tick > MOVE_IMU_TIMEOUT_MS) {
            Move_Stop();
            move_sync_to_odom();
            g_move_active = 0;
            return 0;
        }

        /* 1. 读编码器 → 里程计 */
        int32_t cur_pos[4];
        dbg_enc_ok = move_read_all_encoders(cur_pos);
        move_update_odom(cur_pos);

        /* 2. 距离误差 */
        float dx = tx - move_x;
        float dy = ty - move_y;
        float dist = move_sqrt(dx * dx + dy * dy);

        /* 2a. 斜走降速: 麦轮45°只有2轮驱动(纯轴4轮), 驱动力能力÷2
         *     diag_scale = max(|dx|,|dy|)/dist: 纯轴=1.0, 45°=0.707
         *     平方后: 纯轴=1.0(满速), 45°=0.50(半速), 匹配驱动轮数比 */
        float abs_dx = move_abs(dx);
        float abs_dy = move_abs(dy);
        float max_comp = (abs_dx > abs_dy) ? abs_dx : abs_dy;
        float diag_scale = (dist > 0.001f) ? (max_comp / dist) : 1.0f;
        if (diag_scale > 1.0f) diag_scale = 1.0f;
        float eff_max_speed = max_speed * diag_scale * diag_scale;
        if (diag_scale < 0.99f) eff_max_speed *= 0.8f;  /* 斜走额外降速防打滑 */

        /* 软启动: 前RAMP_TIME_MS内线性加速 (减少起步打滑) */
        float ramp = (float)(move_tick() - t0) / (float)MOVE_RAMP_TIME_MS;
        if (ramp > 1.0f) ramp = 1.0f;
        eff_max_speed *= ramp;

        /* 2b. 计算实际接近速度 (D项需要) */
        uint32_t now_tick = move_tick();
        float dt_s = (float)(now_tick - prev_tick) * 0.001f;
        if (dt_s < 0.001f) dt_s = 0.001f;

        float v_approach = 0.0f;
        if (prev_dist >= 0.0f) {
            v_approach = (prev_dist - dist) / dt_s;
            v_approach_lp = 0.6f * v_approach_lp + 0.4f * v_approach;
        }
        prev_dist = dist;
        prev_tick = now_tick;

        /* 3. 到位检查 */
        if (dist <= tol) {
            Move_Stop();
            move_sync_to_odom();
            g_move_active = 0;
            return 1;
        }

        /* 4. PD控制: P + 超速阻尼 (D只在电机实际速度>P命令时介入) */
        float p_speed = dist * MOVE_POS_KP;
        float excess = v_approach_lp - p_speed;   /* >0 = 电机比命令快(滞后) */
        float speed = p_speed;
        if (excess > 0.0f) {
            speed -= MOVE_POS_KD * excess;         /* 只削超速部分 */
        }
        if (speed < MOVE_MIN_SPEED) speed = MOVE_MIN_SPEED;
        if (speed > eff_max_speed) speed = eff_max_speed;

        /* 4b. 减速区: sqrt制动曲线 (第一性原理: d=v²/2a → v=√(2ad))
         *    线性减速初始减速度=2×sqrt, 电机跟不上→过冲;
         *    sqrt曲线初始减速度温和, 接近目标时自然降速, 匹配电机物理 */
        if (dist < MOVE_DECEL_DIST) {
            float ratio = dist / MOVE_DECEL_DIST;
            float decel = eff_max_speed * move_sqrt(ratio);
            if (decel < MOVE_MIN_SPEED) decel = MOVE_MIN_SPEED;
            if (speed > decel) speed = decel;
        }

        /* 4c. 蠕变区: 线性渐变限速 (边界=CREEP_SPEED, 目标=MIN_SPEED)
         *    消除硬限速台阶跳变, 电机无需瞬间大幅减速→无残余震荡 */
        if (dist < MOVE_CREEP_DIST) {
            float ratio = dist / MOVE_CREEP_DIST;   /* 1(边界) → 0(目标) */
            float creep_cap = MOVE_MIN_SPEED + (MOVE_CREEP_SPEED - MOVE_MIN_SPEED) * ratio;
            if (speed > creep_cap) speed = creep_cap;
        }

        /* 5. 场坐标系分解 (+X=右, +Y=前) */
        float vx_f = (dx / dist) * speed;   /* 右移分量 */
        float vy_f = (dy / dist) * speed;   /* 前进分量 */

        /* 6. Yaw保持: err是CCW+, 取反→CW+给SetFieldVelocity
         *    死区内不修正, 防止IMU噪声导致直线摇摆 */
        float yaw_err = (-move_target_yaw) - g_imu_yaw;
        while (yaw_err >  180.0f) yaw_err -= 360.0f;
        while (yaw_err < -180.0f) yaw_err += 360.0f;
        float wz = 0.0f;
        if (move_abs(yaw_err) > MOVE_YAW_HOLD_DEADZONE) {
            wz = yaw_err * MOVE_YAW_KP;
            wz = move_clamp(wz, -MOVE_YAW_HOLD_LIMIT, MOVE_YAW_HOLD_LIMIT);
        }

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

    /* 预测性停止: 追踪IMU角速度, 提前补偿30ms stop延迟 */
    float prev_imu_yaw = g_imu_yaw;
    uint32_t prev_imu_tick = t0;
    float yaw_rate_lp = 0.0f;   /* 滤波后的角速度 °/s */

    for (;;) {
        if (move_tick() - t0 >= timeout_ms) {
            Move_Stop();
            move_sync_to_odom();
            g_move_active = 0;
            return 0;
        }

        /* IMU通信看门狗: 200ms无更新→急停 */
        if (move_tick() - g_imu_last_tick > MOVE_IMU_TIMEOUT_MS) {
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
        /* 归一化到[-180,180]: 始终走最短路径 */
        while (err >  180.0f) err -= 360.0f;
        while (err < -180.0f) err += 360.0f;

        /* 计算角速度 (°/s), 用于预测性停止 */
        uint32_t now_imu_tick = move_tick();
        float dt_imu = (now_imu_tick - prev_imu_tick) * 0.001f;
        if (dt_imu < 0.001f) dt_imu = 0.001f;
        float dyaw = g_imu_yaw - prev_imu_yaw;
        /* 归一化dyaw避免360°跳变 */
        while (dyaw >  180.0f) dyaw -= 360.0f;
        while (dyaw < -180.0f) dyaw += 360.0f;
        float yaw_rate = dyaw / dt_imu;
        /* 低通滤波: 抑制IMU噪声 */
        yaw_rate_lp = 0.7f * yaw_rate_lp + 0.3f * yaw_rate;
        prev_imu_yaw = g_imu_yaw;
        prev_imu_tick = now_imu_tick;

        /* 预测性到位判定: 补偿30ms stop延迟 (CAN 20ms + 电机 10ms)
         * stop_distance = |角速度| × 0.03s (度)
         * 当 |err| <= 容差 + stop_distance 时提前stop */
        float stop_distance = move_abs(yaw_rate_lp) * 0.03f;
        if (move_abs(err) <= MOVE_YAW_TOL_DEG + stop_distance) {
            Move_Stop();
            move_yaw = -g_imu_yaw;   /* CW正 = -IMU */
            move_sync_to_odom();
            g_move_active = 0;
            return 1;
        }

        /* P控制 → 旋转速度 */
        float wz = err * MOVE_YAW_KP;
        /* 近距离(<5°)用更低最低速度, 减少滑行过冲 */
        float min_spd = (move_abs(err) < 5.0f) ? MOVE_YAW_FINE_SPEED
                                                : MOVE_MIN_SPEED;
        if (wz > 0.0f && wz < min_spd)  wz =  min_spd;
        if (wz < 0.0f && wz > -min_spd) wz = -min_spd;
        wz = move_clamp(wz, -max_speed, max_speed);

        /* 减速区: sqrt制动曲线 (第一性原理: θ=ω²/2α → ω=√(2αθ)) */
        {
            float abs_err = move_abs(err);
            if (abs_err < MOVE_YAW_DECEL_DEG) {
                float ratio = abs_err / MOVE_YAW_DECEL_DEG;
                float decel = max_speed * move_sqrt(ratio);
                if (decel < MOVE_MIN_SPEED) decel = MOVE_MIN_SPEED;
                if (move_abs(wz) > decel)
                    wz = (wz >= 0.0f) ? decel : -decel;
            }
        }

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

    /* PD速度阻尼 */
    float prev_main_dist = -1.0f;
    uint32_t prev_tick_ax = t0;
    float v_approach_lp_ax = 0.0f;

    for (;;) {
        if (move_tick() - t0 >= timeout_ms) {
            Move_Stop();
            move_sync_to_odom();
            g_move_active = 0;
            return 0;
        }

        /* IMU通信看门狗: 200ms无更新→急停 */
        if (move_tick() - g_imu_last_tick > MOVE_IMU_TIMEOUT_MS) {
            Move_Stop();
            move_sync_to_odom();
            g_move_active = 0;
            return 0;
        }

        /* 读编码器 → 里程计 */
        int32_t cur_pos[4];
        dbg_enc_ok = move_read_all_encoders(cur_pos);
        move_update_odom(cur_pos);

        /* 主轴误差 */
        float main_err = (axis == MOVE_AXIS_X) ? (tx - move_x) : (ty - move_y);
        float main_dist = move_abs(main_err);

        /* 计算实际接近速度 (预测制动需要) */
        uint32_t now_tick_ax = move_tick();
        float dt_ax = (float)(now_tick_ax - prev_tick_ax) * 0.001f;
        if (dt_ax < 0.001f) dt_ax = 0.001f;

        float v_app_ax = 0.0f;
        if (prev_main_dist >= 0.0f) {
            v_app_ax = (prev_main_dist - main_dist) / dt_ax;
            v_approach_lp_ax = 0.6f * v_approach_lp_ax + 0.4f * v_app_ax;
        }
        prev_main_dist = main_dist;
        prev_tick_ax = now_tick_ax;

        /* 到位检查 */
        if (main_dist <= main_tol) {
            Move_Stop();
            move_sync_to_odom();
            g_move_active = 0;
            return 1;
        }

        /* 软启动: 前RAMP_TIME_MS内线性加速 */
        float ramp_ax = (float)(move_tick() - t0) / (float)MOVE_RAMP_TIME_MS;
        if (ramp_ax > 1.0f) ramp_ax = 1.0f;
        float ramped_main_speed = main_speed * ramp_ax;

        /* 主轴PD控制: P + 超速阻尼 */
        float p_spd_ax = main_dist * MOVE_POS_KP;
        float excess_ax = v_approach_lp_ax - p_spd_ax;
        float main_spd = p_spd_ax;
        if (excess_ax > 0.0f) {
            main_spd -= MOVE_POS_KD * excess_ax;
        }
        if (main_spd < MOVE_MIN_SPEED) main_spd = MOVE_MIN_SPEED;
        if (main_spd > ramped_main_speed) main_spd = ramped_main_speed;

        /* 减速区: sqrt制动曲线 (第一性原理: d=v²/2a → v=√(2ad)) */
        if (main_dist < MOVE_DECEL_DIST) {
            float ratio = main_dist / MOVE_DECEL_DIST;
            float decel = ramped_main_speed * move_sqrt(ratio);
            if (decel < MOVE_MIN_SPEED) decel = MOVE_MIN_SPEED;
            if (main_spd > decel) main_spd = decel;
        }

        /* 蠕变区: 线性渐变限速 (同MoveToAccurateTimed) */
        if (main_dist < MOVE_CREEP_DIST) {
            float ratio = main_dist / MOVE_CREEP_DIST;
            float creep_cap = MOVE_MIN_SPEED + (MOVE_CREEP_SPEED - MOVE_MIN_SPEED) * ratio;
            if (main_spd > creep_cap) main_spd = creep_cap;
        }

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

        /* Yaw保持: err是CCW+, 取反→CW+给SetFieldVelocity
         *    死区内不修正, 防止IMU噪声导致直线摇摆 */
        float yaw_err = (-move_target_yaw) - g_imu_yaw;
        while (yaw_err >  180.0f) yaw_err -= 360.0f;
        while (yaw_err < -180.0f) yaw_err += 360.0f;
        float wz = 0.0f;
        if (move_abs(yaw_err) > MOVE_YAW_HOLD_DEADZONE) {
            wz = yaw_err * MOVE_YAW_KP;
            wz = move_clamp(wz, -MOVE_YAW_HOLD_LIMIT, MOVE_YAW_HOLD_LIMIT);
        }

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

/* ================================================================
 *  控制函数 — MovePathPurePursuit (Pure Pursuit 路径跟踪)
 *
 *  上位机预插值密集点 → 装载 → MCU端前视点自主跟踪。
 *  平移通道: 速度向量指向前视点 (全向平移, 无需转向角)。
 *  航向通道: 在约束点(heading!=999)之间按弧长线性插值, 独立控制。
 *  点之间不停车 → 前视点沿路径连续滑动 → 速度连续 → 平滑。
 * ================================================================ */

/* 路径缓冲装载 (ISR上下文调用, 仅写全局, 无FreeRTOS调用) */
void Move_PathBegin(uint8_t count, float speed)
{
    g_path_count    = 0;
    g_path_expected = count;
    g_path_speed    = speed;
}

void Move_PathAddPoint(float x, float y, float heading)
{
    if (g_path_count < MOVE_PP_MAX_PTS) {
        g_path_pts[g_path_count].x       = x;
        g_path_pts[g_path_count].y       = y;
        g_path_pts[g_path_count].heading = heading;
        g_path_count++;
    }
}

/* ── Pure Pursuit 静态辅助 ── */

/* 自由航向判定: heading≈999 表示该点不约束航向 */
static int pp_is_free(float h)
{
    return move_abs(h - MOVE_PP_FREE_HEADING) < 1.0f;
}

/* 累计弧长: 路径 pts[0]→pts[k] 的折线长度 m */
static float pp_arc_to(const PathPt_t *pts, int k)
{
    float s = 0.0f;
    for (int i = 0; i < k; i++) {
        float dx = pts[i + 1].x - pts[i].x;
        float dy = pts[i + 1].y - pts[i].y;
        s += move_sqrt(dx * dx + dy * dy);
    }
    return s;
}

/* 角度归一化到 [-180, +180] */
static float pp_norm180(float a)
{
    while (a >  180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

/* 最短路径角度插值: 从a到b按比例t∈[0,1] (处理±180跨界) */
static float pp_interp_angle(float a, float b, float t)
{
    float d = pp_norm180(b - a);
    return pp_norm180(a + d * t);
}

/**
 * @brief  Pure Pursuit 路径跟踪 (阻塞式, NavTask调用)
 *
 * 控制环 (~33ms):
 *   1. 读4轮编码器 → 里程计积分
 *   2. 单调推进 closest = 距车最近的路径点索引
 *   3. 到位判定: closest到末点 且 距末点<=容差 → 完成
 *   4. 前视点 la = 从closest向前首个距车>=lookahead的点 (末端饱和到末点)
 *   5. 剩余弧长 → sqrt末端减速 + 软启动
 *   6. 平移速度 = 指向前视点的单位向量 × 速度 (场坐标 +X右 +Y前)
 *   7. 航向目标 = 约束点间按弧长线性插值
 *   8. 航向P控制 → wz (CW+)
 *   9. Move_SetFieldVelocity 发送
 *
 * @return 1=完成, 0=超时/IMU失联/装载无效
 */
uint8_t MovePathPurePursuit(void)
{
    int   n     = (int)g_path_count;
    float speed = g_path_speed;

    /* 装载校验: 至少2点, 且实收==预告 */
    if (n < 2 || n != (int)g_path_expected) {
        return 0;
    }

    g_move_active = 1;
    move_target_yaw = move_yaw;
    enc_has_last = false;

    const PathPt_t *pts = g_path_pts;
    int closest = 0;                 /* 最近点索引 (单调递增) */

    uint32_t t0 = move_tick();
    uint32_t prev_iter_tick = t0;      /* [调试] 测真实控制环周期 */

    for (;;) {
        /* 超时 */
        if (move_tick() - t0 >= MOVE_PP_TIMEOUT_MS) {
            Move_Stop(); move_sync_to_odom(); g_move_active = 0;
            return 0;
        }
        /* IMU通信看门狗 */
        if (move_tick() - g_imu_last_tick > MOVE_IMU_TIMEOUT_MS) {
            Move_Stop(); move_sync_to_odom(); g_move_active = 0;
            return 0;
        }

        /* 1. 读编码器 → 里程计 */
        int32_t cur_pos[4];
        dbg_enc_ok = move_read_all_encoders(cur_pos);
        move_update_odom(cur_pos);

        float rx = move_x, ry = move_y;

        /* 2. 推进closest到距车最近的点 (只前进, 防U形路径回跳) */
        while (closest < n - 1) {
            float dcx = pts[closest].x     - rx, dcy = pts[closest].y     - ry;
            float dnx = pts[closest + 1].x - rx, dny = pts[closest + 1].y - ry;
            if (dnx * dnx + dny * dny <= dcx * dcx + dcy * dcy) closest++;
            else break;
        }

        /* 3. 终点到位: 最近点已是末点 且 距末点容差内 */
        float ex = pts[n - 1].x - rx, ey = pts[n - 1].y - ry;
        float end_dist = move_sqrt(ex * ex + ey * ey);
        if (closest >= n - 1 && end_dist <= MOVE_PP_TOL) {
            Move_Stop(); move_sync_to_odom(); g_move_active = 0;
            return 1;
        }

        /* 4. 先算速度 (软启动 + 末端sqrt减速): L依赖当前速度 */
        float rem = pp_arc_to(pts, n - 1) - pp_arc_to(pts, closest);
        float v_cmd = speed;
        if (rem < MOVE_DECEL_DIST) {
            float ratio = rem / MOVE_DECEL_DIST;
            float decel = speed * move_sqrt(ratio);
            if (decel < MOVE_MIN_SPEED) decel = MOVE_MIN_SPEED;
            if (v_cmd > decel) v_cmd = decel;
        }
        float ramp = (float)(move_tick() - t0) / (float)MOVE_RAMP_TIME_MS;
        if (ramp > 1.0f) ramp = 1.0f;
        v_cmd *= ramp;

        /* 5. 速度自适应前视距离 L = K×V + L_min, 封顶L_max */
        float lookahead = MOVE_PP_LA_K * v_cmd + MOVE_PP_LA_MIN;
        if (lookahead > MOVE_PP_LA_MAX) lookahead = MOVE_PP_LA_MAX;

        /* 5b. 前视点: 从closest向前找首个距车>=lookahead的点 */
        int la = closest;
        while (la < n - 1) {
            float lx = pts[la].x - rx, ly = pts[la].y - ry;
            if (move_sqrt(lx * lx + ly * ly) >= lookahead) break;
            la++;
        }
        float tx = pts[la].x, ty = pts[la].y;

        /* 6. 平移速度: 指向前视点 (场坐标 +X右 +Y前) */
        float dx = tx - rx, dy = ty - ry;
        float d  = move_sqrt(dx * dx + dy * dy);
        float vx_f = 0.0f, vy_f = 0.0f;
        if (d > 0.001f) {
            vx_f = (dx / d) * v_cmd;
            vy_f = (dy / d) * v_cmd;
        }

        /* 7. 航向目标: 约束点间按弧长线性插值
         *    pc=closest之前(含)最近约束点, nc=closest之后最近约束点 */
        float target_yaw = move_yaw;     /* 默认保持当前航向 */
        int pc = -1, nc = -1;
        for (int k = closest; k >= 0; k--)     { if (!pp_is_free(pts[k].heading)) { pc = k; break; } }
        for (int k = closest + 1; k < n; k++)  { if (!pp_is_free(pts[k].heading)) { nc = k; break; } }
        if (pc >= 0 && nc >= 0) {
            float sp = pp_arc_to(pts, pc);
            float sn = pp_arc_to(pts, nc);
            float sc = pp_arc_to(pts, closest);
            float frac = (sn > sp) ? (sc - sp) / (sn - sp) : 0.0f;
            if (frac < 0.0f) frac = 0.0f;
            if (frac > 1.0f) frac = 1.0f;
            target_yaw = pp_interp_angle(pts[pc].heading, pts[nc].heading, frac);
        } else if (pc >= 0) {
            target_yaw = pts[pc].heading;    /* 末约束点之后: 保持末约束航向 */
        } else if (nc >= 0) {
            target_yaw = pts[nc].heading;    /* 首约束点之前: 提前对准首约束航向 */
        }

        /* 8. 航向P控制 (CW+, 与MoveArc一致直接传SetFieldVelocity) */
        float yaw_err = pp_norm180(target_yaw - move_yaw);
        float wz = 0.0f;
        if (move_abs(yaw_err) > MOVE_YAW_HOLD_DEADZONE) {
            wz = yaw_err * MOVE_YAW_KP;
            wz = move_clamp(wz, -MOVE_PP_YAW_LIMIT, MOVE_PP_YAW_LIMIT);
        }

        /* 9. 发送速度 + 同步里程计 + 控制环延时 */
        /* [调试用,定位后删除] 遥测内部状态, 区分位置冻结vs电机不听 */
        uint32_t now_iter = move_tick();
        uint16_t loop_ms = (uint16_t)(now_iter - prev_iter_tick);
        prev_iter_tick = now_iter;
        SendPathDebugToPC(move_x, move_y, (int16_t)closest, (int16_t)la,
                          vx_f, vy_f, wz, target_yaw, loop_ms, dbg_enc_ok);
        Move_SetFieldVelocity(vx_f, vy_f, wz);
        move_sync_to_odom();
        move_delay(MOVE_CTRL_PERIOD_MS);
    }
}
