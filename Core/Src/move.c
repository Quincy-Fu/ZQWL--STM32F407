/**
 * @file    move.c
 * @brief   底层运动控制模块 — 移植自Blu3 Move层
 *
 * 30ms控制环: P环位置控制 + 编码器回读XY/航向里程计.
 * 阻塞式API, 在NavTask中调用.
 * 注: 全程使用编码器yaw; IMU仅保留诊断/校准, 不参与运动角度闭环.
 *
 * 坐标系: vy>0=前进, vx>0=右移, wz>0=CW(顺时针)
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
volatile uint16_t dbg_loop_ms = 0;                     /* 控制环实际周期 ms (Watch看这个, 正常~28ms) */
volatile uint8_t g_path_debug_en = 1;                  /* 调试帧开关: 1=开(默认) 0=关(省3ms/循环, Watch设0) */

/* ================================================================
 *  外部引用 (freertos.c / imu_protocol.c)
 * ================================================================ */
extern volatile float g_odom_x;        /* 里程计X, 同步用 */
extern volatile float g_odom_y;        /* 里程计Y, 同步用 */
extern volatile float g_odom_theta;    /* 里程计theta(CW+弧度), 同步用 */
/* [调试用,定位后删除] 路径跟踪遥测, 定义在freertos.c */
extern void SendPathDebugToPC(float mx, float my, int16_t wp_idx, int16_t total,
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
static uint8_t arc_yaw_l_sum_active = 0; /* 1=圆弧段使用MOVE_ARC_YAW_L_SUM, 0=普通段使用MOVE_YAW_L_SUM */

/* 每轮命令速度 (回读失败时的fallback) */
static float cmd_wheel_rpm[4] = {0, 0, 0, 0};

/* ── 路径点跟踪缓冲 (ISR装载, NavTask读取) ──
 * 装载流程: Move_PathBegin(预告count+speed) → 多次Move_PathAddPoint → EXEC触发执行。
 * 执行期间上位机等待RESP, 不会再发POINT帧, 故无并发写读竞争。 */
static PathPt_t g_path_pts[MOVE_WP_MAX_PTS];
static volatile uint8_t g_path_count    = 0;   /* 已装载点数 */
static volatile uint8_t g_path_expected = 0;   /* BEGIN预告的点数 (校验用) */
static volatile float   g_path_speed    = MOVE_WP_SPEED;
static volatile uint8_t g_path_load_error = 0; /* 装载期间发现非法点/超长/模式错误 */

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
 *   move_yaw += ((d0-d1+d2-d3)/4)/YAW_L_SUM       CW正(全程编码器)
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

    /* 麦轮正运动学 → 体坐标系位移/航向增量 */
    float dx_body = (d0 - d1 - d2 + d3) * 0.25f;   /* 右移 */
    float dy_body = (d0 + d1 + d2 + d3) * 0.25f;   /* 前进 */
    float yaw_l_sum = arc_yaw_l_sum_active ? MOVE_ARC_YAW_L_SUM : MOVE_YAW_L_SUM;
    float dtheta_cw = ((d0 - d1 + d2 - d3) * 0.25f) / yaw_l_sum * 57.2957795f;

    move_yaw += dtheta_cw;

    /* 场坐标变换: 数学旋转仍用CCW正, 外部move_yaw保持CW正。 */
    float yaw_deg = -move_yaw;   /* 外部CW正 → 内部CCW正 */
    float cy = move_cos(yaw_deg);
    float sy = move_sin(yaw_deg);

    /* 体坐标 → 场坐标 (+X=右, +Y=前, CW正)
     * 这是 Move_SetFieldVelocity 场→体变换的逆矩阵:
     *   场→体: vx_body =  vx*cos + vy*sin;  vy_body = -vx*sin + vy*cos
     *   体→场: dX = dx*cos - dy*sin;        dY =  dx*sin + dy*cos  ← sin 取反
     * 旧代码误用场→体公式做体→场, heading≠0时左右方向搞反→圆弧跑偏 */
    float dX_field = dx_body * cy - dy_body * sy;
    float dY_field = dx_body * sy + dy_body * cy;

    /* 累加 */
    move_x += dX_field;
    move_y += dY_field;

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
    move_yaw = yaw_deg;               /* CW正, 编码器yaw积分基准 */
    move_target_yaw = move_yaw;

    /* 同步到全局里程计 (CommTask读取上报, 约定: dx=右, dy=前, theta=CW+弧度) */
    __disable_irq();
    g_odom_x = x;                      /* 右 = dx (同向) */
    g_odom_y = y;                      /* 前 = dy (同向) */
    g_odom_theta = move_yaw * 0.01745329f;  /* CW+弧度, 与 move_yaw 同源同约定 */
    __enable_irq();

    /* 重置编码器基准, 避免下次odom跳变 */
    enc_has_last = false;
}

void Move_ResetPose(void)
{
    Move_InitPose(0.0f, 0.0f, 0.0f);  /* 当前编码器航向基准归零 */
}

float Move_GetYaw(void)
{
    return move_yaw;
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

    /* 旋转: 场→体。数学旋转用CCW正, move_yaw对外保持CW正。 */
    float internal_yaw = -move_yaw;   /* CCW正 */
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
    g_odom_theta = move_yaw * 0.01745329f;  /* CW+弧度, 与 move_yaw 同源同约定 */
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
 *   4. Yaw保持: 编码器yaw误差 * YAW_KP → wz
 *   5. 发送4轮速度
 *   6. 检查到位/超时
 *
 * @return 1=到位, 0=超时
 */
uint8_t MoveToAccurateTimed(float tx, float ty, float max_speed,
                            float tol, uint32_t timeout_ms)
{
    g_move_active = 1;

    /* 锁定目标航向 (外部CW+ = move_yaw, 普通段编码器来源) */
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

        /* 6. Yaw保持: current-target为正表示已偏CW, 需要输出CCW修正。
         *    死区内不修正, 防编码器量化/机械微抖导致直线摇摆 */
        float yaw_err = move_yaw - move_target_yaw;  /* CW+: current-target → CCW+误差 */
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

    uint8_t settle_cnt = 0;       /* settle计数: 停稳等待后连续控制周期在容差内才算到位 */
    uint8_t stopped = 0;          /* 当前是否已停电机并等待滑行稳定 */
    uint8_t had_stopped = 0;      /* 本次RotateTo是否已经触发过预测停止 */
    uint32_t stopped_tick = t0;   /* 最近一次停电机时刻 */
    uint32_t settle_last_tick = t0; /* 上次计入settle的控制周期 */

    /* 预检查: 若起始误差已在ACCEPT内 (如重复发送同一角度命令),
     * 先停电机并等待机械滑行稳定, 避免无意义微动。 */
    {
        float init_err = target_yaw_deg - move_yaw;
        while (init_err >  180.0f) init_err -= 360.0f;
        while (init_err < -180.0f) init_err += 360.0f;
        if (move_abs(init_err) <= MOVE_YAW_ACCEPT_DEG) {
            Move_Stop();
            stopped = 1;
            had_stopped = 1;
            stopped_tick = move_tick();
            settle_last_tick = stopped_tick;
        }
    }

    /* 追踪编码器角速度 (用于预测性停止) */
    float prev_yaw_enc = move_yaw;   /* 上一帧编码器yaw (CW正) */
    uint32_t prev_yaw_tick = t0;
    float yaw_rate_lp = 0.0f;   /* 滤波后的角速度 °/s */

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

        /* 航向误差 (CW正, 正→右转CW)
         * err = target(CW+) - current(CW+) = target_yaw_deg - move_yaw */
        float err = target_yaw_deg - move_yaw;
        /* 归一化到[-180,180]: 始终走最短路径 */
        while (err >  180.0f) err -= 360.0f;
        while (err < -180.0f) err += 360.0f;

        /* 计算编码器角速度 (°/s), 用于预测性停止 */
        uint32_t now_yaw_tick = move_tick();
        float dt_yaw = (now_yaw_tick - prev_yaw_tick) * 0.001f;
        if (dt_yaw < 0.001f) dt_yaw = 0.001f;
        float dyaw = move_yaw - prev_yaw_enc;
        /* 归一化dyaw避免360°跳变 */
        while (dyaw >  180.0f) dyaw -= 360.0f;
        while (dyaw < -180.0f) dyaw += 360.0f;
        float yaw_rate = dyaw / dt_yaw;
        /* 低通滤波: 抑制编码器读数抖动 */
        yaw_rate_lp = 0.7f * yaw_rate_lp + 0.3f * yaw_rate;
        prev_yaw_enc = move_yaw;
        prev_yaw_tick = now_yaw_tick;

        /* settle到位:
         * 1) 预测性停止只负责提前松手, 不负责立即返回完成;
         * 2) 首停后必须等待机械滑行;
         * 3) 若停稳后误差仍超过ACCEPT, 继续P控制拉回, 不让上位机再发第二次命令。 */
        float abs_err = move_abs(err);
        float predicted_overshoot = move_abs(yaw_rate_lp) * ((float)MOVE_YAW_STOP_LATENCY_MS / 1000.0f);
        uint8_t should_stop = (abs_err <= MOVE_YAW_TOL_DEG + predicted_overshoot) ? 1u : 0u;
        uint8_t acceptable_after_stop = (had_stopped && abs_err <= MOVE_YAW_ACCEPT_DEG) ? 1u : 0u;

        if (should_stop) {
            /* 容差内(或预测将进入): 先松手, 等滑行结束后再判定 */
            if (!stopped) {
                Move_Stop();
                stopped = 1;
                had_stopped = 1;
                stopped_tick = move_tick();
                settle_last_tick = stopped_tick;
                settle_cnt = 0;
            }
        }

        if (stopped) {
            uint8_t wait_ok = ((move_tick() - stopped_tick) >= MOVE_YAW_SETTLE_WAIT_MS) ? 1u : 0u;

            if (!should_stop && !acceptable_after_stop) {
                /* 滑行后偏差仍大: 重新进入P控制, 本次命令内自行拉回。 */
                stopped = 0;
                settle_cnt = 0;
            } else if (wait_ok && ((move_tick() - settle_last_tick) >= MOVE_CTRL_PERIOD_MS)) {
                settle_last_tick = move_tick();
                if ((abs_err <= MOVE_YAW_TOL_DEG) || acceptable_after_stop) {
                    settle_cnt++;
                    if (settle_cnt >= MOVE_YAW_SETTLE_FRAMES) {
                        move_sync_to_odom();
                        g_move_active = 0;
                        return 1;
                    }
                } else {
                    settle_cnt = 0;
                }
            }
        }

        /* P控制 → 旋转速度 (仅在未处于停稳等待时执行) */
        if (!stopped) {
            float wz = err * MOVE_YAW_KP;
            /* 近距离(<5°)用更低最低速度, 减少滑行过冲 */
            float min_spd = (abs_err < 5.0f) ? MOVE_YAW_FINE_SPEED
                                                    : MOVE_MIN_SPEED;
            if (wz > 0.0f && wz < min_spd)  wz =  min_spd;
            if (wz < 0.0f && wz > -min_spd) wz = -min_spd;

            /* 触发停止后: 大误差用正常速度快速拉回, 只剩小误差才用 FINE_SPEED 细调 */
            float effective_max = (had_stopped && abs_err < 10.0f) ? MOVE_YAW_FINE_SPEED : max_speed;
            wz = move_clamp(wz, -effective_max, effective_max);

            /* 减速区: sqrt制动曲线 (第一性原理: θ=ω²/2α → ω=√(2αθ)) */
            {
                if (abs_err < MOVE_YAW_DECEL_DEG) {
                    float ratio = abs_err / MOVE_YAW_DECEL_DEG;
                    float decel = effective_max * move_sqrt(ratio);
                    if (decel < MOVE_MIN_SPEED) decel = MOVE_MIN_SPEED;
                    if (move_abs(wz) > decel)
                        wz = (wz >= 0.0f) ? decel : -decel;
                }
            }

            /* 纯旋转: vx=0, vy=0 */
            Move_SetRobotVelocity(0.0f, 0.0f, wz);
        }

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
    uint8_t main_arrived = 0;           /* 主轴已到位(副轴纠正中), 主轴停转 */

    for (;;) {
        if (move_tick() - t0 >= timeout_ms) {
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

        /* 到位检查: 主轴+副轴都到位才完成
         * 主轴到位但副轴没到 → 标记main_arrived, 主轴停转, 副轴继续纠正
         * (边走边调到位, 不需要到位后补偿轮) */
        float lock_err_chk = (axis == MOVE_AXIS_X) ? (lock_val - move_y) : (lock_val - move_x);
        if (main_dist <= main_tol) {
            if (move_abs(lock_err_chk) <= lock_tol) {
                /* 两轴都到位 → 完成 */
                Move_Stop();
                move_sync_to_odom();
                g_move_active = 0;
                return 1;
            }
            main_arrived = 1;  /* 主轴到位, 副轴还在纠正中 */
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

        /* 主轴已到位: 停主轴, 只修副轴 (防主轴越过目标) */
        if (main_arrived) main_spd = 0.0f;

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

        /* Yaw保持: current-target为正表示已偏CW, 需要输出CCW修正。
         * 死区内不修正, 防IMU噪声导致直线摇摆。 */
        float yaw_err = move_yaw - move_target_yaw;  /* CW+: current-target → CCW+误差 */
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
 *  控制函数 — MoveArcTrack (圆弧轨迹跟踪)
 *
 *  弧长参数化: θ=s/R, 目标姿态=start_yaw+dir×θ
 *  前馈: wz_ff=(v/R)×ARC_YAW_L_SUM → 消除曲线上稳态滞后(无需积分)
 *  反馈: wz_p=KP×(yaw_err) → 修正 transient 误差
 *  径向P修正保持机器人在弧线上
 *  位置算θ(非时间), 速度波动不影响角度精度
 * ================================================================ */

uint8_t MoveArcTrack(float radius, float speed, int dir,
                     float sweep_deg, uint32_t timeout_ms)
{
    if (radius < 0.01f || speed < 0.01f || dir == 0) return 0;
    dir = (dir > 0) ? 1 : -1;
    if (sweep_deg < 0.0f) sweep_deg = -sweep_deg;

    g_move_active = 1;
    arc_yaw_l_sum_active = 1;
    enc_has_last = false;

    float start_yaw = move_yaw;             /* CW正, 圆弧起点使用编码器yaw */
    float cx = move_x + (float)dir * radius * move_cos(start_yaw);
    float cy = move_y - (float)dir * radius * move_sin(start_yaw);

    /* 初始极角 (标准 atan2, CCW+ 从+X) */
    float dx0 = move_x - cx, dy0 = move_y - cy;
    float prev_angle = move_atan2(dy0, dx0);

    float swept = 0.0f;       /* 编码器位置估算 (切线/径向/航向目标用) */
    float swept_yaw = 0.0f;   /* 编码器航向估算 (减速/停止用) */
    float prev_yaw = start_yaw;  /* CW+ 累积用 (编码器yaw) */
    float yaw_rate_lp = 0.0f;  /* 编码器角速度 °/s (给停止预测用) */
    uint32_t t0 = move_tick();
    uint32_t prev_yaw_tick = t0;
    uint32_t prev_iter_tick = t0;

    /* 弧线终点目标航向 (CW+), settle阶段用 */
    float final_target_yaw = start_yaw + (float)dir * sweep_deg;
    while (final_target_yaw >  180.0f) final_target_yaw -= 360.0f;
    while (final_target_yaw < -180.0f) final_target_yaw += 360.0f;

    /* 弧线终点目标位置: 绕圆心旋转 start→end
     * 外部dir=+1表示CW右转, 但下面的二维旋转矩阵以CCW为正,
     * 所以理论终点位置必须使用 -dir*sweep_deg。 */
    float v0x = move_x - cx;
    float v0y = move_y - cy;
    float theta_end = -(float)dir * sweep_deg;  /* 数学旋转角: CCW为正 */
    float cos_te = move_cos(theta_end);
    float sin_te = move_sin(theta_end);
    float end_x = cx + v0x * cos_te - v0y * sin_te;
    float end_y = cy + v0x * sin_te + v0y * cos_te;

    for (;;) {
        if (move_tick() - t0 >= timeout_ms) {
            arc_yaw_l_sum_active = 0;
            enc_has_last = false;
            Move_Stop(); move_sync_to_odom(); g_move_active = 0;
            return 0;
        }

        /* 读编码器 → 里程计 */
        int32_t cur_pos[4];
        dbg_enc_ok = move_read_all_encoders(cur_pos);
        move_update_odom(cur_pos);

        /* 当前位置相对圆心 */
        float dx = move_x - cx;
        float dy = move_y - cy;
        float r = move_sqrt(dx * dx + dy * dy);

        /* 累计扫过角度 (增量法, 避免 atan2 跳变) */
        float cur_angle = move_atan2(dy, dx);
        float delta = cur_angle - prev_angle;
        while (delta >  180.0f) delta -= 360.0f;
        while (delta < -180.0f) delta += 360.0f;
        swept += move_abs(delta);
        prev_angle = cur_angle;

        /* 编码器航向累计 (用于减速和停止判断) */
        float cur_yaw_raw = move_yaw;
        float yaw_delta = cur_yaw_raw - prev_yaw;
        while (yaw_delta >  180.0f) yaw_delta -= 360.0f;
        while (yaw_delta < -180.0f) yaw_delta += 360.0f;
        swept_yaw += move_abs(yaw_delta);
        prev_yaw = cur_yaw_raw;

        /* 编码器角速度 (停止预测用, 非命令速度) */
        uint32_t now_yaw_tick = move_tick();
        float dt_yaw = (now_yaw_tick - prev_yaw_tick) * 0.001f;
        if (dt_yaw < 0.001f) dt_yaw = 0.001f;
        float yaw_rate = move_abs(yaw_delta) / dt_yaw;
        yaw_rate_lp = 0.7f * yaw_rate_lp + 0.3f * yaw_rate;
        prev_yaw_tick = now_yaw_tick;

        /* 圆弧启动限加速度: 避免0.30m/s时200ms ramp带来过大的横向冲击 */
        float elapsed_s = (float)(move_tick() - t0) * 0.001f;
        float vf = speed;
        float vf_accel = MOVE_ARC_ACCEL * elapsed_s;
        if (vf > vf_accel) vf = vf_accel;

        /* 末端减速: yaw和位置扫角任一方没跟上, 都不能按完成处理。
         * 之前只按swept_yaw减速/停止, 会出现角度到了但弧长少走, 最终位置偏左。 */
        float arc_progress = (swept_yaw < swept) ? swept_yaw : swept;
        float remaining_deg = sweep_deg - arc_progress;
        float s_rem = (remaining_deg > 0.0f) ?
                      (remaining_deg * (3.14159265f / 180.0f) * radius) : 0.0f;
        float vf_profile = move_sqrt(2.0f * MOVE_ARC_ACCEL * s_rem);
        if (vf_profile < MOVE_CREEP_SPEED) vf_profile = MOVE_CREEP_SPEED;
        if (vf_profile < vf) vf = vf_profile;

        /* 完成检查 + 停止预测: yaw预测到位且位置扫角也走够, 才进入settle */
        float predicted_overshoot = yaw_rate_lp * ((float)MOVE_ARC_STOP_LATENCY_MS / 1000.0f);
        uint8_t yaw_done = (swept_yaw + predicted_overshoot >= sweep_deg);
        uint8_t pos_sweep_done = (swept + MOVE_ARC_SWEEP_TOL_DEG >= sweep_deg);
        if (yaw_done && pos_sweep_done) {
            Move_Stop();
            move_delay(100);  /* 机械惯性 settling 缓冲 */

            /* settle: 位置+航向联合P修正, 连续2帧在容差内才完成
             * 原来只被动等惯性消减→只修航向不修XY→几厘米偏差不修正
             * 现在: P拉向终点(end_x,end_y) + yaw hold, 位置和航向都修正
             * end_x/end_y 在弧线开始时按CW/CCW约定算好, 此处只使用目标结果 */
            uint8_t settle_cnt = 0;
            uint32_t settle_t0 = move_tick();
            while (move_tick() - settle_t0 < 1000u) {  /* 最多1000ms (含位置修正) */
                int32_t settle_pos[4];
                move_read_all_encoders(settle_pos);
                move_update_odom(settle_pos);
                /* settle 阶段继续使用编码器yaw, 与弧线主循环一致 */

                /* 位置误差 (场坐标: x=右, y=前) */
                float pos_err_x = end_x - move_x;
                float pos_err_y = end_y - move_y;
                float pos_err_dist = move_sqrt(pos_err_x * pos_err_x
                                             + pos_err_y * pos_err_y);

                /* 航向误差 (CW+) */
                float yaw_err = final_target_yaw - move_yaw;
                while (yaw_err >  180.0f) yaw_err -= 360.0f;
                while (yaw_err < -180.0f) yaw_err += 360.0f;

                /* 联合判定: 位置+航向都在容差内才算这一帧OK */
                if (pos_err_dist <= MOVE_ARC_POS_TOL &&
                    move_abs(yaw_err) <= MOVE_YAW_ACCEPT_DEG) {
                    settle_cnt++;
                    Move_Stop();
                    if (settle_cnt >= 2) break;
                } else {
                    settle_cnt = 0;
                    /* 位置P修正: 拉向终点 */
                    float vx = pos_err_x * MOVE_ARC_SETTLE_KP;
                    float vy = pos_err_y * MOVE_ARC_SETTLE_KP;
                    float spd = move_sqrt(vx * vx + vy * vy);
                    /* 最低速度地板: 克服静摩擦 (tolerance宽, 极限环风险低) */
                    if (spd > 0.0001f && spd < MOVE_MIN_SPEED) {
                        float s = MOVE_MIN_SPEED / spd;
                        vx *= s; vy *= s; spd = MOVE_MIN_SPEED;
                    }
                    /* 最高限速: gentle修正, 防冲击 */
                    if (spd > MOVE_ARC_SETTLE_MAX_SPEED) {
                        float s = MOVE_ARC_SETTLE_MAX_SPEED / spd;
                        vx *= s; vy *= s;
                    }
                    /* 航向保持 (同MoveTo yaw hold, 死区内不修防摇摆) */
                    float wz = 0.0f;
                    if (move_abs(yaw_err) > MOVE_YAW_HOLD_DEADZONE) {
                        wz = yaw_err * MOVE_YAW_KP;
                        wz = move_clamp(wz, -MOVE_YAW_HOLD_LIMIT,
                                              MOVE_YAW_HOLD_LIMIT);
                    }
                    Move_SetFieldVelocity(vx, vy, wz);
                }
                move_delay(MOVE_CTRL_PERIOD_MS);
            }
            Move_Stop();  /* 确保停驱动 (settle收敛或超时都走这里) */
            arc_yaw_l_sum_active = 0;
            enc_has_last = false;

            /* 最终航向修正: 仅在误差超过ACCEPT(0.4°)时才触发RotateTo
             * 弧线停止后只修正航向, 避免位置修正带来的蠕动 */
            float final_yaw = move_yaw;   /* 编码器CW+ */
            float yaw_err_end = final_target_yaw - final_yaw;
            while (yaw_err_end >  180.0f) yaw_err_end -= 360.0f;
            while (yaw_err_end < -180.0f) yaw_err_end += 360.0f;
            if (move_abs(yaw_err_end) > MOVE_YAW_ACCEPT_DEG) {
                RotateTo(final_target_yaw, MOVE_YAW_TURN_LIMIT);
            }

            /* 圆弧结束后保持编码器yaw, 后续普通段继续同一来源积分 */
            move_sync_to_odom();
            g_move_active = 0;
            return 1;
        }

        /* 切线方向: 垂直于半径, 朝运动方向 */
        float inv_r = (r > 1e-6f) ? (1.0f / r) : 0.0f;
        float tx =  (float)dir * dy * inv_r;
        float ty = -(float)dir * dx * inv_r;

        /* 径向修正: 拉回目标半径 */
        float r_err = r - radius;
        float vf_xy = vf * MOVE_ARC_XY_GAIN;
        float radial_spd = r_err * MOVE_ARC_KP_RADIAL;
        if (radial_spd >  vf_xy) radial_spd =  vf_xy;
        if (radial_spd < -vf_xy) radial_spd = -vf_xy;
        float vx_f = vf_xy * tx - radial_spd * dx * inv_r;
        float vy_f = vf_xy * ty - radial_spd * dy * inv_r;

        /* 速度相关径向外推: 实车高速圆弧会切内圈/半径少走时, 沿圆心->车体方向前馈。
         * 这比固定车体右向补偿更符合圆弧几何, 左右圆弧都按实际圆心方向处理。 */
        float outward_comp = MOVE_ARC_OUTWARD_COMP_K * vf * vf / radius;
        outward_comp = move_clamp(outward_comp, -MOVE_ARC_OUTWARD_COMP_MAX,
                                                MOVE_ARC_OUTWARD_COMP_MAX);
        vx_f += outward_comp * dx * inv_r;
        vy_f += outward_comp * dy * inv_r;

        /* 航向: 前馈 + P反馈 */
        float target_yaw = start_yaw + (float)dir * swept;
        float yaw_err = target_yaw - move_yaw;
        while (yaw_err >  180.0f) yaw_err -= 360.0f;
        while (yaw_err < -180.0f) yaw_err += 360.0f;

        float wz_ff = (float)dir * (vf / radius) * MOVE_ARC_YAW_L_SUM;
        float wz_p  = yaw_err * MOVE_WP_YAW_KP;
        float wz = wz_ff + wz_p;
        if (wz >  MOVE_WP_YAW_MAX) wz =  MOVE_WP_YAW_MAX;
        if (wz < -MOVE_WP_YAW_MAX) wz = -MOVE_WP_YAW_MAX;

        /* 发送 */
        uint32_t now_iter = move_tick();
        uint16_t loop_ms = (uint16_t)(now_iter - prev_iter_tick);
        dbg_loop_ms = loop_ms;
        prev_iter_tick = now_iter;
        if (g_path_debug_en) {
            SendPathDebugToPC(move_x, move_y, (int16_t)(swept * 10), (int16_t)(sweep_deg * 10),
                              vx_f, vy_f, wz, target_yaw, loop_ms, dbg_enc_ok);
        }
        Move_SetFieldVelocity(vx_f, vy_f, wz);
        move_sync_to_odom();
        move_delay(MOVE_CTRL_PERIOD_MS);
    }
}

/* ================================================================
 *  控制函数 — MovePathTrack (路径跟踪)
 *
 *  线段投影 + 横向误差修正 + 起始航向保持。
 *  普通点不停; 关键点作为终点时先位置到位, 再单独RotateTo到目标角。
 * ================================================================ */

/* 路径缓冲装载 (ISR上下文调用, 仅写全局, 无FreeRTOS调用) */
void Move_PathBegin(uint8_t count, float speed)
{
    g_path_count    = 0;
    g_path_expected = count;
    g_path_load_error = 0;

    if (count < 2 || count > MOVE_WP_MAX_PTS) {
        g_path_load_error = 1;
    }

    if (!(speed > 0.01f)) {
        speed = MOVE_WP_SPEED;
    }
    if (speed > MOVE_MAX_SPEED) {
        speed = MOVE_MAX_SPEED;
    }
    g_path_speed = speed;
}

void Move_PathAddPoint(float x, float y, float target_theta, uint8_t mode)
{
    if (g_path_count >= MOVE_WP_MAX_PTS || g_path_count >= g_path_expected) {
        g_path_load_error = 1;
        return;
    }

    if (mode != PATH_MODE_NORMAL && mode != PATH_MODE_KEY) {
        g_path_load_error = 1;
        return;
    }

    g_path_pts[g_path_count].x            = x;
    g_path_pts[g_path_count].y            = y;
    g_path_pts[g_path_count].target_theta = target_theta;
    g_path_pts[g_path_count].mode         = mode;
    g_path_count++;
}

/* ── 路径跟踪器 ── */

/* ── 路径航向保持 ───────────────────────────────────────────────
 * path 主循环只负责连续平移追线, 不在普通点按切线方向强行转头。
 * 原来的“切线跟随 + 横向追线”会让麦轮XY和yaw强耦合, 实车表现为蛇形震荡。
 * 终点需要指定角度时, 先让位置停稳, 再复用 RotateTo 单独闭环转角。
 */
static float path_yaw_hold_ctrl(float target_yaw, float yaw)
{
    float yaw_err = target_yaw - yaw;
    while (yaw_err >  180.0f) yaw_err -= 360.0f;
    while (yaw_err < -180.0f) yaw_err += 360.0f;

    if (move_abs(yaw_err) <= MOVE_YAW_HOLD_DEADZONE) {
        return 0.0f;
    }

    float wz = yaw_err * MOVE_YAW_KP;
    return move_clamp(wz, -MOVE_YAW_HOLD_LIMIT, MOVE_YAW_HOLD_LIMIT);
}

/**
 * @brief  路径跟踪: 线段投影 + 横向修正 + 姿态控制
 *
 *         投影机器人位置到当前线段 → 前进速度沿切线 +
 *         横向修正∝偏移 → 投影过末端自动切下一段。
 *         姿态: 移动中保持起始航向; 终点若为关键点, 位置到位后再单独RotateTo.
 *
 * @return 1=完成, 0=超时/装载无效
 */
uint8_t MovePathTrack(void)
{
    int   n     = (int)g_path_count;
    float speed = g_path_speed;

    if (g_path_load_error || n < 2 || n != (int)g_path_expected) return 0;

    float total_len = 0.0f;
    for (int i = 0; i < n - 1; i++) {
        float dx = g_path_pts[i+1].x - g_path_pts[i].x;
        float dy = g_path_pts[i+1].y - g_path_pts[i].y;
        total_len += move_sqrt(dx * dx + dy * dy);
    }
    uint32_t path_timeout_ms = MOVE_WP_TIMEOUT_MS;
    if (speed > 0.01f && total_len > 0.001f) {
        uint32_t dyn_to = (uint32_t)((total_len / speed) * 3000.0f) + 5000UL;
        if (dyn_to > path_timeout_ms) path_timeout_ms = dyn_to;
    }

    g_move_active = 1;
    enc_has_last = false;

    const PathPt_t *p = g_path_pts;
    int seg = 0;                          /* 当前线段: p[seg]→p[seg+1] */
    float path_hold_yaw = move_yaw;        /* path平移阶段保持起始航向, 防切线转头导致蛇形震荡 */

    uint32_t t0 = move_tick();
    uint32_t prev_iter_tick = t0;

    for (;;) {
        if (move_tick() - t0 >= path_timeout_ms) {
            Move_Stop(); move_sync_to_odom(); g_move_active = 0;
            return 0;
        }
        /* 1. 读编码器 → 里程计 */
        int32_t cur_pos[4];
        dbg_enc_ok = move_read_all_encoders(cur_pos);
        move_update_odom(cur_pos);
        float rx = move_x, ry = move_y;

        /* 2. 当前线段向量 + 投影 */
        float sdx = p[seg+1].x - p[seg].x;
        float sdy = p[seg+1].y - p[seg].y;
        float slen2 = sdx * sdx + sdy * sdy;

        float t = 0.0f;
        if (slen2 < 1e-8f) {
            if (seg < n - 2) { seg++; continue; }
            t = 1.0f;  /* 终点退化段: 直接进入终点收敛/航向检查 */
        } else {
            float rxr = rx - p[seg].x, ryr = ry - p[seg].y;
            t = (rxr * sdx + ryr * sdy) / slen2;  /* [0=起点, 1=终点] */
        }

        /* 3. 普通通过点: 进入通过半径或投影越过端点即切下一段, 不停车 */
        if (t >= 1.0f && seg < n - 2) {
            if (p[seg+1].mode == PATH_MODE_NORMAL) {
                seg++;
                continue;
            }
        }
        if (seg < n - 2 && p[seg+1].mode == PATH_MODE_NORMAL) {
            float nx = p[seg+1].x - rx;
            float ny = p[seg+1].y - ry;
            if (move_sqrt(nx * nx + ny * ny) <= MOVE_WP_PASS_RADIUS) {
                seg++;
                continue;
            }
        } else if (t >= 1.0f && seg < n - 2) {
            /* 中间关键点保留为“减速通过点”: 不在单条path内停等上位机动作。
             * 需要取放/识别停稳时, 应把路线拆成多条path, 让关键点作为终点。 */
            seg++;
            continue;
        }

        /* 4. 末段: 投影超过终点 → 朝终点直线收敛 */
        int at_end = 0;
        if (seg >= n - 2 && t >= 1.0f) {
            float exx = p[n-1].x - rx, eyy = p[n-1].y - ry;
            float dist_end = move_sqrt(exx * exx + eyy * eyy);
            if (dist_end <= MOVE_WP_END_TOL) {
                Move_Stop();
                move_sync_to_odom();
                if (p[n-1].mode != PATH_MODE_KEY) {
                    g_move_active = 0;
                    return 1;
                }

                float final_yaw_err = p[n-1].target_theta - move_yaw;
                while (final_yaw_err >  180.0f) final_yaw_err -= 360.0f;
                while (final_yaw_err < -180.0f) final_yaw_err += 360.0f;
                if (move_abs(final_yaw_err) <= MOVE_YAW_ACCEPT_DEG) {
                    g_move_active = 0;
                    return 1;
                }

                enc_has_last = false;
                uint8_t rot_ok = RotateTo(p[n-1].target_theta, MOVE_YAW_TURN_LIMIT);
                move_sync_to_odom();
                g_move_active = 0;
                return rot_ok;
            }
            t = 1.0f;      /* 钳位: 横向误差 = 机器人→终点 */
            at_end = 1;     /* 路径已尽: 停前进, 只靠横向修正收敛 */
        }

        /* 5. 横向误差 = 机器人位置 - 路径上最近点 */
        float slen   = move_sqrt(slen2);
        float inv_sl = (slen > 1e-6f) ? (1.0f / slen) : 0.0f;
        float tx     = sdx * inv_sl;          /* 单位切线 X */
        float ty     = sdy * inv_sl;          /* 单位切线 Y */
        float cx     = p[seg].x + t * sdx;    /* 路径上最近点 */
        float cy     = p[seg].y + t * sdy;
        float ex     = rx - cx, ey = ry - cy; /* 横向误差向量 */
        float elen   = move_sqrt(ex * ex + ey * ey);

        /* 6. 前进速度: 沿切线, ramp + 末端sqrt减速 */
        float ramp = (float)(move_tick() - t0) / (float)MOVE_RAMP_TIME_MS;
        if (ramp > 1.0f) ramp = 1.0f;
        float vf = speed * ramp;

        if (!at_end) {
            float rem = (1.0f - t) * slen;       /* 剩余当前段 */
            for (int i = seg + 1; i < n - 1; i++) {
                float dx2 = p[i+1].x - p[i].x, dy2 = p[i+1].y - p[i].y;
                rem += move_sqrt(dx2 * dx2 + dy2 * dy2);
            }
            if (rem < MOVE_DECEL_DIST) {
                float decel = speed * move_sqrt(rem / MOVE_DECEL_DIST);
                if (decel < MOVE_MIN_SPEED) decel = MOVE_MIN_SPEED;
                if (vf > decel) vf = decel;
            }
            /* 关键点减速: 接近关键点时降速, 给姿态环收敛时间 */
            if (p[seg+1].mode == PATH_MODE_KEY) {
                float d2k = (1.0f - t) * slen;
                if (d2k < MOVE_WP_KEY_DECEL_DIST) {
                    float kvf = speed * move_sqrt(d2k / MOVE_WP_KEY_DECEL_DIST);
                    if (kvf < MOVE_WP_KEY_MIN_SPEED) kvf = MOVE_WP_KEY_MIN_SPEED;
                    if (vf > kvf) vf = kvf;
                }
            }
        } else {
            vf = 0.0f;  /* 路径已尽: 停前进, 只靠横向修正收敛到终点 */
        }

        /* 7. 横向修正: ∝误差, 方向指向路径 */
        float vl = elen * MOVE_WP_LAT_KP;
        float vl_cap = at_end ? MOVE_WP_SETTLE_MAX_SPEED : (vf * MOVE_WP_LAT_MAX_RATIO);
        if (vl > vl_cap) vl = vl_cap;
        float inv_el = (elen > 0.001f) ? (1.0f / elen) : 0.0f;

        float vx_f = vf * tx - vl * ex * inv_el;  /* 前进 + 向路径修正 */
        float vy_f = vf * ty - vl * ey * inv_el;

        /* 8. 姿态控制: 移动中只保持起始航向, 不跟随折线路径切线转头 */
        float wz = path_yaw_hold_ctrl(path_hold_yaw, move_yaw);
        float target_yaw = path_hold_yaw;

        /* 9. 发送 */
        uint32_t now_iter = move_tick();
        uint16_t loop_ms  = (uint16_t)(now_iter - prev_iter_tick);
        dbg_loop_ms = loop_ms;
        prev_iter_tick = now_iter;
        if (g_path_debug_en) {
            SendPathDebugToPC(move_x, move_y, (int16_t)seg, (int16_t)n,
                              vx_f, vy_f, wz, target_yaw, loop_ms, dbg_enc_ok);
        }
        Move_SetFieldVelocity(vx_f, vy_f, wz);
        move_sync_to_odom();
        move_delay(MOVE_CTRL_PERIOD_MS);
    }
}
