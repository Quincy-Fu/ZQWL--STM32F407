/**
 * @file    move.c
 * @brief   底层运动控制模块 — 移植自Blu3 Move层
 *
 * 30ms控制环: P环位置控制 + 编码器回读XY + yaw反馈.
 * 阻塞式API, 在NavTask中调用.
 * 注: yaw反馈源可运行期切换；IMU源启用时编码器yaw仅作掉线兜底.
 *
 * 坐标系: vy>0=前进, vx>0=右移, wz>0=CW(顺时针)
 * 电机地址: FL=0x01, FR=0x02, RL=0x03, RR=0x04
 * 右轮(FR/RR)镜像安装, motor_emit方向反转。
 */

#include "move.h"
#include "Emm_V5.h"
#include "can.h"
#include "imu_protocol.h"
#include "uart_protocol.h"
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
volatile uint8_t g_move_yaw_source = MOVE_YAW_SOURCE_DEFAULT;

/* ── 编码器诊断 (LCD显示用) ── */
volatile int32_t dbg_enc_raw[4]  = {0, 0, 0, 0};   /* 最近一次原始S_CPOS */
volatile int32_t dbg_enc_delta[4] = {0, 0, 0, 0};   /* 最近一次delta (counts) */
volatile int32_t dbg_enc_ok = 0;                     /* 成功读到的轮数 */
volatile uint8_t dbg_enc_fail = 0;                   /* 最近失败原因: 0=成功 1=超时 2=错地址 3=错功能码 4=错DLC */
volatile uint32_t dbg_enc_bad_delta = 0;             /* 合理性检查失败次数 (delta超限) */
volatile int16_t dbg_cmd_rpm[4] = {0, 0, 0, 0};     /* 最近一次发给各电机的RPM命令 */
volatile uint16_t dbg_loop_ms = 0;                     /* 控制环实际周期 ms (Watch看这个, 正常~28ms) */
volatile uint8_t g_path_debug_en = 0;                  /* 调试帧默认关闭，避免UART拥塞影响正常响应 */

/* ================================================================
 *  外部引用 (freertos.c / imu_protocol.c)
 * ================================================================ */
extern volatile float g_odom_x;        /* 里程计X, 同步用 */
extern volatile float g_odom_y;        /* 里程计Y, 同步用 */
extern volatile float g_odom_theta;    /* 里程计theta(CW+弧度), 同步用 */
extern volatile float g_imu_yaw;       /* IMU滤波后yaw, 原始方向由安装决定 */
extern volatile uint8_t g_imu_verified;
extern volatile uint32_t g_imu_last_tick;
extern void RotateQueue_Push(uint8_t pos);    /* 圆弧过程中把转盘槽位请求加入队列 */
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
static uint8_t arc_tt_enable = 0;         /* 1=本次圆弧按进度触发转盘 */
static float   arc_tt_deg[3] = {0.0f, 0.0f, 0.0f};
static uint8_t arc_tt_slot[3] = {0, 0, 0};
#if MOVE_YAW_USE_IMU
static float   move_imu_yaw_offset = 0.0f; /* 外部同步yaw - IMU原始CW yaw */
static uint8_t move_imu_yaw_aligned = 0;   /* 1=IMU yaw已和move_yaw建立offset */
static uint8_t move_imu_yaw_have_last = 0; /* 1=已有上一帧被接受的IMU yaw */
#endif

/* yaw源诊断: Watch窗口可观察, 用于判断是否发生IMU跳变/断联兜底。 */
volatile uint32_t dbg_imu_yaw_fallback_count = 0;
volatile uint32_t dbg_imu_yaw_jump_reject_count = 0;
volatile float    dbg_imu_yaw_reject_delta = 0.0f;

/* BODY_POS诊断: 用于判断四轮位置命令是否有单电机漏执行。 */
volatile uint32_t dbg_body_pos_verify_fail_count = 0;
volatile int32_t  dbg_body_pos_expected[4] = {0, 0, 0, 0};
volatile int32_t  dbg_body_pos_actual[4] = {0, 0, 0, 0};

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

static void move_arc_request_turntable(uint8_t slot)
{
    if (slot > ROTATE_STATE_MAX) return;
    RotateQueue_Push(slot);
}

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

static float move_norm_deg180(float deg)
{
    while (deg >  180.0f) deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}

static uint8_t move_to_accurate_timed_ex(float tx, float ty, float max_speed,
                                         float tol, uint32_t timeout_ms,
                                         float decel_dist,
                                         float min_speed, float creep_speed);
static uint8_t move_to_yaw_timed_ex(float tx, float ty, float target_yaw_deg,
                                    float max_speed, float tol,
                                    uint32_t timeout_ms, float decel_dist,
                                    float min_speed, float creep_speed,
                                    uint8_t stop_on_done);
static void move_sync_to_odom(void);

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

#if MOVE_YAW_USE_IMU
static uint8_t move_get_imu_yaw_cw(float *out_yaw_cw)
{
    if (!out_yaw_cw) return 0;

    float imu_yaw_snapshot;
    uint8_t imu_verified_snapshot;
    uint32_t imu_tick_snapshot;
    __disable_irq();
    imu_yaw_snapshot = g_imu_yaw;
    imu_verified_snapshot = g_imu_verified;
    imu_tick_snapshot = g_imu_last_tick;
    __enable_irq();

    if (!imu_verified_snapshot || imu_tick_snapshot == 0u) {
        return 0;
    }
    if ((xTaskGetTickCount() - imu_tick_snapshot) > pdMS_TO_TICKS(MOVE_IMU_TIMEOUT_MS)) {
        return 0;
    }

    /* 用户体系约定CW为正；当前IMU接线/安装下原始yaw已与系统方向一致。 */
    *out_yaw_cw = MOVE_IMU_YAW_SIGN * imu_yaw_snapshot;
    return 1;
}
#endif

void Move_UpdateYawFeedback(float encoder_delta_cw_deg)
{
#if MOVE_YAW_USE_IMU
    if (g_move_yaw_source == MOVE_YAW_SOURCE_IMU) {
        float imu_yaw_cw;
        if (move_get_imu_yaw_cw(&imu_yaw_cw)) {
            float imu_yaw_mapped = imu_yaw_cw + move_imu_yaw_offset;
            uint8_t accept_imu = 1;

            if (move_imu_yaw_aligned && move_imu_yaw_have_last) {
                float delta = imu_yaw_mapped - move_yaw;
                while (delta >  180.0f) delta -= 360.0f;
                while (delta < -180.0f) delta += 360.0f;
                if (move_abs(delta) > MOVE_IMU_YAW_MAX_STEP_DEG) {
                    accept_imu = 0;
                    dbg_imu_yaw_jump_reject_count++;
                    dbg_imu_yaw_reject_delta = delta;
                }
            }

            if (accept_imu) {
                __disable_irq();
                if (!move_imu_yaw_aligned) {
                    move_imu_yaw_offset = move_yaw - imu_yaw_cw;
                    imu_yaw_mapped = move_yaw;
                    move_imu_yaw_aligned = 1;
                }
                move_yaw = imu_yaw_mapped;
                move_imu_yaw_have_last = 1;
                __enable_irq();
                return;
            }

            /* IMU帧新鲜但跳变异常: 不接受本帧, 本周期改用编码器增量。 */
            dbg_imu_yaw_fallback_count++;
        } else {
            dbg_imu_yaw_fallback_count++;
        }
    }
#else
    if (g_move_yaw_source == MOVE_YAW_SOURCE_IMU) {
        dbg_imu_yaw_fallback_count++;
    }
#endif

    /* IMU未就绪、掉线或跳变异常时, 用编码器增量兜底, 避免角度环失控。 */
    __disable_irq();
    move_yaw += encoder_delta_cw_deg;
    __enable_irq();
}

uint8_t Move_SetYawSource(uint8_t source)
{
    if (source != MOVE_YAW_SOURCE_ENCODER && source != MOVE_YAW_SOURCE_IMU) {
        return 0;
    }

#if MOVE_YAW_USE_IMU
    if (source == MOVE_YAW_SOURCE_IMU) {
        float imu_yaw_cw;
        __disable_irq();
        g_move_yaw_source = MOVE_YAW_SOURCE_IMU;
        __enable_irq();

        if (move_get_imu_yaw_cw(&imu_yaw_cw)) {
            __disable_irq();
            move_imu_yaw_offset = move_yaw - imu_yaw_cw;
            move_imu_yaw_aligned = 1;
            move_imu_yaw_have_last = 0;
            __enable_irq();
        } else {
            __disable_irq();
            move_imu_yaw_aligned = 0;
            move_imu_yaw_have_last = 0;
            __enable_irq();
        }
        return 1;
    }

    __disable_irq();
    g_move_yaw_source = MOVE_YAW_SOURCE_ENCODER;
    move_imu_yaw_aligned = 0;
    move_imu_yaw_have_last = 0;
    __enable_irq();
    return 1;
#else
    if (source == MOVE_YAW_SOURCE_IMU) {
        return 0;
    }
    __disable_irq();
    g_move_yaw_source = MOVE_YAW_SOURCE_ENCODER;
    __enable_irq();
    return 1;
#endif
}

uint8_t Move_GetYawSource(void)
{
    return g_move_yaw_source;
}

/* ================================================================
 *  电机驱动层
 * ================================================================ */

/**
 * @brief  设置4轮速度并同步启动
 *
 * 四轮先下发待同步速度(snF=true), 再广播同步触发。
 * 这样每个控制周期四轮同时切换速度, 避免逐轮立即执行造成瞬时斜移。
 *
 * @param w 4轮速度数组 m/s, 正=前进
 */
static void move_set_wheels(const float w[4])
{
    /* 逐轮写入待同步速度, 四轮都收到后再统一启动。 */
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

        Emm_V5_Vel_Control(wheel_addr[i], dir, vel, MOVE_ACC_DEFAULT, true);
        move_delay(MOVE_CMD_DELAY_MS);
    }

    move_delay(MOVE_CMD_DELAY_MS);
    Emm_V5_Synchronous_motion(0x00);
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
 *   yaw反馈: IMU优先, 编码器增量仅作兜底
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

    Move_UpdateYawFeedback(dtheta_cw);

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
    move_yaw = yaw_deg;               /* CW正, 外部同步后的全局yaw */

#if MOVE_YAW_USE_IMU
    if (g_move_yaw_source == MOVE_YAW_SOURCE_IMU) {
        float imu_yaw_cw;
        if (move_get_imu_yaw_cw(&imu_yaw_cw)) {
            move_imu_yaw_offset = yaw_deg - imu_yaw_cw;
            move_imu_yaw_aligned = 1;
            move_imu_yaw_have_last = 0;
        } else {
            /* IMU还没出有效帧时先保留外部yaw；首个有效IMU帧到来时再无跳变对齐。 */
            move_imu_yaw_aligned = 0;
            move_imu_yaw_have_last = 0;
        }
    } else {
        move_imu_yaw_aligned = 0;
        move_imu_yaw_have_last = 0;
    }
#endif

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
    Move_InitPose(0.0f, 0.0f, 0.0f);  /* 当前航向基准归零 */
}

float Move_GetYaw(void)
{
    Move_UpdateYawFeedback(0.0f);
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
    Move_UpdateYawFeedback(0.0f);

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
    /*
     * 用同步急停保证四轮同时锁死。
     * 原因: 任何一个电机漏收同步急停帧, 广播同步后该轮仍可能保持旧速度,
     * 会在停车瞬间造成偏航。因此重复装载同步急停帧, 每轮都广播同步触发。
     */
    for (uint8_t rep = 0; rep < MOVE_STOP_SYNC_REPEATS; rep++) {
        for (uint8_t i = 0; i < 4; i++) {
            Emm_V5_Stop_Now(wheel_addr[i], true);
            move_delay(MOVE_STOP_CMD_DELAY_MS);
        }
        move_delay(MOVE_STOP_CMD_DELAY_MS);
        Emm_V5_Synchronous_motion(0x00);
        move_delay(MOVE_STOP_CMD_DELAY_MS);
    }

    for (uint8_t i = 0; i < 4; i++) {
        cmd_wheel_rpm[i] = 0.0f;
        dbg_cmd_rpm[i] = 0;
    }
}

static void move_refresh_odom_once(void)
{
    int32_t cur_pos[4];
    dbg_enc_ok = move_read_all_encoders(cur_pos);
    move_update_odom(cur_pos);
    move_sync_to_odom();
}

static uint8_t move_fine_goto_field(float tx, float ty, uint32_t timeout_ms)
{
    uint8_t ok = 0;
    for (uint8_t attempt = 0; attempt <= MOVE_FINE_RECHECK_MAX; attempt++) {
        ok = move_to_accurate_timed_ex(tx, ty,
                                       MOVE_FINE_LOOP_SPEED,
                                       MOVE_FINE_LOOP_TOL,
                                       timeout_ms,
                                       MOVE_FINE_LOOP_DECEL_DIST,
                                       MOVE_FINE_LOOP_MIN_SPEED,
                                       MOVE_FINE_LOOP_CREEP_SPEED);
        if (!ok) {
            return 0;
        }

        if (MOVE_FINE_RECHECK_SETTLE_MS > 0u) {
            move_delay(MOVE_FINE_RECHECK_SETTLE_MS);
        }
        move_refresh_odom_once();

        float dx = tx - move_x;
        float dy = ty - move_y;
        float dist = move_sqrt(dx * dx + dy * dy);
        if (dist <= MOVE_FINE_LOOP_TOL) {
            return 1;
        }
    }

    /* 已完成限定次数补偿。即使编码器残差略大，也认为本次微调已尽力执行，
     * 避免视觉上层因1~2mm残差被硬中断；后续由视觉复检继续修正。 */
    return 1;
}

static uint8_t move_fine_segment_body(float dx_body_m, float dy_body_m,
                                      uint32_t timeout_ms)
{
    float dist = move_sqrt(dx_body_m * dx_body_m + dy_body_m * dy_body_m);
    if (dist <= MOVE_FINE_AXIS_EPS) {
        return 1;
    }

    /* dx/dy是车体坐标相对位移；位置环吃场地坐标目标点。 */
    Move_UpdateYawFeedback(0.0f);
    float yaw_deg = -move_yaw;
    float cy = move_cos(yaw_deg);
    float sy = move_sin(yaw_deg);
    float dx_field = dx_body_m * cy - dy_body_m * sy;
    float dy_field = dx_body_m * sy + dy_body_m * cy;
    float tx = move_x + dx_field;
    float ty = move_y + dy_field;

    return move_fine_goto_field(tx, ty, timeout_ms);
}

uint8_t MoveFinePositionBody(float dx_body_m, float dy_body_m,
                             uint32_t timeout_ms)
{
    float dist = move_sqrt(dx_body_m * dx_body_m + dy_body_m * dy_body_m);
    if (dist <= MOVE_FINE_LOOP_TOL) {
        return 1;
    }

    if (timeout_ms == 0u) timeout_ms = MOVE_FINE_LOOP_TIMEOUT_MS;

    /* 视觉微调优先走单轴，避免麦轮小距离斜走耦合误差；分量大的轴先走。 */
    float abs_dx = move_abs(dx_body_m);
    float abs_dy = move_abs(dy_body_m);
    if (abs_dx >= abs_dy) {
        if (!move_fine_segment_body(dx_body_m, 0.0f, timeout_ms)) return 0;
        if (!move_fine_segment_body(0.0f, dy_body_m, timeout_ms)) return 0;
    } else {
        if (!move_fine_segment_body(0.0f, dy_body_m, timeout_ms)) return 0;
        if (!move_fine_segment_body(dx_body_m, 0.0f, timeout_ms)) return 0;
    }
    return 1;
}

static uint8_t move_body_pos_dir(float wheel_m, uint8_t idx)
{
    uint8_t dir = (wheel_m >= 0.0f) ? 0u : 1u;
    if (wheel_mirror[idx]) {
        dir = dir ? 0u : 1u;
    }
    return dir;
}

static uint32_t move_body_pos_pulses(float wheel_m)
{
    float dist_m = move_abs(wheel_m);
    /* 位置模式 clk 是电机轴输入脉冲数。对 45°麦轮，电机/轮子转一圈，
     * 车体前进或横移的有效位移是 πD×cos45，而不是完整轮周长 πD。
     * 这个模型与 oflow_calib.c 的位置模式标定一致。 */
    float rev = dist_m / (3.14159265f * MOVE_WHEEL_D * MOVE_BODY_POS_MECANUM_FACTOR);
    return (uint32_t)(rev * (float)MOVE_BODY_POS_PULSES_PER_REV + 0.5f);
}

static uint8_t move_body_pos_read_snapshot(int32_t pos[4])
{
    for (uint8_t i = 0; i < 4; i++) {
        uint8_t ok = 0;
        for (uint8_t retry = 0; retry < 2; retry++) {
            if (move_read_encoder(i, &pos[i])) {
                ok = 1;
                break;
            }
            move_delay(5);
        }
        if (!ok) {
            return 0;
        }
    }
    return 1;
}

static int32_t move_body_pos_expected_encoder_counts(float wheel_m, uint8_t idx)
{
    /* S_CPOS原始方向和逻辑轮方向在右侧镜像轮相反。 */
    float raw_m = wheel_mirror[idx] ? -wheel_m : wheel_m;
    float counts_f = raw_m / MOVE_ENC_TO_M;
    if (counts_f >= 0.0f) {
        return (int32_t)(counts_f + 0.5f);
    }
    return (int32_t)(counts_f - 0.5f);
}

static uint8_t move_body_pos_verify(const float wheel_m[4],
                                    const int32_t start_pos[4],
                                    const int32_t end_pos[4])
{
    uint8_t ok = 1;
    for (uint8_t i = 0; i < 4; i++) {
        int32_t expected = move_body_pos_expected_encoder_counts(wheel_m[i], i);
        int32_t actual = enc_safe_delta(end_pos[i], start_pos[i]);
        int32_t abs_expected = expected >= 0 ? expected : -expected;
        int32_t abs_actual = actual >= 0 ? actual : -actual;

        dbg_body_pos_expected[i] = expected;
        dbg_body_pos_actual[i] = actual;

        if (abs_expected < MOVE_BODY_POS_VERIFY_MIN_COUNTS) {
            continue;
        }

        if (((expected > 0) && (actual <= 0)) ||
            ((expected < 0) && (actual >= 0))) {
            ok = 0;
            continue;
        }

        if ((float)abs_actual < (float)abs_expected * MOVE_BODY_POS_VERIFY_MIN_RATIO) {
            ok = 0;
        }
    }

    if (!ok) {
        dbg_body_pos_verify_fail_count++;
    }
    return ok;
}

static void move_body_pos_apply_odom(float dx_body_m, float dy_body_m)
{
    /* 开环位移没有实时编码器积分，这里按命令值同步一次坐标。
     * 上位机任务点通常会在 ring 结束后再 sync_pose，最终以任务坐标为准。 */
    Move_UpdateYawFeedback(0.0f);
    float yaw_deg = -move_yaw;
    float cy = move_cos(yaw_deg);
    float sy = move_sin(yaw_deg);
    move_x += dx_body_m * cy - dy_body_m * sy;
    move_y += dx_body_m * sy + dy_body_m * cy;
    move_sync_to_odom();
    enc_has_last = false;
}

static void move_body_pos_force_speed_zero(void)
{
    /* BODY_POS 使用驱动器位置模式；退出后再慢速下发一次速度模式0速。
     * 目的: 防止个别电机仍停留在位置模式末端保持/残余修正，下一条path起步时四轮不同步。 */
    for (uint8_t i = 0; i < 4; i++) {
        Emm_V5_Vel_Control(wheel_addr[i], 0u, 0u, MOVE_ACC_DEFAULT, true);
        move_delay(MOVE_BODY_POS_EXIT_CMD_GAP_MS);
    }
    move_delay(MOVE_BODY_POS_EXIT_SYNC_MS);
    Emm_V5_Synchronous_motion(0x00);
    move_delay(MOVE_BODY_POS_EXIT_SYNC_MS);
}

uint8_t MoveBodyPositionOpenLoop(float dx_body_m, float dy_body_m)
{

    float dist = move_sqrt(dx_body_m * dx_body_m + dy_body_m * dy_body_m);
    if (dist <= 0.0005f) {
        return 1;
    }
    if (dist > MOVE_BODY_POS_MAX_DIST) {
        return 0;
    }

    /* 麦轮逆运动学的位移版本: 正值表示该轮按“前进方向”等效转动。 */
    float wheel_m[4] = {
        dy_body_m + dx_body_m,   /* FL */
        dy_body_m - dx_body_m,   /* FR */
        dy_body_m - dx_body_m,   /* RL */
        dy_body_m + dx_body_m    /* RR */
    };

    uint32_t pulses[4];
    uint32_t max_pulses = 0u;
    for (uint8_t i = 0; i < 4; i++) {
        pulses[i] = move_body_pos_pulses(wheel_m[i]);
        if (pulses[i] > max_pulses) max_pulses = pulses[i];
    }
    if (max_pulses == 0u) {
        return 1;
    }

    g_move_active = 1;
    Move_UpdateYawFeedback(0.0f);
    move_target_yaw = move_yaw;
    enc_has_last = false;
    move_delay(30);  /* 等OdomTask退出本轮CAN回读, 避免抢同一条S_CPOS响应 */

    int32_t start_pos[4];
    int32_t end_pos[4];
    if (!move_body_pos_read_snapshot(start_pos)) {
        g_move_active = 0;
        enc_has_last = false;
        return 0;
    }

    for (uint8_t i = 0; i < 4; i++) {
        Emm_V5_Pos_Control(wheel_addr[i],
                           move_body_pos_dir(wheel_m[i], i),
                           MOVE_BODY_POS_VEL_RPM,
                           MOVE_BODY_POS_ACC,
                           pulses[i],
                           false,  /* 相对位置 */
                           true);  /* 等广播同步触发 */
        move_delay(MOVE_BODY_POS_CMD_GAP_MS);  /* 13字节命令会拆成2帧CAN，留足帧间隔 */
    }
    move_delay(MOVE_BODY_POS_CMD_GAP_MS);
    Emm_V5_Synchronous_motion(0x00);

    float rev = (float)max_pulses / (float)MOVE_BODY_POS_PULSES_PER_REV;
    uint32_t motion_ms = (uint32_t)(rev * 60.0f /
                                   (float)MOVE_BODY_POS_VEL_RPM * 1000.0f);
    uint32_t wait_ms = motion_ms + MOVE_BODY_POS_MARGIN_MS + MOVE_BODY_POS_SETTLE_MS;
    if (wait_ms < MOVE_BODY_POS_MIN_WAIT_MS) {
        wait_ms = MOVE_BODY_POS_MIN_WAIT_MS;
    }
    move_delay(wait_ms);

    for (uint8_t i = 0; i < 4; i++) {
        cmd_wheel_rpm[i] = 0.0f;
        dbg_cmd_rpm[i] = 0;
    }

    if (!move_body_pos_read_snapshot(end_pos) ||
        !move_body_pos_verify(wheel_m, start_pos, end_pos)) {
        Move_Stop();
        enc_has_last = false;
        g_move_active = 0;
        return 0;
    }

    /* 位置模式估算到位后，先同步停四轮再返回响应。
     * 目的: 避免下一条GOTO/FINE_MOVE刚接上时，驱动器仍在做位置模式末端保持/残余修正。 */
    Move_Stop();
    move_body_pos_force_speed_zero();
    if (MOVE_BODY_POS_RELEASE_MS > 0u) {
        move_delay(MOVE_BODY_POS_RELEASE_MS);
    }

    move_body_pos_apply_odom(dx_body_m, dy_body_m);
    g_move_active = 0;
    return 1;
}

static void move_cd_fixed_apply_odom(float vx_body, float vy_body,
                                     float wz_body, float yaw_l_sum,
                                     uint32_t dt_ms)
{
    if (dt_ms == 0u) return;

    float dt = (float)dt_ms * 0.001f;

    /* 用命令速度做开环位姿估算。IMU有效时yaw由Move_UpdateYawFeedback接管；
     * IMU无效时用命令wz按当前有效L_SUM积分, 保证POSE不会完全停在旧值。 */
    float dtheta_cw = 0.0f;
    if (yaw_l_sum > 0.001f) {
        dtheta_cw = (wz_body / yaw_l_sum) * dt * 57.2957795f;
    }
    Move_UpdateYawFeedback(dtheta_cw);

    float yaw_deg = -move_yaw;
    float cy = move_cos(yaw_deg);
    float sy = move_sin(yaw_deg);
    float dx_body = vx_body * dt;
    float dy_body = vy_body * dt;

    move_x += dx_body * cy - dy_body * sy;
    move_y += dx_body * sy + dy_body * cy;
    move_sync_to_odom();
}

static void move_cd_fixed_hold(float vx_body, float vy_body, float wz_body,
                               float yaw_l_sum, uint32_t duration_ms)
{
    uint32_t end_tick;
    uint32_t last_tick;

    Move_SetRobotVelocity(vx_body, vy_body, wz_body);
    last_tick = move_tick();
    end_tick = last_tick + duration_ms;

    for (;;) {
        uint32_t now_tick;
        uint32_t step_ms;

        if ((int32_t)(end_tick - last_tick) <= 0) {
            break;
        }

        step_ms = MOVE_CD_FIXED_LOOP_MS;
        if (step_ms > (uint32_t)(end_tick - last_tick)) {
            step_ms = (uint32_t)(end_tick - last_tick);
        }
        move_delay(step_ms);

        now_tick = move_tick();
        if ((int32_t)(now_tick - end_tick) > 0) {
            now_tick = end_tick;
        }
        if ((int32_t)(now_tick - last_tick) > 0) {
            move_cd_fixed_apply_odom(vx_body, vy_body, wz_body,
                                     yaw_l_sum, now_tick - last_tick);
            last_tick = now_tick;
        }

        if ((int32_t)(end_tick - last_tick) <= 0) {
            break;
        }
        Move_SetRobotVelocity(vx_body, vy_body, wz_body);
    }
}

uint8_t MoveCDFixedArcTrack(void)
{
#if MOVE_CD_FIXED_ARC_ENABLE
    g_move_active = 1;
    enc_has_last = false;
    Move_UpdateYawFeedback(0.0f);
    move_target_yaw = move_yaw;

    /* 第一段: 从 -0.662,0.25,-90° 连贯过渡到 -0.9,0.25,-69°。
     * 第二段: 直接接固定圆弧速度表, 不再停下来切换GOTO/TURNTO/ARC。 */
    arc_yaw_l_sum_active = 0;
    move_cd_fixed_hold(MOVE_CD_FIXED_PRE_VX,
                       MOVE_CD_FIXED_PRE_VY,
                       MOVE_CD_FIXED_PRE_WZ,
                       MOVE_YAW_L_SUM,
                       MOVE_CD_FIXED_PRE_MS);

#if MOVE_CD_FIXED_TRACK_ARC
    /* 固定过渡段结束后, 将里程计同步到理论圆弧起点。
     * 这段只用于 C/D 写死流程, 目的不是通用定位, 而是让后续闭环圆弧从稳定几何起点计算圆心。 */
    Move_InitPose(MOVE_CD_FIXED_START_X,
                  MOVE_CD_FIXED_START_Y,
                  MOVE_CD_FIXED_START_YAW);
    {
        uint32_t arc_to = (uint32_t)((MOVE_CD_FIXED_ARC_SWEEP_DEG * 3.14159265f / 180.0f)
                          * MOVE_CD_FIXED_ARC_RADIUS / MOVE_CD_FIXED_ARC_VY
                          * 1000.0f * 2.5f) + 15000UL;
        uint8_t ok = MoveArcTrack(MOVE_CD_FIXED_ARC_RADIUS,
                                  MOVE_CD_FIXED_ARC_VY,
                                  MOVE_CD_FIXED_ARC_DIR,
                                  MOVE_CD_FIXED_ARC_SWEEP_DEG,
                                  arc_to);
        arc_yaw_l_sum_active = 0;
        g_move_active = 0;
        return ok;
    }
#else
    arc_yaw_l_sum_active = 1;
    move_cd_fixed_hold(0.0f,
                       MOVE_CD_FIXED_ARC_VY,
                       MOVE_CD_FIXED_ARC_WZ,
                       MOVE_ARC_YAW_L_SUM,
                       MOVE_CD_FIXED_ARC_MS);

    Move_Stop();
    move_delay(80);
    arc_yaw_l_sum_active = 0;
    move_sync_to_odom();
    g_move_active = 0;
    return 1;
#endif
#else
    return 0;
#endif
}

/* ================================================================
 *  同步: move_* → g_odom_* (控制环结束时调用)
 * ================================================================ */
static void move_sync_to_odom(void)
{
    Move_UpdateYawFeedback(0.0f);

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
 *   4. Yaw保持: yaw反馈误差 * YAW_KP → wz
 *   5. 发送4轮速度
 *   6. 检查到位/超时
 *
 * @return 1=到位, 0=超时
 */
static uint8_t move_to_accurate_timed_ex(float tx, float ty, float max_speed,
                                         float tol, uint32_t timeout_ms,
                                         float decel_dist,
                                         float min_speed, float creep_speed)
{
    g_move_active = 1;
    Move_UpdateYawFeedback(0.0f);

    /* 锁定目标航向 (外部CW+ = move_yaw, IMU优先) */
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
            move_delay(MOVE_STOP_SETTLE_MS);
            move_refresh_odom_once();

            float final_dx = tx - move_x;
            float final_dy = ty - move_y;
            float final_dist = move_sqrt(final_dx * final_dx + final_dy * final_dy);
            if (final_dist <= tol) {
                g_move_active = 0;
                return 1;
            }

            /* 停车后滑出容差, 继续低速拉回；重置速度估计避免D项沿用停车前速度。 */
            prev_dist = final_dist;
            prev_tick = move_tick();
            v_approach_lp = 0.0f;
            enc_has_last = false;
            continue;
        }

        /* 4. PD控制: P + 超速阻尼 (D只在电机实际速度>P命令时介入) */
        float p_speed = dist * MOVE_POS_KP;
        float excess = v_approach_lp - p_speed;   /* >0 = 电机比命令快(滞后) */
        float speed = p_speed;
        if (excess > 0.0f) {
            speed -= MOVE_POS_KD * excess;         /* 只削超速部分 */
        }
        if (speed < min_speed) speed = min_speed;
        if (speed > eff_max_speed) speed = eff_max_speed;

        /* 4b. 减速区: sqrt制动曲线 (第一性原理: d=v²/2a → v=√(2ad))
         *    线性减速初始减速度=2×sqrt, 电机跟不上→过冲;
         *    sqrt曲线初始减速度温和, 接近目标时自然降速, 匹配电机物理 */
        if (decel_dist > 0.001f && dist < decel_dist) {
            float ratio = dist / decel_dist;
            float decel = eff_max_speed * move_sqrt(ratio);
            if (decel < min_speed) decel = min_speed;
            if (speed > decel) speed = decel;
        }

        /* 4c. 蠕变区: 线性渐变限速 (边界=creep_speed, 目标=min_speed)
         *    消除硬限速台阶跳变, 电机无需瞬间大幅减速→无残余震荡 */
        if (dist < MOVE_CREEP_DIST) {
            float ratio = dist / MOVE_CREEP_DIST;   /* 1(边界) → 0(目标) */
            float creep_cap = min_speed + (creep_speed - min_speed) * ratio;
            if (speed > creep_cap) speed = creep_cap;
        }

        /* 5. 场坐标系分解 (+X=右, +Y=前) */
        float vx_f = (dx / dist) * speed;   /* 右移分量 */
        float vy_f = (dy / dist) * speed;   /* 前进分量 */

        /* 6. Yaw保持: current-target为正表示已偏CW, 需要输出CCW修正。
         *    死区内不修正, 防IMU噪声/机械微抖导致直线摇摆 */
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

/**
 * @brief  点到点移动同时把yaw平滑拉到目标角。
 *
 * 用于圆弧前过渡段: 平移仍复用MoveTo的距离P/减速/蠕变逻辑；
 * yaw目标按路径进度线性推进，并在路径前 MOVE_GOTO_YAW_DONE_RATIO 完成，
 * 让车体到圆弧起点前已基本对准切线方向。
 */
static uint8_t move_to_yaw_timed_ex(float tx, float ty, float target_yaw_deg,
                                    float max_speed, float tol,
                                    uint32_t timeout_ms, float decel_dist,
                                    float min_speed, float creep_speed,
                                    uint8_t stop_on_done)
{
    g_move_active = 1;
    Move_UpdateYawFeedback(0.0f);

    float start_x = move_x;
    float start_y = move_y;
    float start_yaw = move_yaw;
    float yaw_delta = move_norm_deg180(target_yaw_deg - start_yaw);
    float total_dx = tx - start_x;
    float total_dy = ty - start_y;
    float total_dist = move_sqrt(total_dx * total_dx + total_dy * total_dy);

    move_target_yaw = target_yaw_deg;
    enc_has_last = false;

    uint32_t t0 = move_tick();
    float prev_dist = -1.0f;
    uint32_t prev_tick = t0;
    float v_approach_lp = 0.0f;

    for (;;) {
        if (move_tick() - t0 >= timeout_ms) {
            Move_Stop();
            move_sync_to_odom();
            g_move_active = 0;
            return 0;
        }

        int32_t cur_pos[4];
        dbg_enc_ok = move_read_all_encoders(cur_pos);
        move_update_odom(cur_pos);

        float dx = tx - move_x;
        float dy = ty - move_y;
        float dist = move_sqrt(dx * dx + dy * dy);

        float progress = 1.0f;
        if (total_dist > 0.001f) {
            progress = 1.0f - (dist / total_dist);
            if (progress < 0.0f) progress = 0.0f;
            if (progress > 1.0f) progress = 1.0f;
        }
        float yaw_progress = progress / MOVE_GOTO_YAW_DONE_RATIO;
        if (yaw_progress > 1.0f) yaw_progress = 1.0f;
        float yaw_target_now = start_yaw + yaw_delta * yaw_progress;

        float final_yaw_err = move_norm_deg180(target_yaw_deg - move_yaw);
        if (dist <= tol && move_abs(final_yaw_err) <= MOVE_GOTO_YAW_ACCEPT_DEG) {
            if (stop_on_done) {
                Move_Stop();
            }
            move_sync_to_odom();
            g_move_active = 0;
            return 1;
        }

        float vx_f = 0.0f;
        float vy_f = 0.0f;

        if (dist > tol) {
            float abs_dx = move_abs(dx);
            float abs_dy = move_abs(dy);
            float max_comp = (abs_dx > abs_dy) ? abs_dx : abs_dy;
            float diag_scale = (dist > 0.001f) ? (max_comp / dist) : 1.0f;
            if (diag_scale > 1.0f) diag_scale = 1.0f;
            float eff_max_speed = max_speed * diag_scale * diag_scale;
            if (diag_scale < 0.99f) eff_max_speed *= 0.8f;

            float ramp = (float)(move_tick() - t0) / (float)MOVE_RAMP_TIME_MS;
            if (ramp > 1.0f) ramp = 1.0f;
            eff_max_speed *= ramp;

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

            float p_speed = dist * MOVE_POS_KP;
            float excess = v_approach_lp - p_speed;
            float speed = p_speed;
            if (excess > 0.0f) {
                speed -= MOVE_POS_KD * excess;
            }
            if (speed < min_speed) speed = min_speed;
            if (speed > eff_max_speed) speed = eff_max_speed;

            if (decel_dist > 0.001f && dist < decel_dist) {
                float ratio = dist / decel_dist;
                float decel = eff_max_speed * move_sqrt(ratio);
                if (decel < min_speed) decel = min_speed;
                if (speed > decel) speed = decel;
            }

            if (dist < MOVE_CREEP_DIST) {
                float ratio = dist / MOVE_CREEP_DIST;
                float creep_cap = min_speed + (creep_speed - min_speed) * ratio;
                if (speed > creep_cap) speed = creep_cap;
            }

            vx_f = (dx / dist) * speed;
            vy_f = (dy / dist) * speed;
        } else {
            /* 位置已到但角度还差一点时，只保留yaw修正，避免末端绕点画圈。 */
            yaw_target_now = target_yaw_deg;
        }

        float yaw_err = move_norm_deg180(move_yaw - yaw_target_now);
        float wz = 0.0f;
        if (move_abs(yaw_err) > MOVE_YAW_HOLD_DEADZONE) {
            wz = yaw_err * MOVE_WP_YAW_KP;
            wz = move_clamp(wz, -MOVE_WP_YAW_MAX, MOVE_WP_YAW_MAX);
        }

        Move_SetFieldVelocity(vx_f, vy_f, -wz);
        move_sync_to_odom();
        move_delay(MOVE_CTRL_PERIOD_MS);
    }
}

uint8_t MoveToAccurateTimed(float tx, float ty, float max_speed,
                            float tol, uint32_t timeout_ms)
{
    return move_to_accurate_timed_ex(tx, ty, max_speed, tol,
                                     timeout_ms, MOVE_DECEL_DIST,
                                     MOVE_MIN_SPEED, MOVE_CREEP_SPEED);
}

uint8_t MoveTo(float tx, float ty, float max_speed)
{
    return MoveToAccurateTimed(tx, ty, max_speed,
                               MOVE_DEFAULT_TOL, MOVE_DEFAULT_TIMEOUT_MS);
}

uint8_t MoveToYawTimed(float tx, float ty, float target_yaw_deg,
                       float max_speed, uint32_t timeout_ms,
                       uint8_t stop_on_done)
{
    return move_to_yaw_timed_ex(tx, ty, target_yaw_deg, max_speed,
                                MOVE_DEFAULT_TOL, timeout_ms,
                                MOVE_WP_MOVE_DECEL_DIST,
                                MOVE_MIN_SPEED, MOVE_CREEP_SPEED,
                                stop_on_done);
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
#if MOVE_YAW_USE_IMU
    g_move_active = 1;
    Move_UpdateYawFeedback(0.0f);
    move_target_yaw = target_yaw_deg;
    enc_has_last = false;

    uint32_t t0 = move_tick();

    uint8_t settle_cnt = 0;         /* 停稳后连续满足容差的计数 */
    uint8_t stopped = 0;            /* 当前是否已经停电机等待滑行稳定 */
    uint8_t had_stopped = 0;        /* 本次命令是否已经触发过预测停止 */
    uint32_t stopped_tick = t0;     /* 最近一次停电机时刻 */
    uint32_t settle_last_tick = t0; /* 上次计入settle的控制周期 */

    /* IMU分支保留预测停止，但不能预测到位后立刻返回。
     * 必须先停稳复检；如果滑行后误差仍大，本次命令内小速度拉回。 */
    {
        float init_err = move_norm_deg180(target_yaw_deg - move_yaw);
        if (move_abs(init_err) <= MOVE_IMU_ACCEPT_DEG) {
            Move_Stop();
            stopped = 1;
            had_stopped = 1;
            stopped_tick = move_tick();
            settle_last_tick = stopped_tick;
        }
    }

    float prev_yaw_fb = move_yaw;   /* CW正, 来源由Move_UpdateYawFeedback统一处理 */
    uint32_t prev_yaw_tick = t0;
    float yaw_rate_lp = 0.0f;

    for (;;) {
        if (move_tick() - t0 >= timeout_ms) {
            Move_Stop();
            move_sync_to_odom();
            g_move_active = 0;
            return 0;
        }

        /* 读编码器用于XY里程计；IMU有效时yaw取IMU, IMU掉线时自动退回编码器增量。 */
        int32_t cur_pos[4];
        move_read_all_encoders(cur_pos);
        move_update_odom(cur_pos);

        /* 内部统一CW正: 目标-当前为正时输出右转(CW)速度。 */
        float err = move_norm_deg180(target_yaw_deg - move_yaw);

        uint32_t now_yaw_tick = move_tick();
        float dt_yaw = (now_yaw_tick - prev_yaw_tick) * 0.001f;
        if (dt_yaw < 0.001f) dt_yaw = 0.001f;
        float dyaw = move_norm_deg180(move_yaw - prev_yaw_fb);
        float yaw_rate = dyaw / dt_yaw;
        yaw_rate_lp = 0.7f * yaw_rate_lp + 0.3f * yaw_rate;
        prev_yaw_fb = move_yaw;
        prev_yaw_tick = now_yaw_tick;

        float abs_err = move_abs(err);
        float stop_distance = move_abs(yaw_rate_lp) * MOVE_IMU_STOP_LATENCY_S;
        uint8_t should_stop = (abs_err <= MOVE_IMU_ROTATE_TOL_DEG + stop_distance) ? 1u : 0u;
        uint8_t acceptable_after_stop = (had_stopped && abs_err <= MOVE_IMU_ACCEPT_DEG) ? 1u : 0u;

        if (should_stop && !stopped) {
            Move_Stop();
            stopped = 1;
            had_stopped = 1;
            stopped_tick = move_tick();
            settle_last_tick = stopped_tick;
            settle_cnt = 0;
        }

        if (stopped) {
            uint8_t wait_ok = ((move_tick() - stopped_tick) >= MOVE_YAW_SETTLE_WAIT_MS) ? 1u : 0u;

            if (!should_stop && !acceptable_after_stop) {
                /* 停稳后偏差仍大: 继续在本次命令内拉回, 不交给上位机重复发送。 */
                stopped = 0;
                settle_cnt = 0;
            } else if (wait_ok && ((move_tick() - settle_last_tick) >= MOVE_CTRL_PERIOD_MS)) {
                settle_last_tick = move_tick();
                if ((abs_err <= MOVE_IMU_ROTATE_TOL_DEG) || acceptable_after_stop) {
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

        if (!stopped) {
            float wz = err * MOVE_YAW_KP;
            float min_spd = (abs_err < 5.0f) ? MOVE_YAW_FINE_SPEED : MOVE_MIN_SPEED;
            if (wz > 0.0f && wz < min_spd)  wz =  min_spd;
            if (wz < 0.0f && wz > -min_spd) wz = -min_spd;

            /* 首停后只允许小速度拉回，避免IMU噪声触发大幅反向修正。 */
            float effective_max = (had_stopped && abs_err < 10.0f) ? MOVE_YAW_FINE_SPEED : max_speed;
            wz = move_clamp(wz, -effective_max, effective_max);

            if (abs_err < MOVE_YAW_DECEL_DEG) {
                float ratio = abs_err / MOVE_YAW_DECEL_DEG;
                float decel = effective_max * move_sqrt(ratio);
                if (decel < MOVE_MIN_SPEED) decel = MOVE_MIN_SPEED;
                if (move_abs(wz) > decel) {
                    wz = (wz >= 0.0f) ? decel : -decel;
                }
            }

            Move_SetRobotVelocity(0.0f, 0.0f, wz);
        }

        move_sync_to_odom();
        move_delay(MOVE_CTRL_PERIOD_MS);
    }
#else
    g_move_active = 1;
    Move_UpdateYawFeedback(0.0f);
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

    /* 追踪反馈角速度 (用于预测性停止) */
    float prev_yaw_fb = move_yaw;   /* 上一帧yaw反馈 (CW正) */
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

        /* 计算反馈角速度 (°/s), 用于预测性停止 */
        uint32_t now_yaw_tick = move_tick();
        float dt_yaw = (now_yaw_tick - prev_yaw_tick) * 0.001f;
        if (dt_yaw < 0.001f) dt_yaw = 0.001f;
        float dyaw = move_yaw - prev_yaw_fb;
        /* 归一化dyaw避免360°跳变 */
        while (dyaw >  180.0f) dyaw -= 360.0f;
        while (dyaw < -180.0f) dyaw += 360.0f;
        float yaw_rate = dyaw / dt_yaw;
        /* 低通滤波: 抑制反馈读数抖动 */
        yaw_rate_lp = 0.7f * yaw_rate_lp + 0.3f * yaw_rate;
        prev_yaw_fb = move_yaw;
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
#endif
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
    Move_UpdateYawFeedback(0.0f);
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
    Move_UpdateYawFeedback(0.0f);

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

    uint8_t tt_enable = arc_tt_enable;
    float tt_deg[3] = {arc_tt_deg[0], arc_tt_deg[1], arc_tt_deg[2]};
    uint8_t tt_slot[3] = {arc_tt_slot[0], arc_tt_slot[1], arc_tt_slot[2]};
    uint8_t tt_done[3] = {0, 0, 0};

    g_move_active = 1;
    arc_yaw_l_sum_active = 1;
    enc_has_last = false;
    Move_UpdateYawFeedback(0.0f);

    float start_yaw = move_yaw;             /* CW正, 圆弧起点使用当前yaw反馈 */
    float cx = move_x + (float)dir * radius * move_cos(start_yaw);
    float cy = move_y - (float)dir * radius * move_sin(start_yaw);

    /* 初始极角 (标准 atan2, CCW+ 从+X) */
    float dx0 = move_x - cx, dy0 = move_y - cy;
    float prev_angle = move_atan2(dy0, dx0);

    float swept = 0.0f;       /* 编码器位置估算 (切线/径向/航向目标用) */
    float swept_yaw = 0.0f;   /* yaw反馈扫角估算 (减速/停止用) */
    float prev_yaw = start_yaw;  /* CW+ 累积用 (当前yaw反馈) */
    float yaw_rate_lp = 0.0f;  /* yaw反馈角速度 °/s (给停止预测用) */
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

        /* yaw反馈累计 (用于减速和停止判断) */
        float cur_yaw_raw = move_yaw;
        float yaw_delta = cur_yaw_raw - prev_yaw;
        while (yaw_delta >  180.0f) yaw_delta -= 360.0f;
        while (yaw_delta < -180.0f) yaw_delta += 360.0f;
        swept_yaw += move_abs(yaw_delta);
        prev_yaw = cur_yaw_raw;

        /* yaw反馈角速度 (停止预测用, 非命令速度) */
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
        if (tt_enable) {
            for (uint8_t i = 0; i < 3; i++) {
                if (!tt_done[i] && tt_deg[i] > 0.0f && tt_deg[i] <= sweep_deg &&
                    arc_progress >= tt_deg[i]) {
                    move_arc_request_turntable(tt_slot[i]);
                    tt_done[i] = 1;
                }
            }
        }
        float remaining_deg = sweep_deg - arc_progress;
        float s_rem = (remaining_deg > 0.0f) ?
                      (remaining_deg * (3.14159265f / 180.0f) * radius) : 0.0f;
        float vf_profile = move_sqrt(2.0f * MOVE_ARC_ACCEL * s_rem);
        if (vf_profile < MOVE_ARC_CREEP_SPEED) vf_profile = MOVE_ARC_CREEP_SPEED;
        if (vf_profile < vf) vf = vf_profile;

        /* 完成检查 + 停止预测: yaw预测到位且位置扫角也走够, 才进入settle */
        float predicted_overshoot = yaw_rate_lp * ((float)MOVE_ARC_STOP_LATENCY_MS / 1000.0f);
        uint8_t yaw_done = (swept_yaw + predicted_overshoot >= sweep_deg);
        uint8_t pos_sweep_done = (swept + MOVE_ARC_SWEEP_TOL_DEG >= sweep_deg);
        if (yaw_done && pos_sweep_done) {
            if (tt_enable) {
                for (uint8_t i = 0; i < 3; i++) {
                    if (!tt_done[i] && tt_deg[i] > 0.0f && tt_deg[i] <= sweep_deg) {
                        move_arc_request_turntable(tt_slot[i]);
                        tt_done[i] = 1;
                    }
                }
            }
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
                /* settle 阶段继续使用同一yaw反馈源, 与弧线主循环一致 */

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

            /* 圆弧结束后保持当前yaw反馈源, 后续普通段继续同一来源 */
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

uint8_t MoveArcTrackWithTurntable(float radius, float speed, int dir,
                                  float sweep_deg,
                                  float trigger1_deg, uint8_t slot1,
                                  float trigger2_deg, uint8_t slot2,
                                  float trigger3_deg, uint8_t slot3,
                                  uint32_t timeout_ms)
{
    arc_tt_deg[0] = trigger1_deg;
    arc_tt_deg[1] = trigger2_deg;
    arc_tt_deg[2] = trigger3_deg;
    arc_tt_slot[0] = slot1;
    arc_tt_slot[1] = slot2;
    arc_tt_slot[2] = slot3;
    arc_tt_enable = 1;
    uint8_t ret = MoveArcTrack(radius, speed, dir, sweep_deg, timeout_ms);
    arc_tt_enable = 0;
    return ret;
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

static float path_dist2(const PathPt_t *a, const PathPt_t *b)
{
    float dx = b->x - a->x;
    float dy = b->y - a->y;
    return dx * dx + dy * dy;
}

/**
 * @brief  路径执行: MCU内部顺序执行位置点/转角点
 *
 * 这版优先稳定性: 不再做投影追线和关键点反向拉回, 避免短线段过点后后退震荡。
 * 坐标不同: 复用已验证的 MoveToAccurateTimed; 坐标相同且mode=KEY: 复用RotateTo。
 *
 * @return 1=完成, 0=超时/装载无效
 */
uint8_t MovePathTrack(void)
{
    int   n     = (int)g_path_count;
    float speed = g_path_speed;

    if (g_path_load_error || n < 2 || n != (int)g_path_expected) return 0;

    float total_len = 0.0f;
    uint8_t zero_turn_count = 0;
    for (int i = 0; i < n - 1; i++) {
        float dx = g_path_pts[i+1].x - g_path_pts[i].x;
        float dy = g_path_pts[i+1].y - g_path_pts[i].y;
        float seg_len2 = dx * dx + dy * dy;
        if (seg_len2 < 1e-8f && g_path_pts[i+1].mode == PATH_MODE_KEY) {
            zero_turn_count++;
        } else {
            total_len += move_sqrt(seg_len2);
        }
    }

    uint32_t path_timeout_ms = MOVE_WP_TIMEOUT_MS;
    if (speed > 0.01f && total_len > 0.001f) {
        uint32_t dyn_to = (uint32_t)((total_len / speed) * 5000.0f)
                        + (uint32_t)zero_turn_count * 8000UL + 12000UL;
        if (dyn_to > path_timeout_ms) path_timeout_ms = dyn_to;
    }
    uint32_t t0 = move_tick();

    for (int i = 1; i < n; i++) {
        if (move_tick() - t0 >= path_timeout_ms) {
            Move_Stop();
            move_sync_to_odom();
            g_move_active = 0;
            return 0;
        }

        if (path_dist2(&g_path_pts[i-1], &g_path_pts[i]) < 1e-8f) {
            if (g_path_pts[i].mode == PATH_MODE_KEY) {
                if (!RotateTo(g_path_pts[i].target_theta, MOVE_WP_ZERO_YAW_LIMIT)) {
                    Move_Stop();
                    move_sync_to_odom();
                    g_move_active = 0;
                    return 0;
                }
            }
            continue;
        }

        float seg_timeout_s = 8.0f;
        float seg_len = move_sqrt(path_dist2(&g_path_pts[i-1], &g_path_pts[i]));
        if (speed > 0.01f) {
            seg_timeout_s += (seg_len / speed) * 4.0f;
        }
        uint32_t seg_timeout_ms = (uint32_t)(seg_timeout_s * 1000.0f);

        if (!move_to_accurate_timed_ex(g_path_pts[i].x, g_path_pts[i].y,
                                       speed, MOVE_WP_END_TOL,
                                       seg_timeout_ms, MOVE_WP_MOVE_DECEL_DIST,
                                       MOVE_MIN_SPEED, MOVE_CREEP_SPEED)) {
            Move_Stop();
            move_sync_to_odom();
            g_move_active = 0;
            return 0;
        }
    }

    g_move_active = 0;
    return 1;
}
