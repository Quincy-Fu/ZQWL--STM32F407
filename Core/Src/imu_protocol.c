#include "imu_protocol.h"
#include <string.h>

/* ============================================================
 * IMU 协议解析 (基于商家例程逻辑, 不依赖硬件)
 * ============================================================ */

/* 环形缓冲 */
static volatile uint8_t  s_rx_buf[IMU_RX_BUF_SIZE];
static volatile uint16_t s_rx_write = 0;
static volatile uint16_t s_rx_read  = 0;

/* 解析状态机 */
enum {
    RX_HEAD1 = 0,
    RX_HEAD2,
    RX_LEN,
    RX_FUNC,
    RX_DATA
};

/* 内部缓存: 只存 yaw (用户当前只需要这个) */
static volatile float s_yaw = 0.0f;

/* 调试变量 */
volatile uint8_t  imu_last_func         = 0;
volatile uint32_t imu_frame_count       = 0;
volatile uint8_t  imu_last_checksum_ok  = 0;
volatile uint32_t imu_rx_byte_count     = 0;   // total bytes received (debug)
volatile float    imu_raw_yaw           = 0.0f; // raw yaw before conversion (debug)

/* ---- 环形缓冲接口 ---- */

void imu_protocol_push_byte(uint8_t b)
{
    imu_rx_byte_count++;
    s_rx_buf[s_rx_write] = b;
    uint16_t next = (uint16_t)((s_rx_write + 1) % IMU_RX_BUF_SIZE);
    /* 缓冲满时丢弃最旧字节 */
    if (next == s_rx_read) {
        s_rx_read = (uint16_t)((s_rx_read + 1) % IMU_RX_BUF_SIZE);
    }
    s_rx_write = next;
}

static int s_rx_pop(uint8_t *out)
{
    if (s_rx_read == s_rx_write) return -1;
    *out = s_rx_buf[s_rx_read];
    s_rx_read = (uint16_t)((s_rx_read + 1) % IMU_RX_BUF_SIZE);
    return 0;
}

/* 4 字节小端 → float (IMU 上报数据是小端) */
static float to_float(const uint8_t *b)
{
    float v;
    memcpy(&v, b, sizeof(float));
    return v;
}

/* ---- 帧数据解析 ---- */
static void parse_frame(uint8_t func, const uint8_t *data, uint8_t data_len)
{
    switch (func) {
        case IMU_FUNC_EULER:
            /* roll @ data[0..3], pitch @ data[4..7], yaw @ data[8..11]
             * IMU 上报弧度, 转成角度存 (g_imu_yaw 单位度, 更直观) */
            if (data_len >= 12) {
                float raw = to_float(&data[8]);
                imu_raw_yaw = raw;  /* debug: check if ~1.57 for 90deg (radians) or ~90 (degrees) */
                s_yaw = raw * 57.2957795f;
            }
            break;
        /* 其他功能码暂不解析 (用户只要 yaw, YAGNI) */
        default:
            break;
    }
}

/* ---- 主解析循环 (任务调用) ---- */
void imu_protocol_process(void)
{
    static uint8_t state = RX_HEAD1;
    static uint8_t frame_len = 0;
    static uint8_t frame_func = 0;
    static uint8_t frame_buf[64];
    static uint8_t frame_idx = 0;

    uint8_t b;
    while (s_rx_pop(&b) == 0) {
        switch (state) {
            case RX_HEAD1:
                state = (b == IMU_FRAME_HEAD1) ? RX_HEAD2 : RX_HEAD1;
                break;
            case RX_HEAD2:
                state = (b == IMU_FRAME_HEAD2) ? RX_LEN : RX_HEAD1;
                break;
            case RX_LEN:
                frame_len = b;
                state = RX_FUNC;
                break;
            case RX_FUNC:
                frame_func = b;
                frame_idx = 0;
                state = RX_DATA;
                break;
            case RX_DATA: {
                /* 数据长度 = frame_len - 4(头2+len+func) - 1(校验) = frame_len - 5
                 * 但例程算法是 data_length = frame_len - 4, 最后 1 字节是校验 */
                uint8_t data_len = (frame_len >= 4) ? (uint8_t)(frame_len - 4) : 0;
                if (data_len == 0 || data_len > sizeof(frame_buf)) {
                    state = RX_HEAD1;
                    break;
                }
                frame_buf[frame_idx++] = b;
                if (frame_idx >= data_len) {
                    /* 校验: 前面所有字节累加 (除校验本身) */
                    uint8_t calc = (uint8_t)(IMU_FRAME_HEAD1 + IMU_FRAME_HEAD2
                                             + frame_len + frame_func);
                    for (uint8_t i = 0; i < data_len - 1; i++) {
                        calc = (uint8_t)(calc + frame_buf[i]);
                    }
                    uint8_t recv = frame_buf[data_len - 1];

                    imu_last_func = frame_func;
                    if (calc == recv) {
                        parse_frame(frame_func, frame_buf, (uint8_t)(data_len - 1));
                        imu_last_checksum_ok = 1;
                        imu_frame_count++;
                    } else {
                        imu_last_checksum_ok = 0;
                    }
                    state = RX_HEAD1;
                }
            } break;
            default:
                state = RX_HEAD1;
                break;
        }
    }
}

bool imu_protocol_get_yaw(float *out)
{
    if (!out) return false;
    *out = s_yaw;
    return true;
}
