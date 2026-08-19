#ifndef __IMU_PROTOCOL_H
#define __IMU_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

/* ============================================================
 * IMU 串口协议解析层 (独立于硬件, 纯软件)
 *
 * 帧格式 (基于商家例程, 0x7E 0x23 头):
 *   [0x7E][0x23][长度][功能码][数据 N 字节][校验和]
 *   长度 = 4 + N + 1 (含头 + 数据 + 校验)
 *   校验和 = (0x7E + 0x23 + 长度 + 功能码 + 数据) & 0xFF
 *
 * IMU 自动上报, 默认 25Hz
 * yaw 上电归零 (相对开机朝向)
 * yaw 单位待实测 (假设弧度, 见 CLAUDE.md)
 * ============================================================ */

#define IMU_FRAME_HEAD1 0x7E
#define IMU_FRAME_HEAD2 0x23

#define IMU_FUNC_VERSION      0x01
#define IMU_FUNC_RAW_ACCEL    0x04
#define IMU_FUNC_RAW_GYRO     0x0A
#define IMU_FUNC_RAW_MAG      0x10
#define IMU_FUNC_QUAT         0x16
#define IMU_FUNC_EULER        0x26    /* roll, pitch, yaw (float × 3) */
#define IMU_FUNC_BARO         0x32
#define IMU_FUNC_RETURN_STATE 0x81

/* Command function codes (MCU -> IMU) */
#define IMU_FUNC_SET_OUTPUT_FREQ 0x60   /* param: freq (10-100 Hz) */
#define IMU_FUNC_SET_ALGO_TYPE   0x61   /* param: 0x06=6-axis, 0x09=9-axis, + 0x5F */
#define IMU_FUNC_CALIB_IMU       0x70   /* param: 0x01, 0x5F; IMU must be stationary */

#define IMU_RX_BUF_SIZE 256

/* ISR 调用: 推 1 字节到环形缓冲 */
void imu_protocol_push_byte(uint8_t b);

/* 任务调用: 解析环形缓冲中的完整帧 */
void imu_protocol_process(void);

/* 取最新 yaw (单位待实测: 弧度 or 角度) */
bool imu_protocol_get_yaw(float *out);

/* Reset yaw zeroing state and discard pending RX bytes.
 * Use after IMU internal calibration so the next fresh Euler frame becomes 0 deg. */
void imu_protocol_reset_yaw_zero(void);

/* 调试变量 (Keil 在线调试器看) */
extern volatile uint8_t  imu_last_func;          /* 最近一帧的功能码 */
extern volatile uint32_t imu_frame_count;        /* 校验通过的帧计数 */
extern volatile uint8_t  imu_last_checksum_ok;    /* 最近一帧校验结果 */
extern volatile uint32_t imu_rx_byte_count;      /* 收到的总字节数 (调试用) */
extern volatile uint32_t imu_yaw_frame_count;    /* 校验通过的Euler yaw帧计数 */
extern volatile float    imu_raw_yaw;            /* 原始 yaw (转换前, 确认单位用) */
extern volatile uint32_t imu_return_state_count; /* IMU 命令完成/状态返回帧计数 */

#endif /* __IMU_PROTOCOL_H */
