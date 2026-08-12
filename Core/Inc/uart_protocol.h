#ifndef UART_PROTOCOL_H
#define UART_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

// ????
#define PROTOCOL_HEADER1         0xAA
#define PROTOCOL_HEADER2         0x55

// ????(? serial.py ????)
#define TYPE_CMD_VEL             0x01
#define TYPE_POSE                0x02
#define TYPE_ROTATE              0x03
#define TYPE_ARM                 0x04
#define TYPE_LIGHT               0x05
#define TYPE_RUN                 0x06   // PC->MCU: 启动(模拟按键PD15), payload 空
#define TYPE_RUN_RESP            0x07   // MCU->PC: 启动响应, payload 1B status
#define TYPE_ROTATE_RESP         0x08   // MCU->PC: 转盘响应(估算移动时间后), payload 1B status
#define TYPE_ARM_RESP            0x09   // MCU->PC: 机械臂响应, payload 1B status
#define TYPE_LIGHT_RESP          0x0A   // MCU->PC: 补光灯响应, payload 1B status

// Navigation command types (Stage 3)
// Even = command (PC->MCU), Odd = response (MCU->PC)
#define TYPE_CMD_GOTO            0x10
#define TYPE_CMD_GOTO_RESP       0x11
#define TYPE_CMD_TOX             0x12
#define TYPE_CMD_TOX_RESP        0x13
#define TYPE_CMD_TOY             0x14
#define TYPE_CMD_TOY_RESP        0x15
#define TYPE_CMD_TURNTO          0x16
#define TYPE_CMD_TURNTO_RESP     0x17
#define TYPE_CMD_FINE_MOVE       0x18
#define TYPE_CMD_FINE_RESP       0x19
#define TYPE_CMD_SYNC_POSE       0x1A
#define TYPE_CMD_SYNC_RESP       0x1B
#define TYPE_CMD_ARC             0x1C
#define TYPE_CMD_ARC_RESP        0x1D
#define TYPE_CMD_CALIB_HEIGHT    0x1E
#define TYPE_CMD_CALIB_HEIGHT_RESP 0x1F
#define TYPE_CMD_CALIB_OFFSET   0x20
#define TYPE_CMD_CALIB_OFFSET_RESP 0x21
// 路径跟踪 (线段投影+横向修正+关键点航向)
#define TYPE_CMD_PATH_BEGIN     0x22   // payload: speed(f32)+count(u8)
#define TYPE_CMD_PATH_POINT     0x23   // payload: x(f32)+y(f32)+target_theta(f32)+mode(u8)+pad(3B) = 16B
#define TYPE_CMD_PATH_EXEC      0x24   // payload: 空, 触发执行
#define TYPE_CMD_PATH_RESP      0x25   // payload: result(u8) 1=完成 0=超时/中止
#define TYPE_CMD_PATH_DEBUG     0x26   // [调试用,定位后删除] MCU→PC: move_x/y+wp_idx+total+vx_f/vy_f+wz+target_yaw

// 视觉微调 (Stage 4: 到位后视觉闭环方向微调)
#define TYPE_CMD_VISION_NUDGE        0x27   // PC->MCU: payload 1B direction (0=stop+lock, 1=fwd, 2=back, 3=left, 4=right)
#define TYPE_CMD_VISION_NUDGE_RESP   0x28   // MCU->PC: payload 1B status (1=executed)

// 转盘零点设置 (一次性标定: 将当前位置存为零点并写入 flash)
#define TYPE_CMD_SET_ZERO        0x29   // PC->MCU: payload 空, 调用 Emm_V5_Origin_Set_O(svF=true)
#define TYPE_CMD_SET_ZERO_RESP   0x2A   // MCU->PC: payload 1B status (1=saved)

// 视觉校正 (Stage 5: 弧后视觉闭环, fine_move + sync_pose 原子组合)
#define TYPE_CMD_VISION_CORRECT        0x2B   // PC->MCU: 16B = field dx_mm + field dy_mm + target_x_m + target_y_m
#define TYPE_CMD_VISION_CORRECT_RESP   0x2C   // MCU->PC: 1B status (1=arrived and odom synced, 0=move failed)

// IMU 零偏校准 (车必须静止; MCU 转发 IMU 0x70 校准命令并等待完成)
#define TYPE_CMD_IMU_CALIB             0x2D   // PC->MCU: payload 空
#define TYPE_CMD_IMU_CALIB_RESP        0x2E   // MCU->PC: payload 1B status (1=IMU帧恢复)

// ????payload?? (3?float)
#define PAYLOAD_SIZE_VEL         12
// ???payload?? (3?float)
#define PAYLOAD_SIZE_POSE        12
// ???: header(2) + type(1) + len(1) + crc(2) = 6
#define FRAME_OVERHEAD           6

// ???????(?DMA????)
#define UART_RX_BUF_SIZE         256

// CRC16-CCITT (poly 0x1021, init 0xFFFF)
uint16_t CRC16_CCITT(const uint8_t* data, uint16_t len);

// ?????? (type=0x02)
uint16_t PackPoseFrame(float x, float y, float theta, uint8_t* out_buf);

// ?????? (???)
typedef enum {
    PARSER_IDLE = 0,
    PARSER_GOT_AA,
    PARSER_GOT_TYPE,
    PARSER_GOT_LEN,
    PARSER_RECV_PAYLOAD,
    PARSER_GOT_CRC_LO,
    PARSER_GOT_CRC_HI,
} ParserState_t;

typedef struct {
    ParserState_t state;
    uint8_t type;
    uint8_t len;
    uint8_t payload[64];   // ??payload 64??
    uint8_t payload_idx;
    uint8_t crc_lo;
    uint8_t crc_hi;
} UartParser_t;

void UartParser_Init(UartParser_t* parser);
bool UartParser_FeedByte(UartParser_t* parser, uint8_t byte, uint8_t* out_type, uint8_t* out_payload, uint8_t* out_len);

// Navigation packet: ISR -> NavTask via queue
#define NAV_CMD_GOTO       0x01
#define NAV_CMD_TOX        0x02
#define NAV_CMD_TOY        0x03
#define NAV_CMD_TURNTO     0x04
#define NAV_CMD_FINE_MOVE  0x05
#define NAV_CMD_SYNC_POSE  0x06   /* f[0]=x_m, f[1]=y_m, f[2]=yaw_deg(可选), f[4]=1 表示同步 yaw */
#define NAV_CMD_ARC        0x07   /* 圆弧(MoveArcTrack): f[0]=半径m, f[1]=方向(+1右/-1左), f[2]=扫过角度°, f[3]=速度(0=默认) */
#define NAV_CMD_CALIB_HEIGHT 0x08
#define NAV_CMD_CALIB_OFFSET 0x09
#define NAV_CMD_PATH       0x0A
#define NAV_CMD_PATH_TEST  0x0B   /* 内置路径测试 (无线调试器触发, 实际走 NAV_CMD_PATH) */
#define NAV_CMD_ARC_TRACK  0x0C   /* 圆弧轨迹跟踪: f[0]=半径m, f[1]=速度m/s, f[2]=方向(±1), f[3]=扫过角度° */
#define NAV_CMD_RUN        0x0D   /* 启动按键模拟: PD15 脉冲500ms, 无参数, 回复 TYPE_RUN_RESP */
#define NAV_CMD_VISION_NUDGE 0x0E  /* 视觉微调: f[0]=direction(0=stop+lock,1=fwd,2=back,3=left,4=right) */
#define NAV_CMD_VISION_CORRECT 0x0F /* f[0]=field dx_mm, f[1]=field dy_mm, f[2]=target_x_m, f[3]=target_y_m */
#define NAV_CMD_IMU_CALIB  0x10   /* IMU gyro/accel zero-bias calibration, no params */

typedef struct {
    uint8_t  cmd;
    float    f[5];   // cmd-dependent parameters
} NavPacket_t;

// Pack a 1-byte result response frame
uint16_t PackNavResult(uint8_t type, uint8_t status, uint8_t* out_buf);

#endif
