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
// Pure Pursuit 路径跟踪 (上位机预插值密集点, MCU端自主跟踪)
#define TYPE_CMD_PATH_BEGIN     0x22   // payload: speed(f32)+count(u8)
#define TYPE_CMD_PATH_POINT     0x23   // payload: x(f32)+y(f32)+heading(f32), heading=999=自由点
#define TYPE_CMD_PATH_EXEC      0x24   // payload: 空, 触发执行
#define TYPE_CMD_PATH_RESP      0x25   // payload: result(u8) 1=完成 0=超时/中止
#define TYPE_CMD_PATH_DEBUG     0x26   // [调试用,定位后删除] MCU→PC: move_x/y+closest+la+vx_f/vy_f+wz+target_yaw

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
#define NAV_CMD_SYNC_POSE  0x06
#define NAV_CMD_ARC        0x07
#define NAV_CMD_CALIB_HEIGHT 0x08
#define NAV_CMD_CALIB_OFFSET 0x09
#define NAV_CMD_PATH       0x0A

typedef struct {
    uint8_t  cmd;
    float    f[5];   // cmd-dependent parameters
} NavPacket_t;

// Pack a 1-byte result response frame
uint16_t PackNavResult(uint8_t type, uint8_t status, uint8_t* out_buf);

#endif
