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

#endif
