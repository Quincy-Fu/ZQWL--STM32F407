#include "uart_protocol.h"
#include <string.h>

// CRC16-CCITT: poly=0x1021, init=0xFFFF (?serial.py????)
uint16_t CRC16_CCITT(const uint8_t* data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
            crc &= 0xFFFF;
        }
    }
    return crc;
}

// ??????: AA 55 02 0C [x][y][theta] [crc_lo][crc_hi]
// ??????,out_buf????20??
uint16_t PackPoseFrame(float x, float y, float theta, uint8_t* out_buf)
{
    uint8_t* p = out_buf;
    
    // ??
    *p++ = PROTOCOL_HEADER1;
    *p++ = PROTOCOL_HEADER2;
    
    // type + len
    *p++ = TYPE_POSE;
    *p++ = PAYLOAD_SIZE_POSE;
    
    // payload: 3?float??
    uint8_t* float_ptr = (uint8_t*)&x;
    for (int i = 0; i < 4; i++) *p++ = float_ptr[i];
    float_ptr = (uint8_t*)&y;
    for (int i = 0; i < 4; i++) *p++ = float_ptr[i];
    float_ptr = (uint8_t*)&theta;
    for (int i = 0; i < 4; i++) *p++ = float_ptr[i];
    
    // ??CRC: ?? type + len + payload
    uint16_t crc = CRC16_CCITT(out_buf + 2, 2 + PAYLOAD_SIZE_POSE);
    
    // CRC??
    *p++ = crc & 0xFF;
    *p++ = (crc >> 8) & 0xFF;
    
    return (uint16_t)(p - out_buf);
}

// ??????
void UartParser_Init(UartParser_t* parser)
{
    parser->state = PARSER_IDLE;
    parser->type = 0;
    parser->len = 0;
    parser->payload_idx = 0;
    parser->crc_lo = 0;
    parser->crc_hi = 0;
    memset(parser->payload, 0, sizeof(parser->payload));
}

// ???????,??true???????CRC??
// out_type: ??????
// out_payload: ??payload??
// out_len: ??payload??
bool UartParser_FeedByte(UartParser_t* parser, uint8_t byte, 
                         uint8_t* out_type, uint8_t* out_payload, uint8_t* out_len)
{
    switch (parser->state) {
        case PARSER_IDLE:
            if (byte == PROTOCOL_HEADER1) {
                parser->state = PARSER_GOT_AA;
            }
            break;
            
        case PARSER_GOT_AA:
            if (byte == PROTOCOL_HEADER2) {
                parser->state = PARSER_GOT_TYPE;
            } else if (byte == PROTOCOL_HEADER1) {
                // ????AA,???????
            } else {
                parser->state = PARSER_IDLE;
            }
            break;
            
        case PARSER_GOT_TYPE:
            parser->type = byte;
            parser->state = PARSER_GOT_LEN;
            break;
            
        case PARSER_GOT_LEN:
            parser->len = byte;
            parser->payload_idx = 0;
            memset(parser->payload, 0, sizeof(parser->payload));
            if (parser->len > sizeof(parser->payload)) {
                // len exceeds buffer, reject frame to prevent parser stall
                parser->state = PARSER_IDLE;
            } else if (parser->len == 0) {
                parser->state = PARSER_GOT_CRC_LO;
            } else {
                parser->state = PARSER_RECV_PAYLOAD;
            }
            break;
            
        case PARSER_RECV_PAYLOAD:
            if (parser->payload_idx < sizeof(parser->payload)) {
                parser->payload[parser->payload_idx++] = byte;
            }
            if (parser->payload_idx >= parser->len) {
                parser->state = PARSER_GOT_CRC_LO;
            }
            break;
            
        case PARSER_GOT_CRC_LO:
            parser->crc_lo = byte;
            parser->state = PARSER_GOT_CRC_HI;
            break;
            
        case PARSER_GOT_CRC_HI:
            parser->crc_hi = byte;
            {
                // ??CRC
                uint16_t recv_crc = parser->crc_lo | ((uint16_t)parser->crc_hi << 8);
                uint8_t body[128];
                body[0] = parser->type;
                body[1] = parser->len;
                memcpy(body + 2, parser->payload, parser->len);
                uint16_t calc_crc = CRC16_CCITT(body, 2 + parser->len);
                
                if (recv_crc == calc_crc) {
                    // CRC??,?????
                    *out_type = parser->type;
                    *out_len = parser->len;
                    memcpy(out_payload, parser->payload, parser->len);
                    parser->state = PARSER_IDLE;
                    return true;
                } else {
                    // CRC??
                    parser->state = PARSER_IDLE;
                }
            }
            break;
    }
    
    return false;
}
