#include "imu_uart.h"
#include "usart.h"
#include "imu_protocol.h"

/* 单字节接收缓冲 (HAL 每次接收 1 字节) */
static uint8_t s_rx_byte;

void imu_uart_start_rx(void)
{
    HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1);
}

/* HAL 接收完成回调: USART1 收到 1 字节时调用 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        /* 推入协议解析环形缓冲 */
        imu_protocol_push_byte(s_rx_byte);
        /* 继续接收下一字节 (维持接收循环) */
        HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1);
    }
    /* USART6 DMA接收由stm32f4xx_it.c的IDLE中断管理, 不在这里处理 */
}

/* Send a command frame to IMU via USART1 TX.
 * Frame: [0x7E][0x23][len][func][params...][checksum]
 * len = 4 + param_len + 1 (head2 + len + func + params + checksum)
 * checksum = sum of all preceding bytes & 0xFF */
void imu_uart_send_cmd(uint8_t func, const uint8_t *params, uint8_t param_len)
{
    uint8_t frame[8];
    uint8_t frame_len = (uint8_t)(4 + param_len + 1);
    uint8_t checksum = 0;

    frame[0] = IMU_FRAME_HEAD1;
    frame[1] = IMU_FRAME_HEAD2;
    frame[2] = frame_len;
    frame[3] = func;
    for (uint8_t i = 0; i < param_len; i++) {
        frame[4 + i] = params[i];
    }
    for (uint8_t i = 0; i < frame_len - 1; i++) {
        checksum += frame[i];
    }
    frame[frame_len - 1] = checksum;

    HAL_UART_Transmit(&huart1, frame, frame_len, 100);
}

/* Switch IMU to 6-axis algorithm (no magnetometer).
 * Protocol: func=0x61, param1=0x06 (6-axis), param2=0x5F (fixed).
 * No response expected from IMU. */
void imu_uart_set_6axis(void)
{
    uint8_t params[2] = {0x06, 0x5F};
    imu_uart_send_cmd(IMU_FUNC_SET_ALGO_TYPE, params, 2);
}

/* Trigger IMU gyro/accel calibration.
 * Protocol: func=0x70, param1=0x01, param2=0x5F (from example driver).
 * IMU must be stationary during calibration (up to 7 seconds).
 * IMU responds with RETURN_STATE (0x81) frame on completion. */
void imu_uart_calibrate_imu(void)
{
    uint8_t params[2] = {0x01, 0x5F};
    imu_uart_send_cmd(IMU_FUNC_CALIB_IMU, params, 2);
}
