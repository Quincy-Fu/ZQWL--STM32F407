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
    /* USART6 等其他 UART 不在这里处理, 留给各自驱动 */
}
