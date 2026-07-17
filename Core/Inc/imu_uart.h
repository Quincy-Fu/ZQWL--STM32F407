#ifndef __IMU_UART_H
#define __IMU_UART_H

#include "main.h"

/* ============================================================
 * IMU 串口接收 (HAL 版, USART1)
 *
 * 启动后 HAL_UART_Receive_IT 接收 1 字节, 收到后在
 * HAL_UART_RxCpltCallback 里推入 imu_protocol 环形缓冲,
 * 并再次启动接收下一字节. 替代例程 F10x 的 ISR + 修 2 个 bug:
 *   - 例程 ISR 死循环等 RXNE (大忌) → HAL 不死等
 *   - 例程满 128 字节才推 → 改成每字节就推
 * ============================================================ */

/* 启动 USART1 接收 (任务里调一次, 之后回调自动维持) */
void imu_uart_start_rx(void);

#endif /* __IMU_UART_H */
