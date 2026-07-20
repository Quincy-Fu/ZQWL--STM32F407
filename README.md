# zqwl_bot_ws

STM32F407ZGT6 + FreeRTOS + CubeMX，麦轮底盘下位机。

## 硬件平台

- MCU: STM32F407ZGT6 (168MHz, 1MB Flash, 192KB RAM)
- RTOS: FreeRTOS v10.3.1 (CMSIS v1)
- 工具链: Keil MDK-ARM 5.32, STM32Cube FW_F4 V1.27.1, CubeMX 6.7.0

## 外设

| 外设 | 引脚 | 速率 | 用途 |
|------|------|------|------|
| CAN1 | PB8=RX, PB9=TX | 500kbps | 4× Emm_V5.0 闭环步进电机 |
| SPI1 | PA5=SCK, PA6=MISO, PA7=MOSI | 2.625Mbps Mode 3 | PMW3901 光流传感器 |
| SPI2 | PB13=SCK, PB14=MISO, PB15=MOSI | 5.25Mbps Mode 0 | ILI9488 3.5" LCD (320×480) |
| USART1 | PA9=TX, PA10=RX | 115200 8N1 | 9 轴 IMU (0x7E 0x23 协议) |
| USART6 | PC6=TX, PC7=RX | 115200 8N1 | 预留 |
| SWD | PA13=SWDIO, PA14=SWCLK | — | 调试 |

GPIO: PD9=LCD_CS, PD10=LCD_RST, PD11=LCD_DC, PD12=LCD_LED, PE4=PMW3901_CS.

## 源文件

### Core/Inc/（头文件）

| 文件 | 说明 |
|------|------|
| `Emm_V5.h` | 电机 CAN 协议库。地址宏 (0x01~0x04)、`SysParams_t` 参数索引枚举、13 个控制 API 声明 |
| `pmw3901.h` | 光流驱动。安装参数 (高度/分辨率)、寄存器地址、`pmw3901_init()` / `pmw3901_read_motion()` |
| `imu_protocol.h` | IMU 帧协议。帧头 `0x7E 0x23`、功能码 (欧拉角=0x26)、环形缓冲 API |
| `imu_uart.h` | IMU 串口接收。`imu_uart_start_rx()` 启动单字节 DMA 接收循环 |
| `lcd_ili9488.h` | LCD 驱动。320×480、RGB565 颜色宏、`LCD_Init()` / `LCD_Clear()` / `LCD_Print()` |
| `can.h` | CAN1 外设。`CAN_t` 结构体 (含 rxData/rxFrameFlag)、`can_SendCmd()` |
| `spi.h` | SPI1/SPI2 句柄 (`hspi1`, `hspi2`) 和 `MX_SPIx_Init()` |
| `usart.h` | USART1/USART6 句柄和 `MX_USARTx_UART_Init()` |
| `gpio.h` | GPIO 初始化声明 |
| `main.h` | SPI CS 引脚宏 (PE2/PE3/PE4)、`Error_Handler()` |
| `FreeRTOSConfig.h` | FreeRTOS 配置 (抢占式/1kHz tick/最大优先级 7/堆 64512B/静态+动态分配) |

### Core/Src/（源文件）

| 文件 | 说明 |
|------|------|
| `main.c` | 入口。`HAL_Init` → 时钟 168MHz → 各外设 Init → CAN 启动 → IMU 串口启动 → `MX_FREERTOS_Init` → `osKernelStart` |
| `freertos.c` | **所有业务逻辑**。6 个任务、全局变量 (里程计/IMU/光流/目标速度)、`motor_emit()`、几何参数宏 |
| `Emm_V5.c` | 电机协议实现。13 个函数均通过 `can_SendCmd()` 发 CAN 帧：速度控制 (`0xF6`)、位置控制 (`0xFD`)、使能 (`0xF3`)、停止 (`0xFE`)、同步等 |
| `can.c` | CAN1 500kbps 初始化 + `can_SendCmd` (支持 >8 字节分包，ExtId=[addr<<8\|packNum]) + `CAN1_RX0_IRQHandler` |
| `pmw3901.c` | 光流驱动实现。SPI1 Mode 3 读写寄存器、上电初始化 (验证 Product ID → POR 寄存器优化 → 验证 Observation)、Motion Burst 12 字节读 |
| `imu_protocol.c` | IMU 协议解析。256B 环形缓冲 + 5 状态机解析帧、累加和校验、`IMU_FUNC_EULER` → 小端 float yaw × 57.2958 转度 |
| `imu_uart.c` | USART1 单字节 `HAL_UART_Receive_IT` + 回调自动续接，每字节推入协议环形缓冲 |
| `lcd_ili9488.c` | LCD 驱动实现。SPI2 写命令/数据、ILI9488 初始化序列 (Arduino 例程)、8×16 ASCII 字体 (43 字符)、`LCD_Print` 字符渲染 |
| `spi.c` | SPI1/SPI2 初始化 + `HAL_SPI_MspInit` (引脚 AF 配置) |
| `usart.c` | USART1/USART6 115200 8N1 初始化 |
| `gpio.c` | 所有 GPIO 初始化 (LCD 控制脚 + 触摸预留 + SPI CS) |
| `stm32f4xx_it.c` | 中断向量: CAN1_RX0、USART1、USART6、TIM1_UP_TIM10、HardFault 等 |
| `stm32f4xx_hal_msp.c` | HAL MSP (外设时钟/中断优先级) |
| `stm32f4xx_hal_timebase_tim.c` | TIM1 为 HAL 提供 `HAL_Delay` / `HAL_GetTick` |

## FreeRTOS 任务

| 任务 | 入口 | 优先级 | 栈 | 周期 | 功能 |
|------|------|--------|-----|------|------|
| MotorTask | `StartTask02` | High | 512W | ~100ms | 电机使能、光流初始化、逆运动学计算 RPM、CAN 发速度命令 |
| OdomTask | `StartOdomTask` | Normal | 512W | ~60ms | 轮询 4 电机 S_CPOS 位置 → 正运动学 → 世界系累加 `g_odom_x/y/theta` |
| OptFlowTask | `StartOptFlowTask` | Normal | 512W | 10ms | 光流 Motion Burst → 转弯检测 (yaw 变化 > 1.1° 丢弃) → 累加 `g_optflow_x/y` |
| ImuTask | `StartImuTask` | Normal | 512W | 10ms | 解析 IMU 帧 → 更新 `g_imu_yaw` (度) |
| DisplayTask | `StartDisplayTask` | BelowNormal | 512W | 200ms | LCD 刷新调试数据: 里程计 XYZ / IMU yaw / 光流 XY + Squal |
| defaultTask | `StartDefaultTask` | Normal | 128W | 空闲 | 空循环，预留给 Monitor/心跳 |

## 里程计融合

三个传感器分工，不分主次：

| 传感器 | 提供 | 强项 | 弱项 |
|--------|------|------|------|
| 电机编码器 (Emm_V5.0) | x, y | 直线位移准 | θ 推算受轴距误差影响；打滑失效 |
| IMU (USART1) | θ | 不受打滑/轴距误差影响 | 长期零漂 (比赛几分钟可控) |
| 光流 (PMW3901) | x, y 辅助 | 不受打滑影响 | 偏心安装，转弯引入假位移 |

策略：
- **θ** → 直接读 IMU yaw 覆盖 `g_odom_theta`，不用编码器推 θ
- **x, y** → 编码器正运动学，用 IMU θ 旋转到世界系
- **光流** → 只在直线段累加 (`|d_yaw| < 1.1°`)，转弯段丢帧

## 关键常量

### 几何参数 ([freertos.c](Core/Src/freertos.c) PD 区)

```c
WHEEL_DIAMETER_M       = 0.065f   // 轮径 65mm
WHEEL_BASE_HALF_X_M    = 0.085f   // 半轴距 85mm (前后)
WHEEL_BASE_HALF_Y_M    = 0.085f   // 半轮距 85mm
L_SUM_M = 0.170f                   // 自动算 (半轴距 + 半轮距)
RPM_PER_MPS = 60/(2π × 0.0325)    // m/s → RPM 换算
```

### 电机

- CAN 地址: FL=0x01, FR=0x02, RL=0x03, RR=0x04
- 协议: ExtId=[addr<<8 | packNum], Data=[func, payload..., 0x6B checksum]
- 速度上限: `MOTOR_VEL_LIMIT = 5000` RPM
- 命令间隔: ≥ 10ms (防止丢帧)
- 右侧镜像安装: `motor_emit` 内部通过 `is_right` 标志自动反转方向
- 编码器: 1 unit = 1/65536 圈, `ENC_TO_M = π×D/65536` m/unit

### 光流 (PMW3901)

- 安装高度: 0.10m (占位，待装好后量实物校准)
- 像素→米: `PMW_PIX_TO_M = 0.002131946f`
- 有效数据: Observation = 0xBF, SQUAL ≥ 0x19

### IMU

- 协议: 帧头 `0x7E 0x23`, 欧拉角功能码 `0x26`, 累加和校验
- yaw 单位: 度 (内部 `× 57.29578` rad→deg)
- 上报频率: 25Hz
- 上电自动归零

## 全局变量（任务间共享）

定义在 [freertos.c](Core/Src/freertos.c) Variables 区，`volatile` + 临界区 `__disable_irq()` 保护：

| 变量 | 类型 | 单位 | 生产者 | 消费者 |
|------|------|------|--------|--------|
| `g_tgt_vx / vy / omega` | int16 | m/s×100 / rad/s×100 | MotorTask (测试) / CommTask (未来) | MotorTask |
| `g_odom_x / y` | float | m | OdomTask | MotorTask, DisplayTask |
| `g_odom_theta` | float | rad | OdomTask (读 IMU) | MotorTask, DisplayTask |
| `g_imu_yaw` | float | deg | ImuTask | OdomTask, OptFlowTask, DisplayTask |
| `g_optflow_x / y` | float | m | OptFlowTask | DisplayTask |
| `g_optflow_obs / squal` | uint8 | — | OptFlowTask | DisplayTask |
| `g_optflow_init_ok` | bool | — | MotorTask (写) | OptFlowTask (读) |
| `can` | CAN_t | — | CAN1_RX0_IRQHandler | OdomTask |

## Emm_V5.0 CAN 协议要点

- **速度模式**: ExtId `0x0A01`, Data `F6 xx xx (dir) xx xx (speed) 6B`
- **S_CPOS 读位置**: 回复 DLC=7, `rxData[0]=0x36`, `rxData[1]=符号位`, `rxData[2..5]=大端 uint32 位置`
- **使能**: 电机上电后必须显式发 `En_Control(true)`, 否则不响应速度命令
- 电机 En 状态存在 EEPROM，但代码初始化时必须显式使能 (不依赖记忆)

## ILI9488 LCD

- 接口: SPI2 Mode 0, 5.25MHz, 控制脚 PD9~PD12
- 色彩: 18-bit/pixel (3 字节 RGB666，与 Arduino 例程一致)
- 字体: 内嵌 8×16 ASCII (大写 A-Z + 数字 0-9 + 部分符号, 43 字符)
- 用途: 调试信息显示 (DisplayTask 5Hz 刷新)

## 未完成

- CommTask: 上位机通讯 (USART6 预留)
- 位置环 / 路径跟随
- 光流安装高度校准 (目前 0.10m 占位)
- IMU yaw 单位实测确认 (目前假设例程正确: 弧度×57.3→度)
- 触摸屏驱动 (XPT2046, 引脚已预留 PC9~PC12/PG8, 未写驱动)

## 约束

详见 [CLAUDE.md](CLAUDE.md):
- 协议/外设行为必须查官方例程，不臆想
- FreeRTOS 任务必须在 CubeMX 中创建
- CubeMX 重新生成时 USER CODE 区保留，其他覆盖
- 中文注释可能被 CubeMX 编码损坏，建议 USER CODE 区用英文注释
