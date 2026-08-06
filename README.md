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

GPIO: PD9=LCD_CS, PD10=LCD_RST, PD11=LCD_DC, PD12=LCD_LED（开机置高点亮背光）, PE7=PMW3901_CS。

## 源文件

### Core/Inc/（头文件）

| 文件 | 说明 |
|------|------|
| `Emm_V5.h` | 电机 CAN 协议库。地址宏 (0x01~0x04)、`SysParams_t` 参数索引枚举、13 个控制 API 声明 |
| `pmw3901.h` | 光流驱动。安装参数、寄存器地址、运行时 `pmw_pix_to_m` 比例系数、`pmw3901_init()` / `pmw3901_read_motion()` |
| `imu_protocol.h` | IMU 帧协议。帧头 `0x7E 0x23`、功能码 (欧拉角=0x26, 6轴切换=0x61, 校准=0x70)、环形缓冲 API、调试变量 |
| `imu_uart.h` | IMU 串口收发。`imu_uart_start_rx()` 启动接收、`imu_uart_set_6axis()` / `imu_uart_calibrate_imu()` 发命令 |
| `lcd_ili9488.h` | LCD 驱动。320×480、RGB565 颜色宏、`LCD_Init()` / `LCD_Clear()` / `LCD_Print()` |
| `can.h` | CAN1 外设。`CAN_t` 结构体 (含 rxData/rxFrameFlag)、`can_SendCmd()` |
| `spi.h` | SPI1/SPI2 句柄 (`hspi1`, `hspi2`) 和 `MX_SPIx_Init()` |
| `usart.h` | USART1/USART6 句柄和 `MX_USARTx_UART_Init()` |
| `gpio.h` | GPIO 初始化声明 |
| `main.h` | SPI CS 引脚宏 (PE7/PE8/PE9)、`Error_Handler()` |
| `oflow.h` | 光流处理模块。配置参数 (偏移量/采样周期/squal 阈值)、全局状态变量 (oflow_x/y/vx/vy/squal_avg)、API: `OFlow_TaskLoop` / `OFlow_Reset` / `OFlow_GetPose` |
| `oflow_calib.h` | 光流标定模块。位置模式参数 (3200脉冲/圈/200RPM)、`OFlowCalibResult_t` 结构体、API: `OFlowCalib_Height` / `OFlowCalib_Offset` / `OFlowCalib_GetPixToM` / `OFlowCalib_SetPixToM` |
| `FreeRTOSConfig.h` | FreeRTOS 配置 (抢占式/1kHz tick/最大优先级 7/堆 64512B/静态+动态分配) |

### Core/Src/（源文件）

| 文件 | 说明 |
|------|------|
| `main.c` | 入口。`HAL_Init` → 时钟 168MHz → 各外设 Init → CAN 启动 → IMU 串口启动 → `MX_FREERTOS_Init` → `osKernelStart` |
| `freertos.c` | **所有业务逻辑**。8 个任务、全局变量 (里程计/IMU/目标速度)、`motor_emit()`、几何参数宏 |
| `Emm_V5.c` | 电机协议实现。13 个函数均通过 `can_SendCmd()` 发 CAN 帧：速度控制 (`0xF6`)、位置控制 (`0xFD`)、使能 (`0xF3`)、停止 (`0xFE`)、同步等 |
| `can.c` | CAN1 500kbps 初始化 + `can_SendCmd` (支持 >8 字节分包，ExtId=[addr<<8\|packNum]) + `CAN1_RX0_IRQHandler` |
| `pmw3901.c` | 光流驱动实现。SPI1 Mode 3 读写寄存器、上电初始化、Motion Burst 12 字节读 |
| `oflow.c` | 光流处理：100Hz Motion Burst 读 → squal 过滤 → 坐标映射 → 偏心补偿 → 体→场旋转 → 积分到 oflow_x/y |
| `oflow_calib.c` | 光流标定实现：高度标定 (4 轮 Pos_Control 同步行走 + 光流累计 → pix_to_m)、偏心标定 (原地 360° → 反推 Lx/Ly) |
| `imu_protocol.c` | IMU 协议解析。256B 环形缓冲 + 5 状态机解析帧、累加和校验、`IMU_FUNC_EULER` → 小端 float yaw × 57.2958 转度、**首帧 yaw 记为 0° 基准（软件零点）** |
| `imu_uart.c` | USART1 单字节 `HAL_UART_Receive_IT` + 回调自动续接，每字节推入协议环形缓冲；命令发送组帧 (`0x7E 0x23` 头 + 累加和) |
| `lcd_ili9488.c` | LCD 驱动实现。SPI2 写命令/数据、ILI9488 初始化序列、8×16 ASCII 字体 (43 字符)、`LCD_Print` 字符渲染、LCD 调试变量 |
| `spi.c` | SPI1/SPI2 初始化 + `HAL_SPI_MspInit` (引脚 AF 配置) |
| `usart.c` | USART1/USART6 115200 8N1 初始化 |
| `gpio.c` | 所有 GPIO 初始化 (LCD 控制脚 + 触摸预留 + SPI CS) |
| `stm32f4xx_it.c` | 中断向量: CAN1_RX0、USART1、USART6、TIM1_UP_TIM10、HardFault 等 |
| `stm32f4xx_hal_msp.c` | HAL MSP (外设时钟/中断优先级) |
| `stm32f4xx_hal_timebase_tim.c` | TIM1 为 HAL 提供 `HAL_Delay` / `HAL_GetTick` |

## FreeRTOS 任务

| 任务 | 入口 | 优先级 | 栈 | 周期 | 状态 / 功能 |
|------|------|--------|-----|------|------|
| MotorTask | `StartTask02` | High | 512W | ~100ms | **当前停用**（空循环）。原功能：电机使能、逆运动学算 RPM、CAN 发速度命令 |
| OdomTask | `StartOdomTask` | Normal | 512W | ~60ms | 轮询 4 电机 S_CPOS 位置 → 正运动学 → 世界系累加 `g_odom_x/y/theta` |
| OptFlowTask | `StartOptFlowTask` | Normal | 512W | 10ms | PMW3901 光流: `OFlow_TaskLoop()` — 100Hz Motion Burst 读 → 偏心补偿 → 场坐标积分 |
| ImuTask | `StartImuTask` | Normal | 512W | 10ms | 发 6 轴切换命令 → 解析 IMU 帧 → 更新 `g_imu_yaw` (度) → 帧计数达标置 `g_imu_verified` |
| DisplayTask | `StartDisplayTask` | BelowNormal | 512W | 200ms | LCD 刷新: IMU 状态/帧计数、YAW、里程计 X/Y/THETA |
| ServoTask | `StartServoTask` | Normal | 512W | — | 空占位，舵机逻辑待填 |
| LightTask | `StartLightTask` | Low | 512W | — | 空占位，灯光逻辑待填 |
| defaultTask | `StartDefaultTask` | Normal | 128W | 空闲 | 空循环，预留给 Monitor/心跳 |

## 里程计融合

当前用编码器 + IMU + 光流三个传感器：

| 传感器 | 提供 | 强项 | 弱项 |
|--------|------|------|------|
| 电机编码器 (Emm_V5.0) | x, y | 直线位移准 | 打滑失效 |
| IMU (USART1) | θ | 不受打滑/轴距误差影响 | 长期零漂（6 轴模式已抑制） |
| 光流 (PMW3901) | x, y (独立) | 不受轮打滑影响 | 受地面纹理/高度/光照影响 |

策略：
- **θ** → 直接读 IMU yaw（度→弧度）覆盖 `g_odom_theta`，不用编码器推 θ
- **x, y** → 编码器正运动学算出体系位移，用 IMU θ 旋转到世界系累加
- **光流 x, y** → 独立积分，可用于监测/融合（当前独立运行，尚未与编码器融合）

正运动学（麦轮，右侧镜像安装取反）：

```
dx_body = ( d_FL + d_FR + d_RL - d_RR') × 0.25    # d_FR'、d_RR' 已取反
dy_body = (-d_FL + d_FR' + d_RL - d_RR') × 0.25
x += dx_body·cosθ - dy_body·sinθ
y += dx_body·sinθ + dy_body·cosθ
```

## 关键常量

### 几何参数 ([freertos.c](Core/Src/freertos.c) PD 区)

```c
WHEEL_DIAMETER_M       = 0.065f   // 轮径 65mm
WHEEL_BASE_HALF_X_M    = 0.085f   // 半轴距 85mm (前后)
WHEEL_BASE_HALF_Y_M    = 0.085f   // 半轮距 85mm
L_SUM_M = 0.170f                  // 自动算 (半轴距 + 半轮距)
RPM_PER_MPS = 60/(2π × 0.0325)    // m/s → RPM 换算
```

### 电机

- CAN 地址: FL=0x01, FR=0x02, RL=0x03, RR=0x04
- 协议: ExtId=[addr<<8 | packNum], Data=[func, payload..., 0x6B checksum]
- 速度上限: `MOTOR_VEL_LIMIT = 5000` RPM
- 右侧镜像安装: `motor_emit` 内部通过 `is_right` 标志自动反转方向
- 编码器: 1 unit = 1/65536 圈, `ENC_TO_M = π×D/65536` m/unit

### IMU

- 协议: 帧头 `0x7E 0x23`, 欧拉角功能码 `0x26`, 累加和校验
- 6 轴切换: 功能码 `0x61`, 参数 `{0x06, 0x5F}`（开机发送，抑制磁漂；不确定是否存 flash 故每次重发）
- 校准: 功能码 `0x70`, 参数 `{0x01, 0x5F}`（**开机不发**，函数 `imu_uart_calibrate_imu()` 保留供手动调用，需静止约 7s）
- yaw 单位: IMU 上报弧度，内部 `× 57.29578` 转度
- yaw 零点: **软件零点**——开机后第一个有效帧的 yaw 记为 0° 基准，之后显示值 = 原始值 − 基准
- 上报频率: 25Hz
- 校验: 连续收到 ≥ `IMU_VERIFY_FRAMES`(10) 个合法帧 → `g_imu_verified = 1`

## 全局变量（任务间共享）

定义在 [freertos.c](Core/Src/freertos.c) Variables 区，`volatile` + 临界区 `__disable_irq()` 保护：

| 变量 | 类型 | 单位 | 生产者 | 消费者 |
|------|------|------|--------|--------|
| `g_tgt_vx / vy / omega` | int16 | m/s×100 / rad/s×100 | CommTask (未来) / 调试器 | MotorTask |
| `g_odom_x / y` | float | m | OdomTask | DisplayTask |
| `g_odom_theta` | float | rad | OdomTask (读 IMU) | DisplayTask |
| `g_imu_yaw` | float | deg | ImuTask | OdomTask, DisplayTask |
| `g_imu_verified` | uint8 | — | ImuTask | DisplayTask |
| `can` | CAN_t | — | CAN1_RX0_IRQHandler | OdomTask |

IMU 调试变量（[imu_protocol.c](Core/Src/imu_protocol.c)，Keil 在线看）：`imu_frame_count`（合法帧计数）、`imu_rx_byte_count`（收到总字节数）、`imu_last_func`、`imu_last_checksum_ok`、`imu_raw_yaw`（原始弧度）。

## Emm_V5.0 CAN 协议要点

- **速度模式**: ExtId `0x0A01`, Data `F6 xx xx (dir) xx xx (speed) 6B`
- **S_CPOS 读位置**: 回复 DLC=7, `rxData[0]=0x36`, `rxData[1]=符号位`, `rxData[2..5]=大端 uint32 位置`
- **使能**: 电机上电后必须显式发 `En_Control(true)`, 否则不响应速度命令
- 电机 En 状态存在 EEPROM，但代码初始化时必须显式使能 (不依赖记忆)

## ILI9488 LCD

- 接口: SPI2 Mode 0, 5.25MHz, 控制脚 PD9~PD12（CS/RST/DC/LED）
- 色彩: 18-bit/pixel (3 字节 RGB666，COLMOD `0x3A=0x66`)
- **初始化序列关键规则**: 多参数命令的命令字节只发一次（DC 低），参数连续发（DC 高）。ILI9488 重发命令字节会使参数索引归零——逐字节重发命令会只写入最后一个参数（VCOM `0xC5` 被写坏 → 白屏）。序列与官方 LCDWIKI 例程逐字节一致，`0x3A` 提前发送。
- 方向: MADCTL `0x36=0x08`（竖屏 320×480, BGR）。旋转方法见下方「LCD 旋转」小节。
- 字体: 内嵌 8×16 ASCII (大写 A-Z + 数字 0-9 + 部分符号, 43 字符)，**仅大写**
- 调试变量 ([lcd_ili9488.c](Core/Src/lcd_ili9488.c)): `g_lcd_init_done` / `g_lcd_clear_done` / `g_lcd_spi_calls` / `g_lcd_spi_err` / `g_lcd_last_st`

### LCD 旋转（MADCTL 0x36）

旋转/镜像由 MADCTL 寄存器（命令 `0x36`）的位组合控制，整屏（含文字）一起转，文字始终可读：

| 位 | 值 | 含义 |
|----|-----|------|
| MY | 0x80 | 页地址顺序（上下翻转） |
| MX | 0x40 | 列地址顺序（左右翻转） |
| MV | 0x20 | 页/列互换（横竖屏切换的关键位） |
| ML | 0x10 | 垂直刷新顺序 |
| BGR | 0x08 | 色彩顺序（本屏必须置 1） |
| MH | 0x04 | 水平刷新顺序 |

官方 LCDWIKI 例程的 ILI9488 旋转值（源自 `LCDWIKI_SPI.cpp` 的 `Set_Rotation`，case ID_9488）：

| 方向 | MADCTL | 位组合 |
|------|--------|--------|
| 0°（竖屏 320×480） | 0xC8 | MY\|MX\|BGR |
| 90°（横屏 480×320） | 0xA8 | MY\|MV\|BGR |
| 180°（竖屏倒置） | 0x18 | ML\|BGR |
| 270°（横屏反向） | 0x78 | MX\|ML\|MV\|BGR |

**重要：上表不能直接照搬。** 本模块实测可用的竖屏值是 `0x08`（仅 BGR），与官方例程的 0°（0xC8）差了 MX|MY——说明这块屏的贴合方向和官方参考不一致，旋转值必须以 `0x08` 为基准在本机上实测确定：

- 横屏：在 `0x08` 基础上置 MV 位 → 先试 `0x28`；若画面镜像，再按需翻转 MX(0x40)/MY(0x80) 凑出正确方向。
- 改 MADCTL 后**必须同步对调** [lcd_ili9488.h](Core/Inc/lcd_ili9488.h) 的 `LCD_WIDTH`/`LCD_HEIGHT`（横屏为 480×320），否则 `LCD_Clear`/`LCD_SetWindow` 坐标范围错误。
- 代码位置：[lcd_ili9488.c](Core/Src/lcd_ili9488.c) `LCD_Init()` 中 `LCD_WriteCmd(0x36)` 后的 `LCD_WriteData(0x08)`。
- 字体逐像素走坐标绘制，旋转后自动跟随，无需改动。

## 光流模块 (PMW3901)

### 硬件

- 传感器: PMW3901MB-TXQT (PixArt 光流, SPI1 Mode 3, PE7 片选, 2.625Mbps)
- 安装方向: X 朝车前, Y 朝车左 (传感器本体丝印)
- 分辨率: 约 0.00213 m/pixel @ 10cm 高度 (`PMW_RESOLUTION_M = 0.002131946f`)

### 坐标映射

PMW3901 原始输出经安装方向 + 取反后映射到底盘体坐标：

```
dx_body (右) = -dy_raw × pix_to_m
dy_body (前) = -dx_raw × pix_to_m
```

### 偏心补偿

传感器不在底盘几何中心时，旋转会产生寄生位移：

```
dx_center = dx_sensor + dθ × offset_y
dy_center = dy_sensor - dθ × offset_x
```

其中 `dθ` 由 IMU yaw 差分计算 (经低通滤波, alpha=0.3)，`offset_x/y` 为体坐标偏移 (右正/前正)。

### 处理流程 (oflow.c — OFlow_TaskLoop)

1. 启动延时 2500ms → `pmw3901_init()` 检查 ProductID/InvProductID
2. 10ms 周期 (`vTaskDelayUntil`) 读 Motion Burst 12 字节
3. 有效性判断: `observation == 0xBF && squal >= 0x19`，否则跳过
4. 坐标映射 + 偏心补偿
5. 体坐标→场坐标旋转 (用 `g_imu_yaw`)，累加到 `oflow_x/y`
6. 速度估计: `oflow_vx/vy = dx/dt, dy/dt`

### 全局状态变量

| 变量 | 类型 | 单位 | 说明 |
|------|------|------|------|
| `oflow_x / oflow_y` | float | m | 光流场坐标 (右/前正) |
| `oflow_vx / oflow_vy` | float | m/s | 光流速度估计 |
| `oflow_squal_avg` | float | — | 平均表面质量 |
| `oflow_valid_count` | uint32 | — | 有效采样计数 |
| `oflow_invalid_count` | uint32 | — | 无效采样计数 |
| `oflow_accum_dx_raw / dy_raw` | int32 | pixel | 调试用累计原始像素 |
| `oflow_sensor_ok` | uint8 | — | 传感器状态 (1=正常) |

### 标定 (oflow_calib.c)

#### 高度标定 — `OFlowCalib_Height(axis, num_revolutions, result)`

用电机位置模式 (16细分=3200脉冲/圈) 驱动 4 轮同步行走已知距离，同时累计光流像素：

```
pix_to_m = actual_distance_m / accum_pixels_on_motion_axis
estimated_height_m = pix_to_m / PMW_RESOLUTION_M
```

参数: `axis=0` 前进 (Y 轴), `axis=1` 侧移 (X 轴), `num_revolutions` 转数 (如 5.0)。
成功后自动更新全局 `pmw_pix_to_m`。

#### 偏心标定 — `OFlowCalib_Offset(offset_x_out, offset_y_out)`

原地转 360° (调用 `RotateTo(360°, ...)`)，光流理论位移应为零，不为零的差值反推偏移量：

```
Lx = oflow_y_360 / (2π)
Ly = -oflow_x_360 / (2π)
```

标定结果填入 [oflow.h](Core/Inc/oflow.h) 的 `OFLOW_OFFSET_X_M / OFLOW_OFFSET_Y_M`。

### 配置参数

| 宏 (oflow.h) | 默认值 | 说明 |
|---|---|---|
| `OFLOW_OFFSET_X_M` | 0.000f | 传感器右偏量 m (偏心标定后填入) |
| `OFLOW_OFFSET_Y_M` | 0.000f | 传感器前偏量 m |
| `OFLOW_SAMPLE_MS` | 10 | 采样周期 (100Hz) |
| `OFLOW_SQUAL_MIN` | 0x19 | 最低可信 squal |
| `OFLOW_OMEGA_LPFA` | 0.3f | 角速度低通系数 (0~1, 小=更平滑) |
| `OFLOW_INIT_DELAY_MS` | 2500 | 启动等待 ms |

## 运动控制 (move.c)

### 坐标约定

- 体坐标: +X=右, +Y=前
- 场坐标: +X=右, +Y=前 (Blu3 场)
- 角度: CW 正 (顺时针为正)

### wz 符号约定

`Move_SetRobotVelocity(vx, vy, wz)` 的运动学将 wz 视为 **CW+**。右侧电机镜像安装后，正 wz 产生顺时针旋转。所有调用者直接传 CW+ 值，无需取反。

### 关键参数 (move.h)

| 宏 | 值 | 说明 |
|---|---|---|
| `MOVE_CMD_DELAY_MS` | 10 | 电机 CAN 帧间最小间隔 (Emm_V5 硬件约束) |
| `MOVE_ACC_DEFAULT` | 30 | 默认加速度 |
| `MOVE_YAW_SPEED_LIMIT` | 80.0f | 旋转速度上限 deg/s |
| `MOVE_YAW_TURN_LIMIT` | 40.0f | 转弯角速度限制 deg/s |
| `MOVE_LIN_SPEED_DEFAULT` | 0.3f | 默认直线速度 m/s |
| `MOVE_ROTATE_KP` | 0.03f | 角度 P 控制器增益 |

### 运动函数

| 函数 | 功能 |
|------|------|
| `Move_SetRobotVelocity(vx, vy, wz)` | 底层: 体坐标速度 → 麦轮逆运动学 → 4 电机 CAN |
| `Move_SetFieldVelocity(vx_f, vy_f, wz)` | 场坐标速度 → 旋转到体坐标 → 调 SetRobotVelocity |
| `MoveToTimed(dist, angle, timeout_ms)` | 沿指定方向直线移动 (开环时间) |
| `MoveToAccurateTimed(dist, angle, timeout_ms)` | 直线移动 (PID 闭环) |
| `MoveToAxisLockTimed(target_x, target_y, timeout_ms)` | 轴锁定直线移动 (防麦轮漂移) |
| `RotateTo(target_deg, timeout_ms)` | 原地旋转到目标角度 |
| `MoveArc(cx, cy, r, start_deg, end_deg, timeout_ms)` | 圆弧运动 |

## 未完成

- MotorTask 重新启用（当前为空循环停用）
- CommTask: 上位机通讯 (USART6 预留)
- 位置环 / 路径跟随
- 光流与编码器里程计融合（当前独立运行）
- 光流高度标定实测 + 偏心标定 (OFLOW_OFFSET_X/Y_M 待填入)
- ServoTask / LightTask 业务逻辑
- 触摸屏驱动 (XPT2046, 引脚已预留 PC9~PC12/PG8, 未写驱动)

## 约束

详见 [CLAUDE.md](CLAUDE.md):
- 协议/外设行为必须查官方例程，不臆想
- FreeRTOS 任务必须在 CubeMX 中创建
- CubeMX 重新生成时 USER CODE 区保留，其他覆盖
- 中文注释可能被 CubeMX 编码损坏，建议 USER CODE 区用英文注释
