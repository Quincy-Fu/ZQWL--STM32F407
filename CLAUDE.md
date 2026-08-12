# zqwl_bot_ws 项目约束

STM32F407 + FreeRTOS + CubeMX 工程。麦轮底盘下位机。

## 不臆想原则（最高优先级）

**任何不知道的协议格式、外设行为、API 用法、寄存器位定义、时序约定——绝不假设，绝不"按常见格式猜"。必须查官方例程或问用户。**

**Why**：曾因假设 Emm_V5.0 的 S_CPOS 回复帧格式（猜"功能码+4字节位置/DLC=5"），实际官方格式是 DLC=7、位置在 rxData[2..5]、还有符号位在 rxData[1]、功能码 0x36——猜错导致 OdomTask 解析不到任何回复，里程计完全没数据，车一直走不停。这种"看起来对、实际错"的 bug 排查极费时间。

**How to apply**：
1. 涉及外部设备协议（电机、传感器、IMU 等）的代码——先找官方例程，例程在工作空间里有，比如 `e:\Robotics\DECEMBER COMPETITION\所用元气件信息\电机驱动\Emm_V5.0闭环步进资料\例程_STM32F407\` 下各子文件夹。**找不到例程就问用户要，不要猜。**
2. 涉及 STM32 HAL 库 API 行为不确定的——查 HAL 文档或 ST 官方 example，不猜
3. 涉及 FreeRTOS API 行为不确定的——查 FreeRTOS 官方文档，不猜
4. 涉及 CubeMX 配置选项不确定的——问用户，不猜
5. **用户没明说但需要决策的细节**：默认按官方例程/ST 文档做法；没有例程时停下来问用户

用户原话："任何东西我都会给你例程，没有你就问，我只要不说，绝对不能自己乱写臆想。"

## FreeRTOS 任务管理约束

**新建 FreeRTOS 任务时，必须让用户在 CubeMX 里建，不要自己在 USER CODE 区硬编码 `osThreadDef` + `osThreadCreate`。**

**Why**：CubeMX 是这个工程的"源"。在 USER CODE 区硬编码任务创建语句虽然能跑，但绕过了 CubeMX 的可视化管理——以后改任务参数（栈大小、优先级）要在代码里找，不在 GUI 里改；而且 CubeMX 重新生成代码时这些硬编码语句虽然保留（USER CODE 区），但跟 CubeMX 生成的 `osThreadDef` 风格不一致，维护成本高。

**How to apply**：
- 需要新任务时，**先停下来告诉用户**：任务名、Entry Function 名、优先级、Stack Size，等用户在 CubeMX → FREERTOS → Tasks and Queues → Add Task 里建好并生成代码
- CubeMX 生成后会自动产生空的 `Start<TaskName>` 函数壳，里面有 `USER CODE BEGIN Start<TaskName> ... END` 区——业务代码贴这里
- 不要在 `USER CODE BEGIN RTOS_THREADS` 区自己加 `osThreadDef`/`osThreadCreate`
- 不要在 `USER CODE BEGIN FunctionPrototypes` 区自己加任务函数原型（CubeMX 会自动生成）

## CubeMX 重新生成代码的常见坑

CubeMX 重新生成代码时，**USER CODE BEGIN/END 之间保留，其他地方全部覆盖**。所以：
- 自定义变量声明必须放在 USER CODE 区，不要写在 CubeMX 生成的代码行之间
- 自定义函数的实现可以放在 `USER CODE BEGIN Application` 或对应任务的 USER CODE 区
- ISR（中断服务程序）里的代码也要放在 `USER CODE BEGIN <IRQn>_0` 区，包括临时变量声明

**USER CODE 区的中文注释可能被 CubeMX 编码损坏**：
CubeMX 保留 USER CODE 区内容时，如果原文件是 UTF-8 而 CubeMX 写出时混了 GBK 或 BOM 处理出错，中文注释的字节会被破坏成乱码，编译器把乱码字节当成标识符 token 解析，报莫名其妙的错误，比如：
- `identifier "取v成..." is undefined`（乱码字节被当 token）
- `expected a ";"`（找不到分号）
- `variable "i" was declared but never referenced`（声明被乱码打断）

**规避**：
- 新增或修改注释统一使用中文；不要为了规避编码问题改成英文注释
- CubeMX 重新生成后要检查文件编码（VSCode 右下角看编码，UTF-8 无 BOM 最稳）
- 如果编译报奇怪 token 错误，先看是不是 USER CODE 区某行末尾混了乱码字节，用 `sed -n '<行号>p' file | cat -A` 看原始字节
- 修复时整行替换为清晰中文注释或删除损坏注释，不要保留乱码字节

## 文件位置约定

- 任务入口函数实现：`Core/Src/freertos.c` 的对应 USER CODE 区
- 电机协议库：`Core/Src/Emm_V5.c` + `Core/Inc/Emm_V5.h`
- CAN 收发：`Core/Src/can.c` + `Core/Inc/can.h`
- 全局变量（任务间共享）：`freertos.c` 的 `USER CODE BEGIN Variables` 区
- 几何参数宏：`freertos.c` 的 `USER CODE BEGIN PD` 区

## 硬件/约定文档

- 麦轮方向、电机使能、CAN 命令连发延时：见 `WHEEL_DIRECTION.md`
- 几何参数（轮径 65mm、半轴距 85mm×125mm）在 `freertos.c` 的 PD 区，量过实物改那里
- 电机 CAN 地址：FL=0x01, FR=0x02, RL=0x03, RR=0x04

## 验证纪律

- 改完代码主动让用户用 Keil 编译验证（不要只改不验）
- VSCode IntelliSense 的 `#include errors detected` / `无法打开 源 文件 "stddef.h"` 之类的报错无视——Keil 用自己的 ARM 工具链，不依赖 VSCode include path
- Keil 编译报错才需要处理

## 待实测确认项（不臆想原则）

这些点 datasheet 没明确或例程代码自相矛盾，等装好实测：

1. **IMU yaw 单位**：弧度 or 角度？
   - 例程代码 `IMU_UART_GetEuler` 把 s_yaw 乘 RAD2DEG(57.29) 输出 → 暗示 s_yaw 是**弧度**
   - 但例程注释自相矛盾（中文"角度" vs 英文"in radians"）
   - 用户当前理解：按弧度
   - **实测方法**：让 IMU 转 90 度，看 `g_imu_yaw` 变化：
     - 变成 1.57 (≈π/2) → 弧度，代码对
     - 变成 90 → 角度，需要改：`g_imu_yaw` 当角度用，后续融合到 g_odom_theta 时（弧度）要 `× π/180`
   - 代码位置：[freertos.c](Core/Src/freertos.c) `g_imu_yaw = yaw;`，[imu_protocol.c](Core/Src/imu_protocol.c) `s_yaw = to_float(&data[8]);`

2. **光流输出符号方向**：datasheet 没明确，按物理原理推断取反（图像反向）
   - 实测：手推车前进 0.5m，看 `g_optflow_x` 是 +0.5 还是 -0.5
   - 代码位置：[freertos.c](Core/Src/freertos.c) `g_optflow_x += -dx_pix * PMW_PIX_TO_M;` 那个负号

3. **光流安装高度**：当前 `PMW_HEIGHT_M=0.10f`（pmw3901.h）是占位，等装好量实物或用已知距离校准
