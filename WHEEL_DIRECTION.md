# 麦轮底盘 — 电机方向与硬件约定

## 关键事实

**右侧电机与左侧电机镜像安装**（电机本体在底盘上"对着的"）。要让底盘整体前进，必须给两侧电机发相反方向的命令。

## Emm_V5.0 协议方向定义

`Emm_V5_Vel_Control(addr, dir, vel, acc, snF)` 中的 `dir`：
- `0` = CW（顺时针）
- `1` = CCW（逆时针）

## 本底盘方向映射

| 电机 | 地址 | 安装侧 | 整体前进时的 dir | 备注 |
|---|---|---|---|---|
| MOTOR_FL | 0x01 | 左前 | `0` (CW)  | 左侧电机，CW = 向前 |
| MOTOR_FR | 0x02 | 右前 | `1` (CCW) | 右侧镜像，CCW = 向前 |
| MOTOR_RL | 0x03 | 左后 | `0` (CW)  | 左侧电机，CW = 向前 |
| MOTOR_RR | 0x04 | 右后 | `1` (CCW) | 右侧镜像，CCW = 向前 |

## 反向运动学（vx/vy/omega → 4 轮 RPM）的注意事项

麦轮逆运动学算出的是"轮子线速度"（带符号：正=前进方向）。映射到 `Emm_V5_Vel_Control` 的 `dir + vel` 时：

- 左侧两轮（FL/RL）：算出来的 RPM > 0 → `dir=0`；RPM < 0 → `dir=1`，`vel=abs(RPM)`
- 右侧两轮（FR/RR）：算出来的 RPM > 0 → `dir=1`；RPM < 0 → `dir=0`，`vel=abs(RPM)`

或者更简洁的写法：每个轮子定义一个 `sign` 系数，左侧 `sign=+1`，右侧 `sign=-1`，最终电机方向 = 算出的 RPM × sign 的符号决定。

## 之前踩过的坑

### 1. En 使能状态不持久

1、2 能转、3、4 不转：根因是 Emm_V5.0 的 **En 使能状态**。1、2 在上位机调试时被使能过（存 EEPROM），3、4 没使能。代码必须在启动前显式调用 `Emm_V5_En_Control(addr, true, false)` 给所有电机使能一次。

### 2. 命令连发必须插延时

CAN 硬件层不需要软件延时（`HAL_CAN_AddTxMessage` 进邮箱即返回，控制器自动排队发送，且 `AutoRetransmission=ENABLE` 时瞬时冲突会自动重发）。但 **Emm_V5.0 电机端处理一条命令期间会短暂不响应后续帧**，连发太快后发的电机会漏收。

实测：四电机连发 `Emm_V5_Vel_Control`（无延时）→ 只有地址 1、2 转，3、4 不转。每条之间插 10ms 后四电机全转。

**结论**：每条 `Emm_V5_Vel_Control` / `Emm_V5_Pos_Control` / `Emm_V5_Stop_Now` / `Emm_V5_En_Control` 之间至少 `osDelay(10)`，多个电机同步触发前再加 `osDelay(30)` 等所有电机收完。

## 硬件清单（参考）

- 4× Emm_V5.0 闭环步进电机（带 CAN 通讯）
- 麦轮（每转一圈 3200 脉冲，16 细分）
- STM32F407 + TJA1050/SN65HVD230 CAN 收发器
- CAN1: PB8=RX, PB9=TX, 500kbps
- 总线两端必须各加 120Ω 端接电阻
