# 坐标系说明

## 外部约定 (API / 上位机 / path.py)

| 轴 | 正方向 | 说明 |
|---|---|---|
| X | 右 (right) | 从车头方向看去的右侧 |
| Y | 前进 (forward) | 车头上电朝向为+Y |
| θ/yaw | 顺时针 (CW) | 与用户角度一致, 0°=前, 90°=右 |

原点: 上电位置。Yaw: 上电朝向 = 0°。

## 与Blu3内部坐标的关系

外部约定与Blu3场坐标方向完全一致:
- X(右) = Blu3 dx(右)
- Y(前) = Blu3 dy(前)

旋转方向: 全链路统一 CW正 (命令 / 控制 / 里程计 / 位姿上报)。
- move_yaw: CW正(度), 全程由编码器积分维护
- g_odom_theta: CW正(弧度) = move_yaw × π/180
- g_imu_yaw/g_imu_yaw_raw: IMU原始航向, 正方向由安装决定; 仅保留诊断/校准用, 不参与运动角度闭环

## 里程计公式

```
// 正运动学 (从4轮编码器到位移)
dy_body = (d_FL + d_FR + d_RL + d_RR) / 4    // 前进
dx_body = (d_FL - d_FR - d_RL + d_RR) / 4    // 右移 (右轮镜像已取反)

// 普通段航向角: 由编码器积分到外部CW正
move_yaw += ((d_FL - d_FR + d_RL - d_RR) / 4) / MOVE_YAW_L_SUM * 180/pi

// 圆弧段航向角: 与普通段一致, 仍由编码器积分
move_yaw += ((d_FL - d_FR + d_RL - d_RR) / 4) / MOVE_ARC_YAW_L_SUM * 180/pi

// 体坐标 → 场坐标 (公式按 CW+ 角写; 代码内部先转 CCW 再用标准旋转阵)
move_x +=  dx_body * cos(yaw) + dy_body * sin(yaw)     // +X = 右
move_y += -dx_body * sin(yaw) + dy_body * cos(yaw)     // +Y = 前
```

## 麦轮逆运动学 (内部, 不对外暴露)

```
w_FL = vy + vx + wz    // 左前
w_FR = vy - vx - wz    // 右前 (镜像安装)
w_RL = vy - vx + wz    // 左后
w_RR = vy + vx - wz    // 右后 (镜像安装)

其中: vy=前进, vx=右移, wz=CW旋转 (顺时针正, 与move_yaw一致)
```

## 调用示例

```c
// 前进0.5m (Y=前)
MoveTo(0.0, 0.5, 0.15);

// 右移0.3m (X=右, 轴锁定保持Y不变)
MoveToAxisLock(0.3, 0.0, 0.15, 0.03, 0.005, 0.008, MOVE_AXIS_X);

// 原地右转90° (CW正)
RotateTo(90.0, 0.15);

// 圆弧: 从当前位姿出发右转90°, 半径0.3m (圆心自动算, 车头沿切线)
// 上位机 arc 命令新语义: 半径 / 方向(+1右转,-1左转) / 扫过角度°
MoveArcTrack(0.3, 0.10, +1, 90.0, 60000);
```

## CommTask位姿上报

CommTask读取的g_odom_x/y/theta由move_sync_to_odom()同步 (运动中),
待机时由OdomTask用编码器维护XY和yaw:
```c
g_odom_x     = move_x;                          // 右方向 m
g_odom_y     = move_y;                          // 前进方向 m
g_odom_theta = move_yaw * (pi/180);             // 弧度, CW正
```
上报前归一到 [-π, π)。上位机 test_comm.py 显示 yaw(CW+)。
