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

唯一差异是旋转方向:
- 外部 θ: CW正 (顺时针)
- 内部 g_imu_yaw: CCW正 (逆时针)
- 映射: move_yaw = -g_imu_yaw

## 里程计公式

```
// 正运动学 (从4轮编码器到位移)
dy_body = (d_FL + d_FR + d_RL + d_RR) / 4    // 前进
dx_body = (d_FL - d_FR - d_RL + d_RR) / 4    // 右移 (右轮镜像已取反)

// 体坐标 → 场坐标 (yaw_deg = g_imu_yaw, CCW正)
move_x +=  dx_body * cos(yaw_deg) + dy_body * sin(yaw_deg)     // +X = 右
move_y += -dx_body * sin(yaw_deg) + dy_body * cos(yaw_deg)     // +Y = 前
move_yaw = -yaw_deg                                              // CW正 = -IMU
```

## 麦轮逆运动学 (内部, 不对外暴露)

```
w_FL = vy + vx + wz    // 左前
w_FR = vy - vx - wz    // 右前 (镜像安装)
w_RL = vy - vx + wz    // 左后
w_RR = vy + vx - wz    // 右后 (镜像安装)

其中: vy=前进, vx=右移, wz=CCW旋转 (Blu3体坐标)
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

CommTask读取的g_odom_x/y/theta由move_sync_to_odom()同步:
```c
g_odom_x     = move_x;                          // 右方向 m
g_odom_y     = move_y;                          // 前进方向 m
g_odom_theta = g_imu_yaw * (pi/180);            // 弧度, CCW正 (内部)
```

上位机当前不使用POSE反馈 (导航命令模式)。
