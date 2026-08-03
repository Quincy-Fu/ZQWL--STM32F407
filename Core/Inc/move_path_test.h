/**
 * @file    move_path_test.h
 * @brief   内置路径测试模块 — 无线调试器触发, 无需上位机
 *
 * 用法 (Keil 无线调试器):
 *   1. 在 Watch 窗口添加: g_path_test_speed, g_path_test_radius, g_path_test_trigger
 *   2. 修改 speed / radius (可选)
 *   3. 设置 g_path_test_trigger = 测试编号:
 *        1 = 半圆 (semicircle)
 *        2 = S形 (两个半圆反向连接)
 *        3 = 8字形 (Bernoulli双纽线)
 *   4. 车自动执行, g_path_test_done 计数+1, last_result 存结果
 *
 * 架构: 低优先级后台任务轮询 trigger 变量, 生成路径 → 装入 Move 缓冲 →
 *       排队 NAV_CMD_PATH 给 NavTask 执行. 形状生成函数也可直接从 Keil 调用.
 *
 * 可移植: 形状生成只依赖 math.h + Move_Path API, 换项目只需 move.h 的路径接口.
 */

#ifndef __MOVE_PATH_TEST_H
#define __MOVE_PATH_TEST_H

#include <stdint.h>

/* ── 触发 ID (Keil: 设置 g_path_test_trigger 为此值) ────────── */
#define PATH_TEST_NONE       0   /* 空闲 / 复位 */
#define PATH_TEST_SEMICIRCLE 1   /* 半圆: MoveArcTrack(右转180°), 无需路径点 */
#define PATH_TEST_S_CURVE    2   /* S形: 两段半圆(先右后左), (0,0)→(0,4r) */
#define PATH_TEST_FIGURE8    3   /* 8字: Bernoulli双纽线, 起点(0,0), 总宽≈2.4r */

extern volatile uint8_t g_path_test_trigger;  /* 设置此值触发测试, 任务自动清零 */

/* ── 可调参数 (Keil Watch 窗口直接改) ─────────────────────────── */
extern volatile float   g_path_test_speed;    /* 执行速度 m/s   (默认 0.30) */
extern volatile float   g_path_test_radius;   /* 特征尺寸 m     (默认 0.30) */
extern volatile uint8_t g_path_test_yaw_ff;   /* 0=关闭角速度前馈(默认), 1=开启 */

/* ── 状态 (只读, Keil Watch 观察) ────────────────────────────── */
extern volatile uint8_t g_path_test_done;     /* 累计完成次数 */
extern volatile uint8_t g_path_test_last;     /* 最近结果: 1=完成, 0=超时/中止 */
extern volatile uint8_t g_path_test_active;   /* 1=测试正在执行(NavTask读, 完成后清零) */

/* ── 形状生成 (可直接从 Keil "Call" 调用) ─────────────────────── */
/**
 * 半圆: 圆心(R,0), 起点(0,0)→(R,R)→终点(2R,0)
 * 初始切线=0°(朝前), 终点切线=180°(朝后), 平滑右转
 * mode=0: MCU运行时自动算切线, 不需预存theta
 * @return 生成点数
 */
int PathGen_Semicircle(float speed, float radius);

/**
 * S形: 两段反向半圆, (0,0)→(2R,2R)→(0,4R)
 * 全mode=0, MCU算切线
 * @return 生成点数
 */
int PathGen_SCurve(float speed, float radius);

/**
 * 8字: Bernoulli双纽线, 起点(0,0), 绕一圈回到起点
 * 全mode=0, MCU算切线
 * @return 生成点数
 */
int PathGen_Figure8(float speed, float radius);

/* ── FreeRTOS ─────────────────────────────────────────────────── */
/**
 * 任务入口. 在 freertos.c MX_FREERTOS_Init() 中创建,
 * 低优先级轮询 g_path_test_trigger.
 */
void StartPathTestTask(void const *argument);

#endif /* __MOVE_PATH_TEST_H */
