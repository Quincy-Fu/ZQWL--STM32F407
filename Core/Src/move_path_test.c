/**
 * @file    move_path_test.c
 * @brief   内置路径测试 — 形状生成 + FreeRTOS 触发任务
 *
 * 架构:
 *   形状生成函数 (PathGen_*) 是纯数学 → Move_Path API 的封装,
 *   可独立复用. FreeRTOS 任务 (StartPathTestTask) 轮询全局触发变量,
 *   调用形状生成 → 排队给 NavTask 执行.
 *
 * 添加新形状:
 *   1. 写 PathGen_Xxx() 函数, 调 Move_PathBegin + Move_PathAddPoint
 *   2. 在 move_path_test.h 加 PATH_TEST_XXX ID
 *   3. 在 StartPathTestTask 的 switch 加 case
 */

#include "move_path_test.h"
#include "move.h"
#include "uart_protocol.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include <math.h>

#define PI_F  3.14159265f

/* ── 外部: NavQueue (freertos.c 定义) ─────────────────────────── */
extern osMessageQId NavQueueHandle;

/* ── 全局参数 ─────────────────────────────────────────────────── */
volatile float   g_path_test_speed  = 0.30f;
volatile float   g_path_test_radius = 0.30f;
volatile uint8_t g_path_test_yaw_ff = 0;     /* 0=关闭角速度前馈(默认), 1=开启 */

/* ── 全局状态 ─────────────────────────────────────────────────── */
volatile uint8_t g_path_test_trigger = 0;
volatile uint8_t g_path_test_done    = 0;
volatile uint8_t g_path_test_last    = 0;
volatile uint8_t g_path_test_active  = 0;

/* ================================================================
 *  形状生成
 * ================================================================ */

int PathGen_Semicircle(float speed, float radius)
{
    /* 半圆(右转): 圆心(R,0), (0,0)→(R,R)→(2R,0)
     * alpha: π → 0, 走上侧弧
     * mode=0: 车头沿切线方向P控制(初始切线=0°匹配朝向)
     * 调参: 改半径调radius, 改方向(左转)把y取负 */
    int n = 31;
    Move_PathBegin((uint8_t)n, speed);
    for (int i = 0; i < n; i++) {
        float alpha = PI_F - PI_F * (float)i / (float)(n - 1);  /* π → 0 */
        float x = radius + radius * cosf(alpha);   /* 0 → 2R */
        float y = radius * sinf(alpha);             /* 0 → R → 0 */
        Move_PathAddPoint(x, y, 0.0f, PATH_MODE_NORMAL);
    }
    return n;
}

int PathGen_SCurve(float speed, float radius)
{
    /* S形: 两段反向半圆, (0,0)→(2R,2R)→(0,4R)
     * 第一段右转, 第二段左转, 全mode=0 */
    int n_half = 24;
    int total  = n_half * 2;
    Move_PathBegin((uint8_t)total, speed);

    /* 第一段: 右半圆, 圆心(R,R), 起点(0,0) → 终点(2R,2R) */
    for (int i = 0; i < n_half; i++) {
        float alpha = -PI_F * 0.5f + PI_F * (float)i / (float)(n_half - 1);
        float x = radius + radius * cosf(alpha);
        float y = radius + radius * sinf(alpha);
        Move_PathAddPoint(x, y, 0.0f, PATH_MODE_NORMAL);
    }
    /* 第二段: 左半圆, 圆心(R,3R), 起点(2R,2R) → 终点(0,4R) */
    for (int i = 0; i < n_half; i++) {
        float alpha = PI_F * 0.5f - PI_F * (float)i / (float)(n_half - 1);
        float x = radius + radius * cosf(alpha);
        float y = 3.0f * radius + radius * sinf(alpha);
        Move_PathAddPoint(x, y, 0.0f, PATH_MODE_NORMAL);
    }
    return total;
}

int PathGen_Figure8(float speed, float radius)
{
    /* 8字: Bernoulli双纽线, 起点(0,0), 绕一圈回起点
     * x(t)=R·sint/(1+cos²t), y(t)=R·sint·cost/(1+cos²t)
     * 全mode=0, MCU算切线 */
    int n = 64;
    float R = radius;
    Move_PathBegin((uint8_t)n, speed);
    for (int i = 0; i < n; i++) {
        float t   = 2.0f * PI_F * (float)i / (float)(n - 1);
        float ct  = cosf(t);
        float st  = sinf(t);
        float den = 1.0f + ct * ct;
        float x = R * st / den;
        float y = R * st * ct / den;
        Move_PathAddPoint(x, y, 0.0f, PATH_MODE_NORMAL);
    }
    return n;
}

/* ================================================================
 *  运行器
 * ================================================================ */

/**
 * 排队 NAV_CMD_PATH 给 NavTask, NavTask 调用 MovePathTrack() 阻塞执行.
 * 调用前: 形状生成函数已填充 g_path_pts[] 和 g_path_count.
 */
static void PathTest_Run(void)
{
    NavPacket_t pkt;
    pkt.cmd = NAV_CMD_PATH;
    pkt.f[0] = 0.0f; pkt.f[1] = 0.0f; pkt.f[2] = 0.0f;
    pkt.f[3] = 0.0f; pkt.f[4] = 0.0f;
    xQueueSend(NavQueueHandle, &pkt, 100);
}

/**
 * 排队 NAV_CMD_ARC_TRACK 给 NavTask, NavTask 调用 MoveArcTrack() 阻塞执行.
 * 不需要预生成路径点, 圆弧几何在控制器内部实时计算.
 */
static void PathTest_RunArc(float radius, float speed, int dir, float sweep_deg)
{
    NavPacket_t pkt;
    pkt.cmd = NAV_CMD_ARC_TRACK;
    pkt.f[0] = radius;
    pkt.f[1] = speed;
    pkt.f[2] = (float)dir;
    pkt.f[3] = sweep_deg;
    pkt.f[4] = 0.0f;
    xQueueSend(NavQueueHandle, &pkt, 100);
}

/* ================================================================
 *  FreeRTOS 任务
 * ================================================================ */

/**
 * 低优先级后台轮询任务.
 * 用法: Keil Watch 窗口设 g_path_test_trigger = 1/2/3, 任务自动执行.
 * 执行期间 g_move_active=1 (NavTask 设置), 防止重复触发.
 */
void StartPathTestTask(void const *argument)
{
    (void)argument;

    for (;;) {
        osDelay(200);   /* 200ms 轮询, 调试足够快 */

        uint8_t trigger = g_path_test_trigger;
        if (trigger == PATH_TEST_NONE)
            continue;

        /* 安全检查: NavTask 正在跑别的路径, 等它结束再触发 */
        if (g_move_active)
            continue;

        /* 立即复位, 防重复触发 */
        g_path_test_trigger = PATH_TEST_NONE;

        float spd = g_path_test_speed;
        float rad = g_path_test_radius;

        switch (trigger) {
            case PATH_TEST_SEMICIRCLE:
                /* 半圆(右转): 圆弧轨迹跟踪, 无需路径点 */
                g_path_test_active = 1;
                PathTest_RunArc(rad, spd, +1, 180.0f);
                continue;   /* 已发送, 不走 PathTest_Run */
            case PATH_TEST_S_CURVE:
                PathGen_SCurve(spd, rad);
                break;
            case PATH_TEST_FIGURE8:
                PathGen_Figure8(spd, rad);
                break;
            default:
                continue;   /* 无效 ID, 跳过 */
        }

        /* S形/8字: 路径点方式 */
        g_path_test_active = 1;
        PathTest_Run();
    }
}
