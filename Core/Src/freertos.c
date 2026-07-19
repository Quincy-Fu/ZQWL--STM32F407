/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "can.h"
#include "Emm_V5.h"
#include "pmw3901.h"
#include "imu_protocol.h"
#include "imu_uart.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// ===== 麦轮底盘几何参数 (量过实物, 改这里即�????) =====
#define WHEEL_DIAMETER_M       0.065f    // 轮径 65mm (实测)
#define WHEEL_RADIUS_M         (WHEEL_DIAMETER_M * 0.5f)
#define WHEEL_BASE_HALF_X_M    0.085f    // 半轴�????(前后) 85mm
#define WHEEL_BASE_HALF_Y_M    0.085f    // 半轮距 85mm (全轮距 170mm)
#define L_SUM_M                (WHEEL_BASE_HALF_X_M + WHEEL_BASE_HALF_Y_M)

// 单位换算: m/s �???? 轮子 RPM
#define RPM_PER_MPS            (60.0f / (2.0f * 3.14159265f * WHEEL_RADIUS_M))

// 目标速度单位约定 (上位�????/里程计都用这�????)
// vx, vy: m/s × 100  (int16, 范围±327m/s, 实际车�?1够用)
// omega:  rad/s × 100
#define SPD_SCALE              100.0f

// Emm_V5.0 速度上限
#define MOTOR_VEL_LIMIT       5000

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

// 目标速度 (初�??0, 后续由测试时序或 CommTask 更新)
// 单位: vx,vy = m/s×100, omega = rad/s×100
volatile int16_t g_tgt_vx    = 0;
volatile int16_t g_tgt_vy    = 0;
volatile int16_t g_tgt_omega = 0;

// 里程计输�???? (OdomTask �????, MotorTask/CommTask �????)
// 单位: x,y = �????, theta = 弧度 (�????=CCW 逆时�????)
volatile float g_odom_x     = 0.0f;
volatile float g_odom_y     = 0.0f;
volatile float g_odom_theta = 0.0f;

// 光流里程�?? (OptFlowTask �??, 暂时不和 g_odom 融合, 先单独输出验�??)
// 单位: �??, base_link 坐标�?? (正x=前进, 正y=左移)
volatile float g_optflow_x = 0.0f;
volatile float g_optflow_y = 0.0f;

// 光流调试状�?? (Keil 在线调试器看, 或后�?? CommTask �??)
volatile bool    g_optflow_init_ok = false;   // pmw3901_init 返回�??
volatile uint8_t g_optflow_obs     = 0;       // �??近一�?? observation (�?? 0xBF)
volatile uint8_t g_optflow_squal   = 0;       // �??近一�?? squal (表面质量)

// IMU 朝向角 (ImuTask 写, 后续融合到 g_odom_theta)
// 单位待实测确认: 弧度 or 角度 (假设弧度, 例程写法)
// 上电归零, 相对开机朝向
volatile float g_imu_yaw = 0.0f;

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId MotorTaskHandle;
osThreadId OdomTaskHandle;
osThreadId OptFlowTaskHandle;
osThreadId ImuTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

// 带符�???? RPM �???? Emm_V5 命令 (dir + abs vel)
// is_right: 右侧电机镜像安装, �???? RPM �???? dir=1(CCW)
static void motor_emit(uint8_t addr, float rpm_signed, bool is_right);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);
void StartOdomTask(void const * argument);
void StartOptFlowTask(void const * argument);
void StartImuTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of MotorTask */
  osThreadDef(MotorTask, StartTask02, osPriorityHigh, 0, 512);
  MotorTaskHandle = osThreadCreate(osThread(MotorTask), NULL);

  /* definition and creation of OdomTask */
  osThreadDef(OdomTask, StartOdomTask, osPriorityNormal, 0, 512);
  OdomTaskHandle = osThreadCreate(osThread(OdomTask), NULL);

  /* definition and creation of OptFlowTask */
  osThreadDef(OptFlowTask, StartOptFlowTask, osPriorityNormal, 0, 512);
  OptFlowTaskHandle = osThreadCreate(osThread(OptFlowTask), NULL);

  /* definition and creation of ImuTask */
  osThreadDef(ImuTask, StartImuTask, osPriorityNormal, 0, 512);
  ImuTaskHandle = osThreadCreate(osThread(ImuTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */

  // defaultTask 现在空循�????, 后续可改造成 MonitorTask (心跳/看门�????/调试日志)
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the MotorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */

  // 显式使能四个电机�????�???? (必须, 未在上位机使能过的电机不响应速度命令)
  Emm_V5_En_Control(MOTOR_FL, true, false); osDelay(10);
  Emm_V5_En_Control(MOTOR_FR, true, false); osDelay(10);
  Emm_V5_En_Control(MOTOR_RL, true, false); osDelay(10);
  Emm_V5_En_Control(MOTOR_RR, true, false); osDelay(100);

  // 上电延时2s等待电机初始化完�??
  osDelay(2000);

  // 初始�?? PMW3901 光流 (失败不阻�??, MotorTask 继续; OptFlowTask �?? g_optflow_init_ok 决定挂起)
  g_optflow_init_ok = pmw3901_init();  

  // 时序测试: 左移6s �???? 右移6s �???? 旋转6s �???? 停止
  // 单位: vx,vy = m/s×100, omega = rad/s×100
  // 旋转�???? omega=100(1 rad/s), 6秒转�????6弧度�????344°, 接近�????�????
  // 里程计反馈测�????: 直行0.5m �???? 原地�????90° �???? �????
  // 验证: 量车实际走位, 接近 0.5m �???? 90° = 里程计对
  int phase = 0;
  float theta0 = 0.0f;   // 阶段2 �????始时�???? theta
  for(;;) {
    // 1. 基于里程计反馈的阶段切换
    switch (phase) {
    case 0:  // 阶段1: 直行 0.2 m/s, 走够 0.5 m 切阶�????2
      g_tgt_vx = 20;  g_tgt_vy = 0;  g_tgt_omega = 0;
      if (g_odom_x >= 0.5f) {
        phase = 1;
        theta0 = g_odom_theta;   // 记录转圈起始 theta
      }
      break;
    case 1:  // 阶段2: 原地 CCW �????, 转够 π/2 (90°) 切阶�????3
      g_tgt_vx = 0;  g_tgt_vy = 0;  g_tgt_omega = 100;
      if (g_odom_theta - theta0 >= 1.5708f) {  // π/2
        phase = 2;
      }
      break;
    case 2:  // 阶段3: 停止
    default:
      g_tgt_vx = 0;  g_tgt_vy = 0;  g_tgt_omega = 0;
      break;
    }

    // 2. 读目标�?�度 (int16×100 �???? float m/s, rad/s)
    float vx = g_tgt_vx / SPD_SCALE;
    float vy = g_tgt_vy / SPD_SCALE;
    float w  = g_tgt_omega / SPD_SCALE;

    // 2. X型麦轮�?�运动学 (base_link 坐标�????)
    //    �???? vx = 前进, �???? vy = 左移, �???? w = CCW(逆时�????)
    //    算的�????"车体视角的轮�???? RPM", �????=车前进方�????
    //    motor_emit �???? is_right 参数会处理右侧镜像安�????, 这里不再取负
    float rpm_FL = (vx - vy - L_SUM_M * w) * RPM_PER_MPS;
    float rpm_FR = (vx + vy + L_SUM_M * w) * RPM_PER_MPS;
    float rpm_RL = (vx + vy - L_SUM_M * w) * RPM_PER_MPS;
    float rpm_RR = (vx - vy + L_SUM_M * w) * RPM_PER_MPS;

    // 3. �???? CAN 命令 (每条�????10ms防电机端漏收, 详见 WHEEL_DIRECTION.md)
    //    左侧正→dir=0(CW=前进)  右侧正→dir=1(CCW=前进, 因镜像安�????)
    motor_emit(MOTOR_FL, rpm_FL, false); osDelay(10);
    motor_emit(MOTOR_FR, rpm_FR, true);  osDelay(10);
    motor_emit(MOTOR_RL, rpm_RL, false); osDelay(10);
    motor_emit(MOTOR_RR, rpm_RR, true);  osDelay(10);

    osDelay(50);  // 周期 ~100ms (4×10ms 发命�???? + 50ms 让出 + 余量)
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartOdomTask */
/**
* @brief Function implementing the OdomTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartOdomTask */
void StartOdomTask(void const * argument)
{
  /* USER CODE BEGIN StartOdomTask */

  // Emm_V5.0 编码�???: 1 单位 S_CPOS = 1/65536 �??? = π×D/65536 �???
  const float ENC_TO_M = 3.14159265f * WHEEL_DIAMETER_M / 65536.0f;

  const uint8_t addr_map[4] = {MOTOR_FL, MOTOR_FR, MOTOR_RL, MOTOR_RR};

  // 上次和本轮的 4 个电机原始位�???
  int32_t last_pos[4] = {0, 0, 0, 0};
  int32_t cur_pos[4]  = {0, 0, 0, 0};
  bool has_last = false;          // 首轮无法算增�???, 只填 last_pos
  uint8_t cur_motor = 0;          // 当前正在读的电机索引 0..3

  // 等电机就�??? (MotorTask 里也�??? 2s 等待, 这里再等 500ms 错峰)
  osDelay(2500);

  for(;;) {
    // 1. 给当前电机发 S_CPOS 读命�???
    Emm_V5_Read_Sys_Params(addr_map[cur_motor], S_CPOS);

    // 2. 等回�??? (带超�??? 20ms, 期间过滤掉非本电机的回执�???)
    //    官方例程 S_CPOS 回复格式 (DLC=7):
    //    rxData[0]=0x36 功能�???, rxData[1]=符号�???(�???0为负), rxData[2..5]=大端4字节位置, rxData[6]=0x6B 校验
    uint32_t t_start = HAL_GetTick();
    while (HAL_GetTick() - t_start < 20) {
      if (can.rxFrameFlag) {
        uint8_t rx_addr = (uint8_t)(can.CAN_RxMsg.ExtId >> 8);
        if (rx_addr == addr_map[cur_motor] &&
            can.rxData[0] == 0x36 &&
            can.CAN_RxMsg.DLC == 7) {
          // 拼接 4 字节位置 (大端)
          uint32_t pos_u = ((uint32_t)can.rxData[2] << 24) |
                           ((uint32_t)can.rxData[3] << 16) |
                           ((uint32_t)can.rxData[4] << 8)  |
                           ((uint32_t)can.rxData[5] << 0);
          int32_t pos = (int32_t)pos_u;
          if (can.rxData[1]) pos = -pos;   // 符号�???
          cur_pos[cur_motor] = pos;
          can.rxFrameFlag = false;
          break;
        }
        // 不是当前电机�??? S_CPOS 回复, 清标志继续等
        can.rxFrameFlag = false;
      }
      osDelay(1);
    }
    // 超时没收�???: cur_pos[cur_motor] 保持上轮�???, 差�??=0 (相当于该电机没动)

    // 3. 切换下一个电�???
    cur_motor = (uint8_t)((cur_motor + 1) % 4);

    // 4. 4 个电机都读完 (cur_motor 回到 0) 算一次正运动�???
    if (cur_motor == 0) {
      if (has_last) {
        // 4 个轮子的线位移增�??? (�???), 右侧镜像安装取负
        float d_FL = (float)(cur_pos[0] - last_pos[0]) * ENC_TO_M;
        float d_FR = -(float)(cur_pos[1] - last_pos[1]) * ENC_TO_M;
        float d_RL = (float)(cur_pos[2] - last_pos[2]) * ENC_TO_M;
        float d_RR = -(float)(cur_pos[3] - last_pos[3]) * ENC_TO_M;

        // 麦轮正运动学 (base_link 坐标系增�???)
        float dx_body = (d_FL + d_FR + d_RL + d_RR) * 0.25f;
        float dy_body = (-d_FL + d_FR + d_RL - d_RR) * 0.25f;
        float cur_theta = g_imu_yaw * 0.01745329f;  // ponytail: g_imu_yaw 是角度, cosf/sinf 要弧度, deg->rad

        // 旋转到世界系累加 (用本周期�???始时�??? theta)
        float ct = cosf(cur_theta);
        float st = sinf(cur_theta);
        // 临界�???: 写两个全�???变量中间别被插队�???, �???单关中断
        __disable_irq();
        g_odom_x     += dx_body * ct - dy_body * st;
        g_odom_y     += dx_body * st + dy_body * ct;
        g_odom_theta  = cur_theta;  // 直接覆盖, 不累加
        __enable_irq();
      }

      // 保存本轮位置作为下轮�???"上次"
      last_pos[0] = cur_pos[0];
      last_pos[1] = cur_pos[1];
      last_pos[2] = cur_pos[2];
      last_pos[3] = cur_pos[3];
      has_last = true;
    }

    osDelay(10);  // 每个电机�??? 1 �???: �???+�???+osDelay �??? ~15ms, 4 �??? ~60ms
  }
  /* USER CODE END StartOdomTask */
}

/* USER CODE BEGIN Header_StartOptFlowTask */
/**
* @brief Function implementing the OptFlowTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartOptFlowTask */
void StartOptFlowTask(void const * argument)
{
  /* USER CODE BEGIN StartOptFlowTask */

  // 等电机初始化 + pmw3901_init 完成 (MotorTask 上电序列�? 2.1s, 这里�? 2.5s 错峰)
  osDelay(2500);

  // 光流没初始化好就挂起, 避免狂发 SPI
  if (!g_optflow_init_ok) {
    vTaskSuspend(NULL);
  }

  int16_t dx_pix, dy_pix;
  uint8_t squal, obs;
  static float last_yaw = 0.0f;  // ponytail: for turn detection

  for(;;) {
    // 1. �? motion burst 12 字节
    pmw3901_read_motion(&dx_pix, &dy_pix, &squal, &obs);

    // 2. 暴露调试状�?? (Keil 在线调试器看)
    g_optflow_obs   = obs;
    g_optflow_squal = squal;

    // 3. 只在 observation 正常 + squal 够高时采�? (datasheet 7.2)
    if (obs == PMW_OBSERVATION_OK && squal >= PMW_SQUAL_MIN) {
      // ponytail: turn segment disables optical flow (off-center install causes false motion)
      //   |d_yaw| > 0.02 rad (~1.1 deg) = turning, drop frame
      float d_yaw = g_imu_yaw - last_yaw;
      last_yaw = g_imu_yaw;
      if (fabsf(d_yaw) < 1.1f) {  // ~1.1 deg = 0.02 rad, smaller = straight
        float dx_m = -dx_pix * PMW_PIX_TO_M;
        float dy_m = -dy_pix * PMW_PIX_TO_M;
        __disable_irq();
        g_optflow_x += dx_m;
        g_optflow_y += dy_m;
        __enable_irq();
      }
    }
    // obs != 0xBF �? squal < 0x19 时丢弃本�? (光流表面纹理不够或异�?)

    osDelay(10);  // 100Hz
  }
  /* USER CODE END StartOptFlowTask */
}

/* USER CODE BEGIN Header_StartImuTask */
/**
* @brief Function implementing the ImuTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartImuTask */
void StartImuTask(void const * argument)
{
  /* USER CODE BEGIN StartImuTask */

  // 等其他任务初始化完成 (MotorTask 启动 + CAN 等)
  osDelay(2500);

  // 启动 USART1 接收 (HAL_UART_Receive_IT 1 字节 + 回调自动维持)
  imu_uart_start_rx();

  float yaw;
  for(;;) {
    // 1. 解析环形缓冲中的完整帧 (更新内部 s_yaw)
    imu_protocol_process();

    // 2. 取最新 yaw 存到全局 (单位待实测)
    if (imu_protocol_get_yaw(&yaw)) {
      __disable_irq();
      g_imu_yaw = yaw;
      __enable_irq();
    }

    osDelay(10);  // 100Hz 解析 (IMU 25Hz 上报, 100Hz 解析够)
  }
  /* USER CODE END StartImuTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

// 带符�???? RPM �???? Emm_V5 命令 (dir + abs vel)
// 左侧电机�???? RPM �???? dir=0(CW), 右侧电机�???? RPM �???? dir=1(CCW) 因镜像安�????
static void motor_emit(uint8_t addr, float rpm_signed, bool is_right)
{
  int16_t rpm = (int16_t)rpm_signed;
  uint8_t dir;
  if (is_right) {
    dir = (rpm >= 0) ? 1 : 0;
  } else {
    dir = (rpm >= 0) ? 0 : 1;
  }
  uint16_t vel = (uint16_t)(rpm >= 0 ? rpm : -rpm);
  if (vel > MOTOR_VEL_LIMIT) vel = MOTOR_VEL_LIMIT;
  Emm_V5_Vel_Control(addr, dir, vel, 10, false);
}

/* USER CODE END Application */
