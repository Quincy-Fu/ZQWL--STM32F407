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
#include "lcd_ili9488.h"
#include <math.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// ===== 麦轮底盘几何参数 (量过实物, 改这里即�?????) =====
#define WHEEL_DIAMETER_M       0.065f    // 轮径 65mm (实测)
#define WHEEL_RADIUS_M         (WHEEL_DIAMETER_M * 0.5f)
#define WHEEL_BASE_HALF_X_M    0.085f    // 半轴�?????(前后) 85mm
#define WHEEL_BASE_HALF_Y_M    0.085f    // 半轮�? 85mm (全轮�? 170mm)
#define L_SUM_M                (WHEEL_BASE_HALF_X_M + WHEEL_BASE_HALF_Y_M)

// 单位换算: m/s �????? 轮子 RPM
#define RPM_PER_MPS            (60.0f / (2.0f * 3.14159265f * WHEEL_RADIUS_M))

// 目标速度单位约定 (上位�?????/里程计都用这�?????)
// vx, vy: m/s × 100  (int16, 范围±327m/s, 实际车�?1够用)
// omega:  rad/s × 100
#define SPD_SCALE              100.0f

// Emm_V5.0 speed limit
#define MOTOR_VEL_LIMIT       5000

// ===== Position control parameters =====
#define POS_KP_XY      2.0f    // P gain: m/s per m position error
#define POS_KP_THETA   2.0f    // P gain: rad/s per rad heading error
#define POS_DEAD_XY    0.02f   // 2cm deadband
#define POS_DEAD_TH    0.05f   // ~3deg deadband
#define POS_VMAX_XY    0.5f    // max translational speed (m/s)
#define POS_VMAX_W     1.5f    // max rotational speed (rad/s)

// ===== Optical flow fusion =====
#define OPTFLOW_WEIGHT  0.4f   // blend: 0=all encoder, 1=all optflow
#define TURN_THRESH_RAD 0.05f  // yaw change threshold for turn detect (~3deg)

// ===== IMU verification =====
#define IMU_VERIFY_FRAMES  10     // min valid frames to confirm IMU OK
#define IMU_VERIFY_TIMEOUT 10000  // 10s timeout (ms)

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

// 里程计输�????? (OdomTask �?????, MotorTask/CommTask �?????)
// 单位: x,y = �?????, theta = 弧度 (�?????=CCW 逆时�?????)
volatile float g_odom_x     = 0.0f;
volatile float g_odom_y     = 0.0f;
volatile float g_odom_theta = 0.0f;

// 光流里程�??? (OptFlowTask �???, 暂时不和 g_odom 融合, 先单独输出验�???)
// 单位: �???, base_link 坐标�??? (正x=前进, 正y=左移)
volatile float g_optflow_x = 0.0f;
volatile float g_optflow_y = 0.0f;

// 光流调试状�?? (Keil 在线调试器看, 或后�??? CommTask �???)
volatile bool    g_optflow_init_ok = false;   // pmw3901_init 返回�???
volatile uint8_t g_optflow_obs     = 0;       // �???近一�??? observation (�??? 0xBF)
volatile uint8_t g_optflow_squal   = 0;       // �???近一�??? squal (表面质量)
volatile int16_t g_optflow_last_dx_pix = 0;    // last frame raw pixel delta X (debug, not reset)
volatile int16_t g_optflow_last_dy_pix = 0;    // last frame raw pixel delta Y (debug, not reset)
volatile float   g_optflow_last_dx_m   = 0.0f; // last frame dx in meters (debug, not reset)
volatile float   g_optflow_last_dy_m   = 0.0f; // last frame dy in meters (debug, not reset)
volatile uint32_t g_optflow_frame_count = 0;   // valid frame counter (debug)

// IMU 朝向�? (ImuTask �?, 后续融合�? g_odom_theta)
// 单位待实测确�?: 弧度 or 角度 (假设弧度, 例程写法)
// 上电归零, 相对�?机朝�?
volatile float g_imu_yaw = 0.0f;

// Optical flow delta since last odom read (body frame, meters)
// OptFlowTask accumulates here, OdomTask reads and resets each cycle
volatile float g_optflow_dx = 0.0f;
volatile float g_optflow_dy = 0.0f;

// Position control target (world frame)
// Set by CommTask (future) or Keil debugger for testing
volatile float g_tgt_x     = 0.0f;   // target x (m)
volatile float g_tgt_y     = 0.0f;   // target y (m)
volatile float g_tgt_theta = 0.0f;   // target heading (rad)
volatile uint8_t g_pos_mode    = 0;  // 0=velocity mode, 1=position mode
volatile uint8_t g_pos_reached = 0;  // 1=target reached

// IMU verification: set to 1 when enough valid frames received
volatile uint8_t g_imu_verified = 0;

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId MotorTaskHandle;
osThreadId OdomTaskHandle;
osThreadId OptFlowTaskHandle;
osThreadId ImuTaskHandle;
osThreadId DisplayTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

// 带符�????? RPM �????? Emm_V5 命令 (dir + abs vel)
// is_right: 右侧电机镜像安装, �????? RPM �????? dir=1(CCW)
static void motor_emit(uint8_t addr, float rpm_signed, bool is_right);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);
void StartOdomTask(void const * argument);
void StartOptFlowTask(void const * argument);
void StartImuTask(void const * argument);
void StartDisplayTask(void const * argument);

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

  /* definition and creation of DisplayTask */
  osThreadDef(DisplayTask, StartDisplayTask, osPriorityBelowNormal, 0, 512);
  DisplayTaskHandle = osThreadCreate(osThread(DisplayTask), NULL);

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

  // 显式使能四个电机�?????�????? (必须, 未在上位机使能过的电机不响应速度命令)
  Emm_V5_En_Control(MOTOR_FL, true, false); osDelay(10);
  Emm_V5_En_Control(MOTOR_FR, true, false); osDelay(10);
  Emm_V5_En_Control(MOTOR_RL, true, false); osDelay(10);
  Emm_V5_En_Control(MOTOR_RR, true, false); osDelay(100);

  // 上电延时2s等待电机初始化完�???
  osDelay(2000);


  // 初始�??? PMW3901 光流 (失败不阻�???, MotorTask 继续; OptFlowTask �??? g_optflow_init_ok 决定挂起)
  g_optflow_init_ok = pmw3901_init();  

  // --- IMU verification: wait for valid frames before starting ---
  // DisplayTask shows IMU status on LCD during this wait
  {
    uint32_t t0 = HAL_GetTick();
    while (!g_imu_verified) {
      if (HAL_GetTick() - t0 > IMU_VERIFY_TIMEOUT) {
        break;  // timeout: proceed without IMU (encoder-only theta)
      }
      osDelay(100);
    }
  }

  // Main control loop: position mode or velocity mode
  for(;;) {
    float vx, vy, w;

    if (g_pos_mode) {
      // --- Position control mode ---
      // P controller: world-frame position error -> world-frame velocity
      float ex  = g_tgt_x - g_odom_x;
      float ey  = g_tgt_y - g_odom_y;
      float eth = g_tgt_theta - g_odom_theta;
      // Normalize theta error to [-pi, pi]
      while (eth >  3.14159265f) eth -= 6.28318530f;
      while (eth < -3.14159265f) eth += 6.28318530f;

      // Deadband: zero out small errors to avoid jitter
      if (fabsf(ex)  < POS_DEAD_XY) ex  = 0.0f;
      if (fabsf(ey)  < POS_DEAD_XY) ey  = 0.0f;
      if (fabsf(eth) < POS_DEAD_TH) eth = 0.0f;

      // Reached flag
      g_pos_reached = (ex == 0.0f && ey == 0.0f && eth == 0.0f) ? 1 : 0;

      // World-frame P controller
      float vx_w = POS_KP_XY * ex;
      float vy_w = POS_KP_XY * ey;
      w = POS_KP_THETA * eth;

      // Rotate world-frame velocity to body frame
      float ct = cosf(g_odom_theta);
      float st = sinf(g_odom_theta);
      vx =  vx_w * ct + vy_w * st;
      vy = -vx_w * st + vy_w * ct;

      // Velocity limits
      float v_mag = sqrtf(vx * vx + vy * vy);
      if (v_mag > POS_VMAX_XY) {
        float scale = POS_VMAX_XY / v_mag;
        vx *= scale;
        vy *= scale;
      }
      if (w >  POS_VMAX_W) w =  POS_VMAX_W;
      if (w < -POS_VMAX_W) w = -POS_VMAX_W;
    } else {
      // --- Velocity control mode ---
      // g_tgt_vx/vy/omega set by CommTask (future) or Keil debugger
      vx = g_tgt_vx    / SPD_SCALE;
      vy = g_tgt_vy    / SPD_SCALE;
      w  = g_tgt_omega / SPD_SCALE;
    }

    // Inverse kinematics: body velocity -> 4 wheel RPM
    // vx=forward, vy=left, w=CCW; motor_emit handles right-side mirror
    float rpm_FL = (vx - vy - L_SUM_M * w) * RPM_PER_MPS;
    float rpm_FR = (vx + vy + L_SUM_M * w) * RPM_PER_MPS;
    float rpm_RL = (vx + vy - L_SUM_M * w) * RPM_PER_MPS;
    float rpm_RR = (vx - vy + L_SUM_M * w) * RPM_PER_MPS;

    // Send CAN commands (10ms spacing to prevent frame loss)
    motor_emit(MOTOR_FL, rpm_FL, false); osDelay(10);
    motor_emit(MOTOR_FR, rpm_FR, true);  osDelay(10);
    motor_emit(MOTOR_RL, rpm_RL, false); osDelay(10);
    motor_emit(MOTOR_RR, rpm_RR, true);  osDelay(10);

    osDelay(50);  // ~100ms cycle
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

  // Emm_V5.0 编码�????: 1 单位 S_CPOS = 1/65536 �???? = π×D/65536 �????
  const float ENC_TO_M = 3.14159265f * WHEEL_DIAMETER_M / 65536.0f;

  const uint8_t addr_map[4] = {MOTOR_FL, MOTOR_FR, MOTOR_RL, MOTOR_RR};

  // 上次和本轮的 4 个电机原始位�????
  int32_t last_pos[4] = {0, 0, 0, 0};
  int32_t cur_pos[4]  = {0, 0, 0, 0};
  bool has_last = false;          // 首轮无法算增�????, 只填 last_pos
  uint8_t cur_motor = 0;          // 当前正在读的电机索引 0..3
  float last_theta = 0.0f;        // previous IMU theta for turn detection

  // 等电机就�???? (MotorTask 里也�???? 2s 等待, 这里再等 500ms 错峰)
  osDelay(2500);

  for(;;) {
    // 1. 给当前电机发 S_CPOS 读命�????
    Emm_V5_Read_Sys_Params(addr_map[cur_motor], S_CPOS);

    // 2. 等回�???? (带超�???? 20ms, 期间过滤掉非本电机的回执�????)
    //    官方例程 S_CPOS 回复格式 (DLC=7):
    //    rxData[0]=0x36 功能�????, rxData[1]=符号�????(�????0为负), rxData[2..5]=大端4字节位置, rxData[6]=0x6B 校验
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
          if (can.rxData[1]) pos = -pos;   // 符号�????
          cur_pos[cur_motor] = pos;
          can.rxFrameFlag = false;
          break;
        }
        // 不是当前电机�???? S_CPOS 回复, 清标志继续等
        can.rxFrameFlag = false;
      }
      osDelay(1);
    }
    // 超时没收�????: cur_pos[cur_motor] 保持上轮�????, 差�??=0 (相当于该电机没动)

    // 3. 切换下一个电�????
    cur_motor = (uint8_t)((cur_motor + 1) % 4);

    // 4. 4 个电机都读完 (cur_motor 回到 0) 算一次正运动�????
    if (cur_motor == 0) {
      if (has_last) {
        // 4 个轮子的线位移增�???? (�????), 右侧镜像安装取负
        float d_FL = (float)(cur_pos[0] - last_pos[0]) * ENC_TO_M;
        float d_FR = -(float)(cur_pos[1] - last_pos[1]) * ENC_TO_M;
        float d_RL = (float)(cur_pos[2] - last_pos[2]) * ENC_TO_M;
        float d_RR = -(float)(cur_pos[3] - last_pos[3]) * ENC_TO_M;

        // 麦轮正运动学 (base_link 坐标系增�????)
        float dx_body = (d_FL + d_FR + d_RL + d_RR) * 0.25f;
        float dy_body = (-d_FL + d_FR + d_RL - d_RR) * 0.25f;
        float cur_theta = g_imu_yaw * 0.01745329f;  // ponytail: g_imu_yaw 是角�?, cosf/sinf 要弧�?, deg->rad

        // --- Optical flow fusion ---
        // Read and reset optical flow delta (accumulated by OptFlowTask)
        float of_dx, of_dy;
        __disable_irq();
        of_dx = g_optflow_dx;
        of_dy = g_optflow_dy;
        g_optflow_dx = 0.0f;
        g_optflow_dy = 0.0f;
        __enable_irq();

        // Turn detection: if yaw changed significantly, discard optical flow
        float d_theta = cur_theta - last_theta;
        while (d_theta >  3.14159265f) d_theta -= 6.28318530f;
        while (d_theta < -3.14159265f) d_theta += 6.28318530f;

        if (fabsf(d_theta) < TURN_THRESH_RAD && g_optflow_init_ok) {
            // Straight segment: blend encoder + optical flow
            dx_body = (1.0f - OPTFLOW_WEIGHT) * dx_body + OPTFLOW_WEIGHT * of_dx;
            dy_body = (1.0f - OPTFLOW_WEIGHT) * dy_body + OPTFLOW_WEIGHT * of_dy;
        }
        // else: turning, use encoder only (optflow delta already reset/discarded)
        last_theta = cur_theta;

        // 旋转到世界系累加 (用本周期�????始时�???? theta)
        float ct = cosf(cur_theta);
        float st = sinf(cur_theta);
        // 临界�????: 写两个全�????变量中间别被插队�????, �????单关中断
        __disable_irq();
        g_odom_x     += dx_body * ct - dy_body * st;
        g_odom_y     += dx_body * st + dy_body * ct;
        g_odom_theta  = cur_theta;  // 直接覆盖, 不累�?
        __enable_irq();
      }

      // 保存本轮位置作为下轮�????"上次"
      last_pos[0] = cur_pos[0];
      last_pos[1] = cur_pos[1];
      last_pos[2] = cur_pos[2];
      last_pos[3] = cur_pos[3];
      has_last = true;
    }

    osDelay(10);  // 每个电机�???? 1 �????: �????+�????+osDelay �???? ~15ms, 4 �???? ~60ms
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

  // 等电机初始化 + pmw3901_init 完成 (MotorTask 上电序列�?? 2.1s, 这里�?? 2.5s 错峰)
  osDelay(2500);

  // 光流没初始化好就挂起, 避免狂发 SPI
  if (!g_optflow_init_ok) {
    vTaskSuspend(NULL);
  }

  int16_t dx_pix, dy_pix;
  uint8_t squal, obs;
  static float last_yaw = 0.0f;  // ponytail: for turn detection

  for(;;) {
    // 1. �?? motion burst 12 字节
    pmw3901_read_motion(&dx_pix, &dy_pix, &squal, &obs);

    // 2. Expose debug state (Keil debugger)
    g_optflow_obs   = obs;
    g_optflow_squal = squal;
    g_optflow_last_dx_pix = dx_pix;   // raw pixel delta, always updated
    g_optflow_last_dy_pix = dy_pix;

    // 3. 只在 observation 正常 + squal 够高时采�?? (datasheet 7.2)
    if (obs == PMW_OBSERVATION_OK && squal >= PMW_SQUAL_MIN) {
      // ponytail: turn segment disables optical flow (off-center install causes false motion)
      //   |d_yaw| > 0.02 rad (~1.1 deg) = turning, drop frame
      float d_yaw = g_imu_yaw - last_yaw;
      last_yaw = g_imu_yaw;
      if (fabsf(d_yaw) < 1.1f) {  // ~1.1 deg = 0.02 rad, smaller = straight
        float dx_m = -dx_pix * PMW_PIX_TO_M;
        float dy_m = -dy_pix * PMW_PIX_TO_M;
        __disable_irq();
        g_optflow_dx += dx_m;   // delta for OdomTask fusion
        g_optflow_dy += dy_m;
        g_optflow_x  += dx_m;   // running total for display
        g_optflow_y  += dy_m;
        g_optflow_last_dx_m = dx_m;   // per-frame debug (not reset by OdomTask)
        g_optflow_last_dy_m = dy_m;
        g_optflow_frame_count++;
        __enable_irq();
      }
    }
    // obs != 0xBF �?? squal < 0x19 时丢弃本�?? (光流表面纹理不够或异�??)

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

  // 等其他任务初始化完成 (MotorTask 启动 + CAN �?)
  osDelay(2500);

  // 启动 USART1 接收 (HAL_UART_Receive_IT 1 字节 + 回调自动维持)
  imu_uart_start_rx();

  float yaw;
  for(;;) {
    // 1. 解析环形缓冲中的完整�? (更新内部 s_yaw)
    imu_protocol_process();

    // 2. 取最�? yaw 存到全局 (单位待实�?)
    if (imu_protocol_get_yaw(&yaw)) {
      __disable_irq();
      g_imu_yaw = yaw;
      __enable_irq();
    }

    // IMU verification: set flag when enough valid frames received
    if (!g_imu_verified && imu_frame_count >= IMU_VERIFY_FRAMES) {
      g_imu_verified = 1;
    }

    osDelay(10);  // 100Hz 解析 (IMU 25Hz 上报, 100Hz 解析�?)
  }
  /* USER CODE END StartImuTask */
}

/* USER CODE BEGIN Header_StartDisplayTask */
/**
* @brief Function implementing the DisplayTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDisplayTask */
void StartDisplayTask(void const * argument)
{
  /* USER CODE BEGIN StartDisplayTask */
  LCD_Init();
  LCD_Clear(LCD_BLACK);

  char buf[40];
  for(;;)
  {
    // Snapshot all shared variables
    float ox, oy, ot, iy, ofx, ofy, tx, ty, tt;
    uint8_t squal, imu_ok, pos_mode, pos_reached;
    uint32_t fc;
    __disable_irq();
    ox  = g_odom_x;
    oy  = g_odom_y;
    ot  = g_odom_theta;
    iy  = g_imu_yaw;
    ofx = g_optflow_x;
    ofy = g_optflow_y;
    squal = g_optflow_squal;
    imu_ok = g_imu_verified;
    fc = imu_frame_count;
    tx = g_tgt_x;
    ty = g_tgt_y;
    tt = g_tgt_theta;
    pos_mode = g_pos_mode;
    pos_reached = g_pos_reached;
    __enable_irq();

    // Title
    LCD_Print(10, 10, "===== ZQWL ODOMETRY =====", LCD_CYAN, LCD_BLACK);

    // IMU status (most important for first-time verification)
    if (imu_ok) {
      snprintf(buf, sizeof(buf), "IMU: OK  FC:%lu", (unsigned long)fc);
      LCD_Print(10, 40, buf, LCD_GREEN, LCD_BLACK);
    } else {
      snprintf(buf, sizeof(buf), "IMU: WAIT  FC:%lu", (unsigned long)fc);
      LCD_Print(10, 40, buf, LCD_RED, LCD_BLACK);
    }
    snprintf(buf, sizeof(buf), "IMU YAW: %+.2f DEG", (double)iy);
    LCD_Print(10, 60, buf, LCD_YELLOW, LCD_BLACK);

    // Encoder odometry
    LCD_Print(10, 90, "ODOM:", LCD_WHITE, LCD_BLACK);
    snprintf(buf, sizeof(buf), "X:%+.3f Y:%+.3f", (double)ox, (double)oy);
    LCD_Print(10, 110, buf, LCD_WHITE, LCD_BLACK);
    snprintf(buf, sizeof(buf), "THETA: %+.3f RAD", (double)ot);
    LCD_Print(10, 130, buf, LCD_WHITE, LCD_BLACK);

    // Optical flow
    snprintf(buf, sizeof(buf), "OPTF: X:%+.3f Y:%+.3f", (double)ofx, (double)ofy);
    LCD_Print(10, 160, buf, LCD_GREEN, LCD_BLACK);
    snprintf(buf, sizeof(buf), "SQUAL: 0x%02X", squal);
    LCD_Print(10, 180, buf, LCD_GREEN, LCD_BLACK);

    // Position control status
    if (pos_mode) {
      snprintf(buf, sizeof(buf), "POS TGT: %+.2f %+.2f %+.2f", (double)tx, (double)ty, (double)tt);
      LCD_Print(10, 210, buf, LCD_MAGENTA, LCD_BLACK);
      float ex = tx - ox, ey = ty - oy;
      snprintf(buf, sizeof(buf), "ERR: %+.3f %+.3f %s", (double)ex, (double)ey,
               pos_reached ? "REACHED" : "MOVING");
      LCD_Print(10, 230, buf, pos_reached ? LCD_GREEN : LCD_MAGENTA, LCD_BLACK);
    } else {
      LCD_Print(10, 210, "MODE: VELOCITY", LCD_GRAY, LCD_BLACK);
      LCD_Print(10, 230, "SET G_POS_MODE=1 FOR POS", LCD_GRAY, LCD_BLACK);
    }

    osDelay(200);  // ~5 Hz
  }
    /* USER CODE END StartDisplayTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

// 带符�????? RPM �????? Emm_V5 命令 (dir + abs vel)
// 左侧电机�????? RPM �????? dir=0(CW), 右侧电机�????? RPM �????? dir=1(CCW) 因镜像安�?????
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
