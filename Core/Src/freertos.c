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
#include "imu_protocol.h"
#include "imu_uart.h"
#include "lcd_ili9488.h"
#include "uart_protocol.h"
#include "move.h"
#include "goto_pos.h"
#include "oflow.h"
#include "light.h"
#include "tim.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// ===== Mecanum chassis geometry (measured, edit here) =====
#define WHEEL_DIAMETER_M       0.065f    // wheel diameter 65mm
#define WHEEL_RADIUS_M         (WHEEL_DIAMETER_M * 0.5f)
#define WHEEL_BASE_HALF_X_M    0.085f    // half wheelbase (front-rear) 85mm
#define WHEEL_BASE_HALF_Y_M    0.085f    // half track width 85mm
#define L_SUM_M                (WHEEL_BASE_HALF_X_M + WHEEL_BASE_HALF_Y_M)

// Unit conversion: m/s -> wheel RPM
#define RPM_PER_MPS            (60.0f / (2.0f * 3.14159265f * WHEEL_RADIUS_M))

// Velocity command scale (upper computer convention)
// vx, vy: m/s x 100 (int16), omega: rad/s x 100
#define SPD_SCALE              100.0f

// Emm_V5.0 speed limit
#define MOTOR_VEL_LIMIT        5000

// IMU verification
#define IMU_VERIFY_FRAMES      10     // min valid frames to confirm IMU OK
#define IMU_VERIFY_TIMEOUT     10000  // timeout (ms)

// ===== 5th motor: 5-slot position actuator (like a servo, CAN addr 0x05) =====
// 16 microstep -> 3200 pulses/rev (vendor example), 5 slots x 72 deg = 360 deg
#define POS_MOTOR_ADDR         0x05   // CAN address of the 5th motor
#define POS_PULSES_PER_REV     3200u  // 16 microstep: 3200 pulses = 1 rev
#define POS_SLOT_COUNT         5      // number of slots
#define POS_PULSES_PER_SLOT    (POS_PULSES_PER_REV / POS_SLOT_COUNT)  // 640 = 72 deg
#define POS_MOVE_VEL_RPM       1000   // slot-to-slot move speed (RPM)
#define POS_MOVE_ACC           10     // acceleration gear (0 = direct start)
#define POS_HOME_WAIT_MS       2500   // wait for power-on homing to complete

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

// ===== Velocity command input (float, set by CommTask DataQueue or Keil debugger) =====
// Unit: vx=right(m/s), vy=forward(m/s), omega=CW(rad/s)
volatile float g_tgt_vx    = 0.0f;   // right+
volatile float g_tgt_vy    = 0.0f;   // forward+
volatile float g_tgt_omega = 0.0f;   // CW+

// ===== Odometry output (THE core result) =====
// Updated by OdomTask (standby) / Move module (active), read by CommTask / DisplayTask
// Blu3 field frame, origin = power-on position & heading
//   x: right (m), y: forward (m), theta: CCW heading (rad) = g_imu_yaw
volatile float g_odom_x     = 0.0f;   // position x (m), right+
volatile float g_odom_y     = 0.0f;   // position y (m), forward+
volatile float g_odom_theta = 0.0f;   // heading (rad), CCW positive

// ===== IMU yaw (internal, used by OdomTask) =====
// Unit: degrees (imu_protocol.c converts raw rad -> deg), CCW positive
// Zero at power-on heading
volatile float g_imu_yaw = 0.0f;

// IMU status: 1 = verified OK
volatile uint8_t g_imu_verified = 0;

// ===== 5th motor slot command =====
// Set by CommTask (future) or Keil debugger.
// Valid: 0..4 = target slot index (each slot = 72 deg, slot 0 = homing origin)
volatile uint8_t g_target_gear = 0;

// Turntable diagnostic counters
volatile uint32_t g_pos_cmd_count = 0;   // CAN position commands sent
volatile uint8_t  g_pos_homed     = 0;   // 1 = homing completed

// NavTask (Stage 3)
osThreadId NavTaskHandle;
osMessageQId NavQueueHandle;

// USART6 mutex: protects concurrent HAL_UART_Transmit between CommTask and NavTask
osMutexId Uart6MutexHandle;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId MotorTaskHandle;
osThreadId OdomTaskHandle;
osThreadId OptFlowTaskHandle;
osThreadId ImuTaskHandle;
osThreadId DisplayTaskHandle;
osThreadId ServoTaskHandle;
osThreadId LightTaskHandle;
osThreadId PosMotorTaskHandle;
osThreadId CommTaskHandle;
osMessageQId DataQueueHandle;

// Light control: set by ISR dispatch, consumed by LightTask
extern volatile uint8_t  g_light_pending_id;
extern volatile uint8_t  g_light_pending_on;
extern volatile uint8_t  g_light_pending;

// ARM servo state control: set by ISR dispatch, consumed by ServoTask
extern volatile uint8_t g_arm_state;  // 1-8 = predefined pose states

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

// Signed RPM -> Emm_V5 velocity command (dir + abs vel)
// is_right: right-side motors mirror-mounted, positive RPM -> dir=1(CCW)
static void motor_emit(uint8_t addr, float rpm_signed, bool is_right);

// CommTask / upper-PC communication
void StartCommTask(void const * argument);
void SendPoseToPC(float x, float y, float theta);

// NavTask / navigation command execution (Stage 3)
void StartNavTask(void const * argument);
void SendNavResultToPC(uint8_t type, uint8_t status);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);
void StartOdomTask(void const * argument);
void StartOptFlowTask(void const * argument);
void StartImuTask(void const * argument);
void StartDisplayTask(void const * argument);
void StartServoTask(void const * argument);
void StartLightTask(void const * argument);
void StartPosMotorTask(void const * argument);

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
  osMutexDef(Uart6Mutex);
  Uart6MutexHandle = osMutexCreate(osMutex(Uart6Mutex));
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* definition and creation of DataQueue */
  osMessageQDef(DataQueue, 10, DataPacket_t);
  DataQueueHandle = osMessageCreate(osMessageQ(DataQueue), NULL);

  /* NavQueue: ISR -> NavTask (navigation commands, Stage 3) */
  osMessageQDef(NavQueue, 5, NavPacket_t);
  NavQueueHandle = osMessageCreate(osMessageQ(NavQueue), NULL);
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

  /* definition and creation of ServoTask */
  osThreadDef(ServoTask, StartServoTask, osPriorityNormal, 0, 512);
  ServoTaskHandle = osThreadCreate(osThread(ServoTask), NULL);

  /* definition and creation of LightTask */
  osThreadDef(LightTask, StartLightTask, osPriorityLow, 0, 512);
  LightTaskHandle = osThreadCreate(osThread(LightTask), NULL);

  /* definition and creation of PosMotorTask */
  osThreadDef(PosMotorTask, StartPosMotorTask, osPriorityNormal, 0, 512);
  PosMotorTaskHandle = osThreadCreate(osThread(PosMotorTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* definition and creation of CommTask */
  osThreadDef(CommTask, StartCommTask, osPriorityNormal, 0, 256);
  CommTaskHandle = osThreadCreate(osThread(CommTask), NULL);

  /* NavTask: executes blocking navigation commands (Stage 3) */
  osThreadDef(NavTask, StartNavTask, osPriorityNormal, 0, 512);
  NavTaskHandle = osThreadCreate(osThread(NavTask), NULL);
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

  // Enable 4 motor drivers
  Emm_V5_En_Control(MOTOR_FL, true, false); osDelay(10);
  Emm_V5_En_Control(MOTOR_FR, true, false); osDelay(10);
  Emm_V5_En_Control(MOTOR_RL, true, false); osDelay(10);
  Emm_V5_En_Control(MOTOR_RR, true, false); osDelay(100);

  // Wait for motor driver initialization
  osDelay(2000);

  // Velocity control loop
  // g_tgt_vx/vy/omega: set by DataQueue (upper PC CMD_VEL) or Keil debugger
  // Unit: vx,vy = m/s, omega = rad/s
  TickType_t last_cmd_tick = xTaskGetTickCount();
  #define VEL_CMD_TIMEOUT_MS  500  // auto-stop if no CMD_VEL for 500ms

  for(;;) {
    // Check DataQueue for new velocity commands (non-blocking)
    DataPacket_t pkt;
    if (xQueueReceive(DataQueueHandle, &pkt, 0) == pdTRUE) {
      float vx, vy, w;
      memcpy(&vx, pkt.data,     4);
      memcpy(&vy, pkt.data + 4, 4);
      memcpy(&w,  pkt.data + 8, 4);
      __disable_irq();
      g_tgt_vx    = vx;
      g_tgt_vy    = vy;
      g_tgt_omega = w;
      __enable_irq();
      last_cmd_tick = xTaskGetTickCount();
    }

    // Safety: auto-stop if no CMD_VEL received within timeout
    if ((xTaskGetTickCount() - last_cmd_tick) > pdMS_TO_TICKS(VEL_CMD_TIMEOUT_MS)) {
      __disable_irq();
      g_tgt_vx    = 0.0f;
      g_tgt_vy    = 0.0f;
      g_tgt_omega = 0.0f;
      __enable_irq();
    }

    // CMD_VEL convention: vx=right(+), vy=forward(+), w=CW(+)
    // Map to MotorTask internal (vx_fwd=vy, vy_left=-vx, w_CCW=-w):
    float vx = g_tgt_vy;     // forward = vy_new
    float vy = -g_tgt_vx;    // left = -(right)
    float w  = -g_tgt_omega; // CCW = -(CW)

    // Inverse kinematics: body velocity -> 4 wheel RPM
    // right-side motors mirror-mounted
    float rpm_FL = (vx - vy - L_SUM_M * w) * RPM_PER_MPS;
    float rpm_FR = (vx + vy + L_SUM_M * w) * RPM_PER_MPS;
    float rpm_RL = (vx + vy - L_SUM_M * w) * RPM_PER_MPS;
    float rpm_RR = (vx - vy + L_SUM_M * w) * RPM_PER_MPS;

    // Stage 3: skip CAN output when Move module is controlling motors
    if (!g_move_active) {
      // Send CAN velocity commands (10ms spacing to prevent frame loss)
      motor_emit(MOTOR_FL, rpm_FL, false); osDelay(10);
      motor_emit(MOTOR_FR, rpm_FR, true);  osDelay(10);
      motor_emit(MOTOR_RL, rpm_RL, false); osDelay(10);
      motor_emit(MOTOR_RR, rpm_RR, true);  osDelay(10);
    }

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

  // Encoder resolution: 1 S_CPOS unit = 1/65536 rev = pi*D/65536 meters
  const float ENC_TO_M = 3.14159265f * WHEEL_DIAMETER_M / 65536.0f;
  const uint8_t addr_map[4] = {MOTOR_FL, MOTOR_FR, MOTOR_RL, MOTOR_RR};

  int32_t last_pos[4] = {0, 0, 0, 0};
  int32_t cur_pos[4]  = {0, 0, 0, 0};
  bool has_last = false;
  uint8_t cur_motor = 0;

  // Wait for motor drivers to be ready
  osDelay(2500);

  for(;;) {
    // Stage 3: skip CAN read when Move module is controlling motors.
    // Reset baseline so OdomTask resumes cleanly without double-counting
    // the encoder movement that Move already integrated.
    if (g_move_active) {
      has_last = false;
      osDelay(20);
      continue;
    }

    // 1. Send S_CPOS read command to current motor
    Emm_V5_Read_Sys_Params(addr_map[cur_motor], S_CPOS);

    // 2. Wait for response (20ms timeout)
    //    S_CPOS reply (DLC=7): [0]=0x36, [1]=sign, [2..5]=big-endian pos, [6]=checksum
    uint32_t t_start = HAL_GetTick();
    while (HAL_GetTick() - t_start < 20) {
      if (can.rxFrameFlag) {
        uint8_t rx_addr = (uint8_t)(can.CAN_RxMsg.ExtId >> 8);
        if (rx_addr == addr_map[cur_motor] &&
            can.rxData[0] == 0x36 &&
            can.CAN_RxMsg.DLC == 7) {
          uint32_t pos_u = ((uint32_t)can.rxData[2] << 24) |
                           ((uint32_t)can.rxData[3] << 16) |
                           ((uint32_t)can.rxData[4] << 8)  |
                           ((uint32_t)can.rxData[5] << 0);
          int32_t pos = (int32_t)pos_u;
          if (can.rxData[1]) pos = -pos;
          cur_pos[cur_motor] = pos;
          can.rxFrameFlag = false;
          break;
        }
        can.rxFrameFlag = false;
      }
      osDelay(1);
    }

    // 3. Advance to next motor
    cur_motor = (uint8_t)((cur_motor + 1) % 4);

    // 4. All 4 motors read -> forward kinematics + world-frame integration
    if (cur_motor == 0) {
      if (has_last) {
        // Wheel linear displacement (right-side mirror: negate)
        float d_FL = (float)(cur_pos[0] - last_pos[0]) * ENC_TO_M;
        float d_FR = -(float)(cur_pos[1] - last_pos[1]) * ENC_TO_M;
        float d_RL = (float)(cur_pos[2] - last_pos[2]) * ENC_TO_M;
        float d_RR = -(float)(cur_pos[3] - last_pos[3]) * ENC_TO_M;

        // Mecanum forward kinematics -> body-frame displacement
        // (must match move.c move_update_odom exactly)
        float dx_body = (d_FL - d_FR - d_RL + d_RR) * 0.25f;  // right
        float dy_body = (d_FL + d_FR + d_RL + d_RR) * 0.25f;  // forward

        // Heading from IMU (g_imu_yaw in degrees -> radians for trig)
        float theta = g_imu_yaw * 0.01745329f;

        // Rotate body -> world frame (X=right, Y=forward, theta=CCW)
        // Same rotation as move.c move_update_odom
        float ct = cosf(theta);
        float st = sinf(theta);
        __disable_irq();
        g_odom_x     += dx_body * ct + dy_body * st;   // right
        g_odom_y     += -dx_body * st + dy_body * ct;  // forward
        g_odom_theta  = theta;
        __enable_irq();
      }

      last_pos[0] = cur_pos[0];
      last_pos[1] = cur_pos[1];
      last_pos[2] = cur_pos[2];
      last_pos[3] = cur_pos[3];
      has_last = true;
    }

    osDelay(10);  // ~15ms per motor read, 4 motors ~60ms per odom cycle
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

  // PMW3901 光流处理: 初始化传感器 → 10ms 采样 → 偏心补偿 → 独立里程计
  OFlow_TaskLoop();

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

  // Wait for other tasks to initialize
  osDelay(2500);

  // Start USART1 reception FIRST (so the IMU's response frames are captured)
  imu_uart_start_rx();

  // Switch IMU to 6-axis algorithm (no magnetometer) -- stops drift.
  // Re-sent every boot: it's only one frame (cheap), and we haven't confirmed
  // the setting persists in flash, so re-sending is the safe choice.
  imu_uart_set_6axis();
  osDelay(100);  // wait for internal mode switch

  // Per-boot 7s gyro/accel calibration REMOVED (wasted ~7s; the IMU keeps its
  // calibration in flash). imu_uart_calibrate_imu() still exists in imu_uart.c
  // for a manual re-calibration if ever needed.

  float yaw;
  for(;;) {
    // Parse complete frames from ring buffer
    imu_protocol_process();

    // Get latest yaw -> global (unit: degrees, from imu_protocol.c)
    if (imu_protocol_get_yaw(&yaw)) {
      __disable_irq();
      g_imu_yaw = yaw;
      __enable_irq();
    }

    // IMU verification: set flag when enough valid frames received
    if (!g_imu_verified && imu_frame_count >= IMU_VERIFY_FRAMES) {
      g_imu_verified = 1;
    }

    osDelay(10);  // 100Hz parse (IMU reports at 25Hz)
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
  extern volatile uint32_t g_rx_nav_count;
  LCD_Init();

  LCD_Clear(LCD_BLACK);

  char buf[40];
  for(;;)
  {
    float ox, oy, ot, iy;
    float mx, my, myaw;
    uint8_t imu_ok, mact;
    uint32_t fc, navcnt;
    __disable_irq();
    ox = g_odom_x;
    oy = g_odom_y;
    ot = g_odom_theta;
    iy = g_imu_yaw;
    imu_ok = g_imu_verified;
    fc = imu_frame_count;
    mact = g_move_active;
    navcnt = g_rx_nav_count;
    mx = move_x;
    my = move_y;
    myaw = move_yaw;
    __enable_irq();

    LCD_Print(10, 10, "-- ZQWL ODOMETRY --", LCD_CYAN, LCD_BLACK);

    if (imu_ok) {
      snprintf(buf, sizeof(buf), "IMU: OK  FC:%lu", (unsigned long)fc);
      LCD_Print(10, 40, buf, LCD_GREEN, LCD_BLACK);
    } else {
      snprintf(buf, sizeof(buf), "IMU: WAIT  FC:%lu", (unsigned long)fc);
      LCD_Print(10, 40, buf, LCD_RED, LCD_BLACK);
    }
    snprintf(buf, sizeof(buf), "YAW: %.2f DEG", (double)iy);
    LCD_Print(10, 60, buf, LCD_YELLOW, LCD_BLACK);

    LCD_Print(10, 90, "ODOM:", LCD_WHITE, LCD_BLACK);
    snprintf(buf, sizeof(buf), "X:%.3f Y:%.3f", (double)ox, (double)oy);
    LCD_Print(10, 110, buf, LCD_WHITE, LCD_BLACK);
    snprintf(buf, sizeof(buf), "THETA: %.3f RAD", (double)ot);
    LCD_Print(10, 130, buf, LCD_WHITE, LCD_BLACK);

    snprintf(buf, sizeof(buf), "NAV:%s N:%lu",
             mact ? "ACT" : "IDL", (unsigned long)navcnt);
    LCD_Print(10, 160, buf, LCD_MAGENTA, LCD_BLACK);
    snprintf(buf, sizeof(buf), "MV:%.3f %.3f", (double)mx, (double)my);
    LCD_Print(10, 180, buf, LCD_MAGENTA, LCD_BLACK);

    osDelay(200);  // ~5Hz refresh
  }

  /* USER CODE END StartDisplayTask */
}

/* USER CODE BEGIN Header_StartServoTask */
/**
* @brief Function implementing the ServoTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartServoTask */
void StartServoTask(void const * argument)
{
  /* USER CODE BEGIN StartServoTask */

  // Servo control via TIM2 PWM (50Hz)
  //   Servo 1 = TIM2_CH2 = PA1
  //   Servo 2 = TIM2_CH3 = PA2
  //   1 count = 1us, servo pulse range: 500-2500us (0-180 deg)
  //   CCR = 500 + angle_deg * 2000 / 180
  //
  // ARM command (TYPE_ARM, payload 1B): [state] = 1-8
  // Each state maps to a predefined (servo1, servo2) angle pair.
  // TODO: fill in actual angles for each state below.

  // State lookup: index 0 = power-on default, 1-8 = arm poses
  // {servo1_angle, servo2_angle} in degrees (0-180)
  static const uint8_t arm_poses[9][2] = {
      { 90,  90},  // [0] power-on default
      { 90,  90},  // [1] state 1 - TODO
      { 90,  90},  // [2] state 2 - TODO
      { 90,  90},  // [3] state 3 - TODO
      { 90,  90},  // [4] state 4 - TODO
      { 90,  90},  // [5] state 5 - TODO
      { 90,  90},  // [6] state 6 - TODO
      { 90,  90},  // [7] state 7 - TODO
      { 90,  90},  // [8] state 8 - TODO
  };

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

  // Center both servos at 90 deg on power-on (CCR=1500)
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 1500);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 1500);

  uint8_t last_state = 0;

  for(;;)
  {
    uint8_t state = g_arm_state;
    if (state >= 1 && state <= 8 && state != last_state) {
      uint8_t a1 = arm_poses[state][0];
      uint8_t a2 = arm_poses[state][1];
      uint32_t ccr1 = 500 + (uint32_t)a1 * 2000 / 180;
      uint32_t ccr2 = 500 + (uint32_t)a2 * 2000 / 180;
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, ccr1);
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, ccr2);
      last_state = state;
    }
    osDelay(10);
  }
  
  /* USER CODE END StartServoTask */
}

/* USER CODE BEGIN Header_StartLightTask */
/**
* @brief Function implementing the LightTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLightTask */
void StartLightTask(void const * argument)
{
  /* USER CODE BEGIN StartLightTask */
  // 3 fill lights on TIM3 CH1(PB4) / CH2(PB5) / CH3(PB0), 1kHz PWM
  Light_Init();
  Light_SetAll(0);  // all off at power-on

  for(;;)
  {
    if (g_light_pending) {
      uint8_t id = g_light_pending_id;
      uint8_t bright = g_light_pending_on ? 100 : 0;
      if (id == 0) {
        Light_SetAll(bright);
      } else {
        Light_SetBright(id, bright);
      }
      g_light_pending = 0;
    }
    osDelay(10);
  }
  /* USER CODE END StartLightTask */
}

/* USER CODE BEGIN Header_StartPosMotorTask */
/**
* @brief Function implementing the PosMotorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPosMotorTask */
void StartPosMotorTask(void const * argument)
{
  /* USER CODE BEGIN StartPosMotorTask */

  // Wait for the Emm_V5.0 driver to boot (vendor example ~2s, OdomTask uses 2.5s)
  osDelay(2500);

  // Trigger single-turn nearest homing: motor returns to the saved origin (= slot 0).
  // The origin must have been set+saved ONCE at install time: align the mechanism
  // to slot 0, then via vendor host PC -> O_Set -> Set O (save). See manual 7.1.
  // The driver's position counter resets on power-off, so this homing re-establishes
  // the absolute reference at every power-on. Homing move is <=180 deg at 30 RPM
  // -> completes in <=~1s; we wait longer as a safe margin.
  Emm_V5_Origin_Trigger_Return(POS_MOTOR_ADDR, 0 /*nearest*/, 0 /*no sync*/);

  // Wait for homing to finish. This task is SEND-ONLY (never reads CAN responses)
  // to avoid contending with OdomTask over the single shared RX buffer (can.rxData).
  osDelay(POS_HOME_WAIT_MS);

  g_pos_homed = 1;  // homing done
  uint8_t cur_gear = 0;  // after homing the mechanism is at slot 0

  for(;;) {
    uint8_t tgt = g_target_gear;

    if (tgt < POS_SLOT_COUNT && tgt != cur_gear) {
      // Absolute position move to slot tgt (= tgt * 72 deg from origin).
      // 13 bytes -> 2 CAN frames via can_SendCmd (with HAL_Delay(10) between frames).
      // No critical section: motor addr 0x05 differs from OdomTask's 0x01-0x04,
      // so CAN frame interleaving is harmless.
      Emm_V5_Pos_Control(POS_MOTOR_ADDR, 0 /*CW*/, POS_MOVE_VEL_RPM, POS_MOVE_ACC,
                         (uint32_t)tgt * POS_PULSES_PER_SLOT,
                         true  /*absolute*/,
                         false /*no sync*/);
      cur_gear = tgt;
      g_pos_cmd_count++;
      osDelay(30);  // post-command cooldown: let motor driver process
    }

    osDelay(20);
  }

  /* USER CODE END StartPosMotorTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

extern UART_HandleTypeDef huart6;

// NavTask: blocking navigation command execution (Stage 3)
// Receives NavPacket_t from ISR via NavQueue, calls Move/Goto layer,
// sends result back to upper PC via USART6.
void StartNavTask(void const * argument)
{
  (void)argument;

  for (;;) {
    NavPacket_t nav;
    if (xQueueReceive(NavQueueHandle, &nav, portMAX_DELAY) == pdTRUE) {
      g_move_active = 1;
      uint8_t result = 0;

      switch (nav.cmd) {
        case NAV_CMD_GOTO:
          result = ToPointClose(nav.f[0], nav.f[1]);
          SendNavResultToPC(TYPE_CMD_GOTO_RESP, result);
          break;

        case NAV_CMD_TOX:
          ToX(nav.f[0]);
          SendNavResultToPC(TYPE_CMD_TOX_RESP, 1);
          break;

        case NAV_CMD_TOY:
          ToY(nav.f[0]);
          SendNavResultToPC(TYPE_CMD_TOY_RESP, 1);
          break;

        case NAV_CMD_TURNTO:
          result = RotateTo(nav.f[0], MOVE_YAW_TURN_LIMIT);
          SendNavResultToPC(TYPE_CMD_TURNTO_RESP, result);
          break;

        case NAV_CMD_FINE_MOVE: {
          float tx = move_x + nav.f[0] * 0.001f;
          float ty = move_y + nav.f[1] * 0.001f;
          result = MoveToAccurateTimed(tx, ty, GOTO_CORRECT_SPEED,
                                       GOTO_CORRECT_TOL, 2500);
          SendNavResultToPC(TYPE_CMD_FINE_RESP, result);
          break;
        }

        case NAV_CMD_SYNC_POSE:
          Move_InitPose(nav.f[0], nav.f[1], g_imu_yaw);
          SendNavResultToPC(TYPE_CMD_SYNC_RESP, 1);
          break;

        case NAV_CMD_ARC:
          result = MoveArc(nav.f[0], nav.f[1], nav.f[2],
                           nav.f[3], nav.f[4], MOVE_ARC_SPEED);
          SendNavResultToPC(TYPE_CMD_ARC_RESP, result);
          break;

        default:
          break;
      }

      g_move_active = 0;
    }
  }
}

// Send 1-byte navigation result response to upper PC (mutex-protected)
void SendNavResultToPC(uint8_t type, uint8_t status)
{
  uint8_t buf[8];
  uint16_t len = PackNavResult(type, status, buf);
  if (osMutexWait(Uart6MutexHandle, 100) == osOK) {
    HAL_UART_Transmit(&huart6, buf, len, 50);
    osMutexRelease(Uart6MutexHandle);
  }
}

// CommTask: send odometry pose to upper PC at 50Hz via USART6
void StartCommTask(void const * argument)
{
  (void)argument;
  for (;;) {
    float px, py, pt;
    __disable_irq();
    px = g_odom_x;
    py = g_odom_y;
    // Read IMU yaw directly (always current, even when stationary)
    // g_imu_yaw is in degrees, CCW positive, zero at power-on
    pt = g_imu_yaw * 0.01745329f;  // deg -> rad
    __enable_irq();
    SendPoseToPC(px, py, pt);
    osDelay(20);  // 50Hz
  }
}

// Signed RPM -> Emm_V5 velocity command (dir + abs vel)
// Left motors: positive RPM -> dir=0(CW); Right (mirror): positive RPM -> dir=1(CCW)
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
