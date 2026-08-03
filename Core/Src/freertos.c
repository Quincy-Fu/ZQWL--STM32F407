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
#include "pmw3901.h"
#include "oflow_calib.h"
#include "xpt2046.h"
#include "light.h"
#include "tim.h"
#include "move_path_test.h"
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
volatile uint32_t g_imu_last_tick = 0;   /* xTaskGetTickCount of last yaw update */

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

// �????�???? CubeMX-managed handles that we also reference in USER CODE �????�????
// These MUST be here (not in CubeMX area) to survive code regeneration.
osMessageQId DataQueueHandle;
osThreadId   CommTaskHandle;

// �????�???? Extern declarations for ISR-set global variables �????�????
// Defined in stm32f4xx_it.c dispatch_frame(), consumed by ServoTask/LightTask.
// MUST be here (not in CubeMX area) to survive code regeneration.
extern volatile uint8_t  g_light_pending_id;
extern volatile uint8_t  g_light_pending_on;
extern volatile uint8_t  g_light_pending;
extern volatile uint8_t  g_arm_state;

// 偏心标定诊断 (定义�?? oflow_calib.c)
extern volatile int32_t  calib_dbg_dx;
extern volatile int32_t  calib_dbg_dy;
extern volatile uint8_t  calib_dbg_squal;
extern volatile uint8_t  calib_dbg_obs;
extern volatile float    calib_dbg_total_deg;
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
osThreadId PathTestTaskHandle;

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
void SendPathDebugToPC(float mx, float my, int16_t wp_idx, int16_t total,
                       float vx_f, float vy_f, float wz, float target_yaw,
                       uint16_t loop_ms, uint8_t enc_ok);  /* [调试�?,定位后删除] */

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
void StartTask10(void const * argument);

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

  /* definition and creation of PathTestTask */
  osThreadDef(PathTestTask, StartTask10, osPriorityLow, 0, 512);
  PathTestTaskHandle = osThreadCreate(osThread(PathTestTask), NULL);

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
    // Reset baseline AND cur_motor so OdomTask resumes cleanly without
    // double-counting the encoder movement that Move already integrated.
    // cur_motor=0 forces a fresh 4-motor read cycle: all cur_pos[] will be
    // overwritten with post-GOTO values BEFORE last_pos is set, preventing
    // stale pre-GOTO values from creating huge false deltas.
    if (g_move_active) {
      has_last = false;
      cur_motor = 0;
      osDelay(20);
      continue;
    }

    // 1. Send S_CPOS read command to current motor
    //    Pre-init cur_pos to last_pos (same fix as move.c move_read_all_encoders):
    //    if read times out, delta=0 instead of retaining a stale pre-GOTO value.
    //    Clear stale rxFrameFlag (same fix as move.c move_read_encoder).
    cur_pos[cur_motor] = last_pos[cur_motor];
    can.rxFrameFlag = false;
    Emm_V5_Read_Sys_Params(addr_map[cur_motor], S_CPOS);

    // 2. Wait for response (20ms timeout)
    //    S_CPOS reply (DLC=7): [0]=0x36, [1]=sign, [2..5]=big-endian pos, [6]=checksum
    //    Read from ISR snapshot to avoid shared buffer race (see stm32f4xx_it.c).
    uint32_t t_start = HAL_GetTick();
    while (HAL_GetTick() - t_start < 20) {
      if (can.rxFrameFlag) {
        uint8_t rx_addr = (uint8_t)(can.rxSnap.ExtId >> 8);
        uint8_t sd[8];
        for (uint8_t k = 0; k < 8; k++) sd[k] = can.rxSnapData[k];
        if (rx_addr == addr_map[cur_motor] &&
            sd[0] == 0x36 &&
            can.rxSnap.DLC == 7) {
          uint32_t pos_u = ((uint32_t)sd[2] << 24) |
                           ((uint32_t)sd[3] << 16) |
                           ((uint32_t)sd[4] << 8)  |
                           ((uint32_t)sd[5] << 0);
          int32_t pos = (int32_t)pos_u;
          if (sd[1]) pos = -pos;   /* sign-magnitude */
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
        // Delta sanity check (same as move.c MOVE_ENC_MAX_DELTA=50000):
        // normal max ~88RPM×65536/60×0.12s �? 6900 counts per 120ms cycle.
        // 50000 = 7x margin. Exceeding it means a read failed in the previous
        // cycle and last_pos still holds a stale pre-GOTO value �? skip
        // integration, just update baseline so next cycle starts fresh.
        int32_t delta[4];
        bool delta_ok = true;
        for (uint8_t i = 0; i < 4; i++) {
          delta[i] = (int32_t)((uint32_t)cur_pos[i] - (uint32_t)last_pos[i]);
          int32_t ad = delta[i];
          if (ad < 0) ad = -ad;
          if (ad > 50000) { delta_ok = false; break; }
        }

        if (delta_ok) {
          // Wheel linear displacement (right-side mirror: negate)
          // uint32 subtraction handles 32-bit wraparound correctly via 2's complement
          float d_FL = (float)delta[0] * ENC_TO_M;
          float d_FR = -(float)delta[1] * ENC_TO_M;
          float d_RL = (float)delta[2] * ENC_TO_M;
          float d_RR = -(float)delta[3] * ENC_TO_M;

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
      }

      // Always update baseline (even if integration was skipped):
      // cur_pos was pre-init'd to last_pos on failed reads, so this is safe.
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

  // PMW3901 光流处理: 初始化传感器 �????? 10ms 采样 �????? 偏心补偿 �????? 独立里程�?????
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

  /* 临时: 重新校准�?螺仪零偏 (保持静止, �?7�?)
   * 校准已完�?, 注释掉避免每次开机等7�? */
  /* imu_uart_calibrate_imu(); */
  /* osDelay(7000); */

  /* �????�???? �????阶低通滤�????: 平滑IMU噪声, 防止角度环振�???? �????�????
   * IMU输出25Hz, alpha=0.5 �???? 时间常数�????58ms
   * 0.2时旋转中滞后0.54°(3.4°/s), 0.5时滞�????0.14°
   * 噪声由RotateToTimed 3帧settle兜底 */
  #define IMU_YAW_LPF_ALPHA  0.5f
  float yaw;
  float yaw_filtered = 0.0f;
  uint8_t yaw_filter_init = 0;
  for(;;) {
    // Parse complete frames from ring buffer
    imu_protocol_process();

    // Get latest yaw -> global (unit: degrees, from imu_protocol.c)
    if (imu_protocol_get_yaw(&yaw)) {
      /* 低�?�滤�????: filtered = filtered + alpha * (raw - filtered) */
      if (!yaw_filter_init) {
        yaw_filtered = yaw;       /* 首次直接赋�??, 避免启动瞬变 */
        yaw_filter_init = 1;
      } else {
        yaw_filtered += IMU_YAW_LPF_ALPHA * (yaw - yaw_filtered);
      }
      __disable_irq();
      g_imu_yaw = yaw_filtered;
      g_imu_last_tick = xTaskGetTickCount();
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
  // Optical flow globals from oflow.h
  extern volatile float oflow_x, oflow_y, oflow_vx, oflow_vy;
  extern volatile float oflow_squal_avg;
  extern volatile uint32_t oflow_valid_count, oflow_invalid_count;
  extern volatile uint8_t oflow_sensor_ok;

  // Encoder diagnostics from move.c
  extern volatile int32_t dbg_enc_raw[4];
  extern volatile int32_t dbg_enc_delta[4];
  extern volatile int32_t dbg_enc_ok;
  extern volatile uint8_t dbg_enc_fail;
  extern volatile uint32_t dbg_enc_bad_delta;
  extern volatile int16_t dbg_cmd_rpm[4];

  LCD_Init();
  LCD_Clear(LCD_BLACK);

  // Initialize XPT2046 touch screen
  uint8_t touch_ok = XPT2046_Init();

  // Touch button state (debounced)
  uint8_t btn_sync_held = 0;  // SYNC 0,0 button pressed flag
  uint8_t btn_goto_held = 0;  // GOTO 0,0.3 button pressed flag

  // Settings page state machine
  uint8_t  page_mode = 0;   // 0=main, 1=settings page
  int      set_x_cm  = 0;   // X in cm (step 5)
  int      set_y_cm  = 0;   // Y in cm (step 5)
  uint8_t  set_yaw_i = 0;   // yaw index 0-7 (each = 45 deg)
  uint8_t  prev_touched = 0; // edge detect: was touched last cycle?

  char buf[42];
  for(;;)
  {
    // �?????�????? snapshot all shared variables �?????�?????
    float ox, oy, iy;
    uint8_t imu_ok, mact;
    float ofx, ofy, ofvx, ofvy, ofsq;
    uint32_t ofok, ofbad;
    uint8_t ofsen;
    uint8_t tg, homed;
    uint32_t pcnt;
    uint16_t pmw_shut;
    uint8_t pmw_led;

    __disable_irq();
    ox = g_odom_x;
    oy = g_odom_y;
    iy = g_imu_yaw;
    imu_ok = g_imu_verified;
    mact = g_move_active;
    ofx = oflow_x;
    ofy = oflow_y;
    ofvx = oflow_vx;
    ofvy = oflow_vy;
    ofsq = oflow_squal_avg;
    ofok = oflow_valid_count;
    ofbad = oflow_invalid_count;
    ofsen = oflow_sensor_ok;
    pmw_shut = pmw_last_shutter;
    pmw_led = pmw_led_readback;
    tg = g_target_gear;
    homed = g_pos_homed;
    pcnt = g_pos_cmd_count;
    /* encoder diag snapshot */
    int32_t ed_raw[4], ed_delta[4], ed_ok;
    uint8_t ed_fail;
    uint32_t ed_bad;
    int16_t ed_rpm[4];
    for (int _i = 0; _i < 4; _i++) { ed_raw[_i] = dbg_enc_raw[_i]; ed_delta[_i] = dbg_enc_delta[_i]; ed_rpm[_i] = dbg_cmd_rpm[_i]; }
    ed_ok = dbg_enc_ok;
    ed_fail = dbg_enc_fail;
    ed_bad = dbg_enc_bad_delta;
    __enable_irq();

    // Normalize yaw to 0-360 range for display
    float yaw_disp = iy;
    while (yaw_disp < 0.0f) yaw_disp += 360.0f;
    while (yaw_disp >= 360.0f) yaw_disp -= 360.0f;

    // ════════════════════════════════════════
    //  Section 1: ODOMETRY  (y = 0 ~ 80)
    // ════════════════════════════════════════
    LCD_Print(4, 0, "-- ODOMETRY --", LCD_CYAN, LCD_BLACK);

    snprintf(buf, sizeof(buf), "X:%+.3f  Y:%+.3f M",
             (double)ox, (double)oy);
    LCD_Print(4, 20, buf, LCD_WHITE, LCD_BLACK);

    snprintf(buf, sizeof(buf), "YAW: %6.1f DEG", (double)yaw_disp);
    LCD_Print(4, 40, buf, LCD_YELLOW, LCD_BLACK);

    snprintf(buf, sizeof(buf), "IMU:%s  NAV:%s",
             imu_ok ? "OK" : "WAIT", mact ? "BUSY" : "IDLE");
    LCD_Print(4, 60, buf, imu_ok ? LCD_GREEN : LCD_RED, LCD_BLACK);

    // separator
    LCD_Print(4, 82, "------------------------------", LCD_GRAY, LCD_BLACK);

    // ════════════════════════════════════════
    //  Section 2: OPTICAL FLOW  (y = 100 ~ 200)
    // ════════════════════════════════════════
    LCD_Print(4, 100, "-- OPTICAL FLOW --", LCD_GREEN, LCD_BLACK);

    snprintf(buf, sizeof(buf), "FX:%+.3f FY:%+.3f M",
             (double)ofx, (double)ofy);
    LCD_Print(4, 120, buf, LCD_WHITE, LCD_BLACK);

    snprintf(buf, sizeof(buf), "VX:%+.3f VY:%+.3f",
             (double)ofvx, (double)ofvy);
    LCD_Print(4, 140, buf, LCD_WHITE, LCD_BLACK);

    snprintf(buf, sizeof(buf), "SQ:%.0f V:%lu B:%lu",
             (double)ofsq, (unsigned long)ofok, (unsigned long)ofbad);
    LCD_Print(4, 160, buf, LCD_GRAY, LCD_BLACK);

    snprintf(buf, sizeof(buf), "%s SHUT:%u LED:0x%02X",
             ofsen ? "SENSOR:OK" : "SENSOR:NO",
             (unsigned)pmw_shut, (unsigned)pmw_led);
    LCD_Print(4, 180, buf, ofsen ? LCD_GREEN : LCD_RED, LCD_BLACK);

    // separator
    LCD_Print(4, 202, "------------------------------", LCD_GRAY, LCD_BLACK);

    // ════════════════════════════════════════
    //  Section 3: TURNTABLE  (y = 220 ~ 300)
    // ════════════════════════════════════════
    LCD_Print(4, 220, "-- TURNTABLE --", LCD_MAGENTA, LCD_BLACK);

    snprintf(buf, sizeof(buf), "SLOT: %d (%3d DEG)", (int)tg, (int)tg * 72);
    LCD_Print(4, 240, buf, LCD_WHITE, LCD_BLACK);

    snprintf(buf, sizeof(buf), "HOME:%s  CMD:%lu",
             homed ? "OK" : "WAIT", (unsigned long)pcnt);
    LCD_Print(4, 260, buf, homed ? LCD_GREEN : LCD_RED, LCD_BLACK);

    // Show all 5 slots, highlight current target
    {
      char slots[32];
      int pos = 0;
      for (int i = 0; i < 5; i++) {
        if (i == (int)tg) {
          pos += snprintf(slots + pos, sizeof(slots) - pos, "[%d]", i);
        } else {
          pos += snprintf(slots + pos, sizeof(slots) - pos, " %d ", i);
        }
      }
      slots[pos] = '\0';
      LCD_Print(4, 280, slots, LCD_CYAN, LCD_BLACK);
    }

    // separator
    LCD_Print(4, 302, "------------------------------", LCD_GRAY, LCD_BLACK);

    // ════════════════════════════════════════
    //  Section 3b: ENCODER DIAG  (y = 305 ~ 340)
    // ════════════════════════════════════════
    {
      snprintf(buf, sizeof(buf), "D:%d %d %d %d R:%d F:%d",
               (int)ed_delta[0], (int)ed_delta[1],
               (int)ed_delta[2], (int)ed_delta[3],
               (int)ed_ok, (int)ed_fail);
      LCD_Print(4, 305, buf, ed_fail ? LCD_RED : LCD_GRAY, LCD_BLACK);

      snprintf(buf, sizeof(buf), "V:%d %d %d %d B:%lu",
               (int)ed_rpm[0], (int)ed_rpm[1],
               (int)ed_rpm[2], (int)ed_rpm[3],
               (unsigned long)ed_bad);
      LCD_Print(4, 322, buf, ed_bad ? LCD_YELLOW : LCD_GRAY, LCD_BLACK);
    }

    // ════════════════════════════════════════
    //  Section 4: TOUCH SCREEN  (y = 320 ~ 470)
    // ════════════════════════════════════════

    // Read touch state once per cycle
    TouchPoint_t tp;
    uint8_t touched = XPT2046_ReadTouch(&tp);
    uint8_t tap = touched && !prev_touched;  // rising edge = new tap

    // �????�???? Page state machine �????�????
    if (page_mode == 0) {
      // �????�???? MAIN PAGE: 4 touch buttons + settings opener �????�????
      LCD_Print(4, 320, "-- TOUCH --", LCD_YELLOW, LCD_BLACK);

      if (touched) {
        snprintf(buf, sizeof(buf), "X:%3d Y:%3d", (int)tp.x, (int)tp.y);
        LCD_Print(4, 340, buf, LCD_GREEN, LCD_BLACK);
      } else {
        LCD_Print(4, 340, "X:--- Y:---  ", LCD_GRAY, LCD_BLACK);
      }

      // [SYNC 0,0] button (x=4~150, y=370~400)
      uint16_t c1 = LCD_GRAY;
      if (touched && tp.x <= 150 && tp.y >= 370 && tp.y <= 400) {
        c1 = LCD_GREEN;
        if (!btn_sync_held) {
          btn_sync_held = 1;
          __disable_irq();
          g_odom_x = 0.0f; g_odom_y = 0.0f;
          __enable_irq();
          Move_InitPose(0.0f, 0.0f, g_imu_yaw);
        }
      } else { btn_sync_held = 0; }
      LCD_Print(4, 370, "[SYNC 0,0]   ", c1, LCD_BLACK);

      // [GOTO 0,0.3] button (x=4~150, y=410~440)
      uint16_t c2 = LCD_GRAY;
      if (touched && tp.x <= 150 && tp.y >= 410 && tp.y <= 440) {
        c2 = LCD_GREEN;
        if (!btn_goto_held) {
          btn_goto_held = 1;
          NavPacket_t nav;
          nav.cmd = NAV_CMD_GOTO;
          nav.f[0] = 0.0f; nav.f[1] = 0.3f;
          xQueueSend(NavQueueHandle, &nav, 10);
        }
      } else { btn_goto_held = 0; }
      LCD_Print(4, 410, "[GOTO 0,0.3] ", c2, LCD_BLACK);

      // [HOME] button (x=160~310, y=410~440)
      uint16_t c3 = LCD_GRAY;
      if (touched && tp.x >= 160 && tp.y >= 410 && tp.y <= 440) {
        c3 = LCD_GREEN;
        NavPacket_t nav;
        nav.cmd = NAV_CMD_GOTO;
        nav.f[0] = 0.0f; nav.f[1] = 0.0f;
        xQueueSend(NavQueueHandle, &nav, 10);
      }
      LCD_Print(160, 410, "[HOME]      ", c3, LCD_BLACK);

      // [SET POSE] button (x=160~310, y=370~400) �???? opens settings page
      uint16_t c4 = LCD_GRAY;
      if (touched && tp.x >= 160 && tp.y >= 370 && tp.y <= 400) {
        c4 = LCD_YELLOW;
        if (tap) {
          page_mode = 1;
          set_x_cm = 0;
          set_y_cm = 0;
          set_yaw_i = 0;
        }
      }
      LCD_Print(160, 370, "[SET POSE]  ", c4, LCD_BLACK);

      LCD_Print(4, 450, touch_ok ? "TOUCH: OK" : "TOUCH: FAIL",
                touch_ok ? LCD_GREEN : LCD_RED, LCD_BLACK);

    } else {
      // ══════════════════════════════════════
      //  SETTINGS PAGE: set X, Y, Yaw
      // ══════════════════════════════════════
      LCD_Print(4, 315, "-- SET POSE --", LCD_YELLOW, LCD_BLACK);

      // �????�???? Row 1: X (y=335~360) �????�????
      //  [X-]  X: +0.00 M  [X+]
      uint16_t xm_c = LCD_GRAY, xp_c = LCD_GRAY;
      if (touched && tp.x < 70 && tp.y >= 335 && tp.y <= 360) {
        xm_c = LCD_GREEN;
        if (tap && set_x_cm > -100) set_x_cm -= 5;
      }
      if (touched && tp.x > 250 && tp.y >= 335 && tp.y <= 360) {
        xp_c = LCD_GREEN;
        if (tap && set_x_cm < 100) set_x_cm += 5;
      }
      LCD_Print(4, 338, "[X-]", xm_c, LCD_BLACK);
      snprintf(buf, sizeof(buf), "X:%+.2f M", (double)(set_x_cm * 0.05f));
      LCD_Print(60, 338, buf, LCD_WHITE, LCD_BLACK);
      LCD_Print(260, 338, "[X+]", xp_c, LCD_BLACK);

      // �????�???? Row 2: Y (y=365~390) �????�????
      uint16_t ym_c = LCD_GRAY, yp_c = LCD_GRAY;
      if (touched && tp.x < 70 && tp.y >= 365 && tp.y <= 390) {
        ym_c = LCD_GREEN;
        if (tap && set_y_cm > -100) set_y_cm -= 5;
      }
      if (touched && tp.x > 250 && tp.y >= 365 && tp.y <= 390) {
        yp_c = LCD_GREEN;
        if (tap && set_y_cm < 100) set_y_cm += 5;
      }
      LCD_Print(4, 368, "[Y-]", ym_c, LCD_BLACK);
      snprintf(buf, sizeof(buf), "Y:%+.2f M", (double)(set_y_cm * 0.05f));
      LCD_Print(60, 368, buf, LCD_WHITE, LCD_BLACK);
      LCD_Print(260, 368, "[Y+]", yp_c, LCD_BLACK);

      // �????�???? Row 3: YAW (y=395~420) �????�????
      uint16_t ywm_c = LCD_GRAY, ywp_c = LCD_GRAY;
      if (touched && tp.x < 70 && tp.y >= 395 && tp.y <= 420) {
        ywm_c = LCD_GREEN;
        if (tap) set_yaw_i = (set_yaw_i + 7) % 8;  // -1 mod 8
      }
      if (touched && tp.x > 250 && tp.y >= 395 && tp.y <= 420) {
        ywp_c = LCD_GREEN;
        if (tap) set_yaw_i = (set_yaw_i + 1) % 8;   // +1 mod 8
      }
      LCD_Print(4, 398, "[A-]", ywm_c, LCD_BLACK);
      snprintf(buf, sizeof(buf), "YAW: %3d DEG", (int)set_yaw_i * 45);
      LCD_Print(60, 398, buf, LCD_WHITE, LCD_BLACK);
      LCD_Print(260, 398, "[A+]", ywp_c, LCD_BLACK);

      // �????�???? Row 4: CONFIRM / CANCEL (y=430~460) �????�????
      uint16_t cf_c = LCD_GRAY, cc_c = LCD_GRAY;
      if (touched && tp.x < 150 && tp.y >= 430 && tp.y <= 465) {
        cf_c = LCD_GREEN;
        if (tap) {
          // Apply: sync odometry to (x, y) with chosen yaw
          float sx = set_x_cm * 0.05f;
          float sy = set_y_cm * 0.05f;
          __disable_irq();
          g_odom_x = sx;
          g_odom_y = sy;
          __enable_irq();
          Move_InitPose(sx, sy, (float)set_yaw_i * 45.0f);
          page_mode = 0;
        }
      }
      if (touched && tp.x >= 160 && tp.y >= 430 && tp.y <= 465) {
        cc_c = LCD_RED;
        if (tap) {
          page_mode = 0;  // cancel, discard values
        }
      }
      LCD_Print(4, 435, "[CONFIRM]    ", cf_c, LCD_BLACK);
      LCD_Print(160, 435, "[CANCEL]     ", cc_c, LCD_BLACK);
    }

    prev_touched = touched;
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
  // PWM already initialized in main.c before FreeRTOS starts.
  // This task only handles on/off commands from PC (g_light_pending).

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

/* USER CODE BEGIN Header_StartTask10 */
/**
* @brief Function implementing the PathTestTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask10 */
void StartTask10(void const * argument)
{
  /* USER CODE BEGIN StartTask10 */
  StartPathTestTask(argument);
  /* USER CODE END StartTask10 */
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

        case NAV_CMD_PATH:
          result = MovePathTrack();
          SendNavResultToPC(TYPE_CMD_PATH_RESP, result);
          if (g_path_test_active) {
              g_path_test_last   = result;
              g_path_test_done  += 1;
              g_path_test_active = 0;
          }
          break;

        case NAV_CMD_ARC_TRACK: {
          float arc_radius  = nav.f[0];
          float arc_speed   = nav.f[1];
          int   arc_dir     = (nav.f[2] >= 0.0f) ? 1 : -1;
          float arc_sweep   = nav.f[3];
          uint32_t arc_to   = (uint32_t)MOVE_WP_TIMEOUT_MS;
          result = MoveArcTrack(arc_radius, arc_speed, arc_dir, arc_sweep, arc_to);
          SendNavResultToPC(TYPE_CMD_PATH_RESP, result);
          if (g_path_test_active) {
              g_path_test_last   = result;
              g_path_test_done  += 1;
              g_path_test_active = 0;
          }
          break;
        }

        case NAV_CMD_CALIB_HEIGHT: {
          /* 挂起OptFlowTask: cal_wait_done直接读PMW3901, 防SPI1竞争 */
          if (OptFlowTaskHandle) vTaskSuspend(OptFlowTaskHandle);
          OFlowCalibResult_t cal_res;
          uint8_t axis = (uint8_t)nav.f[0];
          float revs = nav.f[1];
          result = OFlowCalib_Height(axis, revs, &cal_res);
          /* 发回结果: status(1)+pix_to_m(4)+height(4)+distance(4)+pixels(4)+valid(4)+invalid(4)=25 */
          {
            uint8_t buf[32];
            buf[0] = PROTOCOL_HEADER1; buf[1] = PROTOCOL_HEADER2;
            buf[2] = TYPE_CMD_CALIB_HEIGHT_RESP;
            buf[3] = 25;  /* payload len */
            buf[4] = result;
            memcpy(&buf[5],  &cal_res.pix_to_m_result, 4);
            memcpy(&buf[9],  &cal_res.estimated_height_m, 4);
            memcpy(&buf[13], &cal_res.actual_distance_m, 4);
            int32_t tp = (axis == 0) ? cal_res.accum_dx_pixels : cal_res.accum_dy_pixels;
            memcpy(&buf[17], &tp, 4);
            memcpy(&buf[21], &cal_res.valid_samples, 4);
            memcpy(&buf[25], &cal_res.invalid_samples, 4);
            uint16_t crc = CRC16_CCITT(&buf[2], 27);  /* type+len+payload(25) */
            buf[29] = crc & 0xFF; buf[30] = (crc >> 8) & 0xFF;
            if (osMutexWait(Uart6MutexHandle, 100) == osOK) {
              HAL_UART_Transmit(&huart6, buf, 31, 50);
              osMutexRelease(Uart6MutexHandle);
            }
          }
          if (OptFlowTaskHandle) vTaskResume(OptFlowTaskHandle);
          break;
        }

        case NAV_CMD_CALIB_OFFSET: {
          float ox = 0.0f, oy = 0.0f;
          result = OFlowCalib_Offset(&ox, &oy);
          /* 发回结果: status(1)+ox(4)+oy(4)+dx(4)+dy(4)+squal(1)+obs(1)+total_deg(4)=23 */
          {
            uint8_t buf[36];
            buf[0] = PROTOCOL_HEADER1; buf[1] = PROTOCOL_HEADER2;
            buf[2] = TYPE_CMD_CALIB_OFFSET_RESP;
            buf[3] = 23;  /* payload len */
            buf[4] = result;
            memcpy(&buf[5], &ox, 4);
            memcpy(&buf[9], &oy, 4);
            /* 诊断数据: PMW3901 原始累计 (int32) + 表面质量 + 实际转角 */
            int32_t dbg_dx = calib_dbg_dx;
            int32_t dbg_dy = calib_dbg_dy;
            memcpy(&buf[13], &dbg_dx, 4);
            memcpy(&buf[17], &dbg_dy, 4);
            buf[21] = calib_dbg_squal;
            buf[22] = calib_dbg_obs;
            float dbg_deg = calib_dbg_total_deg;
            memcpy(&buf[23], &dbg_deg, 4);
            uint16_t crc = CRC16_CCITT(&buf[2], 25);  /* type+len+payload(23) */
            buf[27] = crc & 0xFF; buf[28] = (crc >> 8) & 0xFF;
            if (osMutexWait(Uart6MutexHandle, 100) == osOK) {
              HAL_UART_Transmit(&huart6, buf, 29, 50);
              osMutexRelease(Uart6MutexHandle);
            }
          }
          break;
        }

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

/* [调试�?,定位后删除] 路径跟踪遥测: 每控制周期发�?帧到上位�?,
 * 用于区分"控制器位置冻�?"vs"电机不执行命�?"。payload 32字节:
 * move_x(f32)+move_y(f32)+wp_idx(i16)+total(i16)+vx_f(f32)+vy_f(f32)+wz(f32)+target_yaw(f32)
 * +loop_ms(u16)+enc_ok(u8)+pad(u8) */
void SendPathDebugToPC(float mx, float my, int16_t wp_idx, int16_t total,
                       float vx_f, float vy_f, float wz, float target_yaw,
                       uint16_t loop_ms, uint8_t enc_ok)
{
  uint8_t buf[44];
  buf[0] = PROTOCOL_HEADER1; buf[1] = PROTOCOL_HEADER2;
  buf[2] = TYPE_CMD_PATH_DEBUG;
  buf[3] = 32;                       /* payload len */
  memcpy(&buf[4],  &mx,        4);
  memcpy(&buf[8],  &my,        4);
  memcpy(&buf[12], &wp_idx,    2);
  memcpy(&buf[14], &total,     2);
  memcpy(&buf[16], &vx_f,      4);
  memcpy(&buf[20], &vy_f,      4);
  memcpy(&buf[24], &wz,        4);
  memcpy(&buf[28], &target_yaw,4);
  memcpy(&buf[32], &loop_ms,   2);
  buf[34] = enc_ok;
  buf[35] = 0;                       /* pad */
  uint16_t crc = CRC16_CCITT(&buf[2], 34);   /* type+len+payload(32) */
  buf[36] = crc & 0xFF; buf[37] = (crc >> 8) & 0xFF;
  if (osMutexWait(Uart6MutexHandle, 100) == osOK) {
    HAL_UART_Transmit(&huart6, buf, 38, 50);
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
