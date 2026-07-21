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
#include <math.h>
#include <stdio.h>
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

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

// ===== Velocity command input =====
// Set by CommTask (future) or Keil debugger
// Unit: vx,vy = m/s x 100, omega = rad/s x 100
volatile int16_t g_tgt_vx    = 0;
volatile int16_t g_tgt_vy    = 0;
volatile int16_t g_tgt_omega = 0;

// ===== Odometry output (THE core result) =====
// Updated by OdomTask, read by CommTask / DisplayTask / Keil debugger
// World frame, origin = power-on position & heading
//   x: forward (m), y: left (m), theta: CCW heading (rad)
volatile float g_odom_x     = 0.0f;   // position x (m)
volatile float g_odom_y     = 0.0f;   // position y (m)
volatile float g_odom_theta = 0.0f;   // heading (rad), CCW positive

// ===== IMU yaw (internal, used by OdomTask) =====
// Unit: degrees (imu_protocol.c converts raw rad -> deg), CCW positive
// Zero at power-on heading
volatile float g_imu_yaw = 0.0f;

// IMU status: 1 = verified OK
volatile uint8_t g_imu_verified = 0;

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId MotorTaskHandle;
osThreadId OdomTaskHandle;
osThreadId OptFlowTaskHandle;
osThreadId ImuTaskHandle;
osThreadId DisplayTaskHandle;
osThreadId ServoTaskHandle;
osThreadId LightTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

// Signed RPM -> Emm_V5 velocity command (dir + abs vel)
// is_right: right-side motors mirror-mounted, positive RPM -> dir=1(CCW)
static void motor_emit(uint8_t addr, float rpm_signed, bool is_right);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);
void StartOdomTask(void const * argument);
void StartOptFlowTask(void const * argument);
void StartImuTask(void const * argument);
void StartDisplayTask(void const * argument);
void StartServoTask(void const * argument);
void StartLightTask(void const * argument);

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

  /* definition and creation of ServoTask */
  osThreadDef(ServoTask, StartServoTask, osPriorityNormal, 0, 512);
  ServoTaskHandle = osThreadCreate(osThread(ServoTask), NULL);

  /* definition and creation of LightTask */
  osThreadDef(LightTask, StartLightTask, osPriorityLow, 0, 512);
  LightTaskHandle = osThreadCreate(osThread(LightTask), NULL);

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

  // Motor test disabled: do not enable drivers, do not send velocity commands
  // To re-enable: restore Emm_V5_En_Control + velocity loop
  for(;;) {
    osDelay(100);
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
        float dx_body = (d_FL + d_FR + d_RL + d_RR) * 0.25f;
        float dy_body = (-d_FL + d_FR + d_RL - d_RR) * 0.25f;

        // Heading from IMU (g_imu_yaw in degrees -> radians for trig)
        float theta = g_imu_yaw * 0.01745329f;

        // Rotate body -> world frame and accumulate
        float ct = cosf(theta);
        float st = sinf(theta);
        __disable_irq();
        g_odom_x     += dx_body * ct - dy_body * st;
        g_odom_y     += dx_body * st + dy_body * ct;
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

  // PMW3901 optical flow not in use; suspend immediately
  vTaskSuspend(NULL);

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
  LCD_Init();

  LCD_Clear(LCD_BLACK);

  char buf[40];
  for(;;)
  {
    float ox, oy, ot, iy;
    uint8_t imu_ok;
    uint32_t fc;
    __disable_irq();
    ox = g_odom_x;
    oy = g_odom_y;
    ot = g_odom_theta;
    iy = g_imu_yaw;
    imu_ok = g_imu_verified;
    fc = imu_frame_count;
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
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
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
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartLightTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

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
