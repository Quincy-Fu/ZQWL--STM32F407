/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "can.h"
#include "cmsis_os.h"
#include "uart_protocol.h"
#include "move.h"
#include <string.h>
extern osMessageQId DataQueueHandle;
extern osMessageQId NavQueueHandle;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// USART6 DMA接收解析 (上位机�?�信)
static volatile uint16_t dma_read_pos = 0;
static UartParser_t uart6_parser;

extern volatile uint8_t g_target_gear;

volatile uint32_t g_rx_cmd_vel_count = 0;
volatile uint32_t g_rx_rotate_count  = 0;
volatile uint32_t g_rx_arm_count     = 0;
volatile uint32_t g_rx_light_count   = 0;
volatile uint32_t g_rx_nav_count     = 0;

// LIGHT command pending flag (consumed by LightTask)
volatile uint8_t g_light_pending_id = 0;   // 0=all, 1-3=specific light
volatile uint8_t g_light_pending_on = 0;   // 0=off, 1=on
volatile uint8_t g_light_pending    = 0;   // 1=new command pending

// ROTATE command pending flag (consumed by PosMotorTask, 执行完回 TYPE_ROTATE_RESP)
volatile uint8_t g_rotate_pending_pos = 0; // 目标槽位 0-4
volatile uint8_t g_rotate_pending     = 0; // 1=new command pending

// ARM command pending flag (consumed by ServoTask, 执行完回 TYPE_ARM_RESP)
volatile uint8_t g_arm_pending = 0;        // 1=new command pending

// ARM servo state (consumed by ServoTask)
// ARM payload: [state(1B)] = 1 byte
//   state: 0-7, each maps to a predefined (servo1_angle, servo2_angle) pair
//   0 = power-on default pose; 与上位机约定统一: 从0起始
volatile uint8_t g_arm_state = 0;    // 0=default pose, 1-7=arm pose state

static inline void dispatch_frame(uint8_t type, const uint8_t *payload, uint8_t len)
{
    /* 队列在调度器启动后才创建; 上电早期若已收到帧, 直接丢弃防 NULL 句柄 */
    if (!NavQueueHandle || !DataQueueHandle) return;

    if (type == TYPE_CMD_VEL && len == PAYLOAD_SIZE_VEL) {
        DataPacket_t pkt;
        pkt.len = 12;
        memcpy(&pkt.data[0], payload, 12);
        BaseType_t xHigher = pdFALSE;
        xQueueSendFromISR(DataQueueHandle, &pkt, &xHigher);
        portYIELD_FROM_ISR(xHigher);
        g_rx_cmd_vel_count++;
    } else if (type == TYPE_ROTATE && len >= 1) {
        uint8_t pos = payload[0];
        if (pos < 5) {
            g_target_gear = pos;          /* LCD 显示用 */
            g_rotate_pending_pos = pos;   /* PosMotorTask 执行+回响应 */
            g_rotate_pending = 1;
        }
        g_rx_rotate_count++;
    } else if (type == TYPE_ARM) {
        if (len >= 1) {
            uint8_t state = payload[0];
            if (state <= 7) {          /* 0-7, 与上位机统一从0起始 */
                g_arm_state = state;
                g_arm_pending = 1;     /* ServoTask 执行+回响应 */
            }
        }
        g_rx_arm_count++;
    } else if (type == TYPE_LIGHT) {
        if (len >= 2) {
            g_light_pending_id = payload[0];  // 0=all, 1-4=specific (4=PA3/TIM5)
            g_light_pending_on = payload[1];  // 0=off, nonzero=on
            g_light_pending = 1;
        }
        g_rx_light_count++;
    } else if (type == TYPE_RUN && len == 0) {
        NavPacket_t nav; nav.cmd = NAV_CMD_RUN;
        BaseType_t h = pdFALSE;
        xQueueSendFromISR(NavQueueHandle, &nav, &h);
        portYIELD_FROM_ISR(h);
    }
    // --- Navigation commands -> NavQueue (Stage 3) ---
    else if (type == TYPE_CMD_GOTO && len == 8) {
        NavPacket_t nav; nav.cmd = NAV_CMD_GOTO;
        memcpy(&nav.f[0], payload, 4);
        memcpy(&nav.f[1], payload + 4, 4);
        BaseType_t h = pdFALSE;
        xQueueSendFromISR(NavQueueHandle, &nav, &h);
        portYIELD_FROM_ISR(h);
        g_rx_nav_count++;
    }
    else if (type == TYPE_CMD_TOX && len == 4) {
        NavPacket_t nav; nav.cmd = NAV_CMD_TOX;
        memcpy(&nav.f[0], payload, 4);
        BaseType_t h = pdFALSE;
        xQueueSendFromISR(NavQueueHandle, &nav, &h);
        portYIELD_FROM_ISR(h);
        g_rx_nav_count++;
    }
    else if (type == TYPE_CMD_TOY && len == 4) {
        NavPacket_t nav; nav.cmd = NAV_CMD_TOY;
        memcpy(&nav.f[0], payload, 4);
        BaseType_t h = pdFALSE;
        xQueueSendFromISR(NavQueueHandle, &nav, &h);
        portYIELD_FROM_ISR(h);
        g_rx_nav_count++;
    }
    else if (type == TYPE_CMD_TURNTO && len == 4) {
        NavPacket_t nav; nav.cmd = NAV_CMD_TURNTO;
        memcpy(&nav.f[0], payload, 4);
        BaseType_t h = pdFALSE;
        xQueueSendFromISR(NavQueueHandle, &nav, &h);
        portYIELD_FROM_ISR(h);
        g_rx_nav_count++;
    }
    else if (type == TYPE_CMD_FINE_MOVE && len == 8) {
        NavPacket_t nav; nav.cmd = NAV_CMD_FINE_MOVE;
        memcpy(&nav.f[0], payload, 4);
        memcpy(&nav.f[1], payload + 4, 4);
        BaseType_t h = pdFALSE;
        xQueueSendFromISR(NavQueueHandle, &nav, &h);
        portYIELD_FROM_ISR(h);
        g_rx_nav_count++;
    }
    else if (type == TYPE_CMD_SYNC_POSE && len == 8) {
        NavPacket_t nav; nav.cmd = NAV_CMD_SYNC_POSE;
        memcpy(&nav.f[0], payload, 4);
        memcpy(&nav.f[1], payload + 4, 4);
        BaseType_t h = pdFALSE;
        xQueueSendFromISR(NavQueueHandle, &nav, &h);
        portYIELD_FROM_ISR(h);
        g_rx_nav_count++;
    }
    else if (type == TYPE_CMD_ARC && (len == 12 || len == 16)) {
        /* 新圆弧语义: f[0]=半径m, f[1]=方向(+1右转/-1左转), f[2]=扫过角度°,
         * f[3]=速度m/s(可选, 12B帧时由NavTask用默认速度). 圆心由下位机按当前位姿自动算. */
        NavPacket_t nav; nav.cmd = NAV_CMD_ARC;
        nav.f[0] = 0.0f; nav.f[1] = 0.0f; nav.f[2] = 0.0f; nav.f[3] = 0.0f;
        memcpy(&nav.f[0], payload, 4);
        memcpy(&nav.f[1], payload + 4, 4);
        memcpy(&nav.f[2], payload + 8, 4);
        if (len == 16) memcpy(&nav.f[3], payload + 12, 4);
        BaseType_t h = pdFALSE;
        xQueueSendFromISR(NavQueueHandle, &nav, &h);
        portYIELD_FROM_ISR(h);
        g_rx_nav_count++;
    }
    else if (type == TYPE_CMD_CALIB_HEIGHT && len == 8) {
        NavPacket_t nav; nav.cmd = NAV_CMD_CALIB_HEIGHT;
        memcpy(&nav.f[0], payload, 4);      /* axis (0=Y, 1=X) */
        memcpy(&nav.f[1], payload + 4, 4);  /* num_revolutions */
        BaseType_t h = pdFALSE;
        xQueueSendFromISR(NavQueueHandle, &nav, &h);
        portYIELD_FROM_ISR(h);
        g_rx_nav_count++;
    }
    else if (type == TYPE_CMD_CALIB_OFFSET && len == 4) {
        NavPacket_t nav; nav.cmd = NAV_CMD_CALIB_OFFSET;
        BaseType_t h = pdFALSE;
        xQueueSendFromISR(NavQueueHandle, &nav, &h);
        portYIELD_FROM_ISR(h);
        g_rx_nav_count++;
    }
    // --- 路径点跟�?? ---
    else if (type == TYPE_CMD_PATH_BEGIN && len == 5) {
        float speed; memcpy(&speed, payload, 4);
        uint8_t count = payload[4];
        Move_PathBegin(count, speed);   /* 仅写全局缓冲, 不入�?? */
        g_rx_nav_count++;
    }
    else if (type == TYPE_CMD_PATH_POINT && len == 16) {
        float x, y, target_theta;
        memcpy(&x,            payload,      4);
        memcpy(&y,            payload + 4,  4);
        memcpy(&target_theta, payload + 8,  4);
        uint8_t mode = payload[12];
        Move_PathAddPoint(x, y, target_theta, mode);  /* 追加到全�??缓冲, 不入�?? */
        g_rx_nav_count++;
    }
    else if (type == TYPE_CMD_PATH_EXEC) {
        NavPacket_t nav; nav.cmd = NAV_CMD_PATH;
        BaseType_t h = pdFALSE;
        xQueueSendFromISR(NavQueueHandle, &nav, &h);
        portYIELD_FROM_ISR(h);
        g_rx_nav_count++;
    }
    // --- 视觉微调 (Stage 4) ---
    else if (type == TYPE_CMD_VISION_NUDGE && len == 1) {
        NavPacket_t nav; nav.cmd = NAV_CMD_VISION_NUDGE;
        nav.f[0] = (float)payload[0];   /* direction: 0=stop+lock, 1=fwd, 2=back, 3=left, 4=right */
        BaseType_t h = pdFALSE;
        xQueueSendFromISR(NavQueueHandle, &nav, &h);
        portYIELD_FROM_ISR(h);
        g_rx_nav_count++;
    }
}
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;
extern TIM_HandleTypeDef htim1;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles CAN1 RX0 interrupts.
  */
void CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX0_IRQn 0 */

  uint8_t i = 0;
  /* Drain ALL pending messages from FIFO to prevent frame loss.
   * Old code only read 1 frame per ISR; remaining frames get stuck
   * because the FIFO pending interrupt doesn't re-fire.
   *
   * Snapshot fix: 每次 GetRxMessage 后保存完整帧�???? rxSnap/rxSnapData.
   * move.c / OdomTask 从快照读�????, 避免共享 rxData 被后续帧覆盖
   * 导致部分读取 (符号字节被篡�???? �???? 编码器符号翻�???? �???? 里程计爆�????).
   * drain循环中多次覆盖快�????, �????终保留最后一�???? �???? 这是�????佳可用帧. */
  while(HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0)
  {
    if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, (CAN_RxHeaderTypeDef *)(&can.CAN_RxMsg), (uint8_t *)(&can.rxData)) == HAL_OK)
    {
      for(i=can.CAN_RxMsg.DLC; i < 8; i++) { can.rxData[i] = 0; }
      /* 保存快照: 完整帧头 + 数据副本 */
      can.rxSnap = can.CAN_RxMsg;
      for(i=0; i<8; i++) { can.rxSnapData[i] = can.rxData[i]; }
      can.rxFrameFlag = true;
    }
  }

  /* USER CODE END CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX0_IRQn 1 */

  /* USER CODE END CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
  */
void TIM1_UP_TIM10_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 0 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream1 global interrupt.
  */
void DMA2_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream1_IRQn 0 */

  /* USER CODE END DMA2_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart6_rx);
  /* USER CODE BEGIN DMA2_Stream1_IRQn 1 */

  /* USER CODE END DMA2_Stream1_IRQn 1 */
}

/**
  * @brief This function handles USART6 global interrupt.
  */
void USART6_IRQHandler(void)
{
  /* USER CODE BEGIN USART6_IRQn 0 */

  /* USER CODE END USART6_IRQn 0 */
  HAL_UART_IRQHandler(&huart6);
  /* USER CODE BEGIN USART6_IRQn 1 */
  // USART6 空闲中断: DMA接收 + 协议解析 (上位机�?�信)
  if (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_IDLE) != RESET) {
    __HAL_UART_CLEAR_IDLEFLAG(&huart6);

    // 计算DMA当前写位�???????
    uint16_t write_pos = RX_BUF_SIZE
        - __HAL_DMA_GET_COUNTER(&hdma_usart6_rx);

    // 处理新字�???????
    if (write_pos != dma_read_pos) {
      if (write_pos > dma_read_pos) {
        for (uint16_t i = dma_read_pos; i < write_pos; i++) {
          uint8_t out_type, out_payload[64], out_len;
          if (UartParser_FeedByte(&uart6_parser, RxDMA_Buf[i],
              &out_type, out_payload, &out_len)) {
            dispatch_frame(out_type, out_payload, out_len);
          }
        }
      } else {
        // 缓冲区回�??????? (CIRCULAR模式或NORMAL重启�???????)
        for (uint16_t i = dma_read_pos; i < RX_BUF_SIZE; i++) {
          uint8_t out_type, out_payload[64], out_len;
          if (UartParser_FeedByte(&uart6_parser, RxDMA_Buf[i],
              &out_type, out_payload, &out_len)) {
            dispatch_frame(out_type, out_payload, out_len);
          }
        }
        for (uint16_t i = 0; i < write_pos; i++) {
          uint8_t out_type, out_payload[64], out_len;
          if (UartParser_FeedByte(&uart6_parser, RxDMA_Buf[i],
              &out_type, out_payload, &out_len)) {
            dispatch_frame(out_type, out_payload, out_len);
          }
        }
      }
      dma_read_pos = write_pos;
    }

    // DMA NORMAL模式: 缓冲区满后DMA停止, �???????要重�???????
    if (huart6.RxState == HAL_UART_STATE_READY) {
      dma_read_pos = 0;
      HAL_UART_Receive_DMA(&huart6, RxDMA_Buf, RX_BUF_SIZE);
    }
  }
  /* USER CODE END USART6_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
