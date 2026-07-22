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
#include <string.h>
extern osMessageQId DataQueueHandle;
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

static inline void dispatch_frame(uint8_t type, const uint8_t *payload, uint8_t len)
{
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
        if (pos < 5) g_target_gear = pos;
        g_rx_rotate_count++;
    } else if (type == TYPE_ARM) {
        g_rx_arm_count++;
    } else if (type == TYPE_LIGHT) {
        g_rx_light_count++;
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
  if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, (CAN_RxHeaderTypeDef *)(&can.CAN_RxMsg), (uint8_t *)(&can.rxData)) == HAL_OK)
  {
    for(i=can.CAN_RxMsg.DLC; i < 8; i++) { can.rxData[i] = 0; } can.rxFrameFlag = true;
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

    // 计算DMA当前写位�?
    uint16_t write_pos = RX_BUF_SIZE
        - __HAL_DMA_GET_COUNTER(&hdma_usart6_rx);

    // 处理新字�?
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
        // 缓冲区回�? (CIRCULAR模式或NORMAL重启�?)
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

    // DMA NORMAL模式: 缓冲区满后DMA停止, �?要重�?
    if (huart6.RxState == HAL_UART_STATE_READY) {
      dma_read_pos = 0;
      HAL_UART_Receive_DMA(&huart6, RxDMA_Buf, RX_BUF_SIZE);
    }
  }
  /* USER CODE END USART6_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
