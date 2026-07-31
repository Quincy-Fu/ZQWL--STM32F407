/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SPI1_CS3_Pin GPIO_PIN_7
#define SPI1_CS3_GPIO_Port GPIOE
#define SPI1_CS2_Pin GPIO_PIN_8
#define SPI1_CS2_GPIO_Port GPIOE
#define SPI1_CS1_Pin GPIO_PIN_9
#define SPI1_CS1_GPIO_Port GPIOE
#define IMU_TX_Pin GPIO_PIN_9
#define IMU_TX_GPIO_Port GPIOA
#define IMU_RX_Pin GPIO_PIN_10
#define IMU_RX_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

typedef struct {
    uint8_t data[128];
    uint16_t len;
} DataPacket_t;

#define RX_BUF_SIZE  256

extern uint8_t RxDMA_Buf[RX_BUF_SIZE];
extern volatile uint8_t currentHalf;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
