/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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
#include "can.h"
#include "cmsis_os.h"
#include "shared_vars.h"

/* USER CODE BEGIN 0 */

__IO CAN_t can = {0};

/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_9TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  /* Enable AutoBusOff management (CubeMX sets DISABLE, resets on regen).
   * ABOM lets CAN auto-recover from bus-off state caused by errors. */
  CAN1->MCR |= CAN_MCR_ABOM;
  /* USER CODE END CAN1_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PB8     ------> CAN1_RX
    PB9     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */
  /* Override CubeMX priority 5 �?? 0 (highest, matches vendor example).
   * CubeMX resets line 89 to 5 on regeneration; this override is safe. */
  HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN1 GPIO Configuration
    PB8     ------> CAN1_RX
    PB9     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/**
	* @brief   初始化过滤器
	*/
void USER_CAN1_Filter_Init(void)
{
	CAN_FilterTypeDef  sFilterConfig;

	__IO uint8_t id_o, im_o; __IO uint16_t id_l, id_h, im_l, im_h;
	id_o = (0x00);
	id_h = (uint16_t)((uint16_t)id_o >> 5);
	id_l = (uint16_t)((uint16_t)id_o << 11) | CAN_ID_EXT;
	im_o = (0x00);
	im_h = (uint16_t)((uint16_t)im_o >> 5);
	im_l = (uint16_t)((uint16_t)im_o << 11) | CAN_ID_EXT;

	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = id_h;
	sFilterConfig.FilterIdLow = id_l;
	sFilterConfig.FilterMaskIdHigh = im_h;
	sFilterConfig.FilterMaskIdLow = im_l;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 0;

	while(HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK);
}

/**
	* @brief   CAN发�?�多字节
	* @param   cmd  命令缓冲（cmd[0]=addr，后续为功能�?????+数据+0x6B�?????
	* @param   len  长度（含 addr �????? 0x6B�?????
	*/
void can_SendCmd(__IO uint8_t *cmd, uint8_t len)
{
	static uint32_t TxMailbox; __IO uint8_t i = 0, j = 0, k = 0, l = 0, packNum = 0;
	CAN_TxHeaderTypeDef TxMsg = {0};
	uint8_t txData[8] = {0};
	uint8_t locked = 0;

	if ((CanTxMutexHandle != NULL) && (osKernelRunning() != 0)) {
		if (osMutexWait(CanTxMutexHandle, osWaitForever) == osOK) {
			locked = 1;
		}
	}

	// 去掉ID地址和校验，数据长度
	j = len - 2;

	while(i < j)
	{
		k = j - i;

		TxMsg.StdId = 0x00;
		TxMsg.ExtId = ((uint32_t)cmd[0] << 8) | (uint32_t)packNum;
		txData[0] = cmd[1];
		TxMsg.IDE = CAN_ID_EXT;
		TxMsg.RTR = CAN_RTR_DATA;

		// 小于8字节，最后一�?????
		if(k < 8)
		{
			for(l=0; l < k; l++,i++) { txData[l + 1] = cmd[i + 2]; } TxMsg.DLC = k + 1;
		}
		// 大于8字节，分包发送，每次�?????多发8个字�?????
		else
		{
			for(l=0; l < 7; l++,i++) { txData[l + 1] = cmd[i + 2]; } TxMsg.DLC = 8;
		}

		while(HAL_CAN_AddTxMessage(&hcan1, (CAN_TxHeaderTypeDef *)(&TxMsg), (uint8_t *)(&txData), (&TxMailbox)) != HAL_OK);

		++packNum;
		if (i < j) {
			HAL_Delay(10);  // inter-frame delay for Emm_V5 RX buffer spacing (multi-frame only)
		}
	}

	if (locked) {
		osMutexRelease(CanTxMutexHandle);
	}
}

/* USER CODE END 1 */
