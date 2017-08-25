/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V.
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include "can.h"
#include "can2.h"
#include "serial.h"
#include "oled2004.h"
#include "driverDisplay.h"
#include "nodeMiscHelpers.h"
#include "nodeConf.h"
#include "../../CAN_ID.h"

// RTOS Task functions + helpers
#include "Can_Processor.h"
#include "motCan_Processor.h"
#include "Node_Manager.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi3_rx;
DMA_HandleTypeDef hdma_spi3_tx;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

WWDG_HandleTypeDef hwwdg;

osThreadId Can_ProcessorHandle;
osThreadId MotCanProcessorHandle;
osThreadId Switch_ReaderHandle;
osThreadId NodeManagerHandle;
osMessageQId mainCanTxQHandle;
osMessageQId mainCanRxQHandle;
osMessageQId motCanRxQHandle;
osMessageQId motCanTxQHandle;
osMessageQId BadNodesQHandle;
osTimerId WWDGTmrHandle;
osTimerId LSigTmrHandle;
osTimerId RSigTmrHandle;
osMutexId swMtxHandle;
osMutexId controlVarsMtxHandle;
osSemaphoreId motReqSemHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t ackPressed = 0;
OLED_HandleTypeDef holed1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_WWDG_Init(void);
static void MX_SPI3_Init(void);
void doProcessCan(void const * argument);
void doMotCan(void const * argument);
void doSwitches(void const * argument);
void doNodeManager(void const * argument);
void TmrKickDog(void const * argument);
void toggleLSig(void const * argument);
void toggleRSig(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_CAN_TxCpltCallback(CAN_HandleTypeDef* hcan);
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan);
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan);
void TmrHBTimeout(void * argument);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
QueueHandle_t * nodeEntryMtxHandle = (QueueHandle_t[MAX_NODE_NUM]){0};
osTimerId * nodeTmrHandle = (osTimerId[MAX_NODE_NUM]){0};			// Timer for each node's timeout timer
nodeEntry * nodeTable = (nodeEntry[MAX_NODE_NUM]){0};
controlVars userInput;
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	selfStatusWord = ACTIVE;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_USART2_UART_Init();
  MX_WWDG_Init();
  MX_SPI3_Init();

  /* USER CODE BEGIN 2 */
	OLED_init(&holed1, &hspi3, SCREEN_CS_GPIO_Port, SCREEN_CS_Pin);

  setupNodeTable();
  nodeTable[cc_nodeID].nodeStatusWord = ACTIVE;		// Set initial status to ACTIVE

  Serial2_begin();
	Serial2_writeBuf("Booting... \n");

	bxCan_begin(&hcan1, &mainCanRxQHandle, &mainCanTxQHandle);
	bxCan_addMaskedFilterStd(swOffset,0xFF0,0); // Filter: Status word group (ignore nodeID)
    bxCan_addMaskedFilterStd(fwOffset,0xFF0,0); // Filter: Firmware version group (ignore nodeID)
    bxCan_addMaskedFilterStd(p2pOffset,0xFF0,0); // Filter: p2p command group (ignore nodeID)
	bxCan_addMaskedFilterStd(0x201, 0x7FF, 0);

    bxCan2_begin(&hcan2, &motCanRxQHandle, &motCanTxQHandle);
	bxCan2_addMaskedFilterStd(0,0,0);
	bxCan2_addMaskedFilterExt(0,0,0);

	// bxCan2_begin(&hcan2, &motCanRxQHandle, &mainCanTxQHandle);
	// bxCan_addMaskedFilterStd(p2pOffset,0xFF0,0);
  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of swMtx */
  osMutexDef(swMtx);
  swMtxHandle = osMutexCreate(osMutex(swMtx));

  /* definition and creation of controlVarsMtx */
  osMutexDef(controlVarsMtx);
  controlVarsMtxHandle = osMutexCreate(osMutex(controlVarsMtx));

  /* USER CODE BEGIN RTOS_MUTEX */
  // Node table entry mutex
  // Every entry has a mutex that is associated with the nodeID
  for(uint8_t i =0; i < MAX_NODE_NUM; i++){
	  osMutexDef(i);
	  nodeEntryMtxHandle[i] = (QueueHandle_t)osMutexCreate(osMutex(i));
  }
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of motReqSem */
  osSemaphoreDef(motReqSem);
  motReqSemHandle = osSemaphoreCreate(osSemaphore(motReqSem), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of WWDGTmr */
  osTimerDef(WWDGTmr, TmrKickDog);
  WWDGTmrHandle = osTimerCreate(osTimer(WWDGTmr), osTimerPeriodic, NULL);

  /* definition and creation of LSigTmr */
  osTimerDef(LSigTmr, toggleLSig);
  LSigTmrHandle = osTimerCreate(osTimer(LSigTmr), osTimerPeriodic, NULL);

  /* definition and creation of RSigTmr */
  osTimerDef(RSigTmr, toggleRSig);
  RSigTmrHandle = osTimerCreate(osTimer(RSigTmr), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  xTimerChangePeriod(LSigTmrHandle,Turn_sig_Interval,portMAX_DELAY);
  xTimerStop(LSigTmrHandle,portMAX_DELAY);
  xTimerChangePeriod(RSigTmrHandle,Turn_sig_Interval,portMAX_DELAY);
  xTimerStop(RSigTmrHandle,portMAX_DELAY);
  osTimerStart(WWDGTmrHandle, WD_Interval);

  // Node heartbeat timeout timers
//    for(uint8_t TmrID = 0; TmrID < MAX_NODE_NUM; TmrID++){
//  	  osTimerDef(TmrID, TmrHBTimeout);
//  	  // TODO: Consider passing the nodeTmrHandle+Offset or NULL
//  	  nodeTmrHandle[TmrID] = osTimerCreate(osTimer(TmrID), osTimerOnce, (void*)TmrID);	// TmrID here is stored directly as a variable
//  	  //DISCUSS changePeriod starts the damn timers...
//  	  xTimerChangePeriod(nodeTmrHandle[TmrID], Node_HB_Interval, portMAX_DELAY);
//  	  xTimerStop(nodeTmrHandle[TmrID], portMAX_DELAY);
//  	  //TODO investigate the timer crashes
//  	  // One-shot timer since it should be refreshed by the Can Processor upon node HB reception
//    }     //fucktarded the macro oops
	for(uint8_t TmrID = 0; TmrID < MAX_NODE_NUM; TmrID++){
		nodeTmrHandle[TmrID] = xTimerCreate(NULL,Node_HB_Interval,pdFALSE,(void*)TmrID,TmrHBTimeout);
	}
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of Can_Processor */
  osThreadDef(Can_Processor, doProcessCan, osPriorityRealtime, 0, 512);
  Can_ProcessorHandle = osThreadCreate(osThread(Can_Processor), NULL);

  /* definition and creation of MotCanProcessor */
  osThreadDef(MotCanProcessor, doMotCan, osPriorityAboveNormal, 0, 512);
  MotCanProcessorHandle = osThreadCreate(osThread(MotCanProcessor), NULL);

  /* definition and creation of Switch_Reader */
  osThreadDef(Switch_Reader, doSwitches, osPriorityAboveNormal, 0, 512);
  Switch_ReaderHandle = osThreadCreate(osThread(Switch_Reader), NULL);

  /* definition and creation of NodeManager */
  osThreadDef(NodeManager, doNodeManager, osPriorityHigh, 0, 512);
  NodeManagerHandle = osThreadCreate(osThread(NodeManager), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of mainCanTxQ */
  osMessageQDef(mainCanTxQ, 32, Can_frame_t);
  mainCanTxQHandle = osMessageCreate(osMessageQ(mainCanTxQ), NULL);

  /* definition and creation of mainCanRxQ */
  osMessageQDef(mainCanRxQ, 32, Can_frame_t);
  mainCanRxQHandle = osMessageCreate(osMessageQ(mainCanRxQ), NULL);

  /* definition and creation of motCanRxQ */
  osMessageQDef(motCanRxQ, 32, Can_frame_t);
  motCanRxQHandle = osMessageCreate(osMessageQ(motCanRxQ), NULL);

  /* definition and creation of motCanTxQ */
  osMessageQDef(motCanTxQ, 16, Can_frame_t);
  motCanTxQHandle = osMessageCreate(osMessageQ(motCanTxQ), NULL);

  /* definition and creation of BadNodesQ */
  osMessageQDef(BadNodesQ, 32, uint8_t);
  BadNodesQHandle = osMessageCreate(osMessageQ(BadNodesQ), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */


  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* CAN1 init function */
static void MX_CAN1_Init(void)
{

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 5;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SJW = CAN_SJW_1TQ;
  hcan1.Init.BS1 = CAN_BS1_13TQ;
  hcan1.Init.BS2 = CAN_BS2_2TQ;
  hcan1.Init.TTCM = DISABLE;
  hcan1.Init.ABOM = ENABLE;
  hcan1.Init.AWUM = DISABLE;
  hcan1.Init.NART = DISABLE;
  hcan1.Init.RFLM = DISABLE;
  hcan1.Init.TXFP = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* CAN2 init function */
static void MX_CAN2_Init(void)
{

  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 10;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SJW = CAN_SJW_1TQ;
  hcan2.Init.BS1 = CAN_BS1_13TQ;
  hcan2.Init.BS2 = CAN_BS2_2TQ;
  hcan2.Init.TTCM = DISABLE;
  hcan2.Init.ABOM = ENABLE;
  hcan2.Init.AWUM = DISABLE;
  hcan2.Init.NART = DISABLE;
  hcan2.Init.RFLM = DISABLE;
  hcan2.Init.TXFP = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI3 init function */
static void MX_SPI3_Init(void)
{

  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 230400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* WWDG init function */
static void MX_WWDG_Init(void)
{

  hwwdg.Instance = WWDG;
  hwwdg.Init.Prescaler = WWDG_PRESCALER_8;
  hwwdg.Init.Window = 127;
  hwwdg.Init.Counter = 127;
  hwwdg.Init.EWIMode = WWDG_EWI_DISABLE;
  if (HAL_WWDG_Init(&hwwdg) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);

}

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
        * Free pins are configured automatically as Analog (this feature is enabled through
        * the Code Generation settings)
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SCREEN_CS_GPIO_Port, SCREEN_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, ENGAGE_LIGHT_Pin|RIGHT_LIGHT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LEFT_LIGHT_GPIO_Port, LEFT_LIGHT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BRK_LIGHT_GPIO_Port, BRK_LIGHT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LEFT_SIG_SWITCH_Pin */
  GPIO_InitStruct.Pin = LEFT_SIG_SWITCH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(LEFT_SIG_SWITCH_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SCREEN_L_BTN_Pin SCREEN_R_BTN_Pin */
  GPIO_InitStruct.Pin = SCREEN_L_BTN_Pin|SCREEN_R_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA6 PA7
                           PA8 PA9 PA10 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SCREEN_CS_Pin */
  GPIO_InitStruct.Pin = SCREEN_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SCREEN_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ENGAGE_LIGHT_Pin RIGHT_LIGHT_Pin */
  GPIO_InitStruct.Pin = ENGAGE_LIGHT_Pin|RIGHT_LIGHT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : ACK_BTN_Pin */
  GPIO_InitStruct.Pin = ACK_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ACK_BTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BRK_SWITCH_Pin RIGHT_SIG_SWITCH_Pin HAZARD_SWITCH_Pin */
  GPIO_InitStruct.Pin = BRK_SWITCH_Pin|RIGHT_SIG_SWITCH_Pin|HAZARD_SWITCH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB2 PB10 PB12
                           PB13 PB3 PB4 PB8
                           PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_12
                          |GPIO_PIN_13|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_8
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PC6 PC7 PC8 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LEFT_LIGHT_Pin */
  GPIO_InitStruct.Pin = LEFT_LIGHT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LEFT_LIGHT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BRK_LIGHT_Pin */
  GPIO_InitStruct.Pin = BRK_LIGHT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BRK_LIGHT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void TmrHBTimeout(void * argument){
	// TODO: Test if using point in the line below breaks this function
	uint8_t timerID = (uint8_t)pvTimerGetTimerID((TimerHandle_t)argument);
	nodeTable[timerID].nodeConnectionState = UNRELIABLE;
	if((timerID) != mc_nodeID){
		xQueueSend(BadNodesQHandle, &timerID, portMAX_DELAY);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	switch(GPIO_Pin){
	case ACK_BTN_Pin:
		ackPressed = 1;
		break;
	default:
		break;
	}
}

void HAL_CAN_TxCpltCallback(CAN_HandleTypeDef* hcan){
  if(hcan==&hcan1){
    CAN1_TxCpltCallback(hcan);
  }else if(hcan==&hcan2){
    CAN2_TxCpltCallback(hcan);
  }
}

void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan){
  if(hcan==&hcan1){
    CAN1_RxCpltCallback(hcan);
  }else if(hcan==&hcan2){
    CAN2_RxCpltCallback(hcan);
  }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan){
  if(hcan==&hcan1){
    CAN1_ErrorCallback(hcan);
  }else if(hcan==&hcan2){
    CAN2_ErrorCallback(hcan);
  }
}

/* USER CODE END 4 */

/* doProcessCan function */
void doProcessCan(void const * argument)
{

  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;){
	  // Wrapper function for the CAN Processing Logic
	  // Handles all CAN Protocol Suite based responses and tasks
	  Can_Processor();
  }
  /* USER CODE END 5 */
}

/* doMotCan function */
void doMotCan(void const * argument)
{
  /* USER CODE BEGIN doMotCan */
  osDelay(8);
  OLED_writeFrame(&holed1, "Welcome aboard \"B9\"                           POLARIS       ");
  DD_init(&holed1);

  /* Infinite loop */
  for(;;)
  {
	motCan_Processor();
  }
  /* USER CODE END doMotCan */
}

/* doSwitches function */
void doSwitches(void const * argument)
{
  /* USER CODE BEGIN doSwitches */
	uint32_t lastState = 0;
  /* Infinite loop */
  for(;;)
  {
	uint32_t currentState = readSwitches();

	if(ackPressed){     //check ack
		sendAckPressed();
		ackPressed = 0;
	}

	if(currentState != lastState){
		reportSwitches(currentState);

		if(currentState & 1<<HAZARD_SWITCH){   //hazard on
			if(~lastState & 1<<HAZARD_SWITCH){     // hazard up edge
				HAL_GPIO_WritePin(LEFT_LIGHT_GPIO_Port,LEFT_LIGHT_Pin,GPIO_PIN_SET);
				HAL_GPIO_WritePin(RIGHT_LIGHT_GPIO_Port,RIGHT_LIGHT_Pin,GPIO_PIN_SET);
				xTimerReset(LSigTmrHandle, portMAX_DELAY);
				xTimerReset(RSigTmrHandle, portMAX_DELAY);
			}
		}else{      //hazard off
			if(lastState & 1<<HAZARD_SWITCH){      // hazard down edge
				if(~currentState & 1<<LEFT_SIG_SWITCH){
					xTimerStop(LSigTmrHandle, portMAX_DELAY);
					HAL_GPIO_WritePin(LEFT_LIGHT_GPIO_Port,LEFT_LIGHT_Pin,GPIO_PIN_RESET);
				}
				if(~currentState & 1<<RIGHT_SIG_SWITCH){
					xTimerStop(RSigTmrHandle, portMAX_DELAY);
					HAL_GPIO_WritePin(RIGHT_LIGHT_GPIO_Port,RIGHT_LIGHT_Pin,GPIO_PIN_RESET);
				}
			}
			if(currentState & 1<<LEFT_SIG_SWITCH && ~lastState & 1<<LEFT_SIG_SWITCH){   //left up edge
				HAL_GPIO_WritePin(LEFT_LIGHT_GPIO_Port,LEFT_LIGHT_Pin,GPIO_PIN_SET);
				xTimerReset(LSigTmrHandle, portMAX_DELAY);
			}else if(~currentState & 1<<LEFT_SIG_SWITCH && lastState & 1<<LEFT_SIG_SWITCH){   //left down edge
				HAL_GPIO_WritePin(LEFT_LIGHT_GPIO_Port,LEFT_LIGHT_Pin,GPIO_PIN_RESET);
				xTimerStop(LSigTmrHandle, portMAX_DELAY);   //left light is more important than right light
			}else if(currentState & 1<<RIGHT_SIG_SWITCH && ~lastState & 1<<RIGHT_SIG_SWITCH){   //right up edge
				HAL_GPIO_WritePin(RIGHT_LIGHT_GPIO_Port,RIGHT_LIGHT_Pin,GPIO_PIN_SET);
				xTimerReset(RSigTmrHandle, portMAX_DELAY);
			}else if(~currentState & 1<<RIGHT_SIG_SWITCH && lastState & 1<<RIGHT_SIG_SWITCH){   //right down edge
				HAL_GPIO_WritePin(RIGHT_LIGHT_GPIO_Port,RIGHT_LIGHT_Pin,GPIO_PIN_RESET);
				xTimerStop(RSigTmrHandle, portMAX_DELAY);
			}
		}

		if(currentState & 1<<BRK_SWITCH){       //break light
			HAL_GPIO_WritePin(BRK_LIGHT_GPIO_Port,BRK_LIGHT_Pin,GPIO_PIN_SET);
		}else{
			HAL_GPIO_WritePin(BRK_LIGHT_GPIO_Port,BRK_LIGHT_Pin,GPIO_PIN_RESET);
		}
		lastState = currentState;
	}
	osDelay(Switch_Interval);
  }
  /* USER CODE END doSwitches */
}

/* doNodeManager function */
void doNodeManager(void const * argument)
{
  /* USER CODE BEGIN doNodeManager */
  /* Infinite loop */
  for(;;)
  {
	// Wrapper for the Node_Manager task
    Node_Manager();
  }
  /* USER CODE END doNodeManager */
}

/* TmrKickDog function */
void TmrKickDog(void const * argument)
{
  /* USER CODE BEGIN TmrKickDog */
  taskENTER_CRITICAL();
  HAL_WWDG_Refresh(&hwwdg);
  taskEXIT_CRITICAL();
  /* USER CODE END TmrKickDog */
}

/* toggleLSig function */
void toggleLSig(void const * argument)
{
  /* USER CODE BEGIN toggleLSig */
  HAL_GPIO_TogglePin(LEFT_LIGHT_GPIO_Port,LEFT_LIGHT_Pin);
  /* USER CODE END toggleLSig */
}

/* toggleRSig function */
void toggleRSig(void const * argument)
{
  /* USER CODE BEGIN toggleRSig */
  HAL_GPIO_TogglePin(RIGHT_LIGHT_GPIO_Port,RIGHT_LIGHT_Pin);
  /* USER CODE END toggleRSig */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
/* USER CODE BEGIN Callback 0 */

/* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
/* USER CODE BEGIN Callback 1 */

/* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
	ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */

/**
  * @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
