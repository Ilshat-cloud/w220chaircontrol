/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"

CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8] ={0,};
uint8_t RxData[8] ={0,};
uint8_t Usart_RX_Tail=0, Usart_RX_Head=0;  
uint8_t Usart_TX_Tail=0, Usart_TX_Head=0;;  
uint32_t TxMailbox=0, ErrorCNT=0;
uint8_t CAN_Tx_buffer_tail=0,CAN_Tx_buffer_head=0,RTS=1;
typedef struct
{ 
  uint8_t TxData[8];
  uint32_t StdID;
} CAN_TxBuf_TypeDef;
CAN_TxBuf_TypeDef CAN_TxBuf_ring[16];

uint8_t CAN_Rx_buffer_tail=0,CAN_Rx_buffer_head=0;
typedef struct
{ 
  uint8_t RxData[8];
  uint32_t StdID;
} CAN_RxBuf_TypeDef;
CAN_RxBuf_TypeDef CAN_RxBuf_ring[16];

typedef struct
{ 
  uint8_t Usart_buf_RX[15];
} Usart_RxBuf_TypeDef;
Usart_RxBuf_TypeDef Usart_RxBuf_ring[16];//message format is Ch232-5, ID-2, DATA 8
typedef struct
{ 
  uint8_t Usart_buf_TX[15];
} Usart_TxBuf_TypeDef;
Usart_TxBuf_TypeDef Usart_TxBuf_ring[16];//message to send


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

IWDG_HandleTypeDef hiwdg;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* Definitions for mainTask */
osThreadId_t mainTaskHandle;
const osThreadAttr_t mainTask_attributes = {
  .name = "mainTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CANnetwork */
osThreadId_t CANnetworkHandle;
const osThreadAttr_t CANnetwork_attributes = {
  .name = "CANnetwork",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal3,
};
/* Definitions for Bluetooth */
osThreadId_t BluetoothHandle;
const osThreadAttr_t Bluetooth_attributes = {
  .name = "Bluetooth",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal3,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN_Init(void);
static void MX_IWDG_Init(void);
static void MX_USART1_UART_Init(void);
void StartMainTask(void *argument);
void StartCANnetwork(void *argument);
void StartBluetooth(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_CAN_Init();
#ifndef Debug
  MX_IWDG_Init();
#endif  
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
#ifdef Debug
  printf("xyi pizda \r\n");
#endif  
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

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
  /* creation of mainTask */
  mainTaskHandle = osThreadNew(StartMainTask, NULL, &mainTask_attributes);

  /* creation of CANnetwork */
  CANnetworkHandle = osThreadNew(StartCANnetwork, NULL, &CANnetwork_attributes);

  /* creation of Bluetooth */
  BluetoothHandle = osThreadNew(StartBluetooth, NULL, &Bluetooth_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */
  CAN_FilterTypeDef sFilterConfig;
  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 24;
  hcan.Init.Mode = CAN_MODE_SILENT_LOOPBACK;
  hcan.Init.SyncJumpWidth = CAN_SJW_2TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = ENABLE;
  hcan.Init.AutoWakeUp = ENABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
  sFilterConfig.FilterBank=0;
  sFilterConfig.FilterMode=CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale=CAN_FILTERSCALE_16BIT;
  sFilterConfig.FilterIdHigh=0x0000;
  sFilterConfig.FilterIdLow=0x0000;
  sFilterConfig.FilterMaskIdHigh=0x0000;
  sFilterConfig.FilterMaskIdLow=0x0000;
  sFilterConfig.FilterFIFOAssignment=CAN_RX_FIFO0;
  sFilterConfig.FilterActivation=ENABLE;
  if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig)!=HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(error_CAN_led_GPIO_Port, error_CAN_led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_Line1_Pin|GPIO_Line2_Pin|CAN_transmit_led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Heater_left_LED_Pin|Heater_right_LED_Pin|GPIO_Line3_Pin|GPIO_Line4_Pin
                          |UART_RX_led_Pin|UART_TX_led_Pin|CAN_Recieve_led_Pin|BT_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : error_CAN_led_Pin */
  GPIO_InitStruct.Pin = error_CAN_led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(error_CAN_led_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : A_Pin B_Pin C_Pin D_Pin
                           E_Pin F_Pin */
  GPIO_InitStruct.Pin = A_Pin|B_Pin|C_Pin|D_Pin
                          |E_Pin|F_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO_Line1_Pin GPIO_Line2_Pin CAN_transmit_led_Pin */
  GPIO_InitStruct.Pin = GPIO_Line1_Pin|GPIO_Line2_Pin|CAN_transmit_led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Heater_left_LED_Pin Heater_right_LED_Pin UART_RX_led_Pin UART_TX_led_Pin
                           CAN_Recieve_led_Pin */
  GPIO_InitStruct.Pin = Heater_left_LED_Pin|Heater_right_LED_Pin|UART_RX_led_Pin|UART_TX_led_Pin
                          |CAN_Recieve_led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO_Line3_Pin GPIO_Line4_Pin BT_EN_Pin */
  GPIO_InitStruct.Pin = GPIO_Line3_Pin|GPIO_Line4_Pin|BT_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BT_State_Pin */
  GPIO_InitStruct.Pin = BT_State_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BT_State_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  
  if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, CAN_RxBuf_ring[CAN_Rx_buffer_head].RxData)==HAL_OK)
  {
    HAL_GPIO_TogglePin(CAN_Recieve_led_GPIO_Port,CAN_Recieve_led_Pin);
    CAN_RxBuf_ring[CAN_Rx_buffer_head].StdID=RxHeader.StdId;
    CAN_Rx_buffer_head++;
    CAN_Rx_buffer_head=(CAN_Rx_buffer_head>15)?0:CAN_Rx_buffer_head;
    
  }
  
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{


}



void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
  uint32_t er = HAL_CAN_GetError(hcan);
  HAL_GPIO_WritePin(error_CAN_led_GPIO_Port,error_CAN_led_Pin,GPIO_PIN_SET);  ////////check CAN error somehow and maybe do something
  memset(Usart_TxBuf_ring[Usart_TX_Head].Usart_buf_TX,0,15);
  sprintf((char*)Usart_TxBuf_ring[Usart_TX_Head].Usart_buf_TX ,"ER CAN %1u",er);
 #ifdef Debug
  printf(Usart_TxBuf_ring[Usart_TX_Head].Usart_buf_TX);
#endif
  Usart_TX_Head++;
  Usart_TX_Head=(Usart_TX_Head>15)?0:Usart_TX_Head;
 // HAL_UART_Transmit_DMA(&huart1,(uint8_t*)Ch232,strlen(Ch232));
  ErrorCNT++;
  
}




void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
 
  if (huart==&huart1)
  {
    Usart_RX_Head++;
    Usart_RX_Head=(Usart_RX_Head>15)?0:Usart_RX_Head;
    HAL_UART_Receive_DMA(&huart1,(uint8_t*)Usart_RxBuf_ring[Usart_RX_Head].Usart_buf_RX,15);
    HAL_GPIO_TogglePin(UART_RX_led_GPIO_Port,UART_RX_led_Pin);

  }
  
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
 
  if (huart==&huart1)
  {
    if (Usart_TX_Tail!=Usart_TX_Head)
      {
        HAL_UART_Transmit_DMA(&huart1, Usart_TxBuf_ring[Usart_TX_Tail].Usart_buf_TX, 15);
        Usart_TX_Tail++;
        Usart_TX_Tail=(Usart_TX_Tail>15)?0:Usart_TX_Tail;
      }

  }
  
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  
  if (huart==&huart1)
  {
    uint32_t er=HAL_UART_GetError(&huart1);
    if(er&HAL_UART_ERROR_PE)
    {
    snprintf((char*)Usart_TxBuf_ring[Usart_TX_Head].Usart_buf_TX,15,"Parity error   \n");
    Usart_TX_Head++;
    Usart_TX_Head=(Usart_TX_Head>15)?0:Usart_TX_Head; 
    }
    if(er&HAL_UART_ERROR_NE)
    {
          snprintf((char*)Usart_TxBuf_ring[Usart_TX_Head].Usart_buf_TX,15,"Noise error   \n");
    Usart_TX_Head++;
    Usart_TX_Head=(Usart_TX_Head>15)?0:Usart_TX_Head; 
    }
    if(er&HAL_UART_ERROR_FE)
    {
          snprintf((char*)Usart_TxBuf_ring[Usart_TX_Head].Usart_buf_TX,15,"Frame error   \n");
    Usart_TX_Head++;
    Usart_TX_Head=(Usart_TX_Head>15)?0:Usart_TX_Head; 
    }
    if(er&HAL_UART_ERROR_ORE)
    {
          snprintf((char*)Usart_TxBuf_ring[Usart_TX_Head].Usart_buf_TX,15,"Overrun error \n");
    Usart_TX_Head++;
    Usart_TX_Head=(Usart_TX_Head>15)?0:Usart_TX_Head; 
    }
    if(er&HAL_UART_ERROR_DMA)
    {
          snprintf((char*)Usart_TxBuf_ring[Usart_TX_Head].Usart_buf_TX,15,"DMA transf err\n");
    Usart_TX_Head++;
    Usart_TX_Head=(Usart_TX_Head>15)?0:Usart_TX_Head; 
    }
    huart->ErrorCode=HAL_UART_ERROR_NONE;
    
  }
  
}


/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartMainTask */
/**
  * @brief  Function implementing the mainTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartMainTask */
void StartMainTask(void *argument)
{
  /* USER CODE BEGIN 5 */
   struct button_without_fix {
     GPIO_PinState pos_current;
     GPIO_PinState pos_previous;
  };
  struct button_without_fix Bottom_frontward_L= {GPIO_PIN_RESET,GPIO_PIN_RESET}, Bottom_backward_L= {GPIO_PIN_RESET,GPIO_PIN_RESET}, backrest_recline_L= {GPIO_PIN_RESET,GPIO_PIN_RESET}, 
  backrest_frontward_L= {GPIO_PIN_RESET,GPIO_PIN_RESET}, Headrest_up_L= {GPIO_PIN_RESET,GPIO_PIN_RESET}, Headrest_down_L={GPIO_PIN_RESET,GPIO_PIN_RESET}, M1_L={GPIO_PIN_RESET,GPIO_PIN_RESET},
  M2_L={GPIO_PIN_RESET,GPIO_PIN_RESET},M3_L={GPIO_PIN_RESET,GPIO_PIN_RESET},M_set_L={GPIO_PIN_RESET,GPIO_PIN_RESET},Heater_ON_OFF_L={GPIO_PIN_RESET,GPIO_PIN_RESET},
  Bottom_frontward_R= {GPIO_PIN_RESET,GPIO_PIN_RESET}, Bottom_backward_R= {GPIO_PIN_RESET,GPIO_PIN_RESET}, backrest_recline_R= {GPIO_PIN_RESET,GPIO_PIN_RESET}, 
  backrest_frontward_R= {GPIO_PIN_RESET,GPIO_PIN_RESET}, Headrest_up_R= {GPIO_PIN_RESET,GPIO_PIN_RESET}, Headrest_down_R={GPIO_PIN_RESET,GPIO_PIN_RESET}, M1_R={GPIO_PIN_RESET,GPIO_PIN_RESET},
  M2_R={GPIO_PIN_RESET,GPIO_PIN_RESET},M3_R={GPIO_PIN_RESET,GPIO_PIN_RESET},M_set_R={GPIO_PIN_RESET,GPIO_PIN_RESET},Heater_ON_OFF_R={GPIO_PIN_RESET,GPIO_PIN_RESET};
  GPIO_PinState matrix_line[6][4];

  /* Infinite loop */
  for(;;)
  {
    //-----------------------------buttton processing-----------------------------------------//
    HAL_GPIO_WritePin(GPIO_Line1_GPIO_Port,GPIO_Line1_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_Line2_GPIO_Port,GPIO_Line2_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIO_Line3_GPIO_Port,GPIO_Line3_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIO_Line4_GPIO_Port,GPIO_Line4_Pin,GPIO_PIN_RESET);
    osDelay(2);
    matrix_line[0][0]=HAL_GPIO_ReadPin(A_GPIO_Port,A_Pin);
    matrix_line[1][0]=HAL_GPIO_ReadPin(B_GPIO_Port,B_Pin);      
    matrix_line[2][0]=HAL_GPIO_ReadPin(C_GPIO_Port,C_Pin);      
    matrix_line[3][0]=HAL_GPIO_ReadPin(D_GPIO_Port,D_Pin);
    matrix_line[4][0]=HAL_GPIO_ReadPin(E_GPIO_Port,E_Pin);      
    matrix_line[5][0]=HAL_GPIO_ReadPin(F_GPIO_Port,F_Pin);
    osDelay(2);
    HAL_GPIO_WritePin(GPIO_Line1_GPIO_Port,GPIO_Line1_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIO_Line2_GPIO_Port,GPIO_Line2_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_Line3_GPIO_Port,GPIO_Line3_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIO_Line4_GPIO_Port,GPIO_Line4_Pin,GPIO_PIN_RESET);
    osDelay(2);  
    matrix_line[0][1]=HAL_GPIO_ReadPin(A_GPIO_Port,A_Pin);
    matrix_line[1][1]=HAL_GPIO_ReadPin(B_GPIO_Port,B_Pin);      
    matrix_line[2][1]=HAL_GPIO_ReadPin(C_GPIO_Port,C_Pin);      
    matrix_line[3][1]=HAL_GPIO_ReadPin(D_GPIO_Port,D_Pin);
    matrix_line[4][1]=HAL_GPIO_ReadPin(E_GPIO_Port,E_Pin);      
    matrix_line[5][1]=HAL_GPIO_ReadPin(F_GPIO_Port,F_Pin);  
    osDelay(2);
    HAL_GPIO_WritePin(GPIO_Line1_GPIO_Port,GPIO_Line1_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIO_Line2_GPIO_Port,GPIO_Line2_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIO_Line3_GPIO_Port,GPIO_Line3_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIO_Line4_GPIO_Port,GPIO_Line4_Pin,GPIO_PIN_RESET);
    osDelay(2);     
    matrix_line[0][2]=HAL_GPIO_ReadPin(A_GPIO_Port,A_Pin);
    matrix_line[1][2]=HAL_GPIO_ReadPin(B_GPIO_Port,B_Pin);      
    matrix_line[2][2]=HAL_GPIO_ReadPin(C_GPIO_Port,C_Pin);      
    matrix_line[3][2]=HAL_GPIO_ReadPin(D_GPIO_Port,D_Pin);
    matrix_line[4][2]=HAL_GPIO_ReadPin(E_GPIO_Port,E_Pin);      
    matrix_line[5][2]=HAL_GPIO_ReadPin(F_GPIO_Port,F_Pin);  
    osDelay(2);
    HAL_GPIO_WritePin(GPIO_Line1_GPIO_Port,GPIO_Line1_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIO_Line2_GPIO_Port,GPIO_Line2_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIO_Line3_GPIO_Port,GPIO_Line3_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIO_Line4_GPIO_Port,GPIO_Line4_Pin,GPIO_PIN_SET);
    osDelay(2);    
    matrix_line[0][3]=HAL_GPIO_ReadPin(A_GPIO_Port,A_Pin);
    matrix_line[1][3]=HAL_GPIO_ReadPin(B_GPIO_Port,B_Pin);      
    matrix_line[2][3]=HAL_GPIO_ReadPin(C_GPIO_Port,C_Pin);      
    matrix_line[3][3]=HAL_GPIO_ReadPin(D_GPIO_Port,D_Pin);
    matrix_line[4][3]=HAL_GPIO_ReadPin(E_GPIO_Port,E_Pin);      
    matrix_line[5][3]=HAL_GPIO_ReadPin(F_GPIO_Port,F_Pin);  

    Bottom_frontward_L.pos_previous=Bottom_frontward_L.pos_current;
    Bottom_frontward_L.pos_current=matrix_line[0][0];
 
    Bottom_backward_L.pos_previous=Bottom_backward_L.pos_current;
    Bottom_backward_L.pos_current=matrix_line[0][1];
    
    backrest_recline_L.pos_previous=backrest_recline_L.pos_current;
    backrest_recline_L.pos_current=matrix_line[1][0];    
    
    backrest_frontward_L.pos_previous=backrest_frontward_L.pos_current;
    backrest_frontward_L.pos_current=matrix_line[1][1];   
    
    Headrest_up_L.pos_previous=Headrest_up_L.pos_current;
    Headrest_up_L.pos_current=matrix_line[2][0]; 
    
    Headrest_down_L.pos_previous=Headrest_down_L.pos_current;
    Headrest_down_L.pos_current=matrix_line[2][1]; 
    
    M1_L.pos_previous=M1_L.pos_current;
    M1_L.pos_current=matrix_line[3][0]; 
    
    M2_L.pos_previous=M2_L.pos_current;
    M2_L.pos_current=matrix_line[3][1]; 
    
    M3_L.pos_previous=M3_L.pos_current;
    M3_L.pos_current=matrix_line[4][0]; 
    
    M_set_L.pos_previous=M_set_L.pos_current;
    M_set_L.pos_current=matrix_line[4][1]; 
    
    Heater_ON_OFF_L.pos_previous=Heater_ON_OFF_L.pos_current;
    Heater_ON_OFF_L.pos_current=matrix_line[5][0]; 
    //---------------------right------------------------------//
    Bottom_frontward_R.pos_previous=Bottom_frontward_R.pos_current;
    Bottom_frontward_R.pos_current=matrix_line[0][2];
 
    Bottom_backward_R.pos_previous=Bottom_backward_R.pos_current;
    Bottom_backward_R.pos_current=matrix_line[0][3];
    
    backrest_recline_R.pos_previous=backrest_recline_R.pos_current;
    backrest_recline_R.pos_current=matrix_line[1][2];    
    
    backrest_frontward_R.pos_previous=backrest_frontward_R.pos_current;
    backrest_frontward_R.pos_current=matrix_line[1][3];   
    
    Headrest_up_R.pos_previous=Headrest_up_R.pos_current;
    Headrest_up_R.pos_current=matrix_line[2][2]; 
    
    Headrest_down_R.pos_previous=Headrest_down_R.pos_current;
    Headrest_down_R.pos_current=matrix_line[2][3]; 
    
    M1_R.pos_previous=M1_R.pos_current;
    M1_R.pos_current=matrix_line[3][2]; 
    
    M2_R.pos_previous=M2_R.pos_current;
    M2_R.pos_current=matrix_line[3][3]; 
    
    M3_R.pos_previous=M3_R.pos_current;
    M3_R.pos_current=matrix_line[4][2]; 
    
    M_set_R.pos_previous=M_set_R.pos_current;
    M_set_R.pos_current=matrix_line[4][3]; 
    
    Heater_ON_OFF_R.pos_previous=Heater_ON_OFF_R.pos_current;
    Heater_ON_OFF_R.pos_current=matrix_line[5][2];   
    
    
    
    if ((Bottom_frontward_L.pos_previous==GPIO_PIN_SET)&&(Bottom_frontward_L.pos_current==GPIO_PIN_SET))
    {
      CAN_TxBuf_ring[CAN_Tx_buffer_head].StdID=0x028C;
      CAN_TxBuf_ring[CAN_Tx_buffer_head].TxData[7]=0x00;
      CAN_TxBuf_ring[CAN_Tx_buffer_head].TxData[6]=0x00;
      CAN_TxBuf_ring[CAN_Tx_buffer_head].TxData[5]=0x00;
      CAN_TxBuf_ring[CAN_Tx_buffer_head].TxData[4]=0x00;
      CAN_TxBuf_ring[CAN_Tx_buffer_head].TxData[3]=0x00;
      CAN_TxBuf_ring[CAN_Tx_buffer_head].TxData[2]=0x00;
      CAN_TxBuf_ring[CAN_Tx_buffer_head].TxData[1]=0x00;
      CAN_TxBuf_ring[CAN_Tx_buffer_head].TxData[0]=0x80; //offset 7 len 1      
      CAN_Tx_buffer_head++;
      CAN_Tx_buffer_head=(CAN_Tx_buffer_head>15)?0:CAN_Tx_buffer_head;
    }
    if ((Bottom_backward_L.pos_previous==GPIO_PIN_SET)&&(Bottom_backward_L.pos_current==GPIO_PIN_SET))
    {
      CAN_TxBuf_ring[CAN_Tx_buffer_head].StdID=0x028C;
      CAN_TxBuf_ring[CAN_Tx_buffer_head].TxData[7]=0x00;
      CAN_TxBuf_ring[CAN_Tx_buffer_head].TxData[6]=0x00;
      CAN_TxBuf_ring[CAN_Tx_buffer_head].TxData[5]=0x00;
      CAN_TxBuf_ring[CAN_Tx_buffer_head].TxData[4]=0x00;
      CAN_TxBuf_ring[CAN_Tx_buffer_head].TxData[3]=0x00;
      CAN_TxBuf_ring[CAN_Tx_buffer_head].TxData[2]=0x00;
      CAN_TxBuf_ring[CAN_Tx_buffer_head].TxData[1]=0x00;
      CAN_TxBuf_ring[CAN_Tx_buffer_head].TxData[0]=0x40; //offset 6 len 1      
      CAN_Tx_buffer_head++;
      CAN_Tx_buffer_head=(CAN_Tx_buffer_head>15)?0:CAN_Tx_buffer_head;
    }
    if ((backrest_recline_L.pos_previous==GPIO_PIN_SET)&&(backrest_recline_L.pos_current==GPIO_PIN_SET))
    {
      CAN_TxBuf_ring[CAN_Tx_buffer_head].StdID=0x028C;
      CAN_TxBuf_ring[CAN_Tx_buffer_head].TxData[7]=0x00;
      CAN_TxBuf_ring[CAN_Tx_buffer_head].TxData[6]=0x00;
      CAN_TxBuf_ring[CAN_Tx_buffer_head].TxData[5]=0x00;
      CAN_TxBuf_ring[CAN_Tx_buffer_head].TxData[4]=0x00;
      CAN_TxBuf_ring[CAN_Tx_buffer_head].TxData[3]=0x00;
      CAN_TxBuf_ring[CAN_Tx_buffer_head].TxData[2]=0x00;
      CAN_TxBuf_ring[CAN_Tx_buffer_head].TxData[1]=0x00;
      CAN_TxBuf_ring[CAN_Tx_buffer_head].TxData[0]=0x40; //offset 6 len 1      
      CAN_Tx_buffer_head++;
      CAN_Tx_buffer_head=(CAN_Tx_buffer_head>15)?0:CAN_Tx_buffer_head;
    }
    
    
    
    
    
#ifndef Debug    
    HAL_IWDG_Refresh(&hiwdg);
#endif    
    osDelay(100);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartCANnetwork */
/**
* @brief Function implementing the CANnetwork thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCANnetwork */
void StartCANnetwork(void *argument)
{
  /* USER CODE BEGIN StartCANnetwork */
  snprintf((char*)Usart_TxBuf_ring[Usart_TX_Head].Usart_buf_TX,15,"CAN Network   \n");
  Usart_TX_Head++;
  Usart_TX_Head=(Usart_TX_Head>15)?0:Usart_TX_Head; 
  TxHeader.StdId=0x379;
  TxHeader.ExtId=0;
  TxHeader.RTR=CAN_RTR_DATA;
  TxHeader.IDE=CAN_ID_STD;
  TxHeader.DLC=8;
  TxHeader.TransmitGlobalTime=DISABLE;
  uint8_t iter=0;
  //------------TX-DATA-------------//
  CAN_TxBuf_ring[0].TxData[0]=2;
  CAN_TxBuf_ring[0].TxData[1]=3;
  CAN_TxBuf_ring[0].TxData[2]=0x55;
  CAN_TxBuf_ring[0].TxData[3]=0xAA;
  CAN_TxBuf_ring[0].TxData[4]=4;
  CAN_TxBuf_ring[0].TxData[5]=5;
  CAN_TxBuf_ring[0].TxData[6]=0x55;
  CAN_TxBuf_ring[0].TxData[7]=0xAA;  
  //------------------------------//        
  HAL_CAN_Start(&hcan);
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING|CAN_IT_ERROR|CAN_IT_BUSOFF|CAN_IT_LAST_ERROR_CODE);
  osDelay(2000);  //Bluetooth should be first
  /* Infinite loop */
  for(;;)
  {
    //---------------------send engine on---------------------------//
    if (iter>=0)
    {
      CAN_TxBuf_ring[CAN_Tx_buffer_head].StdID=0x0001;
      CAN_TxBuf_ring[CAN_Tx_buffer_head].TxData[7]=0x00;
      CAN_TxBuf_ring[CAN_Tx_buffer_head].TxData[6]=0x00;
      CAN_TxBuf_ring[CAN_Tx_buffer_head].TxData[5]=0x00;
      CAN_TxBuf_ring[CAN_Tx_buffer_head].TxData[4]=0x00;
      CAN_TxBuf_ring[CAN_Tx_buffer_head].TxData[3]=0x00;
      CAN_TxBuf_ring[CAN_Tx_buffer_head].TxData[2]=0x00;
      CAN_TxBuf_ring[CAN_Tx_buffer_head].TxData[1]=0x00;
      CAN_TxBuf_ring[CAN_Tx_buffer_head].TxData[0]=0xF8;      
      CAN_Tx_buffer_head++;
      CAN_Tx_buffer_head=(CAN_Tx_buffer_head>15)?0:CAN_Tx_buffer_head;
      iter=0;
    }    
    //-----------------------send message to CAN-----------------------------//
    if (CAN_Tx_buffer_tail!=CAN_Tx_buffer_head)
    {
      TxHeader.StdId=CAN_TxBuf_ring[CAN_Tx_buffer_tail].StdID;
      while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan)==0);
      if (HAL_CAN_AddTxMessage(&hcan,&TxHeader,CAN_TxBuf_ring[CAN_Tx_buffer_tail].TxData,&TxMailbox)!=HAL_OK)
      {
        snprintf((char*)Usart_TxBuf_ring[Usart_TX_Head].Usart_buf_TX,15,"ER SEND to CAN\n");
#ifdef Debug
  printf(Usart_TxBuf_ring[Usart_TX_Head].Usart_buf_TX);
#endif
//        Usart_TX_Head++;
//        Usart_TX_Head=(Usart_TX_Head>15)?0:Usart_TX_Head; 
      }     
 #ifdef Debug
 // printf((char*)CAN_TxBuf_ring[CAN_Tx_buffer_tail].StdID);
#endif
      CAN_Tx_buffer_tail++;
      CAN_Tx_buffer_tail=(CAN_Tx_buffer_tail>15)?0:CAN_Tx_buffer_tail;
    // SET USART TX ACCORDING TO THIS RING/////////////////////
    }
    osDelay(10);
    iter++; 

  }
  /* USER CODE END StartCANnetwork */
}

/* USER CODE BEGIN Header_StartBluetooth */
/**
* @brief Function implementing the Bluetooth thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBluetooth */
void StartBluetooth(void *argument)
{
  /* USER CODE BEGIN StartBluetooth */
  HAL_UART_Receive_DMA(&huart1,(uint8_t*)Usart_RxBuf_ring[Usart_RX_Head].Usart_buf_RX,15);
  /* Infinite loop */
  for(;;)
  {
    //-----------------------send message to UART-----------------------------//
    if (CAN_Rx_buffer_tail!=CAN_Rx_buffer_head)
      {
        snprintf((char*)Usart_TxBuf_ring[Usart_TX_Head].Usart_buf_TX,15,"%04X %X %X %X %X%X%X%X%X\n", CAN_RxBuf_ring[CAN_Rx_buffer_tail].StdID, CAN_RxBuf_ring[CAN_Rx_buffer_tail].RxData[7],CAN_RxBuf_ring[CAN_Rx_buffer_tail].RxData[6],
                 CAN_RxBuf_ring[CAN_Rx_buffer_tail].RxData[5],CAN_RxBuf_ring[CAN_Rx_buffer_tail].RxData[4],CAN_RxBuf_ring[CAN_Rx_buffer_tail].RxData[3],CAN_RxBuf_ring[CAN_Rx_buffer_tail].RxData[2],
                 CAN_RxBuf_ring[CAN_Rx_buffer_tail].RxData[1],CAN_RxBuf_ring[CAN_Rx_buffer_tail].RxData[0]);
        Usart_TX_Head++;
        Usart_TX_Head=(Usart_TX_Head>15)?0:Usart_TX_Head; 
        CAN_Rx_buffer_tail++;
        CAN_Rx_buffer_tail=(CAN_Rx_buffer_tail>15)?0:CAN_Rx_buffer_tail;
      }
    if (Usart_TX_Tail!=Usart_TX_Head)
      {
        HAL_UART_Transmit_DMA(&huart1, Usart_TxBuf_ring[Usart_TX_Tail].Usart_buf_TX, 15);
        Usart_TX_Tail++;
        Usart_TX_Tail=(Usart_TX_Tail>15)?0:Usart_TX_Tail;
      }
    //----------------------send message to CAN-------------------------------------//
    if (Usart_RX_Tail!=Usart_RX_Head)
    {
      if (Usart_RxBuf_ring[Usart_RX_Tail].Usart_buf_RX[0]=='C'&&Usart_RxBuf_ring[Usart_RX_Tail].Usart_buf_RX[1]=='h'&&Usart_RxBuf_ring[Usart_RX_Tail].Usart_buf_RX[2]=='2'&&
          Usart_RxBuf_ring[Usart_RX_Tail].Usart_buf_RX[3]=='3'&&Usart_RxBuf_ring[Usart_RX_Tail].Usart_buf_RX[4]=='2')  //equal Ch232 then
      {
        CAN_TxBuf_ring[CAN_Tx_buffer_head].StdID=(Usart_RxBuf_ring[Usart_RX_Tail].Usart_buf_RX[5]<<8)|Usart_RxBuf_ring[Usart_RX_Tail].Usart_buf_RX[6];
        CAN_TxBuf_ring[CAN_Tx_buffer_head].TxData[7]=Usart_RxBuf_ring[Usart_RX_Tail].Usart_buf_RX[7];
        CAN_TxBuf_ring[CAN_Tx_buffer_head].TxData[6]=Usart_RxBuf_ring[Usart_RX_Tail].Usart_buf_RX[8];
        CAN_TxBuf_ring[CAN_Tx_buffer_head].TxData[5]=Usart_RxBuf_ring[Usart_RX_Tail].Usart_buf_RX[9];
        CAN_TxBuf_ring[CAN_Tx_buffer_head].TxData[4]=Usart_RxBuf_ring[Usart_RX_Tail].Usart_buf_RX[10];
        CAN_TxBuf_ring[CAN_Tx_buffer_head].TxData[3]=Usart_RxBuf_ring[Usart_RX_Tail].Usart_buf_RX[11];
        CAN_TxBuf_ring[CAN_Tx_buffer_head].TxData[2]=Usart_RxBuf_ring[Usart_RX_Tail].Usart_buf_RX[12];
        CAN_TxBuf_ring[CAN_Tx_buffer_head].TxData[1]=Usart_RxBuf_ring[Usart_RX_Tail].Usart_buf_RX[13];
        CAN_TxBuf_ring[CAN_Tx_buffer_head].TxData[0]=Usart_RxBuf_ring[Usart_RX_Tail].Usart_buf_RX[14];      
        CAN_Tx_buffer_head++;
        CAN_Tx_buffer_head=(CAN_Tx_buffer_head>15)?0:CAN_Tx_buffer_head;
        
      }
      Usart_RX_Tail++;
      Usart_RX_Tail=(Usart_RX_Tail>15)?0:Usart_RX_Tail;
    }
    osDelay(100);
  }
  /* USER CODE END StartBluetooth */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
