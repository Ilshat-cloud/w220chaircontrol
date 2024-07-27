/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

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
#define error_CAN_led_Pin GPIO_PIN_13
#define error_CAN_led_GPIO_Port GPIOC
#define A_Pin GPIO_PIN_0
#define A_GPIO_Port GPIOA
#define B_Pin GPIO_PIN_1
#define B_GPIO_Port GPIOA
#define C_Pin GPIO_PIN_2
#define C_GPIO_Port GPIOA
#define D_Pin GPIO_PIN_3
#define D_GPIO_Port GPIOA
#define E_Pin GPIO_PIN_4
#define E_GPIO_Port GPIOA
#define F_Pin GPIO_PIN_5
#define F_GPIO_Port GPIOA
#define GPIO_Line1_Pin GPIO_PIN_6
#define GPIO_Line1_GPIO_Port GPIOA
#define GPIO_Line2_Pin GPIO_PIN_7
#define GPIO_Line2_GPIO_Port GPIOA
#define Heater_left_LED_Pin GPIO_PIN_0
#define Heater_left_LED_GPIO_Port GPIOB
#define Heater_right_LED_Pin GPIO_PIN_1
#define Heater_right_LED_GPIO_Port GPIOB
#define GPIO_Line3_Pin GPIO_PIN_10
#define GPIO_Line3_GPIO_Port GPIOB
#define GPIO_Line4_Pin GPIO_PIN_11
#define GPIO_Line4_GPIO_Port GPIOB
#define UART_RX_led_Pin GPIO_PIN_13
#define UART_RX_led_GPIO_Port GPIOB
#define UART_TX_led_Pin GPIO_PIN_14
#define UART_TX_led_GPIO_Port GPIOB
#define CAN_Recieve_led_Pin GPIO_PIN_15
#define CAN_Recieve_led_GPIO_Port GPIOB
#define CAN_transmit_led_Pin GPIO_PIN_8
#define CAN_transmit_led_GPIO_Port GPIOA
#define BT_State_Pin GPIO_PIN_15
#define BT_State_GPIO_Port GPIOA
#define BT_EN_Pin GPIO_PIN_4
#define BT_EN_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define Debug
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
