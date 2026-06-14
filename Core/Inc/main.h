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
extern UART_HandleTypeDef huart1;
extern SPI_HandleTypeDef hspi1;
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RF900_BUSY_Pin GPIO_PIN_4
#define RF900_BUSY_GPIO_Port GPIOA
#define RF900_RST_Pin GPIO_PIN_5
#define RF900_RST_GPIO_Port GPIOA
#define RF900_TXEN_Pin GPIO_PIN_6
#define RF900_TXEN_GPIO_Port GPIOA
#define RF900_RXEN_Pin GPIO_PIN_7
#define RF900_RXEN_GPIO_Port GPIOA
#define RF24_INT1_Pin GPIO_PIN_12
#define RF24_INT1_GPIO_Port GPIOB
#define RF24_INT2_Pin GPIO_PIN_13
#define RF24_INT2_GPIO_Port GPIOB
#define RF24_INT3_Pin GPIO_PIN_14
#define RF24_INT3_GPIO_Port GPIOB
#define RF24_RST_Pin GPIO_PIN_15
#define RF24_RST_GPIO_Port GPIOB
#define RF24_BUSY_Pin GPIO_PIN_8
#define RF24_BUSY_GPIO_Port GPIOA
#define RF24_INT1A11_Pin GPIO_PIN_11
#define RF24_INT1A11_GPIO_Port GPIOA
#define RF24_INT2A12_Pin GPIO_PIN_12
#define RF24_INT2A12_GPIO_Port GPIOA
#define RF24_INT3A13_Pin GPIO_PIN_13
#define RF24_INT3A13_GPIO_Port GPIOA
#define NSS_RF900_Pin GPIO_PIN_14
#define NSS_RF900_GPIO_Port GPIOA
#define NSS_RF24_Pin GPIO_PIN_15
#define NSS_RF24_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
