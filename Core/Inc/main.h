/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define A1_Pin GPIO_PIN_0
#define A1_GPIO_Port GPIOC
#define A2_Pin GPIO_PIN_1
#define A2_GPIO_Port GPIOC
#define B1C2_Pin GPIO_PIN_2
#define B1C2_GPIO_Port GPIOC
#define B2_Pin GPIO_PIN_3
#define B2_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define R_LED1_Pin GPIO_PIN_6
#define R_LED1_GPIO_Port GPIOA
#define G_LED1_Pin GPIO_PIN_7
#define G_LED1_GPIO_Port GPIOA
#define C1_Pin GPIO_PIN_4
#define C1_GPIO_Port GPIOC
#define C2_Pin GPIO_PIN_5
#define C2_GPIO_Port GPIOC
#define SCL_Pin GPIO_PIN_13
#define SCL_GPIO_Port GPIOB
#define SDA_Pin GPIO_PIN_14
#define SDA_GPIO_Port GPIOB
#define D1_Pin GPIO_PIN_6
#define D1_GPIO_Port GPIOC
#define D2_Pin GPIO_PIN_7
#define D2_GPIO_Port GPIOC
#define R_LED2_Pin GPIO_PIN_8
#define R_LED2_GPIO_Port GPIOA
#define G_LED2_Pin GPIO_PIN_9
#define G_LED2_GPIO_Port GPIOA
#define R_LED3_Pin GPIO_PIN_10
#define R_LED3_GPIO_Port GPIOA
#define G_LED3_Pin GPIO_PIN_11
#define G_LED3_GPIO_Port GPIOA
#define R_LED4_Pin GPIO_PIN_12
#define R_LED4_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define G_LED4_Pin GPIO_PIN_15
#define G_LED4_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define GroupA_Button_Pin GPIO_PIN_4
#define GroupA_Button_GPIO_Port GPIOB
#define GroupB_Button_Pin GPIO_PIN_5
#define GroupB_Button_GPIO_Port GPIOB
#define GroupC_Button_Pin GPIO_PIN_6
#define GroupC_Button_GPIO_Port GPIOB
#define GroupD_Button_Pin GPIO_PIN_7
#define GroupD_Button_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
