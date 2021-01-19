/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
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
#define FWE_Pin GPIO_PIN_1
#define FWE_GPIO_Port GPIOA
#define TP156_Pin GPIO_PIN_15
#define TP156_GPIO_Port GPIOB
#define DEBUG_TX_Pin GPIO_PIN_6
#define DEBUG_TX_GPIO_Port GPIOC
#define DEBUG_RX_Pin GPIO_PIN_7
#define DEBUG_RX_GPIO_Port GPIOC
#define TP160_Pin GPIO_PIN_10
#define TP160_GPIO_Port GPIOC
#define EMR_PDCTB_Pin GPIO_PIN_2
#define EMR_PDCTB_GPIO_Port GPIOD
#define EMR_PDCTB_EXTI_IRQn EXTI2_IRQn
#define EMR_IRQ_Pin GPIO_PIN_4
#define EMR_IRQ_GPIO_Port GPIOB
#define EMR_IRQ_EXTI_IRQn EXTI4_IRQn
#define EMR_RST_Pin GPIO_PIN_5
#define EMR_RST_GPIO_Port GPIOB
#define EMR_GPIO1_Pin GPIO_PIN_6
#define EMR_GPIO1_GPIO_Port GPIOB
#define EMR_GPIO0_Pin GPIO_PIN_7
#define EMR_GPIO0_GPIO_Port GPIOB
#define EMR_I2C_SCL_Pin GPIO_PIN_8
#define EMR_I2C_SCL_GPIO_Port GPIOB
#define EMR_I2C_SDA_Pin GPIO_PIN_9
#define EMR_I2C_SDA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
