/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "stm32f7xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LCD_RS_Pin GPIO_PIN_1
#define LCD_RS_GPIO_Port GPIOC
#define LCD_RW_Pin GPIO_PIN_2
#define LCD_RW_GPIO_Port GPIOC
#define LCD_DB0_Pin GPIO_PIN_0
#define LCD_DB0_GPIO_Port GPIOA
#define LCD_DB1_Pin GPIO_PIN_1
#define LCD_DB1_GPIO_Port GPIOA
#define LCD_DB2_Pin GPIO_PIN_2
#define LCD_DB2_GPIO_Port GPIOA
#define LCD_DB3_Pin GPIO_PIN_3
#define LCD_DB3_GPIO_Port GPIOA

#define LED1G_Pin GPIO_PIN_15
#define LED1G_GPIO_Port GPIOB
#define LED2G_Pin GPIO_PIN_12
#define LED2G_GPIO_Port GPIOB
#define LED2R_Pin GPIO_PIN_13
#define LED2R_GPIO_Port GPIOB
#define LED1R_Pin GPIO_PIN_8
#define LED1R_GPIO_Port GPIOB





#define LCD_DB4_Pin GPIO_PIN_4
#define LCD_DB4_GPIO_Port GPIOA
#define LCD_DB5_Pin GPIO_PIN_5
#define LCD_DB5_GPIO_Port GPIOA
#define LCD_DB6_Pin GPIO_PIN_6
#define LCD_DB6_GPIO_Port GPIOA
#define LCD_DB7_Pin GPIO_PIN_7
#define LCD_DB7_GPIO_Port GPIOA
#define LCD_CS1_Pin GPIO_PIN_4
#define LCD_CS1_GPIO_Port GPIOC
#define LCD_CS2_Pin GPIO_PIN_0
#define LCD_CS2_GPIO_Port GPIOB
#define LCD_EN_Pin GPIO_PIN_1
#define LCD_EN_GPIO_Port GPIOB
#define LCD_RST_Pin GPIO_PIN_2
#define LCD_RST_GPIO_Port GPIOB
#define LCD_LED_TIM2_CH3_Pin GPIO_PIN_10
#define LCD_LED_TIM2_CH3_GPIO_Port GPIOB
#define SW_DISPLAY_Pin GPIO_PIN_6
#define SW_DISPLAY_GPIO_Port GPIOC
#define SW_FUEL2_Pin GPIO_PIN_7
#define SW_FUEL2_GPIO_Port GPIOC
#define SW_FUEL1_Pin GPIO_PIN_8
#define SW_FUEL1_GPIO_Port GPIOC
#define USB_RST_Pin GPIO_PIN_15
#define USB_RST_GPIO_Port GPIOA
#define BUT_CANCEL_Pin GPIO_PIN_12
#define BUT_CANCEL_GPIO_Port GPIOC
#define BUT_ENTER_Pin GPIO_PIN_2
#define BUT_ENTER_GPIO_Port GPIOD
#define BUT_LEFT_Pin GPIO_PIN_4
#define BUT_LEFT_GPIO_Port GPIOB
#define BUT_RIGHT_Pin GPIO_PIN_5
#define BUT_RIGHT_GPIO_Port GPIOB
#define BUT_UP_Pin GPIO_PIN_6
#define BUT_UP_GPIO_Port GPIOB
#define BUT_DOWN_Pin GPIO_PIN_7
#define BUT_DOWN_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
