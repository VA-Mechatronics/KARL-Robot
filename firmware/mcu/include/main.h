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
#include "stm32g4xx_hal.h"

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
//#define V1
#define GPIO_Input_PC14_Pin GPIO_PIN_14
#define GPIO_Input_PC14_GPIO_Port GPIOC
#define GPIO_Input_PC15_Pin GPIO_PIN_15
#define GPIO_Input_PC15_GPIO_Port GPIOC
#define GPIO_Input_PF0_Pin GPIO_PIN_0
#define GPIO_Input_PF0_GPIO_Port GPIOF
#define GPIO_Input_PF1_Pin GPIO_PIN_1
#define GPIO_Input_PF1_GPIO_Port GPIOF
#ifdef V1
#define GPIO_Output_DIR_Pin GPIO_PIN_1
#define GPIO_Output_DIR_GPIO_Port GPIOA
#define GPIO_Input_OE_Pin GPIO_PIN_0
#define GPIO_Input_OE_GPIO_Port GPIOA
// Dummy for old versions
#define GPIO_Input_PA0_Pin GPIO_PIN_1
#define GPIO_Input_PA0_GPIO_Port GPIOB
#define GPIO_Output_PA1_Pin GPIO_PIN_2
#define GPIO_Output_PA1_GPIO_Port GPIOB
// End of dummy for old versions
#else
#define GPIO_Output_DIR_Pin GPIO_PIN_2
#define GPIO_Output_DIR_GPIO_Port GPIOA
#define GPIO_Input_OE_Pin GPIO_PIN_3
#define GPIO_Input_OE_GPIO_Port GPIOA
#define GPIO_Input_PA0_Pin GPIO_PIN_0
#define GPIO_Input_PA0_GPIO_Port GPIOA
#define GPIO_Output_PA1_Pin GPIO_PIN_1
#define GPIO_Output_PA1_GPIO_Port GPIOA
#endif
#define GPIO_Output_CS1_Pin GPIO_PIN_4
#define GPIO_Output_CS1_GPIO_Port GPIOA
#define GPIO_Output_EN_Pin GPIO_PIN_0
#define GPIO_Output_EN_GPIO_Port GPIOB
#define GPIO_Output_CS2_Pin GPIO_PIN_12
#define GPIO_Output_CS2_GPIO_Port GPIOB
#define GPIO_Output_BLU_Pin GPIO_PIN_4
#define GPIO_Output_BLU_GPIO_Port GPIOB
#define GPIO_Output_RED_Pin GPIO_PIN_5
#define GPIO_Output_RED_GPIO_Port GPIOB
#define GPIO_Input_CBOOT_Pin GPIO_PIN_7
#define GPIO_Input_CBOOT_GPIO_Port GPIOB
#define GPIO_Input_DI0_Pin GPIO_PIN_6
#define GPIO_Input_DI0_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
