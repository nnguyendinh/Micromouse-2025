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
#include "stm32f2xx_hal.h"

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
I2C_HandleTypeDef* Get_I2C1_Ptr(void);
ADC_HandleTypeDef* Get_HADC1_Ptr(void);
void interruptRoutine();

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ForwardRightReceiver_Pin GPIO_PIN_0
#define ForwardRightReceiver_GPIO_Port GPIOC
#define RightReceiver_Pin GPIO_PIN_1
#define RightReceiver_GPIO_Port GPIOC
#define RightEmitter_Pin GPIO_PIN_2
#define RightEmitter_GPIO_Port GPIOA
#define Buzzer_Pin GPIO_PIN_3
#define Buzzer_GPIO_Port GPIOA
#define LeftEmitter_Pin GPIO_PIN_4
#define LeftEmitter_GPIO_Port GPIOA
#define LeftReceiver_Pin GPIO_PIN_5
#define LeftReceiver_GPIO_Port GPIOA
#define ForwardLeftReceiver_Pin GPIO_PIN_6
#define ForwardLeftReceiver_GPIO_Port GPIOA
#define ForwardLeftEmitter_Pin GPIO_PIN_7
#define ForwardLeftEmitter_GPIO_Port GPIOA
#define Button2_Pin GPIO_PIN_14
#define Button2_GPIO_Port GPIOB
#define Button1_Pin GPIO_PIN_15
#define Button1_GPIO_Port GPIOB
#define LeftEncoderCh1_Pin GPIO_PIN_6
#define LeftEncoderCh1_GPIO_Port GPIOC
#define LeftEncoderCh2_Pin GPIO_PIN_7
#define LeftEncoderCh2_GPIO_Port GPIOC
#define LeftMotorCh1_Pin GPIO_PIN_8
#define LeftMotorCh1_GPIO_Port GPIOA
#define LeftMotorCh2_Pin GPIO_PIN_9
#define LeftMotorCh2_GPIO_Port GPIOA
#define RightMotorCh1_Pin GPIO_PIN_10
#define RightMotorCh1_GPIO_Port GPIOA
#define RightMotorCh2_Pin GPIO_PIN_11
#define RightMotorCh2_GPIO_Port GPIOA
#define Switch3_Pin GPIO_PIN_15
#define Switch3_GPIO_Port GPIOA
#define Switch1_Pin GPIO_PIN_12
#define Switch1_GPIO_Port GPIOC
#define Switch2_Pin GPIO_PIN_2
#define Switch2_GPIO_Port GPIOD
#define RightEncoderCh1_Pin GPIO_PIN_4
#define RightEncoderCh1_GPIO_Port GPIOB
#define RightEncoderCh2_Pin GPIO_PIN_5
#define RightEncoderCh2_GPIO_Port GPIOB
#define Switch4_Pin GPIO_PIN_8
#define Switch4_GPIO_Port GPIOB
#define ForwardRightEmitter_Pin GPIO_PIN_9
#define ForwardRightEmitter_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
