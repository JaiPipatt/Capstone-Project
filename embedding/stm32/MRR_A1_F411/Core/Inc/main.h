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
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define TB_STBY_BOTH_Pin GPIO_PIN_15
#define TB_STBY_BOTH_GPIO_Port GPIOC
#define ENC_M4_IR_A_Pin GPIO_PIN_0
#define ENC_M4_IR_A_GPIO_Port GPIOA
#define ENC_M4_IR_B_Pin GPIO_PIN_1
#define ENC_M4_IR_B_GPIO_Port GPIOA
#define ESP_RX_Pin GPIO_PIN_2
#define ESP_RX_GPIO_Port GPIOA
#define ESP_TX_Pin GPIO_PIN_3
#define ESP_TX_GPIO_Port GPIOA
#define TB_2_A_IN1_M3_IL_Pin GPIO_PIN_4
#define TB_2_A_IN1_M3_IL_GPIO_Port GPIOA
#define TB_2_A_IN2_M3_IL_Pin GPIO_PIN_5
#define TB_2_A_IN2_M3_IL_GPIO_Port GPIOA
#define TB_1_A_PWM_M1_L_Pin GPIO_PIN_6
#define TB_1_A_PWM_M1_L_GPIO_Port GPIOA
#define TB_1_B_PWM_M2_R_Pin GPIO_PIN_7
#define TB_1_B_PWM_M2_R_GPIO_Port GPIOA
#define TB_2_A_PWM_M3_IL_Pin GPIO_PIN_0
#define TB_2_A_PWM_M3_IL_GPIO_Port GPIOB
#define TB_2_B_PWM_M4_IR_Pin GPIO_PIN_1
#define TB_2_B_PWM_M4_IR_GPIO_Port GPIOB
#define TB_1_A_IN1_M1_L_Pin GPIO_PIN_12
#define TB_1_A_IN1_M1_L_GPIO_Port GPIOB
#define TB_1_A_IN2_M1_L_Pin GPIO_PIN_13
#define TB_1_A_IN2_M1_L_GPIO_Port GPIOB
#define TB_1_B_IN1_M2_R_Pin GPIO_PIN_14
#define TB_1_B_IN1_M2_R_GPIO_Port GPIOB
#define TB_1_B_IN2_M2_R_Pin GPIO_PIN_15
#define TB_1_B_IN2_M2_R_GPIO_Port GPIOB
#define ENC_M1_L_A_Pin GPIO_PIN_8
#define ENC_M1_L_A_GPIO_Port GPIOA
#define ENC_M1_L_B_Pin GPIO_PIN_9
#define ENC_M1_L_B_GPIO_Port GPIOA
#define ENC_M2_R_A_Pin GPIO_PIN_15
#define ENC_M2_R_A_GPIO_Port GPIOA
#define ENC_M2_R_B_Pin GPIO_PIN_3
#define ENC_M2_R_B_GPIO_Port GPIOB
#define TB_2_B_IN1_M4_IR_Pin GPIO_PIN_4
#define TB_2_B_IN1_M4_IR_GPIO_Port GPIOB
#define TB_2_B_IN2_M4_IR_Pin GPIO_PIN_5
#define TB_2_B_IN2_M4_IR_GPIO_Port GPIOB
#define ENC_M3_IL_A_Pin GPIO_PIN_6
#define ENC_M3_IL_A_GPIO_Port GPIOB
#define ENC_M3_IL_B_Pin GPIO_PIN_7
#define ENC_M3_IL_B_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
