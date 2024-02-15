/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;

extern I2C_HandleTypeDef hi2c1;

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;


extern CAN_HandleTypeDef hcan1;
extern TIM_HandleTypeDef htim12;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
extern void ecuISRTimerTick(void);
extern void ecuISRIgnitionTimer(void);
extern void ecuISRInjectionATimer(void);
extern void ecuISRInjectionBTimer(void);
extern void ecuISRInjectionCTimer(void);
extern void ecuISRInjectionDTimer(void);
extern void ecuISRHostUART(void);
extern void ecuISRAuxUART(void);
extern void ecuISRcrankshaftTrigger(void);
/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
/* USER CODE END Private defines */


/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Crankshaft_Trigger_Pin GPIO_PIN_5
#define Crankshaft_Trigger_GPIO_Port GPIOA
#define Injector_D_Pin GPIO_PIN_12
#define Injector_D_GPIO_Port GPIOE
#define Injector_C_Pin GPIO_PIN_13
#define Injector_C_GPIO_Port GPIOE
#define Injector_B_Pin GPIO_PIN_14
#define Injector_B_GPIO_Port GPIOE
#define Injector_A_Pin GPIO_PIN_15
#define Injector_A_GPIO_Port GPIOE
#define Throttle_Closed_Switch_Pin GPIO_PIN_10
#define Throttle_Closed_Switch_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_15
#define LD2_GPIO_Port GPIOB
#define Coil_D_Pin GPIO_PIN_7
#define Coil_D_GPIO_Port GPIOC
#define Coil_C_Pin GPIO_PIN_8
#define Coil_C_GPIO_Port GPIOC
#define Coil_B_Pin GPIO_PIN_9
#define Coil_B_GPIO_Port GPIOC
#define Coil_A_Pin GPIO_PIN_8
#define Coil_A_GPIO_Port GPIOA
#define CMP_SIGNAL_CHECK_Pin GPIO_PIN_3
#define CMP_SIGNAL_CHECK_GPIO_Port GPIOB
#define Fan_Control_Pin GPIO_PIN_5
#define Fan_Control_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define I2C_INTERFACE &hi2c1
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
