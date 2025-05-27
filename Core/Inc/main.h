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
//#include "flash.h"
#include "calibration.h"
#include "cli.h"
#include "cmsis_os2.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct {
    float lightLevel;
    float batteryVoltage;
    uint16_t raw_light;
    uint16_t raw_battery;
} SensorData_t;
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
extern ADC_HandleTypeDef hadc1;
extern osMutexId_t dataMutexHandle;
extern osMutexId_t ledMutexHandle;
extern osMutexId_t uartMutexHandle;
extern osMutexId_t flashMutexHandle;
extern FlashCalibrationBlock_t g_flashCalib;
extern SensorData_t g_sensorData;
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define LIGHT_Pin GPIO_PIN_6
#define LIGHT_GPIO_Port GPIOA
#define BATTERY_Pin GPIO_PIN_7
#define BATTERY_GPIO_Port GPIOA
#define MOTION_Pin GPIO_PIN_4
#define MOTION_GPIO_Port GPIOC
#define LED_HALF_Pin GPIO_PIN_0
#define LED_HALF_GPIO_Port GPIOB
#define LED_FULL_Pin GPIO_PIN_1
#define LED_FULL_GPIO_Port GPIOB
#define CHARGE_Pin GPIO_PIN_2
#define CHARGE_GPIO_Port GPIOB
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define ADC_CHANNEL_LIGHT ADC_CHANNEL_6
#define ADC_CHANNEL_BATTERY ADC_CHANNEL_7
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
