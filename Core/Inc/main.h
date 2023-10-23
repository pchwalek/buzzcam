/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32wbxx_hal.h"

#include "app_conf.h"
#include "app_entry.h"
#include "app_common.h"

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
typedef struct {
	uint32_t ChunkID;       /* 0 */
	uint32_t FileSize;      /* 4 */
	uint32_t FileFormat;    /* 8 */
	uint32_t SubChunk1ID;   /* 12 */
	uint32_t SubChunk1Size; /* 16*/
	uint16_t AudioFormat;   /* 20 */
	uint16_t NbrChannels;   /* 22 */
	uint32_t SampleRate;    /* 24 */

	uint32_t ByteRate;      /* 28 */
	uint16_t BlockAlign;    /* 32 */
	uint16_t BitPerSample;  /* 34 */
	uint32_t SubChunk2ID;   /* 36 */
	uint32_t SubChunk2Size; /* 40 */
}WAVE_FormatTypeDef;
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RED_LED_Pin GPIO_PIN_2
#define RED_LED_GPIO_Port GPIOA
#define BLUE_LED_Pin GPIO_PIN_1
#define BLUE_LED_GPIO_Port GPIOA
#define GREEN_LED_Pin GPIO_PIN_0
#define GREEN_LED_GPIO_Port GPIOA
#define P0_15_Pin GPIO_PIN_2
#define P0_15_GPIO_Port GPIOC
#define SD_CS_Pin GPIO_PIN_1
#define SD_CS_GPIO_Port GPIOC
#define EN_3V3_ALT_Pin GPIO_PIN_7
#define EN_3V3_ALT_GPIO_Port GPIOB
#define EN_UWB_REG_Pin GPIO_PIN_10
#define EN_UWB_REG_GPIO_Port GPIOC
#define EN_SD_REG_Pin GPIO_PIN_11
#define EN_SD_REG_GPIO_Port GPIOC
#define EN_MIC_PWR_Pin GPIO_PIN_12
#define EN_MIC_PWR_GPIO_Port GPIOC
#define INT1_IMU_XL_Pin GPIO_PIN_15
#define INT1_IMU_XL_GPIO_Port GPIOA
#define P1_09_Pin GPIO_PIN_0
#define P1_09_GPIO_Port GPIOD
#define EN_BUZZER_PWR_Pin GPIO_PIN_1
#define EN_BUZZER_PWR_GPIO_Port GPIOD
#define BATT_VLTG_Pin GPIO_PIN_13
#define BATT_VLTG_GPIO_Port GPIOB
#define EN_BATT_MON_Pin GPIO_PIN_6
#define EN_BATT_MON_GPIO_Port GPIOC
#define ADC_PD_RST_Pin GPIO_PIN_15
#define ADC_PD_RST_GPIO_Port GPIOB
#define DW_GP6_Pin GPIO_PIN_13
#define DW_GP6_GPIO_Port GPIOC
#define BATT_CHG_Pin GPIO_PIN_4
#define BATT_CHG_GPIO_Port GPIOE
#define ZPFL_TRIG_Pin GPIO_PIN_5
#define ZPFL_TRIG_GPIO_Port GPIOC
#define SD_DETECT_Pin GPIO_PIN_2
#define SD_DETECT_GPIO_Port GPIOB
#define P1_00_Pin GPIO_PIN_4
#define P1_00_GPIO_Port GPIOC
#define INT_MAG_Pin GPIO_PIN_9
#define INT_MAG_GPIO_Port GPIOA
#define BUZZER_PWM_Pin GPIO_PIN_6
#define BUZZER_PWM_GPIO_Port GPIOA
#define P1_01_Pin GPIO_PIN_4
#define P1_01_GPIO_Port GPIOA
#define P0_13_Pin GPIO_PIN_3
#define P0_13_GPIO_Port GPIOA
#define PD14_Pin GPIO_PIN_14
#define PD14_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */
#define SD_SPI_HANDLE hspi1
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
