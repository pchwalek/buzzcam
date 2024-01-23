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
#include "message.pb.h"
#include "cmsis_os2.h"
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
void triggerMark(void *argument);
void mainSystemTask(void *argument);
//void micTask(void *argument);
void acousticSamplingTask(void *argument);
void updateSystemConfig(void *argument);
void sampleTask(void *argument);

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

extern I2C_HandleTypeDef hi2c1;
extern RTC_HandleTypeDef hrtc;
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
#define EN_SD_REG_Pin GPIO_PIN_3
#define EN_SD_REG_GPIO_Port GPIOB
#define EN_UWB_REG_Pin GPIO_PIN_10
#define EN_UWB_REG_GPIO_Port GPIOC
#define EN_SD_REG_2_Pin GPIO_PIN_11
#define EN_SD_REG_2_GPIO_Port GPIOC
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
#define PD13_Pin GPIO_PIN_13
#define PD13_GPIO_Port GPIOD
#define PD12_Pin GPIO_PIN_12
#define PD12_GPIO_Port GPIOD
#define EN_SD_MUX_Pin GPIO_PIN_2
#define EN_SD_MUX_GPIO_Port GPIOD
#define SD_MUX_SEL_Pin GPIO_PIN_7
#define SD_MUX_SEL_GPIO_Port GPIOC
#define SD_DETECT_2_Pin GPIO_PIN_3
#define SD_DETECT_2_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */
#define RTC_NO_REINIT	1

#define SD_SPI_HANDLE hspi1

#define CONFIG_UPDATED_EVENT  0x00000001
#define TERMINATE_EVENT  	  0x00000002
#define COMPLETE_EVENT  	  0x00000004

#define GRAB_SAMPLE_BIT							0x0100
#define TERMINATE_THREAD_BIT					0x0200

#define MAX_INTENSITY 1000

extern packet_t configPacket;
extern packet_t infoPacket;
extern uint8_t buffer[500];
extern size_t message_length;
extern bool status;

extern osMutexId_t messageI2C1_LockHandle;

extern osThreadId_t markThreadId;
extern osThreadId_t configThreadId;
extern osThreadId_t mainSystemThreadId;
extern osThreadId_t micThreadId;
extern osThreadId_t sampleThreadId;
extern osThreadId_t bmeTaskHandle;

extern const osThreadAttr_t markTask_attributes;
extern const osThreadAttr_t configTask_attributes;
extern const osThreadAttr_t mainSystemTask_attributes;
extern const osThreadAttr_t micTask_attributes;
extern const osThreadAttr_t sampleTask_attributes;
extern const osThreadAttr_t micTask_attributes;
extern const osMutexAttr_t messageI2C1_Lock_attributes;
extern const osThreadAttr_t bmeTask_attributes;

void setLED_Blue(uint32_t intensity);
void setLED_Green(uint32_t intensity);
void setLED_Red(uint32_t intensity);
void disableLEDs();
void i2c_error_check(I2C_HandleTypeDef *hi2c);

void updateRTC(uint32_t receivedTime);
void updateRTC_MS(uint64_t receivedTime);
uint64_t getEpoch(void);

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
