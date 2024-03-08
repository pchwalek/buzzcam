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
#include "uwb_i2c_proto.pb.h"
#include "cmsis_os2.h"

#include "projdefs.h"


//#define OPENTHREAD_CONFIG_FILE <openthread_api_config_concurrent.h>

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
void writeDefaultConfig(void);

void SystemClock_Config(void);

void triggerMarkTask(void *argument);
void mainSystemTask(void *argument);
//void micTask(void *argument);
void acousticSamplingTask(void *argument);
void updateSystemConfig(void *argument);
void sampleTask(void *argument);
void alertMainTask(void *argument);

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

typedef struct {
  uint32_t delimiter;
  int16_t mag_cal_vals[3];
} MagCal;

typedef struct{
	uint16_t blue_val;
	uint16_t green_val;
	uint16_t red_val;
	uint16_t duration;
} colorConfig;

typedef struct{
	config_packet_t config;
	uint8_t fromMaster;
} configChange;

typedef struct{
	uint64_t epoch;
	uint32_t ms_from_start;
	uint32_t counter;
} chirp_event_t;

typedef struct{
	uint32_t system_uid;
	beecam_uwb_i2c_downlink_normal_distribution_t normal_distribution;
} uwb_range_packet_multi_t;

extern I2C_HandleTypeDef hi2c1;
extern RTC_HandleTypeDef hrtc;
extern SPI_HandleTypeDef hspi1;
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
#define BATT_VLTGA8_Pin GPIO_PIN_8
#define BATT_VLTGA8_GPIO_Port GPIOA
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
//#define MASTER_NODE 1

#define UWB_ALERT_Pin GPIO_PIN_4
#define UWB_ALERT_GPIO_Port GPIOC

//#define TESTING_ACTIVE	1

//#define FLAC__HAS_OGG 0
//#define PACKAGE_VERSION 2.61
#define TFLAC_32BIT_ONLY 1

#define RTC_NO_REINIT	1

#define SD_SPI_HANDLE hspi1

#define UPDATE_EVENT  		  0x00000001
#define TERMINATE_EVENT  	  0x00000002
#define COMPLETE_EVENT  	  0x00000004
#define RTC_EVENT		  	  0x00000008
#define CHIRP_EVENT		  	  0x00000010

#define CONFIG_UPDATED_EVENT  0x00000001
#define CAMERA_EVENT		  0x00000004
#define FORMAT_MEMORY		  0x00000008
#define OPENTHREAD_EVENT	  0x00000010
#define UWB_START			  0x00000020
#define UWB_UPDATE_ADDR	      0x00000040
#define UWB_UPDATE_RANGE      0x00000080
#define MAG_CAL_EVENT		  0x00000100

#define UWB_MESSAGE_ALERT	  0x00000001
#define UWB_START_RANGING	  0x00000002
#define UWB_GET_INFO		  0x00000004

#define IS_CONFIG_EVENT(X)				(((X & CONFIG_UPDATED_EVENT) == CONFIG_UPDATED_EVENT) ? (1) : (0))
#define IS_OPENTHREAD_EVENT(X)			(((X & OPENTHREAD_EVENT) == OPENTHREAD_EVENT) ? (1) : (0))
#define IS_CAMERA_EVENT(X)				(((X & CAMERA_EVENT) == CAMERA_EVENT) ? (1) : (0))
#define IS_FORMAT_MEMORY_EVENT(X)		(((X & FORMAT_MEMORY) == FORMAT_MEMORY) ? (1) : (0))
#define IS_UWB_START_EVENT(X)			(((X & UWB_START) == UWB_START) ? (1) : (0))
#define IS_UWB_UPDATE_RANGE_EVENT(X)	(((X & UWB_UPDATE_RANGE) == UWB_UPDATE_RANGE) ? (1) : (0))
#define IS_MAG_CAL_EVENT(X)				(((X & MAG_CAL_EVENT) == MAG_CAL_EVENT) ? (1) : (0))

#define GRAB_SAMPLE_BIT							0x0100
#define TERMINATE_THREAD_BIT					0x0200

#define MAX_INTENSITY 1000

extern packet_t configPacket;
extern packet_t infoPacket;
extern uint8_t buffer[500];
extern size_t message_length;
extern bool status;

extern packet_t rxPacket;
extern packet_t txPacket;

extern osMutexId_t messageI2C1_LockHandle;
//extern osMutexId_t messageSPI1_LockHandle;
extern osSemaphoreId_t messageSPI1_LockBinarySemId;
extern osSemaphoreId_t txMsg_LockBinarySemId;
extern osSemaphoreId_t rxMsg_LockBinarySemId;
extern osMessageQueueId_t markPacketQueueId;
extern osMessageQueueId_t ledSeqQueueId;
extern osMessageQueueId_t configChangeQueueId;
extern osMessageQueueId_t timeSyncQueueId;
extern osMessageQueueId_t fileWriteQueueId;
extern osMessageQueueId_t txMsgQueueId;
extern osMessageQueueId_t rxMsgQueueId;

extern osTimerId_t mainTaskUpdateId;
extern osTimerId_t sendSlavesTimestampId;

extern osThreadId_t markThreadId;
extern osThreadId_t configThreadId;
extern osThreadId_t mainSystemThreadId;
extern osThreadId_t micThreadId;
extern osThreadId_t sampleThreadId;
extern osThreadId_t bmeTaskHandle;
extern osThreadId_t chirpTaskHandle;
extern osThreadId_t fileWriteSyncTaskId;
extern osThreadId_t triggerMarkTaskId;
extern osThreadId_t uwbMessageTaskId;
extern osThreadId_t ledSequencerId;

extern osTimerId_t periodicBatteryMonitorTimer_id;

extern const osThreadAttr_t markTask_attributes;
extern const osThreadAttr_t configTask_attributes;
extern const osThreadAttr_t mainSystemTask_attributes;
extern const osThreadAttr_t micTask_attributes;
extern const osThreadAttr_t sampleTask_attributes;
extern const osThreadAttr_t micTask_attributes;
extern const osMutexAttr_t messageI2C1_Lock_attributes;
extern const osSemaphoreAttr_t messageSPI1_Lock_attributes;
extern const osSemaphoreAttr_t txMsg_Lock_attributes;
extern const osSemaphoreAttr_t rxMsg_Lock_attributes;
extern const osThreadAttr_t bmeTask_attributes;
extern const osThreadAttr_t batteryMonitorTask_attributes;
extern const osThreadAttr_t timestampTask_attributes;
extern const osThreadAttr_t triggerMarkTask_attributes;
extern const osThreadAttr_t chirpTask_attributes;
extern const osThreadAttr_t uwbMessageTask_attributes;
extern const osThreadAttr_t ledSequencerTask_attributes;

extern beecam_uwb_i2c_peer_address_t rangingAddr;
extern beecam_uwb_i2c_device_info_t local_uwbInfo;

extern volatile uint8_t coapSetup;

void setLED_Blue(uint32_t intensity);
void setLED_Green(uint32_t intensity);
void setLED_Red(uint32_t intensity);
void disableLEDs();
void toggledRed();
void toggledGreen();
void toggledBlue();
void i2c_error_check(I2C_HandleTypeDef *hi2c);

void uint64ToString(uint64_t num, char* str);

void updateRTC(uint32_t receivedTime);
void updateRTC_MS(uint64_t receivedTime);
uint64_t getEpoch(void);

char * ftoa(double f, char * buf, int precision);

uint8_t check_file_exists(const char* path);

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
