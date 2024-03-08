/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

const osThreadAttr_t markTask_attributes = { .name = "markTask", .attr_bits =
		osThreadDetached, .cb_mem = NULL, .cb_size = 0, .stack_mem = NULL,
		.stack_size = 256*6, .priority = (osPriority_t) osPriorityBelowNormal,
		.tz_module = 0, .reserved = 0 };
osThreadId_t markThreadId;

const osThreadAttr_t configTask_attributes = { .name = "configTask", .attr_bits =
		osThreadDetached, .cb_mem = NULL, .cb_size = 0, .stack_mem = NULL,
		.stack_size = 256*8, .priority = (osPriority_t) osPriorityBelowNormal,
		.tz_module = 0, .reserved = 0 };
osThreadId_t configThreadId;

const osThreadAttr_t mainSystemTask_attributes = { .name = "mainSystemTask", .attr_bits =
		osThreadDetached, .cb_mem = NULL, .cb_size = 0, .stack_mem = NULL,
		.stack_size = 512*4, .priority = (osPriority_t) osPriorityNormal,
		.tz_module = 0, .reserved = 0 };
osThreadId_t mainSystemThreadId;

const osThreadAttr_t sampleTask_attributes = { .name = "sampleTask", .attr_bits =
		osThreadDetached, .cb_mem = NULL, .cb_size = 0, .stack_mem = NULL,
		.stack_size = 256*8, .priority = (osPriority_t) osPriorityBelowNormal,
		.tz_module = 0, .reserved = 0 };
osThreadId_t sampleThreadId;

const osThreadAttr_t micTask_attributes = { .name = "micTask", .attr_bits =
		osThreadDetached, .cb_mem = NULL, .cb_size = 0, .stack_mem = NULL,
		.stack_size = 256*12, .priority = (osPriority_t) osPriorityNormal,
		.tz_module = 0, .reserved = 0 };
osThreadId_t micThreadId;

osThreadId_t bmeTaskHandle;
const osThreadAttr_t bmeTask_attributes = { .name = "bmeTask", .attr_bits =
		osThreadDetached, .cb_mem = NULL, .cb_size = 0, .stack_mem = NULL,
		.stack_size = 512 * 6, .priority = (osPriority_t) osPriorityBelowNormal,
		.tz_module = 0, .reserved = 0 };

const osThreadAttr_t batteryMonitorTask_attributes = { .name = "batteryMonTask", .attr_bits =
		osThreadDetached, .cb_mem = NULL, .cb_size = 0, .stack_mem = NULL,
		.stack_size = 512 * 4, .priority = (osPriority_t) osPriorityBelowNormal,
		.tz_module = 0, .reserved = 0 };

const osThreadAttr_t timestampTask_attributes = { .name = "timestampTask", .attr_bits =
		osThreadDetached, .cb_mem = NULL, .cb_size = 0, .stack_mem = NULL,
		.stack_size = 512 * 5, .priority = (osPriority_t) osPriorityBelowNormal,
		.tz_module = 0, .reserved = 0 };

const osThreadAttr_t triggerMarkTask_attributes = { .name = "tiggerMarkTask", .attr_bits =
		osThreadDetached, .cb_mem = NULL, .cb_size = 0, .stack_mem = NULL,
		.stack_size = 512 * 4, .priority = (osPriority_t) osPriorityAboveNormal,
		.tz_module = 0, .reserved = 0 };

const osThreadAttr_t uwbMessageTask_attributes = { .name = "uwbMessageTask", .attr_bits =
		osThreadDetached, .cb_mem = NULL, .cb_size = 0, .stack_mem = NULL,
		.stack_size = 512 * 4, .priority = (osPriority_t) osPriorityBelowNormal,
		.tz_module = 0, .reserved = 0 };

const osThreadAttr_t ledSequencerTask_attributes = { .name = "ledSeqTask", .attr_bits =
		osThreadDetached, .cb_mem = NULL, .cb_size = 0, .stack_mem = NULL,
		.stack_size = 256*3, .priority = (osPriority_t) osPriorityBelowNormal,
		.tz_module = 0, .reserved = 0 };

const osThreadAttr_t chirpTask_attributes = { .name = "chirpTask",
		.attr_bits = osThreadDetached, .cb_mem = NULL, .cb_size = 0,
		.stack_mem = NULL, .stack_size = 256*3, .priority =
				(osPriority_t) osPriorityAboveNormal, .tz_module = 0, .reserved = 0 };

osMutexId_t messageI2C1_LockHandle;
const osMutexAttr_t messageI2C1_Lock_attributes = { .name = "messageI2C1_Lock" };

//osMutexId_t messageSPI1_LockHandle;
//const osMutexAttr_t messageSPI1_Lock_attributes = { .name = "messageSPI1_Lock" };

const osSemaphoreAttr_t messageSPI1_Lock_attributes = { .name = "messageSPI1_Lock" };
const osSemaphoreAttr_t txMsg_Lock_attributes = { .name = "txMsg_Lock" };
const osSemaphoreAttr_t rxMsg_Lock_attributes = { .name = "rxMsg_Lock" };

osSemaphoreId_t messageSPI1_LockBinarySemId;
osSemaphoreId_t txMsg_LockBinarySemId;
osSemaphoreId_t rxMsg_LockBinarySemId;
osMessageQueueId_t markPacketQueueId;
osMessageQueueId_t ledSeqQueueId;
osMessageQueueId_t configChangeQueueId;
osMessageQueueId_t timeSyncQueueId;
osMessageQueueId_t fileWriteQueueId;
osMessageQueueId_t txMsgQueueId;
osMessageQueueId_t rxMsgQueueId;
osTimerId_t mainTaskUpdateId;
osTimerId_t sendSlavesTimestampId;
/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

