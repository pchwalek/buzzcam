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
		.stack_size = 256*8, .priority = (osPriority_t) osPriorityBelowNormal,
		.tz_module = 0, .reserved = 0 };
osThreadId_t markThreadId;

const osThreadAttr_t configTask_attributes = { .name = "configTask", .attr_bits =
		osThreadDetached, .cb_mem = NULL, .cb_size = 0, .stack_mem = NULL,
		.stack_size = 256*12, .priority = (osPriority_t) osPriorityBelowNormal,
		.tz_module = 0, .reserved = 0 };
osThreadId_t configThreadId;

const osThreadAttr_t mainSystemTask_attributes = { .name = "mainSystemTask", .attr_bits =
		osThreadDetached, .cb_mem = NULL, .cb_size = 0, .stack_mem = NULL,
		.stack_size = 256*8, .priority = (osPriority_t) osPriorityBelowNormal,
		.tz_module = 0, .reserved = 0 };
osThreadId_t mainSystemThreadId;

const osThreadAttr_t sampleTask_attributes = { .name = "sampleTask", .attr_bits =
		osThreadDetached, .cb_mem = NULL, .cb_size = 0, .stack_mem = NULL,
		.stack_size = 256*8, .priority = (osPriority_t) osPriorityBelowNormal,
		.tz_module = 0, .reserved = 0 };
osThreadId_t sampleThreadId;

const osThreadAttr_t micTask_attributes = { .name = "micTask", .attr_bits =
		osThreadDetached, .cb_mem = NULL, .cb_size = 0, .stack_mem = NULL,
		.stack_size = 256*16, .priority = (osPriority_t) osPriorityAboveNormal,
		.tz_module = 0, .reserved = 0 };
osThreadId_t micThreadId;

osThreadId_t bmeTaskHandle;
const osThreadAttr_t bmeTask_attributes = { .name = "bmeTask", .attr_bits =
		osThreadDetached, .cb_mem = NULL, .cb_size = 0, .stack_mem = NULL,
		.stack_size = 512 * 5, .priority = (osPriority_t) osPriorityNormal,
		.tz_module = 0, .reserved = 0 };

osMutexId_t messageI2C1_LockHandle;
const osMutexAttr_t messageI2C1_Lock_attributes = { .name = "messageI2C1_Lock" };

//osMutexId_t messageSPI1_LockHandle;
//const osMutexAttr_t messageSPI1_Lock_attributes = { .name = "messageSPI1_Lock" };

const osSemaphoreAttr_t messageSPI1_Lock_attributes = { .name = "messageSPI1_Lock" };

osSemaphoreId_t messageSPI1_LockBinarySemId;
/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

