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

const osThreadAttr_t markTask_attributes = {
  .name = "markTask",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = 256 * 8
};
osThreadId_t markThreadId;

const osThreadAttr_t configTask_attributes = {
  .name = "configTask",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = 256 * 12
};
osThreadId_t configThreadId;

const osThreadAttr_t mainSystemTask_attributes = {
  .name = "mainSystemTask",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = 256 * 8
};
osThreadId_t mainSystemThreadId;

const osThreadAttr_t sampleTask_attributes = {
  .name = "sampleTask",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = 256 * 8
};
osThreadId_t sampleThreadId;

const osThreadAttr_t micTask_attributes = {
  .name = "micTask",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 256 * 16
};
osThreadId_t micThreadId;
/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

