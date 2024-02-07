/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_entry.h
  * @author  MCD Application Team
  * @brief   Interface to the application
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
#ifndef APP_ENTRY_H
#define APP_ENTRY_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cmsis_os.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum
{
  ot_TL_CmdBusy,
  ot_TL_CmdAvailable
} ot_TL_CmdStatus_t;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */
extern osMutexId_t MtxHciId;
extern osThreadId_t OsTaskMsgM0ToM4Id;      /* Task managing the M0 to M4 messaging        */

extern osThreadId_t AdvUpdateProcessId;
extern osThreadId_t HciUserEvtProcessId;

extern osThreadId_t defaultTaskHandle;
extern volatile uint8_t bluetoothCmdBusy;
extern osThreadId_t ShciUserEvtProcessId;
void ot_StatusNot( ot_TL_CmdStatus_t status );

extern volatile uint8_t stopThread;
/* USER CODE END EV */

/* Exported macros ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
static volatile int FlagCmdProcessingFromM0 = 0;

/* USER CODE END EM */

/* Exported functions ---------------------------------------------*/
void MX_APPE_Config(void);
void MX_APPE_Init(void);
void Init_Exti(void);
void Init_Smps(void);

/* USER CODE BEGIN EF */

/* USER CODE END EF */

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /*APP_ENTRY_H */
