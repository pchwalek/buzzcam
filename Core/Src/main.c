/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"
#include "app_fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "ble_types.h"
#include "pb.h"
#include "pb_decode.h"
#include "pb_encode.h"
#include "message.pb.h"

#include "dts.h"

#include "app_ble.h"
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
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

IPCC_HandleTypeDef hipcc;

RTC_HandleTypeDef hrtc;

SAI_HandleTypeDef hsai_BlockA1;
DMA_HandleTypeDef hdma_sai1_a;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim16;

PCD_HandleTypeDef hpcd_USB_FS;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 256 * 4
};
/* USER CODE BEGIN PV */
uint16_t redVal = 0;
uint16_t greenVal = 0;
uint16_t blueVal = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C3_Init(void);
static void MX_IPCC_Init(void);
static void MX_RTC_Init(void);
static void MX_SAI1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM16_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_I2C1_Init(void);
static void MX_MEMORYMAP_Init(void);
static void MX_RF_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static DTS_STM_Payload_t PackedPayload;


packet_t configPacket = PACKET_INIT_ZERO;
packet_t infoPacket = PACKET_INIT_ZERO;
uint8_t buffer[500];
size_t message_length;
bool status;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Config code for STM32_WPAN (HSE Tuning must be done before system clock configuration) */
  MX_APPE_Config();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* IPCC initialisation */
  MX_IPCC_Init();

  /* USER CODE BEGIN SysInit */
  configPacket.has_header = true;
  configPacket.header.epoch = 1111;
  configPacket.header.system_uid = LL_FLASH_GetUDN();
  configPacket.header.ms_from_start = HAL_GetTick();

  configPacket.which_payload = PACKET_CONFIG_PACKET_TAG;

  configPacket.payload.config_packet.has_audio_config=true;
  configPacket.payload.config_packet.audio_config.bit_resolution=MIC_BIT_RESOLUTION_BIT_RES_16;
  configPacket.payload.config_packet.audio_config.channel_1=true;
  configPacket.payload.config_packet.audio_config.channel_2=true;
  configPacket.payload.config_packet.audio_config.has_audio_compression=true;
  configPacket.payload.config_packet.audio_config.audio_compression.compression_factor=0;
  configPacket.payload.config_packet.audio_config.audio_compression.compression_type=COMPRESSION_TYPE_OPUS;
  configPacket.payload.config_packet.audio_config.audio_compression.enabled=false;
  configPacket.payload.config_packet.audio_config.estimated_record_time=12345678; //placeholder
  configPacket.payload.config_packet.audio_config.sample_freq=MIC_SAMPLE_FREQ_SAMPLE_RATE_48000;

  configPacket.payload.config_packet.has_camera_control=true;
  configPacket.payload.config_packet.camera_control.capture=false;
  configPacket.payload.config_packet.camera_control.pair_with_nearby_cameras=false;
  configPacket.payload.config_packet.camera_control.wakeup_cameras=false;

  configPacket.payload.config_packet.has_low_power_config=true;
  configPacket.payload.config_packet.low_power_config.low_power_mode=false;

  configPacket.payload.config_packet.has_network_state=true;
  configPacket.payload.config_packet.network_state.discovered_device_uid_count=12;
  configPacket.payload.config_packet.network_state.number_of_discovered_devices=configPacket.payload.config_packet.network_state.discovered_device_uid_count;
  configPacket.payload.config_packet.network_state.discovered_device_uid[0] = 0xDEADBEEF;
  configPacket.payload.config_packet.network_state.discovered_device_uid[1] = 0xDEADBEAF;
  configPacket.payload.config_packet.network_state.discovered_device_uid[2] = 0xDEADBEBF;
  configPacket.payload.config_packet.network_state.discovered_device_uid[3] = 0xDEADBECF;
  configPacket.payload.config_packet.network_state.discovered_device_uid[4] = 0xDEADBEDF;
  configPacket.payload.config_packet.network_state.discovered_device_uid[5] = 0xDEADBEEF;
  configPacket.payload.config_packet.network_state.discovered_device_uid[6] = 0xDEADBEFF;
  configPacket.payload.config_packet.network_state.discovered_device_uid[7] = 0xDEADBAEF;
  configPacket.payload.config_packet.network_state.discovered_device_uid[8] = 0xDEADBBEF;
  configPacket.payload.config_packet.network_state.discovered_device_uid[9] = 0xDEADBCEF;
  configPacket.payload.config_packet.network_state.discovered_device_uid[10] = 0xDEADBDEF;
  configPacket.payload.config_packet.network_state.discovered_device_uid[11] = 0xDEADBFEF;
  configPacket.payload.config_packet.network_state.number_of_discovered_devices=123;
  configPacket.payload.config_packet.network_state.force_rediscovery=false;

  configPacket.payload.config_packet.has_sensor_config=true;
  configPacket.payload.config_packet.sensor_config.enable_gas=true;
  configPacket.payload.config_packet.sensor_config.enable_humidity=true;
  configPacket.payload.config_packet.sensor_config.enable_temperature=true;
  configPacket.payload.config_packet.sensor_config.sample_period_ms=1000;

  configPacket.payload.config_packet.schedule_config_count = 2;
  configPacket.payload.config_packet.schedule_config[0].monday = true;
  configPacket.payload.config_packet.schedule_config[0].wednesday = true;
  configPacket.payload.config_packet.schedule_config[0].start_hour = 7;
  configPacket.payload.config_packet.schedule_config[0].start_minute = 35;
  configPacket.payload.config_packet.schedule_config[0].stop_hour = 10;
  configPacket.payload.config_packet.schedule_config[0].stop_minute = 05;
  configPacket.payload.config_packet.schedule_config[1].tuesday = true;
  configPacket.payload.config_packet.schedule_config[1].wednesday = true;
  configPacket.payload.config_packet.schedule_config[1].thursday = true;
  configPacket.payload.config_packet.schedule_config[1].friday = true;
  configPacket.payload.config_packet.schedule_config[1].start_hour = 13;
  configPacket.payload.config_packet.schedule_config[1].start_minute = 03;
  configPacket.payload.config_packet.schedule_config[1].stop_hour = 17;
  configPacket.payload.config_packet.schedule_config[1].stop_minute = 47;

  infoPacket.has_header = true;
  infoPacket.header.epoch = 1111;
  infoPacket.header.system_uid = LL_FLASH_GetUDN();
  infoPacket.header.ms_from_start = HAL_GetTick();

  infoPacket.which_payload = PACKET_SYSTEM_INFO_PACKET_TAG;
  infoPacket.payload.system_info_packet.has_battery_state = true;
  infoPacket.payload.system_info_packet.battery_state.charging=false;
  infoPacket.payload.system_info_packet.battery_state.has_percentage=true;
  infoPacket.payload.system_info_packet.battery_state.percentage=50.0;
  infoPacket.payload.system_info_packet.battery_state.voltage=3.75;

  infoPacket.payload.system_info_packet.has_discovered_devices = true;
  infoPacket.payload.system_info_packet.discovered_devices.device_count=12;
  infoPacket.payload.system_info_packet.discovered_devices.number_of_devices=infoPacket.payload.system_info_packet.discovered_devices.device_count;
  infoPacket.payload.system_info_packet.discovered_devices.device[0].uid = 0xDEADBEEF;
  infoPacket.payload.system_info_packet.discovered_devices.device[1].uid = 0xDEADBEAF;
  infoPacket.payload.system_info_packet.discovered_devices.device[2].uid = 0xDEADBEBF;
  infoPacket.payload.system_info_packet.discovered_devices.device[3].uid = 0xDEADBECF;
  infoPacket.payload.system_info_packet.discovered_devices.device[4].uid = 0xDEADBEDF;
  infoPacket.payload.system_info_packet.discovered_devices.device[5].uid = 0xDEADBEEF;
  infoPacket.payload.system_info_packet.discovered_devices.device[6].uid = 0xDEADBEFF;
  infoPacket.payload.system_info_packet.discovered_devices.device[7].uid = 0xDEADBAEF;
  infoPacket.payload.system_info_packet.discovered_devices.device[8].uid = 0xDEADBBEF;
  infoPacket.payload.system_info_packet.discovered_devices.device[9].uid = 0xDEADBCEF;
  infoPacket.payload.system_info_packet.discovered_devices.device[10].uid = 0xDEADBDEF;
  infoPacket.payload.system_info_packet.discovered_devices.device[11].uid = 0xDEADBFEF;
  infoPacket.payload.system_info_packet.discovered_devices.device[0].range = 1.0;
  infoPacket.payload.system_info_packet.discovered_devices.device[1].range = 2;
  infoPacket.payload.system_info_packet.discovered_devices.device[2].range = 3;
  infoPacket.payload.system_info_packet.discovered_devices.device[3].range = 4;
  infoPacket.payload.system_info_packet.discovered_devices.device[4].range = 5;
  infoPacket.payload.system_info_packet.discovered_devices.device[5].range = 6;
  infoPacket.payload.system_info_packet.discovered_devices.device[6].range = 7;
  infoPacket.payload.system_info_packet.discovered_devices.device[7].range = 8;
  infoPacket.payload.system_info_packet.discovered_devices.device[8].range = 9.12;
  infoPacket.payload.system_info_packet.discovered_devices.device[9].range = 10.2321;
  infoPacket.payload.system_info_packet.discovered_devices.device[10].range =11.2;
  infoPacket.payload.system_info_packet.discovered_devices.device[11].range = 12.1;

  infoPacket.payload.system_info_packet.has_mark_state = true;
//  infoPacket.payload.system_info_packet.mark_state.beep_enabled=false;
  infoPacket.payload.system_info_packet.mark_state.mark_number=123;
  infoPacket.payload.system_info_packet.mark_state.timestamp_unix=123;

  infoPacket.payload.system_info_packet.has_sdcard_state = true;
  infoPacket.payload.system_info_packet.sdcard_state.detected=true;
  infoPacket.payload.system_info_packet.sdcard_state.estimated_remaining_recording_time=1234;
  infoPacket.payload.system_info_packet.sdcard_state.space_remaining=1234;

  infoPacket.payload.system_info_packet.has_simple_sensor_reading = true;
  infoPacket.payload.system_info_packet.simple_sensor_reading.co2=458.0;
  infoPacket.payload.system_info_packet.simple_sensor_reading.humidity=40.0;
  infoPacket.payload.system_info_packet.simple_sensor_reading.index=123;
  infoPacket.payload.system_info_packet.simple_sensor_reading.light_level=100.0;
  infoPacket.payload.system_info_packet.simple_sensor_reading.temperature=76;
  infoPacket.payload.system_info_packet.simple_sensor_reading.timestamp_unix=1234;
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C3_Init();
  MX_RTC_Init();
  MX_SAI1_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM16_Init();
  if (MX_FATFS_Init() != APP_OK) {
    Error_Handler();
  }
  MX_USB_PCD_Init();
  MX_I2C1_Init();
  MX_MEMORYMAP_Init();
  MX_RF_Init();
  /* USER CODE BEGIN 2 */




  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
//  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
  /* USER CODE BEGIN RTOS_THREADS */
  mainSystemThreadId = osThreadNew(mainSystemTask, NULL, &mainSystemTask_attributes);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Init code for STM32_WPAN */
  MX_APPE_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Macro to configure the PLL multiplication factor
  */
  __HAL_RCC_PLL_PLLM_CONFIG(RCC_PLLM_DIV2);

  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_MSI);

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_MEDIUMHIGH);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_10;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS|RCC_PERIPHCLK_RFWAKEUP
                              |RCC_PERIPHCLK_SAI1|RCC_PERIPHCLK_USB;
  PeriphClkInitStruct.PLLSAI1.PLLN = 6;
  PeriphClkInitStruct.PLLSAI1.PLLP = RCC_PLLP_DIV2;
  PeriphClkInitStruct.PLLSAI1.PLLQ = RCC_PLLQ_DIV2;
  PeriphClkInitStruct.PLLSAI1.PLLR = RCC_PLLR_DIV2;
  PeriphClkInitStruct.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_SAI1CLK|RCC_PLLSAI1_USBCLK;
  PeriphClkInitStruct.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLSAI1;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInitStruct.RFWakeUpClockSelection = RCC_RFWKPCLKSOURCE_HSE_DIV1024;
  PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSE;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE0;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN Smps */

  /* USER CODE END Smps */
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00707CBB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x00707CBB;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief IPCC Initialization Function
  * @param None
  * @retval None
  */
static void MX_IPCC_Init(void)
{

  /* USER CODE BEGIN IPCC_Init 0 */

  /* USER CODE END IPCC_Init 0 */

  /* USER CODE BEGIN IPCC_Init 1 */

  /* USER CODE END IPCC_Init 1 */
  hipcc.Instance = IPCC;
  if (HAL_IPCC_Init(&hipcc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IPCC_Init 2 */

  /* USER CODE END IPCC_Init 2 */

}

/**
  * @brief MEMORYMAP Initialization Function
  * @param None
  * @retval None
  */
static void MX_MEMORYMAP_Init(void)
{

  /* USER CODE BEGIN MEMORYMAP_Init 0 */

  /* USER CODE END MEMORYMAP_Init 0 */

  /* USER CODE BEGIN MEMORYMAP_Init 1 */

  /* USER CODE END MEMORYMAP_Init 1 */
  /* USER CODE BEGIN MEMORYMAP_Init 2 */

  /* USER CODE END MEMORYMAP_Init 2 */

}

/**
  * @brief RF Initialization Function
  * @param None
  * @retval None
  */
static void MX_RF_Init(void)
{

  /* USER CODE BEGIN RF_Init 0 */

  /* USER CODE END RF_Init 0 */

  /* USER CODE BEGIN RF_Init 1 */

  /* USER CODE END RF_Init 1 */
  /* USER CODE BEGIN RF_Init 2 */

  /* USER CODE END RF_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = CFG_RTC_ASYNCH_PRESCALER;
  hrtc.Init.SynchPrediv = CFG_RTC_SYNCH_PRESCALER;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.SubSeconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SAI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SAI1_Init(void)
{

  /* USER CODE BEGIN SAI1_Init 0 */

  /* USER CODE END SAI1_Init 0 */

  /* USER CODE BEGIN SAI1_Init 1 */

  /* USER CODE END SAI1_Init 1 */
  hsai_BlockA1.Instance = SAI1_Block_A;
  hsai_BlockA1.Init.AudioMode = SAI_MODEMASTER_RX;
  hsai_BlockA1.Init.Synchro = SAI_ASYNCHRONOUS;
  hsai_BlockA1.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockA1.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
  hsai_BlockA1.Init.MckOverSampling = SAI_MCK_OVERSAMPLING_DISABLE;
  hsai_BlockA1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockA1.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_48K;
  hsai_BlockA1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockA1.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockA1.Init.CompandingMode = SAI_NOCOMPANDING;
  if (HAL_SAI_InitProtocol(&hsai_BlockA1, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_16BIT, 2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SAI1_Init 2 */

  /* USER CODE END SAI1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 31;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 31;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 99;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 49;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */
  HAL_TIM_MspPostInit(&htim16);

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SD_CS_Pin|EN_UWB_REG_Pin|EN_SD_REG_Pin|EN_MIC_PWR_Pin
                          |EN_BATT_MON_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, EN_3V3_ALT_Pin|ADC_PD_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(EN_BUZZER_PWR_GPIO_Port, EN_BUZZER_PWR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : P0_15_Pin DW_GP6_Pin P1_00_Pin */
  GPIO_InitStruct.Pin = P0_15_Pin|DW_GP6_Pin|P1_00_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SD_CS_Pin EN_UWB_REG_Pin EN_SD_REG_Pin EN_MIC_PWR_Pin
                           EN_BATT_MON_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin|EN_UWB_REG_Pin|EN_SD_REG_Pin|EN_MIC_PWR_Pin
                          |EN_BATT_MON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : EN_3V3_ALT_Pin ADC_PD_RST_Pin */
  GPIO_InitStruct.Pin = EN_3V3_ALT_Pin|ADC_PD_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : INT1_IMU_XL_Pin INT_MAG_Pin */
  GPIO_InitStruct.Pin = INT1_IMU_XL_Pin|INT_MAG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : P1_09_Pin PD14_Pin */
  GPIO_InitStruct.Pin = P1_09_Pin|PD14_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : EN_BUZZER_PWR_Pin */
  GPIO_InitStruct.Pin = EN_BUZZER_PWR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(EN_BUZZER_PWR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BATT_CHG_Pin */
  GPIO_InitStruct.Pin = BATT_CHG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BATT_CHG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ZPFL_TRIG_Pin */
  GPIO_InitStruct.Pin = ZPFL_TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ZPFL_TRIG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_DETECT_Pin */
  GPIO_InitStruct.Pin = SD_DETECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SD_DETECT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : P1_01_Pin P0_13_Pin */
  GPIO_InitStruct.Pin = P1_01_Pin|P0_13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
uint64_t GetEpoch(){
	return 1234;
}

void sampleTask(void *argument){
	audio_config_t audio_config;
	memcpy((uint8_t*) &audio_config,(uint8_t*)argument,sizeof(audio_config_t));

//	if(audio_config.has_audio_compression){}
//	if(audio_config.sample_freq == MIC_SAMPLE_FREQ_SAMPLE_RATE_44100){}
//	if(audio_config.channel_1){}
//	if(audio_config.channel_2){}
//	if(audio_config.estimated_record_time){}

	uint32_t flags = 0;

	while(1){
		osDelay(100);

		flags = osThreadFlagsWait(TERMINATE_EVENT, osFlagsWaitAny, 0);
		if(flags & TERMINATE_EVENT){
//		    osThreadFlagsSet(sampleThreadId, COMPLETE_EVENT);
//			osThreadExit();
			vTaskDelete(NULL);
		}

	}
}

void mainSystemTask(void *argument){
	 uint32_t flags = 0;

	/* read current config from SD card */

	/* initiate system */
	if(configPacket.payload.config_packet.has_audio_config & configPacket.payload.config_packet.enable_recording){
		osThreadState_t state = osThreadGetState(sampleThreadId);
		if( (state != osThreadReady) || (state != osThreadRunning) || (state != osThreadInactive) || (state != osThreadBlocked) ){
			sampleThreadId = osThreadNew(sampleTask, &configPacket.payload.config_packet.audio_config, &sampleTask_attributes);
		}
	}

	while(1){
		tBleStatus ret = BLE_STATUS_INVALID_PARAMS;
		while(1){
			setLED_Red(150);
			osDelay(500);
			setLED_Red(0);
			osDelay(500);

			setLED_Green(150);
			osDelay(500);
			setLED_Green(0);
			osDelay(500);

			setLED_Blue(150);
			osDelay(500);
			setLED_Blue(0);
			osDelay(500);
//			deactivateCameraMode();
////			activateCameraMode();
//			osDelay(30000);
//			DTS_CamCtrl(POWER_LONG_PRESS);
//			osDelay(30000);
//			DTS_CamCtrl(WAKEUP_CAMERAS);
//			osDelay(60000);
//			Adv_Request(APP_BLE_LP_ADV);
//			osDelay(10000);
//			DTS_CamCtrl(SCREEN_TOGGLE);
//			osDelay(30000);
//			DTS_CamCtrl(SHUTTER);
//			osDelay(10000);
//			DTS_CamCtrl(SCREEN_TOGGLE);
//			osDelay(60000);
//			DTS_CamCtrl(SHUTTER);
//			osDelay(30000);
//			deactivateCameraMode();
//			osDelay(30000);
//
////			Adv_Request(APP_BLE_LP_ADV);
////			ret = aci_gap_update_adv_data(sizeof(a_ManufData), (uint8_t*) a_ManufData);
		}

		/* wait for an event to occur */
		flags = osThreadFlagsWait(CONFIG_UPDATED_EVENT, osFlagsWaitAny, osWaitForever);
		if(flags & CONFIG_UPDATED_EVENT){
			/* system config has been updated. Restart system with new config */

			/* turn off active threads and wait until finished */
			// alert threads via flags
			osThreadFlagsSet(sampleThreadId, TERMINATE_EVENT);

			// wait for threads to call osThreadExit()
			while(osThreadTerminated != osThreadGetState(sampleThreadId)){
				osDelay(100);
			}
			/* reinitiate system */

		}
	}

	while(1){
		osDelay(10);
	}

	vTaskDelete(NULL);
}

void updateSystemConfig(void *argument){
	config_packet_t new_config;
	memcpy((uint8_t*) &new_config,(uint8_t*)argument,sizeof(config_packet_t));

	/* (1) update config */

	 configPacket.has_header = true;
	 configPacket.header.epoch = GetEpoch();
	 configPacket.header.system_uid = LL_FLASH_GetUDN();
	 configPacket.header.ms_from_start = HAL_GetTick();

	 if(new_config.has_audio_config){
		 memcpy((uint8_t*)&configPacket.payload.config_packet.audio_config,
	 				(uint8_t*)&new_config.audio_config,
						sizeof(new_config.audio_config));
	 }
	 if(new_config.has_camera_control){
		 if(configPacket.payload.config_packet.camera_control.capture){
			 /* trigger a capture */
		 }
		 if(configPacket.payload.config_packet.camera_control.pair_with_nearby_cameras){
			 /* pair with nearby cameras */
		 }
		 if(configPacket.payload.config_packet.camera_control.wakeup_cameras){
			 /* wake up cameras */
		 }
	 }
	 if(new_config.has_low_power_config){
		 memcpy((uint8_t*)&configPacket.payload.config_packet.low_power_config,
		 	 				(uint8_t*)&new_config.low_power_config,
		 						sizeof(new_config.low_power_config));

	 		 if( configPacket.payload.config_packet.low_power_config.low_power_mode){
	 			 /* enable low power mode settings */
	 		 }else{
	 			/* disable low power mode settings */
	 		 }
	 }

	 if(new_config.has_network_state){
	 		 if(configPacket.payload.config_packet.network_state.force_rediscovery){
	 			 /* trigger UWB rediscovery */
	 		 }
	 	 }
	 if(new_config.has_sensor_config){
	 		 memcpy((uint8_t*)&configPacket.payload.config_packet.sensor_config,
	 				(uint8_t*)&new_config.sensor_config,
						sizeof(new_config.sensor_config));
	 	 }

	 if(new_config.schedule_config_count != 0){
		 memcpy((uint8_t*)&configPacket.payload.config_packet.schedule_config,
			 				(uint8_t*)&new_config.schedule_config,
								sizeof(new_config.schedule_config));
		 configPacket.payload.config_packet.schedule_config_count = new_config.schedule_config_count;
	 }

	/* (2) alert master thread that new config exists */

	 osThreadFlagsSet(mainSystemThreadId, CONFIG_UPDATED_EVENT);


	/* (3) update config characteristic */

	tBleStatus ret;
	/* Create a stream that will write to our buffer. */
	pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
	/* Now we are ready to encode the message! */
	status = pb_encode(&stream, PACKET_FIELDS, &configPacket);
	PackedPayload.pPayload = (uint8_t*) buffer;
	PackedPayload.Length = stream.bytes_written;
	if(status) ret = DTS_STM_UpdateChar(BUZZCAM_CONFIG_CHAR_UUID, &PackedPayload);

	vTaskDelete(NULL);
}

void triggerMark(void *argument){
	mark_packet_t new_mark;
	memcpy((uint8_t*) &new_mark,(uint8_t*)argument,sizeof(mark_packet_t));
	infoPacket.payload.system_info_packet.mark_state.mark_number++;
	infoPacket.payload.system_info_packet.mark_state.timestamp_unix=GetEpoch();

	infoPacket.has_header = true;
	infoPacket.header.epoch = infoPacket.payload.system_info_packet.mark_state.timestamp_unix;
	infoPacket.header.ms_from_start = HAL_GetTick();

	/* Create a stream that will write to our buffer. */
	pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
	/* Now we are ready to encode the message! */
	status = pb_encode(&stream, PACKET_FIELDS, &infoPacket);
	PackedPayload.pPayload = (uint8_t*) buffer;
	PackedPayload.Length = stream.bytes_written;
	tBleStatus ret;
	if(status) ret = DTS_STM_UpdateChar(BUZZCAM_INFO_CHAR_UUID, &PackedPayload);

	/* trigger beep if enabled */
	if(new_mark.beep_enabled){
		 /* start buzzer pwm */
		  uint16_t index = 10;
		  HAL_TIM_Base_Start(&htim16);
		  HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);

		  while(1){

		  htim16.Instance->ARR = index;
		  htim16.Instance->CCR1 = index >> 1;

		  osDelay(5);

			index+=2;
			if(index == 500) {
				/* stop buzzer pwm */
				HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);
				osDelay(10);
				break;
			}
		  }
	}

	size_t buffer_size;
	if(new_mark.has_annotation){
		buffer_size = 20 + 1 + 10 + 1 + 1 + strlen(new_mark.annotation) + 1;
	}else{
		buffer_size = 20 + 1 + 10 + 1 + 1 + 1;
	}

//	char result[buffer_size];
//	if(new_mark.has_annotation){
//		sprintf(result, "%lu,%lu,%d,%s", infoPacket.payload.system_info_packet.mark_state.timestamp_unix, infoPacket.header.ms_from_start, new_mark.beep_enabled, new_mark.annotation);
//	}else{
//		sprintf(result, "%lu,%lu,%d,", infoPacket.payload.system_info_packet.mark_state.timestamp_unix, infoPacket.header.ms_from_start, new_mark.beep_enabled);
//	}


	/* save to file */


    // Terminate the thread
	  HAL_TIM_Base_Stop(&htim16);
	  HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);
//    osThreadExit();
    vTaskDelete( NULL );
}

/* USER CODE END 4 */
void setLED_Green(uint32_t intensity){
	if(intensity > MAX_INTENSITY) intensity = MAX_INTENSITY;

	greenVal = intensity;

	if( greenVal == 0){
		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
	}else{
		if( (redVal == 0) && (blueVal == 0) ){
			HAL_TIM_Base_Start(&htim2);
		}
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	}

	htim2.Instance->CCR1 = MAX_INTENSITY - intensity;

	if( (redVal == 0) && (greenVal == 0) && (blueVal == 0) ){
		HAL_TIM_Base_Stop(&htim2);
	}
}

void setLED_Blue(uint32_t intensity){
	if(intensity > MAX_INTENSITY) intensity = MAX_INTENSITY;

	blueVal = intensity;

	if( blueVal == 0){
		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
	}else{
		if( (greenVal == 0) && (redVal == 0) ){
			HAL_TIM_Base_Start(&htim2);
		}
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	}

	htim2.Instance->CCR2 = MAX_INTENSITY - intensity;

	if( (redVal == 0) && (greenVal == 0) && (blueVal == 0) ){
		HAL_TIM_Base_Stop(&htim2);
	}
}


void setLED_Red(uint32_t intensity){
	if(intensity > MAX_INTENSITY) intensity = MAX_INTENSITY;

	redVal = intensity;

	if( redVal == 0){
		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
	}else{
		if( (greenVal == 0) && (blueVal == 0) ){
			HAL_TIM_Base_Start(&htim2);
		}
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	}

	htim2.Instance->CCR3 = MAX_INTENSITY - intensity;

	if( (redVal == 0) && (greenVal == 0) && (blueVal == 0) ){

		HAL_TIM_Base_Stop(&htim2);
	}
}

void disableLEDs(){
	setLED_Green(0);
	setLED_Red(0);
	setLED_Blue(0);

	HAL_TIM_Base_Stop(&htim2);
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
