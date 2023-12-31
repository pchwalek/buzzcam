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
#define MAX_INTENSITY 1000

#define AUDIO_BUFFER_LEN		(48000)
//#define AUDIO_BUFFER_LEN		(1000)
#define AUDIO_BUFFER_HALF_LEN	(AUDIO_BUFFER_LEN >> 1)
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
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart1;

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

FATFS SDFatFs; /* File system object for SD card logical drive */
FIL MyFile; /* File object */
FIL	WavFile;
DIR dir;
FIL file;
UINT bytes_written;
char SDPath[4]; /* SD card logical drive path */


//FX_MEDIA        sd_disk;
//FX_FILE         fx_file;
//FX_FILE			WavFile;

uint32_t media_memory[512 / sizeof(uint32_t)];

WAVE_FormatTypeDef WaveFormat;

uint8_t pHeaderBuff[44];

uint32_t byteswritten = 0;
volatile uint32_t sampleCntr = 0;

uint16_t audioSample[AUDIO_BUFFER_LEN] = {0};

volatile uint8_t SAI_HALF_CALLBACK = 0;
volatile uint8_t SAI_FULL_CALLBACK = 0;
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
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_RF_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
void EnableExtADC(bool state);
void RunADC();

void WAV_RECORD_TEST(void);
static uint32_t WavProcess_HeaderInit(uint8_t* pHeader, WAVE_FormatTypeDef* pWaveFormatStruct);
static uint32_t WavProcess_EncInit(uint32_t Freq, uint8_t *pHeader);
static uint32_t WavProcess_HeaderUpdate(uint8_t* pHeader, uint32_t bytesWritten);
static void WavUpdateHeaderSize(uint64_t totalBytesWritten);
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
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_RF_Init();
  /* USER CODE BEGIN 2 */
//  GPIO_InitTypeDef GPIO_InitStruct = {0};
//  GPIO_InitStruct.Pin = GPIO_PIN_5;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//
//  GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

//	/* Turn on microphone and ADC */
//	HAL_GPIO_WritePin(EN_MIC_PWR_GPIO_Port, EN_MIC_PWR_Pin, GPIO_PIN_SET);
//
//	/* initialize SD card */
//	  HAL_GPIO_WritePin(EN_SD_REG_GPIO_Port, EN_SD_REG_Pin, GPIO_PIN_SET);
//	  HAL_GPIO_WritePin(EN_SD_REG_2_GPIO_Port, EN_SD_REG_2_Pin, GPIO_PIN_SET);
//
//	  /* start mux */
//	  HAL_GPIO_WritePin(EN_SD_MUX_GPIO_Port, EN_SD_MUX_Pin, GPIO_PIN_RESET); // enable mux
//	  HAL_GPIO_WritePin(SD_MUX_SEL_GPIO_Port, SD_MUX_SEL_Pin, GPIO_PIN_RESET);// sd card 1 selected
////	  HAL_GPIO_WritePin(SD_MUX_SEL_GPIO_Port, SD_MUX_SEL_Pin, GPIO_PIN_SET);// sd card 2 selected
//
//while(1){
//	//PB4, PB5, PA5
//
//	HAL_Delay(100);
//	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
//	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_RESET);
//
//	HAL_Delay(100);
//	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
//	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4 | GPIO_PIN_5 , GPIO_PIN_SET);
//}

  enum regAddr
    {
      TEMP_OUT_L        = 0x05, // D
      TEMP_OUT_H        = 0x06, // D

      STATUS_M          = 0x07, // D

      INT_CTRL_M        = 0x12, // D
      INT_SRC_M         = 0x13, // D
      INT_THS_L_M       = 0x14, // D
      INT_THS_H_M       = 0x15, // D

      OFFSET_X_L_M      = 0x16, // D
      OFFSET_X_H_M      = 0x17, // D
      OFFSET_Y_L_M      = 0x18, // D
      OFFSET_Y_H_M      = 0x19, // D
      OFFSET_Z_L_M      = 0x1A, // D
      OFFSET_Z_H_M      = 0x1B, // D
      REFERENCE_X       = 0x1C, // D
      REFERENCE_Y       = 0x1D, // D
      REFERENCE_Z       = 0x1E, // D

      CTRL0             = 0x1F, // D
      CTRL1             = 0x20, // D
      CTRL_REG1_A       = 0x20, // DLH, DLM, DLHC
      CTRL2             = 0x21, // D
      CTRL_REG2_A       = 0x21, // DLH, DLM, DLHC
      CTRL3             = 0x22, // D
      CTRL_REG3_A       = 0x22, // DLH, DLM, DLHC
      CTRL4             = 0x23, // D
      CTRL_REG4_A       = 0x23, // DLH, DLM, DLHC
      CTRL5             = 0x24, // D
      CTRL_REG5_A       = 0x24, // DLH, DLM, DLHC
      CTRL6             = 0x25, // D
      CTRL_REG6_A       = 0x25, // DLHC
      HP_FILTER_RESET_A = 0x25, // DLH, DLM
      CTRL7             = 0x26, // D
      REFERENCE_A       = 0x26, // DLH, DLM, DLHC
      STATUS_A          = 0x27, // D
      STATUS_REG_A      = 0x27, // DLH, DLM, DLHC

      OUT_X_L_A         = 0x28,
      OUT_X_H_A         = 0x29,
      OUT_Y_L_A         = 0x2A,
      OUT_Y_H_A         = 0x2B,
      OUT_Z_L_A         = 0x2C,
      OUT_Z_H_A         = 0x2D,

      FIFO_CTRL         = 0x2E, // D
      FIFO_CTRL_REG_A   = 0x2E, // DLHC
      FIFO_SRC          = 0x2F, // D
      FIFO_SRC_REG_A    = 0x2F, // DLHC

      IG_CFG1           = 0x30, // D
      INT1_CFG_A        = 0x30, // DLH, DLM, DLHC
      IG_SRC1           = 0x31, // D
      INT1_SRC_A        = 0x31, // DLH, DLM, DLHC
      IG_THS1           = 0x32, // D
      INT1_THS_A        = 0x32, // DLH, DLM, DLHC
      IG_DUR1           = 0x33, // D
      INT1_DURATION_A   = 0x33, // DLH, DLM, DLHC
      IG_CFG2           = 0x34, // D
      INT2_CFG_A        = 0x34, // DLH, DLM, DLHC
      IG_SRC2           = 0x35, // D
      INT2_SRC_A        = 0x35, // DLH, DLM, DLHC
      IG_THS2           = 0x36, // D
      INT2_THS_A        = 0x36, // DLH, DLM, DLHC
      IG_DUR2           = 0x37, // D
      INT2_DURATION_A   = 0x37, // DLH, DLM, DLHC

      CLICK_CFG         = 0x38, // D
      CLICK_CFG_A       = 0x38, // DLHC
      CLICK_SRC         = 0x39, // D
      CLICK_SRC_A       = 0x39, // DLHC
      CLICK_THS         = 0x3A, // D
      CLICK_THS_A       = 0x3A, // DLHC
      TIME_LIMIT        = 0x3B, // D
      TIME_LIMIT_A      = 0x3B, // DLHC
      TIME_LATENCY      = 0x3C, // D
      TIME_LATENCY_A    = 0x3C, // DLHC
      TIME_WINDOW       = 0x3D, // D
      TIME_WINDOW_A     = 0x3D, // DLHC

      Act_THS           = 0x3E, // D
      Act_DUR           = 0x3F, // D

      CRA_REG_M         = 0x00, // DLH, DLM, DLHC
      CRB_REG_M         = 0x01, // DLH, DLM, DLHC
      MR_REG_M          = 0x02, // DLH, DLM, DLHC

      SR_REG_M          = 0x09, // DLH, DLM, DLHC
      IRA_REG_M         = 0x0A, // DLH, DLM, DLHC
      IRB_REG_M         = 0x0B, // DLH, DLM, DLHC
      IRC_REG_M         = 0x0C, // DLH, DLM, DLHC

      WHO_AM_I          = 0x0F, // D
      WHO_AM_I_M        = 0x0F, // DLM

      TEMP_OUT_H_M      = 0x31, // DLHC
      TEMP_OUT_L_M      = 0x32, // DLHC


      // dummy addresses for registers in different locations on different devices;
      // the library translates these based on device type
      // value with sign flipped is used as index into translated_regs array

      OUT_X_H_M         = -1,
      OUT_X_L_M         = -2,
      OUT_Y_H_M         = -3,
      OUT_Y_L_M         = -4,
      OUT_Z_H_M         = -5,
      OUT_Z_L_M         = -6,
      // update dummy_reg_count if registers are added here!

      // device-specific register addresses

      DLH_OUT_X_H_M     = 0x03,
      DLH_OUT_X_L_M     = 0x04,
      DLH_OUT_Y_H_M     = 0x05,
      DLH_OUT_Y_L_M     = 0x06,
      DLH_OUT_Z_H_M     = 0x07,
      DLH_OUT_Z_L_M     = 0x08,

      DLM_OUT_X_H_M     = 0x03,
      DLM_OUT_X_L_M     = 0x04,
      DLM_OUT_Z_H_M     = 0x05,
      DLM_OUT_Z_L_M     = 0x06,
      DLM_OUT_Y_H_M     = 0x07,
      DLM_OUT_Y_L_M     = 0x08,

      DLHC_OUT_X_H_M    = 0x03,
      DLHC_OUT_X_L_M    = 0x04,
      DLHC_OUT_Z_H_M    = 0x05,
      DLHC_OUT_Z_L_M    = 0x06,
      DLHC_OUT_Y_H_M    = 0x07,
      DLHC_OUT_Y_L_M    = 0x08,

      D_OUT_X_L_M       = 0x08,
      D_OUT_X_H_M       = 0x09,
      D_OUT_Y_L_M       = 0x0A,
      D_OUT_Y_H_M       = 0x0B,
      D_OUT_Z_L_M       = 0x0C,
      D_OUT_Z_H_M       = 0x0D
    };



#define MAG_ADDR (0x1E << 1)
#define ACC_ADDR (0x19 << 1)

  uint8_t data[10];
  uint8_t txData, rxData;
  HAL_StatusTypeDef status;
  uint8_t accData[6], magData[6];
  uint16_t x_acc, y_acc, z_acc;
  uint16_t x_mag, y_mag, z_mag;

  HAL_GPIO_WritePin(EN_3V3_ALT_GPIO_Port, EN_3V3_ALT_Pin, GPIO_PIN_SET);
  HAL_Delay(100);
//  status = HAL_I2C_Master_Receive(&hi2c1, ACC_ADDR, data, 1, 1000);
  status = HAL_I2C_Mem_Read(&hi2c1, ACC_ADDR,  (enum regAddr) WHO_AM_I, 1,data, 1, 100);

  txData = 0x08; // continous mode, 2g mode, high-resolution mode
//  txData = 0x00; // continous mode, 2g mode, high-resolution mode
  status = HAL_I2C_Mem_Write(&hi2c1, ACC_ADDR, (enum regAddr) CTRL_REG4_A, 1, &txData, 1, 100);

  txData = 0x00; // no filtering
  status = HAL_I2C_Mem_Write(&hi2c1, ACC_ADDR, (enum regAddr) CTRL_REG2_A, 1, &txData, 1, 100);

  txData = 0x27; //enable all channels, no low power mode, HR / Normal / Low-power mode (10 Hz)
  status = HAL_I2C_Mem_Write(&hi2c1, ACC_ADDR, (enum regAddr) CTRL_REG1_A, 1, &txData, 1, 100);

  HAL_Delay(1000);

  status = HAL_I2C_Mem_Read(&hi2c1, ACC_ADDR, (enum regAddr) STATUS_REG_A, 1,&rxData, 1, 100);
  if( (rxData & 0x08) == 0x08){
	  // new data available
	  status = HAL_I2C_Mem_Read(&hi2c1, ACC_ADDR, (enum regAddr) OUT_X_L_A, 1,&accData[0], 1, 100);
	  status = HAL_I2C_Mem_Read(&hi2c1, ACC_ADDR, (enum regAddr) OUT_X_H_A, 1,&accData[1], 1, 100);
	  status = HAL_I2C_Mem_Read(&hi2c1, ACC_ADDR, (enum regAddr) OUT_Y_L_A, 1,&accData[2], 1, 100);
	  status = HAL_I2C_Mem_Read(&hi2c1, ACC_ADDR, (enum regAddr) OUT_Y_H_A, 1,&accData[3], 1, 100);
	  status = HAL_I2C_Mem_Read(&hi2c1, ACC_ADDR, (enum regAddr) OUT_Z_L_A, 1,&accData[4], 1, 100);
	  status = HAL_I2C_Mem_Read(&hi2c1, ACC_ADDR, (enum regAddr) OUT_Z_H_A, 1,&accData[5], 1, 100);

	  HAL_Delay(100);
//	  status = HAL_I2C_Mem_Read(&hi2c1, ACC_ADDR, (enum regAddr) OUT_X_L_A, 1,accData, 6, 100);

	  if(status == HAL_OK){
		  x_acc = (((uint16_t) accData[1]) << 8) | accData[0];
		  y_acc = (((uint16_t) accData[3]) << 8) | accData[2];
		  z_acc = (((uint16_t) accData[5]) << 8) | accData[4];
	  }
  }

  typedef enum {
    LIS2MDL_OFFSET_X_REG_L = 0x45,
    LIS2MDL_OFFSET_X_REG_H = 0x46,
    LIS2MDL_OFFSET_Y_REG_L = 0x47,
    LIS2MDL_OFFSET_Y_REG_H = 0x48,
    LIS2MDL_OFFSET_Z_REG_L = 0x49,
    LIS2MDL_OFFSET_Z_REG_H = 0x4A,
    LIS2MDL_WHO_AM_I = 0x4F,
    LIS2MDL_CFG_REG_A = 0x60,
    LIS2MDL_CFG_REG_B = 0x61,
    LIS2MDL_CFG_REG_C = 0x62,
    LIS2MDL_INT_CRTL_REG = 0x63,
    LIS2MDL_INT_SOURCE_REG = 0x64,
    LIS2MDL_INT_THS_L_REG = 0x65,
    LIS2MDL_STATUS_REG = 0x67,
    LIS2MDL_OUTX_L_REG = 0x68,
    LIS2MDL_OUTX_H_REG = 0x69,
    LIS2MDL_OUTY_L_REG = 0x6A,
    LIS2MDL_OUTY_H_REG = 0x6B,
    LIS2MDL_OUTZ_L_REG = 0x6C,
    LIS2MDL_OUTZ_H_REG = 0x6D,
  } lis2mdl_register_t;

  txData = 0b10000000; // ODR 10Hz, temperature compensation,continous mode
  status = HAL_I2C_Mem_Write(&hi2c1, MAG_ADDR, (lis2mdl_register_t) LIS2MDL_CFG_REG_A, 1, &txData, 1, 100);

  txData = 0b00000001; // digital filter enabled (ODR/4)
  status = HAL_I2C_Mem_Write(&hi2c1, MAG_ADDR, (lis2mdl_register_t) LIS2MDL_CFG_REG_B, 1, &txData, 1, 100);

  txData = 0b00000000;
  status = HAL_I2C_Mem_Write(&hi2c1, MAG_ADDR, (lis2mdl_register_t) LIS2MDL_CFG_REG_C, 1, &txData, 1, 100);

  HAL_Delay(500);

  status = HAL_I2C_Mem_Read(&hi2c1, MAG_ADDR, (lis2mdl_register_t) LIS2MDL_STATUS_REG, 1,&rxData, 1, 100);
   if( (rxData & 0x08) == 0x08){
 	  // new data available
 	  status = HAL_I2C_Mem_Read(&hi2c1, MAG_ADDR, (lis2mdl_register_t) LIS2MDL_OUTX_L_REG, 1,magData, 6, 100);
 	  HAL_Delay(500);
 	  status = HAL_I2C_Mem_Read(&hi2c1, MAG_ADDR, (lis2mdl_register_t) LIS2MDL_OUTX_L_REG, 1,magData, 6, 100);

 	  if(status == HAL_OK){
 		  x_mag = (((uint16_t) magData[1]) << 8) | magData[0];
 		  y_mag = (((uint16_t) magData[3]) << 8) | magData[2];
 		  z_mag = (((uint16_t) magData[5]) << 8) | magData[4];
 	  }
   }

  // shut off accelerometer and magnetometer
  HAL_GPIO_WritePin(EN_3V3_ALT_GPIO_Port, EN_3V3_ALT_Pin, GPIO_PIN_RESET);

//  while(1){};


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
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
//  mainSystemThreadId = osThreadNew(mainSystemTask, NULL, &mainSystemTask_attributes);

  micThreadId = osThreadNew(micTask, &configPacket.payload.config_packet.audio_config, &micTask_attributes);

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
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_9;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV5;
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV25;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS|RCC_PERIPHCLK_RFWAKEUP;
  PeriphClkInitStruct.RFWakeUpClockSelection = RCC_RFWKPCLKSOURCE_LSE;
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  HAL_GPIO_WritePin(GPIOC, SD_CS_Pin|EN_UWB_REG_Pin|EN_SD_REG_2_Pin|EN_MIC_PWR_Pin
                          |EN_BATT_MON_Pin|SD_MUX_SEL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, EN_3V3_ALT_Pin|EN_SD_REG_Pin|ADC_PD_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, EN_BUZZER_PWR_Pin|PD13_Pin|PD12_Pin|EN_SD_MUX_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : P0_15_Pin DW_GP6_Pin P1_00_Pin */
  GPIO_InitStruct.Pin = P0_15_Pin|DW_GP6_Pin|P1_00_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SD_CS_Pin EN_UWB_REG_Pin EN_SD_REG_2_Pin EN_MIC_PWR_Pin
                           EN_BATT_MON_Pin SD_MUX_SEL_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin|EN_UWB_REG_Pin|EN_SD_REG_2_Pin|EN_MIC_PWR_Pin
                          |EN_BATT_MON_Pin|SD_MUX_SEL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : EN_3V3_ALT_Pin EN_SD_REG_Pin ADC_PD_RST_Pin */
  GPIO_InitStruct.Pin = EN_3V3_ALT_Pin|EN_SD_REG_Pin|ADC_PD_RST_Pin;
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

  /*Configure GPIO pins : EN_BUZZER_PWR_Pin PD13_Pin PD12_Pin EN_SD_MUX_Pin */
  GPIO_InitStruct.Pin = EN_BUZZER_PWR_Pin|PD13_Pin|PD12_Pin|EN_SD_MUX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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

  /*Configure GPIO pin : SD_DETECT_2_Pin */
  GPIO_InitStruct.Pin = SD_DETECT_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SD_DETECT_2_GPIO_Port, &GPIO_InitStruct);

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

//			 uint16_t index = 10;
//			  HAL_TIM_Base_Start(&htim16);
//			  HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
//
//			  while(1){
//
//			  htim16.Instance->ARR = index;
//			  htim16.Instance->CCR1 = index >> 1;
//
//			  osDelay(5);
//
//				index+=2;
//				if(index == 500) {
//					/* stop buzzer pwm */
//					HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);
//					osDelay(10);
//					break;
//				}
//			  }


//				osDelay(5000);


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

void micTask(void *argument){
	audio_config_t audio_config;
	memcpy((uint8_t*) &audio_config,(uint8_t*)argument,sizeof(audio_config_t));

	audio_config.bit_resolution=MIC_BIT_RESOLUTION_BIT_RES_16;
	audio_config.channel_1=true;
	audio_config.channel_2=true;
	audio_config.has_audio_compression=true;
	audio_config.audio_compression.compression_factor=0;
	audio_config.audio_compression.compression_type=COMPRESSION_TYPE_OPUS;
	audio_config.audio_compression.enabled=false;
	audio_config.estimated_record_time=12345678; //placeholder
	audio_config.sample_freq=MIC_SAMPLE_FREQ_SAMPLE_RATE_48000;

	/* Turn on microphone and ADC */
	HAL_GPIO_WritePin(EN_MIC_PWR_GPIO_Port, EN_MIC_PWR_Pin, GPIO_PIN_SET);

	/* initialize SD card */
	  HAL_GPIO_WritePin(EN_SD_REG_GPIO_Port, EN_SD_REG_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(EN_SD_REG_2_GPIO_Port, EN_SD_REG_2_Pin, GPIO_PIN_SET);

	  osDelay(1000);
//	  /* start mux */
	  HAL_GPIO_WritePin(EN_SD_MUX_GPIO_Port, EN_SD_MUX_Pin, GPIO_PIN_RESET); // enable mux
	  HAL_GPIO_WritePin(SD_MUX_SEL_GPIO_Port, SD_MUX_SEL_Pin, GPIO_PIN_RESET);// sd card 1 selected
//	  HAL_GPIO_WritePin(SD_MUX_SEL_GPIO_Port, SD_MUX_SEL_Pin, GPIO_PIN_SET);// sd card 2 selected

	  EnableExtADC(true);
	  osDelay(200);
	  RunADC();

	  // SD CS is PC1 for v1 design
	  // SD reg is PC11 for v1 design

	  HAL_Delay(50);

	  char folder_name[20] = "folder";
	//  char file_name[60];;

	  	int folder_number = 0;
	    FILINFO fno;

	  	FRESULT res;

	  	//https://wiki.st.com/stm32mcu/wiki/Introduction_to_FILEX#Migration_from_FatFS_to_FileX
	  	//https://learn.microsoft.com/en-us/azure/rtos/filex/chapter5
	  	/* check if volume exists and can be opened */
	//  	if(FX_PTR_ERROR == fx_media_open(&sd_disk, "exFAT_DISK", fx_stm32_sd_driver, (VOID *)FX_NULL, (VOID *) media_memory, sizeof(media_memory))){

//	    HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);

//	  	uint8_t random_data[4];
//	  	random_data[0] = 0xDE;
//		random_data[1] = 0xAD;
//		random_data[2] = 0xBE;
//		random_data[3] = 0xEF;
//	  	while(1){
//	  	    HAL_SPI_Transmit(&hspi1, random_data, 4, 100);
//	  	}
//	  	while(1){
//	    res = f_mount(&SDFatFs, "", 1);
//	    osDelay(100);
//	  	}

	    res = f_mount(&SDFatFs, "", 1);
	  	if(res != FR_OK){
	  		Error_Handler();
	  	}else{

	//  		while (1) {
	  			// 		    sprintf(folder_name, "random_%d", folder_number);

	//  			status = fx_directory_name_test(
	//  					&sd_disk,
	//  					folder_name);

	  			// 		   HAL_Delay(1000);
	  			// 		    fresult = f_opendir(&fdirectory, folder_name);
	//  			if (status == FX_NOT_FOUND) {
	//  				status = fx_directory_create(&sd_disk, folder_name);
	//  				break;
	//  			}

	  			// 		    f_closedir(&fdirectory);
	//  		}

			sprintf(folder_name, "/audio_%d", folder_number);
			while(1){
				res = f_stat(folder_name,&fno);
				if(res == FR_OK){ //file exists so iterate on number
					folder_number++;
					sprintf(folder_name, "/audio_%d", folder_number);
				}else{
					res = f_mkdir(folder_name);
					if(FR_OK == f_opendir(&dir, folder_name)){
						f_chdir(folder_name);
						break;
					}else{
						Error_Handler();
					}
				}
	//			while(1);
	////			res = f_mkdir(folder_name);
	////			res = f_opendir(&dir, folder_name);
	//			if((res == FR_NO_PATH) || (res == FR_NO_FILE)){
	//				res = f_mkdir(folder_name);
	//				if(FR_OK == f_opendir(&dir, folder_name)){
	//					break;
	//				}else{
	//					Error_Handler();
	//				}
	//
	//				sprintf(folder_name, "/audio_%d", folder_number);
	//				folder_number++;
	//			}
	//			if(res != FR_OK){
	//
	//			}else{
	//				break;
	//			}

			}

	  		/* set to recently created directory */
	//  		status = fx_directory_default_set(
	//  				&sd_disk,
	//  				folder_name);

	//  		if(status != FX_SUCCESS){
	//  			Error_Handler();
	//  		}

			res = f_open(&file, "test.txt", FA_WRITE | FA_CREATE_ALWAYS);
			if(res == FR_OK){
					res = f_write(&file, "Hello, world!", 13, &bytes_written);
			}else Error_Handler();
		    if (res == FR_OK)
		    {
		        // Close the file
		        f_close(&file);

		        // Flush the cached data to the SD card
		        f_sync(&file);
		    }else Error_Handler();

		    WAV_RECORD_TEST();
	//  		char test_string[30] = "test_string!";
	//  		/* Open file for writing (Create) */
	//  		status = fx_file_create(&sd_disk, "STM32_filex.TXT");
	//  		if(status != FX_SUCCESS) Error_Handler();
	//  		status = fx_file_open(&sd_disk, &fx_file, "STM32_filex.TXT", FX_OPEN_FOR_WRITE);
	//  		if(status != FX_SUCCESS) Error_Handler();
	//  		/* Seek to the beginning of the test file.  */
	//  		status =  fx_file_seek(&fx_file, 0);
	//  		if(status != FX_SUCCESS) Error_Handler();
	//  		status = fx_file_write(&fx_file, "1234567890", 10);
	//  		if(status != FX_SUCCESS) Error_Handler();
	//  		status = fx_file_close(&fx_file);
	//  		if(status != FX_SUCCESS) Error_Handler();
	//  		/* flush data */
	//  		status = fx_media_flush(&sd_disk);
	//  		if(status != FX_SUCCESS) Error_Handler();

	//		while(1);
	  	}



	/* exit */
	while(1){
		osDelay(100);
	}
	//turn off microphone and ADC
	HAL_GPIO_WritePin(EN_MIC_PWR_GPIO_Port, EN_MIC_PWR_Pin, GPIO_PIN_RESET);
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


void RunADC(){

	uint8_t data;
	HAL_StatusTypeDef status;

	#define ADAU1979_ADDR				0x11 << 1

	#define ADAU1979_M_POWER			0x00
	#define S_RST						0x01 << 7
	#define PWUP						0x01 << 0
	#define PWDOWN						0x00 << 0

		/* RESET ADAU1979 */
		data = S_RST;
		status = HAL_I2C_Mem_Write(&hi2c3, ADAU1979_ADDR, ADAU1979_M_POWER,
				1, &data, 1, 100);
		osDelay(50);


		/* activate ADC */
		data = PWUP;
		status = HAL_I2C_Mem_Write(&hi2c3, ADAU1979_ADDR, ADAU1979_M_POWER,
				1, &data, 1, 100);

		osDelay(50);

	#define ADAU1979_BLOCK_POWER_SAI	0x04
	#define LR_POL_LOW_HIGH				0x0 << 7
	#define LR_POL_HIGH_LOW				0x1 << 7
	#define BCLKEDGE_FALLING			0x0 << 6
	#define BCLKEDGE_RISING				0x1 << 6
	#define LDO_EN						0x1 << 5
	#define VREF_EN						0x1 << 4
	#define ADC_EN4						0x1 << 3
	#define ADC_EN3						0x1 << 2
	#define ADC_EN2						0x1 << 1
	#define ADC_EN1						0x1 << 0

		  data = LDO_EN | VREF_EN | ADC_EN4 | ADC_EN3 | ADC_EN2 | ADC_EN1;

		// ADC 2 and 4 are disabled
//		data = LDO_EN | VREF_EN | ADC_EN3 | ADC_EN1;
		//    data = LDO_EN | VREF_EN | ADC_EN3 | ADC_EN1;

		status = HAL_I2C_Mem_Write(&hi2c3, ADAU1979_ADDR, ADAU1979_BLOCK_POWER_SAI,
				1, &data, 1, 100);

	#define ADAU1979_SAI_CTRL0			0x05
	#define I2S_FORMAT					0x0 << 6
	#define LEFT_JUSTIFIED				0x1 << 6
	#define STEREO						0x0 << 3
	#define TDM_2						0x1 << 3
	#define TDM_4						0x2 << 3
	#define TDM_8						0x3 << 3
	#define TDM_16						0x4 << 3
	#define SAMPLING_RATE_8_12_KHZ		0x0 << 0
	#define SAMPLING_RATE_16_24_KHZ		0x1 << 0
	#define SAMPLING_RATE_32_48_KHZ		0x2 << 0
	#define SAMPLING_RATE_64_96_KHZ		0x3 << 0
	#define SAMPLING_RATE_128_192_KHZ	0x4 << 0

	//	/* activate ADC */
	//	//  data = I2S_FORMAT | STEREO | SAMPLING_RATE_32_48_KHZ;
	//	data = LEFT_JUSTIFIED | TDM_16 | SAMPLING_RATE_32_48_KHZ;
	//	//  status = HAL_I2C_Mem_Write(&hi2c3, ADAU1979_ADDR, ADAU1979_SAI_CTRL0,
	//	//                                     1, &data, 1, 100);
	//
	//	//  data = LEFT_JUSTIFIED | TDM_8 | SAMPLING_RATE_32_48_KHZ;
	//	status = HAL_I2C_Mem_Write(&hi2c3, ADAU1979_ADDR, ADAU1979_SAI_CTRL0,
	//			1, &data, 1, 100);

		/* ONLY FOR WIND TUNNEL TESTING */
		data = I2S_FORMAT | STEREO | SAMPLING_RATE_32_48_KHZ;
		status = HAL_I2C_Mem_Write(&hi2c3, ADAU1979_ADDR, ADAU1979_SAI_CTRL0,
				1, &data, 1, 100);

	#define ADAU1979_SAI_CTRL1			0x06
	#define SDATAOUT1_OUTPUT			0x0 << 7
	#define SDATAOUT2_OUTPUT			0x1 << 7
	#define SLOT_WIDTH_32				0x0 << 5
	#define SLOT_WIDTH_24				0x1 << 5
	#define SLOT_WIDTH_16				0x2 << 5
	#define DATA_WIDTH_24				0x0 << 4
	#define DATA_WIDTH_16				0x1 << 4
	#define LRCLK_50_DUTY_CYCLE			0x0 << 3
	#define LRCLK_PULSE					0x1 << 3
	#define MSB_FIRST					0x0 << 2
	#define LSB_FIRST					0x1 << 2
	#define BCLKRATE_32_PER_CHANNEL		0x0 << 1
	#define BCLKRATE_16_PER_CHANNEL		0x1 << 1
	#define SAI_SLAVE					0x0 << 0
	#define SAI_MASTER					0x1 << 0

		/* TDM Configuration */
	//	//  data = SDATAOUT1_OUTPUT | SLOT_WIDTH_16 | DATA_WIDTH_16 | LRCLK_50_DUTY_CYCLE | MSB_FIRST | BCLKRATE_16_PER_CHANNEL | SAI_SLAVE;
	//	//  data = SDATAOUT1_OUTPUT | SLOT_WIDTH_16 | DATA_WIDTH_16 | LRCLK_PULSE | MSB_FIRST | BCLKRATE_16_PER_CHANNEL | SAI_SLAVE;
	//	data = SDATAOUT2_OUTPUT | SLOT_WIDTH_16 | DATA_WIDTH_16 | LRCLK_PULSE | MSB_FIRST | BCLKRATE_16_PER_CHANNEL | SAI_SLAVE;
	//
	//	status = HAL_I2C_Mem_Write(&hi2c3, ADAU1979_ADDR, ADAU1979_SAI_CTRL1,
	//			1, &data, 1, 100);

		/* ONLY FOR WIND TUNNEL TESTING */
		data = SDATAOUT1_OUTPUT | SLOT_WIDTH_16 | DATA_WIDTH_16 | LRCLK_PULSE | MSB_FIRST | BCLKRATE_16_PER_CHANNEL | SAI_SLAVE;
		status = HAL_I2C_Mem_Write(&hi2c3, ADAU1979_ADDR, ADAU1979_SAI_CTRL1,
				1, &data, 1, 100);

	#define ADAU1979_SAI_CMAP12			0x07
	#define TDM_CH2_SLOT_10				0x9 << 4
	#define TDM_CH2_SLOT_3				0x3 << 4
	#define TDM_CH2_SLOT_1				0x1 << 4
	#define TDM_CH1_SLOT_15				0xE << 0
	#define TDM_CH1_SLOT_9				0x8 << 0
	#define TDM_CH1_SLOT_0				0x0 << 0
	#define TDM_CH1_SLOT_1				0x1

		/* TDM Config Slots */
	//	//  data = TDM_CH1_SLOT_0 | TDM_CH2_SLOT_3;
	//	data = TDM_CH1_SLOT_9;
	//	//    data = TDM_CH1_SLOT_9 | TDM_CH2_SLOT_10;
	//
	//	//  data = TDM_CH1_SLOT_0 | TDM_CH2_SLOT_10;
	//	status = HAL_I2C_Mem_Write(&hi2c3, ADAU1979_ADDR, ADAU1979_SAI_CMAP12,
	//			1, &data, 1, 100);

		/* ONLY FOR WIND TUNNEL TESTING */
		data = TDM_CH1_SLOT_0 | TDM_CH2_SLOT_1;
		status = HAL_I2C_Mem_Write(&hi2c3, ADAU1979_ADDR, ADAU1979_SAI_CMAP12,
				1, &data, 1, 100);

	#define ADAU1979_SAI_CMAP34			0x08
	#define TDM_CH4_SLOT_12				0xB << 4
	#define TDM_CH4_SLOT_2				0x2 << 4
	#define TDM_CH4_SLOT_1				0x1 << 4
	#define TDM_CH3_SLOT_16				0xF << 0
	#define TDM_CH3_SLOT_11				0xA << 0
	#define TDM_CH3_SLOT_10				0x9 << 0
	#define TDM_CH3_SLOT_2				0x2 << 0
	#define TDM_CH3_SLOT_1				0x1 << 0

		/* TDM Config Slots */
	//	//  data = TDM_CH4_SLOT_1 | TDM_CH3_SLOT_2;
	//	//  data = 0x7 | TDM_CH4_SLOT_12;
	//	data = TDM_CH3_SLOT_10;
	//	//  data = TDM_CH3_SLOT_1;
	//	//    data = TDM_CH3_SLOT_11 | TDM_CH4_SLOT_12;
	//
	//	status = HAL_I2C_Mem_Write(&hi2c3, ADAU1979_ADDR, ADAU1979_SAI_CMAP34,
	//			1, &data, 1, 100);

		/* ONLY FOR WIND TUNNEL TESTING */
//		data = TDM_CH3_SLOT_1;
//		status = HAL_I2C_Mem_Write(&hi2c3, ADAU1979_ADDR, ADAU1979_SAI_CMAP34,
//				1, &data, 1, 100);

	#define ADAU1979_SAI_OVERTEMP		0x09
	#define CH4_EN_OUT					0x1 << 7
	#define CH3_EN_OUT					0x1 << 6
	#define CH2_EN_OUT					0x1 << 5
	#define CH1_EN_OUT					0x1 << 4
	#define DRV_HIZ_EN					0x1 << 3

		/* TDM Channel Configuration */
		  data = DRV_HIZ_EN | CH4_EN_OUT | CH3_EN_OUT | CH2_EN_OUT | CH1_EN_OUT;

		//disable channels 2 and 4
//		data = DRV_HIZ_EN | CH3_EN_OUT | CH1_EN_OUT;
		status = HAL_I2C_Mem_Write(&hi2c3, ADAU1979_ADDR, ADAU1979_SAI_OVERTEMP,
				1, &data, 1, 100);

	#define ADAU1979_POSTADC_GAIN1		0x0A
	#define ADAU1979_POSTADC_GAIN2		0x0B
	#define ADAU1979_POSTADC_GAIN3		0x0C
	#define ADAU1979_POSTADC_GAIN4		0x0D
	#define GAIN_0_DB					0xA0
	#define GAIN_5_625_DB				145 // (60 - x * 0.375) dB
	#define GAIN_9_DB					136
	#define GAIN_15_DB					120
	#define GAIN_18_DB					112
	#define GAIN_25_5_DB				92
	#define GAIN_25_875_DB				91
	#define GAIN_60_DB					0x0

		/* Gain = 60dB - (gain_register) * 0.375dB */

		/* Channel 1 Gain */
	//	data = GAIN_18_DB;
//		data = GAIN_5_625_DB;
//		data = GAIN_25_5_DB;
		data = GAIN_9_DB;
		status = HAL_I2C_Mem_Write(&hi2c3, ADAU1979_ADDR, ADAU1979_POSTADC_GAIN1,
				1, &data, 1, 100);

		/* Channel 2 Gain */
	//	data = GAIN_18_DB;
//		data = GAIN_5_625_DB;
//		data = GAIN_25_5_DB;
		data = GAIN_9_DB;
		status = HAL_I2C_Mem_Write(&hi2c3, ADAU1979_ADDR, ADAU1979_POSTADC_GAIN2,
				1, &data, 1, 100);

		/* Channel 3 Gain */
	//	data = GAIN_18_DB; // gain = 18dB
//		data = GAIN_25_5_DB;
//		data = GAIN_5_625_DB;
		data = GAIN_9_DB;
		status = HAL_I2C_Mem_Write(&hi2c3, ADAU1979_ADDR, ADAU1979_POSTADC_GAIN3,
				1, &data, 1, 100);

		/* Channel 4 Gain */
	//	data = GAIN_18_DB; // gain = 18dB
//		data = GAIN_25_5_DB;
//		data = GAIN_5_625_DB;
		data = GAIN_9_DB;
		status = HAL_I2C_Mem_Write(&hi2c3, ADAU1979_ADDR, ADAU1979_POSTADC_GAIN4,
				1, &data, 1, 100);

	#define ADAU1979_MISC_CONTROL		0x0E
	#define MODE_4_CHANNEL				0x0 << 6
	#define MODE_2_CHANNEL_SUM_MODE		0x1 << 6
	#define MODE_1_CHANNEL_SUM_MODE		0x2 << 6

		/* 4-channel mode, normal operation, */
//		data = MODE_4_CHANNEL;
//		data = MODE_4_CHANNEL | (0x1 <<1); // the 2nd set bit is some reserved spot found on the datasheet
		data = MODE_2_CHANNEL_SUM_MODE;
		status = HAL_I2C_Mem_Write(&hi2c3, ADAU1979_ADDR, ADAU1979_MISC_CONTROL,
				1, &data, 1, 100);

	#define ADAU1979_ASDC_CLIP			0x19
	#define ADAU1979_DC_HPF_CAL			0x1A
	#define DC_HPF_C4_ON				0x1 << 3
	#define DC_HPF_C3_ON				0x1 << 2
	#define DC_HPF_C2_ON				0x1 << 1
	#define DC_HPF_C1_ON				0x1 << 0

		//	/* HPF on for all channels */
		//	data = DC_HPF_C4_ON | DC_HPF_C3_ON | DC_HPF_C2_ON | DC_HPF_C1_ON;
		//	status = HAL_I2C_Mem_Write(&hi2c3, ADAU1979_ADDR, ADAU1979_DC_HPF_CAL,
		//									 1, &data, 1, 100);

	#define ADAU1979_PLL_CONTROL		0x01
	#define PLL_LOCK_REG				0x1 << 7
	#define PLL_NO_AUTO_MUTE			0x0 << 6
	#define PLL_INPUT_MCLK				0x0 << 4
	#define PLL_INPUT_LRCLK				0x1 << 4
	#define PLL_MCS_DIV_256				0x1 << 0
	#define PLL_MCS_DIV_384				0x2 << 0
	#define PLL_MCS_DIV_512				0x3 << 0
	#define PLL_MCS_DIV_768				0x4 << 0
	#define PLL_MCS_DIV_128				0x0 << 0

		//  /* PLL Configuration (MCLK = BCLK = 256 * 22.05kHz, ADC SAMPLE RATE = 22.05 kHz)*/
		/* PLL Configuration (MCLK = BCLK = 128 * 44.6kHz, ADC SAMPLE RATE = 44.6 kHz)*/
		//  data = PLL_NO_AUTO_MUTE | PLL_INPUT_MCLK | PLL_MCS_DIV_256;

		data = PLL_NO_AUTO_MUTE | PLL_INPUT_LRCLK;
		//
		status = HAL_I2C_Mem_Write(&hi2c3, ADAU1979_ADDR, ADAU1979_PLL_CONTROL,
				1, &data, 1, 100);

}

void EnableExtADC(bool state){
	if(state){
		HAL_GPIO_WritePin(ADC_PD_RST_GPIO_Port, ADC_PD_RST_Pin, GPIO_PIN_SET);
//		HAL_GPIO_WritePin(EN_MIC_PWR_GPIO_Port, EN_MIC_PWR_Pin, GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(ADC_PD_RST_GPIO_Port, ADC_PD_RST_Pin, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(EN_MIC_PWR_GPIO_Port, EN_MIC_PWR_Pin, GPIO_PIN_RESET);
//
	}
}

#define MAX_BYTES_PER_WAV_FILE 2000000000
//#define MAX_BYTES_PER_WAV_FILE 10000000

void WAV_RECORD_TEST(void){


	uint64_t totalBytesWritten = 0;
	HAL_StatusTypeDef hal_status;


	char file_name[20] = "wav_";
	uint32_t file_index = 0;

	sprintf(file_name, "wav_%u.wav", file_index);

	/* Create a new file */
//	if(FX_SUCCESS != fx_file_create(&sd_disk, file_name)){
//		Error_Handler();
//	}

//	if(FX_SUCCESS == fx_file_open(&sd_disk, &WavFile, file_name, FX_OPEN_FOR_WRITE))
//			{
			  if(f_open(&WavFile, file_name, FA_CREATE_ALWAYS | FA_WRITE) == FR_OK)
			  {
//		status =  fx_file_seek(&WavFile, 0);
		f_lseek(&WavFile,0);
//		if(status != FX_SUCCESS) Error_Handler();

		/* Initialize header file */
		WavProcess_EncInit(hsai_BlockA1.Init.AudioFrequency, pHeaderBuff);

		/* Write header file */
//		if(FX_SUCCESS ==  fx_file_write(&WavFile, pHeaderBuff, 44))
		if(f_write(&WavFile, pHeaderBuff, 44, (void*)&byteswritten) == FR_OK)
		{
			totalBytesWritten += 44;

			////	         uint32_t testCntr = 0;

			//        	 HAL_SAI_Receive_DMA(&hsai_BlockA1, (uint8_t*) audioSample, AUDIO_BUFFER_LEN);
			//        	  HAL_I2S_Receive_DMA(&hi2s1, audioSample, AUDIO_BUFFER_LEN);

			HAL_SAI_Receive(&hsai_BlockA1, (uint8_t*) audioSample, AUDIO_BUFFER_LEN, 2000);

			//        	  uint8_t data;
			//        	  do{
			//        	  status = HAL_I2C_Mem_Read(&hi2c2, ADAU1979_ADDR, ADAU1979_PLL_CONTROL,
			//        	                                       1, &data, 1, 100);
			//        	  HAL_Delay(100);
			//        	  }while( (data & PLL_LOCK_REG) != PLL_LOCK_REG);

			//        	  while(1);

			//	    	hal_status = HAL_SAI_Receive_DMA(&hsai_BlockA1, (uint8_t*) audioSample, AUDIO_BUFFER_LEN);
			//	    	hal_status = HAL_SAI_Receive_IT(&hsai_BlockA1, (uint8_t*) audioSample, AUDIO_BUFFER_LEN);
			//	    	HAL_Delay(1000);
			hal_status = HAL_SAI_Receive_DMA(&hsai_BlockA1, (uint8_t*) audioSample, AUDIO_BUFFER_LEN);

			//			 while(1);

			// 24000 samples @ 10 channels of audio = 2400 samples of audio
			//   this effectively means one buffer can store 75 ms of audio at 32kHz sampling freq

			/* note: ref hsai_BlockA1.Init.AudioFrequency for exact audio frequency */

			// run forever until power is shut off
			while(1){
				while(sampleCntr < ((100 * 2)/4)){
					//	        	 HAL_SAI_Receive(&hsai_BlockA1,  (uint8_t*) audioSample, AUDIO_BUFFER_LEN * 2, 1000);


					//	        	 f_write(&WavFile, testVar, 2048*4, (void*)&byteswritten);
					//	        	 testCntr++;
					//
					//	        	 if(testCntr>20){
					//	        		 break;
					//	        	 }

					//	 	    	HAL_SAI_Receive(&hsai_BlockA1, (uint8_t*) audioSample, AUDIO_BUFFER_LEN, 2000);
					//	 	    	sampleCntr++;
					//	 	    	if(FX_SUCCESS != fx_file_write(&WavFile, audioSample, 2*AUDIO_BUFFER_HALF_LEN * 2)){
					//	 	    		        			 Error_Handler();
					//	 	    		        		 }
					//     	    	totalBytesWritten += AUDIO_BUFFER_HALF_LEN * 2 * 2;

					// Wait for a notification
					osThreadFlagsWait(0x0001U, osFlagsWaitAny, osWaitForever);

					if(SAI_HALF_CALLBACK){
						SAI_HALF_CALLBACK = 0;

//						if(FX_SUCCESS != fx_file_write(&WavFile, audioSample, AUDIO_BUFFER_HALF_LEN * 2)){
//							Error_Handler();
//						}
						f_write(&WavFile, audioSample, AUDIO_BUFFER_HALF_LEN * 2, (void*)&byteswritten);
						totalBytesWritten += AUDIO_BUFFER_HALF_LEN * 2;

					}
					if(SAI_FULL_CALLBACK){
						SAI_FULL_CALLBACK = 0;
//
//						if(FX_SUCCESS != fx_file_write(&WavFile, &audioSample[AUDIO_BUFFER_HALF_LEN], AUDIO_BUFFER_HALF_LEN * 2)){
//							Error_Handler();
//						}
						f_write(&WavFile, &audioSample[AUDIO_BUFFER_HALF_LEN], AUDIO_BUFFER_HALF_LEN * 2, (void*)&byteswritten);
						totalBytesWritten += AUDIO_BUFFER_HALF_LEN * 2;

					}


				}


				sampleCntr = 0;
				WavUpdateHeaderSize(totalBytesWritten);

				if(totalBytesWritten > MAX_BYTES_PER_WAV_FILE){

			        // Close the file
			        f_close(&WavFile);

			        // Flush the cached data to the SD card
			        f_sync(&WavFile);

//					fx_file_close(&WavFile);
					totalBytesWritten = 0;
					WavProcess_EncInit(hsai_BlockA1.Init.AudioFrequency, pHeaderBuff);
					file_index++;
					sprintf(file_name, "wav_%u.wav", file_index);
					if(f_open(&WavFile, file_name, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK){
						Error_Handler();
					}
//					if(FX_SUCCESS != fx_file_create(&sd_disk, file_name)){
//						Error_Handler();
//					}
//					if(FX_SUCCESS != fx_file_open(&sd_disk, &WavFile, file_name, FX_OPEN_FOR_WRITE)){
//						Error_Handler();
//					}
//					fx_file_seek(&WavFile, 0);
					f_lseek(&WavFile,0);

					if(f_write(&WavFile, pHeaderBuff, 44, (void*)&byteswritten) != FR_OK){
//					if(FX_SUCCESS !=  fx_file_write(&WavFile, pHeaderBuff, 44)){
						Error_Handler();
					}
					totalBytesWritten += 44;

				}

			}
			HAL_SAI_DMAStop(&hsai_BlockA1);
			//	         HAL_I2S_DMAStop(&hi2s1);

//			if(FX_SUCCESS == fx_file_seek(&WavFile, 0))
//			{
			 if(f_lseek(&WavFile, 0) == FR_OK)
			 {
				/* Update the wav file header save it into wav file */
				WavProcess_HeaderUpdate(pHeaderBuff, totalBytesWritten);

//				if(FX_SUCCESS != fx_file_write(&WavFile, pHeaderBuff, 44))
//				{
//					Error_Handler();
//				}
				   if(f_write(&WavFile, pHeaderBuff, 44, (void*)&byteswritten) == FR_OK)
				   {

				   }
			}
			//			status = HAL_SAI_Receive(&hsai_BlockA1, (uint8_t*) audioSample, AUDIO_BUFFER_LEN * 2, 4000);
			//			status = HAL_SAI_Receive(&hsai_BlockA1, (uint8_t*) audioSample, AUDIO_BUFFER_LEN * 2, 4000);
			//        	f_write(&WavFile, audioSample, AUDIO_BUFFER_LEN * 2, (void*)&byteswritten);
			//			status = HAL_SAI_Receive(&hsai_BlockA1, (uint8_t*) audioSample, AUDIO_BUFFER_LEN * 2, 4000);
			//        	f_write(&WavFile, audioSample, AUDIO_BUFFER_LEN * 2, (void*)&byteswritten);
			//			status = HAL_SAI_Receive(&hsai_BlockA1, (uint8_t*) audioSample, AUDIO_BUFFER_LEN * 2, 4000);
			//        	f_write(&WavFile, audioSample, AUDIO_BUFFER_LEN * 2, (void*)&byteswritten);
			//			status = HAL_SAI_Receive(&hsai_BlockA1, (uint8_t*) audioSample, AUDIO_BUFFER_LEN * 2, 4000);
			//        	f_write(&WavFile, audioSample, AUDIO_BUFFER_LEN * 2, (void*)&byteswritten);
			//			status = HAL_SAI_Receive(&hsai_BlockA1, (uint8_t*) audioSample, AUDIO_BUFFER_LEN * 2, 4000);
			//        	f_write(&WavFile, audioSample, AUDIO_BUFFER_LEN * 2, (void*)&byteswritten);

//			fx_file_close(&WavFile);

	        // Close the file
	        f_close(&WavFile);

	        // Flush the cached data to the SD card
	        f_sync(&WavFile);

			//			f_close(&WavFile);

			/* flush data */
//			status = fx_media_flush(&sd_disk);
//			if(status != FX_SUCCESS) Error_Handler();

			//			  HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, GPIO_PIN_RESET);
			//	         LED_Cycle(1000);
			return;

			//      }
		}else Error_Handler();
			}else{
				Error_Handler();
			}
}

uint64_t current_offset;
static void WavUpdateHeaderSize(uint64_t totalBytesWritten){
//	current_offset = WavFile.fx_file_current_file_offset;
	current_offset = WavFile.fptr;
	if(f_lseek(&WavFile,0) == FR_OK)
	{
//	if(FX_SUCCESS == fx_file_seek(&WavFile, 0))
//	{
		/* Update the wav file header save it into wav file */
		WavProcess_HeaderUpdate(pHeaderBuff, totalBytesWritten);

		if((f_write(&WavFile, pHeaderBuff, 44, (void*)&byteswritten)) != FR_OK){
//		if(FX_SUCCESS != fx_file_write(&WavFile, pHeaderBuff, 44))
//		{
			Error_Handler();
		}
	}else{
		Error_Handler();
	}

	/* flush data */
	f_sync(&WavFile);
//	status = fx_media_flush(&sd_disk);
//	if(status != FX_SUCCESS) Error_Handler();

	if(f_lseek(&WavFile,current_offset) != FR_OK)
	{
		Error_Handler();
	}
//	if(FX_SUCCESS != fx_file_seek(&WavFile, current_offset)){
//		Error_Handler();
//	}
}

/*******************************************************************************
                            Static Functions
 *******************************************************************************/

/**
 * @brief  Encoder initialization.
 * @param  Freq: Sampling frequency.
 * @param  pHeader: Pointer to the WAV file header to be written.
 * @retval 0 if success, !0 else.
 */
static uint32_t WavProcess_EncInit(uint32_t Freq, uint8_t *pHeader)
{
	/* Initialize the encoder structure */
	WaveFormat.SampleRate = Freq;        /* Audio sampling frequency */
	WaveFormat.NbrChannels = 2;          /* Number of channels: 1:Mono or 2:Stereo */
	WaveFormat.BitPerSample = 16;        /* Number of bits per sample (16, 24 or 32) */
	WaveFormat.FileSize = 0x001D4C00;    /* Total length of useful audio data (payload) */
	WaveFormat.SubChunk1Size = 44;       /* The file header chunk size */
	WaveFormat.ByteRate = (WaveFormat.SampleRate * \
			(WaveFormat.BitPerSample/8) * \
			WaveFormat.NbrChannels);     /* Number of bytes per second  (sample rate * block align)  */
	WaveFormat.BlockAlign = WaveFormat.NbrChannels * \
			(WaveFormat.BitPerSample/8); /* channels * bits/sample / 8 */

	/* Parse the wav file header and extract required information */
	if(WavProcess_HeaderInit(pHeader, &WaveFormat))
	{
		return 1;
	}
	return 0;
}

/**
 * @brief  Initialize the wave header file
 * @param  pHeader: Header Buffer to be filled
 * @param  pWaveFormatStruct: Pointer to the wave structure to be filled.
 * @retval 0 if passed, !0 if failed.
 */
static uint32_t WavProcess_HeaderInit(uint8_t* pHeader, WAVE_FormatTypeDef* pWaveFormatStruct)
{
	/* Write chunkID, must be 'RIFF'  ------------------------------------------*/
	pHeader[0] = 'R';
	pHeader[1] = 'I';
	pHeader[2] = 'F';
	pHeader[3] = 'F';

	/* Write the file length ---------------------------------------------------*/
	/* The sampling time: this value will be written back at the end of the
     recording operation.  application: 661500 Btyes = 0x000A17FC, byte[7]=0x00, byte[4]=0xFC */
	pHeader[4] = 0x00;
	pHeader[5] = 0x4C;
	pHeader[6] = 0x1D;
	pHeader[7] = 0x00;
	/* Write the file format, must be 'WAVE' -----------------------------------*/
	pHeader[8]  = 'W';
	pHeader[9]  = 'A';
	pHeader[10] = 'V';
	pHeader[11] = 'E';

	/* Write the format chunk, must be'fmt ' -----------------------------------*/
	pHeader[12]  = 'f';
	pHeader[13]  = 'm';
	pHeader[14]  = 't';
	pHeader[15]  = ' ';

	/* Write the length of the 'fmt' data, must be 0x10 ------------------------*/
	pHeader[16]  = 0x10;
	pHeader[17]  = 0x00;
	pHeader[18]  = 0x00;
	pHeader[19]  = 0x00;

	/* Write the audio format, must be 0x01 (PCM) ------------------------------*/
	pHeader[20]  = 0x01;
	pHeader[21]  = 0x00;

	/* Write the number of channels, ie. 0x01 (Mono) ---------------------------*/
	pHeader[22]  = pWaveFormatStruct->NbrChannels;
	pHeader[23]  = 0x00;

	/* Write the Sample Rate in Hz ---------------------------------------------*/
	/* Write Little Endian ie. 8000 = 0x00001F40 => byte[24]=0x40, byte[27]=0x00*/
	pHeader[24]  = (uint8_t)((pWaveFormatStruct->SampleRate & 0xFF));
	pHeader[25]  = (uint8_t)((pWaveFormatStruct->SampleRate >> 8) & 0xFF);
	pHeader[26]  = (uint8_t)((pWaveFormatStruct->SampleRate >> 16) & 0xFF);
	pHeader[27]  = (uint8_t)((pWaveFormatStruct->SampleRate >> 24) & 0xFF);

	/* Write the Byte Rate -----------------------------------------------------*/
	pHeader[28]  = (uint8_t)((pWaveFormatStruct->ByteRate & 0xFF));
	pHeader[29]  = (uint8_t)((pWaveFormatStruct->ByteRate >> 8) & 0xFF);
	pHeader[30]  = (uint8_t)((pWaveFormatStruct->ByteRate >> 16) & 0xFF);
	pHeader[31]  = (uint8_t)((pWaveFormatStruct->ByteRate >> 24) & 0xFF);

	/* Write the block alignment -----------------------------------------------*/
	pHeader[32]  = pWaveFormatStruct->BlockAlign;
	pHeader[33]  = 0x00;

	/* Write the number of bits per sample -------------------------------------*/
	pHeader[34]  = pWaveFormatStruct->BitPerSample;
	pHeader[35]  = 0x00;

	/* Write the Data chunk, must be 'data' ------------------------------------*/
	pHeader[36]  = 'd';
	pHeader[37]  = 'a';
	pHeader[38]  = 't';
	pHeader[39]  = 'a';

	/* Write the number of sample data -----------------------------------------*/
	/* This variable will be written back at the end of the recording operation */
	pHeader[40]  = 0x00;
	pHeader[41]  = 0x4C;
	pHeader[42]  = 0x1D;
	pHeader[43]  = 0x00;

	/* Return 0 if all operations are OK */
	return 0;
}

/**
 * @brief  Initialize the wave header file
 * @param  pHeader: Header Buffer to be filled
 * @param  pWaveFormatStruct: Pointer to the wave structure to be filled.
 * @retval 0 if passed, !0 if failed.
 */
static uint32_t WavProcess_HeaderUpdate(uint8_t* pHeader, uint32_t bytesWritten)
{
	/* Write the file length ---------------------------------------------------*/
	/* The sampling time: this value will be written back at the end of the
     recording operation.  application: 661500 Btyes = 0x000A17FC, byte[7]=0x00, byte[4]=0xFC */
	pHeader[4] = (uint8_t)(bytesWritten);
	pHeader[5] = (uint8_t)(bytesWritten >> 8);
	pHeader[6] = (uint8_t)(bytesWritten >> 16);
	pHeader[7] = (uint8_t)(bytesWritten >> 24);
	/* Write the number of sample data -----------------------------------------*/
	/* This variable will be written back at the end of the recording operation */
	bytesWritten -=44;
	pHeader[40] = (uint8_t)(bytesWritten);
	pHeader[41] = (uint8_t)(bytesWritten >> 8);
	pHeader[42] = (uint8_t)(bytesWritten >> 16);
	pHeader[43] = (uint8_t)(bytesWritten >> 24);

	/* Return 0 if all operations are OK */
	return 0;
}

//volatile uint32_t byteswritten = 0;
void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai){
	//	 f_write(&WavFile, audioSample, AUDIO_BUFFER_HALF_LEN, (void*)&byteswritten);
	SAI_HALF_CALLBACK = 1;

    // Trigger the notification for the task
	osThreadFlagsSet(micThreadId, 0x0001U);
}

void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai){
	//	 f_write(&WavFile, &audioSample[AUDIO_BUFFER_HALF_LEN], AUDIO_BUFFER_HALF_LEN, (void*)&byteswritten);
	sampleCntr++;
	SAI_FULL_CALLBACK = 1;

    // Trigger the notification for the task
	osThreadFlagsSet(micThreadId, 0x0001U);

}
/* USER CODE END 4 */

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
