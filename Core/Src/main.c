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
#include "app_fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_INTENSITY 1000

#define AUDIO_BUFFER_LEN		(24000)
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

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim16;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */
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

uint16_t redVal = 0;
uint16_t greenVal = 0;
uint16_t blueVal = 0;

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
static void MX_RF_Init(void);
/* USER CODE BEGIN PFP */
void run_ADC();
void Power_Enable_ADAU1979(bool state);
void setLED_Green(uint32_t intensity);
void setLED_Blue(uint32_t intensity);
void setLED_Red(uint32_t intensity);
void disableLEDs();

void WAV_RECORD_TEST(void);
static uint32_t WavProcess_HeaderInit(uint8_t* pHeader, WAVE_FormatTypeDef* pWaveFormatStruct);
static uint32_t WavProcess_EncInit(uint32_t Freq, uint8_t *pHeader);
static uint32_t WavProcess_HeaderUpdate(uint8_t* pHeader, uint32_t bytesWritten);
static void WavUpdateHeaderSize(uint64_t totalBytesWritten);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  /* start buzzer pwm */
//  uint16_t index = 10;
//  HAL_TIM_Base_Start(&htim16);
//  HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);

//  while(1){
//
////  htim16.Instance->CCR1 = index;
//
//  htim16.Instance->ARR = index;
//  htim16.Instance->CCR1 = index >> 1;
//
//  HAL_Delay(50);
//
//  /* stop buzzer pwm */
////  HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);
////  HAL_Delay(100);
//
//	index+=2;
//	if(index == 500) index = 10;
//  }

//while(1){
//	  HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_SET);
//	  HAL_Delay(500);
//	  HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);
//	  HAL_Delay(500);
//
//}

	HAL_TIM_Base_Start(&htim2);

//for(int i = 0; i<500; i+=10){
//	  setLED_Green(i);
//	  HAL_Delay(100);
//}


//  setLED_Green(500);
//  HAL_Delay(500);
//  setLED_Blue(500);
//  HAL_Delay(500);
//  setLED_Red(500);
//  HAL_Delay(500);
//  setLED_Green(0);
//  HAL_Delay(500);
//  setLED_Blue(0);
//  HAL_Delay(500);
//  setLED_Red(0);
//
//  while(1);

  /* turn on buzzer regulator */
  HAL_GPIO_WritePin(EN_BUZZER_PWR_GPIO_Port, EN_BUZZER_PWR_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */

  HAL_GPIO_WritePin(EN_3V3_ALT_GPIO_Port, EN_3V3_ALT_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(EN_UWB_REG_GPIO_Port, EN_UWB_REG_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(EN_SD_REG_GPIO_Port, EN_SD_REG_Pin, GPIO_PIN_SET);

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

//    HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
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


//  while(1){
//	  HAL_GPIO_WritePin(BUZZER_PWM_GPIO_Port, BUZZER_PWM_Pin, GPIO_PIN_SET);
//	  HAL_Delay(10);
//	  HAL_GPIO_WritePin(BUZZER_PWM_GPIO_Port, BUZZER_PWM_Pin, GPIO_PIN_RESET);
//	  HAL_Delay(10);
//  }

  Power_Enable_ADAU1979(true);
  HAL_Delay(200);
  run_ADC();

//#define AUDIO_BUFFER_LEN 1000
//  static uint8_t audioSample[4000];

	WAV_RECORD_TEST();

//  while(1){
//	  HAL_SAI_Receive(&hsai_BlockA1, (uint8_t*) audioSample, AUDIO_BUFFER_LEN << 2, 2000);
//  }

  /* USER CODE END 2 */

  /* Init code for STM32_WPAN */
  MX_APPE_Init();

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    MX_APPE_Process();

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
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

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
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
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
  PeriphClkInitStruct.RFWakeUpClockSelection = RCC_RFWKPCLKSOURCE_LSE;
  PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSI;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE1;

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
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
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
//  HAL_GPIO_WritePin(BUZZER_PWM_GPIO_Port, BUZZER_PWM_Pin, GPIO_PIN_RESET);
//  GPIO_InitStruct.Pin = BUZZER_PWM_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_PULLUP;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(BUZZER_PWM_GPIO_Port, &GPIO_InitStruct);
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void run_ADC(){

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
		HAL_Delay(50);

		/* activate ADC */
		data = PWUP;
		status = HAL_I2C_Mem_Write(&hi2c3, ADAU1979_ADDR, ADAU1979_M_POWER,
				1, &data, 1, 100);

		HAL_Delay(50);

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

		//  data = LDO_EN | VREF_EN | ADC_EN4 | ADC_EN3 | ADC_EN2 | ADC_EN1;

		// ADC 2 and 4 are disabled
		data = LDO_EN | VREF_EN | ADC_EN3 | ADC_EN1;
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
		data = TDM_CH1_SLOT_0;
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
		data = TDM_CH3_SLOT_1;
		status = HAL_I2C_Mem_Write(&hi2c3, ADAU1979_ADDR, ADAU1979_SAI_CMAP34,
				1, &data, 1, 100);

	#define ADAU1979_SAI_OVERTEMP		0x09
	#define CH4_EN_OUT					0x1 << 7
	#define CH3_EN_OUT					0x1 << 6
	#define CH2_EN_OUT					0x1 << 5
	#define CH1_EN_OUT					0x1 << 4
	#define DRV_HIZ_EN					0x1 << 3

		/* TDM Channel Configuration */
		//  data = DRV_HIZ_EN | CH4_EN_OUT | CH3_EN_OUT | CH2_EN_OUT | CH1_EN_OUT;

		//disable channels 2 and 4
		data = DRV_HIZ_EN | CH3_EN_OUT | CH1_EN_OUT;
		status = HAL_I2C_Mem_Write(&hi2c3, ADAU1979_ADDR, ADAU1979_SAI_OVERTEMP,
				1, &data, 1, 100);

	#define ADAU1979_POSTADC_GAIN1		0x0A
	#define ADAU1979_POSTADC_GAIN2		0x0B
	#define ADAU1979_POSTADC_GAIN3		0x0C
	#define ADAU1979_POSTADC_GAIN4		0x0D
	#define GAIN_0_DB					0xA0
	#define GAIN_9_DB					136
	#define GAIN_15_DB					120
	#define GAIN_18_DB					112
	#define GAIN_25_5_DB				92
	#define GAIN_25_875_DB				91
	#define GAIN_60_DB					0x0

		/* Gain = 60dB - (gain_register) * 0.375dB */

		/* Channel 1 Gain */
	//	data = GAIN_18_DB;
		data = GAIN_25_5_DB;
		status = HAL_I2C_Mem_Write(&hi2c3, ADAU1979_ADDR, ADAU1979_POSTADC_GAIN1,
				1, &data, 1, 100);

		/* Channel 2 Gain */
	//	data = GAIN_18_DB;
		data = GAIN_25_5_DB;
		status = HAL_I2C_Mem_Write(&hi2c3, ADAU1979_ADDR, ADAU1979_POSTADC_GAIN2,
				1, &data, 1, 100);

		/* Channel 3 Gain */
	//	data = GAIN_18_DB; // gain = 18dB
		data = GAIN_25_5_DB;
		status = HAL_I2C_Mem_Write(&hi2c3, ADAU1979_ADDR, ADAU1979_POSTADC_GAIN3,
				1, &data, 1, 100);

		/* Channel 4 Gain */
	//	data = GAIN_18_DB; // gain = 18dB
		data = GAIN_25_5_DB;
		status = HAL_I2C_Mem_Write(&hi2c3, ADAU1979_ADDR, ADAU1979_POSTADC_GAIN4,
				1, &data, 1, 100);

	#define ADAU1979_MISC_CONTROL		0x0E
	#define MODE_4_CHANNEL				0x0 << 6
	#define MODE_2_CHANNEL_SUM_MODE		0x1 << 6
	#define MODE_1_CHANNEL_SUM_MODE		0x2 << 6

		/* 4-channel mode, normal operation, */
		data = MODE_4_CHANNEL | (0x1 <<1); // the 2nd set bit is some reserved spot found on the datasheet
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

void Power_Enable_ADAU1979(bool state){
	if(state){
		HAL_GPIO_WritePin(ADC_PD_RST_GPIO_Port, ADC_PD_RST_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(EN_MIC_PWR_GPIO_Port, EN_MIC_PWR_Pin, GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(ADC_PD_RST_GPIO_Port, ADC_PD_RST_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(EN_MIC_PWR_GPIO_Port, EN_MIC_PWR_Pin, GPIO_PIN_RESET);

	}
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
}

void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai){
	//	 f_write(&WavFile, &audioSample[AUDIO_BUFFER_HALF_LEN], AUDIO_BUFFER_HALF_LEN, (void*)&byteswritten);
	sampleCntr++;
	SAI_FULL_CALLBACK = 1;

}
/* USER CODE END 4 */

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
