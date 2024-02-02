/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    App/app_ble.c
  * @author  MCD Application Team
  * @brief   BLE Application
  *****************************************************************************
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

#include "app_common.h"

#include "dbg_trace.h"
#include "ble.h"
#include "tl.h"
#include "app_ble.h"

#include "cmsis_os.h"
#include "shci.h"
#include "stm32_lpm.h"
#include "otp.h"

#include "hrs_app.h"
#include "dis_app.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dts.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/

/**
 * security parameters structure
 */
typedef struct _tSecurityParams
{
  /**
   * IO capability of the device
   */
  uint8_t ioCapability;

  /**
   * Authentication requirement of the device
   * Man In the Middle protection required?
   */
  uint8_t mitm_mode;

  /**
   * bonding mode of the device
   */
  uint8_t bonding_mode;

  /**
   * Flag to tell whether OOB data has
   * to be used during the pairing process
   */
  uint8_t OOB_Data_Present;

  /**
   * OOB data to be used in the pairing process if
   * OOB_Data_Present is set to TRUE
   */
  uint8_t OOB_Data[16];

  /**
   * this variable indicates whether to use a fixed pin
   * during the pairing process or a passkey has to be
   * requested to the application during the pairing process
   * 0 implies use fixed pin and 1 implies request for passkey
   */
  uint8_t Use_Fixed_Pin;

  /**
   * minimum encryption key size requirement
   */
  uint8_t encryptionKeySizeMin;

  /**
   * maximum encryption key size requirement
   */
  uint8_t encryptionKeySizeMax;

  /**
   * fixed pin to be used in the pairing process if
   * Use_Fixed_Pin is set to 1
   */
  uint32_t Fixed_Pin;

  /**
   * this flag indicates whether the host has to initiate
   * the security, wait for pairing or does not have any security
   * requirements.\n
   * 0x00 : no security required
   * 0x01 : host should initiate security by sending the slave security
   *        request command
   * 0x02 : host need not send the clave security request but it
   * has to wait for paiirng to complete before doing any other
   * processing
   */
  uint8_t initiateSecurity;
}tSecurityParams;

/**
 * global context
 * contains the variables common to all
 * services
 */
typedef struct _tBLEProfileGlobalContext
{
  /**
   * security requirements of the host
   */
  tSecurityParams bleSecurityParam;

  /**
   * gap service handle
   */
  uint16_t gapServiceHandle;

  /**
   * device name characteristic handle
   */
  uint16_t devNameCharHandle;

  /**
   * appearance characteristic handle
   */
  uint16_t appearanceCharHandle;

  /**
   * connection handle of the current active connection
   * When not in connection, the handle is set to 0xFFFF
   */
  uint16_t connectionHandle;

  /**
   * length of the UUID list to be used while advertising
   */
  uint8_t advtServUUIDlen;

  /**
   * the UUID list to be used while advertising
   */
  uint8_t advtServUUID[100];
  /* USER CODE BEGIN BleGlobalContext_t*/

  /* USER CODE END BleGlobalContext_t */
}BleGlobalContext_t;

typedef struct
{
  BleGlobalContext_t BleApplicationContext_legacy;
  APP_BLE_ConnStatus_t Device_Connection_Status;

  /**
   * ID of the Advertising Timeout
   */
  uint8_t Advertising_mgr_timer_Id;
  /* USER CODE BEGIN PTD_1*/

  /* USER CODE END PTD_1 */
}BleApplicationContext_t;

/* USER CODE BEGIN PTD */
#ifndef CUSTOM_BT_PARAMETERS

/* USER CODE END PTD */

/* Private defines -----------------------------------------------------------*/
#define APPBLE_GAP_DEVICE_NAME_LENGTH 7
#define FAST_ADV_TIMEOUT               (30*1000*1000/CFG_TS_TICK_VAL) /**< 30s */
#define INITIAL_ADV_TIMEOUT            (60*1000*1000/CFG_TS_TICK_VAL) /**< 60s */

#define BD_ADDR_SIZE_LOCAL    6

/* USER CODE BEGIN PD */
#else
#define APPBLE_GAP_DEVICE_NAME_LENGTH 16
#define FAST_ADV_TIMEOUT               (30*1000*1000/CFG_TS_TICK_VAL) /**< 30s */
#define INITIAL_ADV_TIMEOUT            (60*1000*1000/CFG_TS_TICK_VAL) /**< 60s */

#define BD_ADDR_SIZE_LOCAL    6
#endif
char hexToAscii(uint8_t val);
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#ifndef CUSTOM_BT_PARAMETERS
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
PLACE_IN_SECTION("MB_MEM1") ALIGN(4) static TL_CmdPacket_t BleCmdBuffer;

static const uint8_t a_MBdAddr[BD_ADDR_SIZE_LOCAL] =
{
  (uint8_t)((CFG_ADV_BD_ADDRESS & 0x0000000000FF)),
  (uint8_t)((CFG_ADV_BD_ADDRESS & 0x00000000FF00) >> 8),
  (uint8_t)((CFG_ADV_BD_ADDRESS & 0x000000FF0000) >> 16),
  (uint8_t)((CFG_ADV_BD_ADDRESS & 0x0000FF000000) >> 24),
  (uint8_t)((CFG_ADV_BD_ADDRESS & 0x00FF00000000) >> 32),
  (uint8_t)((CFG_ADV_BD_ADDRESS & 0xFF0000000000) >> 40)
};

static uint8_t a_BdAddrUdn[BD_ADDR_SIZE_LOCAL];

/**
 *   Identity root key used to derive IRK and DHK(Legacy)
 */
static const uint8_t a_BLE_CfgIrValue[16] = CFG_BLE_IR;

/**
 * Encryption root key used to derive LTK(Legacy) and CSRK
 */
static const uint8_t a_BLE_CfgErValue[16] = CFG_BLE_ER;

/**
 * These are the two tags used to manage a power failure during OTA
 * The MagicKeywordAdress shall be mapped @0x140 from start of the binary image
 * The MagicKeywordvalue is checked in the ble_ota application
 */
PLACE_IN_SECTION("TAG_OTA_END") const uint32_t MagicKeywordValue = 0x94448A29 ;
PLACE_IN_SECTION("TAG_OTA_START") const uint32_t MagicKeywordAddress = (uint32_t)&MagicKeywordValue;

static BleApplicationContext_t BleApplicationContext;
static uint16_t AdvIntervalMin, AdvIntervalMax;

static const char a_LocalName[] = {AD_TYPE_COMPLETE_LOCAL_NAME ,'B','U','Z','Z','C'};
uint8_t a_ManufData[14] = {sizeof(a_ManufData)-1,
                           AD_TYPE_MANUFACTURER_SPECIFIC_DATA,
                           0x01 /*SKD version */,
                           0x00 /* Generic*/,
                           0x00 /* GROUP A Feature  */,
                           0x00 /* GROUP A Feature */,
                           0x00 /* GROUP B Feature */,
                           0x00 /* GROUP B Feature */,
                           0x00, /* BLE MAC start -MSB */
                           0x00,
                           0x00,
                           0x00,
                           0x00,
                           0x00, /* BLE MAC stop */
                          };

/* USER CODE BEGIN PV */
/* USER CODE BEGIN PV */
#else
PLACE_IN_SECTION("MB_MEM1") ALIGN(4) static TL_CmdPacket_t BleCmdBuffer;

static const uint8_t a_MBdAddr[BD_ADDR_SIZE_LOCAL] = {
		(uint8_t) ((CFG_ADV_BD_ADDRESS & 0x0000000000FF)),
		(uint8_t) ((CFG_ADV_BD_ADDRESS & 0x00000000FF00) >> 8),
		(uint8_t) ((CFG_ADV_BD_ADDRESS & 0x000000FF0000) >> 16),
		(uint8_t) ((CFG_ADV_BD_ADDRESS & 0x0000FF000000) >> 24),
		(uint8_t) ((CFG_ADV_BD_ADDRESS & 0x00FF00000000) >> 32),
		(uint8_t) ((CFG_ADV_BD_ADDRESS & 0xFF0000000000) >> 40) };

static uint8_t a_BdAddrUdn[BD_ADDR_SIZE_LOCAL];
/**
 *   Identity root key used to derive LTK and CSRK
 */
static const uint8_t a_BLE_CfgIrValue[16] = CFG_BLE_IRK;

/**
 * Encryption root key used to derive LTK and CSRK
 */
static const uint8_t a_BLE_CfgErValue[16] = CFG_BLE_ERK;


				/**
				 * These are the two tags used to manage a power failure during OTA
				 * The MagicKeywordAdress shall be mapped @0x140 from start of the binary image
				 * The MagicKeywordvalue is checked in the ble_ota application
				 */
//				PLACE_IN_SECTION("TAG_OTA_END") const uint32_t MagicKeywordValue = 0x94448A29;
//				PLACE_IN_SECTION("TAG_OTA_START") const uint32_t MagicKeywordAddress = (uint32_t)&MagicKeywordValue;
//
				PLACE_IN_SECTION("BLE_APP_CONTEXT") static BleApplicationContext_t BleApplicationContext;
				PLACE_IN_SECTION("BLE_APP_CONTEXT") static uint16_t AdvIntervalMin,
AdvIntervalMax;
//				static BleApplicationContext_t BleApplicationContext;
//				static uint16_t AdvIntervalMin, AdvIntervalMax;


uint8_t a_ManufData[14] = { sizeof(a_ManufData) - 1,
AD_TYPE_MANUFACTURER_SPECIFIC_DATA, 0x01/*SKD version */, 0x00 /* Generic*/,
		0x00 /* GROUP A Feature  */, 0x00 /* GROUP A Feature */,
		0x00 /* GROUP B Feature */, 0x00 /* GROUP B Feature */, 0x00, /* BLE MAC start -MSB */
		0x00, 0x00, 0x00, 0x00, 0x00, /* BLE MAC stop */

};

uint8_t a_ManufDataCameraWakeup[28];

#endif

osThreadId_t LinkConfigProcessId;
osThreadId_t AdvCancelProcessId;
osThreadId_t AdvReqProcessId;
/* USER CODE END PV */

/* Global variables ----------------------------------------------------------*/
osMutexId_t MtxHciId;
osSemaphoreId_t SemHciId;
osThreadId_t AdvUpdateProcessId;
osThreadId_t HciUserEvtProcessId;

const osThreadAttr_t AdvUpdateProcess_attr = {
    .name = CFG_ADV_UPDATE_PROCESS_NAME,
    .attr_bits = CFG_ADV_UPDATE_PROCESS_ATTR_BITS,
    .cb_mem = CFG_ADV_UPDATE_PROCESS_CB_MEM,
    .cb_size = CFG_ADV_UPDATE_PROCESS_CB_SIZE,
    .stack_mem = CFG_ADV_UPDATE_PROCESS_STACK_MEM,
    .priority = CFG_ADV_UPDATE_PROCESS_PRIORITY,
    .stack_size = CFG_ADV_UPDATE_PROCESS_STACK_SIZE
};

const osThreadAttr_t HciUserEvtProcess_attr = {
    .name = CFG_HCI_USER_EVT_PROCESS_NAME,
    .attr_bits = CFG_HCI_USER_EVT_PROCESS_ATTR_BITS,
    .cb_mem = CFG_HCI_USER_EVT_PROCESS_CB_MEM,
    .cb_size = CFG_HCI_USER_EVT_PROCESS_CB_SIZE,
    .stack_mem = CFG_HCI_USER_EVT_PROCESS_STACK_MEM,
    .priority = CFG_HCI_USER_EVT_PROCESS_PRIORITY,
    .stack_size = CFG_HCI_USER_EVT_PROCESS_STACK_SIZE
};

/* Private function prototypes -----------------------------------------------*/
static void HciUserEvtProcess(void *argument);
static void BLE_UserEvtRx(void *p_Payload);
static void BLE_StatusNot(HCI_TL_CmdStatus_t Status);
static void Ble_Tl_Init(void);
static void Ble_Hci_Gap_Gatt_Init(void);
static const uint8_t* BleGetBdAddress(void);
void Adv_Request(APP_BLE_ConnStatus_t NewStatus);
static void Add_Advertisment_Service_UUID(uint16_t servUUID);
static void Adv_Mgr(void);
static void AdvUpdateProcess(void *argument);
static void Adv_Update(void);

/* USER CODE BEGIN PFP */
char a_LocalName[20];
char a_buzzCamName[20];
char a_camName[20];
/* USER CODE END PFP */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Functions Definition ------------------------------------------------------*/
void APP_BLE_Init(void)
{
  SHCI_CmdStatus_t status;
  /* USER CODE BEGIN APP_BLE_Init_1 */
#ifndef CUSTOM_BT_PARAMETERS
  /* USER CODE END APP_BLE_Init_1 */
  SHCI_C2_Ble_Init_Cmd_Packet_t ble_init_cmd_packet =
  {
    {{0,0,0}},                          /**< Header unused */
    {0,                                 /** pBleBufferAddress not used */
     0,                                 /** BleBufferSize not used */
     CFG_BLE_NUM_GATT_ATTRIBUTES,
     CFG_BLE_NUM_GATT_SERVICES,
     CFG_BLE_ATT_VALUE_ARRAY_SIZE,
     CFG_BLE_NUM_LINK,
     CFG_BLE_DATA_LENGTH_EXTENSION,
     CFG_BLE_PREPARE_WRITE_LIST_SIZE,
     CFG_BLE_MBLOCK_COUNT,
     CFG_BLE_MAX_ATT_MTU,
     CFG_BLE_PERIPHERAL_SCA,
     CFG_BLE_CENTRAL_SCA,
     CFG_BLE_LS_SOURCE,
     CFG_BLE_MAX_CONN_EVENT_LENGTH,
     CFG_BLE_HSE_STARTUP_TIME,
     CFG_BLE_VITERBI_MODE,
     CFG_BLE_OPTIONS,
     0,
     CFG_BLE_MAX_COC_INITIATOR_NBR,
     CFG_BLE_MIN_TX_POWER,
     CFG_BLE_MAX_TX_POWER,
     CFG_BLE_RX_MODEL_CONFIG,
     CFG_BLE_MAX_ADV_SET_NBR,
     CFG_BLE_MAX_ADV_DATA_LEN,
     CFG_BLE_TX_PATH_COMPENS,
     CFG_BLE_RX_PATH_COMPENS,
     CFG_BLE_CORE_VERSION,
     CFG_BLE_OPTIONS_EXT
    }
  };

  /**
   * Initialize Ble Transport Layer
   */
  Ble_Tl_Init();

  /**
   * Do not allow standby in the application
   */
  UTIL_LPM_SetOffMode(1 << CFG_LPM_APP_BLE, UTIL_LPM_DISABLE);

  /**
   * Register the hci transport layer to handle BLE User Asynchronous Events
   */
  HciUserEvtProcessId = osThreadNew(HciUserEvtProcess, NULL, &HciUserEvtProcess_attr);

  /**
   * Starts the BLE Stack on CPU2
   */
  status = SHCI_C2_BLE_Init(&ble_init_cmd_packet);
  if (status != SHCI_Success)
  {
    APP_DBG_MSG("  Fail   : SHCI_C2_BLE_Init command, result: 0x%02x\n\r", status);
    /* if you are here, maybe CPU2 doesn't contain STM32WB_Copro_Wireless_Binaries, see Release_Notes.html */
    Error_Handler();
  }
  else
  {
    APP_DBG_MSG("  Success: SHCI_C2_BLE_Init command\n\r");
  }

  /**
   * Initialization of HCI & GATT & GAP layer
   */
  Ble_Hci_Gap_Gatt_Init();

  /**
   * Initialization of the BLE Services
   */
  SVCCTL_Init();

  /**
   * Initialization of the BLE App Context
   */
  BleApplicationContext.Device_Connection_Status = APP_BLE_IDLE;
  BleApplicationContext.BleApplicationContext_legacy.connectionHandle = 0xFFFF;

  /**
   * From here, all initialization are BLE application specific
   */
  AdvUpdateProcessId = osThreadNew(AdvUpdateProcess, NULL, &AdvUpdateProcess_attr);

  /**
   * Initialization of ADV - Ad Manufacturer Element - Support OTA Bit Mask
   */
#if (BLE_CFG_OTA_REBOOT_CHAR != 0)
  a_ManufData[sizeof(a_ManufData)-8] = CFG_FEATURE_OTA_REBOOT;
#endif /* BLE_CFG_OTA_REBOOT_CHAR != 0 */

  /**
   * Initialize DIS Application
   */
  DISAPP_Init();

  /**
   * Initialize HRS Application
   */
  HRSAPP_Init();

  /* USER CODE BEGIN APP_BLE_Init_3 */
  /* USER CODE BEGIN APP_BLE_Init_3 */
#else
  SHCI_C2_Ble_Init_Cmd_Packet_t ble_init_cmd_packet =
   {
     {{0,0,0}},                          /**< Header unused */
     {0,                                 /** pBleBufferAddress not used */
      0,                                 /** BleBufferSize not used */
      CFG_BLE_NUM_GATT_ATTRIBUTES,
      CFG_BLE_NUM_GATT_SERVICES,
      CFG_BLE_ATT_VALUE_ARRAY_SIZE,
      CFG_BLE_NUM_LINK,
      CFG_BLE_DATA_LENGTH_EXTENSION,
      CFG_BLE_PREPARE_WRITE_LIST_SIZE,
      CFG_BLE_MBLOCK_COUNT,
      CFG_BLE_MAX_ATT_MTU,
      CFG_BLE_SLAVE_SCA,
      CFG_BLE_MASTER_SCA,
      CFG_BLE_LS_SOURCE,
      CFG_BLE_MAX_CONN_EVENT_LENGTH,
      CFG_BLE_HSE_STARTUP_TIME,
      CFG_BLE_VITERBI_MODE,
      CFG_BLE_OPTIONS,
      0,
      CFG_BLE_MAX_COC_INITIATOR_NBR,
      CFG_BLE_MIN_TX_POWER,
      CFG_BLE_MAX_TX_POWER
//	  ,
//      CFG_BLE_RX_MODEL_CONFIG,
//      CFG_BLE_MAX_ADV_SET_NBR,
//      CFG_BLE_MAX_ADV_DATA_LEN,
//      CFG_BLE_TX_PATH_COMPENS,
//      CFG_BLE_RX_PATH_COMPENS,
//      CFG_BLE_CORE_VERSION,
//      CFG_BLE_OPTIONS_EXT
     }
   };

  a_ManufDataCameraWakeup[0] = sizeof(a_ManufDataCameraWakeup) - 1;
  a_ManufDataCameraWakeup[1] = AD_TYPE_MANUFACTURER_SPECIFIC_DATA;
  /* set the manufacturing data for wakeon packet */
  a_ManufDataCameraWakeup[2] = 0x4c;
  a_ManufDataCameraWakeup[3] = 0x00;
  a_ManufDataCameraWakeup[4] = 0x02;
  a_ManufDataCameraWakeup[5] = 0x15;
  a_ManufDataCameraWakeup[6] = 0x09;
  a_ManufDataCameraWakeup[7] = 0x4f;
  a_ManufDataCameraWakeup[8] = 0x52;
  a_ManufDataCameraWakeup[9] = 0x42;
  a_ManufDataCameraWakeup[10] = 0x49;
  a_ManufDataCameraWakeup[11] = 0x54;
  a_ManufDataCameraWakeup[12] = 0x09;
  a_ManufDataCameraWakeup[13] = 0xff;
  a_ManufDataCameraWakeup[14] = 0x0f;
  a_ManufDataCameraWakeup[15] = 0x00;
    /* note: see powerOnPrevConnectedCameras() for bytes 16-21 */
  a_ManufDataCameraWakeup[22] = 0x00;
  a_ManufDataCameraWakeup[23] = 0x00;
  a_ManufDataCameraWakeup[24] = 0x00;
  a_ManufDataCameraWakeup[25] = 0x00;
  a_ManufDataCameraWakeup[26] = 0xe4;
  a_ManufDataCameraWakeup[27] = 0x01;

   /**
    * Initialize Ble Transport Layer
    */
   Ble_Tl_Init();

   /**
    * Do not allow standby in the application
    */
   UTIL_LPM_SetOffMode(1 << CFG_LPM_APP_BLE, UTIL_LPM_DISABLE);

   /**
    * Register the hci transport layer to handle BLE User Asynchronous Events
    */
   HciUserEvtProcessId = osThreadNew(HciUserEvtProcess, NULL, &HciUserEvtProcess_attr);

   /**
    * Starts the BLE Stack on CPU2
    */
   status = SHCI_C2_BLE_Init(&ble_init_cmd_packet);
   if (status != SHCI_Success)
   {
     APP_DBG_MSG("  Fail   : SHCI_C2_BLE_Init command, result: 0x%02x\n\r", status);
     /* if you are here, maybe CPU2 doesn't contain STM32WB_Copro_Wireless_Binaries, see Release_Notes.html */
     Error_Handler();
   }
   else
   {
     APP_DBG_MSG("  Success: SHCI_C2_BLE_Init command\n\r");
   }

   /**
    * Initialization of HCI & GATT & GAP layer
    */
   Ble_Hci_Gap_Gatt_Init();

   /**
    * Initialization of the BLE Services
    */
   SVCCTL_Init();

   /**
    * Initialization of the BLE App Context
    */
   BleApplicationContext.Device_Connection_Status = APP_BLE_IDLE;
   BleApplicationContext.BleApplicationContext_legacy.connectionHandle = 0xFFFF;

   /**
    * From here, all initialization are BLE application specific
    */
   AdvUpdateProcessId = osThreadNew(AdvUpdateProcess, NULL, &AdvUpdateProcess_attr);

   /**
    * Initialization of ADV - Ad Manufacturer Element - Support OTA Bit Mask
    */
 #if (BLE_CFG_OTA_REBOOT_CHAR != 0)
   a_ManufData[sizeof(a_ManufData)-8] = CFG_FEATURE_OTA_REBOOT;
 #endif /* BLE_CFG_OTA_REBOOT_CHAR != 0 */
#endif

#ifndef CUSTOM_BT_PARAMETERS
  /* USER CODE END APP_BLE_Init_3 */

  /**
   * Create timer to handle the connection state machine
   */
  HW_TS_Create(CFG_TIM_PROC_ID_ISR, &(BleApplicationContext.Advertising_mgr_timer_Id), hw_ts_SingleShot, Adv_Mgr);

  /**
   * Make device discoverable
   */
  BleApplicationContext.BleApplicationContext_legacy.advtServUUID[0] = AD_TYPE_16_BIT_SERV_UUID;
  BleApplicationContext.BleApplicationContext_legacy.advtServUUIDlen = 1;
  Add_Advertisment_Service_UUID(HEART_RATE_SERVICE_UUID);

  /* Initialize intervals for reconnexion without intervals update */
  AdvIntervalMin = CFG_FAST_CONN_ADV_INTERVAL_MIN;
  AdvIntervalMax = CFG_FAST_CONN_ADV_INTERVAL_MAX;

  /**
   * Start to Advertise to be connected by Collector
   */
  Adv_Request(APP_BLE_FAST_ADV);

  /* USER CODE BEGIN APP_BLE_Init_2 */
  /* USER CODE BEGIN APP_BLE_Init_2 */
#else
  /**
   * Create timer to handle the connection state machine
   */
  HW_TS_Create(CFG_TIM_PROC_ID_ISR, &(BleApplicationContext.Advertising_mgr_timer_Id), hw_ts_SingleShot, Adv_Mgr);

  /**
   * Make device discoverable
   */
  BleApplicationContext.BleApplicationContext_legacy.advtServUUID[0] = AD_TYPE_16_BIT_SERV_UUID;
  BleApplicationContext.BleApplicationContext_legacy.advtServUUIDlen = 1;
  Add_Advertisment_Service_UUID(BUZZCAM_SERVICE_UUID);

  /* Initialize intervals for reconnexion without intervals update */
  AdvIntervalMin = CFG_FAST_CONN_ADV_INTERVAL_MIN;
  AdvIntervalMax = CFG_FAST_CONN_ADV_INTERVAL_MAX;

  /**
   * Start to Advertise to be connected by Collector
   */
  Adv_Request(APP_BLE_FAST_ADV);
#endif
  /* USER CODE END APP_BLE_Init_2 */

  return;
}

SVCCTL_UserEvtFlowStatus_t SVCCTL_App_Notification(void *p_Pckt)
{
  hci_event_pckt    *p_event_pckt;
  evt_le_meta_event *p_meta_evt;
  evt_blecore_aci   *p_blecore_evt;
  uint8_t           Tx_phy, Rx_phy;
  tBleStatus        ret = BLE_STATUS_INVALID_PARAMS;
  hci_le_connection_complete_event_rp0        *p_connection_complete_event;
  hci_disconnection_complete_event_rp0        *p_disconnection_complete_event;
  hci_le_phy_update_complete_event_rp0        *p_evt_le_phy_update_complete;
#if (CFG_DEBUG_APP_TRACE != 0)
  hci_le_connection_update_complete_event_rp0 *p_connection_update_complete_event;
#endif /* CFG_DEBUG_APP_TRACE != 0 */

  /* USER CODE BEGIN SVCCTL_App_Notification */

  /* USER CODE END SVCCTL_App_Notification */

  p_event_pckt = (hci_event_pckt*) ((hci_uart_pckt *) p_Pckt)->data;

  switch (p_event_pckt->evt)
  {
    case HCI_DISCONNECTION_COMPLETE_EVT_CODE:
    {
      p_disconnection_complete_event = (hci_disconnection_complete_event_rp0 *) p_event_pckt->data;

      if (p_disconnection_complete_event->Connection_Handle == BleApplicationContext.BleApplicationContext_legacy.connectionHandle)
      {
        BleApplicationContext.BleApplicationContext_legacy.connectionHandle = 0;
        BleApplicationContext.Device_Connection_Status = APP_BLE_IDLE;
        APP_DBG_MSG(">>== HCI_DISCONNECTION_COMPLETE_EVT_CODE\n");
        APP_DBG_MSG("     - Connection Handle:   0x%x\n     - Reason:    0x%x\n\r",
                    p_disconnection_complete_event->Connection_Handle,
                    p_disconnection_complete_event->Reason);

        /* USER CODE BEGIN EVT_DISCONN_COMPLETE_2 */

        /* USER CODE END EVT_DISCONN_COMPLETE_2 */
      }

      /* USER CODE BEGIN EVT_DISCONN_COMPLETE_1 */

      /* USER CODE END EVT_DISCONN_COMPLETE_1 */

      /* restart advertising */
      Adv_Request(APP_BLE_FAST_ADV);

      /* USER CODE BEGIN EVT_DISCONN_COMPLETE */

      /* USER CODE END EVT_DISCONN_COMPLETE */
      break; /* HCI_DISCONNECTION_COMPLETE_EVT_CODE */
    }

    case HCI_LE_META_EVT_CODE:
    {
      p_meta_evt = (evt_le_meta_event*) p_event_pckt->data;
      /* USER CODE BEGIN EVT_LE_META_EVENT */

      /* USER CODE END EVT_LE_META_EVENT */
      switch (p_meta_evt->subevent)
      {
        case HCI_LE_CONNECTION_UPDATE_COMPLETE_SUBEVT_CODE:
#if (CFG_DEBUG_APP_TRACE != 0)
          p_connection_update_complete_event = (hci_le_connection_update_complete_event_rp0 *) p_meta_evt->data;
          APP_DBG_MSG(">>== HCI_LE_CONNECTION_UPDATE_COMPLETE_SUBEVT_CODE\n");
          APP_DBG_MSG("     - Connection Interval:   %.2f ms\n     - Connection latency:    %d\n     - Supervision Timeout: %d ms\n\r",
                       p_connection_update_complete_event->Conn_Interval*1.25,
                       p_connection_update_complete_event->Conn_Latency,
                       p_connection_update_complete_event->Supervision_Timeout*10);
#endif /* CFG_DEBUG_APP_TRACE != 0 */

          /* USER CODE BEGIN EVT_LE_CONN_UPDATE_COMPLETE */

          /* USER CODE END EVT_LE_CONN_UPDATE_COMPLETE */
          break;

        case HCI_LE_PHY_UPDATE_COMPLETE_SUBEVT_CODE:
          p_evt_le_phy_update_complete = (hci_le_phy_update_complete_event_rp0*)p_meta_evt->data;
          APP_DBG_MSG("==>> HCI_LE_PHY_UPDATE_COMPLETE_SUBEVT_CODE - ");
          if (p_evt_le_phy_update_complete->Status == 0)
          {
            APP_DBG_MSG("status ok \n");
          }
          else
          {
            APP_DBG_MSG("status nok \n");
          }

          ret = hci_le_read_phy(BleApplicationContext.BleApplicationContext_legacy.connectionHandle, &Tx_phy, &Rx_phy);
          if (ret != BLE_STATUS_SUCCESS)
          {
            APP_DBG_MSG("==>> hci_le_read_phy : fail\n\r");
          }
          else
          {
            APP_DBG_MSG("==>> hci_le_read_phy - Success \n");

            if ((Tx_phy == TX_2M) && (Rx_phy == RX_2M))
            {
              APP_DBG_MSG("==>> PHY Param  TX= %d, RX= %d \n\r", Tx_phy, Rx_phy);
            }
            else
            {
              APP_DBG_MSG("==>> PHY Param  TX= %d, RX= %d \n\r", Tx_phy, Rx_phy);
            }
          }
          /* USER CODE BEGIN EVT_LE_PHY_UPDATE_COMPLETE */

          /* USER CODE END EVT_LE_PHY_UPDATE_COMPLETE */
          break;

        case HCI_LE_CONNECTION_COMPLETE_SUBEVT_CODE:
        {
          p_connection_complete_event = (hci_le_connection_complete_event_rp0 *) p_meta_evt->data;
          /**
           * The connection is done, there is no need anymore to schedule the LP ADV
           */

          HW_TS_Stop(BleApplicationContext.Advertising_mgr_timer_Id);

          APP_DBG_MSG(">>== HCI_LE_CONNECTION_COMPLETE_SUBEVT_CODE - Connection handle: 0x%x\n", p_connection_complete_event->Connection_Handle);
          APP_DBG_MSG("     - Connection established with Central: @:%02x:%02x:%02x:%02x:%02x:%02x\n",
                      p_connection_complete_event->Peer_Address[5],
                      p_connection_complete_event->Peer_Address[4],
                      p_connection_complete_event->Peer_Address[3],
                      p_connection_complete_event->Peer_Address[2],
                      p_connection_complete_event->Peer_Address[1],
                      p_connection_complete_event->Peer_Address[0]);
          APP_DBG_MSG("     - Connection Interval:   %.2f ms\n     - Connection latency:    %d\n     - Supervision Timeout: %d ms\n\r",
                      p_connection_complete_event->Conn_Interval*1.25,
                      p_connection_complete_event->Conn_Latency,
                      p_connection_complete_event->Supervision_Timeout*10
                     );

          if (BleApplicationContext.Device_Connection_Status == APP_BLE_LP_CONNECTING)
          {
            /* Connection as client */
            BleApplicationContext.Device_Connection_Status = APP_BLE_CONNECTED_CLIENT;
          }
          else
          {
            /* Connection as server */
            BleApplicationContext.Device_Connection_Status = APP_BLE_CONNECTED_SERVER;
          }
          BleApplicationContext.BleApplicationContext_legacy.connectionHandle = p_connection_complete_event->Connection_Handle;
          /* USER CODE BEGIN HCI_EVT_LE_CONN_COMPLETE */

          /* USER CODE END HCI_EVT_LE_CONN_COMPLETE */
          break; /* HCI_LE_CONNECTION_COMPLETE_SUBEVT_CODE */
        }

        default:
          /* USER CODE BEGIN SUBEVENT_DEFAULT */

          /* USER CODE END SUBEVENT_DEFAULT */
          break;
      }

      /* USER CODE BEGIN META_EVT */

      /* USER CODE END META_EVT */
      break; /* HCI_LE_META_EVT_CODE */
    }

    case HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE:
      p_blecore_evt = (evt_blecore_aci*) p_event_pckt->data;
      /* USER CODE BEGIN EVT_VENDOR */

      /* USER CODE END EVT_VENDOR */
      switch (p_blecore_evt->ecode)
      {
        /* USER CODE BEGIN ecode */

        /* USER CODE END ecode */

        case ACI_GAP_PROC_COMPLETE_VSEVT_CODE:
          APP_DBG_MSG(">>== ACI_GAP_PROC_COMPLETE_VSEVT_CODE \r");
          /* USER CODE BEGIN EVT_BLUE_GAP_PROCEDURE_COMPLETE */

          /* USER CODE END EVT_BLUE_GAP_PROCEDURE_COMPLETE */
          break; /* ACI_GAP_PROC_COMPLETE_VSEVT_CODE */

        /* USER CODE BEGIN BLUE_EVT */

        /* USER CODE END BLUE_EVT */
      }
      break; /* HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE */

      /* USER CODE BEGIN EVENT_PCKT */

      /* USER CODE END EVENT_PCKT */

    default:
      /* USER CODE BEGIN ECODE_DEFAULT*/

      /* USER CODE END ECODE_DEFAULT*/
      break;
  }

  return (SVCCTL_UserEvtFlowEnable);
}

APP_BLE_ConnStatus_t APP_BLE_Get_Server_Connection_Status(void)
{
  return BleApplicationContext.Device_Connection_Status;
}

/* USER CODE BEGIN FD*/
void APP_BLE_Init_Dyn_1( void )
{
	  SHCI_CmdStatus_t status;
	#ifndef CUSTOM_BT_PARAMETERS
	  SHCI_C2_Ble_Init_Cmd_Packet_t ble_init_cmd_packet =
	  {
	    {{0,0,0}},                          /**< Header unused */
	    {0,                                 /** pBleBufferAddress not used */
	     0,                                 /** BleBufferSize not used */
	     CFG_BLE_NUM_GATT_ATTRIBUTES,
	     CFG_BLE_NUM_GATT_SERVICES,
	     CFG_BLE_ATT_VALUE_ARRAY_SIZE,
	     CFG_BLE_NUM_LINK,
	     CFG_BLE_DATA_LENGTH_EXTENSION,
	     CFG_BLE_PREPARE_WRITE_LIST_SIZE,
	     CFG_BLE_MBLOCK_COUNT,
	     CFG_BLE_MAX_ATT_MTU,
	     CFG_BLE_PERIPHERAL_SCA,
	     CFG_BLE_CENTRAL_SCA,
	     CFG_BLE_LS_SOURCE,
	     CFG_BLE_MAX_CONN_EVENT_LENGTH,
	     CFG_BLE_HSE_STARTUP_TIME,
	     CFG_BLE_VITERBI_MODE,
	     CFG_BLE_OPTIONS,
	     0,
	     CFG_BLE_MAX_COC_INITIATOR_NBR,
	     CFG_BLE_MIN_TX_POWER,
	     CFG_BLE_MAX_TX_POWER,
	     CFG_BLE_RX_MODEL_CONFIG,
	     CFG_BLE_MAX_ADV_SET_NBR,
	     CFG_BLE_MAX_ADV_DATA_LEN,
	     CFG_BLE_TX_PATH_COMPENS,
	     CFG_BLE_RX_PATH_COMPENS,
	     CFG_BLE_CORE_VERSION,
	     CFG_BLE_OPTIONS_EXT
	    }
	  };

	  /**
	   * Initialize Ble Transport Layer
	   */
	  Ble_Tl_Init();

	  /**
	   * Do not allow standby in the application
	   */
	  UTIL_LPM_SetOffMode(1 << CFG_LPM_APP_BLE, UTIL_LPM_DISABLE);

	  /**
	   * Register the hci transport layer to handle BLE User Asynchronous Events
	   */
	  HciUserEvtProcessId = osThreadNew(HciUserEvtProcess, NULL, &HciUserEvtProcess_attr);

	  /**
	   * Starts the BLE Stack on CPU2
	   */
	  status = SHCI_C2_BLE_Init(&ble_init_cmd_packet);
	  if (status != SHCI_Success)
	  {
	    APP_DBG_MSG("  Fail   : SHCI_C2_BLE_Init command, result: 0x%02x\n\r", status);
	    /* if you are here, maybe CPU2 doesn't contain STM32WB_Copro_Wireless_Binaries, see Release_Notes.html */
	    Error_Handler();
	  }
	  else
	  {
	    APP_DBG_MSG("  Success: SHCI_C2_BLE_Init command\n\r");
	  }

	  /**
	   * Initialization of HCI & GATT & GAP layer
	   */
	  Ble_Hci_Gap_Gatt_Init();

	  /**
	   * Initialization of the BLE Services
	   */
	  SVCCTL_Init();

	  /**
	   * Initialization of the BLE App Context
	   */
	  BleApplicationContext.Device_Connection_Status = APP_BLE_IDLE;
	  BleApplicationContext.BleApplicationContext_legacy.connectionHandle = 0xFFFF;

	  /**
	   * From here, all initialization are BLE application specific
	   */
	  AdvUpdateProcessId = osThreadNew(AdvUpdateProcess, NULL, &AdvUpdateProcess_attr);

	  /**
	   * Initialization of ADV - Ad Manufacturer Element - Support OTA Bit Mask
	   */
	#if (BLE_CFG_OTA_REBOOT_CHAR != 0)
	  a_ManufData[sizeof(a_ManufData)-8] = CFG_FEATURE_OTA_REBOOT;
	#endif /* BLE_CFG_OTA_REBOOT_CHAR != 0 */

	  /**
	   * Initialize DIS Application
	   */
	  DISAPP_Init();

	  /**
	   * Initialize HRS Application
	   */
	  HRSAPP_Init();


	#else
//	  SHCI_C2_Ble_Init_Cmd_Packet_t ble_init_cmd_packet =
//	   {
//	     {{0,0,0}},                          /**< Header unused */
//	     {0,                                 /** pBleBufferAddress not used */
//	      0,                                 /** BleBufferSize not used */
//	      CFG_BLE_NUM_GATT_ATTRIBUTES,
//	      CFG_BLE_NUM_GATT_SERVICES,
//	      CFG_BLE_ATT_VALUE_ARRAY_SIZE,
//	      CFG_BLE_NUM_LINK,
//	      CFG_BLE_DATA_LENGTH_EXTENSION,
//	      CFG_BLE_PREPARE_WRITE_LIST_SIZE,
//	      CFG_BLE_MBLOCK_COUNT,
//	      CFG_BLE_MAX_ATT_MTU,
//	      CFG_BLE_SLAVE_SCA,
//	      CFG_BLE_MASTER_SCA,
//	      CFG_BLE_LS_SOURCE,
//	      CFG_BLE_MAX_CONN_EVENT_LENGTH,
//	      CFG_BLE_HSE_STARTUP_TIME,
//	      CFG_BLE_VITERBI_MODE,
//	      CFG_BLE_OPTIONS,
//	      0,
//	      CFG_BLE_MAX_COC_INITIATOR_NBR,
//	      CFG_BLE_MIN_TX_POWER,
//	      CFG_BLE_MAX_TX_POWER,
//	      CFG_BLE_RX_MODEL_CONFIG,
//	      CFG_BLE_MAX_ADV_SET_NBR,
//	      CFG_BLE_MAX_ADV_DATA_LEN,
//	      CFG_BLE_TX_PATH_COMPENS,
//	      CFG_BLE_RX_PATH_COMPENS,
//	      CFG_BLE_CORE_VERSION,
//	      CFG_BLE_OPTIONS_EXT
//	     }
//	   };

	  SHCI_C2_Ble_Init_Cmd_Packet_t ble_init_cmd_packet =
	  {
	    {{0,0,0}},                          /**< Header unused */
	    {0,                                 /** pBleBufferAddress not used */
	    0,                                  /** BleBufferSize not used */
	    CFG_BLE_NUM_GATT_ATTRIBUTES,
	    CFG_BLE_NUM_GATT_SERVICES,
	    CFG_BLE_ATT_VALUE_ARRAY_SIZE,
	    CFG_BLE_NUM_LINK,
	    CFG_BLE_DATA_LENGTH_EXTENSION,
	    CFG_BLE_PREPARE_WRITE_LIST_SIZE,
	    CFG_BLE_MBLOCK_COUNT,
	    CFG_BLE_MAX_ATT_MTU,
	    CFG_BLE_SLAVE_SCA,
	    CFG_BLE_MASTER_SCA,
	    CFG_BLE_LS_SOURCE,
	    CFG_BLE_MAX_CONN_EVENT_LENGTH,
	    CFG_BLE_HSE_STARTUP_TIME,
	    CFG_BLE_VITERBI_MODE,
	    CFG_BLE_OPTIONS,
	    0,
	    CFG_BLE_MAX_COC_INITIATOR_NBR,
	    CFG_BLE_MIN_TX_POWER,
	    CFG_BLE_MAX_TX_POWER}
	  };


	  a_ManufDataCameraWakeup[0] = sizeof(a_ManufDataCameraWakeup) - 1;
	  a_ManufDataCameraWakeup[1] = AD_TYPE_MANUFACTURER_SPECIFIC_DATA;
	  /* set the manufacturing data for wakeon packet */
	  a_ManufDataCameraWakeup[2] = 0x4c;
	  a_ManufDataCameraWakeup[3] = 0x00;
	  a_ManufDataCameraWakeup[4] = 0x02;
	  a_ManufDataCameraWakeup[5] = 0x15;
	  a_ManufDataCameraWakeup[6] = 0x09;
	  a_ManufDataCameraWakeup[7] = 0x4f;
	  a_ManufDataCameraWakeup[8] = 0x52;
	  a_ManufDataCameraWakeup[9] = 0x42;
	  a_ManufDataCameraWakeup[10] = 0x49;
	  a_ManufDataCameraWakeup[11] = 0x54;
	  a_ManufDataCameraWakeup[12] = 0x09;
	  a_ManufDataCameraWakeup[13] = 0xff;
	  a_ManufDataCameraWakeup[14] = 0x0f;
	  a_ManufDataCameraWakeup[15] = 0x00;
	    /* note: see powerOnPrevConnectedCameras() for bytes 16-21 */
	  a_ManufDataCameraWakeup[22] = 0x00;
	  a_ManufDataCameraWakeup[23] = 0x00;
	  a_ManufDataCameraWakeup[24] = 0x00;
	  a_ManufDataCameraWakeup[25] = 0x00;
	  a_ManufDataCameraWakeup[26] = 0xe4;
	  a_ManufDataCameraWakeup[27] = 0x01;

	   /**
	    * Initialize Ble Transport Layer
	    */
	   Ble_Tl_Init();

	   /**
	    * Do not allow standby in the application
	    */
	   UTIL_LPM_SetOffMode(1 << CFG_LPM_APP_BLE, UTIL_LPM_DISABLE);

	   /**
	    * Register the hci transport layer to handle BLE User Asynchronous Events
	    */
	   HciUserEvtProcessId = osThreadNew(HciUserEvtProcess, NULL, &HciUserEvtProcess_attr);

	   /**
	    * Starts the BLE Stack on CPU2
	    */
	   status = SHCI_C2_BLE_Init(&ble_init_cmd_packet);
	   if (status != SHCI_Success)
	   {
	     APP_DBG_MSG("  Fail   : SHCI_C2_BLE_Init command, result: 0x%02x\n\r", status);
	     /* if you are here, maybe CPU2 doesn't contain STM32WB_Copro_Wireless_Binaries, see Release_Notes.html */
	     Error_Handler();
	   }
	   else
	   {
	     APP_DBG_MSG("  Success: SHCI_C2_BLE_Init command\n\r");
	   }

	   /**
	    * Initialization of HCI & GATT & GAP layer
	    */
	   Ble_Hci_Gap_Gatt_Init();

	   /**
	    * Initialization of the BLE Services
	    */
	   SVCCTL_Init();

	   /**
	    * Initialization of the BLE App Context
	    */
	   BleApplicationContext.Device_Connection_Status = APP_BLE_IDLE;
	   BleApplicationContext.BleApplicationContext_legacy.connectionHandle = 0xFFFF;

	   /**
	    * From here, all initialization are BLE application specific
	    */
	   AdvUpdateProcessId = osThreadNew(AdvUpdateProcess, NULL, &AdvUpdateProcess_attr);

	   /**
	    * Initialization of ADV - Ad Manufacturer Element - Support OTA Bit Mask
	    */
	 #if (BLE_CFG_OTA_REBOOT_CHAR != 0)
	   a_ManufData[sizeof(a_ManufData)-8] = CFG_FEATURE_OTA_REBOOT;
	 #endif /* BLE_CFG_OTA_REBOOT_CHAR != 0 */
	#endif

	#ifndef CUSTOM_BT_PARAMETERS
	  /* USER CODE END APP_BLE_Init_3 */

	  /**
	   * Create timer to handle the connection state machine
	   */
	  HW_TS_Create(CFG_TIM_PROC_ID_ISR, &(BleApplicationContext.Advertising_mgr_timer_Id), hw_ts_SingleShot, Adv_Mgr);

	  /**
	   * Make device discoverable
	   */
	  BleApplicationContext.BleApplicationContext_legacy.advtServUUID[0] = AD_TYPE_16_BIT_SERV_UUID;
	  BleApplicationContext.BleApplicationContext_legacy.advtServUUIDlen = 1;
	  Add_Advertisment_Service_UUID(HEART_RATE_SERVICE_UUID);

	  /* Initialize intervals for reconnexion without intervals update */
	  AdvIntervalMin = CFG_FAST_CONN_ADV_INTERVAL_MIN;
	  AdvIntervalMax = CFG_FAST_CONN_ADV_INTERVAL_MAX;

	  /**
	   * Start to Advertise to be connected by Collector
	   */
	  Adv_Request(APP_BLE_FAST_ADV);


	#else

	  aci_hal_set_radio_activity_mask(0x0006);
	  /**
	   * Create timer to handle the connection state machine
	   */
	  HW_TS_Create(CFG_TIM_PROC_ID_ISR, &(BleApplicationContext.Advertising_mgr_timer_Id), hw_ts_SingleShot, Adv_Mgr);



#endif
	  return;
}

void APP_BLE_Init_Dyn_2( void ) {
	  /**
	   * Make device discoverable
	   */
	  BleApplicationContext.BleApplicationContext_legacy.advtServUUID[0] = AD_TYPE_16_BIT_SERV_UUID;
	  BleApplicationContext.BleApplicationContext_legacy.advtServUUIDlen = 1;
	  Add_Advertisment_Service_UUID(BUZZCAM_SERVICE_UUID);

	//  /**
	//   * Make device discoverable
	//   */
	//  BleApplicationContext.BleApplicationContext_legacy.advtServUUID[0] = NULL; //TODO: the heartbeat example uses: AD_TYPE_16_BIT_SERV_UUID
	//  BleApplicationContext.BleApplicationContext_legacy.advtServUUIDlen = 0;
	//  /* Initialize intervals for reconnexion without intervals update */
	AdvIntervalMin = CFG_FAST_CONN_ADV_INTERVAL_MIN;
	AdvIntervalMax = CFG_FAST_CONN_ADV_INTERVAL_MAX;

	/**
	 * Start to Advertise to be connected by P2P Client
	 */
#ifndef DYNAMIC_MODE
	Adv_Request(APP_BLE_FAST_ADV);
#else
	osThreadFlagsSet(AdvUpdateProcessId, 1);
//	Adv_Request(APP_BLE_LP_ADV);
#endif
	/* USER CODE BEGIN APP_BLE_Init_2 */

	/* USER CODE END APP_BLE_Init_2 */
	return;
}

void SVCCTL_InitCustomSvc(void) {
#ifndef TESTING_ACTIVE
	DTS_STM_Init();
#endif
}

void SVCCTL_SvcInit(void)
{
//  BAS_Init();
//
//  BLS_Init();
//
//  CRS_STM_Init();
//
//  //DIS_Init();
//
//  EDS_STM_Init();
//
//  HIDS_Init();
//
////  HRS_Init();
////
////  HTS_Init();
//
//  IAS_Init();
//
//  LLS_Init();
//
//  TPS_Init();
//
//  MOTENV_STM_Init();
//
//  P2PS_STM_Init();
//
//  ZDD_STM_Init();
//
//  OTAS_STM_Init();

//  BVOPUS_STM_Init();
//
//  MESH_Init();

  SVCCTL_InitCustomSvc();

  return;
}

/* USER CODE END FD*/

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/
static void Ble_Tl_Init(void)
{
  HCI_TL_HciInitConf_t Hci_Tl_Init_Conf;

  MtxHciId = osMutexNew(NULL);
  SemHciId = osSemaphoreNew(1, 0, NULL); /*< Create the semaphore and make it busy at initialization */

  Hci_Tl_Init_Conf.p_cmdbuffer = (uint8_t*)&BleCmdBuffer;
  Hci_Tl_Init_Conf.StatusNotCallBack = BLE_StatusNot;
  hci_init(BLE_UserEvtRx, (void*) &Hci_Tl_Init_Conf);

  return;
}

static void Ble_Hci_Gap_Gatt_Init(void)
{
  uint8_t role;
  uint16_t gap_service_handle, gap_dev_name_char_handle, gap_appearance_char_handle;
  const uint8_t *p_bd_addr;
  uint32_t srd_bd_addr[2];
  uint16_t a_appearance[1] = {BLE_CFG_GAP_APPEARANCE};
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;
  /* USER CODE BEGIN Ble_Hci_Gap_Gatt_Init*/

  /* USER CODE END Ble_Hci_Gap_Gatt_Init*/

  APP_DBG_MSG("==>> Start Ble_Hci_Gap_Gatt_Init function\n");

  /**
   * Initialize HCI layer
   */
  /*HCI Reset to synchronise BLE Stack*/
  ret = hci_reset();
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : hci_reset command, result: 0x%x \n", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: hci_reset command\n");
  }

  /**
   * Write the BD Address
   */
  p_bd_addr = BleGetBdAddress();
  ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET, CONFIG_DATA_PUBADDR_LEN, (uint8_t*) p_bd_addr);
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_hal_write_config_data command - CONFIG_DATA_PUBADDR_OFFSET, result: 0x%x \n", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_hal_write_config_data command - CONFIG_DATA_PUBADDR_OFFSET\n");
    APP_DBG_MSG("  Public Bluetooth Address: %02x:%02x:%02x:%02x:%02x:%02x\n",p_bd_addr[5],p_bd_addr[4],p_bd_addr[3],p_bd_addr[2],p_bd_addr[1],p_bd_addr[0]);
  }

#if (CFG_BLE_ADDRESS_TYPE == GAP_PUBLIC_ADDR)
  /* BLE MAC in ADV Packet */
  a_ManufData[ sizeof(a_ManufData)-6] = p_bd_addr[5];
  a_ManufData[ sizeof(a_ManufData)-5] = p_bd_addr[4];
  a_ManufData[ sizeof(a_ManufData)-4] = p_bd_addr[3];
  a_ManufData[ sizeof(a_ManufData)-3] = p_bd_addr[2];
  a_ManufData[ sizeof(a_ManufData)-2] = p_bd_addr[1];
  a_ManufData[ sizeof(a_ManufData)-1] = p_bd_addr[0];
#endif /* CFG_BLE_ADDRESS_TYPE == GAP_PUBLIC_ADDR */

  /**
   * Static random Address
   * The two upper bits shall be set to 1
   * The lowest 32bits is read from the UDN to differentiate between devices
   * The RNG may be used to provide a random number on each power on
   */
  srd_bd_addr[1] =  0x0000ED6E;
  srd_bd_addr[0] =  LL_FLASH_GetUDN( );
  aci_hal_write_config_data( CONFIG_DATA_RANDOM_ADDRESS_OFFSET, CONFIG_DATA_RANDOM_ADDRESS_LEN, (uint8_t*)srd_bd_addr );


  /**
   * Static random Address
   * The two upper bits shall be set to 1
   * The lowest 32bits is read from the UDN to differentiate between devices
   * The RNG may be used to provide a random number on each power on
   */
#if (CFG_IDENTITY_ADDRESS == GAP_STATIC_RANDOM_ADDR)
#if defined(CFG_STATIC_RANDOM_ADDRESS)
  a_srd_bd_addr[0] = CFG_STATIC_RANDOM_ADDRESS & 0xFFFFFFFF;
  a_srd_bd_addr[1] = (uint32_t)((uint64_t)CFG_STATIC_RANDOM_ADDRESS >> 32);
  a_srd_bd_addr[1] |= 0xC000; /* The two upper bits shall be set to 1 */
#else
  /* Get RNG semaphore */
  while(LL_HSEM_1StepLock(HSEM, CFG_HW_RNG_SEMID));

  /* Enable RNG */
  __HAL_RNG_ENABLE(&hrng);

  /* Enable HSI48 oscillator */
  LL_RCC_HSI48_Enable();
  /* Wait until HSI48 is ready */
  while(! LL_RCC_HSI48_IsReady());

  if (HAL_RNG_GenerateRandomNumber(&hrng, &a_srd_bd_addr[1]) != HAL_OK)
  {
    /* Random number generation error */
    Error_Handler();
  }
  if (HAL_RNG_GenerateRandomNumber(&hrng, &a_srd_bd_addr[0]) != HAL_OK)
  {
    /* Random number generation error */
    Error_Handler();
  }
  a_srd_bd_addr[1] |= 0xC000; /* The two upper bits shall be set to 1 */

  /* Disable HSI48 oscillator */
  LL_RCC_HSI48_Disable();

  /* Disable RNG */
  __HAL_RNG_DISABLE(&hrng);

  /* Release RNG semaphore */
  LL_HSEM_ReleaseLock(HSEM, CFG_HW_RNG_SEMID, 0);
#endif /* CFG_STATIC_RANDOM_ADDRESS */
#endif

#if (CFG_BLE_ADDRESS_TYPE != GAP_PUBLIC_ADDR)
  /* BLE MAC in ADV Packet */
  a_ManufData[ sizeof(a_ManufData)-6] = a_srd_bd_addr[1] >> 8 ;
  a_ManufData[ sizeof(a_ManufData)-5] = a_srd_bd_addr[1];
  a_ManufData[ sizeof(a_ManufData)-4] = a_srd_bd_addr[0] >> 24;
  a_ManufData[ sizeof(a_ManufData)-3] = a_srd_bd_addr[0] >> 16;
  a_ManufData[ sizeof(a_ManufData)-2] = a_srd_bd_addr[0] >> 8;
  a_ManufData[ sizeof(a_ManufData)-1] = a_srd_bd_addr[0];

  ret = aci_hal_write_config_data(CONFIG_DATA_RANDOM_ADDRESS_OFFSET, CONFIG_DATA_RANDOM_ADDRESS_LEN, (uint8_t*)a_srd_bd_addr);
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_hal_write_config_data command - CONFIG_DATA_RANDOM_ADDRESS_OFFSET, result: 0x%x \n", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_hal_write_config_data command - CONFIG_DATA_RANDOM_ADDRESS_OFFSET\n");
    APP_DBG_MSG("  Random Bluetooth Address: %02x:%02x:%02x:%02x:%02x:%02x\n", (uint8_t)(a_srd_bd_addr[1] >> 8),
                                                                               (uint8_t)(a_srd_bd_addr[1]),
                                                                               (uint8_t)(a_srd_bd_addr[0] >> 24),
                                                                               (uint8_t)(a_srd_bd_addr[0] >> 16),
                                                                               (uint8_t)(a_srd_bd_addr[0] >> 8),
                                                                               (uint8_t)(a_srd_bd_addr[0]));
  }
#endif /* CFG_BLE_ADDRESS_TYPE != GAP_PUBLIC_ADDR */

  /**
   * Write Identity root key used to derive IRK and DHK(Legacy)
   */
  ret = aci_hal_write_config_data(CONFIG_DATA_IR_OFFSET, CONFIG_DATA_IR_LEN, (uint8_t*)a_BLE_CfgIrValue);
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_hal_write_config_data command - CONFIG_DATA_IR_OFFSET, result: 0x%x \n", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_hal_write_config_data command - CONFIG_DATA_IR_OFFSET\n");
  }

  /**
   * Write Encryption root key used to derive LTK and CSRK
   */
  ret = aci_hal_write_config_data(CONFIG_DATA_ER_OFFSET, CONFIG_DATA_ER_LEN, (uint8_t*)a_BLE_CfgErValue);
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_hal_write_config_data command - CONFIG_DATA_ER_OFFSET, result: 0x%x \n", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_hal_write_config_data command - CONFIG_DATA_ER_OFFSET\n");
  }

  /**
   * Set TX Power.
   */
  ret = aci_hal_set_tx_power_level(1, CFG_TX_POWER);
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_hal_set_tx_power_level command, result: 0x%x \n", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_hal_set_tx_power_level command\n");
  }

  /**
   * Initialize GATT interface
   */
  ret = aci_gatt_init();
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_init command, result: 0x%x \n", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_init command\n");
  }

  /**
   * Initialize GAP interface
   */
  role = 0;

#if (BLE_CFG_PERIPHERAL == 1)
  role |= GAP_PERIPHERAL_ROLE;
#endif /* BLE_CFG_PERIPHERAL == 1 */

#if (BLE_CFG_CENTRAL == 1)
  role |= GAP_CENTRAL_ROLE;
#endif /* BLE_CFG_CENTRAL == 1 */

/* USER CODE BEGIN Role_Mngt*/
  uint32_t UID = LL_FLASH_GetUDN();

    const char local_name_buzzcam[] = { AD_TYPE_COMPLETE_LOCAL_NAME, 'B','u','z','z','C','a','m','_',
  		  hexToAscii(UID >> 28),
  		  hexToAscii(UID >> 24),
  		  hexToAscii(UID >> 20),
  		  hexToAscii(UID >> 16),
  		  hexToAscii(UID >> 12),
  		  hexToAscii(UID >> 8),
  		  hexToAscii(UID >> 4),
  		  hexToAscii(UID)};
    memcpy(a_buzzCamName,local_name_buzzcam,sizeof(local_name_buzzcam));

    const char local_name[] = { AD_TYPE_COMPLETE_LOCAL_NAME,'I','n','s','t','a','3','6','0',' ','G','P','S',' ','R','e','m','o','t','e'};
    memcpy(a_camName,local_name,sizeof(local_name));

//    		'_',
//    		  hexToAscii(UID >> 28),
//			  hexToAscii(UID >> 24),
//			  hexToAscii(UID >> 20),
//			  hexToAscii(UID >> 16),
//			  hexToAscii(UID >> 12),
//			  hexToAscii(UID >> 8),
//			  hexToAscii(UID >> 4),
//			  hexToAscii(UID)};

    memcpy(a_LocalName,local_name_buzzcam,sizeof(local_name_buzzcam));

/* USER CODE END Role_Mngt */

  if (role > 0)
  {
    const char *name = "BUZZC";
    ret = aci_gap_init(role,
                       CFG_PRIVACY,
                       APPBLE_GAP_DEVICE_NAME_LENGTH,
                       &gap_service_handle,
                       &gap_dev_name_char_handle,
                       &gap_appearance_char_handle);

    if (ret != BLE_STATUS_SUCCESS)
    {
      APP_DBG_MSG("  Fail   : aci_gap_init command, result: 0x%x \n", ret);
    }
    else
    {
      APP_DBG_MSG("  Success: aci_gap_init command\n");
    }

    ret = aci_gatt_update_char_value(gap_service_handle, gap_dev_name_char_handle, 0, strlen(name), (uint8_t *) name);
    if (ret != BLE_STATUS_SUCCESS)
    {
      BLE_DBG_SVCCTL_MSG("  Fail   : aci_gatt_update_char_value - Device Name\n");
    }
    else
    {
      BLE_DBG_SVCCTL_MSG("  Success: aci_gatt_update_char_value - Device Name\n");
    }
  }

  ret = aci_gatt_update_char_value(gap_service_handle,
                                   gap_appearance_char_handle,
                                   0,
                                   2,
                                   (uint8_t *)&a_appearance);
  if (ret != BLE_STATUS_SUCCESS)
  {
    BLE_DBG_SVCCTL_MSG("  Fail   : aci_gatt_update_char_value - Appearance\n");
  }
  else
  {
    BLE_DBG_SVCCTL_MSG("  Success: aci_gatt_update_char_value - Appearance\n");
  }

  /**
   * Initialize Default PHY
   */
  ret = hci_le_set_default_phy(ALL_PHYS_PREFERENCE,TX_2M_PREFERRED,RX_2M_PREFERRED);
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : hci_le_set_default_phy command, result: 0x%x \n", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: hci_le_set_default_phy command\n");
  }

  /**
   * Initialize IO capability
   */
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.ioCapability = CFG_IO_CAPABILITY;
  ret = aci_gap_set_io_capability(BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.ioCapability);
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gap_set_io_capability command, result: 0x%x \n", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gap_set_io_capability command\n");
  }

  /**
   * Initialize authentication
   */
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.mitm_mode = CFG_MITM_PROTECTION;
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.encryptionKeySizeMin = CFG_ENCRYPTION_KEY_SIZE_MIN;
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.encryptionKeySizeMax = CFG_ENCRYPTION_KEY_SIZE_MAX;
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.Use_Fixed_Pin = CFG_USED_FIXED_PIN;
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.Fixed_Pin = CFG_FIXED_PIN;
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.bonding_mode = CFG_BONDING_MODE;
  /* USER CODE BEGIN Ble_Hci_Gap_Gatt_Init_1*/

  /* USER CODE END Ble_Hci_Gap_Gatt_Init_1*/

  ret = aci_gap_set_authentication_requirement(BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.bonding_mode,
                                               BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.mitm_mode,
                                               CFG_SC_SUPPORT,
                                               CFG_KEYPRESS_NOTIFICATION_SUPPORT,
                                               BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.encryptionKeySizeMin,
                                               BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.encryptionKeySizeMax,
                                               BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.Use_Fixed_Pin,
                                               BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.Fixed_Pin,
                                               CFG_IDENTITY_ADDRESS);
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gap_set_authentication_requirement command, result: 0x%x \n", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gap_set_authentication_requirement command\n");
  }

  /**
   * Initialize whitelist
   */
  if (BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.bonding_mode)
  {
    ret = aci_gap_configure_whitelist();
    if (ret != BLE_STATUS_SUCCESS)
    {
      APP_DBG_MSG("  Fail   : aci_gap_configure_whitelist command, result: 0x%x \n", ret);
    }
    else
    {
      APP_DBG_MSG("  Success: aci_gap_configure_whitelist command\n");
    }
  }
  APP_DBG_MSG("==>> End Ble_Hci_Gap_Gatt_Init function\n\r");
}

void Adv_Request(APP_BLE_ConnStatus_t NewStatus)
{
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;
  uint16_t Min_Inter, Max_Inter;

  if (NewStatus == APP_BLE_FAST_ADV)
  {
    Min_Inter = AdvIntervalMin;
    Max_Inter = AdvIntervalMax;
  }
  else
  {
    Min_Inter = CFG_LP_CONN_ADV_INTERVAL_MIN;
    Max_Inter = CFG_LP_CONN_ADV_INTERVAL_MAX;
  }

  /**
   * Stop the timer, it will be restarted for a new shot
   * It does not hurt if the timer was not running
   */
  HW_TS_Stop(BleApplicationContext.Advertising_mgr_timer_Id);

  if ((NewStatus == APP_BLE_LP_ADV)
      && ((BleApplicationContext.Device_Connection_Status == APP_BLE_FAST_ADV)
          || (BleApplicationContext.Device_Connection_Status == APP_BLE_LP_ADV)))
  {
    /* Connection in ADVERTISE mode have to stop the current advertising */
    ret = aci_gap_set_non_discoverable();
    if (ret != BLE_STATUS_SUCCESS)
    {
      APP_DBG_MSG("==>> aci_gap_set_non_discoverable - Stop Advertising Failed , result: %d \n", ret);
    }
    else
    {
      APP_DBG_MSG("==>> aci_gap_set_non_discoverable - Successfully Stopped Advertising \n");
    }
  }

  BleApplicationContext.Device_Connection_Status = NewStatus;
  /* Start Fast or Low Power Advertising */
  ret = aci_gap_set_discoverable(ADV_IND,
                                 Min_Inter,
                                 Max_Inter,
                                 CFG_BLE_ADDRESS_TYPE,
                                 NO_WHITE_LIST_USE, /* use white list */
                                 sizeof(a_LocalName),
                                 (uint8_t*) &a_LocalName,
                                 BleApplicationContext.BleApplicationContext_legacy.advtServUUIDlen,
                                 BleApplicationContext.BleApplicationContext_legacy.advtServUUID,
                                 0,
                                 0);
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("==>> aci_gap_set_discoverable - fail, result: 0x%x \n", ret);
  }
  else
  {
    APP_DBG_MSG("==>> aci_gap_set_discoverable - Success\n");
  }

  /* Update Advertising data */
  ret = aci_gap_update_adv_data(sizeof(a_ManufData), (uint8_t*) a_ManufData);
  if (ret != BLE_STATUS_SUCCESS)
  {
    if (NewStatus == APP_BLE_FAST_ADV)
    {
      APP_DBG_MSG("==>> Start Fast Advertising Failed , result: %d \n\r", ret);
    }
    else
    {
      APP_DBG_MSG("==>> Start Low Power Advertising Failed , result: %d \n\r", ret);
    }
  }
  else
  {
    if (NewStatus == APP_BLE_FAST_ADV)
    {
      APP_DBG_MSG("==>> Success: Start Fast Advertising \n\r");
      /* Start Timer to STOP ADV - TIMEOUT - and next Restart Low Power Advertising */
      HW_TS_Start(BleApplicationContext.Advertising_mgr_timer_Id, INITIAL_ADV_TIMEOUT);
    }
    else
    {
      APP_DBG_MSG("==>> Success: Start Low Power Advertising \n\r");
    }
  }

  return;
}

const uint8_t* BleGetBdAddress(void)
{
  uint8_t *p_otp_addr;
  const uint8_t *p_bd_addr;
  uint32_t udn;
  uint32_t company_id;
  uint32_t device_id;

  udn = LL_FLASH_GetUDN();

  if (udn != 0xFFFFFFFF)
  {
    company_id = LL_FLASH_GetSTCompanyID();
    device_id = LL_FLASH_GetDeviceID();

    /**
     * Public Address with the ST company ID
     * bit[47:24] : 24bits (OUI) equal to the company ID
     * bit[23:16] : Device ID.
     * bit[15:0] : The last 16bits from the UDN
     * Note: In order to use the Public Address in a final product, a dedicated
     * 24bits company ID (OUI) shall be bought.
     */
    a_BdAddrUdn[0] = (uint8_t)(udn & 0x000000FF);
    a_BdAddrUdn[1] = (uint8_t)((udn & 0x0000FF00) >> 8);
    a_BdAddrUdn[2] = (uint8_t)device_id;
    a_BdAddrUdn[3] = (uint8_t)(company_id & 0x000000FF);
    a_BdAddrUdn[4] = (uint8_t)((company_id & 0x0000FF00) >> 8);
    a_BdAddrUdn[5] = (uint8_t)((company_id & 0x00FF0000) >> 16);

    p_bd_addr = (const uint8_t *)a_BdAddrUdn;
  }
  else
  {
    p_otp_addr = OTP_Read(0);
    if (p_otp_addr)
    {
      p_bd_addr = ((OTP_ID0_t*)p_otp_addr)->bd_address;
    }
    else
    {
      p_bd_addr = a_MBdAddr;
    }
  }

  return p_bd_addr;
}

/* USER CODE BEGIN FD_LOCAL_FUNCTION */

/* USER CODE END FD_LOCAL_FUNCTION */

/*************************************************************
 *
 *SPECIFIC FUNCTIONS
 *
 *************************************************************/
static void Add_Advertisment_Service_UUID(uint16_t servUUID)
{
  BleApplicationContext.BleApplicationContext_legacy.advtServUUID[BleApplicationContext.BleApplicationContext_legacy.advtServUUIDlen] =
      (uint8_t) (servUUID & 0xFF);
  BleApplicationContext.BleApplicationContext_legacy.advtServUUIDlen++;
  BleApplicationContext.BleApplicationContext_legacy.advtServUUID[BleApplicationContext.BleApplicationContext_legacy.advtServUUIDlen] =
      (uint8_t) (servUUID >> 8) & 0xFF;
  BleApplicationContext.BleApplicationContext_legacy.advtServUUIDlen++;

  return;
}

static void Adv_Mgr(void)
{
  /**
   * The code shall be executed in the background as an aci command may be sent
   * The background is the only place where the application can make sure a new aci command
   * is not sent if there is a pending one
   */
  osThreadFlagsSet(AdvUpdateProcessId, 1);

  return;
}

static void AdvUpdateProcess(void *argument)
{
  UNUSED(argument);

  for(;;)
  {
    osThreadFlagsWait(1, osFlagsWaitAny, osWaitForever);
    Adv_Update();
  }
}

static void Adv_Update(void)
{
  Adv_Request(APP_BLE_LP_ADV);

  return;
}

static void HciUserEvtProcess(void *argument)
{
  UNUSED(argument);

  for(;;)
  {
    osThreadFlagsWait(1, osFlagsWaitAny, osWaitForever);
    hci_user_evt_proc();
  }
}

/* USER CODE BEGIN FD_SPECIFIC_FUNCTIONS */

/* USER CODE END FD_SPECIFIC_FUNCTIONS */
/*************************************************************
 *
 * WRAP FUNCTIONS
 *
 *************************************************************/
void hci_notify_asynch_evt(void* p_Data)
{
  UNUSED(p_Data);
  osThreadFlagsSet(HciUserEvtProcessId, 1);

  return;
}

void hci_cmd_resp_release(uint32_t Flag)
{
  UNUSED(Flag);
  osSemaphoreRelease(SemHciId);

  return;
}

void hci_cmd_resp_wait(uint32_t Timeout)
{
  UNUSED(Timeout);
  osSemaphoreAcquire(SemHciId, osWaitForever);

  return;
}

static void BLE_UserEvtRx(void *p_Payload)
{
  SVCCTL_UserEvtFlowStatus_t svctl_return_status;
  tHCI_UserEvtRxParam *p_param;

  p_param = (tHCI_UserEvtRxParam *)p_Payload;

  svctl_return_status = SVCCTL_UserEvtRx((void *)&(p_param->pckt->evtserial));
  if (svctl_return_status != SVCCTL_UserEvtFlowDisable)
  {
    p_param->status = HCI_TL_UserEventFlow_Enable;
  }
  else
  {
    p_param->status = HCI_TL_UserEventFlow_Disable;
  }

  return;
}

static void BLE_StatusNot(HCI_TL_CmdStatus_t Status)
{
  switch (Status)
  {
    case HCI_TL_CmdBusy:
      osMutexAcquire(MtxHciId, osWaitForever);
      /* USER CODE BEGIN HCI_TL_CmdBusy */

      /* USER CODE END HCI_TL_CmdBusy */
      break;

    case HCI_TL_CmdAvailable:
      osMutexRelease(MtxHciId);
      /* USER CODE BEGIN HCI_TL_CmdAvailable */

      /* USER CODE END HCI_TL_CmdAvailable */
      break;

    default:
      /* USER CODE BEGIN Status */

      /* USER CODE END Status */
      break;
  }

  return;
}

void SVCCTL_ResumeUserEventFlow(void)
{
  hci_resume_flow();

  return;
}

/* USER CODE BEGIN FD_WRAP_FUNCTIONS */
char hexToAscii(uint8_t val){
	// only look at first 4 bits
	val = val & (0x0F);
	if(val<10) return val+48;
	else return val+87;
}
/* USER CODE END FD_WRAP_FUNCTIONS */
