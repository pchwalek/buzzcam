/* USER CODE BEGIN Header */
/**
 ******************************************************************************
  * File Name          : App/app_thread.c
  * Description        : Thread Application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "app_common.h"
#include "utilities_common.h"
#include "app_entry.h"
#include "dbg_trace.h"
#include "app_thread.h"
#include "stm32wbxx_core_interface_def.h"
#include "openthread_api_wb.h"
#include "shci.h"
#include "stm_logging.h"
#include "app_conf.h"
#include "stm32_lpm.h"
#include "cmsis_os.h"
#if (CFG_USB_INTERFACE_ENABLE != 0)
#include "vcp.h"
#include "vcp_conf.h"
#endif /* (CFG_USB_INTERFACE_ENABLE != 0) */

/* Private includes -----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "coap.h"
#include "main.h"
#include "queue.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private defines -----------------------------------------------------------*/
#define C_SIZE_CMD_STRING       256U
#define C_PANID                 0x1234U
#define C_CHANNEL_NB            23U

/* FreeRtos stacks attributes */
const osThreadAttr_t ThreadMsgM0ToM4Process_attr = {
    .name = CFG_THREAD_MSG_M0_TO_M4_PROCESS_NAME,
    .attr_bits = CFG_THREAD_MSG_M0_TO_M4_PROCESS_ATTR_BITS,
    .cb_mem = CFG_THREAD_MSG_M0_TO_M4_PROCESS_CB_MEM,
    .cb_size = CFG_THREAD_MSG_M0_TO_M4_PROCESS_CB_SIZE,
    .stack_mem = CFG_THREAD_MSG_M0_TO_M4_PROCESS_STACK_MEM,
    .priority = CFG_THREAD_MSG_M0_TO_M4_PROCESS_PRIORITY,
    .stack_size = CFG_THREAD_MSG_M0_TO_M4_PROCESS_STACK_SIZE
};

const osThreadAttr_t ThreadCliProcess_attr = {
     .name = CFG_THREAD_CLI_PROCESS_NAME,
     .attr_bits = CFG_THREAD_CLI_PROCESS_ATTR_BITS,
     .cb_mem = CFG_THREAD_CLI_PROCESS_CB_MEM,
     .cb_size = CFG_THREAD_CLI_PROCESS_CB_SIZE,
     .stack_mem = CFG_THREAD_CLI_PROCESS_STACK_MEM,
     .priority = CFG_THREAD_CLI_PROCESS_PRIORITY,
     .stack_size = CFG_THREAD_CLI_PROCESS_STACK_SIZE
 };

/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macros ------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define C_RESSOURCE             "buzz"

#define COAP_SEND_TIMEOUT               (5*100*1000/CFG_TS_TICK_VAL) /**< 1s */
#define THREAD_CHANGE_MODE_TIMEOUT      (1*1000*1000/CFG_TS_TICK_VAL) /**< 1s */
#define THREAD_LINK_POLL_PERIOD         (5*1000*1000/CFG_TS_TICK_VAL) /**< 5s */

#define OT_AUTOSTART_MODE 1 // Automatic OT start and COAP after reset
							// ste to 0 for GRL testing

static QueueHandle_t MoNotifQueue;

/* USER CODE END PM */

/* Private function prototypes -----------------------------------------------*/
static void APP_THREAD_CheckWirelessFirmwareInfo(void);
static void APP_THREAD_DeviceConfig(void);
static void APP_THREAD_StateNotif(uint32_t NotifFlags, void *pContext);
static void APP_THREAD_TraceError(const char * pMess, uint32_t ErrCode);
#if (CFG_FULL_LOW_POWER == 0)
static void Send_CLI_To_M0(void);
#endif /* (CFG_FULL_LOW_POWER == 0) */
static void Send_CLI_Ack_For_OT(void);
static void HostTxCb( void );
static void Wait_Getting_Ack_From_M0(void);
static void Receive_Ack_From_M0(void);
static void Receive_Notification_From_M0(void);
#if (CFG_HW_LPUART1_ENABLED == 1)
extern void MX_LPUART1_UART_Init(void);
#endif
#if (CFG_HW_USART1_ENABLED == 1)
extern void MX_USART1_UART_Init(void);
#endif
#if (CFG_USB_INTERFACE_ENABLE != 0)
static uint32_t ProcessCmdString(uint8_t* buf , uint32_t len);
#else
#if (CFG_FULL_LOW_POWER == 0)
static void RxCpltCallback(void);
#endif /* (CFG_FULL_LOW_POWER == 0) */
#endif /* (CFG_USB_INTERFACE_ENABLE != 0) */

/* FreeRTos wrapper functions */
static void APP_THREAD_FreeRTOSProcessMsgM0ToM4Task(void *argument);
#if (CFG_FULL_LOW_POWER == 0)
static void APP_THREAD_FreeRTOSSendCLIToM0Task(void *argument);
#endif /* (CFG_FULL_LOW_POWER == 0) */

/* USER CODE BEGIN PFP */
static void APP_THREAD_SendCoapMsg(void);
static void APP_THREAD_SendCoapMulticastRequest(uint8_t command);
static void APP_THREAD_CoapRequestHandler(void                * pContext,
                                          otMessage           * pMessage,
                                          const otMessageInfo * pMessageInfo);
static void APP_THREAD_SetSleepyEndDeviceMode(void);
static void APP_THREAD_CoapTimingElapsed(void);
//static void ot_StatusNot( ot_TL_CmdStatus_t status );
osMutexId_t MtxOtCmdId;
osMutexId_t MtxOtAckId;

static void ot_Ack( ot_TL_CmdStatus_t status );

void Ot_Cmd_TransferWithNotif(void);
/* USER CODE END PFP */

/* Private variables -----------------------------------------------*/
#if (CFG_USB_INTERFACE_ENABLE != 0)
static uint8_t TmpString[C_SIZE_CMD_STRING];
static uint8_t VcpRxBuffer[sizeof(TL_CmdSerial_t)];        /* Received Data over USB are stored in this buffer */
static uint8_t VcpTxBuffer[sizeof(TL_EvtPacket_t) + 254U]; /* Transmit buffer over USB */
#else
#if (CFG_FULL_LOW_POWER == 0)
static uint8_t aRxBuffer[C_SIZE_CMD_STRING];
#endif /* (CFG_FULL_LOW_POWER == 0) */
#endif /* (CFG_USB_INTERFACE_ENABLE != 0) */

#if (CFG_FULL_LOW_POWER == 0)
static uint8_t CommandString[C_SIZE_CMD_STRING];
#endif /* (CFG_FULL_LOW_POWER == 0) */
static __IO uint16_t indexReceiveChar = 0;
static __IO uint16_t CptReceiveCmdFromUser = 0;

static TL_CmdPacket_t *p_thread_otcmdbuffer;
static TL_EvtPacket_t *p_thread_notif_M0_to_M4;
static __IO uint32_t  CptReceiveMsgFromM0 = 0;
static volatile int FlagReceiveAckFromM0 = 0;


PLACE_IN_SECTION("MB_MEM1") ALIGN(4) static TL_TH_Config_t ThreadConfigBuffer;
PLACE_IN_SECTION("MB_MEM2") ALIGN(4) static TL_CmdPacket_t ThreadOtCmdBuffer;
PLACE_IN_SECTION("MB_MEM2") ALIGN(4) static uint8_t ThreadNotifRspEvtBuffer[sizeof(TL_PacketHeader_t) + TL_EVT_HDR_SIZE + 255U];
PLACE_IN_SECTION("MB_MEM2") ALIGN(4) static TL_CmdPacket_t ThreadCliCmdBuffer;
PLACE_IN_SECTION("MB_MEM2") ALIGN(4) static TL_CmdPacket_t ThreadCliNotBuffer;

osThreadId_t OsTaskMsgM0ToM4Id;      /* Task managing the M0 to M4 messaging        */
#if (CFG_FULL_LOW_POWER == 0)
static osThreadId_t OsTaskCliId;            /* Task used to manage CLI comamnd             */
#endif /* (CFG_FULL_LOW_POWER == 0) */

/* USER CODE BEGIN PV */
static otCoapResource OT_Ressource = {C_RESSOURCE, APP_THREAD_CoapRequestHandler,"buzz", NULL};
static otMessageInfo OT_MessageInfo = {0};
static uint8_t OT_Command = 0;
static uint8_t OT_ReceivedCommand = 0;
static otMessage   * pOT_Message = NULL;
static otLinkModeConfig OT_LinkMode = {0};
static uint32_t sleepyEndDeviceFlag = FALSE;
static uint8_t sedCoapTimerID;
static uint8_t setThreadModeTimerID;
static void APP_THREAD_CoapSendDataResponse(otMessage    * pMessage,
    const otMessageInfo * pMessageInfo);

/* Debug */
static uint32_t DebugRxCoapCpt = 0;
static uint32_t DebugTxCoapCpt = 0;

static uint8_t PayloadWrite[30]= {0};
static uint8_t PayloadRead[30]= {0};

static volatile uint8_t waitingForAck = 0;

extern volatile uint8_t g_ot_notification_allowed;

/* USER CODE END PV */

/* Functions Definition ------------------------------------------------------*/

//void APP_THREAD_Init( void )
//{
//  /* USER CODE BEGIN APP_THREAD_INIT_1 */
//
//  /* USER CODE END APP_THREAD_INIT_1 */
//
//  SHCI_CmdStatus_t ThreadInitStatus;
//
//  /* Check the compatibility with the Coprocessor Wireless Firmware loaded */
//  APP_THREAD_CheckWirelessFirmwareInfo();
//
//#if (CFG_USB_INTERFACE_ENABLE != 0)
//  VCP_Init(&VcpTxBuffer[0], &VcpRxBuffer[0]);
//#endif /* (CFG_USB_INTERFACE_ENABLE != 0) */
//
//  /* Register cmdbuffer */
//  APP_THREAD_RegisterCmdBuffer(&ThreadOtCmdBuffer);
//
//  /**
//   * Do not allow standby in the application
//   */
//  UTIL_LPM_SetOffMode(1 << CFG_LPM_APP_THREAD, UTIL_LPM_DISABLE);
//
//  /* Init config buffer and call TL_THREAD_Init */
//  APP_THREAD_TL_THREAD_INIT();
//
//  /* Configure UART for sending CLI command from M4 */
////  APP_THREAD_Init_UART_CLI();
//
//  /* Send Thread start system cmd to M0 */
//  ThreadInitStatus = SHCI_C2_THREAD_Init();
//
//  /* Prevent unused argument(s) compilation warning */
//  UNUSED(ThreadInitStatus);
//
//  /* USER CODE BEGIN APP_THREAD_INIT_TIMER */
//
//  /* USER CODE END APP_THREAD_INIT_TIMER */
//
//  /* Create the different FreeRTOS tasks requested to run this Thread application*/
//  OsTaskMsgM0ToM4Id = osThreadNew(APP_THREAD_FreeRTOSProcessMsgM0ToM4Task, NULL,&ThreadMsgM0ToM4Process_attr);
//
//  /* USER CODE BEGIN APP_THREAD_INIT_FREERTOS */
//
//  /* USER CODE END APP_THREAD_INIT_FREERTOS */
//
//  /* Configure the Thread device at start */
//  APP_THREAD_DeviceConfig();
//
//  /* USER CODE BEGIN APP_THREAD_INIT_2 */
//
//  /* USER CODE END APP_THREAD_INIT_2 */
//}

/**
  * @brief  Trace the error or the warning reported.
  * @param  ErrId :
  * @param  ErrCode
  * @retval None
  */
void APP_THREAD_Error(uint32_t ErrId, uint32_t ErrCode)
{
  /* USER CODE BEGIN APP_THREAD_Error_1 */

  /* USER CODE END APP_THREAD_Error_1 */
  switch(ErrId)
  {
  case ERR_REC_MULTI_MSG_FROM_M0 :
    APP_THREAD_TraceError("ERROR : ERR_REC_MULTI_MSG_FROM_M0 ", ErrCode);
    break;
  case ERR_THREAD_SET_STATE_CB :
    APP_THREAD_TraceError("ERROR : ERR_THREAD_SET_STATE_CB ",ErrCode);
    break;
  case ERR_THREAD_SET_CHANNEL :
    APP_THREAD_TraceError("ERROR : ERR_THREAD_SET_CHANNEL ",ErrCode);
    break;
  case ERR_THREAD_SET_PANID :
    APP_THREAD_TraceError("ERROR : ERR_THREAD_SET_PANID ",ErrCode);
    break;
  case ERR_THREAD_IPV6_ENABLE :
    APP_THREAD_TraceError("ERROR : ERR_THREAD_IPV6_ENABLE ",ErrCode);
    break;
  case ERR_THREAD_START :
    APP_THREAD_TraceError("ERROR: ERR_THREAD_START ", ErrCode);
    break;
  case ERR_THREAD_ERASE_PERSISTENT_INFO :
    APP_THREAD_TraceError("ERROR : ERR_THREAD_ERASE_PERSISTENT_INFO ",ErrCode);
    break;
  case ERR_THREAD_CHECK_WIRELESS :
    APP_THREAD_TraceError("ERROR : ERR_THREAD_CHECK_WIRELESS ",ErrCode);
    break;
  /* USER CODE BEGIN APP_THREAD_Error_2 */

  /* USER CODE END APP_THREAD_Error_2 */
  default :
    APP_THREAD_TraceError("ERROR Unknown ", 0);
    break;
  }

}

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/

/**
 * @brief Thread initialization.
 * @param  None
 * @retval None
 */
static void APP_THREAD_DeviceConfig(void)
{
  otError error;
  otNetworkKey networkKey = {{0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF}};

//  error = otInstanceErasePersistentInfo(NULL);
//  if (error != OT_ERROR_NONE)
//  {
//    APP_THREAD_Error(ERR_THREAD_ERASE_PERSISTENT_INFO,error);
//  }
  otInstanceFinalize(NULL);
  otInstanceInitSingle();
  error = otSetStateChangedCallback(NULL, APP_THREAD_StateNotif, NULL);
  if (error != OT_ERROR_NONE)
  {
    APP_THREAD_Error(ERR_THREAD_SET_STATE_CB,error);
  }
  error = otLinkSetChannel(NULL, C_CHANNEL_NB);
  if (error != OT_ERROR_NONE)
  {
    APP_THREAD_Error(ERR_THREAD_SET_CHANNEL,error);
  }
  error = otLinkSetPanId(NULL, C_PANID);
  if (error != OT_ERROR_NONE)
  {
    APP_THREAD_Error(ERR_THREAD_SET_PANID,error);
  }
  error = otThreadSetNetworkKey(NULL, &networkKey);
  if (error != OT_ERROR_NONE)
  {
    APP_THREAD_Error(ERR_THREAD_SET_NETWORK_KEY,error);
  }
  error = otIp6SetEnabled(NULL, true);
  if (error != OT_ERROR_NONE)
  {
    APP_THREAD_Error(ERR_THREAD_IPV6_ENABLE,error);
  }
  error = otThreadSetEnabled(NULL, true);
  if (error != OT_ERROR_NONE)
  {
    APP_THREAD_Error(ERR_THREAD_START,error);
  }

  /* USER CODE BEGIN DEVICECONFIG */
  error = otCoapStart(NULL, OT_DEFAULT_COAP_PORT);
  if (error != OT_ERROR_NONE)
  {
    APP_THREAD_Error(ERR_THREAD_COAP_START,error);
  }
  /* Add COAP resources */
  otCoapAddResource(NULL, &OT_Ressource);
  /* USER CODE END DEVICECONFIG */
}

/**
 * @brief Thread notification when the state changes.
 * @param  aFlags  : Define the item that has been modified
 *         aContext: Context
 *
 * @retval None
 */
static void APP_THREAD_StateNotif(uint32_t NotifFlags, void *pContext)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(pContext);

  /* USER CODE BEGIN APP_THREAD_STATENOTIF */

  /* USER CODE END APP_THREAD_STATENOTIF */

  if ((NotifFlags & (uint32_t)OT_CHANGED_THREAD_ROLE) == (uint32_t)OT_CHANGED_THREAD_ROLE)
  {
    switch (otThreadGetDeviceRole(NULL))
    {
    case OT_DEVICE_ROLE_DISABLED:
      /* USER CODE BEGIN OT_DEVICE_ROLE_DISABLED */

      /* USER CODE END OT_DEVICE_ROLE_DISABLED */
      break;
    case OT_DEVICE_ROLE_DETACHED:
      /* USER CODE BEGIN OT_DEVICE_ROLE_DETACHED */

      /* USER CODE END OT_DEVICE_ROLE_DETACHED */
      break;
    case OT_DEVICE_ROLE_CHILD:
      /* USER CODE BEGIN OT_DEVICE_ROLE_CHILD */

      /* USER CODE END OT_DEVICE_ROLE_CHILD */
      break;
    case OT_DEVICE_ROLE_ROUTER :
      /* USER CODE BEGIN OT_DEVICE_ROLE_ROUTER */

      /* USER CODE END OT_DEVICE_ROLE_ROUTER */
      break;
    case OT_DEVICE_ROLE_LEADER :
      /* USER CODE BEGIN OT_DEVICE_ROLE_LEADER */

      /* USER CODE END OT_DEVICE_ROLE_LEADER */
      break;
    default:
      /* USER CODE BEGIN DEFAULT */

      /* USER CODE END DEFAULT */
      break;
    }
  }
}

/**
  * @brief  Warn the user that an error has occurred.In this case,
  *         the LEDs on the Board will start blinking.
  *
  * @param  pMess  : Message associated to the error.
  * @param  ErrCode: Error code associated to the module (OpenThread or other module if any)
  * @retval None
  */
static void APP_THREAD_TraceError(const char * pMess, uint32_t ErrCode)
{
  /* USER CODE BEGIN TRACE_ERROR */

  /* USER CODE END TRACE_ERROR */
}

/**
 * @brief Check if the Coprocessor Wireless Firmware loaded supports Thread
 *        and display associated informations
 * @param  None
 * @retval None
 */
static void APP_THREAD_CheckWirelessFirmwareInfo(void)
{
  WirelessFwInfo_t wireless_info_instance;
  WirelessFwInfo_t* p_wireless_info = &wireless_info_instance;

  if (SHCI_GetWirelessFwInfo(p_wireless_info) != SHCI_Success)
  {
    APP_THREAD_Error((uint32_t)ERR_THREAD_CHECK_WIRELESS, (uint32_t)ERR_INTERFACE_FATAL);
  }
  else
  {
    APP_DBG("**********************************************************");
    APP_DBG("WIRELESS COPROCESSOR FW:");
    /* Print version */
    APP_DBG("VERSION ID = %d.%d.%d", p_wireless_info->VersionMajor, p_wireless_info->VersionMinor, p_wireless_info->VersionSub);

    switch(p_wireless_info->StackType)
    {
    case INFO_STACK_TYPE_THREAD_FTD :
      APP_DBG("FW Type : Thread FTD");
      break;
    case INFO_STACK_TYPE_THREAD_MTD :
      APP_DBG("FW Type : Thread MTD");
      break;
    case INFO_STACK_TYPE_BLE_THREAD_FTD_STATIC :
      APP_DBG("FW Type : Static Concurrent Mode BLE/Thread");
      break;
    case INFO_STACK_TYPE_BLE_THREAD_FTD_DYNAMIC :
       APP_DBG("FW Type : Static Concurrent Mode BLE/Thread");
       break;
    default :
      /* No Thread device supported ! */
      APP_THREAD_Error((uint32_t)ERR_THREAD_CHECK_WIRELESS, (uint32_t)ERR_INTERFACE_FATAL);
      break;
    }
    APP_DBG("**********************************************************");
  }
}

/*************************************************************
 *
 * FREERTOS WRAPPER FUNCTIONS
 *
*************************************************************/
static void APP_THREAD_FreeRTOSProcessMsgM0ToM4Task(void *argument)
{
  UNUSED(argument);
	uint8_t NotUsed = 0;


  for(;;)
  {
    /* USER CODE BEGIN APP_THREAD_FREERTOS_PROCESS_MSG_M0_TO_M4_1 */

    /* USER END END APP_THREAD_FREERTOS_PROCESS_MSG_M0_TO_M4_1 */
//    osThreadFlagsWait(1,osFlagsWaitAll,osWaitForever);
//    APP_THREAD_ProcessMsgM0ToM4();

	xQueueReceive(MoNotifQueue, &NotUsed, portMAX_DELAY);

//	while(g_ot_notification_allowed == 0){
//		osDelay(1);
//	}

	while(waitingForAck && !FlagReceiveAckFromM0){
		osDelay(1);
	}

	if (uxQueueMessagesWaiting(MoNotifQueue) > 1U) {
		APP_THREAD_Error(ERR_REC_MULTI_MSG_FROM_M0, 0);
	} else {
		OpenThread_CallBack_Processing();
	}
    /* USER CODE BEGIN APP_THREAD_FREERTOS_PROCESS_MSG_M0_TO_M4_2 */
//    FlagCmdProcessingFromM0 = 1;
    /* USER END END APP_THREAD_FREERTOS_PROCESS_MSG_M0_TO_M4_2 */
  }
}

#if (CFG_FULL_LOW_POWER == 0)
static void APP_THREAD_FreeRTOSSendCLIToM0Task(void *argument)
{
  UNUSED(argument);
  for(;;)
  {
    /* USER CODE BEGIN APP_THREAD_FREERTOS_SEND_CLI_TO_M0_1 */

    /* USER END END APP_THREAD_FREERTOS_SEND_CLI_TO_M0_1 */
    osThreadFlagsWait(1,osFlagsWaitAll,osWaitForever);
   // Send_CLI_To_M0();
    /* USER CODE BEGIN APP_THREAD_FREERTOS_SEND_CLI_TO_M0_2 */

    /* USER END END APP_THREAD_FREERTOS_SEND_CLI_TO_M0_2 */
  }
}
#endif /* (CFG_FULL_LOW_POWER == 0) */

/* USER CODE BEGIN FREERTOS_WRAPPER_FUNCTIONS */

/* USER CODE END FREERTOS_WRAPPER_FUNCTIONS */
const osSemaphoreAttr_t message_Lock_attributes = { .name = "generic" };

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS */
/**
 * @brief Main entry point for the Thread Application
 * @param  none
 * @retval None
 */
void APP_THREAD_Init_Dyn_1( void )
{
  /* USER CODE BEGIN APP_THREAD_INIT_1 */
	UTIL_LPM_SetStopMode(1 << CFG_LPM_APP_THREAD, UTIL_LPM_DISABLE);

	MtxOtCmdId = osSemaphoreNew(1, 1, &message_Lock_attributes);
	MtxOtAckId = osSemaphoreNew(1, 1, &message_Lock_attributes);

  /* USER CODE END APP_THREAD_INIT_1 */

  SHCI_CmdStatus_t ThreadInitStatus;

  /* Check the compatibility with the Coprocessor Wireless Firmware loaded */
  APP_THREAD_CheckWirelessFirmwareInfo();

#if (CFG_USB_INTERFACE_ENABLE != 0)
  VCP_Init(&VcpTxBuffer[0], &VcpRxBuffer[0]);
#endif /* (CFG_USB_INTERFACE_ENABLE != 0) */
  /* Register cmdbuffer */
  APP_THREAD_RegisterCmdBuffer(&ThreadOtCmdBuffer);

  /**
   * Do not allow standby in the application
   */
  UTIL_LPM_SetOffMode(1 << CFG_LPM_APP_THREAD, UTIL_LPM_DISABLE);

  /* Init config buffer and call TL_THREAD_Init */
  APP_THREAD_TL_THREAD_INIT();

  /* Configure UART for sending CLI command from M4 */
//  APP_THREAD_Init_UART_CLI();

  /* Send Thread start system cmd to M0 */
  ThreadInitStatus = SHCI_C2_THREAD_Init();

  /* Prevent unused argument(s) compilation warning */
  UNUSED(ThreadInitStatus);

  MoNotifQueue = xQueueCreate(10, sizeof(uint8_t));
  	if (MoNotifQueue == NULL) {
  		Error_Handler();  	}


  // TODO: added this call and remove the lines after since (I think) they are unnecessary
  OsTaskMsgM0ToM4Id = osThreadNew(APP_THREAD_FreeRTOSProcessMsgM0ToM4Task, NULL,&ThreadMsgM0ToM4Process_attr);



//  /* Register task */
//  /* Create the different tasks */
//  UTIL_SEQ_RegTask( 1<<(uint32_t)CFG_TASK_MSG_FROM_M0_TO_M4, UTIL_SEQ_RFU, APP_THREAD_ProcessMsgM0ToM4);
//  UTIL_SEQ_RegTask( 1<<(uint32_t)CFG_TASK_COAP_MSG_BUTTON, UTIL_SEQ_RFU, APP_THREAD_SendCoapMsg);
//
//  UTIL_SEQ_RegTask( 1<<(uint32_t)CFG_TASK_COAP_SEND_MSG, UTIL_SEQ_RFU,APP_THREAD_SendCoapMsg);
//  UTIL_SEQ_RegTask( 1<<(uint32_t)CFG_TASK_SET_THREAD_MODE, UTIL_SEQ_RFU,APP_THREAD_SetSleepyEndDeviceMode);
}

void APP_THREAD_Init_Dyn_2(void) {
  /* Initialize and configure the Thread device*/
  APP_THREAD_DeviceConfig();

  //TODO: removed below as per Thread-only FreeRTOS example
  /**
   * Create timer to handle COAP request sending
   */
//  HW_TS_Create(CFG_TIM_PROC_ID_ISR, &sedCoapTimerID, hw_ts_Repeated, APP_THREAD_CoapTimingElapsed);
//  SHCI_C2_RADIO_AllowLowPower(THREAD_IP,TRUE);
//  UTIL_LPM_SetStopMode(1 << CFG_LPM_APP_THREAD, UTIL_LPM_ENABLE);
  /* Allow the 800_15_4 IP to enter in low power mode */
}

void APP_THREAD_Stop(void)
{
  otError error;
  /* STOP THREAD */
  error = otThreadSetEnabled(NULL, false);
  if (error != OT_ERROR_NONE)
  {
     APP_THREAD_Error(ERR_THREAD_STOP,error);
  }
}

void APP_THREAD_CleanCallbacks(void)
{
  otRemoveStateChangeCallback(NULL, APP_THREAD_StateNotif, NULL);
  otCoapRemoveResource(NULL, &OT_Ressource);

//  /* Remove Timers if any */
//  HW_TS_Delete(setThreadModeTimerID);
//  HW_TS_Delete(sedCoapTimerID);
}

/* USER CODE END FD_LOCAL_FUNCTIONS */

/*************************************************************
 *
 * WRAP FUNCTIONS
 *
 *************************************************************/

void APP_THREAD_RegisterCmdBuffer(TL_CmdPacket_t* p_buffer)
{
  p_thread_otcmdbuffer = p_buffer;
}

Thread_OT_Cmd_Request_t* THREAD_Get_OTCmdPayloadBuffer(void)
{
  return (Thread_OT_Cmd_Request_t*)p_thread_otcmdbuffer->cmdserial.cmd.payload;
}

Thread_OT_Cmd_Request_t* THREAD_Get_OTCmdRspPayloadBuffer(void)
{
  return (Thread_OT_Cmd_Request_t*)((TL_EvtPacket_t *)p_thread_otcmdbuffer)->evtserial.evt.payload;
}

Thread_OT_Cmd_Request_t* THREAD_Get_NotificationPayloadBuffer(void)
{
  return (Thread_OT_Cmd_Request_t*)(p_thread_notif_M0_to_M4)->evtserial.evt.payload;
}

/**
 * @brief  This function is used to transfer the Ot commands from the
 *         M4 to the M0.
 *
 * @param   None
 * @return  None
 */
void Ot_Cmd_Transfer(void)
{
//	ot_StatusNot(ot_TL_CmdBusy);
	   if(osOK != osMutexAcquire(MtxHciId, osWaitForever)){
	    	Error_Handler();
	    }

  /* OpenThread OT command cmdcode range 0x280 .. 0x3DF = 352 */
  p_thread_otcmdbuffer->cmdserial.cmd.cmdcode = 0x280U;
  /* Size = otCmdBuffer->Size (Number of OT cmd arguments : 1 arg = 32bits so multiply by 4 to get size in bytes)
   * + ID (4 bytes) + Size (4 bytes) */
  uint32_t l_size = ((Thread_OT_Cmd_Request_t*)(p_thread_otcmdbuffer->cmdserial.cmd.payload))->Size * 4U + 8U;
  p_thread_otcmdbuffer->cmdserial.cmd.plen = l_size;

  TL_OT_SendCmd();

  /* Wait completion of cmd */
  Wait_Getting_Ack_From_M0();

  if(osOK != osMutexRelease(MtxHciId)){
   	Error_Handler();
   }

//  ot_StatusNot(ot_TL_CmdAvailable);

}

void ot_StatusNot( ot_TL_CmdStatus_t status )
{
if(MtxOtCmdId == 0){
	return;
}

  switch (status)
  {
    case ot_TL_CmdBusy:
        if(osOK != osSemaphoreAcquire( MtxOtCmdId, osWaitForever )){
        	Error_Handler();
        }
      break;

    case ot_TL_CmdAvailable:
        if(osOK != osSemaphoreRelease( MtxOtCmdId )){
        	Error_Handler();
        }
      break;

    default:
      break;
  }
  return;
}

static void ot_Ack( ot_TL_CmdStatus_t status )
{
  switch (status)
  {
    case ot_TL_CmdBusy:
    	osSemaphoreAcquire( MtxOtAckId, osWaitForever );
      break;

    case ot_TL_CmdAvailable:
    	osSemaphoreRelease( MtxOtAckId );
      break;

    default:
      break;
  }
  return;
}

/**
 * @brief  This function is used to transfer the Ot commands from the
 *         M4 to the M0 with Notification M0 to M4 allowed.
 *
 * @param   None
 * @return  None
 */
void Ot_Cmd_TransferWithNotif(void)
{
  /* Flag to specify to UTIL_SEQ_EvtIdle that M0 to M4 notifications are allowed */
  g_ot_notification_allowed = 1U;



  Ot_Cmd_Transfer();
}


/**
 * @brief  This function is called when acknowledge from OT command is received from the M0+.
 *
 * @param   Otbuffer : a pointer to TL_EvtPacket_t
 * @return  None
 */
void TL_OT_CmdEvtReceived( TL_EvtPacket_t * Otbuffer )
{
	//WARNING: CALLED FROM ISR

  /* Prevent unused argument(s) compilation warning */
  UNUSED(Otbuffer);

  Receive_Ack_From_M0();

  /* Does not allow OpenThread M0 to M4 notification */
  g_ot_notification_allowed = 0U;
}

/**
 * @brief  This function is called when notification from M0+ is received.
 *
 * @param   Notbuffer : a pointer to TL_EvtPacket_t
 * @return  None
 */
void TL_THREAD_NotReceived( TL_EvtPacket_t * Notbuffer )
{
  p_thread_notif_M0_to_M4 = Notbuffer;

  Receive_Notification_From_M0();
}

/**
  * @brief  This function is called before sending any ot command to the M0
  *         core. The purpose of this function is to be able to check if
  *         there are no notifications coming from the M0 core which are
  *         pending before sending a new ot command.
  * @param  None
  * @retval None
  */
void Pre_OtCmdProcessing(void)
{
	/* We want to only run this if no other thread
	 * is running an M0 command
	 *
	 * Possible threads:
	 *  ShciUserEvtProcessId
	 *  HciUserEvtProcessId
	 *  AdvUpdateProcessId
	 *  OsTaskMsgM0ToM4Id (APP_THREAD_FreeRTOSProcessMsgM0ToM4Task)
	 *
	 *  If no M0 commands are run, we can trigger
	 *  OsTaskMsgM0ToM4Id
	 */

//	while(CptReceiveMsgFromM0 > 0);
//	  while (FlagCmdProcessingFromM0 == 0)
//	  {
//	//	  osDelay(1);
//	  }
//	  FlagCmdProcessingFromM0 = 0;

//	if(xTaskGetSchedulerState() == taskSCHEDULER_RUNNING){
//		FlagCmdProcessingFromM0 = 1;
//	  ot_StatusNot(ot_TL_CmdBusy);
//	}

//	osThreadYield();
//	ot_StatusNot(ot_TL_CmdBusy);
//
//	osThreadFlagsSet(OsTaskMsgM0ToM4Id,1);

//		while(bluetoothCmdBusy == 1){
//
//		}
//	  osMutexAcquire(MtxHciId, osWaitForever);

//	osMutexAcquire(MtxThreadId, osWaitForever);

//    if(osOK != osMutexAcquire(MtxHciId, osWaitForever)){
//    	Error_Handler();
//    }
	ot_StatusNot(ot_TL_CmdBusy);

	/* only OsTaskMsgM0ToM4Id should be allowed to run while
	 * the other tasks are disabled
	 */

	//todo


}

/**
  * @brief  This function waits for getting an acknowledgment from the M0.
  *
  * @param  None
  * @retval None
  */

static void Wait_Getting_Ack_From_M0(void)
{
	// called in SHCI process
	waitingForAck = 1;
	while (FlagReceiveAckFromM0 == 0) {
	}
	FlagReceiveAckFromM0 = 0;
	waitingForAck = 0;

//    if(osOK != osMutexRelease(MtxHciId)){
//    	Error_Handler();
//    }
	ot_StatusNot(ot_TL_CmdAvailable);
}

/**
  * @brief  Receive an acknowledgment from the M0+ core.
  *         Each command send by the M4 to the M0 are acknowledged.
  *         This function is called under interrupt.
  * @param  None
  * @retval None
  */
static void Receive_Ack_From_M0(void)
{
	// Called from ISR!!
	FlagReceiveAckFromM0 = 1;
//	 ot_Ack(ot_TL_CmdAvailable);
//	  osThreadFlagsSet(OsTaskMsgM0ToM4Id,1);

//  FlagReceiveAckFromM0 = 1;
//  FlagCmdProcessingFromM0 = 0;



}

/**
  * @brief  Receive a notification from the M0+ through the IPCC.
  *         This function is called under interrupt.
  * @param  None
  * @retval None
  */
static void Receive_Notification_From_M0(void)
{
	// From ISR!

	/* The xHigherPriorityTaskWoken parameter must be initialized to pdFALSE as
	 it will get set to pdTRUE inside the interrupt safe API function if a
	 context switch is required. */
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	uint8_t NotUsed = 0;
	xQueueSendToFrontFromISR(MoNotifQueue, &NotUsed, &xHigherPriorityTaskWoken);

	/* Pass the xHigherPriorityTaskWoken value into portEND_SWITCHING_ISR(). If
	 xHigherPriorityTaskWoken was set to pdTRUE inside xSemaphoreGiveFromISR()
	 then calling portEND_SWITCHING_ISR() will request a context switch. If
	 xHigherPriorityTaskWoken is still pdFALSE then calling
	 portEND_SWITCHING_ISR() will have no effect */
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

#if (CFG_USB_INTERFACE_ENABLE != 0)
#else
#if (CFG_FULL_LOW_POWER == 0)
static void RxCpltCallback(void)
{
  /* Filling buffer and wait for '\r' char */
  if (indexReceiveChar < C_SIZE_CMD_STRING)
  {
    CommandString[indexReceiveChar++] = aRxBuffer[0];
    if (aRxBuffer[0] == '\r')
    {
      CptReceiveCmdFromUser = 1U;

      /* UART task scheduling*/
      osThreadFlagsSet(OsTaskCliId,1);
    }
  }

  /* Once a character has been sent, put back the device in reception mode */
//  HW_UART_Receive_IT(CFG_CLI_UART, aRxBuffer, 1U, RxCpltCallback);
}
#endif /* (CFG_FULL_LOW_POWER == 0) */
#endif /* (CFG_USB_INTERFACE_ENABLE != 0) */

#if (CFG_USB_INTERFACE_ENABLE != 0)
/**
 * @brief Process the command strings.
 *        As soon as a complete command string has been received, the task
 *        in charge of sending the command to the M0 is scheduled
 * @param  None
 * @retval None
 */
static uint32_t  ProcessCmdString( uint8_t* buf , uint32_t len )
{
  uint32_t i,j,tmp_start;
  tmp_start = 0;
  uint32_t res = 0;

  i= 0;
  while ((buf[i] != '\r') && (i < len))
  {
    i++;
  }

  if (i != len)
  {
    memcpy(CommandString, buf,(i+1));
    indexReceiveChar = i + 1U; /* Length of the buffer containing the command string */
    osThreadFlagsSet(OsTaskCliId,1)
    tmp_start = i;
    for (j = 0; j < (len - tmp_start - 1U) ; j++)
    {
      buf[j] = buf[tmp_start + j + 1U];
    }
    res = len - tmp_start - 1U;
  }
  else
  {
    res = len;
  }
  return res; /* Remaining characters in the temporary buffer */
}
#endif/* (CFG_USB_INTERFACE_ENABLE != 0) */

#if (CFG_FULL_LOW_POWER == 0)
/**
 * @brief Process sends receive CLI command to M0.
 * @param  None
 * @retval None
 */
static void Send_CLI_To_M0(void)
{
  memset(ThreadCliCmdBuffer.cmdserial.cmd.payload, 0x0U, 255U);
  memcpy(ThreadCliCmdBuffer.cmdserial.cmd.payload, CommandString, indexReceiveChar);
  ThreadCliCmdBuffer.cmdserial.cmd.plen = indexReceiveChar;
  ThreadCliCmdBuffer.cmdserial.cmd.cmdcode = 0x0;

  /* Clear receive buffer, character counter and command complete */
  CptReceiveCmdFromUser = 0;
  indexReceiveChar = 0;
  memset(CommandString, 0, C_SIZE_CMD_STRING);

  TL_CLI_SendCmd();
}
#endif /* (CFG_FULL_LOW_POWER == 0) */

/**
 * @brief Send notification for CLI TL Channel.
 * @param  None
 * @retval None
 */
static void Send_CLI_Ack_For_OT(void)
{

  /* Notify M0 that characters have been sent to UART */
  TL_THREAD_CliSendAck();
}

/**
 * @brief Perform initialization of CLI UART interface.
 * @param  None
 * @retval None
 */
void APP_THREAD_Init_UART_CLI(void)
{
#if (CFG_FULL_LOW_POWER == 0)
  OsTaskCliId = osThreadNew(APP_THREAD_FreeRTOSSendCLIToM0Task, NULL,&ThreadCliProcess_attr);
#endif /* (CFG_FULL_LOW_POWER == 0) */

#if (CFG_USB_INTERFACE_ENABLE != 0)
#else
#if (CFG_FULL_LOW_POWER == 0)
//  MX_USART1_UART_Init();
//  HW_UART_Receive_IT(CFG_CLI_UART, aRxBuffer, 1, RxCpltCallback);
#endif /* (CFG_FULL_LOW_POWER == 0) */
#endif /* (CFG_USB_INTERFACE_ENABLE != 0) */
}

/**
 * @brief Perform initialization of TL for THREAD.
 * @param  None
 * @retval None
 */
void APP_THREAD_TL_THREAD_INIT(void)
{
//  ThreadConfigBuffer.p_ThreadOtCmdRspBuffer = (uint8_t*)&ThreadOtCmdBuffer;
//  ThreadConfigBuffer.p_ThreadNotAckBuffer = (uint8_t*)ThreadNotifRspEvtBuffer;
//  ThreadConfigBuffer.p_ThreadCliRspBuffer = (uint8_t*)&ThreadCliCmdBuffer;

  ThreadConfigBuffer.p_ThreadOtCmdRspBuffer = (uint8_t*)&ThreadOtCmdBuffer;
  ThreadConfigBuffer.p_ThreadNotAckBuffer = (uint8_t*)ThreadNotifRspEvtBuffer;
  ThreadConfigBuffer.p_ThreadCliRspBuffer = (uint8_t*)&ThreadCliCmdBuffer;
  ThreadConfigBuffer.p_ThreadCliNotBuffer = (uint8_t*)&ThreadCliNotBuffer;

  TL_THREAD_Init( &ThreadConfigBuffer );
}

/**
 * @brief  This function is called when notification on CLI TL Channel from M0+ is received.
 *
 * @param   Notbuffer : a pointer to TL_EvtPacket_t
 * @return  None
 */
void TL_THREAD_CliNotReceived( TL_EvtPacket_t * Notbuffer )
{
  TL_CmdPacket_t* l_CliBuffer = (TL_CmdPacket_t*)Notbuffer;
  uint8_t l_size = l_CliBuffer->cmdserial.cmd.plen;

  /* WORKAROUND: if string to output is "> " then respond directly to M0 and do not output it */
  if (strcmp((const char *)l_CliBuffer->cmdserial.cmd.payload, "> ") != 0)
  {
    /* Write to CLI UART */
#if (CFG_USB_INTERFACE_ENABLE != 0)
    VCP_SendData( l_CliBuffer->cmdserial.cmd.payload, l_size, HostTxCb);
#else
//    HW_UART_Transmit_IT(CFG_CLI_UART, l_CliBuffer->cmdserial.cmd.payload, l_size, HostTxCb);
#endif /*USAGE_OF_VCP */
  }
  else
  {
    Send_CLI_Ack_For_OT();
  }
}

/**
 * @brief  End of transfer callback for CLI UART sending.
 *
 * @param   Notbuffer : a pointer to TL_EvtPacket_t
 * @return  None
 */
void HostTxCb(void)
{
  Send_CLI_Ack_For_OT();
}

/**
 * @brief Process the messages coming from the M0.
 * @param  None
 * @retval None
 */
//void APP_THREAD_ProcessMsgM0ToM4(void)
//{
//  if (CptReceiveMsgFromM0 != 0)
//  {
//    /* If CptReceiveMsgFromM0 is > 1. it means that we did not serve all the events from the radio */
//    if (CptReceiveMsgFromM0 > 1U)
//    {
//      APP_THREAD_Error(ERR_REC_MULTI_MSG_FROM_M0, 0);
//    }
//    else
//    {
//      OpenThread_CallBack_Processing();
//    }
//    /* Reset counter */
//    CptReceiveMsgFromM0 = 0;
//  }
//}

#if (CFG_USB_INTERFACE_ENABLE != 0)
/**
 * @brief  This function is called when thereare some data coming
 *         from the Hyperterminal via the USB port
 *         Data received over USB OUT endpoint are sent over CDC interface
 *         through this function.
 * @param  Buf: Buffer of data received
 * @param  Len: Number of data received (in bytes)
 * @retval Number of characters remaining in the buffer and not yet processed
 */
void VCP_DataReceived(uint8_t* Buf , uint32_t *Len)
{
  uint32_t i,flag_continue_checking = TRUE;
  uint32_t char_remaining = 0;
  static uint32_t len_total = 0;

  /* Copy the characteres in the temporary buffer */
  for (i = 0; i < *Len; i++)
  {
    TmpString[len_total++] = Buf[i];
  }

  /* Process the buffer commands one by one     */
  /* A command is limited by a \r caracaters    */
  while (flag_continue_checking == TRUE)
  {
    char_remaining = ProcessCmdString(TmpString,len_total);
    /* If char_remaining is equal to len_total, it means that the command string is not yet
     * completed.
     * If char_remaining is equal to 0, it means that the command string has
     * been entirely processed.
     */
    if ((char_remaining == 0) || (char_remaining == len_total))
    {
      flag_continue_checking = FALSE;
    }
    len_total = char_remaining;
  }
}
#endif /* (CFG_USB_INTERFACE_ENABLE != 0) */

/* USER CODE BEGIN FD_WRAP_FUNCTIONS */
/**
  * @brief Handler called when the server receives a COAP request.
  * @param pContext : Context
  * @param pMessage : Message
  * @param pMessageInfo : Message information
  * @retval None
  */
static void APP_THREAD_CoapRequestHandler(void                * pContext,
                                          otMessage           * pMessage,
                                          const otMessageInfo * pMessageInfo)
{
  do
  {
//    if (otCoapMessageGetType(pMessage) != OT_COAP_TYPE_NON_CONFIRMABLE)
//    {
//      break;
//    }

//    if (otCoapMessageGetCode(pMessage) != OT_COAP_CODE_PUT)
//    {
//      break;
//    }
	  uint16_t receivedBytes;
	  receivedBytes = otMessageRead(pMessage, otMessageGetOffset(pMessage), PayloadRead, sizeof(PayloadRead));
	  if (strncmp(PayloadRead, "test", 4) == 0) {
		  toggledBlue();
			osThreadFlagsSet(AdvUpdateProcessId, 1);

	  } else {
//		  printf("The string does not start with 'test'.\n");
	  }

//    if (otMessageRead(pMessage, otMessageGetOffset(pMessage), &OT_ReceivedCommand, 1U) != 1U)
//    {
//      APP_THREAD_Error(ERR_THREAD_MESSAGE_READ, 0);
//    }

    if (OT_ReceivedCommand == 1U)
    {
//      BSP_LED_Toggle(LED3);
//      APP_DBG("**** Recept COAP nb **** %d ",DebugRxCoapCpt++);
    }

    /* If Message is Confirmable, send response */
    if (otCoapMessageGetType(pMessage) == OT_COAP_TYPE_CONFIRMABLE)
    {
      APP_THREAD_CoapSendDataResponse(pMessage, pMessageInfo);
    }

  } while (false);
}

/**
 * @brief This function acknowledges the data reception by sending an ACK
 *    back to the sender.
 * @param  pMessage coap message
 * @param  pMessageInfo message info pointer
 * @retval None
 */
static otMessage* pOT_MessageResponse = NULL;
static void APP_THREAD_CoapSendDataResponse(otMessage  * pMessage,
    const otMessageInfo * pMessageInfo)
{
  otError  error = OT_ERROR_NONE;

  do{
    APP_DBG("APP_THREAD_CoapSendDataResponse");

    pOT_MessageResponse = otCoapNewMessage(NULL, NULL);
    if (pOT_MessageResponse == NULL)
    {
      APP_DBG("WARNING : pOT_MessageResponse = NULL ! -> exit now");
      break;
    }

    otCoapMessageInitResponse(pOT_MessageResponse,
        pMessage,
        OT_COAP_TYPE_ACKNOWLEDGMENT,
        OT_COAP_CODE_VALID);

    error = otCoapSendResponse(NULL, pOT_MessageResponse, pMessageInfo);
    if (error != OT_ERROR_NONE && pOT_MessageResponse != NULL)
    {
      otMessageFree(pOT_MessageResponse);
//      APP_THREAD_Error(ERR_THREAD_COAP_DATA_RESPONSE,error);
    }
  }while(false);
}
/* USER CODE END FD_WRAP_FUNCTIONS */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
