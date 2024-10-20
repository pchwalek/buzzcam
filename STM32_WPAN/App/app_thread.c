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
#include "pb.h"
#include "pb_decode.h"
#include "pb_encode.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private defines -----------------------------------------------------------*/
#define C_SIZE_CMD_STRING       256U
#define C_PANID                 0x1234U
#define C_CHANNEL_NB            23U

/* FreeRtos stacks attributes */
const osThreadAttr_t ThreadMsgM0ToM4Process_attr = { .name =
		CFG_THREAD_MSG_M0_TO_M4_PROCESS_NAME, .attr_bits =
				CFG_THREAD_MSG_M0_TO_M4_PROCESS_ATTR_BITS, .cb_mem =
						CFG_THREAD_MSG_M0_TO_M4_PROCESS_CB_MEM, .cb_size =
								CFG_THREAD_MSG_M0_TO_M4_PROCESS_CB_SIZE, .stack_mem =
										CFG_THREAD_MSG_M0_TO_M4_PROCESS_STACK_MEM, .priority =
												CFG_THREAD_MSG_M0_TO_M4_PROCESS_PRIORITY, .stack_size =
														CFG_THREAD_MSG_M0_TO_M4_PROCESS_STACK_SIZE };

const osThreadAttr_t ThreadCliProcess_attr = { .name =
		CFG_THREAD_CLI_PROCESS_NAME, .attr_bits =
				CFG_THREAD_CLI_PROCESS_ATTR_BITS, .cb_mem =
						CFG_THREAD_CLI_PROCESS_CB_MEM,
						.cb_size = CFG_THREAD_CLI_PROCESS_CB_SIZE, .stack_mem =
								CFG_THREAD_CLI_PROCESS_STACK_MEM, .priority =
										CFG_THREAD_CLI_PROCESS_PRIORITY, .stack_size =
												CFG_THREAD_CLI_PROCESS_STACK_SIZE };

/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macros ------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define C_RESSOURCE             "buzz"
#define C_CONFIG_RESSOURCE		"config"

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
static void APP_THREAD_TraceError(const char *pMess, uint32_t ErrCode);
#if (CFG_FULL_LOW_POWER == 0)
static void Send_CLI_To_M0(void);
#endif /* (CFG_FULL_LOW_POWER == 0) */
static void Send_CLI_Ack_For_OT(void);
static void HostTxCb(void);
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
void createConfigThread(void *pvParameter1, uint32_t ulParameter2);
//void vProcessInterface( void *pvParameter1, uint32_t ulParameter2 );

static void APP_THREAD_SendCoapMsg(void);
static void APP_THREAD_SendCoapMulticastRequest(uint8_t command);
static void APP_THREAD_CoapRequestHandler(void *pContext, otMessage *pMessage,
		const otMessageInfo *pMessageInfo);
static void APP_THREAD_CoapConfigHandler(void *pContext, otMessage *pMessage,
		const otMessageInfo *pMessageInfo);
static void APP_THREAD_SetSleepyEndDeviceMode(void);
static void APP_THREAD_CoapTimingElapsed(void);

void updateTable(uwb_range_t newEntry);

static void APP_THREAD_CoapSendRequest(otCoapResource *aCoapRessource,
		otCoapType aCoapType, otCoapCode aCoapCode, const char *aStringAddress,
		const otIp6Address *aPeerAddress, uint8_t *aPayload, uint16_t Size,
		otCoapResponseHandler aHandler, void *aContext);

//static void ot_StatusNot( ot_TL_CmdStatus_t status );
osMutexId_t MtxOtCmdId;
osMutexId_t MtxOtAckId;

static configChange configMsg;

static void ot_Ack(ot_TL_CmdStatus_t status);

void Ot_Cmd_TransferWithNotif(void);

void sendConfig(otIp6Address addr);

static colorConfig color;
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
static __IO uint32_t CptReceiveMsgFromM0 = 0;
static volatile int FlagReceiveAckFromM0 = 0;

PLACE_IN_SECTION("MB_MEM1") ALIGN(4) static TL_TH_Config_t ThreadConfigBuffer;
PLACE_IN_SECTION("MB_MEM2") ALIGN(4) static TL_CmdPacket_t ThreadOtCmdBuffer;
PLACE_IN_SECTION("MB_MEM2") ALIGN(4) static uint8_t ThreadNotifRspEvtBuffer[sizeof(TL_PacketHeader_t)
																			+ TL_EVT_HDR_SIZE + 255U];
PLACE_IN_SECTION("MB_MEM2") ALIGN(4) static TL_CmdPacket_t ThreadCliCmdBuffer;
PLACE_IN_SECTION("MB_MEM2") ALIGN(4) static TL_CmdPacket_t ThreadCliNotBuffer;

osThreadId_t OsTaskMsgM0ToM4Id; /* Task managing the M0 to M4 messaging        */
#if (CFG_FULL_LOW_POWER == 0)
static osThreadId_t OsTaskCliId;            /* Task used to manage CLI comamnd             */
#endif /* (CFG_FULL_LOW_POWER == 0) */

/* USER CODE BEGIN PV */
static uint32_t prev_master_epoch = 0;
static uint32_t prev_master_uid = 0;

static otCoapResource OT_ConfigRessource = { C_CONFIG_RESSOURCE,
		APP_THREAD_CoapConfigHandler, "config", NULL };
static otCoapResource OT_Ressource = { C_RESSOURCE,
		APP_THREAD_CoapRequestHandler, "buzz", NULL };
static otMessageInfo OT_MessageInfo = { 0 };
static uint8_t OT_Command = 0;
static uint8_t OT_ReceivedCommand = 0;
static otMessage *pOT_Message = NULL;
static otLinkModeConfig OT_LinkMode = { 0 };
static uint32_t sleepyEndDeviceFlag = FALSE;
static uint8_t sedCoapTimerID;
static uint8_t setThreadModeTimerID;
static void APP_THREAD_CoapSendDataResponse(otMessage *pMessage,
		const otMessageInfo *pMessageInfo);

/* Debug */
static uint32_t DebugRxCoapCpt = 0;
static uint32_t DebugTxCoapCpt = 0;

static uint8_t PayloadWrite[30] = { 0 };
static uint8_t PayloadRead[30] = { 0 };

fileWriteSync_t fileWriteSync;
timestampSync_t  timestampSync;

static volatile uint8_t waitingForAck = 0;

extern volatile uint8_t g_ot_notification_allowed;

uwb_packet_t uwb_table = { 0 };

uwb_info_t connectedNodeInfo[MAX_CONN_COUNT] = { 0 };
uwb_ranges_t rangesUWB[MAX_UWB_RANGES];

uint8_t addInfotoTableUWB(uwb_info_t *new_info);
void updateRangeTableUWB(uint32_t system_ID_1, uint32_t system_ID_2,
		float range, float std_dev);
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
void APP_THREAD_Error(uint32_t ErrId, uint32_t ErrCode) {
	/* USER CODE BEGIN APP_THREAD_Error_1 */

	/* USER CODE END APP_THREAD_Error_1 */
	switch (ErrId) {
	case ERR_REC_MULTI_MSG_FROM_M0:
		APP_THREAD_TraceError("ERROR : ERR_REC_MULTI_MSG_FROM_M0 ", ErrCode);
		break;
	case ERR_THREAD_SET_STATE_CB:
		APP_THREAD_TraceError("ERROR : ERR_THREAD_SET_STATE_CB ", ErrCode);
		break;
	case ERR_THREAD_SET_CHANNEL:
		APP_THREAD_TraceError("ERROR : ERR_THREAD_SET_CHANNEL ", ErrCode);
		break;
	case ERR_THREAD_SET_PANID:
		APP_THREAD_TraceError("ERROR : ERR_THREAD_SET_PANID ", ErrCode);
		break;
	case ERR_THREAD_IPV6_ENABLE:
		APP_THREAD_TraceError("ERROR : ERR_THREAD_IPV6_ENABLE ", ErrCode);
		break;
	case ERR_THREAD_START:
		APP_THREAD_TraceError("ERROR: ERR_THREAD_START ", ErrCode);
		break;
	case ERR_THREAD_ERASE_PERSISTENT_INFO:
		APP_THREAD_TraceError("ERROR : ERR_THREAD_ERASE_PERSISTENT_INFO ",
				ErrCode);
		break;
	case ERR_THREAD_CHECK_WIRELESS:
		APP_THREAD_TraceError("ERROR : ERR_THREAD_CHECK_WIRELESS ", ErrCode);
		break;
		/* USER CODE BEGIN APP_THREAD_Error_2 */

		/* USER CODE END APP_THREAD_Error_2 */
	default:
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
static void APP_THREAD_DeviceConfig(void) {
	otError error;
	otNetworkKey networkKey = { { 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66,
			0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF } };

	//  error = otInstanceErasePersistentInfo(NULL);
	//  if (error != OT_ERROR_NONE)
	//  {
	//    APP_THREAD_Error(ERR_THREAD_ERASE_PERSISTENT_INFO,error);
	//  }
	otInstanceFinalize(NULL);
	otInstanceInitSingle();
	error = otSetStateChangedCallback(NULL, APP_THREAD_StateNotif, NULL);
	if (error != OT_ERROR_NONE) {
		Error_Handler();
	}
	error = otLinkSetChannel(NULL,
			configPacket.payload.config_packet.network_state.channel);
	if (error != OT_ERROR_NONE) {
		Error_Handler();
	}
	error = otLinkSetPanId(NULL,
			configPacket.payload.config_packet.network_state.pan_id);
	if (error != OT_ERROR_NONE) {
		Error_Handler();
	}
	error = otThreadSetNetworkKey(NULL, &networkKey);
	if (error != OT_ERROR_NONE) {
		Error_Handler();
	}
	otThreadSetChildTimeout(NULL, 15);
	error = otIp6SetEnabled(NULL, true);
	if (error != OT_ERROR_NONE) {
		Error_Handler();
	}
	error = otThreadSetEnabled(NULL, true);
	if (error != OT_ERROR_NONE) {
		Error_Handler();
	}

	/* USER CODE BEGIN DEVICECONFIG */
	error = otCoapStart(NULL, OT_DEFAULT_COAP_PORT);
	if (error != OT_ERROR_NONE) {
		APP_THREAD_Error(ERR_THREAD_COAP_START, error);
	}
	/* Add COAP resources */
	otCoapAddResource(NULL, &OT_Ressource);
	otCoapAddResource(NULL, &OT_ConfigRessource);

	coapSetup = 1;

	//  if(configPacket.payload.config_packet.network_state.master_node){
	//
	//  }
	//
	//  else if(configPacket.payload.config_packet.network_state.slave_sync){
	//
	//  }
	/* USER CODE END DEVICECONFIG */
}

/**
 * @brief Thread notification when the state changes.
 * @param  aFlags  : Define the item that has been modified
 *         aContext: Context
 *
 * @retval None
 */
volatile uint8_t gettingUWB_Info = 0;
static void APP_THREAD_StateNotif(uint32_t NotifFlags, void *pContext) {
	/* Prevent unused argument(s) compilation warning */
	UNUSED(pContext);

	/* USER CODE BEGIN APP_THREAD_STATENOTIF */

	/* USER CODE END APP_THREAD_STATENOTIF */

	if ((NotifFlags & (uint32_t) OT_CHANGED_THREAD_ROLE)
			== (uint32_t) OT_CHANGED_THREAD_ROLE) {
		switch (otThreadGetDeviceRole(NULL)) {
		case OT_DEVICE_ROLE_DISABLED:
			/* USER CODE BEGIN OT_DEVICE_ROLE_DISABLED */
			prev_master_uid = 0;
			local_uwbInfo.has_uwb = 0;
			/* USER CODE END OT_DEVICE_ROLE_DISABLED */
			break;
		case OT_DEVICE_ROLE_DETACHED:
			/* USER CODE BEGIN OT_DEVICE_ROLE_DETACHED */
			prev_master_uid = 0;
			local_uwbInfo.has_uwb = 0;
			/* USER CODE END OT_DEVICE_ROLE_DETACHED */
			break;
		case OT_DEVICE_ROLE_CHILD:
			/* USER CODE BEGIN OT_DEVICE_ROLE_CHILD */
			/* request info from onboard UWB node */
			if(local_uwbInfo.has_uwb == 0){
				if(!gettingUWB_Info){
					osThreadFlagsSet(uwbMessageTaskId, UWB_GET_INFO);
					gettingUWB_Info = 1;
				}
			}
			if((configPacket.payload.config_packet.network_state.master_node == 0) &&
					(configPacket.payload.config_packet.network_state.slave_sync == 1)){
				if(prev_master_uid == 0) alertMaster(); // if never received a packet from master
			}
			/* USER CODE END OT_DEVICE_ROLE_CHILD */
			break;
		case OT_DEVICE_ROLE_ROUTER:
			/* USER CODE BEGIN OT_DEVICE_ROLE_ROUTER */
			/* request info from onboard UWB node */
			if(local_uwbInfo.has_uwb == 0){
				if(!gettingUWB_Info){
					osThreadFlagsSet(uwbMessageTaskId, UWB_GET_INFO);
					gettingUWB_Info = 1;
				}
			}
			if((configPacket.payload.config_packet.network_state.master_node == 0) &&
					(configPacket.payload.config_packet.network_state.slave_sync == 1)){
				if(prev_master_uid == 0) alertMaster(); // if never received a packet from master
			}
			break;
		case OT_DEVICE_ROLE_LEADER:
			/* USER CODE BEGIN OT_DEVICE_ROLE_LEADER */
			/* request info from onboard UWB node */
			if(local_uwbInfo.has_uwb == 0){
				if(!gettingUWB_Info){
					osThreadFlagsSet(uwbMessageTaskId, UWB_GET_INFO);
					gettingUWB_Info = 1;
				}
			}
			if((configPacket.payload.config_packet.network_state.master_node == 0) &&
					(configPacket.payload.config_packet.network_state.slave_sync == 1)){
				if(prev_master_uid == 0) alertMaster(); // if never received a packet from master
			}else{

			}

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
static void APP_THREAD_TraceError(const char *pMess, uint32_t ErrCode) {
	/* USER CODE BEGIN TRACE_ERROR */

	/* USER CODE END TRACE_ERROR */
}

/**
 * @brief Check if the Coprocessor Wireless Firmware loaded supports Thread
 *        and display associated informations
 * @param  None
 * @retval None
 */
static void APP_THREAD_CheckWirelessFirmwareInfo(void) {
	WirelessFwInfo_t wireless_info_instance;
	WirelessFwInfo_t *p_wireless_info = &wireless_info_instance;

	if (SHCI_GetWirelessFwInfo(p_wireless_info) != SHCI_Success) {
		APP_THREAD_Error((uint32_t) ERR_THREAD_CHECK_WIRELESS,
				(uint32_t) ERR_INTERFACE_FATAL);
	} else {
		APP_DBG("**********************************************************");
		APP_DBG("WIRELESS COPROCESSOR FW:");
		/* Print version */
		APP_DBG("VERSION ID = %d.%d.%d", p_wireless_info->VersionMajor,
				p_wireless_info->VersionMinor, p_wireless_info->VersionSub);

		switch (p_wireless_info->StackType) {
		case INFO_STACK_TYPE_THREAD_FTD:
			APP_DBG("FW Type : Thread FTD")
			;
			break;
		case INFO_STACK_TYPE_THREAD_MTD:
			APP_DBG("FW Type : Thread MTD")
			;
			break;
		case INFO_STACK_TYPE_BLE_THREAD_FTD_STATIC:
			APP_DBG("FW Type : Static Concurrent Mode BLE/Thread")
			;
			break;
		case INFO_STACK_TYPE_BLE_THREAD_FTD_DYNAMIC:
			APP_DBG("FW Type : Static Concurrent Mode BLE/Thread")
			;
			break;
		default:
			/* No Thread device supported ! */
			APP_THREAD_Error((uint32_t) ERR_THREAD_CHECK_WIRELESS,
					(uint32_t) ERR_INTERFACE_FATAL);
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
static void APP_THREAD_FreeRTOSProcessMsgM0ToM4Task(void *argument) {
	UNUSED(argument);
	uint8_t NotUsed = 0;

	for (;;) {
		/* USER CODE BEGIN APP_THREAD_FREERTOS_PROCESS_MSG_M0_TO_M4_1 */

		/* USER END END APP_THREAD_FREERTOS_PROCESS_MSG_M0_TO_M4_1 */
		//    osThreadFlagsWait(1,osFlagsWaitAll,osWaitForever);
		//    APP_THREAD_ProcessMsgM0ToM4();
		xQueueReceive(MoNotifQueue, &NotUsed, portMAX_DELAY);

		//	while(g_ot_notification_allowed == 0){
		//		osDelay(1);
		//	}

		while (waitingForAck && !FlagReceiveAckFromM0) {
			osDelay(1);
		}

		if (osOK != osMutexAcquire(MtxHciId, osWaitForever)) {
			Error_Handler();
		}
		if (osOK != osMutexRelease(MtxHciId)) {
			Error_Handler();
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
void APP_THREAD_Init_Dyn_1(void) {
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
		Error_Handler();
	}

	// TODO: added this call and remove the lines after since (I think) they are unnecessary
	OsTaskMsgM0ToM4Id = osThreadNew(APP_THREAD_FreeRTOSProcessMsgM0ToM4Task,
			NULL, &ThreadMsgM0ToM4Process_attr);

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

void APP_THREAD_Stop(void) {
	otError error;
	/* STOP THREAD */
	error = otThreadSetEnabled(NULL, false);
	if (error != OT_ERROR_NONE) {
		APP_THREAD_Error(ERR_THREAD_STOP, error);
	}
}

void APP_THREAD_CleanCallbacks(void) {
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

void APP_THREAD_RegisterCmdBuffer(TL_CmdPacket_t *p_buffer) {
	p_thread_otcmdbuffer = p_buffer;
}

Thread_OT_Cmd_Request_t* THREAD_Get_OTCmdPayloadBuffer(void) {
	return (Thread_OT_Cmd_Request_t*) p_thread_otcmdbuffer->cmdserial.cmd.payload;
}

Thread_OT_Cmd_Request_t* THREAD_Get_OTCmdRspPayloadBuffer(void) {
	return (Thread_OT_Cmd_Request_t*) ((TL_EvtPacket_t*) p_thread_otcmdbuffer)->evtserial.evt.payload;
}

Thread_OT_Cmd_Request_t* THREAD_Get_NotificationPayloadBuffer(void) {
	return (Thread_OT_Cmd_Request_t*) (p_thread_notif_M0_to_M4)->evtserial.evt.payload;
}

/**
 * @brief  This function is used to transfer the Ot commands from the
 *         M4 to the M0.
 *
 * @param   None
 * @return  None
 */
void Ot_Cmd_Transfer(void) {
	//	ot_StatusNot(ot_TL_CmdBusy);
	if (osOK != osMutexAcquire(MtxHciId, osWaitForever)) {
		Error_Handler();
	}

	/* OpenThread OT command cmdcode range 0x280 .. 0x3DF = 352 */
	p_thread_otcmdbuffer->cmdserial.cmd.cmdcode = 0x280U;
	/* Size = otCmdBuffer->Size (Number of OT cmd arguments : 1 arg = 32bits so multiply by 4 to get size in bytes)
	 * + ID (4 bytes) + Size (4 bytes) */
	uint32_t l_size =
			((Thread_OT_Cmd_Request_t*) (p_thread_otcmdbuffer->cmdserial.cmd.payload))->Size
			* 4U + 8U;
	p_thread_otcmdbuffer->cmdserial.cmd.plen = l_size;

	TL_OT_SendCmd();

	/* Wait completion of cmd */
	Wait_Getting_Ack_From_M0();

	if (osOK != osMutexRelease(MtxHciId)) {
		Error_Handler();
	}

	//  ot_StatusNot(ot_TL_CmdAvailable);

}

void ot_StatusNot(ot_TL_CmdStatus_t status) {
	if (MtxOtCmdId == 0) {
		return;
	}

	switch (status) {
	case ot_TL_CmdBusy:
		if (osOK != osSemaphoreAcquire(MtxOtCmdId, osWaitForever)) {
			Error_Handler();
		}
		break;

	case ot_TL_CmdAvailable:
		if (osOK != osSemaphoreRelease(MtxOtCmdId)) {
			Error_Handler();
		}
		break;

	default:
		break;
	}
	return;
}

static void ot_Ack(ot_TL_CmdStatus_t status) {
	switch (status) {
	case ot_TL_CmdBusy:
		osSemaphoreAcquire(MtxOtAckId, osWaitForever);
		break;

	case ot_TL_CmdAvailable:
		osSemaphoreRelease(MtxOtAckId);
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
void Ot_Cmd_TransferWithNotif(void) {
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
void TL_OT_CmdEvtReceived(TL_EvtPacket_t *Otbuffer) {
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
void TL_THREAD_NotReceived(TL_EvtPacket_t *Notbuffer) {
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
void Pre_OtCmdProcessing(void) {
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

}

/**
 * @brief  This function waits for getting an acknowledgment from the M0.
 *
 * @param  None
 * @retval None
 */

static void Wait_Getting_Ack_From_M0(void) {
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
static void Receive_Ack_From_M0(void) {
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
static void Receive_Notification_From_M0(void) {
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
static void Send_CLI_Ack_For_OT(void) {

	/* Notify M0 that characters have been sent to UART */
	TL_THREAD_CliSendAck();
}

/**
 * @brief Perform initialization of CLI UART interface.
 * @param  None
 * @retval None
 */
void APP_THREAD_Init_UART_CLI(void) {
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
void APP_THREAD_TL_THREAD_INIT(void) {
	//  ThreadConfigBuffer.p_ThreadOtCmdRspBuffer = (uint8_t*)&ThreadOtCmdBuffer;
	//  ThreadConfigBuffer.p_ThreadNotAckBuffer = (uint8_t*)ThreadNotifRspEvtBuffer;
	//  ThreadConfigBuffer.p_ThreadCliRspBuffer = (uint8_t*)&ThreadCliCmdBuffer;

	ThreadConfigBuffer.p_ThreadOtCmdRspBuffer = (uint8_t*) &ThreadOtCmdBuffer;
	ThreadConfigBuffer.p_ThreadNotAckBuffer =
			(uint8_t*) ThreadNotifRspEvtBuffer;
	ThreadConfigBuffer.p_ThreadCliRspBuffer = (uint8_t*) &ThreadCliCmdBuffer;
	ThreadConfigBuffer.p_ThreadCliNotBuffer = (uint8_t*) &ThreadCliNotBuffer;

	TL_THREAD_Init(&ThreadConfigBuffer);
}

/**
 * @brief  This function is called when notification on CLI TL Channel from M0+ is received.
 *
 * @param   Notbuffer : a pointer to TL_EvtPacket_t
 * @return  None
 */
void TL_THREAD_CliNotReceived(TL_EvtPacket_t *Notbuffer) {
	TL_CmdPacket_t *l_CliBuffer = (TL_CmdPacket_t*) Notbuffer;
	uint8_t l_size = l_CliBuffer->cmdserial.cmd.plen;

	/* WORKAROUND: if string to output is "> " then respond directly to M0 and do not output it */
	if (strcmp((const char*) l_CliBuffer->cmdserial.cmd.payload, "> ") != 0) {
		/* Write to CLI UART */
#if (CFG_USB_INTERFACE_ENABLE != 0)
		VCP_SendData( l_CliBuffer->cmdserial.cmd.payload, l_size, HostTxCb);
#else
		//    HW_UART_Transmit_IT(CFG_CLI_UART, l_CliBuffer->cmdserial.cmd.payload, l_size, HostTxCb);
#endif /*USAGE_OF_VCP */
	} else {
		Send_CLI_Ack_For_OT();
	}
}

/**
 * @brief  End of transfer callback for CLI UART sending.
 *
 * @param   Notbuffer : a pointer to TL_EvtPacket_t
 * @return  None
 */
void HostTxCb(void) {
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
static void APP_THREAD_CoapRequestHandler(void *pContext, otMessage *pMessage,
		const otMessageInfo *pMessageInfo) {
	do {
		//    if (otCoapMessageGetType(pMessage) != OT_COAP_TYPE_NON_CONFIRMABLE)
		//    {
		//      break;
		//    }

		//    if (otCoapMessageGetCode(pMessage) != OT_COAP_CODE_PUT)
		//    {
		//      break;
		//    }
		uint16_t receivedBytes;
		receivedBytes = otMessageRead(pMessage, otMessageGetOffset(pMessage),
				PayloadRead, sizeof(PayloadRead));
		if (strncmp(PayloadRead, "light", 5) == 0) {
			toggledGreen();
			//			osThreadFlagsSet(AdvUpdateProcessId, 1);

		} else {
			//		  printf("The string does not start with 'test'.\n");
		}

		//    if (otMessageRead(pMessage, otMessageGetOffset(pMessage), &OT_ReceivedCommand, 1U) != 1U)
		//    {
		//      APP_THREAD_Error(ERR_THREAD_MESSAGE_READ, 0);
		//    }

		if (OT_ReceivedCommand == 1U) {
			//      BSP_LED_Toggle(LED3);
			//      APP_DBG("**** Recept COAP nb **** %d ",DebugRxCoapCpt++);
		}

		/* If Message is Confirmable, send response */
		if (otCoapMessageGetType(pMessage) == OT_COAP_TYPE_CONFIRMABLE) {
			APP_THREAD_CoapSendDataResponse(pMessage, pMessageInfo);
		}

	} while (false);
}


static void APP_THREAD_CoapConfigHandler(void *pContext, otMessage *pMessage,
		const otMessageInfo *pMessageInfo) {
	do {
		//    if (otCoapMessageGetType(pMessage) != OT_COAP_TYPE_NON_CONFIRMABLE)
		//    {
		//      break;
		//    }

		if (otCoapMessageGetCode(pMessage) == OT_COAP_CODE_PUT) {

			uint16_t receivedBytes;
			receivedBytes = otMessageRead(pMessage,
					otMessageGetOffset(pMessage), buffer, sizeof(buffer));

			uint8_t status;
			/* Create a stream that reads from the buffer. */
			pb_istream_t stream = pb_istream_from_buffer(buffer, receivedBytes);
			/* Now we are ready to decode the message. */
			status = pb_decode(&stream, PACKET_FIELDS, &rxPacket);

			if (status) {

				switch (rxPacket.which_payload) {
				case PACKET_MARK_PACKET_TAG:
					//					state = osThreadGetState(markThreadId);
					//					if( (state != osThreadReady) || (state != osThreadRunning) || (state != osThreadInactive) || (state != osThreadBlocked) ){
					//						markThreadId = osThreadNew(triggerMark, &rxPacket.payload.mark_packet, &markTask_attributes);
					//					}
					break;
				case PACKET_SYSTEM_INFO_PACKET_TAG:
					break;
				case PACKET_CONFIG_PACKET_TAG:
					/* only update config if in slave mode and coming from master*/
					if (configPacket.payload.config_packet.network_state.slave_sync &&
						(rxPacket.payload.config_packet.network_state.master_node == 1)) {

						/* we require a header from the master */
						if(!rxPacket.has_header) break;

						/* check if we already received this packet before */
						if( (prev_master_epoch == rxPacket.header.epoch) &&
								(prev_master_uid == rxPacket.header.system_uid) &&
								(configPacket.payload.config_packet.enable_recording == rxPacket.payload.config_packet.enable_recording)){
							break;
						}

						/* if never received a config before, likely this is a fresh slave so send UWB info */
						if(prev_master_uid == 0) sendUWB_InfoToNode((peer_address_t*) &local_uwbInfo.uwb.address, pMessageInfo->mPeerAddr);

						prev_master_epoch = rxPacket.header.epoch;
						prev_master_uid = rxPacket.header.system_uid;

						/* update internal timestamp if a valid timestamp received */
						if(rxPacket.header.epoch > 1707866274000){
							updateRTC_MS(rxPacket.header.epoch);
						}


//						sendUWB_InfoToNode(
//								(peer_address_t*) &local_uwbInfo.uwb.address,
//								pMessageInfo->mPeerAddr);

						//						osThreadState_t state = osThreadGetState(configThreadId);
						//						if( (state != osThreadReady) || (state != osThreadRunning) || (state != osThreadInactive) || (state != osThreadBlocked) ){
						//							xTimerPendFunctionCallFromISR(createConfigThread, &rxPacket.payload.config_packet, NULL, pdFALSE);
						//						}
						osThreadState_t state = osThreadGetState(
								configThreadId);
						if ((state != osThreadReady)
								|| (state != osThreadRunning)
								|| (state != osThreadInactive)
								|| (state != osThreadBlocked)) {
							memcpy(&configMsg.config, &rxPacket.payload.config_packet, sizeof(config_packet_t));

							/* do not want to overwrite network config */
//							memcpy(&configMsg.config.network_state,&configPacket.payload.config_packet.network_state,sizeof(configMsg.config.network_state));

							configMsg.fromMaster = 1;
							osMessageQueuePut(configChangeQueueId, &configMsg, 0, 100);
//							configThreadId = osThreadNew(updateSystemConfig,
//									&configMsg,
//									&configTask_attributes);
						}
					}

					/* failure conditon: if another master node is broadcasting, demote yourself to slave_node */
					else if ((rxPacket.payload.config_packet.network_state.master_node
							== 1)
							&& (configPacket.payload.config_packet.network_state.master_node
									== 1)) {
						configPacket.payload.config_packet.network_state.slave_sync =
								1;
						configPacket.payload.config_packet.network_state.master_node =
								0;

						//						osThreadState_t state = osThreadGetState(configThreadId);
						//						if( (state != osThreadReady) || (state != osThreadRunning) || (state != osThreadInactive) || (state != osThreadBlocked) ){
						//							xTimerPendFunctionCallFromISR(createConfigThread, &rxPacket.payload.config_packet, NULL, pdFALSE);
						//						}
						osThreadState_t state = osThreadGetState(
								configThreadId);
						if ((state != osThreadReady)
								|| (state != osThreadRunning)
								|| (state != osThreadInactive)
								|| (state != osThreadBlocked)) {
							memcpy(&configMsg.config, &rxPacket.payload.config_packet, sizeof(config_packet_t));
							configMsg.fromMaster = 1;
							osMessageQueuePut(configChangeQueueId, &configMsg, 0, 100);
//							configThreadId = osThreadNew(updateSystemConfig,
//									&configMsg,
//									&configTask_attributes);
						}
					}
					break;
				case PACKET_SPECIAL_FUNCTION_TAG:
					switch (rxPacket.payload.special_function.which_payload) {
					case SPECIAL_FUNCTION_FORMAT_SDCARD_TAG:
						break;
					case SPECIAL_FUNCTION_CAMERA_CONTROL_TAG:
						if (rxPacket.payload.special_function.payload.camera_control.capture) {
							//todo: trigger any connected cameras
						} else if (rxPacket.payload.special_function.payload.camera_control.pair_with_nearby_cameras) {
							//todo: put device in pairing mode for any nearby cameras
						} else if (rxPacket.payload.special_function.payload.camera_control.wakeup_cameras) {
							//todo: wakeup any sleeping cameras
						}
						break;
					case SPECIAL_FUNCTION_UWB_PACKET_TAG:
						if (rxPacket.payload.special_function.payload.uwb_packet.start_ranging) {
							//todo: turn on UWB if not already on
						}
						if (rxPacket.payload.special_function.payload.uwb_packet.turn_on_uwb) {
							//todo: turn on UWB if not already on
						}
						if (rxPacket.payload.special_function.payload.uwb_packet.ranges_count
								> 0) {
							// update UWB entry
							for (int i = 0; i++;
									i < rxPacket.payload.special_function.payload.uwb_packet.ranges_count) {
								updateRangeTableUWB(rxPacket.header.system_uid,
										rxPacket.payload.special_function.payload.uwb_packet.ranges[i].system_uid,
										rxPacket.payload.special_function.payload.uwb_packet.ranges[i].range,
										rxPacket.payload.special_function.payload.uwb_packet.ranges[i].std_dev);
							}

							//								updateTable(rxPacket.payload.special_function.payload.uwb_packet.ranges[0]);
						}
						break;
					case SPECIAL_FUNCTION_OPENTHREAD_SYNC_TIME_TAG:
						if (configPacket.payload.config_packet.network_state.slave_sync) {
							if (rxPacket.header.epoch < 1707859083) {
								updateRTC_MS(rxPacket.header.epoch);
							}
							color.blue_val = 100;
							color.green_val = 100;
							color.red_val = 100;
							color.duration = 500;
							osMessageQueuePut(ledSeqQueueId, &color, 0, 0);
						}
						break;
					case SPECIAL_FUNCTION_MAG_CALIBRATION_TAG:
						break;
					case SPECIAL_FUNCTION_SLAVE_REQ_CONFIG_TAG:
						/* if a master node, unicast config and UWB information to a slave device */
						if (configPacket.payload.config_packet.network_state.master_node) {
							sendConfig(pMessageInfo->mPeerAddr);
							sendUWB_InfoToNode((peer_address_t*) &local_uwbInfo.uwb.address, pMessageInfo->mPeerAddr);
						}
						/* if slave device, unicast just uwb information */
						else{
							sendUWB_InfoToNode((peer_address_t*) &local_uwbInfo.uwb.address, pMessageInfo->mPeerAddr);
						}
						break;
					case SPECIAL_FUNCTION_TIMESTAMP_TAG:
						timestampSync.slave_epoch = getEpoch();
						timestampSync.master_epoch = rxPacket.header.epoch;

						fileWriteSync.msgType = TIMESTAMP_MSG;
						fileWriteSync.msgTime = timestampSync.master_epoch;
						fileWriteSync.msgLength = sizeof(timestampSync_t);
						memcpy(fileWriteSync.data, &timestampSync, sizeof(timestampSync_t));
//						osMessageQueuePut(timeSyncQueueId, &timestampSync, 0, 0);
						osMessageQueuePut(fileWriteQueueId, &fileWriteSync, 0, 0);

						color.blue_val = 100;
						color.green_val = 100;
						color.red_val = 100;
						color.duration = 500;
						osMessageQueuePut(ledSeqQueueId, &color, 0, 0);
						break;
					case SPECIAL_FUNCTION_UWB_INFO_TAG:

						if(NEW_ENTRY == addInfotoTableUWB(
								&rxPacket.payload.special_function.payload.uwb_info)){
							sendUWB_InfoToNode((peer_address_t*) &local_uwbInfo.uwb.address, pMessageInfo->mPeerAddr);
						}

						/* if message was a broadcast, the broadcasting node was just turned on */
						else if((pMessageInfo->mSockAddr.mFields.m8[0] == 0xff) &&
								(pMessageInfo->mSockAddr.mFields.m8[1] <= 0x03)){
							sendUWB_InfoToNode((peer_address_t*) &local_uwbInfo.uwb.address, pMessageInfo->mPeerAddr);
						}

						break;
					default:
						break;
					}
					default:
						break;
				}

			}

			if (strncmp(PayloadRead, "light", 5) == 0) {
				toggledGreen();
				//			osThreadFlagsSet(AdvUpdateProcessId, 1);

			} else {
				//		  printf("The string does not start with 'test'.\n");
			}

			break;
		}

		/* If Message is Confirmable, send response */
		if (otCoapMessageGetType(pMessage) == OT_COAP_TYPE_CONFIRMABLE) {
			APP_THREAD_CoapSendDataResponse(pMessage, pMessageInfo);
		}

	} while (false);

}

void updateTable(uwb_range_t newEntry) {

	for (int i = 0; i < sizeof(uwb_table); i++) {

		if (uwb_table.ranges[i].system_uid == newEntry.system_uid) {
			//update existing entry
			memcpy(&uwb_table.ranges[i], &newEntry, sizeof(newEntry));
			return;
		} else if (i == uwb_table.ranges_count) {
			//new entry
			memcpy(&uwb_table.ranges[i], &newEntry, sizeof(newEntry));
			uwb_table.ranges_count++;
			return;
		}
	}
}

void updateRangeTableUWB(uint32_t system_ID_1, uint32_t system_ID_2,
		float range, float std_dev) {
	for (int i = 0; i < MAX_UWB_RANGES; i++) {
		if ((rangesUWB[i].system_uid_1 == system_ID_1)
				&& (rangesUWB[i].system_uid_2 == system_ID_2)) {
			rangesUWB[i].range = range;
			rangesUWB[i].std_dev = std_dev;
			return;
		} else if ((rangesUWB[i].system_uid_1 == system_ID_1)
				&& (rangesUWB[i].system_uid_2 == system_ID_2)) {
			rangesUWB[i].range = range;
			rangesUWB[i].std_dev = std_dev;
			return;
		} else if (((rangesUWB[i].system_uid_1 == 0)
				&& (rangesUWB[i].system_uid_2 == 0))) {
			rangesUWB[i].system_uid_1 = system_ID_1;
			rangesUWB[i].system_uid_2 = system_ID_2;
			rangesUWB[i].range = range;
			rangesUWB[i].std_dev = std_dev;
			return;
		}
	}
}

uint32_t totalConnectedNodes(uwb_info_t *nodeInfo) {
	int i = 0;
	for (i = 0; i < MAX_UWB_RANGES; i++) {
		if (nodeInfo[i].system_uid == 0) {
			return (i);
		}
	}

	return MAX_UWB_RANGES;
}

/**
 * @brief This function acknowledges the data reception by sending an ACK
 *    back to the sender.
 * @param  pMessage coap message
 * @param  pMessageInfo message info pointer
 * @retval None
 */
static otMessage *pOT_MessageResponse = NULL;
static void APP_THREAD_CoapSendDataResponse(otMessage *pMessage,
		const otMessageInfo *pMessageInfo) {
	otError error = OT_ERROR_NONE;

	do {
		APP_DBG("APP_THREAD_CoapSendDataResponse");

		pOT_MessageResponse = otCoapNewMessage(NULL, NULL);
		if (pOT_MessageResponse == NULL) {
			APP_DBG("WARNING : pOT_MessageResponse = NULL ! -> exit now");
			break;
		}

		otCoapMessageInitResponse(pOT_MessageResponse, pMessage,
				OT_COAP_TYPE_ACKNOWLEDGMENT, OT_COAP_CODE_VALID);

		error = otCoapSendResponse(NULL, pOT_MessageResponse, pMessageInfo);
		if (error != OT_ERROR_NONE && pOT_MessageResponse != NULL) {
			otMessageFree(pOT_MessageResponse);
			//      APP_THREAD_Error(ERR_THREAD_COAP_DATA_RESPONSE,error);
		}
	} while (false);
}

/**
 * @brief Send a CoAP request with defined parameters.
 *
 * @param[in]  aCoapRessource   A pointer to a otCoapResource.
 * @param[in]  aCoapType        otCoapType.
 * @param[in]  aCoapCode        otCoapCode.
 * @param[in]  aStringAddress   A pointer to a NULL-terminated string representing the address. Example: "FF03::1" for Multicast.
 * @param[in]  aPeerAddress     A pointer to otIp6Address Peer Address.
 * @param[in]  aPayload         A pointer to payload.
 * @param[in]  aHandler         A pointer to CoAP response handler.
 * @param[in]  aContext         A pointer to application specific context.
 *
 * @retval none.
 */
static void APP_THREAD_CoapSendRequest(otCoapResource *aCoapRessource,
		otCoapType aCoapType, otCoapCode aCoapCode, const char *aStringAddress,
		const otIp6Address *aPeerAddress, uint8_t *aPayload, uint16_t Size,
		otCoapResponseHandler aHandler, void *aContext) {
	otError error = OT_ERROR_NONE;

	do {
		pOT_Message = otCoapNewMessage(NULL, NULL);
		if (pOT_Message == NULL) {
			APP_THREAD_Error(ERR_THREAD_COAP_NEW_MSG, error);
			break;
		}

		otCoapMessageInit(pOT_Message, aCoapType, aCoapCode);
		otCoapMessageAppendUriPathOptions(pOT_Message,
				aCoapRessource->mUriPath);
		otCoapMessageSetPayloadMarker(pOT_Message);

		if ((aPayload != NULL) && (Size > 0)) {
			error = otMessageAppend(pOT_Message, aPayload, Size);
			if (error != OT_ERROR_NONE) {
				Error_Handler();
				break;
			}
		} else {
			APP_DBG("APP_THREAD_CoapSendRequest: No payload passed");
		}

		memset(&OT_MessageInfo, 0, sizeof(OT_MessageInfo));
		OT_MessageInfo.mPeerPort = OT_DEFAULT_COAP_PORT;

		if ((aPeerAddress == NULL) && (aStringAddress != NULL)) {
			otIp6AddressFromString(aStringAddress, &OT_MessageInfo.mPeerAddr);
		} else if (aPeerAddress != NULL) {
			memcpy(&OT_MessageInfo.mPeerAddr, aPeerAddress,
					sizeof(OT_MessageInfo.mPeerAddr));
		} else {
			Error_Handler();
		}

		if (aCoapType == OT_COAP_TYPE_NON_CONFIRMABLE) {
			APP_DBG("aCoapType == OT_COAP_TYPE_NON_CONFIRMABLE");
			error = otCoapSendRequest(NULL, pOT_Message, &OT_MessageInfo,
					NULL,
					NULL);
		}
		if (aCoapType == OT_COAP_TYPE_CONFIRMABLE) {
			APP_DBG("aCoapType == OT_COAP_TYPE_CONFIRMABLE");
			error = otCoapSendRequest(NULL, pOT_Message, &OT_MessageInfo,
					aHandler, aContext);
		}
	} while (false);
	if (error != OT_ERROR_NONE && pOT_Message != NULL) {
		otMessageFree(pOT_Message);
		APP_THREAD_Error(ERR_THREAD_COAP_SEND_REQUEST, error);
	}
}

void sendConfigToNodes(bool record_enable) {
	osSemaphoreAcquire(txMsg_LockBinarySemId, osWaitForever);
	/* Create a stream that will write to our buffer. */
	pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
	configPacket.header.epoch = getEpoch();
	/* Now we are ready to encode the message! */
	bool prevState = configPacket.payload.config_packet.enable_recording;
	configPacket.payload.config_packet.enable_recording = record_enable;
	status = pb_encode(&stream, PACKET_FIELDS, &configPacket);

	configPacket.payload.config_packet.enable_recording = prevState;

	//todo: cant stop ISR's for bottom command so would be good to issue a high priority thread call for below
	APP_THREAD_CoapSendRequest(&OT_ConfigRessource,
			OT_COAP_TYPE_NON_CONFIRMABLE, OT_COAP_CODE_PUT,
			MULICAST_FTD_MED,
			NULL, buffer, stream.bytes_written,
			NULL,
			NULL);
	osSemaphoreRelease(txMsg_LockBinarySemId);
}

void sendTimeToNodes(void) {
	osSemaphoreAcquire(txMsg_LockBinarySemId, osWaitForever);

	//		memset(p, NULL, sizeof(txPacket));
	/* Create a stream that will write to our buffer. */
	pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
	/* Now we are ready to encode the message! */
	txPacket.which_payload = PACKET_SPECIAL_FUNCTION_TAG;
	txPacket.payload.special_function.which_payload =
			SPECIAL_FUNCTION_TIMESTAMP_TAG;
	txPacket.payload.special_function.payload.timestamp = true;
	txPacket.header.epoch = getEpoch();
	txPacket.header.ms_from_start = HAL_GetTick();
	status = pb_encode(&stream, PACKET_FIELDS, &txPacket);

	//	configPacket.which_payload = PACKET_CONFIG_PACKET_TAG;
	//		portEXIT_CRITICAL();

	//todo: cant stop ISR's for bottom command so would be good to issue a high priority thread call for below
	APP_THREAD_CoapSendRequest(&OT_ConfigRessource,
			OT_COAP_TYPE_NON_CONFIRMABLE, OT_COAP_CODE_PUT,
			MULICAST_FTD_MED,
			NULL, buffer, stream.bytes_written,
			NULL,
			NULL);

	color.blue_val = 100;
	color.green_val = 100;
	color.red_val = 100;
	color.duration = 500;
	osMessageQueuePut(ledSeqQueueId, &color, 0, 0);

	timestampSync.slave_epoch = txPacket.header.epoch;
	timestampSync.master_epoch = txPacket.header.epoch;
	osMessageQueuePut(timeSyncQueueId, &timestampSync, 0, 0);

	osSemaphoreRelease(txMsg_LockBinarySemId);

}

void sendUWB_InfoToNode(peer_address_t *peer_addr, otIp6Address addr) {
	//	portENTER_CRITICAL();
	/* Create a stream that will write to our buffer. */
	osSemaphoreAcquire(txMsg_LockBinarySemId, osWaitForever);

	pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
	/* Now we are ready to encode the message! */
	//		memset(&txPacket, NULL, sizeof(txPacket));
	txPacket.which_payload = PACKET_SPECIAL_FUNCTION_TAG;
	txPacket.has_header = true;
	txPacket.header.system_uid = LL_FLASH_GetUDN();
	txPacket.header.epoch = getEpoch();
	txPacket.payload.special_function.which_payload =
			SPECIAL_FUNCTION_UWB_INFO_TAG;
	txPacket.payload.special_function.payload.uwb_info.has_uwb_addr = true;
	memcpy(&txPacket.payload.special_function.payload.uwb_info.uwb_addr,
			peer_addr, sizeof(peer_address_t));
	txPacket.payload.special_function.payload.uwb_info.system_uid =
			txPacket.header.system_uid;

	status = pb_encode(&stream, PACKET_FIELDS, &txPacket);
	//	portEXIT_CRITICAL();

	//todo: cant stop ISR's for bottom command so would be good to issue a high priority thread call for below
	APP_THREAD_CoapSendRequest(&OT_ConfigRessource,
			OT_COAP_TYPE_NON_CONFIRMABLE, OT_COAP_CODE_PUT,
			NULL, &addr, buffer, stream.bytes_written,
			NULL,
			NULL);

	osSemaphoreRelease(txMsg_LockBinarySemId);

}

void sendUWB_InfoToNodes(peer_address_t *peer_addr) {
	//	portENTER_CRITICAL();
	/* Create a stream that will write to our buffer. */
	//	pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
	/* Now we are ready to encode the message! */
	//	memset(&txPacket, NULL, sizeof(txPacket));
	osSemaphoreAcquire(txMsg_LockBinarySemId, osWaitForever);

	pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

	txPacket.which_payload = PACKET_SPECIAL_FUNCTION_TAG;
	txPacket.has_header = true;
	txPacket.header.system_uid = LL_FLASH_GetUDN();
	txPacket.header.epoch = getEpoch();
	txPacket.payload.special_function.which_payload =
			SPECIAL_FUNCTION_UWB_INFO_TAG;
	txPacket.payload.special_function.payload.uwb_info.has_uwb_addr = true;
	memcpy(&txPacket.payload.special_function.payload.uwb_info.uwb_addr,
			peer_addr, sizeof(peer_address_t));
	txPacket.payload.special_function.payload.uwb_info.system_uid =
			txPacket.header.system_uid;

	status = pb_encode(&stream, PACKET_FIELDS, &txPacket);
	//	portEXIT_CRITICAL();

	//todo: cant stop ISR's for bottom command so would be good to issue a high priority thread call for below
	APP_THREAD_CoapSendRequest(&OT_ConfigRessource,
			OT_COAP_TYPE_NON_CONFIRMABLE, OT_COAP_CODE_PUT,
			MULICAST_FTD_MED,
			NULL, buffer, stream.bytes_written,
			NULL,
			NULL);

	osSemaphoreRelease(txMsg_LockBinarySemId);

}

void sendConfig(otIp6Address addr) {
	osSemaphoreAcquire(txMsg_LockBinarySemId, osWaitForever);
	/* Create a stream that will write to our buffer. */
	pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
	txPacket.header.epoch = getEpoch();
	/* Now we are ready to encode the message! */
	status = pb_encode(&stream, PACKET_FIELDS, &configPacket);
//	portEXIT_CRITICAL();

	//todo: cant stop ISR's for bottom command so would be good to issue a high priority thread call for below
	APP_THREAD_CoapSendRequest(&OT_ConfigRessource,
			OT_COAP_TYPE_NON_CONFIRMABLE, OT_COAP_CODE_PUT,
			NULL, &addr, buffer, stream.bytes_written,
			NULL,
			NULL);
	osSemaphoreRelease(txMsg_LockBinarySemId);
}

void alertMaster(void) {

	//	memset(&txPacket, NULL, sizeof(txPacket));
	osSemaphoreAcquire(txMsg_LockBinarySemId, osWaitForever);

	txPacket.which_payload = PACKET_SPECIAL_FUNCTION_TAG;
	txPacket.payload.special_function.which_payload =
			SPECIAL_FUNCTION_SLAVE_REQ_CONFIG_TAG;
	txPacket.payload.special_function.payload.slave_req_config = true;

	/* Create a stream that will write to our buffer. */
	pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
	/* Now we are ready to encode the message! */
	status = pb_encode(&stream, PACKET_FIELDS, &txPacket);

	APP_THREAD_CoapSendRequest(&OT_ConfigRessource,
			OT_COAP_TYPE_NON_CONFIRMABLE, OT_COAP_CODE_PUT,
			MULICAST_FTD_MED,
			NULL, buffer, stream.bytes_written,
			NULL,
			NULL);

	osSemaphoreRelease(txMsg_LockBinarySemId);

}

///* The callback function that will execute in the context of the daemon task.
//Note callback functions must all use this same prototype. */
//void vProcessInterface( void *pvParameter1, uint32_t ulParameter2 )
//{
//BaseType_t xInterfaceToService;
//
//    /* The interface that requires servicing is passed in the second
//    parameter.  The first parameter is not used in this case. */
//    xInterfaceToService = ( BaseType_t ) &ulParameter2(pvParameter1);
//
//    /* ...Perform the processing here... */
//}

//void createConfigThread(void *pvParameter1, uint32_t ulParameter2){
//	config_packet_t config;
//	memcpy(&config, pvParameter1, sizeof(config_packet_t));
//	osThreadState_t state = osThreadGetState(configThreadId);
//	if( (state != osThreadReady) || (state != osThreadRunning) || (state != osThreadInactive) || (state != osThreadBlocked) ){
//		configThreadId = osThreadNew(updateSystemConfig, &config, &configTask_attributes);
//	}
//}


/* returns 1 if new entry */
#include "dts.h"
#include "app_ble.h"
static DTS_STM_Payload_t PackedPayload;
static void updateTableInInfoPacketUWB(){
	int i = 0;
	for (i = 0; i < MAX_CONN_COUNT; i++) {
		if (connectedNodeInfo[i].system_uid == 0) {
			break;
		}else{
			infoPacket.payload.system_info_packet.discovered_devices[i].uid = connectedNodeInfo[i].system_uid;
			infoPacket.payload.system_info_packet.discovered_devices[i].range = 0;
		}
	}
	infoPacket.payload.system_info_packet.discovered_devices_count = i;

	osSemaphoreAcquire(txMsg_LockBinarySemId, osWaitForever);
	/* Create a stream that will write to our buffer. */
	pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
	/* Now we are ready to encode the message! */
	uint8_t status = pb_encode(&stream, PACKET_FIELDS, &infoPacket);
	PackedPayload.pPayload = (uint8_t*) buffer;
	PackedPayload.Length = stream.bytes_written;
    if(status) DTS_STM_UpdateChar(BUZZCAM_INFO_CHAR_UUID,(uint8_t*) &PackedPayload);
    osSemaphoreRelease(txMsg_LockBinarySemId);
}

uint8_t addInfotoTableUWB(uwb_info_t *new_info) {
	for (int i = 0; i < MAX_CONN_COUNT; i++) {
		if (connectedNodeInfo[i].system_uid == new_info->system_uid) {
			memcpy(&connectedNodeInfo[i], new_info, sizeof(uwb_info_t));
			return UPDATE_EXISTING_ENTRY;
		} else if (connectedNodeInfo[i].system_uid == 0) {
			memcpy(&connectedNodeInfo[i], new_info, sizeof(uwb_info_t));
			updateTableInInfoPacketUWB();
			return NEW_ENTRY;
		}
	}
	return MAX_UWB_DEV_REACHED;
}
/* USER CODE END FD_WRAP_FUNCTIONS */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
