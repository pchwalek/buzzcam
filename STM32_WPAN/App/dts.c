/**
 ******************************************************************************
 * @file    dts.c
 * @author  MCD Application Team
 * @brief   Data Transfer Service (Custom)
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "ble_common.h"
#include "ble.h"
#include "dbg_trace.h"
#include "dts.h"   

//#include "captivate_config.h"

//#include "main.h"

//#include "lp5523.h"

#include "time.h"
#include "rtc.h"

//#include "app_thread.h"
#include "app_conf.h"

#include "packet.h"

#include "fram.h"

#define UUID_128_SUPPORTED 0
#define	NUM_OF_CHARACTERISTICS 6 //https://community.st.com/s/question/0D50X00009XkYAvSAN/sensortile-bluenrgms-custom-service-aci

#define ENTER_DFU_MODE_UPON_RESET 1
#define ENTER_BLUE_GREEN_TRANSITION 2
#define ENTER_RED_FLASH_TRANSITION 3

#define PROTOBUF_RX	1

#ifdef PROTOBUF_RX
uint8_t rxBuffer[250];
static air_spec_config_packet_t rxConfigPacket = AIR_SPEC_CONFIG_PACKET_INIT_ZERO;
//union ColorComplex airspecColors;
#else
RX_PacketHeader rxPacketHeader;
#endif



#if (UUID_128_SUPPORTED == 1)
#define DT_UUID_LENGTH  UUID_TYPE_128
#else
#define DT_UUID_LENGTH  UUID_TYPE_16
#endif

const uint8_t DT_REQ_CHAR_INFO_UUID[2] = { 0x71, 0xCE };
const uint8_t DT_REQ_CHAR_CONFIG_UUID[2] = { 0x72, 0xCE };
const uint8_t DT_REQ_CHAR_RX_UUID[2] = { 0x73, 0xCE };
const uint8_t DT_REQ_SERV_UUID[2] = { 0x70, 0xCE };


/* Private typedef -----------------------------------------------------------*/
typedef enum {
	DTS_STM_NOTIFICATION_MASK = (1 << 0), DTS_STM_INDICATION_MASK = (1 << 1),
} ClientCharConfMask_t;

typedef struct {
	uint16_t DataTransferSvcHdle; /**< Service handle */
	uint16_t DataTransferRxCharHdle; /**< Characteristic handle */
	uint16_t DataTransferCharControlHdle; /**< Characteristic handle */
	uint16_t DataTransferCharInfoHdle;
} DataTransferSvcContext_t;

/* Private defines -----------------------------------------------------------*/
#define DATA_TRANSFER_NOTIFICATION_LEN_MAX                                 DATA_NOTIFICATION_MAX_PACKET_SIZE

/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static tBleStatus TX_Update_Char(DTS_STM_Payload_t *pDataValue);
static tBleStatus SensorConfig_Update_Char(DTS_STM_Payload_t *pDataValue);
static SVCCTL_EvtAckStatus_t DTS_Event_Handler(void *pckt);
static DataTransferSvcContext_t aDataTransferContext;
extern uint16_t Att_Mtu_Exchanged;

#ifdef PROTOBUF_RX
//light_control_packet_t receivedColor;
//static union ColorComplex receivedColor;
//blue_green_transition_t blueGreenTranRX;
//
//blink_calibration_t blinkCalRX;
//
//red_flash_task_t redFlashRX;
system_state_t tempState;
#else
struct LogMessage receivedCntrlPacket;
union ColorComplex receivedColor;
union BlueGreenTransition blueGreenTranRX;
union RedFlash redFlashRX;
#endif
uint8_t sensorCtrl[26];
time_t receivedEpoch;

/* Functions Definition ------------------------------------------------------*/
/* Private functions ----------------------------------------------------------*/

/**
 * @brief  Event handler
 * @param  Event: Address of the buffer holding the Event
 * @retval Ack: Return whether the Event has been managed or not
 */
volatile osThreadState_t threadState;
static SVCCTL_EvtAckStatus_t DTS_Event_Handler(void *Event) {
	SVCCTL_EvtAckStatus_t return_value;
	hci_event_pckt *event_pckt;
	evt_blue_aci *blue_evt;
	aci_gatt_attribute_modified_event_rp0 *attribute_modified;
	aci_att_exchange_mtu_resp_event_rp0 *exchange_mtu_resp;
	aci_gatt_write_permit_req_event_rp0 *write_permit_req;

	DTS_STM_App_Notification_evt_t Notification;

	return_value = SVCCTL_EvtNotAck;
	event_pckt = (hci_event_pckt*) (((hci_uart_pckt*) Event)->data);

	switch (event_pckt->evt) {
	case EVT_VENDOR: {
		blue_evt = (evt_blue_aci*) event_pckt->data;

		switch (blue_evt->ecode) {
		case EVT_BLUE_ATT_EXCHANGE_MTU_RESP:
#ifdef NUCLEO_LED_ACTIVE
        	 BSP_LED_On(LED_BLUE);
#endif
			APP_DBG_MSG("EVT_BLUE_ATT_EXCHANGE_MTU_RESP \n");
			exchange_mtu_resp =
					(aci_att_exchange_mtu_resp_event_rp0*) blue_evt->data;
			APP_DBG_MSG("MTU_size = %d \n",exchange_mtu_resp->Server_RX_MTU );
			Att_Mtu_Exchanged = exchange_mtu_resp->Server_RX_MTU;
#ifdef NUCLEO_LED_ACTIVE
     	 BSP_LED_On(LED_GREEN);
#endif
			break;
			/* server */
		case EVT_BLUE_GATT_ATTRIBUTE_MODIFIED: {
			attribute_modified =
					(aci_gatt_attribute_modified_event_rp0*) blue_evt->data;
//			if (attribute_modified->Attr_Handle
//					== (aDataTransferContext.DataTransferTxCharHdle + 2)) {
//				/**
//				 * Notify to application to start measurement
//				 */
//				if (attribute_modified->Attr_Data[0]
//						& DTS_STM_NOTIFICATION_MASK) {
//					APP_DBG_MSG("notification enabled\n");
//					Notification.Evt_Opcode = DTS_STM__NOTIFICATION_ENABLED;
//					DTS_Notification(&Notification);
//				} else {
//					APP_DBG_MSG("notification disabled\n");
//					Notification.Evt_Opcode = DTS_STM_NOTIFICATION_DISABLED;
//					DTS_Notification(&Notification);
//				}
//			}
//
//			if (attribute_modified->Attr_Handle
//					== (aDataTransferContext.DataTransferRxCharHdle + 1)) {
//				return_value = SVCCTL_EvtAckFlowEnable;
//
//				Notification.Evt_Opcode = DTS_STM_DATA_RECEIVED;
//				Notification.DataTransfered.Length =
//						attribute_modified->Attr_Data_Length;
//				DTS_Notification(&Notification);
//
//#ifdef PROTOBUF_RX
//
//				volatile uint8_t status;
//
//				/* Create a stream that reads from the buffer. */
//				pb_istream_t stream = pb_istream_from_buffer(attribute_modified->Attr_Data, attribute_modified->Attr_Data_Length);
//
//				/* Now we are ready to decode the message. */
//				status = pb_decode(&stream, AIR_SPEC_CONFIG_PACKET_FIELDS, &rxConfigPacket);
//
//				osMessageQueuePut(bleRX_QueueHandle, &rxConfigPacket, 0, 0);
//
//#else
//				// parse header
//						memcpy(&rxPacketHeader, attribute_modified->Attr_Data, sizeof(RX_PacketHeader));
//
//						/* make sure whatever mechanism is used below, it copies the memory of the payload
//								because there's a chance it can be overwritten before a thread can handle
//								the data */
//						if(rxPacketHeader.packetType == CONTROL_LED_PKT_TYPE){
//							osMessageQueuePut(lightsComplexQueueHandle,
//									(attribute_modified->Attr_Data + sizeof(RX_PacketHeader)), 0, 0);
//						}
//						else if(rxPacketHeader.packetType == SET_CLK_PKT_TYPE){
//							// do nothing since this will get handled at the end
//						}
//						else if(rxPacketHeader.packetType == CNTRL_SENSORS_PKT_TYPE){
//							memcpy(&sensorCtrl[0],
//									attribute_modified->Attr_Data + sizeof(RX_PacketHeader),
//									rxPacketHeader.payloadSize);
//							controlSensors(&sensorCtrl[0], rxPacketHeader.payloadSize / 2);
//						}
//						else if(rxPacketHeader.packetType == CONFIG_SENSORS_PKT_TYPE){
//
//						}
//						else if(rxPacketHeader.packetType == TRIGGER_FUNC_PKT_TYPE){
//							uint8_t functionIdentifier;
//							memcpy(&functionIdentifier,
//									attribute_modified->Attr_Data + sizeof(RX_PacketHeader),
//									sizeof(uint8_t));
//
//							if(functionIdentifier == ENTER_DFU_MODE_UPON_RESET){
//								ledEnterDFUNotification();
//								enterDFUMode();
//							}
//							else if(functionIdentifier == ENTER_BLUE_GREEN_TRANSITION){
//								memcpy(&blueGreenTranRX, attribute_modified->Attr_Data + sizeof(RX_PacketHeader) + 1, sizeof(union BlueGreenTransition));
//								osThreadTerminate(blueGreenTranTaskHandle); // terminate any existing running thread
//								resetLED();
//								if(blueGreenTranRX.val.start_bit == 1){
//									blueGreenTranTaskHandle = osThreadNew(BlueGreenTransitionTask, &blueGreenTranRX, &blueGreenTask_attributes);
//								}
//							}
//							else if(functionIdentifier == ENTER_RED_FLASH_TRANSITION){
//								memcpy(&redFlashRX, attribute_modified->Attr_Data + sizeof(RX_PacketHeader) + 1, sizeof(union RedFlash));
//								osThreadTerminate(redFlashTaskHandle); // terminate any existing running thread
//								resetLED();
//								if(redFlashRX.val_flash.start_bit == 1){
//									redFlashTaskHandle = osThreadNew(RedFlashTask, &redFlashRX, &redFlashTask_attributes);
//								}
//							}
//						}
//
//						// update RTC with header's timestamp
//						if(rxPacketHeader.epoch != 0){
//							updateRTC_MS(rxPacketHeader.epoch);
//						}
//#endif
//
//
//			}

//			if (attribute_modified->Attr_Handle
//								== (aDataTransferContext.DataTransferSensorConfigHdle + 1)) {
//
//				uint8_t status;
//
//				/* Create a stream that reads from the buffer. */
//				pb_istream_t stream = pb_istream_from_buffer(attribute_modified->Attr_Data, attribute_modified->Attr_Data_Length);
//
//				/* Now we are ready to decode the message. */
//				status = pb_decode(&stream, SYSTEM_STATE_FIELDS, &tempState);
//
//				// todo: theoretically, unnecessary to do another memcpy.
//				// Can integrate the top line with this one and remove tempState entirely.
//				// Worried about corruption.
//				memcpy(&sysState,&tempState,sizeof(system_state_t));
//
////				if(attribute_modified->Attr_Data_Length == sizeof(struct SensorConfig)){
//
//
////					memcpy(&sensorConfig, attribute_modified->Attr_Data, sizeof(struct SensorConfig));
//					ingestSensorConfig(&sysState);
//				    extMemWriteData(START_ADDR+4, (uint8_t*) &sysState, sizeof(system_state_t));
////				    if()
////					if(sensorConfig.epoch != 0){
////						updateRTC(sensorConfig.epoch);
////					}
////				};
//
//			}



//			// if LED characteristic was modified
//			if (attribute_modified->Attr_Handle
//					== (aDataTransferContext.DataTransferRxCharLedHdle + 1)) {
//#ifdef NUCLEO_LED_ACTIVE
//        	  	  	  	  BSP_LED_Toggle(LED_BLUE);
//#endif
//				memcpy(&receivedColor, attribute_modified->Attr_Data,
//						sizeof(receivedColor));
//				//FrontLightsSet(&receivedColor);
//				osMessageQueuePut(lightsComplexQueueHandle, &receivedColor, 0, 0);
//
//			}
//
//			// if system config was modified
//			if (attribute_modified->Attr_Handle
//					== (aDataTransferContext.DataTransferRxCharControlHdle + 1)) {
//#ifdef NUCLEO_LED_ACTIVE
//        	  BSP_LED_Toggle(LED_GREEN);
//#endif
//				memcpy(&receivedCntrlPacket, attribute_modified->Attr_Data,
//						sizeof(struct LogMessage));
//
////				osMessageQueuePut(togLoggingQueueHandle, &receivedCntrlPacket,
////						0U, 0U);
//
//			}
//
//			// if epoch was set
//			if (attribute_modified->Attr_Handle
//					== (aDataTransferContext.DataTransferRxCharTimeHdle + 1)) {
//#ifdef NUCLEO_LED_ACTIVE
//						  BSP_LED_Toggle(LED_GREEN);
//#endif
//				memcpy(&receivedEpoch, attribute_modified->Attr_Data,
//						sizeof(receivedEpoch));
//
//				receivedEpoch = receivedEpoch / 1000;
////				updateRTC(receivedEpoch);
//
//			}

		}
			break;
		case EVT_BLUE_GATT_TX_POOL_AVAILABLE:
//			Resume_Notification();
			break;

		case EVT_BLUE_GATT_WRITE_PERMIT_REQ:
			APP_DBG_MSG("write permit req\r\n");
			write_permit_req =
					(aci_gatt_write_permit_req_event_rp0*) blue_evt->data;
			aci_gatt_write_resp(write_permit_req->Connection_Handle,
					write_permit_req->Attribute_Handle, 0, 0,
					write_permit_req->Data_Length, write_permit_req->Data);
			break;

		default:
			break;
		}
	}
		break; /* HCI_EVT_VENDOR_SPECIFIC */

	default:
		break;
	}

	return (return_value);
}/* end SVCCTL_EvtAckStatus_t */

/**
 * @brief  Feature Characteristic update
 * @param  Service_Instance: Instance of the service to which the characteristic belongs
 * @param  pFeatureValue: The address of the new value to be written
 * @retval None
 */
//https://www.st.com/resource/en/programming_manual/pm0271-stm32wb-ble-stack-programming-guidelines-stmicroelectronics.pdf
# define MAX_PACKET_LENGTH	243 // https://www.compel.ru/wordpress/wp-content/uploads/2019/12/en.dm00598033.pdf
//# define MAX_PACKET_LENGTH	500 // https://www.compel.ru/wordpress/wp-content/uploads/2019/12/en.dm00598033.pdf

void enterDFUMode(void){
	*((unsigned long *)0x2000020c) = 0xDEADBEEF;
	__disable_irq();
	NVIC_SystemReset();
}

//static tBleStatus TX_Update_Char(DTS_STM_Payload_t *pDataValue) {
//	tBleStatus ret;
//
//	/**
//	 *  Notification Data Transfer Packet
//	 */
//	if(pDataValue->Length < MAX_PACKET_LENGTH){
//	  ret = aci_gatt_update_char_value(aDataTransferContext.DataTransferSvcHdle,
//			  aDataTransferContext.DataTransferTxCharHdle, 0, /* charValOffset */
//			  pDataValue->Length, /* charValueLen */
//			  (uint8_t*) pDataValue->pPayload);
//	}
//	else if(pDataValue->Length <= DATA_NOTIFICATION_MAX_PACKET_SIZE){
//
//	    uint16_t packetLen = pDataValue->Length;
//	    uint16_t offset = 0;
//
//	    while(packetLen > MAX_PACKET_LENGTH){
//	      aci_gatt_update_char_value_ext (0,
//				       aDataTransferContext.DataTransferSvcHdle,
//				       aDataTransferContext.DataTransferTxCharHdle,
//					0x00, //dont notify
//					pDataValue->Length,
//					offset,
//					MAX_PACKET_LENGTH,
//					((uint8_t*) pDataValue->pPayload) + offset);
//	      offset += MAX_PACKET_LENGTH;
//	      packetLen -= MAX_PACKET_LENGTH;
//	    }
//
//	    ret = aci_gatt_update_char_value_ext (0,
//	    					     aDataTransferContext.DataTransferSvcHdle,
//	    					     aDataTransferContext.DataTransferTxCharHdle,
//	    	0x01, //notify
//		pDataValue->Length,
//		offset,
//		packetLen,
//	    	((uint8_t*) pDataValue->pPayload) + offset);
//	}
//
//	return ret;
//}/* end TX_Update_Char() */

//static tBleStatus SensorConfig_Update_Char(DTS_STM_Payload_t *pDataValue) {
//	tBleStatus ret;
//
//	if(!aDataTransferContext.DataTransferSvcHdle){
//		return PACKET_UNDEFINED_ERR;
//	}
//	/**
//	 *  Notification Data Transfer Packet
//	 */
//	if(pDataValue->Length < MAX_PACKET_LENGTH){
//	  ret = aci_gatt_update_char_value(aDataTransferContext.DataTransferSvcHdle,
//			  aDataTransferContext.DataTransferSensorConfigHdle, 0, /* charValOffset */
//			  pDataValue->Length, /* charValueLen */
//			  (uint8_t*) pDataValue->pPayload);
//	}
////	else if(pDataValue->Length <= DATA_NOTIFICATION_MAX_PACKET_SIZE){
////
////	    uint16_t packetLen = pDataValue->Length;
////	    uint16_t offset = 0;
////
////	    while(packetLen > MAX_PACKET_LENGTH){
////	      aci_gatt_update_char_value_ext (0,
////				       aDataTransferContext.DataTransferSvcHdle,
////				       aDataTransferContext.DataTransferSensorConfigHdle,
////					0x00, //dont notify
////					pDataValue->Length,
////					offset,
////					MAX_PACKET_LENGTH,
////					((uint8_t*) pDataValue->pPayload) + offset);
////	      offset += MAX_PACKET_LENGTH;
////	      packetLen -= MAX_PACKET_LENGTH;
////	    }
////
////	    ret = aci_gatt_update_char_value_ext (0,
////	    					     aDataTransferContext.DataTransferSvcHdle,
////	    					     aDataTransferContext.DataTransferTxCharHdle,
////	    	0x01, //notify
////		pDataValue->Length,
////		offset,
////		packetLen,
////	    	((uint8_t*) pDataValue->pPayload) + offset);
////	}
//
//	return ret;
//}/* end TX_Update_Char() */

/* Public functions ----------------------------------------------------------*/

/**
 * @brief  Service initialization
 * @param  None
 * @retval None
 */
void DTS_STM_Init(void) {
	tBleStatus hciCmdResult = BLE_STATUS_FAILED;

	/**
	 *	Register the event handler to the BLE controller
	 */
	SVCCTL_RegisterSvcHandler(DTS_Event_Handler);

	/* DT service and characteristics */
//	hciCmdResult = aci_gatt_add_service(DT_UUID_LENGTH,
//				(Service_UUID_t*) DATA_TRANSFER_SERVICE_UUID,
//				PRIMARY_SERVICE, 1 + 3 * NUM_OF_CHARACTERISTICS,
//				&(aDataTransferContext.DataTransferSvcHdle));
	hciCmdResult = aci_gatt_add_service(DT_UUID_LENGTH,
			(Service_UUID_t*) DT_REQ_SERV_UUID,
			PRIMARY_SERVICE, 1 + 3 * NUM_OF_CHARACTERISTICS,
			&(aDataTransferContext.DataTransferSvcHdle));
	if (hciCmdResult != 0) {
		APP_DBG_MSG("error add service 0x%x\n", hciCmdResult);
#ifdef NUCLEO_LED_ACTIVE
    BSP_LED_On(LED_RED);
#endif
	}

	/**
	 *  Add info Characteristic
	 */
	hciCmdResult = aci_gatt_add_char(aDataTransferContext.DataTransferSvcHdle,
	DT_UUID_LENGTH, (Char_UUID_t*) DT_REQ_CHAR_INFO_UUID,
	DATA_TRANSFER_NOTIFICATION_LEN_MAX,
	CHAR_PROP_NOTIFY | CHAR_PROP_READ,
	ATTR_PERMISSION_NONE,
	GATT_NOTIFY_ATTRIBUTE_WRITE, /* gattEvtMask */
	10, /* encryKeySize */
	1, /* isVariable */
	&(aDataTransferContext.DataTransferCharInfoHdle));

	if (hciCmdResult != 0) {
		APP_DBG_MSG("error add char Tx 0x%x\n", hciCmdResult);
#ifdef NUCLEO_LED_ACTIVE
    BSP_LED_On(LED_RED);
#endif
	}

	/**
	 *  Add Data Transfer RX Characteristic (not intended to be used in the end)
	 */
	hciCmdResult = aci_gatt_add_char(aDataTransferContext.DataTransferSvcHdle,
	DT_UUID_LENGTH, (Char_UUID_t*) DT_REQ_CHAR_CONFIG_UUID, DATA_TRANSFER_NOTIFICATION_LEN_MAX, /* DATA_TRANSFER_NOTIFICATION_LEN_MAX, */
	CHAR_PROP_NOTIFY | CHAR_PROP_READ,
	ATTR_PERMISSION_NONE,
	GATT_NOTIFY_ATTRIBUTE_WRITE, //GATT_NOTIFY_WRITE_REQ_AND_WAIT_FOR_APPL_RESP, /* gattEvtMask */
			10, /* encryKeySize */
			1, /* isVariable */
			&(aDataTransferContext.DataTransferCharControlHdle));

	if (hciCmdResult != 0) {
		APP_DBG_MSG("error add char Tx\n");

#ifdef NUCLEO_LED_ACTIVE
    BSP_LED_On(LED_RED);
#endif
	}

	hciCmdResult = aci_gatt_add_char(aDataTransferContext.DataTransferSvcHdle,
	DT_UUID_LENGTH, (Char_UUID_t*) DT_REQ_CHAR_RX_UUID, DATA_TRANSFER_NOTIFICATION_LEN_MAX, /* DATA_TRANSFER_NOTIFICATION_LEN_MAX, */
	CHAR_PROP_WRITE_WITHOUT_RESP | CHAR_PROP_NOTIFY | CHAR_PROP_READ,
	ATTR_PERMISSION_NONE,
	GATT_NOTIFY_ATTRIBUTE_WRITE, //GATT_NOTIFY_WRITE_REQ_AND_WAIT_FOR_APPL_RESP, /* gattEvtMask */
			10, /* encryKeySize */
			1, /* isVariable */
			&(aDataTransferContext.DataTransferRxCharHdle));

//	updateSystemConfig_BLE(&sysState);

	if (hciCmdResult != 0) {
		APP_DBG_MSG("error add char Tx\n");

#ifdef NUCLEO_LED_ACTIVE
    BSP_LED_On(LED_RED);
#endif
	}

	return;
}

void UpdateSystemInfo(DTS_STM_Payload_t* pPayload){

}

void UpdateConfig(DTS_STM_Payload_t* pPayload){

}

/**
 * @brief  Characteristic update
 * @param  UUID: UUID of the characteristic
 * @param  Service_Instance: Instance of the service to which the characteristic belongs
 * 
 */
tBleStatus DTS_STM_UpdateChar(uint16_t UUID, uint8_t *pPayload) {
	tBleStatus result = BLE_STATUS_INVALID_PARAMS;
	switch (UUID) {
	case BUZZCAM_INFO_CHAR_UUID:
		result = UpdateSystemInfo((DTS_STM_Payload_t*) pPayload);
	case BUZZCAM_CONFIG_CHAR_UUID:
		result = UpdateConfig((DTS_STM_Payload_t*) pPayload);
	default:
		break;
	}
	return result;
}/* end DTS_STM_UpdateChar() */


//tBleStatus DTS_STM_UpdateCharThroughput(DTS_STM_Payload_t *pDataValue) {
//	tBleStatus result = BLE_STATUS_INVALID_PARAMS;
//	/**
//	 *  Notification Data Transfer Packet
//	 */
//	result = aci_gatt_update_char_value(
//			aDataTransferContext.DataTransferSvcHdle,
//			aDataTransferContext.DataTransferTxChar3Hdle, 0, /* charValOffset */
//			pDataValue->Length, /* charValueLen */
//			(uint8_t*) pDataValue->pPayload);
//	return result;
//}/* end DTS_STM_UpdateChar() */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
