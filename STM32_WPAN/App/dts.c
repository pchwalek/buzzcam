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
//#include "rtc.h"

//#include "app_thread.h"
#include "app_conf.h"

#include "app_ble.h"

#include "cmsis_os2.h"

//#include "packet.h"

//#include "fram.h"

#include "pb.h"
#include "pb_decode.h"
#include "pb_encode.h"
#include "message.pb.h"

#include "main.h"

static DTS_STM_Payload_t PackedPayload;

#define UUID_128_SUPPORTED 0
#define	NUM_OF_CHARACTERISTICS 6 //https://community.st.com/s/question/0D50X00009XkYAvSAN/sensortile-bluenrgms-custom-service-aci

#define ENTER_DFU_MODE_UPON_RESET 1
#define ENTER_BLUE_GREEN_TRANSITION 2
#define ENTER_RED_FLASH_TRANSITION 3

#define PROTOBUF_RX	1

#ifdef PROTOBUF_RX
uint8_t rxBuffer[250];

//union ColorComplex airspecColors;
#else
RX_PacketHeader rxPacketHeader;
#endif



#if (UUID_128_SUPPORTED == 1)
#define DT_UUID_LENGTH  UUID_TYPE_128
#else
#define DT_UUID_LENGTH  UUID_TYPE_16
#endif


const uint8_t SHUTTER_CODE[9] = { 0xFC, 0xEF, 0xFE, 0x86, 0x00, 0x03, 0x01, 0x02, 0x00};
const uint8_t MODE_CODE[9] = { 0xFC, 0xEF, 0xFE, 0x86, 0x00, 0x03, 0x01, 0x01, 0x00};
const uint8_t POWER_SHORT_PRESS_CODE[9] = { 0xFC, 0xEF, 0xFE, 0x86, 0x00, 0x03, 0x01, 0x00, 0x00};
const uint8_t POWER_LONG_PRESS_CODE[9] = { 0xFC, 0xEF, 0xFE, 0x86, 0x00, 0x03, 0x01, 0x00, 0x03};


const uint8_t DT_REQ_CHAR_INFO_UUID[2] = { 0x71, 0xCE };
const uint8_t DT_REQ_CHAR_CONFIG_UUID[2] = { 0x72, 0xCE };
const uint8_t DT_REQ_CHAR_RX_UUID[2] = { 0x73, 0xCE };
//const uint8_t DT_REQ_SERV_UUID[2] = { 0x70, 0xCE };
const uint8_t DT_REQ_SERV_UUID[2] = { 0x80, 0xCE };
const uint8_t DT_CAMERA_CTRL_UUID[2] = { 0x82, 0xCE };
const uint8_t DT_CAMERA_CE81_UUID[2] = { 0x81, 0xCE };
const uint8_t DT_CAMERA_CE83_UUID[2] = { 0x83, 0xCE };

/* Private typedef -----------------------------------------------------------*/
typedef enum {
	DTS_STM_NOTIFICATION_MASK = (1 << 0), DTS_STM_INDICATION_MASK = (1 << 1),
} ClientCharConfMask_t;

typedef struct {
	uint16_t DataTransferSvcHdle; /**< Service handle */
	uint16_t DataTransferRxCharHdle; /**< Characteristic handle */
	uint16_t DataTransferCharControlHdle; /**< Characteristic handle */
	uint16_t DataTransferCharInfoHdle;
	uint16_t DataCameraCtrlCharHdle;
	uint16_t DataCameraCE81CharHdle;
	uint16_t DataCameraCE83CharHdle;
} DataTransferSvcContext_t;

/* Private defines -----------------------------------------------------------*/
#define DATA_TRANSFER_NOTIFICATION_LEN_MAX                                 DATA_NOTIFICATION_MAX_PACKET_SIZE

/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
//static tBleStatus TX_Update_Char(DTS_STM_Payload_t *pDataValue);
//static tBleStatus SensorConfig_Update_Char(DTS_STM_Payload_t *pDataValue);
static tBleStatus BLE_UpdateSystemInfo(DTS_STM_Payload_t* pPayload);
static tBleStatus BLE_UpdateConfig(DTS_STM_Payload_t* pPayload);
static SVCCTL_EvtAckStatus_t DTS_Event_Handler(void *pckt);
static DataTransferSvcContext_t aDataTransferContext;
extern uint16_t Att_Mtu_Exchanged;

static configChange configMsg;

#ifdef PROTOBUF_RX
//light_control_packet_t receivedColor;
//static union ColorComplex receivedColor;
//blue_green_transition_t blueGreenTranRX;
//
//blink_calibration_t blinkCalRX;
//
//red_flash_task_t redFlashRX;
//system_state_t tempState;
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
	evt_blecore_aci *blue_evt;
	aci_gatt_attribute_modified_event_rp0 *attribute_modified;
	aci_att_exchange_mtu_resp_event_rp0 *exchange_mtu_resp;
	aci_gatt_write_permit_req_event_rp0 *write_permit_req;

	DTS_STM_App_Notification_evt_t Notification;

	return_value = SVCCTL_EvtNotAck;
	event_pckt = (hci_event_pckt*) (((hci_uart_pckt*) Event)->data);

	switch (event_pckt->evt) {
	case HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE: {
		blue_evt = (evt_blecore_aci*) event_pckt->data;

		switch (blue_evt->ecode) {
		case ACI_ATT_EXCHANGE_MTU_RESP_VSEVT_CODE:
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
		case ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE: {
			attribute_modified =
					(aci_gatt_attribute_modified_event_rp0*) blue_evt->data;
			if (attribute_modified->Attr_Handle
					== (aDataTransferContext.DataTransferSvcHdle + 5)) {
				/* received update */
				volatile uint8_t status;

				/* Create a stream that reads from the buffer. */
				pb_istream_t stream = pb_istream_from_buffer(attribute_modified->Attr_Data, attribute_modified->Attr_Data_Length);

				/* Now we are ready to decode the message. */
				status = pb_decode(&stream, PACKET_FIELDS, &rxPacket);

				osThreadState_t state;
				if(status){
					if(rxPacket.header.epoch > 1707866274000){
						updateRTC_MS(rxPacket.header.epoch);
					}

					// update system
					switch(rxPacket.which_payload){
					case PACKET_MARK_PACKET_TAG:
						state = osThreadGetState(markThreadId);
						if( (state != osThreadReady) || (state != osThreadRunning) || (state != osThreadInactive) || (state != osThreadBlocked) ){
							osMessageQueuePut (markPacketQueueId, &rxPacket.payload.mark_packet, 0, 0);
						}
						break;
					case PACKET_SYSTEM_INFO_PACKET_TAG:
						/* external systems shouldn't update system info */
						break;
					case PACKET_CONFIG_PACKET_TAG:
						/* start thread that interrupts systems, updates parameters, and restarts system */
						state = osThreadGetState(configThreadId);
						if( (state != osThreadReady) || (state != osThreadRunning) || (state != osThreadInactive) || (state != osThreadBlocked) ){
							memcpy(&configMsg.config, &rxPacket.payload.config_packet, sizeof(config_packet_t));
							configMsg.fromMaster = 0;
							configThreadId = osThreadNew(updateSystemConfig,
									&configMsg,
									&configTask_attributes);
						}
						break;
					case PACKET_SPECIAL_FUNCTION_TAG:
						switch(rxPacket.payload.special_function.which_payload){
						case SPECIAL_FUNCTION_FORMAT_SDCARD_TAG:
							if(rxPacket.payload.special_function.payload.format_sdcard){
								//todo: format any connected SD cards (WARNING: stop all threads to do this)
							}
							break;
						case SPECIAL_FUNCTION_CAMERA_CONTROL_TAG:
							if(rxPacket.payload.special_function.payload.camera_control.capture){
								//todo: trigger any connected cameras
								DTS_CamCtrl(SHUTTER);
							}
							else if(rxPacket.payload.special_function.payload.camera_control.pair_with_nearby_cameras){
								//todo: put device in pairing mode for any nearby cameras
								activateCameraMode();
							}
							else if(rxPacket.payload.special_function.payload.camera_control.wakeup_cameras){
								//todo: wakeup any sleeping cameras
								DTS_CamCtrl(WAKEUP_CAMERAS);
							}
							break;
						case SPECIAL_FUNCTION_UWB_PACKET_TAG:
							if(rxPacket.payload.special_function.payload.uwb_packet.start_ranging){
								//todo: change this to UWB_START_RANGING and also populate "rangingAddr"
								osThreadFlagsSet(uwbMessageTaskId, UWB_START_RANGING);
							}
							break;
						case SPECIAL_FUNCTION_OPENTHREAD_SYNC_TIME_TAG:
							if(rxPacket.payload.special_function.payload.openthread_sync_time){
								//todo: implement function to sync time from this node with all other nodes via OT
								sendConfigToNodes();
							}
							break;
						case SPECIAL_FUNCTION_MAG_CALIBRATION_TAG:
							if(rxPacket.payload.special_function.payload.mag_calibration){
								//todo: implement mag calibration
								osThreadFlagsSet(mainSystemThreadId, MAG_CAL_EVENT);
							}
							break;
						case SPECIAL_FUNCTION_SLAVE_REQ_CONFIG_TAG:
							//todo
							break;
						case SPECIAL_FUNCTION_TIMESTAMP_TAG:
							//todo
							break;
						case SPECIAL_FUNCTION_DFU_MODE_TAG:
							enterDFUMode();
							break;
						default: break;
						}
					default: break;
					}
				}else{
					// decode failed. Invalid packet likely received.

				}

			}
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
		case ACI_GATT_TX_POOL_AVAILABLE_VSEVT_CODE:
//			Resume_Notification();
			break;

		case ACI_GATT_WRITE_PERMIT_REQ_VSEVT_CODE:
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
//	hciCmdResult = aci_gatt_add_service(UUID_TYPE_16, (Service_UUID_t*) DT_SERVICE_UUID,
//		PRIMARY_SERVICE, 8, &(aDataTransferContext.DataTransferSvcHdle));
//
//		if (hciCmdResult != 0) {
//			APP_DBG_MSG("error add service 0x%x\n", hciCmdResult);
//	#ifdef NUCLEO_LED_ACTIVE
//	    BSP_LED_On(LED_RED);
//	#endif
//		}
//		/**
//		 *  Add RX Characteristic
//		 */
//	hciCmdResult = aci_gatt_add_char(aDataTransferContext.DataTransferSvcHdle,
//		UUID_TYPE_128, DT_REQ_CHAR_INFO_UUID, 247,
//		CHAR_PROP_WRITE,
//		ATTR_PERMISSION_NONE,
//		GATT_NOTIFY_ATTRIBUTE_WRITE | GATT_NOTIFY_WRITE_REQ_AND_WAIT_FOR_APPL_RESP, /* gattEvtMask */
//		10, /* encryKeySize */
//		1, /* isVariable */
//		&(aDataTransferContext.DataTransferCharInfoHdle));
//
//		if (hciCmdResult != 0) {
//			APP_DBG_MSG("error add service 0x%x\n", hciCmdResult);
//	#ifdef NUCLEO_LED_ACTIVE
//	    BSP_LED_On(LED_RED);
//	#endif
//		}

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
		Error_Handler();
#ifdef NUCLEO_LED_ACTIVE
    BSP_LED_On(LED_RED);
#endif
	}

	hciCmdResult = aci_gatt_add_char(aDataTransferContext.DataTransferSvcHdle,
	DT_UUID_LENGTH, (Char_UUID_t*) DT_REQ_CHAR_CONFIG_UUID, DATA_TRANSFER_NOTIFICATION_LEN_MAX, /* DATA_TRANSFER_NOTIFICATION_LEN_MAX, */
	CHAR_PROP_NOTIFY | CHAR_PROP_READ | CHAR_PROP_WRITE | CHAR_PROP_WRITE_WITHOUT_RESP,
	ATTR_PERMISSION_NONE,
	GATT_NOTIFY_ATTRIBUTE_WRITE, //GATT_NOTIFY_WRITE_REQ_AND_WAIT_FOR_APPL_RESP, /* gattEvtMask */
			10, /* encryKeySize */
			1, /* isVariable */
			&(aDataTransferContext.DataTransferCharControlHdle));

	if (hciCmdResult != 0) {
		APP_DBG_MSG("error add char Tx\n");
		Error_Handler();
#ifdef NUCLEO_LED_ACTIVE
    BSP_LED_On(LED_RED);
#endif
	}


//	/**
//	 *  Add Data Transfer RX Characteristic (not intended to be used in the end)
//	 */
//	hciCmdResult = aci_gatt_add_char(aDataTransferContext.DataTransferSvcHdle,
//	DT_UUID_LENGTH, (Char_UUID_t*) DT_REQ_CHAR_RX_UUID, DATA_TRANSFER_NOTIFICATION_LEN_MAX, /* DATA_TRANSFER_NOTIFICATION_LEN_MAX, */
////	CHAR_PROP_WRITE_WITHOUT_RESP,
//	CHAR_PROP_WRITE,
//	ATTR_PERMISSION_NONE,
//	GATT_NOTIFY_ATTRIBUTE_WRITE, //GATT_NOTIFY_WRITE_REQ_AND_WAIT_FOR_APPL_RESP, /* gattEvtMask */
//			10, /* encryKeySize */
//			1, /* isVariable */
//			&(aDataTransferContext.DataTransferRxCharHdle));
//
////	updateSystemConfig_BLE(&sysState);


	/* add camera control characteristic */

	//CE81 (WRITE)
	hciCmdResult = aci_gatt_add_char(aDataTransferContext.DataTransferSvcHdle,
	DT_UUID_LENGTH, (Char_UUID_t*) DT_CAMERA_CE81_UUID, 20, /* DATA_TRANSFER_NOTIFICATION_LEN_MAX, */
//	CHAR_PROP_WRITE_WITHOUT_RESP,
	CHAR_PROP_WRITE,
	ATTR_PERMISSION_NONE,
	GATT_NOTIFY_ATTRIBUTE_WRITE, //GATT_NOTIFY_WRITE_REQ_AND_WAIT_FOR_APPL_RESP, /* gattEvtMask */
			10, /* encryKeySize */
			1, /* isVariable */
			&(aDataTransferContext.DataCameraCE81CharHdle));


	//CE83 (READ) -> curr value: 0x0102
	hciCmdResult = aci_gatt_add_char(aDataTransferContext.DataTransferSvcHdle,
	DT_UUID_LENGTH, (Char_UUID_t*) DT_CAMERA_CE83_UUID, 20, /* DATA_TRANSFER_NOTIFICATION_LEN_MAX, */
//	CHAR_PROP_WRITE_WITHOUT_RESP,
	CHAR_PROP_READ,
	ATTR_PERMISSION_NONE,
	GATT_NOTIFY_ATTRIBUTE_WRITE, //GATT_NOTIFY_WRITE_REQ_AND_WAIT_FOR_APPL_RESP, /* gattEvtMask */
			10, /* encryKeySize */
			1, /* isVariable */
			&(aDataTransferContext.DataCameraCE83CharHdle));

	tBleStatus ret;
	uint8_t stock_val[4] = {1,2};
	ret = aci_gatt_update_char_value(aDataTransferContext.DataTransferSvcHdle,
					  aDataTransferContext.DataCameraCE83CharHdle, 0, /* charValOffset */
					  2, /* charValueLen */
					  (uint8_t*) stock_val);

	//CE82
	hciCmdResult = aci_gatt_add_char(aDataTransferContext.DataTransferSvcHdle,
	DT_UUID_LENGTH, (Char_UUID_t*) DT_CAMERA_CTRL_UUID, 60, /* DATA_TRANSFER_NOTIFICATION_LEN_MAX, */
//	CHAR_PROP_WRITE_WITHOUT_RESP,
	CHAR_PROP_NOTIFY | CHAR_PROP_READ,
	ATTR_PERMISSION_NONE,
	GATT_NOTIFY_ATTRIBUTE_WRITE, //GATT_NOTIFY_WRITE_REQ_AND_WAIT_FOR_APPL_RESP, /* gattEvtMask */
			10, /* encryKeySize */
			1, /* isVariable */
			&(aDataTransferContext.DataCameraCtrlCharHdle));

    DTS_CamCtrl(SHUTTER);

	/* Create a stream that will write to our buffer. */
	pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
	/* Now we are ready to encode the message! */
	status = pb_encode(&stream, PACKET_FIELDS, &infoPacket);
	PackedPayload.pPayload = (uint8_t*) buffer;
	PackedPayload.Length = stream.bytes_written;
    if(status) ret = DTS_STM_UpdateChar(BUZZCAM_INFO_CHAR_UUID, &PackedPayload);

	/* Create a stream that will write to our buffer. */
	stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
	/* Now we are ready to encode the message! */
	status = pb_encode(&stream, PACKET_FIELDS, &configPacket);
	PackedPayload.pPayload = (uint8_t*) buffer;
	PackedPayload.Length = stream.bytes_written;
    if(status) ret = DTS_STM_UpdateChar(BUZZCAM_CONFIG_CHAR_UUID,&PackedPayload);


	if (hciCmdResult != 0) {
		APP_DBG_MSG("error add char Tx\n");

#ifdef NUCLEO_LED_ACTIVE
    BSP_LED_On(LED_RED);
#endif
	}

	return;
}

static tBleStatus BLE_UpdateSystemInfo(DTS_STM_Payload_t* pPayload){
	tBleStatus ret;

	if(pPayload->Length < MAX_PACKET_LENGTH){
		  ret = aci_gatt_update_char_value(aDataTransferContext.DataTransferSvcHdle,
				  aDataTransferContext.DataTransferCharInfoHdle, 0, /* charValOffset */
				  pPayload->Length, /* charValueLen */
				  (uint8_t*) pPayload->pPayload);
		}
		else if(pPayload->Length <= DATA_NOTIFICATION_MAX_PACKET_SIZE){

		    uint16_t packetLen = pPayload->Length;
		    uint16_t offset = 0;

		    while(packetLen > MAX_PACKET_LENGTH){
		      aci_gatt_update_char_value_ext (0,
					       aDataTransferContext.DataTransferSvcHdle,
					       aDataTransferContext.DataTransferCharInfoHdle,
						0x00, //dont notify
						pPayload->Length,
						offset,
						MAX_PACKET_LENGTH,
						((uint8_t*) pPayload->pPayload) + offset);
		      offset += MAX_PACKET_LENGTH;
		      packetLen -= MAX_PACKET_LENGTH;
		    }

		    ret = aci_gatt_update_char_value_ext (0,
			 aDataTransferContext.DataTransferSvcHdle,
			 aDataTransferContext.DataTransferCharInfoHdle,
		    	0x01, //notify
				pPayload->Length,
			offset,
			packetLen,
		    	((uint8_t*) pPayload->pPayload) + offset);
		}

		return ret;
}

static tBleStatus BLE_UpdateConfig(DTS_STM_Payload_t* pPayload){
	tBleStatus ret;


	if(pPayload->Length < MAX_PACKET_LENGTH){
		  ret = aci_gatt_update_char_value(aDataTransferContext.DataTransferSvcHdle,
				  aDataTransferContext.DataTransferCharControlHdle, 0, /* charValOffset */
				  pPayload->Length, /* charValueLen */
				  (uint8_t*) pPayload->pPayload);
		}
		else if(pPayload->Length <= DATA_NOTIFICATION_MAX_PACKET_SIZE){

		    uint16_t packetLen = pPayload->Length;
		    uint16_t offset = 0;

		    while(packetLen > MAX_PACKET_LENGTH){
		      aci_gatt_update_char_value_ext (0,
					       aDataTransferContext.DataTransferSvcHdle,
					       aDataTransferContext.DataTransferCharControlHdle,
						0x00, //dont notify
						pPayload->Length,
						offset,
						MAX_PACKET_LENGTH,
						((uint8_t*) pPayload->pPayload) + offset);
		      offset += MAX_PACKET_LENGTH;
		      packetLen -= MAX_PACKET_LENGTH;
		    }

		    ret = aci_gatt_update_char_value_ext (0,
				 aDataTransferContext.DataTransferSvcHdle,
				 aDataTransferContext.DataTransferCharControlHdle,
		    	0x01, //notify
				pPayload->Length,
			offset,
			packetLen,
		    	((uint8_t*) pPayload->pPayload) + offset);
		}

		return ret;
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
		result = BLE_UpdateSystemInfo((DTS_STM_Payload_t*) pPayload);
		break;
	case BUZZCAM_CONFIG_CHAR_UUID:
		result = BLE_UpdateConfig((DTS_STM_Payload_t*) pPayload);
		break;
	default:
		break;
	}
	return result;
}/* end DTS_STM_UpdateChar() */


tBleStatus DTS_CamCtrl(cameraFn camera_control) {
	tBleStatus result = BLE_STATUS_INVALID_PARAMS;
	switch (camera_control) {
	case SHUTTER:
		result = aci_gatt_update_char_value(aDataTransferContext.DataTransferSvcHdle,
						  aDataTransferContext.DataCameraCtrlCharHdle, 0, /* charValOffset */
						  9, /* charValueLen */
						  (uint8_t*) SHUTTER_CODE);
		break;
	case MODE:
		result = aci_gatt_update_char_value(aDataTransferContext.DataTransferSvcHdle,
						  aDataTransferContext.DataCameraCtrlCharHdle, 0, /* charValOffset */
						  9, /* charValueLen */
						  (uint8_t*) MODE_CODE);
		break;
	case SCREEN_TOGGLE:
		result = aci_gatt_update_char_value(aDataTransferContext.DataTransferSvcHdle,
						  aDataTransferContext.DataCameraCtrlCharHdle, 0, /* charValOffset */
						  9, /* charValueLen */
						  (uint8_t*) POWER_SHORT_PRESS_CODE);
		break;
	case POWER_LONG_PRESS:
		result = aci_gatt_update_char_value(aDataTransferContext.DataTransferSvcHdle,
						  aDataTransferContext.DataCameraCtrlCharHdle, 0, /* charValOffset */
						  9, /* charValueLen */
						  (uint8_t*) POWER_LONG_PRESS_CODE);
		break;
	case WAKEUP_CAMERAS:
		// stop advertising

		// change advertising data

		// start advertising

		wakeupConnectedCameras();

		break;
	default:
		break;
	}
	return result;
}/* end DTS_CamCtrl() */

void wakeupConnectedCameras(void){
	/* used for Insta360 X3 */
	a_ManufDataCameraWakeup[16] = 0x37;
	a_ManufDataCameraWakeup[17] = 0x4b;
	a_ManufDataCameraWakeup[18] = 0x43;
	a_ManufDataCameraWakeup[19] = 0x4d;
	a_ManufDataCameraWakeup[20] = 0x54;
	a_ManufDataCameraWakeup[21] = 0x4b;

//	tBleStatus ret = BLE_STATUS_INVALID_PARAMS;
//	ret = aci_gap_set_non_discoverable();
//	ret = aci_gap_update_adv_data(sizeof(a_ManufDataCameraWakeup), (uint8_t*) a_ManufDataCameraWakeup);
//	a_ManufDataCameraWakeup[0] = 9;

//	osThreadFlagsSet(AdvUpdateProcessId, 1);
	Adv_Request(APP_BLE_CAM_LP_ADV);

	///* used for Insta360 RS 1-inch */
	//manuf_data[14] = 0x38;
	//manuf_data[15] = 0x51;
	//manuf_data[16] = 0x53;
	//manuf_data[17] = 0x4a;
	//manuf_data[18] = 0x38;
	//manuf_data[19] = 0x52;
}

void resetAdvertisingAfterConnectedCamera(void){
	tBleStatus ret = BLE_STATUS_INVALID_PARAMS;
	ret = aci_gap_update_adv_data(sizeof(a_ManufData), (uint8_t*) a_ManufData);
}

void activateCameraMode(void){
	memset(a_LocalName, 0, sizeof(a_LocalName));
    memcpy(a_LocalName,a_camName,sizeof(a_camName));
    Adv_Request(APP_BLE_LP_ADV);
}

void deactivateCameraMode(void){
	memset(a_LocalName, 0, sizeof(a_LocalName));
    memcpy(a_LocalName,a_buzzCamName,sizeof(a_buzzCamName));
    Adv_Request(APP_BLE_LP_ADV);

}


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
