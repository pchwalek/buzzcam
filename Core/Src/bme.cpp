/*
 * ppg.c
 *
 *  Created on: Nov 30, 2021
 *      Author: patrick
 */
#include "bme.h"
#include "Adafruit_BME680.h"
//#include "packet.h"
#include "main.h"
#include "cmsis_os2.h"
#include "portmacro.h"
//#include "captivate_config.h"
//#include "bsec2.h"
//#include "../Middlewares/bsec_2_2_0_0/algo/normal_version/inc/bsec_datatypes.h"
#include "bsec_datatypes.h"
#include "FreeRTOS.h"
#include "task.h"
#include "math.h"
#include "app_fatfs.h"
//#include "fram.h"

//#include "config/FieldAir_HandSanitizer/FieldAir_HandSanitizer.h"
//#include "config/Default_H2S_NonH2S/Default_H2S_NonH2S.h"
//#include "config/bsec_sel_iaq_33v_300s_28d/bsec_serialized_configurations_selectivity.h"
#include "config/bsec_sel_iaq_33v_3s_28d/bsec_serialized_configurations_selectivity.h"


#include "pb.h"
#include "pb_decode.h"
#include "pb_encode.h"
#include "message.pb.h"
#include "dts.h"
#include "app_ble.h"

//#define BME_SAMPLE_PERIOD_MS		3000
//#define MAX_BME_SAMPLES_PACKET	(int)(512-sizeof(PacketHeader))/sizeof(bsecData)
#define MAX_BME_SAMPLES_PACKET		5
#define BME_WAIT_TOL			10
#define BME_SAVE_STATE_PERIOD_MS	7200000 // every 2 hours




typedef enum bme680_accuracy {
    BME680_ACCURACY_UNRELIABLE = 0,
    BME680_ACCURACY_LOW_ACCURACY = 1,
    BME680_ACCURACY_MEDIUM_ACCURACY = 2,
    BME680_ACCURACY_HIGH_ACCURACY = 3
} bme680_accuracy_t;

/* * defined by Bosch */
typedef enum bme680_signal_id {
    BME680_SIGNAL_ID_UNDEFINED = 0,
    BME680_SIGNAL_ID_IAQ = 1,
    BME680_SIGNAL_ID_STATIC_IAQ = 2,
    BME680_SIGNAL_ID_CO2_EQ = 3,
    BME680_SIGNAL_ID_BREATH_VOC_EQ = 4,
    BME680_SIGNAL_ID_RAW_TEMPERATURE = 6,
    BME680_SIGNAL_ID_RAW_PRESSURE = 7,
    BME680_SIGNAL_ID_RAW_HUMIDITY = 8,
    BME680_SIGNAL_ID_RAW_GAS = 9,
    BME680_SIGNAL_ID_STABILIZATION_STATUS = 12,
    BME680_SIGNAL_ID_RUN_IN_STATUS = 13,
    BME680_SIGNAL_ID_SENSOR_HEAT_COMPEN_TEMP = 14,
    BME680_SIGNAL_ID_HEAT_COMPEN_HUMID = 15,
    BME680_SIGNAL_ID_GAS_PERCENTAGE = 21
} bme680_signal_id_t;

typedef struct bme_packet_payload {
    uint64_t timestamp_sensor;
    uint64_t timestamp_unix;
    uint32_t timestamp_ms_from_start;
    float signal;
    uint32_t signal_dimensions;
    bme680_signal_id_t sensor_id;
    bme680_accuracy_t accuracy;
} bme_packet_payload_t;

typedef struct bme_packet {
    uint32_t packet_index;
    uint32_t sample_period;
    uint32_t sensor_id;
    pb_size_t payload_count;
    bme_packet_payload_t payload[12];
} bme_packet_t;

typedef struct bme_sensor_config {
    uint32_t sample_period_ms;
} bme_sensor_config_t;

// Function to format one bme_packet_payload_t element
void formatBmePayload(char *buffer, size_t bufSize, const bme_packet_payload_t *payload) {
//    snprintf(buffer, bufSize, "%.0f,%.0f,%u,%.2f,%u,%d,%d\n",
//    		 (double) payload->timestamp_unix,
//             (double) payload->timestamp_sensor,
//             payload->timestamp_ms_from_start,
//             payload->signal,
//             payload->signal_dimensions,
//             payload->sensor_id,
//             payload->accuracy);

    char temp[20];

    uint64ToString(payload->timestamp_unix, temp);
    strcpy(buffer, temp);
    strcat(buffer, ",");

    // Convert float to string and concatenate
    uint64ToString(payload->timestamp_sensor, temp);
    strcat(buffer, temp);
    strcat(buffer, ",");

    // Convert float to string and concatenate
    utoa(payload->timestamp_ms_from_start, temp, 10);
    strcat(buffer, temp);
    strcat(buffer, ",");

//    utoa(payload->signal, temp, 10);
    sprintf(temp, "%.2f", payload->signal);
    strcat(buffer, temp);
    strcat(buffer, ",");

    utoa(payload->signal_dimensions, temp, 10);
    strcat(buffer, temp);
    strcat(buffer, ",");

    utoa(payload->sensor_id, temp, 10);
    strcat(buffer, temp);
    strcat(buffer, ",");

    utoa(payload->accuracy, temp, 10);
    strcat(buffer, temp);
    strcat(buffer, "\n");
}

static bme_packet_payload_t bmeData[12];

osTimerId_t periodicBMETimer_id;

Adafruit_BME680 bme;

//static uint8_t bmeConfig[BSEC_MAX_PROPERTY_BLOB_SIZE];
//static uint8_t bmeState[BSEC_MAX_STATE_BLOB_SIZE];

static char outputString[500] = {0};

static FIL sensorFile;

static DTS_STM_Payload_t PackedPayload;

void BME_Task(void *argument) {
//	osDelay(2000);
//	sensor_packet_t *packet = NULL;
	volatile uint32_t flags = 0;

	uint8_t errorCounter = 0;

	osThreadFlagsClear(TERMINATE_THREAD_BIT);
//	bool status;

	bme_sensor_config_t sensorSettings;

//	if(argument != NULL){
//		memcpy(&sensorSettings,argument,sizeof(bme_sensor_config_t));
//	}else{
//		sensorSettings.sample_period_ms = 0;
//	}
	//todo: remove the bottom and fix the top
	sensorSettings.sample_period_ms = 5000;



	uint32_t timeSinceLastStateSave = 0;

//	osDelay(500);

	osMutexAcquire(messageI2C1_LockHandle, osWaitForever);
	while (!bme.begin(BME68X_DEFAULT_ADDRESS, &hi2c1, false)) {
		Error_Handler();
		i2c_error_check(&hi2c1);
		osMutexRelease(messageI2C1_LockHandle);
		osDelay(100);
		flags = osThreadFlagsGet();
		if ((flags & TERMINATE_THREAD_BIT) == TERMINATE_THREAD_BIT) {
			osTimerDelete (periodicBMETimer_id);
//			saveBME_StateConfig();
			osMutexAcquire(messageI2C1_LockHandle, osWaitForever);
			bme.soft_reset();
			osMutexRelease(messageI2C1_LockHandle);

			bmeTaskHandle = 0x0;
			osThreadExit();
		}
		osMutexAcquire(messageI2C1_LockHandle, osWaitForever);

	}

	bme.bsecSetConfig(bsec_config_selectivity);
//	recoverBME_StateConfig();

	i2c_error_check(&hi2c1);
	osMutexRelease(messageI2C1_LockHandle);

	bme.bsecSubscribe();

	volatile uint16_t bmeIdx = 0;
	uint32_t bmeID = 0;

	volatile int64_t timeRemaining;



	char file_name[20] = "sensor.csv";

	// add header
	if(check_file_exists(file_name) == FR_NO_FILE){
    	if(f_open(&sensorFile, file_name, FA_CREATE_NEW | FA_WRITE) == FR_OK){
			strcpy(outputString, "timestamp, time_sensor, time_ms_start, signal, signal_dim, sensor_id, accuracy\n");
			f_write(&sensorFile, outputString, strlen(outputString), NULL);
	    	// Flush the cached data to the SD card
	    	f_sync(&sensorFile);
	    	// Close the file
	    	f_close(&sensorFile);

	        memset(outputString, '\0', sizeof(outputString));
    	}
	}

	FRESULT res;
	volatile uint8_t negativeNumber = 0;

	do{
		res = f_open(&sensorFile, file_name, FA_OPEN_APPEND | FA_WRITE | FA_READ);
		if((res != FR_TIMEOUT) && (res != FR_OK)){
			Error_Handler();
		}
	}while( ((res == FR_TIMEOUT) || (osDelay(10) == osOK)) &&
			(res != FR_OK));

	while (1) {

//		if ((flags & GRAB_SAMPLE_BIT) == GRAB_SAMPLE_BIT) {
		if(1){

			osMutexAcquire(messageI2C1_LockHandle, osWaitForever);
			while(!bme.bsecRun()){
				i2c_error_check(&hi2c1);
				osMutexRelease(messageI2C1_LockHandle);
				timeRemaining = floor((bme.bmeConf.next_call/1000000.0) - HAL_GetTick());
				if(timeRemaining > BME_WAIT_TOL){
					osDelay( (timeRemaining-BME_WAIT_TOL) );
				}else if(timeRemaining > 0){
					osDelay(timeRemaining);
				}else{
					negativeNumber = 1;
				}
				osMutexAcquire(messageI2C1_LockHandle, osWaitForever);
			}
			i2c_error_check(&hi2c1);
			osMutexRelease(messageI2C1_LockHandle);

			for(int i = 0; i<bme.outputs.nOutputs; i++){
				setLED_Green(1000);
//				memcpy(&bmeData[bmeIdx++], &bme.outputs.output[i], sizeof(bsecData));
				bmeData[bmeIdx].timestamp_unix = getEpoch();
				bmeData[bmeIdx].timestamp_sensor = bme.outputs.output[i].time_stamp;
				bmeData[bmeIdx].timestamp_ms_from_start = HAL_GetTick();
				bmeData[bmeIdx].signal = bme.outputs.output[i].signal;
				bmeData[bmeIdx].signal_dimensions = bme.outputs.output[i].signal_dimensions;
				bmeData[bmeIdx].sensor_id = static_cast<bme680_signal_id_t>(bme.outputs.output[i].sensor_id);
				bmeData[bmeIdx++].accuracy = static_cast<bme680_accuracy_t>(bme.outputs.output[i].accuracy);

				/* update characteristic */
				if(bme.outputs.output[i].sensor_id == BSEC_OUTPUT_RAW_TEMPERATURE){
					infoPacket.payload.system_info_packet.simple_sensor_reading.temperature=floorf(bme.outputs.output[i].signal * 10) / 10;
				}else if(bme.outputs.output[i].sensor_id == BSEC_OUTPUT_RAW_HUMIDITY){
					infoPacket.payload.system_info_packet.simple_sensor_reading.humidity=floorf(bme.outputs.output[i].signal * 10) / 10;
				}else if(bme.outputs.output[i].sensor_id == BSEC_OUTPUT_CO2_EQUIVALENT){
					infoPacket.payload.system_info_packet.simple_sensor_reading.co2=bme.outputs.output[i].signal;
				}

			}

			if(bme.outputs.nOutputs != 0){

//				res = f_open(&sensorFile, file_name, FA_OPEN_APPEND | FA_WRITE | FA_READ);


				if(res != FR_OK){
					Error_Handler();
				}

			    for (uint8_t i = 0; i < bmeIdx; i++) {
			    	if(i == 0) formatBmePayload(outputString, sizeof(outputString), &bmeData[i]);
			    	else formatBmePayload(outputString+strlen(outputString), sizeof(outputString)-strlen(outputString), &bmeData[i]);
			    }

			    //todo: FR_INVALID_OBJECT after 30+ minutes... why
			    res = f_write(&sensorFile, outputString, strlen(outputString), NULL);
				if(f_write(&sensorFile, outputString, strlen(outputString), NULL) != FR_OK){
					Error_Handler();
				}
				memset(outputString, '\0', sizeof(outputString));


				infoPacket.payload.system_info_packet.has_simple_sensor_reading = true;
				/* Create a stream that will write to our buffer. */
				pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
				/* Now we are ready to encode the message! */
				status = pb_encode(&stream, PACKET_FIELDS, &infoPacket);
				PackedPayload.pPayload = (uint8_t*) buffer;
				PackedPayload.Length = stream.bytes_written;
				if(status) DTS_STM_UpdateChar(BUZZCAM_INFO_CHAR_UUID, (uint8_t*)&PackedPayload);


		    	// Flush the cached data to the SD card
//			    res = f_sync(&sensorFile);
//				if(res != FR_OK){
//					Error_Handler();
//				}

				// Close the file
			    res = f_sync(&sensorFile);
				if(res != FR_OK){
					Error_Handler();
				}
//				memset(&sensorFile, '\0', sizeof(sensorFile));

				bmeID++;
				bmeIdx = 0;

				setLED_Green(0);

			}
//			}

//			if( (HAL_GetTick() - timeSinceLastStateSave) >= BME_SAVE_STATE_PERIOD_MS){
//				saveBME_StateConfig();
//				timeSinceLastStateSave = HAL_GetTick();
//			}

		}
		flags = osThreadFlagsGet();

		if ((flags & TERMINATE_THREAD_BIT) == TERMINATE_THREAD_BIT) {
			// Close the file
		    res = f_close(&sensorFile);

//			osTimerDelete (periodicBMETimer_id);
//			saveBME_StateConfig();
			osMutexAcquire(messageI2C1_LockHandle, osWaitForever);
			bme.soft_reset();
			osMutexRelease(messageI2C1_LockHandle);

			bmeTaskHandle = 0x0;
			osThreadExit();
			break;
		}
	}
}

void saveBME_StateConfig(){
//	uint32_t bsecReturnLen;
//
//	bme.bsecGetConfig(bmeConfig, &bsecReturnLen);
//	bme.bsecGetState(bmeState, &bsecReturnLen);
//
//	taskENTER_CRITICAL();
//	extMemWriteData(BME_CONFIG_ADDR, bmeConfig, BME_CONFIG_SIZE);
//	taskEXIT_CRITICAL();
//	taskENTER_CRITICAL();
//	extMemWriteData(BME_STATE_ADDR, bmeState, BME_STATE_SIZE);
//	taskEXIT_CRITICAL();
}

void recoverBME_StateConfig(){
//	uint8_t conditionedSystem = 0;
//	extMemGetData(BME_FIRST_RUN_ADDR, &conditionedSystem, BME_FIRST_RUN_SIZE);
//
//	if(conditionedSystem == 0){
//		bme.bsecSetConfig(bsec_config_selectivity);
//		saveBME_StateConfig();
//		conditionedSystem = 1;
//		extMemWriteData(BME_FIRST_RUN_ADDR, &conditionedSystem, BME_FIRST_RUN_SIZE);
//	}else{
//		extMemGetData(BME_CONFIG_ADDR, bmeConfig, BME_CONFIG_SIZE);
//		extMemGetData(BME_STATE_ADDR, bmeState, BME_STATE_SIZE);
//
//		bme.bsecSetConfig(bmeConfig);
//		bme.bsecSetState(bmeState);
//	}

}

//static void triggerBMESample(void *argument) {
//	osThreadFlagsSet(bmeTaskHandle, GRAB_SAMPLE_BIT);
//}
