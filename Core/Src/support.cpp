/*
 * support.cpp
 *
 *  Created on: Jan 18, 2025
 *      Author: root
 */

#include "support.h"
#include "main.h"
#include "stm32wbxx_hal_gpio.h"

SystemState systemState = {false, false, false, false, false, false, false, false, false, false, OFF};
SystemPowerSupervisor systemPowerSupervisor = {false, false, false, false, false, false, false, false, false, false};
PowerRegime powerRegime = CRITICAL;

// Controls power to the microphone regulator and FRAM
void Control_Microphone_FRAM_Power(bool enable) {

	if(enable){
		HAL_GPIO_WritePin(EN_MIC_PWR_GPIO_Port, EN_MIC_PWR_Pin, GPIO_PIN_SET);
		MX_I2C3_Init();
	}else if(!systemState.isFRAMActive & !systemState.isMicrophoneActive){
		HAL_GPIO_WritePin(EN_MIC_PWR_GPIO_Port, EN_MIC_PWR_Pin, GPIO_PIN_RESET);
		MX_I2C3_Deinit();

	}
}

// Controls power to the 3.3V secondary regulator for BME688, IMU, LoRa, and GPS Backup
void Control_Secondary_Power(bool enable) {

    if(enable){
    	HAL_GPIO_WritePin(EN_3V3_ALT_GPIO_Port, EN_3V3_ALT_Pin, GPIO_PIN_SET);
    	if(!Is_GPS_Enabled()) Control_GPS_Power(true); // required since I2C1 won't work
    	HAL_Delay(1);
    	osMutexAcquire(messageI2C1_LockHandle, osWaitForever); // this should never be necessary since no device should be communicating at this point
    	MX_I2C1_Init();
    	osMutexRelease(messageI2C1_LockHandle);
    	if(!systemPowerSupervisor.isGPSEnabled){
			turnOnGPSandInit();
			standbyGPSMode();
    	}
    }else if(!systemState.isEnvironmentalSensorActive &&
    		!systemState.isAccelerometerActive &&
			!systemState.isLoRaActive &&
			!systemState.isGPSActive){
    	HAL_GPIO_WritePin(EN_3V3_ALT_GPIO_Port, EN_3V3_ALT_Pin, GPIO_PIN_RESET);
    	if(Is_GPS_Enabled()) Control_GPS_Power(false); // gps isn't active
    	osMutexAcquire(messageI2C1_LockHandle, osWaitForever); // this should never be necessary since no device should be communicating at this point
    	MX_I2C1_Deinit();
    	osMutexRelease(messageI2C1_LockHandle);
    }

}

// Controls power to enable the GPS.
void Control_GPS_Power(bool enable) {
    HAL_GPIO_WritePin(EN_3V3_GPS_GPIO_Port, EN_3V3_GPS_Pin, enable ? GPIO_PIN_SET : GPIO_PIN_RESET);

    if(enable){
		// also need to activate secondary power for backup supply and I2C comms
		if(!Is_Secondary_Enabled()) Control_Secondary_Power(true);
    }else{
    	if(Is_Secondary_Enabled()) Control_Secondary_Power(false);
    }
}

// Controls power to enable the battery monitoring circuit.
void Control_BatteryMonitor_Power(bool enable) {
    HAL_GPIO_WritePin(EN_BATT_MON_GPIO_Port, EN_BATT_MON_Pin, enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

// Controls power to enable the buzzer circuit.
void Control_Buzzer_Power(bool enable) {
    HAL_GPIO_WritePin(EN_BUZZER_PWR_GPIO_Port, EN_BUZZER_PWR_Pin, enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

// Controls power to MAX78000.
void Control_MAX78000_Power(bool enable) {
    HAL_GPIO_WritePin(EN_MAX78000_GPIO_Port, EN_MAX78000_Pin, enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

// Controls power to MAX78000.
void Control_UWB_Power(bool enable) {
    HAL_GPIO_WritePin(EN_UWB_REG_GPIO_Port, EN_UWB_REG_Pin, enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

// Controls power to SD Cards.
void Control_SDCard_Power(SDCardState sdcardState) {
	if(sdcardState == OFF){
		HAL_GPIO_WritePin(EN_SD_REG_GPIO_Port, EN_SD_REG_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(EN_SD_REG_2_GPIO_Port, EN_SD_REG_2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(EN_SD_MUX_GPIO_Port, EN_SD_MUX_Pin, GPIO_PIN_SET); // disable mux
	}else if(sdcardState = SD1_EN){
		HAL_GPIO_WritePin(EN_SD_REG_2_GPIO_Port, EN_SD_REG_2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(EN_SD_REG_GPIO_Port, EN_SD_REG_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(EN_SD_MUX_GPIO_Port, EN_SD_MUX_Pin, GPIO_PIN_RESET); // enable mux
		HAL_GPIO_WritePin(SD_MUX_SEL_GPIO_Port, SD_MUX_SEL_Pin, GPIO_PIN_RESET); // mux select 1
	}else if(sdcardState = SD2_EN){
		HAL_GPIO_WritePin(EN_SD_REG_GPIO_Port, EN_SD_REG_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(EN_SD_REG_2_GPIO_Port, EN_SD_REG_2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(EN_SD_MUX_GPIO_Port, EN_SD_MUX_Pin, GPIO_PIN_RESET); // enable mux
		HAL_GPIO_WritePin(SD_MUX_SEL_GPIO_Port, SD_MUX_SEL_Pin, GPIO_PIN_SET); // mux select 2
	}
}

// Checks if the UWB regulator is enabled.
bool Is_UWB_Enabled() {
    return HAL_GPIO_ReadPin(EN_UWB_REG_GPIO_Port, EN_UWB_REG_Pin) == GPIO_PIN_SET;
}

// Checks if the microphone regulator is enabled.
bool Is_Microphone_Enabled() {
    return HAL_GPIO_ReadPin(EN_MIC_PWR_GPIO_Port, EN_MIC_PWR_Pin) == GPIO_PIN_SET;
}

// Checks if the secondary regulator for LoRa, light sensor, and accelerometer is enabled.
bool Is_Secondary_Enabled() {
    return HAL_GPIO_ReadPin(EN_3V3_ALT_GPIO_Port, EN_3V3_ALT_Pin) == GPIO_PIN_SET;
}

// Checks if the battery monitoring circuit is enabled.
bool Is_BatteryMonitor_Enabled() {
    return HAL_GPIO_ReadPin(EN_BATT_MON_GPIO_Port, EN_BATT_MON_Pin) == GPIO_PIN_SET;
}

// Checks if the gps circuit is enabled.
bool Is_GPS_Enabled() {
    return HAL_GPIO_ReadPin(EN_3V3_GPS_GPIO_Port, EN_3V3_GPS_Pin) == GPIO_PIN_SET;
}

// Checks if the buzzer circuit is enabled.
bool Is_Buzzer_Enabled() {
    return HAL_GPIO_ReadPin(EN_BUZZER_PWR_GPIO_Port, EN_BUZZER_PWR_Pin) == GPIO_PIN_SET;
}

// Check if the MAX78000 is enabled.
bool Is_MAX78000_Enabled() {
    return HAL_GPIO_ReadPin(EN_MAX78000_GPIO_Port, EN_MAX78000_Pin) == GPIO_PIN_SET;
}


// Example usage to turn off all systems
void TurnOffAllSystems() {
	Control_Microphone_FRAM_Power(false);
	Control_Secondary_Power(false);
	Control_GPS_Power(false);
	Control_BatteryMonitor_Power(false);
	Control_Buzzer_Power(false);
	Control_MAX78000_Power(false);
	Control_SDCard_Power(OFF);
}

// Example usage to check system states
void CheckSystemStates() {
    if (Is_Microphone_Enabled()) {
        // Take some action if the microphone is enabled
    }
    if (Is_Secondary_Enabled()) {
        // Take some action if sensors are enabled
    }
    if (Is_BatteryMonitor_Enabled()) {
        // Take some action if battery monitor is enabled
    }
    if (Is_GPS_Enabled()) {
        // Take some action if gps is enabled
    }
    if (Is_Buzzer_Enabled()){
    	// Take some action if buzzer is enabled
    }
    if (Is_MAX78000_Enabled()){
    	// Take some action if MAX78000 is enabled
    }
    if (Is_UWB_Enabled()){
    	// Take some action if UWB is enabled
    }
}

/* returns true if the system is in a new state */
bool updateSystemPowerSupervisor(SystemPowerSupervisor* supervisor, PowerRegime* regime) {
	float batteryVoltage = getBattVltg();
	float batteryPercentage = calculate_battery_percentage(batteryVoltage);
	bool returnVar = false;

    if (batteryPercentage > 50) {
        // Set all variables to true and regime to FULL
    	if(*regime != FULL) returnVar = true;
        *regime = FULL;
        supervisor->isLoRaEnabled = true;
        supervisor->isAccelerometerEnabled = true;
        supervisor->isMicrophoneEnabled = true;
        supervisor->isEnvironmentalSensorEnabled = true;
        supervisor->isGPSEnabled = true;
        supervisor->isBatteryLevelSensingEnabled = true;
        supervisor->isBuzzerEnabled = true;
        supervisor->isMAX78000Enabled = true;
        supervisor->isSDEnabled = true;
    } else if (batteryPercentage > 25) {
        // Set all variables to true and regime to REDUCED
    	if(*regime != REDUCED) returnVar = true;
        *regime = REDUCED;
        supervisor->isLoRaEnabled = true;
        supervisor->isAccelerometerEnabled = true;
        supervisor->isMicrophoneEnabled = true;
        supervisor->isEnvironmentalSensorEnabled = true;
        supervisor->isGPSEnabled = true;
        supervisor->isBatteryLevelSensingEnabled = true;
        supervisor->isBuzzerEnabled = true;
        supervisor->isMAX78000Enabled = true;
        supervisor->isSDEnabled = true;
    } else if (batteryPercentage > 10) {
        // Set particulate sensor to true, others to false, and regime to LOW
    	if(*regime != LOW) returnVar = true;
        *regime = LOW;
        supervisor->isLoRaEnabled = false;
        supervisor->isAccelerometerEnabled = false;
        supervisor->isMicrophoneEnabled = true;
        supervisor->isEnvironmentalSensorEnabled = true;
        supervisor->isGPSEnabled = false;
        supervisor->isBatteryLevelSensingEnabled = true;
        supervisor->isBuzzerEnabled = false;
        supervisor->isMAX78000Enabled = false;
        supervisor->isSDEnabled = true;
    } else {
        // Set all variables to false and regime to CRITICAL
    	if(*regime != CRITICAL) returnVar = true;
        *regime = CRITICAL;
        supervisor->isLoRaEnabled = false;
        supervisor->isAccelerometerEnabled = false;
        supervisor->isMicrophoneEnabled = false;
        supervisor->isEnvironmentalSensorEnabled = false;
        supervisor->isGPSEnabled = false;
        supervisor->isBatteryLevelSensingEnabled = false;
        supervisor->isBuzzerEnabled = false;
        supervisor->isMAX78000Enabled = false;
        supervisor->isSDEnabled = false;
    }

//	if(*regime != FULL) returnVar = true;
//	*regime = FULL;
//	supervisor->isLoRaEnabled = true;

    /* save battery voltage to metadata file if not in CRITICAL power regime */
//    if(*regime != CRITICAL) saveBattVltg(&batteryVoltage, &batteryPercentage);

    return returnVar;
}

float calculate_battery_percentage(double x) {
    // Coefficients of the 5th order polynomial
    const double a4 = 1170.77083281;
    const double a3 = -17658.92750232;
    const double a2 = 99637.00252128;
    const double a1 = -249091.93964883;
    const double a0 = 232712.23959121;
    double result;

    if(x < 3.4){
    	result = 0;
    }else{
		// Calculate the polynomial value at x
		result = a4 * x * x * x * x +
						a3 * x * x * x +
						a2 * x * x +
						a1 * x +
						a0;

	    if(result > 100){
	    	result = 100;
	    }else if(result < 0){
	    	result = 0;
	    }
    }

//    packet.payload.system_info_packet.has_battery_state = true;
//    packet.payload.system_info_packet.battery_state.has_percentage = true;
//	packet.payload.system_info_packet.battery_state.percentage = result;

    return result;
}

//#define BATTERY_ADC_SAMPLES		1
float getBattVltg(void){
//	volatile HAL_StatusTypeDef state;
	if(!Is_BatteryMonitor_Enabled()) Control_BatteryMonitor_Power(true);
	HAL_Delay(1); // give time for voltage to settle
	float battVltg = 0;
//	HAL_StatusTypeDef status;
	//			timestamp = getEpoch();
	HAL_ADC_Start(&hadc1);

//	for(uint16_t i = 0; i<BATTERY_ADC_SAMPLES; i++){
//		status = HAL_ADC_PollForConversion(&hadc1, 100);
//		if(status == HAL_OK){
//			battVltg += HAL_ADC_GetValue(&hadc1);
//		}else{
//			Error_Handler();
//		}
//	}
//
//	battVltg = battVltg / BATTERY_ADC_SAMPLES;
	HAL_ADC_PollForConversion(&hadc1, 100);
	battVltg = (( HAL_ADC_GetValue(&hadc1) * 3.3 /4096) * 2);
	HAL_ADC_Stop(&hadc1);
	Control_BatteryMonitor_Power(false);
//	packet.payload.system_info_packet.has_battery_state = true;
//	packet.payload.system_info_packet.battery_state.has_percentage = false;
//	packet.payload.system_info_packet.battery_state.voltage = battVltg;

	return battVltg;
}

