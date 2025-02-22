/**
 * @file      sx126x_hal.h
 *
 * @brief     Hardware Abstraction Layer for SX126x
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2021. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Semtech corporation nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
 * NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "sx126x_hal.h"
#include "main.h"
/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */


/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */



/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/**
 * Radio data transfer - write
 *
 * @remark Shall be implemented by the user
 *
 * @param [in] context          Radio implementation parameters
 * @param [in] command          Pointer to the buffer to be transmitted
 * @param [in] command_length   Buffer size to be transmitted
 * @param [in] data             Pointer to the buffer to be transmitted
 * @param [in] data_length      Buffer size to be transmitted
 *
 * @returns Operation status
 */
sx126x_hal_status_t sx126x_hal_write( const void* context, const uint8_t* command, const uint16_t command_length,
                                      const uint8_t* data, const uint16_t data_length ){

	HAL_GPIO_WritePin(SPI2_SX1262_CS_GPIO_Port, SPI2_SX1262_CS_Pin, GPIO_PIN_RESET);

	if(command_length > 0){
		if(HAL_OK != HAL_SPI_Transmit(&hspi2, command, command_length, 100)){
			return SX126X_HAL_STATUS_ERROR;
		}
	}

	if(data_length > 0){
		if(HAL_OK != HAL_SPI_Transmit(&hspi2, data, data_length, 100)){
			return SX126X_HAL_STATUS_ERROR;
		}
	}

	HAL_GPIO_WritePin(SPI2_SX1262_CS_GPIO_Port, SPI2_SX1262_CS_Pin, GPIO_PIN_SET);

	//todo: delay required
	osDelay(1);

	return SX126X_HAL_STATUS_OK;
}



/**
 * Radio data transfer - read
 *
 * @remark Shall be implemented by the user
 *
 * @param [in] context          Radio implementation parameters
 * @param [in] command          Pointer to the buffer to be transmitted
 * @param [in] command_length   Buffer size to be transmitted
 * @param [in] data             Pointer to the buffer to be received
 * @param [in] data_length      Buffer size to be received
 *
 * @returns Operation status
 */
sx126x_hal_status_t sx126x_hal_read( const void* context, const uint8_t* command, const uint16_t command_length,
                                     uint8_t* data, const uint16_t data_length ){

//	if( (data_length == 0) && (command_length == 0) ) return SX126X_HAL_STATUS_ERROR;
//
//	uint8_t *rx_data = malloc(data_length + command_length);
//	if(rx_data == NULL) return SX126X_HAL_STATUS_ERROR;

//	memcpy(data, command, command_length);
//	memcpy(&data[command_length], data, data_length);
	HAL_GPIO_WritePin(SPI2_SX1262_CS_GPIO_Port, SPI2_SX1262_CS_Pin, GPIO_PIN_RESET);

	if(command_length > 0){
		if(HAL_OK != HAL_SPI_Transmit(&hspi2, command, command_length, 100)){
			return SX126X_HAL_STATUS_ERROR;
		}
	}

	if(data_length > 0){
		if(HAL_OK != HAL_SPI_Receive(&hspi2, data, data_length, 100)){
			return SX126X_HAL_STATUS_ERROR;
		}
	}

//	HAL_SPI_TransmitReceive(&hspi1, command, rx_data, command_length + data_length, 100);

	HAL_GPIO_WritePin(SPI2_SX1262_CS_GPIO_Port, SPI2_SX1262_CS_Pin, GPIO_PIN_SET);

//	memcpy(data, &rx_data[command_length], data_length);

//	free(rx_data);

	return SX126X_HAL_STATUS_OK;
}

/**
 * Reset the radio
 *
 * @remark Shall be implemented by the user
 *
 * @param [in] context Radio implementation parameters
 *
 * @returns Operation status
 */
sx126x_hal_status_t sx126x_hal_reset( const void* context ){
	HAL_GPIO_WritePin(SX_NRESET_GPIO_Port, SX_NRESET_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(SX_NRESET_GPIO_Port, SX_NRESET_Pin, GPIO_PIN_SET);
}

/**
 * Wake the radio up.
 *
 * @remark Shall be implemented by the user
 *
 * @param [in] context Radio implementation parameters
 *
 * @returns Operation status
 */
sx126x_hal_status_t sx126x_hal_wakeup( const void* context ){
	HAL_GPIO_WritePin(SPI2_SX1262_CS_GPIO_Port, SPI2_SX1262_CS_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(SPI2_SX1262_CS_GPIO_Port, SPI2_SX1262_CS_Pin, GPIO_PIN_SET);
}




/* --- EOF ------------------------------------------------------------------ */
