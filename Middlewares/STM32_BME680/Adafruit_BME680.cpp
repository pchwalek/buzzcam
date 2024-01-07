/*!
 * @file Adafruit_BME680.cpp
 *
 * @mainpage Adafruit BME680 temperature, humidity, barometric pressure and gas
 * sensor driver
 *
 * @section intro_sec Introduction
 *
 * This is the documentation for Adafruit's BME680 driver for the
 * Arduino platform.  It is designed specifically to work with the
 * Adafruit BME680 breakout: https://www.adafruit.com/products/3660
 *
 * These sensors use I2C to communicate, 2 pins (SCL+SDA) are required
 * to interface with the breakout.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * @section author Author
 *
 * Written by Ladyada for Adafruit Industries.
 *
 * @section license License
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

#include "Adafruit_BME680.h"
#include "math.h"
#include "main.h"
#include "bme68x_defs.h"
#include "cmsis_os2.h"

static uint8_t workBuffer[BSEC_MAX_WORKBUFFER_SIZE];

//#define BME680_DEBUG
#define	delay		osDelay
#define millis	HAL_GetTick

#ifndef ARRAY_LEN
#define ARRAY_LEN(array)				(sizeof(array)/sizeof(array[0]))
#endif

/** Our hardware interface functions **/
static int8_t i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len,
                       void *interface);
static int8_t i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len,
                        void *interface);
//static int8_t spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len,
//                       void *interface);
//static int8_t spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len,
//                        void *interface);
static void delay_usec(uint32_t us, void *intf_ptr);
uint32_t GetMicros(void);

// PUBLIC FUNCTIONS

/*!
 *  @brief  Instantiates sensor with i2c.
 *  @param  *theWire
 *          optional Wire object
 */
Adafruit_BME680::Adafruit_BME680(void)
    : _meas_start(0), _meas_period(0) {
}

///*!
// *  @brief  Instantiates sensor with Hardware SPI.
// *  @param  cspin
// *          SPI chip select.
// *  @param  theSPI
// *          optional SPI object
// */
//Adafruit_BME680::Adafruit_BME680(int8_t cspin, SPIClass *theSPI)
//    : _meas_start(0), _meas_period(0) {
//  _spidev = new Adafruit_SPIDevice(cspin, 1000000, SPI_BITORDER_MSBFIRST,
//                                   SPI_MODE0, theSPI);
//}
//
///*!
// *  @brief  Instantiates sensor with Software (bit-bang) SPI.
// *  @param  cspin
// *          SPI chip select
// *  @param  mosipin
// *          SPI MOSI (Data from microcontroller to sensor)
// *  @param  misopin
// *          SPI MISO (Data to microcontroller from sensor)
// *  @param  sckpin
// *          SPI Clock
// */
//Adafruit_BME680::Adafruit_BME680(int8_t cspin, int8_t mosipin, int8_t misopin,
//                                 int8_t sckpin)
//    : _meas_start(0), _meas_period(0) {
//  _spidev = new Adafruit_SPIDevice(cspin, sckpin, misopin, mosipin, 1000000,
//                                   SPI_BITORDER_MSBFIRST, SPI_MODE0);
//}

void Adafruit_BME680::soft_reset(void){
	bme68x_soft_reset(&gas_sensor);
}

/*!
 *  @brief  Initializes the sensor
 *          Hardware ss initialized, verifies it is in the I2C or SPI bus, then
 * reads calibration data in preparation for sensor reads.
 *  @param  addr
 *          Optional parameter for the I2C address of BME680. Default is 0x76
 *  @param  initSettings
 *          Optional parameter for initializing the sensor settings.
 *          Default is true.
 *  @return True on sensor initialization success. False on failure.
 */
bool Adafruit_BME680::begin(uint8_t i2c_address, I2C_HandleTypeDef *i2c_handle, bool initSettings) {

	i2c_han = i2c_handle;
	i2c_addr = i2c_address << 1;

	i2c_dev.i2c_han = i2c_han;
	i2c_dev.i2c_addr = i2c_addr;

  int8_t rslt;

  extTempOffset = 0.0f;

	gas_sensor.chip_id = i2c_addr;
	gas_sensor.intf = BME68X_I2C_INTF;
	gas_sensor.intf_ptr = &i2c_dev;
	gas_sensor.read = i2c_read;
	gas_sensor.write = i2c_write;

  gas_sensor.amb_temp = 25; /* The ambient temperature in deg C is used for
                               defining the heater temperature */
  gas_sensor.delay_us = delay_usec;

  rslt = bme68x_init(&gas_sensor);
#ifdef BME680_DEBUG
  Serial.print(F("Init Result: "));
  Serial.println(rslt);
#endif

  if (rslt != BME68X_OK)
    return false;

#ifdef BME680_DEBUG
  Serial.print("T1 = ");
  Serial.println(gas_sensor.calib.par_t1);
  Serial.print("T2 = ");
  Serial.println(gas_sensor.calib.par_t2);
  Serial.print("T3 = ");
  Serial.println(gas_sensor.calib.par_t3);
  Serial.print("P1 = ");
  Serial.println(gas_sensor.calib.par_p1);
  Serial.print("P2 = ");
  Serial.println(gas_sensor.calib.par_p2);
  Serial.print("P3 = ");
  Serial.println(gas_sensor.calib.par_p3);
  Serial.print("P4 = ");
  Serial.println(gas_sensor.calib.par_p4);
  Serial.print("P5 = ");
  Serial.println(gas_sensor.calib.par_p5);
  Serial.print("P6 = ");
  Serial.println(gas_sensor.calib.par_p6);
  Serial.print("P7 = ");
  Serial.println(gas_sensor.calib.par_p7);
  Serial.print("P8 = ");
  Serial.println(gas_sensor.calib.par_p8);
  Serial.print("P9 = ");
  Serial.println(gas_sensor.calib.par_p9);
  Serial.print("P10 = ");
  Serial.println(gas_sensor.calib.par_p10);
  Serial.print("H1 = ");
  Serial.println(gas_sensor.calib.par_h1);
  Serial.print("H2 = ");
  Serial.println(gas_sensor.calib.par_h2);
  Serial.print("H3 = ");
  Serial.println(gas_sensor.calib.par_h3);
  Serial.print("H4 = ");
  Serial.println(gas_sensor.calib.par_h4);
  Serial.print("H5 = ");
  Serial.println(gas_sensor.calib.par_h5);
  Serial.print("H6 = ");
  Serial.println(gas_sensor.calib.par_h6);
  Serial.print("H7 = ");
  Serial.println(gas_sensor.calib.par_h7);
  Serial.print("G1 = ");
  Serial.println(gas_sensor.calib.par_gh1);
  Serial.print("G2 = ");
  Serial.println(gas_sensor.calib.par_gh2);
  Serial.print("G3 = ");
  Serial.println(gas_sensor.calib.par_gh3);
  Serial.print("G1 = ");
  Serial.println(gas_sensor.calib.par_gh1);
  Serial.print("G2 = ");
  Serial.println(gas_sensor.calib.par_gh2);
  Serial.print("G3 = ");
  Serial.println(gas_sensor.calib.par_gh3);
  Serial.print("Heat Range = ");
  Serial.println(gas_sensor.calib.res_heat_range);
  Serial.print("Heat Val = ");
  Serial.println(gas_sensor.calib.res_heat_val);
  Serial.print("SW Error = ");
  Serial.println(gas_sensor.calib.range_sw_err);
#endif

//  if (initSettings) {
//    setIIRFilterSize(BME68X_FILTER_SIZE_3);
//    setODR(BME68X_ODR_NONE);
//    setHumidityOversampling(BME68X_OS_2X);
//    setPressureOversampling(BME68X_OS_4X);
//    setTemperatureOversampling(BME68X_OS_8X);
//    setGasHeater(320, 150); // 320*C for 150 ms
//  } else {
//    setGasHeater(0, 0);
//  }
  // don't do anything till we request a reading
//  rslt = bme68x_set_op_mode(BME68X_FORCED_MODE, &gas_sensor);

  // Initialize BSEC library before further use
  status = bsec_init();
  if (status != BSEC_OK)
	  return false;

  status = bsec_get_version(&version);
	if (status != BSEC_OK)
	 return false;

	memset(&bmeConf, 0, sizeof(bmeConf));
	memset(&outputs, 0, sizeof(outputs));
	memset(&heatrConf, 0, sizeof(heatrConf));

#ifdef BME680_DEBUG
  Serial.print(F("Opmode Result: "));
  Serial.println(rslt);
#endif

  if (rslt != BME68X_OK)
    return false;

  return true;
}

bool Adafruit_BME680::bsecSubscribe(void) {
    bsecSensor sensorList[] = {
            BSEC_OUTPUT_IAQ,
            BSEC_OUTPUT_RAW_TEMPERATURE,
            BSEC_OUTPUT_RAW_PRESSURE,
            BSEC_OUTPUT_RAW_HUMIDITY,
            BSEC_OUTPUT_RAW_GAS,
            BSEC_OUTPUT_STABILIZATION_STATUS,
            BSEC_OUTPUT_RUN_IN_STATUS,
			BSEC_OUTPUT_CO2_EQUIVALENT,
			BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
//			BSEC_OUTPUT_GAS_ESTIMATE_1,
//			BSEC_OUTPUT_GAS_ESTIMATE_2
    };

    bsec_sensor_configuration_t virtualSensors[BSEC_NUMBER_OUTPUTS], sensorSettings[BSEC_MAX_PHYSICAL_SENSOR];
    uint8_t nSensorSettings = BSEC_MAX_PHYSICAL_SENSOR;
    uint16_t nSensors = ARRAY_LEN(sensorList);

    for (uint8_t i = 0; i < nSensors; i++)
    {
        virtualSensors[i].sensor_id = sensorList[i];
//        virtualSensors[i].sample_rate = BSEC_SAMPLE_RATE_CONT;
		virtualSensors[i].sample_rate = BSEC_SAMPLE_RATE_LP;
//		virtualSensors[i].sample_rate = BSEC_SAMPLE_RATE_SCAN;
    }

    status = bsec_update_subscription(virtualSensors, nSensors, sensorSettings, &nSensorSettings);
    if (status != BSEC_OK){
        return false;
    }
    else{
    	return true;
    }
}

/*!
 *  @brief  Performs a reading and returns the ambient temperature.
 *  @return Temperature in degrees Centigrade
 */
float Adafruit_BME680::readTemperature(void) {
  performReading();
  return temperature;
}

/*!
 *  @brief Performs a reading and returns the barometric pressure.
 *  @return Barometic pressure in Pascals
 */
float Adafruit_BME680::readPressure(void) {
  performReading();
  return pressure;
}

/*!
 *  @brief  Performs a reading and returns the relative humidity.
 *  @return Relative humidity as floating point
 */
float Adafruit_BME680::readHumidity(void) {
  performReading();
  return humidity;
}

/*!
 *  @brief Calculates the resistance of the MOX gas sensor.
 *  @return Resistance in Ohms
 */
uint32_t Adafruit_BME680::readGas(void) {
  performReading();
  return gas_resistance;
}

/*!
 *  @brief  Calculates the altitude (in meters).
 *          Reads the current atmostpheric pressure (in hPa) from the sensor and
 * calculates via the provided sea-level pressure (in hPa).
 *  @param  seaLevel
 *          Sea-level pressure in hPa
 *  @return Altitude in meters
 */
float Adafruit_BME680::readAltitude(float seaLevel) {
  // Equation taken from BMP180 datasheet (page 16):
  //  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

  // Note that using the equation from wikipedia can give bad results
  // at high altitude. See this thread for more information:
  //  http://forums.adafruit.com/viewtopic.php?f=22&t=58064

  float atmospheric = readPressure() / 100.0F;
  return 44330.0 * (1.0 - pow(atmospheric / seaLevel, 0.1903));
}

/*!
 *  @brief  Performs a full reading of all 4 sensors in the BME680.
 *          Assigns the internal Adafruit_BME680#temperature,
 * Adafruit_BME680#pressure, Adafruit_BME680#humidity and
 * Adafruit_BME680#gas_resistance member variables
 *  @return True on success, False on failure
 */
bool Adafruit_BME680::performReading(void) { return endReading(); }

/*! @brief Begin an asynchronous reading.
 *  @return When the reading would be ready as absolute time in millis().
 */
uint32_t Adafruit_BME680::beginReading(void) {
  if (_meas_start != 0) {
    /* A measurement is already in progress */
    return _meas_start + _meas_period;
  }

  int8_t rslt = bme68x_set_op_mode(BME68X_FORCED_MODE, &gas_sensor);
#ifdef BME680_DEBUG
  Serial.print(F("Opmode Result: "));
  Serial.println(rslt);
#endif
  if (rslt != BME68X_OK)
    return false;

  /* Calculate delay period in microseconds */
  uint32_t delayus_period = (uint32_t)bme68x_get_meas_dur(
                                BME68X_FORCED_MODE, &gas_conf, &gas_sensor) +
                            ((uint32_t)gas_heatr_conf.heatr_dur * 1000);
  // Serial.print("measure: ");
  // Serial.println(bme68x_get_meas_dur(BME68X_FORCED_MODE, &gas_conf,
  // &gas_sensor)); Serial.print("heater: ");
  // Serial.println((uint32_t)gas_heatr_conf.heatr_dur * 1000);

  _meas_start = millis();
  _meas_period = delayus_period / 1000;

  return _meas_start + _meas_period;
}

/*! @brief  End an asynchronous reading.
 *          If the asynchronous reading is still in progress, block until it
 * ends. If no asynchronous reading has started, this is equivalent to
 * performReading().
 *  @return Whether success.
 */
bool Adafruit_BME680::endReading(void) {
  uint32_t meas_end = beginReading();

  if (meas_end == 0) {
    return false;
  }

  int remaining_millis = remainingReadingMillis();

  if (remaining_millis > 0) {
#ifdef BME680_DEBUG
    Serial.print(F("Waiting (ms) "));
    Serial.println(remaining_millis);
#endif
    delay(static_cast<unsigned int>(remaining_millis) *
          2); /* Delay till the measurement is ready */
  }
  _meas_start = 0; /* Allow new measurement to begin */
  _meas_period = 0;

#ifdef BME680_DEBUG
  Serial.print(F("t_fine = "));
  Serial.println(gas_sensor.calib.t_fine);
#endif

  struct bme68x_data data;
  uint8_t n_fields;

#ifdef BME680_DEBUG
  Serial.println(F("Getting sensor data"));
#endif

  int8_t rslt =
      bme68x_get_data(BME68X_FORCED_MODE, &data, &n_fields, &gas_sensor);
#ifdef BME680_DEBUG
  Serial.print(F("GetData Result: "));
  Serial.println(rslt);
#endif
  if (rslt != BME68X_OK)
    return false;

  if (n_fields) {
    temperature = data.temperature;
    humidity = data.humidity;
    pressure = data.pressure;

#ifdef BME680_DEBUG
    Serial.print(F("data.status 0x"));
    Serial.println(data.status, HEX);
#endif
    if (data.status & (BME68X_HEAT_STAB_MSK | BME68X_GASM_VALID_MSK)) {
      // Serial.print("Gas resistance: "); Serial.println(data.gas_resistance);
      gas_resistance = data.gas_resistance;
    } else {
      gas_resistance = 0;
      // Serial.println("Gas reading unstable!");
    }
  }

  return true;
}

/*! @brief  Get remaining time for an asynchronous reading.
 *          If the asynchronous reading is still in progress, how many millis
 * until its completion. If the asynchronous reading is completed, 0. If no
 * asynchronous reading has started, -1 or
 * Adafruit_BME680::reading_not_started. Does not block.
 *  @return Remaining millis until endReading will not block if invoked.
 */
int Adafruit_BME680::remainingReadingMillis(void) {
  if (_meas_start != 0) {
    /* A measurement is already in progress */
    int remaining_time = (int)_meas_period - (millis() - _meas_start);
    return remaining_time < 0 ? reading_complete : remaining_time;
  }
  return reading_not_started;
}

/*!
 *  @brief  Enable and configure gas reading + heater
 *  @param  heaterTemp
 *          Desired temperature in degrees Centigrade
 *  @param  heaterTime
 *          Time to keep heater on in milliseconds
 *  @return True on success, False on failure
 */
bool Adafruit_BME680::setGasHeater(uint16_t heaterTemp, uint16_t heaterTime) {

  if ((heaterTemp == 0) || (heaterTime == 0)) {
    gas_heatr_conf.enable = BME68X_DISABLE;
  } else {
    gas_heatr_conf.enable = BME68X_ENABLE;
    gas_heatr_conf.heatr_temp = heaterTemp;
    gas_heatr_conf.heatr_dur = heaterTime;
  }

  int8_t rslt =
      bme68x_set_heatr_conf(BME68X_FORCED_MODE, &gas_heatr_conf, &gas_sensor);
#ifdef BME680_DEBUG
  Serial.print(F("SetHeaterConf Result: "));
  Serial.println(rslt);
#endif
  return rslt == 0;
}

/*!
 *  @brief  Setter for Output Data Rate
 *  @param  odr
 *          Output data rate setting, can be BME68X_ODR_NONE,
 * BME68X_ODR_0_59_MS, BME68X_ODR_10_MS, BME68X_ODR_20_MS, BME68X_ODR_62_5_MS,
 * BME68X_ODR_125_MS, BME68X_ODR_250_MS, BME68X_ODR_500_MS, BME68X_ODR_1000_MS
 *  @return True on success, False on failure
 */

bool Adafruit_BME680::setODR(uint8_t odr) {
  if (odr > BME68X_ODR_NONE)
    return false;

  gas_conf.odr = odr;

  int8_t rslt = bme68x_set_conf(&gas_conf, &gas_sensor);
#ifdef BME680_DEBUG
  Serial.print(F("SetConf Result: "));
  Serial.println(rslt);
#endif
  return rslt == 0;
}

/**
 * @brief Function to fetch data from the sensor into the local buffer
 */
uint8_t Adafruit_BME680::fetchData(void)
{
	nFields = 0;
	bme68xStatus = bme68x_get_data(lastOpMode, sensorData, &nFields, &gas_sensor);
	iFields = 0;

	return nFields;
}

/**
 * @brief Function to get a single data field
 */
uint8_t Adafruit_BME680::getData(bme68xData &data)
{
	if (lastOpMode == BME68X_FORCED_MODE)
	{
		data = sensorData[0];
	} else
	{
		if (nFields)
		{
			/* iFields spans from 0-2 while nFields spans from
			 * 0-3, where 0 means that there is no new data
			 */
			data = sensorData[iFields];
			iFields++;

			/* Limit reading continuously to the last fields read */
			if (iFields >= nFields)
			{
				iFields = nFields - 1;
				return 0;
			}

			/* Indicate if there is something left to read */
			return nFields - iFields;
		}
	}

	return 0;
}

/**
 * @brief Function to set the Temperature, Pressure and Humidity over-sampling
 */
void Adafruit_BME680::setTPH(uint8_t osTemp, uint8_t osPres, uint8_t osHum)
{
	bme68xStatus = bme68x_get_conf(&gas_conf, &gas_sensor);

	if (bme68xStatus == BME68X_OK)
	{
		gas_conf.os_hum = osHum;
		gas_conf.os_temp = osTemp;
		gas_conf.os_pres = osPres;

		bme68xStatus = bme68x_set_conf(&gas_conf, &gas_sensor);
	}
}


/*!
 *  @brief  Setter for Temperature oversampling
 *  @param  oversample
 *          Oversampling setting, can be BME68X_OS_NONE (turn off Temperature
 * reading), BME68X_OS_1X, BME68X_OS_2X, BME68X_OS_4X, BME68X_OS_8X or
 * BME68X_OS_16X
 *  @return True on success, False on failure
 */

bool Adafruit_BME680::setTemperatureOversampling(uint8_t oversample) {
  if (oversample > BME68X_OS_16X)
    return false;

  gas_conf.os_temp = oversample;

  int8_t rslt = bme68x_set_conf(&gas_conf, &gas_sensor);
#ifdef BME680_DEBUG
  Serial.print(F("SetConf Result: "));
  Serial.println(rslt);
#endif
  return rslt == 0;
}

/*!
 *  @brief  Setter for Humidity oversampling
 *  @param  oversample
 *          Oversampling setting, can be BME68X_OS_NONE (turn off Humidity
 * reading), BME68X_OS_1X, BME68X_OS_2X, BME68X_OS_4X, BME68X_OS_8X or
 * BME68X_OS_16X
 *  @return True on success, False on failure
 */
bool Adafruit_BME680::setHumidityOversampling(uint8_t oversample) {
  if (oversample > BME68X_OS_16X)
    return false;

  gas_conf.os_hum = oversample;

  int8_t rslt = bme68x_set_conf(&gas_conf, &gas_sensor);
#ifdef BME680_DEBUG
  Serial.print(F("SetConf Result: "));
  Serial.println(rslt);
#endif
  return rslt == 0;
}

/*!
 *  @brief  Setter for Pressure oversampling
 *  @param  oversample
 *          Oversampling setting, can be BME68X_OS_NONE (turn off Pressure
 * reading), BME68X_OS_1X, BME68X_OS_2X, BME68X_OS_4X, BME68X_OS_8X or
 * BME68X_OS_16X
 *  @return True on success, False on failure
 */
bool Adafruit_BME680::setPressureOversampling(uint8_t oversample) {
  if (oversample > BME68X_OS_16X)
    return false;

  gas_conf.os_pres = oversample;

  int8_t rslt = bme68x_set_conf(&gas_conf, &gas_sensor);
#ifdef BME680_DEBUG
  Serial.print(F("SetConf Result: "));
  Serial.println(rslt);
#endif
  return rslt == 0;
}

/*!
 *  @brief  Setter for IIR filter.
 *  @param  filtersize
 *          Size of the filter (in samples).
 *          Can be BME68X_FILTER_SIZE_0 (no filtering), BME68X_FILTER_SIZE_1,
 * BME68X_FILTER_SIZE_3, BME68X_FILTER_SIZE_7, BME68X_FILTER_SIZE_15,
 * BME68X_FILTER_SIZE_31, BME68X_FILTER_SIZE_63, BME68X_FILTER_SIZE_127
 *  @return True on success, False on failure
 */
bool Adafruit_BME680::setIIRFilterSize(uint8_t filtersize) {
  if (filtersize > BME68X_FILTER_SIZE_127)
    return false;
  gas_conf.filter = filtersize;

  int8_t rslt = bme68x_set_conf(&gas_conf, &gas_sensor);
#ifdef BME680_DEBUG
  Serial.print(F("SetConf Result: "));
  Serial.println(rslt);
#endif
  return rslt == 0;
}

int8_t Adafruit_BME680::readRegister(uint16_t mem_addr, uint8_t *dest,
		uint16_t size, void *intf) {
	if (HAL_OK
			== HAL_I2C_Mem_Read(i2c_han, i2c_addr, mem_addr, 1, dest, size, 10)) {
		return 0;
	} else {
		return -1;
	}
}

int8_t Adafruit_BME680::writeRegister(uint8_t mem_addr, uint8_t *val,
		uint16_t size,  void *intf) {
	if (HAL_OK
			== HAL_I2C_Mem_Write(i2c_han, i2c_addr, mem_addr, 1, val, size, 10)) {
		return 0;
	} else {
		return -1;
	}
}

/*!
 *  @brief  Reads 8 bit values over I2C
 */
int8_t i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf) {

	i2c_interface *_dev = (i2c_interface *)intf;

	if (HAL_OK
			== HAL_I2C_Mem_Read(_dev->i2c_han, _dev->i2c_addr, reg_addr, 1, reg_data, len, 10)) {
		return 0;
	} else {
		return -1;
	}
}

/*!
 *  @brief  Writes 8 bit values over I2C
 */
int8_t i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len,
                 void *intf) {
	i2c_interface *_dev = (i2c_interface *)intf;

	if (HAL_OK
			== HAL_I2C_Mem_Write(_dev->i2c_han, _dev->i2c_addr, reg_addr, 1, const_cast <uint8_t *>(reg_data), len, 10)) {
		return 0;
	} else {
		return -1;
	}
}

static void delay_usec(uint32_t t, void *intf_ptr)
{
  uint32_t start, end;
  start = GetMicros();
  end = start + t;
  if (start < end) {
  	while ((GetMicros() >= start) && (GetMicros() < end)) {
  	  // do nothing
  	}
  } else {
    while ((GetMicros() >= start) || (GetMicros() < end)) {
      // do nothing
    };
  }
//  osDelay(ceil(t/1000.0));

};

uint32_t GetMicros(void){
	return HAL_GetTick() * 1000;
}

bool Adafruit_BME680::bsecGetConfig(uint8_t *config, uint32_t *n_serialized_settings){
	status = bsec_get_configuration(0, config, BSEC_MAX_PROPERTY_BLOB_SIZE, workBuffer, BSEC_MAX_WORKBUFFER_SIZE, n_serialized_settings);

	if (status != BSEC_OK)
		return false;

	return true;
}

bool Adafruit_BME680::bsecGetState(uint8_t *state, uint32_t *n_serialized_state){
	status = bsec_get_state(0, state, BSEC_MAX_STATE_BLOB_SIZE, workBuffer, BSEC_MAX_WORKBUFFER_SIZE, n_serialized_state);

	if (status != BSEC_OK)
		return false;

	return true;
}


bool Adafruit_BME680::bsecSetConfig(const uint8_t *config)
{
    status = bsec_set_configuration(config, BSEC_MAX_PROPERTY_BLOB_SIZE, workBuffer, BSEC_MAX_WORKBUFFER_SIZE);
    if (status != BSEC_OK)
        return false;

    memset(&bmeConf, 0, sizeof(bmeConf));

    return true;
}


bool Adafruit_BME680::bsecSetState(const uint8_t *state){
    status = bsec_set_state(state, BSEC_MAX_STATE_BLOB_SIZE, workBuffer, BSEC_MAX_WORKBUFFER_SIZE);
    if (status != BSEC_OK)
        return false;

    return true;
}

bool Adafruit_BME680::bsecRun(void)
{
    uint8_t nFieldsLeft = 0;
    bme68xData data;
    int64_t currTimeNs = HAL_GetTick() * INT64_C(1000000);
    opMode = bmeConf.op_mode;



    if (currTimeNs >= bmeConf.next_call)
    {
        /* Provides the information about the current sensor configuration that is
           necessary to fulfill the input requirements, eg: operation mode, timestamp
           at which the sensor data shall be fetched etc */
        status = bsec_sensor_control(currTimeNs, &bmeConf);

        if (status != BSEC_OK)
            return false;

        switch (bmeConf.op_mode)
        {
        case BME68X_FORCED_MODE:
            setBme68xConfigForced();
            break;
        case BME68X_PARALLEL_MODE:
            if (opMode != bmeConf.op_mode)
            {
                setBme68xConfigParallel();
            }
            break;

        case BME68X_SLEEP_MODE:
            if (opMode != bmeConf.op_mode)
            {
                setOpMode(BME68X_SLEEP_MODE);
                opMode = BME68X_SLEEP_MODE;
            }
            break;
        }

        if (checkStatus() == BME68X_ERROR)
            return false;

        if (bmeConf.trigger_measurement && bmeConf.op_mode != BME68X_SLEEP_MODE)
        {
            if (fetchData())
            {
                do
                {
                    nFieldsLeft = getData(data);
                    /* check for valid gas data */
                    if (data.status & BME68X_GASM_VALID_MSK)
                    {
                        if (!bsecProcessData(currTimeNs, data))
                        {
                            return false;
                        }
                    }
                } while (nFieldsLeft);
            }

        }

    } else{
//        uint32_t timeLeft_ms = floor( (bmeConf.next_call - currTimeNs) / 1000000);
//        if ( timeLeft_ms > 0){
//        	osDelay(timeLeft_ms);
//        }
    	return false;
    }
    return true;
}

/**
 * @brief Reads data from the BME68X sensor and process it
 */
bool Adafruit_BME680::bsecProcessData(int64_t currTimeNs, const bme68xData &data)
{
    bsec_input_t inputs[BSEC_MAX_PHYSICAL_SENSOR]; /* Temp, Pres, Hum & Gas */
    uint8_t nInputs = 0;
    /* Checks all the required sensor inputs, required for the BSEC library for the requested outputs */
    if (BSEC_CHECK_INPUT(bmeConf.process_data, BSEC_INPUT_HEATSOURCE))
    {
        inputs[nInputs].sensor_id = BSEC_INPUT_HEATSOURCE;
        inputs[nInputs].signal = extTempOffset;
        inputs[nInputs].time_stamp = currTimeNs;
        nInputs++;
    }
    if (BSEC_CHECK_INPUT(bmeConf.process_data, BSEC_INPUT_TEMPERATURE))
    {
#ifdef BME68X_USE_FPU
        inputs[nInputs].sensor_id = BSEC_INPUT_TEMPERATURE;
#else
        inputs[nInputs].sensor_id = BSEC_INPUT_TEMPERATURE / 100.0f;
#endif
        inputs[nInputs].signal = data.temperature;
        inputs[nInputs].time_stamp = currTimeNs;
        nInputs++;
    }
    if (BSEC_CHECK_INPUT(bmeConf.process_data, BSEC_INPUT_HUMIDITY))
    {
#ifdef BME68X_USE_FPU
        inputs[nInputs].sensor_id = BSEC_INPUT_HUMIDITY;
#else
        inputs[nInputs].sensor_id = BSEC_INPUT_HUMIDITY / 1000.0f;
#endif
        inputs[nInputs].signal = data.humidity;
        inputs[nInputs].time_stamp = currTimeNs;
        nInputs++;
    }
    if (BSEC_CHECK_INPUT(bmeConf.process_data, BSEC_INPUT_PRESSURE))
    {
        inputs[nInputs].sensor_id = BSEC_INPUT_PRESSURE;
        inputs[nInputs].signal = data.pressure;
        inputs[nInputs].time_stamp = currTimeNs;
        nInputs++;
    }
    if (BSEC_CHECK_INPUT(bmeConf.process_data, BSEC_INPUT_GASRESISTOR) &&
            (data.status & BME68X_GASM_VALID_MSK))
    {
        inputs[nInputs].sensor_id = BSEC_INPUT_GASRESISTOR;
        inputs[nInputs].signal = data.gas_resistance;
        inputs[nInputs].time_stamp = currTimeNs;
        nInputs++;
    }
    if (BSEC_CHECK_INPUT(bmeConf.process_data, BSEC_INPUT_PROFILE_PART) &&
            (data.status & BME68X_GASM_VALID_MSK))
    {
        inputs[nInputs].sensor_id = BSEC_INPUT_PROFILE_PART;
        inputs[nInputs].signal = (opMode == BME68X_FORCED_MODE) ? 0 : data.gas_index;
        inputs[nInputs].time_stamp = currTimeNs;
        nInputs++;
    }

    if (nInputs > 0)
    {

        outputs.nOutputs = BSEC_NUMBER_OUTPUTS;
        memset(outputs.output, 0, sizeof(outputs.output));

        /* Processing of the input signals and returning of output samples is performed by bsec_do_steps() */
        status = bsec_do_steps(inputs, nInputs, outputs.output, &outputs.nOutputs);

        if (status != BSEC_OK)
            return false;

//        if(bsecDataAvailable)
//		bsecDataAvailable(data, outputs);
    }
    return true;
}

/**
 * @brief Set the BME68X sensor configuration to forced mode
 */
void Adafruit_BME680::setBme68xConfigForced(void)
{
    /* Set the filter, odr, temperature, pressure and humidity settings */
    setTPH(bmeConf.temperature_oversampling, bmeConf.pressure_oversampling, bmeConf.humidity_oversampling);

    if (checkStatus() == BME68X_ERROR)
        return;

    setHeaterProf(bmeConf.heater_temperature, bmeConf.heater_duration);

    if (checkStatus() == BME68X_ERROR)
        return;

   setOpMode(BME68X_FORCED_MODE);
    if (checkStatus() == BME68X_ERROR)
        return;

    opMode = BME68X_FORCED_MODE;
}

/**
 * @brief Set the BME68X sensor configuration to parallel mode
 */
void Adafruit_BME680::setBme68xConfigParallel(void)
{
    uint16_t sharedHeaterDur = 0;

    /* Set the filter, odr, temperature, pressure and humidity settings */
    setTPH(bmeConf.temperature_oversampling, bmeConf.pressure_oversampling, bmeConf.humidity_oversampling);

    if (checkStatus() == BME68X_ERROR)
        return;

    sharedHeaterDur = BSEC_TOTAL_HEAT_DUR - (getMeasDur(BME68X_PARALLEL_MODE) / INT64_C(1000));

    setHeaterProf(bmeConf.heater_temperature_profile, bmeConf.heater_duration_profile, sharedHeaterDur,
            bmeConf.heater_profile_len);

    if (checkStatus() == BME68X_ERROR)
        return;

    setOpMode(BME68X_PARALLEL_MODE);

    if (checkStatus() == BME68X_ERROR)
        return;

    opMode = BME68X_PARALLEL_MODE;
}

/**
 * @brief Function to check if an error / warning has occurred
 */
int8_t Adafruit_BME680::checkStatus(void)
{
	if (bme68xStatus < BME68X_OK)
	{
		return BME68X_ERROR;
	}
	else if(bme68xStatus > BME68X_OK)
	{
		return BME68X_WARNING;
	}
	else
	{
		return BME68X_OK;
	}
}

/**
 * @brief Function to get the measurement duration in microseconds
 */
uint32_t Adafruit_BME680::getMeasDur(uint8_t opMode)
{
	if (opMode == BME68X_SLEEP_MODE)
		opMode = lastOpMode;

	return bme68x_get_meas_dur(opMode, &gas_conf, &gas_sensor);
}

/**
 * @brief Function to set the heater profile for Forced mode
 */
void Adafruit_BME680::setHeaterProf(uint16_t temp, uint16_t dur)
{
	heatrConf.enable = BME68X_ENABLE;
	heatrConf.heatr_temp = temp;
	heatrConf.heatr_dur = dur;

	bme68xStatus = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &heatrConf, &gas_sensor);
}

/**
 * @brief Function to set the heater profile for Sequential mode
 */
void Adafruit_BME680::setHeaterProf(uint16_t *temp, uint16_t *dur, uint8_t profileLen)
{
	heatrConf.enable = BME68X_ENABLE;
	heatrConf.heatr_temp_prof = temp;
	heatrConf.heatr_dur_prof = dur;
	heatrConf.profile_len = profileLen;

	bme68xStatus = bme68x_set_heatr_conf(BME68X_SEQUENTIAL_MODE, &heatrConf, &gas_sensor);

}

/**
 * @brief Function to set the heater profile for Parallel mode
 */
void Adafruit_BME680::setHeaterProf(uint16_t *temp, uint16_t *mul, uint16_t sharedHeatrDur, uint8_t profileLen)
{
	heatrConf.enable = BME68X_ENABLE;
	heatrConf.heatr_temp_prof = temp;
	heatrConf.heatr_dur_prof = mul;
	heatrConf.shared_heatr_dur = sharedHeatrDur;
	heatrConf.profile_len = profileLen;

	bme68xStatus = bme68x_set_heatr_conf(BME68X_PARALLEL_MODE, &heatrConf, &gas_sensor);
}

/**
 * @brief Function to set the operation mode
 */
void Adafruit_BME680::setOpMode(uint8_t opMode)
{
	bme68xStatus = bme68x_set_op_mode(opMode, &gas_sensor);
	if ((bme68xStatus == BME68X_OK) && (opMode != BME68X_SLEEP_MODE))
		lastOpMode = opMode;
}

void Adafruit_BME680::bsecDataAvailable(const bme68xData data, const bsecOutputs outputs)
{
    if (!outputs.nOutputs)
    {
        return;
    }

//    Serial.println("BSEC outputs:\n\ttimestamp = " + String((int) (outputs.output[0].time_stamp / INT64_C(1000000))));
    for (uint8_t i = 0; i < outputs.nOutputs; i++)
    {
        const bsecData output  = outputs.output[i];
        switch (output.sensor_id)
        {
            case BSEC_OUTPUT_IAQ:
//                Serial.println("\tiaq = " + String(output.signal));
//                Serial.println("\tiaq accuracy = " + String((int) output.accuracy));
                break;
            case BSEC_OUTPUT_RAW_TEMPERATURE:
//                Serial.println("\ttemperature = " + String(output.signal));
                break;
            case BSEC_OUTPUT_RAW_PRESSURE:
//                Serial.println("\tpressure = " + String(output.signal));
                break;
            case BSEC_OUTPUT_RAW_HUMIDITY:
//                Serial.println("\thumidity = " + String(output.signal));
                break;
            case BSEC_OUTPUT_RAW_GAS:
//                Serial.println("\tgas resistance = " + String(output.signal));
                break;
            case BSEC_OUTPUT_STABILIZATION_STATUS:
//                Serial.println("\tstabilization status = " + String(output.signal));
                break;
            case BSEC_OUTPUT_RUN_IN_STATUS:
//                Serial.println("\trun in status = " + String(output.signal));
                break;
            default:
                break;
        }
    }
}

///*!
// *  @brief  Reads 8 bit values over SPI
// */
//static int8_t spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len,
//                       void *intf_ptr) {
//  Adafruit_SPIDevice *_dev = (Adafruit_SPIDevice *)intf_ptr;
//
//  reg_addr |= 0x80;
//
//  if (!_dev->write_then_read(&reg_addr, 1, reg_data, len, 0x0)) {
//    return -1;
//  }
//
//  return 0;
//}
//
///*!delay_usecdelay_usec
// *  @brief  Writes 8 bit values over SPI
// */
//static int8_t spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len,
//                        void *intf_ptr) {
//  Adafruit_SPIDevice *_dev = (Adafruit_SPIDevice *)intf_ptr;
//
//  if (!_dev->write((uint8_t *)reg_data, len, &reg_addr, 1)) {
//    return -1;
//  }
//
//  return 0;
//}

//static void delay_usec(uint32_t us, void *intf_ptr) {
//  delayMicroseconds(us);
//  yield();
//}
