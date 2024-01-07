/*!
 * @file Adafruit_BME680.h
 *
 * Adafruit BME680 temperature, humidity, barometric pressure and gas sensor
 * driver
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
 * Written by Ladyada for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

#ifndef __BME680_H__
#define __BME680_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32wbxx_hal.h"
#include "bme68x.h"
#include <Adafruit_Sensor.h>

#include "bsec_datatypes.h"
#include "bsec_interface.h"

typedef struct bme68x_data          bme68xData;
typedef struct bme68x_dev           bme68xDev;
typedef enum   bme68x_intf          bme68xIntf;
typedef struct bme68x_conf          bme68xConf;
typedef struct bme68x_heatr_conf    bme68xHeatrConf;

#define BME68X_ERROR            INT8_C(-1)
#define BME68X_WARNING          INT8_C(1)

#define SEALEVELPRESSURE_HPA (1016.20)

#define BME68X_DEFAULT_ADDRESS (0x76)    ///< The default I2C address
#define BME68X_DEFAULT_SPIFREQ (1000000) ///< The default SPI Clock speed

#define BME680_OS_16X BME68X_OS_16X   ///< Alias for BME680 existing examples
#define BME680_OS_8X BME68X_OS_8X     ///< Alias for BME680 existing examples
#define BME680_OS_4X BME68X_OS_4X     ///< Alias for BME680 existing examples
#define BME680_OS_2X BME68X_OS_2X     ///< Alias for BME680 existing examples
#define BME680_OS_1X BME68X_OS_1X     ///< Alias for BME680 existing examples
#define BME680_OS_NONE BME68X_OS_NONE ///< Alias for BME680 existing examples

#define BME680_FILTER_SIZE_127                                                 \
  BME68X_FILTER_SIZE_127 ///< Alias for BME680 existing examples
#define BME680_FILTER_SIZE_63                                                  \
  BME68X_FILTER_SIZE_63 ///< Alias for BME680 existing examples
#define BME680_FILTER_SIZE_31                                                  \
  BME68X_FILTER_SIZE_31 ///< Alias for BME680 existing examples
#define BME680_FILTER_SIZE_15                                                  \
  BME68X_FILTER_SIZE_15 ///< Alias for BME680 existing examples
#define BME680_FILTER_SIZE_7                                                   \
  BME68X_FILTER_SIZE_7 ///< Alias for BME680 existing examples
#define BME680_FILTER_SIZE_3                                                   \
  BME68X_FILTER_SIZE_3 ///< Alias for BME680 existing examples
#define BME680_FILTER_SIZE_1                                                   \
  BME68X_FILTER_SIZE_1 ///< Alias for BME680 existing examples
#define BME680_FILTER_SIZE_0                                                   \
  BME68X_FILTER_OFF ///< Alias for BME680 existing examples

#define BSEC_CHECK_INPUT(x, shift)		(x & (1 << (shift-1)))
#define BSEC_TOTAL_HEAT_DUR             UINT16_C(140)

typedef bsec_output_t bsecData;
typedef bsec_virtual_sensor_t bsecSensor;

typedef struct
{
    bsecData output[BSEC_NUMBER_OUTPUTS];
    uint8_t nOutputs;
} bsecOutputs;

struct i2c_interface
{
	I2C_HandleTypeDef *i2c_han;
	uint8_t i2c_addr;
};
/*! Adafruit_BME680 Class for both I2C and SPI usage.
 *  Wraps the Bosch library for Arduino usage
 */
class Adafruit_BME680 {
public:
  /** Value returned by remainingReadingMillis indicating no asynchronous
   * reading has been initiated by beginReading. **/
  static constexpr int reading_not_started = -1;
  /** Value returned by remainingReadingMillis indicating asynchronous reading
   * is complete and calling endReading will not block. **/
  static constexpr int reading_complete = 0;

//  Adafruit_BME680(TwoWire *theWire = &Wire);
//  Adafruit_BME680(int8_t cspin, SPIClass *theSPI = &SPI);
//  Adafruit_BME680(int8_t cspin, int8_t mosipin, int8_t misopin, int8_t sckpin);
  Adafruit_BME680(void);

  bool begin(uint8_t i2c_address = BME68X_DEFAULT_ADDRESS, I2C_HandleTypeDef *i2c_handle = NULL, bool initSettings = true);
  float readTemperature();
  float readPressure();
  float readHumidity();
  uint32_t readGas();
  float readAltitude(float seaLevel);

  void soft_reset(void);

  bool setTemperatureOversampling(uint8_t os);
  bool setPressureOversampling(uint8_t os);
  bool setHumidityOversampling(uint8_t os);
  bool setIIRFilterSize(uint8_t fs);
  bool setGasHeater(uint16_t heaterTemp, uint16_t heaterTime);
  bool setODR(uint8_t odr);
  uint32_t getMeasDur(uint8_t opMode);

  bool bsecRun();
  bool bsecProcessData(int64_t currTimeNs, const bme68xData &data);

  void setBme68xConfigForced(void);
  void setBme68xConfigParallel(void);

  void setTPH(uint8_t osTemp, uint8_t osPres, uint8_t osHum);

  bool bsecSetConfig(const uint8_t *config);
  bool bsecGetConfig( uint8_t *config, uint32_t *n_serialized_settings);
  bool bsecGetState( uint8_t *state, uint32_t *n_serialized_state);
  bool bsecSetState(const uint8_t *state);

  /**
   * @brief Function to set the heater profile for Forced mode
   * @param temp : Heater temperature in degree Celsius
   * @param dur  : Heating duration in milliseconds
   */
  void setHeaterProf(uint16_t temp, uint16_t dur);

  /**
   * @brief Function to set the heater profile for Sequential mode
   * @param temp       : Heater temperature profile in degree Celsius
   * @param dur        : Heating duration profile in milliseconds
   * @param profileLen : Length of the profile
   */
  void setHeaterProf(uint16_t *temp, uint16_t *dur, uint8_t profileLen);

  /**
   * @brief Function to set the heater profile for Parallel mode
   * @param temp           : Heater temperature profile in degree Celsius
   * @param mul            : Profile of number of repetitions
   * @param sharedHeatrDur : Shared heating duration in milliseconds
   * @param profileLen     : Length of the profile
   */
  void setHeaterProf(uint16_t *temp, uint16_t *mul, uint16_t sharedHeatrDur, uint8_t profileLen);

  /**
     * @brief Function to set the operation mode
     * @param opMode : BME68X_SLEEP_MODE, BME68X_FORCED_MODE, BME68X_PARALLEL_MODE, BME68X_SEQUENTIAL_MODE
     */
  void setOpMode(uint8_t opMode);

  int8_t checkStatus(void);


  // Perform a reading in blocking mode.
  bool performReading();

  uint32_t beginReading();

  bool endReading();

  int remainingReadingMillis();

  /** Temperature (Celsius) assigned after calling performReading() or
   * endReading() **/
  float temperature;
  /** Pressure (Pascals) assigned after calling performReading() or endReading()
   * **/
  uint32_t pressure;
  /** Humidity (RH %) assigned after calling performReading() or endReading()
   * **/
  float humidity;
  /** Gas resistor (ohms) assigned after calling performReading() or
   * endReading() **/
  uint32_t gas_resistance;

  int8_t writeRegister(uint8_t mem_addr, uint8_t *val, uint16_t size, void *intf);
  int8_t readRegister(uint16_t mem_addr, uint8_t *dest, uint16_t size, void *intf);

  uint8_t fetchData(void);
  uint8_t getData(bme68xData &data);

  void bsecDataAvailable(const bme68xData data, const bsecOutputs outputs);
  bool bsecSubscribe(void);

  bsec_version_t version;
  bsec_library_return_t status;
  int8_t bme68xStatus;
  bme68xHeatrConf heatrConf;
  bme68xData sensorData[3];

  uint8_t nFields, iFields;
  uint8_t lastOpMode;


  bsec_bme_settings_t bmeConf;
  bsecOutputs outputs;

private:
	I2C_HandleTypeDef *i2c_han = NULL;///< Pointer to I2C bus interface
	uint8_t i2c_addr = 0;
	struct i2c_interface i2c_dev;

  int32_t _sensorID;
  uint32_t _meas_start = 0;
  uint16_t _meas_period = 0;

  float extTempOffset;

  /* operating mode of sensor */
  uint8_t opMode;

  struct bme68x_dev gas_sensor;
  struct bme68x_conf gas_conf;
  struct bme68x_heatr_conf gas_heatr_conf;
};

#ifdef __cplusplus
}
#endif

#endif
