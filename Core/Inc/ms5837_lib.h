#include "stm32wbxx_hal.h"
#include "stdint.h"
#include <stdbool.h>


// Enum
typedef enum {
  ms5837_resolution_osr_256 = 0,
  ms5837_resolution_osr_512,
  ms5837_resolution_osr_1024,
  ms5837_resolution_osr_2048,
  ms5837_resolution_osr_4096,
  ms5837_resolution_osr_8192
}ms5837_resolution_osr_t;

enum ms5837_status {
  ms5837_status_ok,
  ms5837_status_no_i2c_acknowledge,
  ms5837_status_i2c_transfer_error,
  ms5837_status_crc_error
};

enum ms5837_status_code {
  ms5837_STATUS_OK = 0,
  ms5837_STATUS_ERR_OVERFLOW = 1,
  ms5837_STATUS_ERR_TIMEOUT = 4
};


void ms5837_Port_Init(I2C_HandleTypeDef *hi2c);
  /**
  * \brief Check whether MS5837 device is connected
  *
  * \return bool : status of MS5837
  *       - true : Device is present
  *       - false : Device is not acknowledging I2C address
  */
HAL_StatusTypeDef ms5837_is_connected(void);

  /**
  * \brief Reset the MS5837 device
  *
  * \return ms5837_status : status of MS5837
  *       - ms5837_status_ok : I2C transfer completed successfully
  *       - ms5837_status_i2c_transfer_error : Problem with i2c transfer
  *       - ms5837_status_no_i2c_acknowledge : I2C did not acknowledge
  */
  enum ms5837_status reset(void);

  /**
  * \brief Set  ADC resolution.
  *
  * \param[in] ms5837_resolution_osr : Resolution requested
  *
  */
  void set_resolution(ms5837_resolution_osr_t res);

  /**
  * \brief Reads the temperature and pressure ADC value and compute the
  * compensated values.
  *
  * \param[out] float* : Celsius Degree temperature value
  * \param[out] float* : mbar pressure value
  *
  * \return ms5837_status : status of MS5837
  *       - ms5837_status_ok : I2C transfer completed successfully
  *       - ms5837_status_i2c_transfer_error : Problem with i2c transfer
  *       - ms5837_status_no_i2c_acknowledge : I2C did not acknowledge
  *       - ms5837_status_crc_error : CRC check error on on the PROM
  * coefficients
  */
  enum ms5837_status ms5837_read_temperature_and_pressure(float *temperature,
                                                   float *pressure);


  enum ms5837_status write_command(uint8_t cmd);
  enum ms5837_status ms5837_read_eeprom_coeff(uint8_t command, uint16_t *coeff);
  bool ms5837_crc_check(uint16_t *n_prom, uint8_t crc);
  enum ms5837_status ms5837_conversion_and_read_adc(uint8_t cmd, uint32_t *adc);
  enum ms5837_status ms5837_read_eeprom(void);

  enum ms5837_status ms5837_write_command(uint8_t);
  enum ms5837_status ms5837_read_eeprom(void);
  float MS5837_depth(float pressure);
  float MS5837_altitude(float pressure);
  enum ms5837_status ms5837_reset(void);
  void MS5837_set_atm_pressure(float pressure);

