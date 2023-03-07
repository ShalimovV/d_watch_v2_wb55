#include "ms5837_lib.h"
#include "stdint.h"
#include "math.h"
#include "cmsis_os.h"

// Constants

// MS5837 device address
#define MS5837_ADDR (0x76 << 1) // 0b1110110

// MS5837 device commands
#define MS5837_RESET_COMMAND 0x1E
#define MS5837_START_PRESSURE_ADC_CONVERSION 0x40
#define MS5837_START_TEMPERATURE_ADC_CONVERSION 0x50
#define MS5837_READ_ADC 0x00

#define MS5837_CONVERSION_OSR_MASK 0x0F

// MS5837 commands
#define MS5837_PROM_ADDRESS_READ_ADDRESS_0 0xA0
#define MS5837_PROM_ADDRESS_READ_ADDRESS_1 0xA2
#define MS5837_PROM_ADDRESS_READ_ADDRESS_2 0xA4
#define MS5837_PROM_ADDRESS_READ_ADDRESS_3 0xA6
#define MS5837_PROM_ADDRESS_READ_ADDRESS_4 0xA8
#define MS5837_PROM_ADDRESS_READ_ADDRESS_5 0xAA
#define MS5837_PROM_ADDRESS_READ_ADDRESS_6 0xAC
#define MS5837_PROM_ADDRESS_READ_ADDRESS_7 0xAE

// Coefficients indexes for temperature and pressure computation
#define MS5837_CRC_INDEX 0
#define MS5837_PRESSURE_SENSITIVITY_INDEX 1
#define MS5837_PRESSURE_OFFSET_INDEX 2
#define MS5837_TEMP_COEFF_OF_PRESSURE_SENSITIVITY_INDEX 3
#define MS5837_TEMP_COEFF_OF_PRESSURE_OFFSET_INDEX 4
#define MS5837_REFERENCE_TEMPERATURE_INDEX 5
#define MS5837_TEMP_COEFF_OF_TEMPERATURE_INDEX 6
#define I2C_TIMEOUT 10
#define MS5837_COEFFICIENT_COUNT 7

#define MS5837_CONVERSION_TIME_OSR_256 1
#define MS5837_CONVERSION_TIME_OSR_512 2
#define MS5837_CONVERSION_TIME_OSR_1024 3
#define MS5837_CONVERSION_TIME_OSR_2048 5
#define MS5837_CONVERSION_TIME_OSR_4096 9
#define MS5837_CONVERSION_TIME_OSR_8192 17


const uint16_t fluidDensity = 1029;
I2C_HandleTypeDef *_MS5837_ui2c;
uint16_t eeprom_coeff[MS5837_COEFFICIENT_COUNT + 1];
bool coeff_read = false;
uint32_t conversion_time[6] = {
      MS5837_CONVERSION_TIME_OSR_256,  MS5837_CONVERSION_TIME_OSR_512,
      MS5837_CONVERSION_TIME_OSR_1024, MS5837_CONVERSION_TIME_OSR_2048,
      MS5837_CONVERSION_TIME_OSR_4096, MS5837_CONVERSION_TIME_OSR_8192};

ms5837_resolution_osr_t ms5837_resolution_osr = ms5837_resolution_osr_8192;

float atm_pressure = 101300;
/**
* \brief Check whether MS5837 device is connected
*
* \return bool : status of MS5837
*       - true : Device is present
*       - false : Device is not acknowledging I2C address
*/
void ms5837_Port_Init(I2C_HandleTypeDef *hi2c) {
	_MS5837_ui2c = hi2c;
}


HAL_StatusTypeDef ms5837_is_connected(void) {
	return HAL_I2C_Master_Transmit(_MS5837_ui2c, MS5837_ADDR, 0x00, 1, I2C_TIMEOUT);
}

/**
* \brief Writes the MS5837 8-bits command with the value passed
*
* \param[in] uint8_t : Command value to be written.
*
* \return ms5837_status : status of MS5837
*       - ms5837_status_ok : I2C transfer completed successfully
*       - ms5837_status_i2c_transfer_error : Problem with i2c transfer
*       - ms5837_status_no_i2c_acknowledge : I2C did not acknowledge
*/
enum ms5837_status ms5837_write_command(uint8_t cmd) {
	HAL_StatusTypeDef i2c_status;
	taskENTER_CRITICAL();
	i2c_status = HAL_I2C_Master_Transmit(_MS5837_ui2c, MS5837_ADDR, &cmd, 1, I2C_TIMEOUT);
	taskEXIT_CRITICAL();

  /* Do the transfer */
  if (i2c_status == HAL_ERROR)
    return ms5837_status_no_i2c_acknowledge;
  if (i2c_status != HAL_OK)
    return ms5837_status_i2c_transfer_error;

  return ms5837_status_ok;
}

/**
* \brief Set  ADC resolution.
*
* \param[in] ms5837_resolution_osr : Resolution requested
*
*/
void ms5837_set_resolution(ms5837_resolution_osr_t res) {
  ms5837_resolution_osr = res;
}

/**
* \brief Reset the MS5837 device
*
* \return ms5837_status : status of MS5837
*       - ms5837_status_ok : I2C transfer completed successfully
*       - ms5837_status_i2c_transfer_error : Problem with i2c transfer
*       - ms5837_status_no_i2c_acknowledge : I2C did not acknowledge
*/
enum ms5837_status ms5837_reset(void) {
  return ms5837_write_command(MS5837_RESET_COMMAND);
}

/**
* \brief Reads the ms5837 EEPROM coefficient stored at address provided.
*
* \param[in] uint8_t : Address of coefficient in EEPROM
* \param[out] uint16_t* : Value read in EEPROM
*
* \return ms5837_status : status of MS5837
*       - ms5837_status_ok : I2C transfer completed successfully
*       - ms5837_status_i2c_transfer_error : Problem with i2c transfer
*       - ms5837_status_no_i2c_acknowledge : I2C did not acknowledge
*       - ms5837_status_crc_error : CRC check error on the coefficients
*/
enum ms5837_status ms5837_read_eeprom_coeff(uint8_t command, uint16_t *coeff) {
  uint8_t buffer[2] = {0,0};
  HAL_StatusTypeDef i2c_status;

  /* Read data */
  taskENTER_CRITICAL();
  i2c_status = HAL_I2C_Master_Transmit(_MS5837_ui2c, MS5837_ADDR, &command, 1, I2C_TIMEOUT);

  HAL_I2C_Master_Receive(_MS5837_ui2c, MS5837_ADDR, &buffer[0], 2,  I2C_TIMEOUT);
  taskEXIT_CRITICAL();
  // Send the conversion command
  if (i2c_status == HAL_ERROR)
    return ms5837_status_no_i2c_acknowledge;

  *coeff = buffer[0] << 8 | buffer[1];

  return ms5837_status_ok;
}

/**
* \brief Reads the ms5837 EEPROM coefficients to store them for computation.
*
* \return ms5837_status : status of MS5837
*       - ms5837_status_ok : I2C transfer completed successfully
*       - ms5837_status_i2c_transfer_error : Problem with i2c transfer
*       - ms5837_status_no_i2c_acknowledge : I2C did not acknowledge
*       - ms5837_status_crc_error : CRC check error on the coefficients
*/
enum ms5837_status ms5837_read_eeprom(void) {
  enum ms5837_status status;
  uint8_t i;

  for (i = 0; i < MS5837_COEFFICIENT_COUNT; i++) {
    status = ms5837_read_eeprom_coeff(MS5837_PROM_ADDRESS_READ_ADDRESS_0 + i * 2,
                               eeprom_coeff + i);
    if (status != ms5837_status_ok)
      return status;
  }
  if (!ms5837_crc_check(eeprom_coeff, (eeprom_coeff[MS5837_CRC_INDEX] & 0xF000) >> 12))
    return ms5837_status_crc_error;

  coeff_read = true;

  return ms5837_status_ok;
}

/**
* \brief CRC check
*
* \param[in] uint16_t *: List of EEPROM coefficients
* \param[in] uint8_t : crc to compare with
*
* \return bool : TRUE if CRC is OK, FALSE if KO
*/
bool ms5837_crc_check(uint16_t *n_prom, uint8_t crc) {
  uint8_t cnt, n_bit;
  uint16_t n_rem, crc_read;

  n_rem = 0x00;
  crc_read = n_prom[0];
  n_prom[MS5837_COEFFICIENT_COUNT] = 0;
  n_prom[0] = (0x0FFF & (n_prom[0])); // Clear the CRC byte

  for (cnt = 0; cnt < (MS5837_COEFFICIENT_COUNT + 1) * 2; cnt++) {

    // Get next byte
    if (cnt % 2 == 1)
      n_rem ^= n_prom[cnt >> 1] & 0x00FF;
    else
      n_rem ^= n_prom[cnt >> 1] >> 8;

    for (n_bit = 8; n_bit > 0; n_bit--) {

      if (n_rem & 0x8000)
        n_rem = (n_rem << 1) ^ 0x3000;
      else
        n_rem <<= 1;
    }
  }
  n_rem >>= 12;
  n_prom[0] = crc_read;

  return (n_rem == crc);
}

/**
* \brief Triggers conversion and read ADC value
*
* \param[in] uint8_t : Command used for conversion (will determine Temperature
* vs Pressure and osr)
* \param[out] uint32_t* : ADC value.
*
* \return ms5837_status : status of MS5837
*       - ms5837_status_ok : I2C transfer completed successfully
*       - ms5837_status_i2c_transfer_error : Problem with i2c transfer
*       - ms5837_status_no_i2c_acknowledge : I2C did not acknowledge
*/
enum ms5837_status ms5837_conversion_and_read_adc(uint8_t cmd, uint32_t *adc) {
  enum ms5837_status status = ms5837_status_ok;
  HAL_StatusTypeDef i2c_status;
  uint8_t buffer[3] = {0, 0, 0};

  /* Read data */
  taskENTER_CRITICAL();
  HAL_I2C_Master_Transmit(_MS5837_ui2c, MS5837_ADDR, &cmd, 1, I2C_TIMEOUT);

  HAL_Delay(conversion_time[(cmd & MS5837_CONVERSION_OSR_MASK) / 2]);
  //HAL_Delay(20);
  //osDelay(conversion_time[(cmd & MS5837_CONVERSION_OSR_MASK) / 2]);

  i2c_status = (HAL_I2C_Master_Transmit(_MS5837_ui2c, MS5837_ADDR, 0x00, 1, I2C_TIMEOUT) != HAL_OK) ;


  HAL_I2C_Master_Receive(_MS5837_ui2c, MS5837_ADDR, &buffer[0], 3,  I2C_TIMEOUT);
  taskEXIT_CRITICAL();

  // delay conversion depending on resolution
  if (status != ms5837_status_ok)
    return status;

  // Send the read command
  // status = ms5837_write_command(MS5837_READ_ADC);
  if (status != ms5837_status_ok)
    return status;

  if (i2c_status == HAL_ERROR)
    return ms5837_status_no_i2c_acknowledge;


  *adc = ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | buffer[2];

  return status;
}

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
*       - ms5837_status_crc_error : CRC check error on the coefficients
*/
enum ms5837_status ms5837_read_temperature_and_pressure(float *temperature,
                                                         float *pressure) {
  enum ms5837_status status = ms5837_status_ok;
  uint32_t adc_temperature, adc_pressure;
  int32_t dT, TEMP;
  int64_t OFF, SENS, P, T2, OFF2, SENS2;
  uint8_t cmd;

  // If first time adc is requested, get EEPROM coefficients
  if (coeff_read == false)
    status = ms5837_read_eeprom();

  if (status != ms5837_status_ok)
    return status;

  // First read temperature
  cmd = ms5837_resolution_osr * 2;
  cmd |= MS5837_START_TEMPERATURE_ADC_CONVERSION;
  status = ms5837_conversion_and_read_adc(cmd, &adc_temperature);
  if (status != ms5837_status_ok)
    return status;

  // Now read pressure
  cmd = ms5837_resolution_osr * 2;
  cmd |= MS5837_START_PRESSURE_ADC_CONVERSION;
  status = ms5837_conversion_and_read_adc(cmd, &adc_pressure);
  if (status != ms5837_status_ok)
    return status;

  if (adc_temperature == 0 || adc_pressure == 0)
    return ms5837_status_i2c_transfer_error;

  // Difference between actual and reference temperature = D2 - Tref
  dT = (int32_t)adc_temperature -
       ((int32_t)eeprom_coeff[MS5837_REFERENCE_TEMPERATURE_INDEX] << 8);

  // Actual temperature = 2000 + dT * TEMPSENS
  TEMP = 2000 +
         ((int64_t)dT *
              (int64_t)eeprom_coeff[MS5837_TEMP_COEFF_OF_TEMPERATURE_INDEX] >>
          23);

  // Second order temperature compensation
  if (TEMP < 2000) {
    T2 = (3 * ((int64_t)dT * (int64_t)dT)) >> 33;
    OFF2 = 3 * ((int64_t)TEMP - 2000) * ((int64_t)TEMP - 2000) >> 1;
    SENS2 = 5 * ((int64_t)TEMP - 2000) * ((int64_t)TEMP - 2000) >> 3;
  } else {
    T2 = (2 * ((int64_t)dT * (int64_t)dT)) >> 37;
    OFF2 = 1 * ((int64_t)TEMP - 2000) * ((int64_t)TEMP - 2000) >> 4;
    SENS2 = 0;
  }

  // OFF = OFF_T1 + TCO * dT
  OFF = ((int64_t)(eeprom_coeff[MS5837_PRESSURE_OFFSET_INDEX]) << 16) +
        (((int64_t)(eeprom_coeff[MS5837_TEMP_COEFF_OF_PRESSURE_OFFSET_INDEX]) *
          dT) >>
         7);
  OFF -= OFF2;

  // Sensitivity at actual temperature = SENS_T1 + TCS * dT
  SENS =
      ((int64_t)eeprom_coeff[MS5837_PRESSURE_SENSITIVITY_INDEX] << 15) +
      (((int64_t)eeprom_coeff[MS5837_TEMP_COEFF_OF_PRESSURE_SENSITIVITY_INDEX] *
        dT) >>
       8);
  SENS -= SENS2;

  // Temperature compensated pressure = D1 * SENS - OFF
  P = (((adc_pressure * SENS) >> 21) - OFF) >> 13;

  *temperature = ((float)TEMP - T2) / 100;
  *pressure = (float)P / 10;

  return status;
}

// The pressure sensor measures absolute pressure, so it will measure the atmospheric pressure + water pressure
// We subtract the atmospheric pressure to calculate the depth with only the water pressure
// The average atmospheric pressure of 101300 pascal is used for the calcuation, but atmospheric pressure varies
// If the atmospheric pressure is not 101300 at the time of reading, the depth reported will be offset
// In order to calculate the correct depth, the actual atmospheric pressure should be measured once in air, and
// that value should subtracted for subsequent depth calculations.
float MS5837_depth(float pressure) {
	return ((pressure * 100) - atm_pressure) / (fluidDensity * 9.80665);
}

float MS5837_altitude(float pressure) {
	return (1-pow((pressure/(atm_pressure / 100)),.190284))*145366.45*.3048;
}

void MS5837_set_atm_pressure(float pressure) {
	atm_pressure = pressure * 100;
}

