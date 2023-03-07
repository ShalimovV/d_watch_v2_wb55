#include "lsm303c.h"

AxesRaw_t accelData = {0, 0, 0};
AxesRaw_t   magData = {0, 0, 0};

const char _bearings[16][3] =  {
		{' ', ' ', 'N'},
		{'N', 'N', 'E'},
		{' ', 'N', 'E'},
		{'E', 'N', 'E'},
		{' ', ' ', 'E'},
		{'E', 'S', 'E'},
		{' ', 'S', 'E'},
		{'S', 'S', 'E'},
		{' ', ' ', 'S'},
		{'S', 'S', 'W'},
		{' ', 'S', 'W'},
		{'W', 'S', 'W'},
		{' ', ' ', 'W'},
		{'W', 'N', 'W'},
		{' ', 'N', 'W'},
		{'N', 'N', 'W'},
	};

#ifdef SPI_SOFTWARE
InterfaceMode_t interfaceMode = MODE_SPI;
#endif

#ifdef SPI_HARDWARE
InterfaceMode_t interfaceMode = MODE_SPI;
#endif

#ifdef I2C_INTERFACE
InterfaceMode_t interfaceMode = MODE_I2C;
I2C_HandleTypeDef *i2c_handle;
#endif

uint8_t who_is_mag = 0, who_is_acc = 0;

#ifdef SPI_HARDWARE
SPI_HandleTypeDef *spi_handle;
#endif

#ifdef SPI_SOFTWARE
bool LSM303C_init() {

	//GPIO_InitTypeDef GPIO_InitStruct = {0};
	 /*Configure GPIO pin Output Level */
	/*
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_7, GPIO_PIN_SET);
	  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_7;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	  */
	if (LSM303C_begin(// Default to I2C bus
							  ///// Interface mode options

							  MODE_SPI,
							  //MODE_I2C,
							  // Initialize magnetometer output data rate to 0.625 Hz (turn on device)
							  ///// Magnetometer output data rate options
							  //MAG_DO_0_625_Hz,
					          //MAG_DO_1_25_Hz,
					          //MAG_DO_2_5_Hz,
					          //MAG_DO_5_Hz,
					          MAG_DO_10_Hz,
					          //MAG_DO_20_Hz,
					          //MAG_DO_40_Hz,
					          //MAG_DO_80_Hz,
							  //// Initialize magnetic field full scale to +/-16 gauss
							  MAG_FS_4_Ga,
							  //MAG_FS_8_Ga,
							  //MAG_FS_12_Ga,
							  //MAG_FS_16_Ga,
							  //// Enabling block data updating
							  MAG_BDU_ENABLE,
							  //MAG_BDU_DISABLE,
							  //// Initialize magnetometer X/Y axes ouput data rate to high-perf mode
							  MAG_OMXY_LOW_POWER,
							  //MAG_OMXY_MEDIUM_PERFORMANCE,
							  //MAG_OMXY_HIGH_PERFORMANCE,
							  //MAG_OMXY_ULTRA_HIGH_PERFORMANCE,
							  //// Initialize magnetometer Z axis performance mode
							  MAG_OMZ_LOW_PW,
							  //MAG_OMZ_MEDIUM_PERFORMANCE,
							  //MAG_OMZ_HIGH_PERFORMANCE,
							  //MAG_OMZ_ULTRA_HIGH_PERFORMANCE,
							  //// Initialize magnetometer run mode. Also enables I2C (bit 7 = 0)
							  MAG_MD_CONTINUOUS,
							  //MAG_MD_SINGLE,
							  //MAG_MD_POWER_DOWN_1,
							  //MAG_MD_POWER_DOWN_2,
							  //// Initialize acceleration full scale to +/-2g
							  ACC_FS_2g,
							  //ACC_FS_4g,
							  //ACC_FS_8g,
							  //// Enable block data updating
							  ACC_BDU_ENABLE,
							  //ACC_BDU_DISABLE,
							  //// Enable X, Y, and Z accelerometer axes
							  //ACC_DISABLE_ALL,
							  //ACC_X_ENABLE,
							  //ACC_Y_ENABLE,
							  //ACC_Z_ENABLE,
							  //ACC_X_ENABLE|ACC_Y_ENABLE,
							  //ACC_X_ENABLE|ACC_Z_ENABLE,
							  //ACC_Y_ENABLE|ACC_Z_ENABLE,
							  ACC_X_ENABLE|ACC_Y_ENABLE|ACC_Z_ENABLE,
							  //// Initialize accelerometer output data rate to 100 Hz (turn on device)
							  //ACC_ODR_POWER_DOWN
							  ACC_ODR_10_Hz
							  //ACC_ODR_50_Hz
							  //ACC_ODR_100_Hz
							  //ACC_ODR_200_Hz
							  //ACC_ODR_400_Hz
							  //ACC_ODR_800_Hz
	          ) == IMU_SUCCESS) return true;
	else return false;

}

status_t LSM303C_SPI_WriteByte(CHIP_t chip, uint8_t reg, uint8_t data)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  uint8_t counter;
  uint16_t twoBytes;

  // Clear the read/write bit (bit 7) to do a write
  reg &= ~_BV(7);
  twoBytes = reg << 8 | data;

  // Set data pin to output
  //bitSet(DIR_REG, DATABIT);
  /*Configure GPIO pins : spi_sck_Pin spi_mosi_Pin */
    GPIO_InitStruct.Pin = DATABIT;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  __disable_irq();

  // Select the chip & deselect the other
  switch (chip)
  {
  case MAG:
    //bitClear(CSPORT_MAG, CSBIT_MAG);
    HAL_GPIO_WritePin(CSPORT_MAG, CSBIT_MAG, GPIO_PIN_RESET);
    //bitSet(CSPORT_XL, CSBIT_XL);
    HAL_GPIO_WritePin(CSPORT_XL, CSBIT_XL, GPIO_PIN_SET);
    break;
  case ACC:
    //bitClear(CSPORT_XL, CSBIT_XL);
    HAL_GPIO_WritePin(CSPORT_XL, CSBIT_XL, GPIO_PIN_RESET);
    //bitSet(CSPORT_MAG, CSBIT_MAG);
    HAL_GPIO_WritePin(CSPORT_MAG, CSBIT_MAG, GPIO_PIN_SET);
    break;
  }

  // Shift out 8-bit address & 8-bit data
  for(counter = 16; counter; counter--)
  {
    //bitWrite(DATAPORTO, DATABIT, twoBytes & 0x8000);
	  if (twoBytes & 0x8000) {
		  HAL_GPIO_WritePin(DATAPORTO, DATABIT, GPIO_PIN_SET);
	  }
	  else {
		  HAL_GPIO_WritePin(DATAPORTO, DATABIT, GPIO_PIN_RESET);
	  }
    // Data is setup, so drop clock edge
    //bitClear(CLKPORT, CLKBIT);
    HAL_GPIO_WritePin(CLKPORT, CLKBIT, GPIO_PIN_RESET);
    //bitSet(CLKPORT, CLKBIT);
    HAL_GPIO_WritePin(CLKPORT, CLKBIT, GPIO_PIN_SET);
    // Shift off sent bit
    twoBytes <<= 1;
  }

  // Unselect chip
  switch (chip)
  {
  case MAG:
    //bitSet(CSPORT_MAG, CSBIT_MAG);
	  HAL_GPIO_WritePin(CSPORT_MAG, CSBIT_MAG, GPIO_PIN_SET);
    break;
  case ACC:
    //bitSet(CSPORT_XL, CSBIT_XL);
	  HAL_GPIO_WritePin(CSPORT_XL, CSBIT_XL, GPIO_PIN_SET);
    break;
  }

  __enable_irq();

  // Set data pin to input
  //bitClear(DIR_REG, DATABIT);
  /*Configure GPIO pins : spi_sck_Pin spi_mosi_Pin */
      GPIO_InitStruct.Pin = DATABIT;
      GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
      HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Is there a way to verify true success?
  return IMU_SUCCESS;
}

// This function uses bit manibulation for higher speed & smaller code
uint8_t LSM303C_SPI_ReadByte(CHIP_t chip, uint8_t data)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  uint8_t counter;

  // Set the read/write bit (bit 7) to do a read
  data |= _BV(7);

  // Set data pin to output
  //bitSet(DIR_REG, DATABIT);
  /*Configure GPIO pins : spi_sck_Pin spi_mosi_Pin */
      GPIO_InitStruct.Pin = DATABIT;
      GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
      HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  __disable_irq();

  // Select the chip & deselect the other
  switch (chip)
   {
   case MAG:
     //bitClear(CSPORT_MAG, CSBIT_MAG);
     HAL_GPIO_WritePin(CSPORT_MAG, CSBIT_MAG, GPIO_PIN_RESET);
     //bitSet(CSPORT_XL, CSBIT_XL);
     HAL_GPIO_WritePin(CSPORT_XL, CSBIT_XL, GPIO_PIN_SET);
     break;
   case ACC:
     //bitClear(CSPORT_XL, CSBIT_XL);
     HAL_GPIO_WritePin(CSPORT_XL, CSBIT_XL, GPIO_PIN_RESET);
     //bitSet(CSPORT_MAG, CSBIT_MAG);
     HAL_GPIO_WritePin(CSPORT_MAG, CSBIT_MAG, GPIO_PIN_SET);
     break;
   }

  // Shift out 8-bit address
  for(counter = 8; counter; counter--)
  {
    //bitWrite(DATAPORTO, DATABIT, data & 0x80);
	if (data & 0x80) {
		HAL_GPIO_WritePin(DATAPORTO, DATABIT, GPIO_PIN_SET);
	}
	else {
		HAL_GPIO_WritePin(DATAPORTO, DATABIT, GPIO_PIN_RESET);
	}

    // Data is setup, so drop clock edge
    //bitClear(CLKPORT, CLKBIT);
	HAL_GPIO_WritePin(CLKPORT, CLKBIT, GPIO_PIN_RESET);
	//bitSet(CLKPORT, CLKBIT);
	HAL_GPIO_WritePin(CLKPORT, CLKBIT, GPIO_PIN_SET);

    // Shift off sent bit
    data <<= 1;
  }

  // Switch data pin to input (0 = INPUT)
  //bitClear(DIR_REG, DATABIT);
  GPIO_InitStruct.Pin = DATABIT;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Shift in register data from address
  for(counter = 8; counter; counter--)
  {
    // Shift data to the left.  Remains 0 after first shift
    data <<= 1;

    //bitClear(CLKPORT, CLKBIT);
    HAL_GPIO_WritePin(CLKPORT, CLKBIT, GPIO_PIN_RESET);
    // Sample on rising egde
    //bitSet(CLKPORT, CLKBIT);
    HAL_GPIO_WritePin(CLKPORT, CLKBIT, GPIO_PIN_SET);
    if (HAL_GPIO_ReadPin(DATAPORTO, DATABIT) == GPIO_PIN_SET)
    {
      data |= 0x01;
    }
  }

  // Unselect chip
    switch (chip)
    {
    case MAG:
      //bitSet(CSPORT_MAG, CSBIT_MAG);
  	  HAL_GPIO_WritePin(CSPORT_MAG, CSBIT_MAG, GPIO_PIN_SET);
      break;
    case ACC:
      //bitSet(CSPORT_XL, CSBIT_XL);
  	  HAL_GPIO_WritePin(CSPORT_XL, CSBIT_XL, GPIO_PIN_SET);
      break;
    }

  __enable_irq();

  return(data);
}

#endif

#ifdef SPI_HARDWARE

bool LSM303C_init(SPI_HandleTypeDef *h_spi) {
	spi_handle = h_spi;
	if (LSM303C_begin(// Default to I2C bus
							  ///// Interface mode options
							  MODE_SPI,
							  //MODE_I2C,
							  // Initialize magnetometer output data rate to 0.625 Hz (turn on device)
							  ///// Magnetometer output data rate options
							  //MAG_DO_0_625_Hz,
					          //MAG_DO_1_25_Hz,
					          //MAG_DO_2_5_Hz,
					          //MAG_DO_5_Hz,
					          //MAG_DO_10_Hz,
					          //MAG_DO_20_Hz,
					          MAG_DO_40_Hz,
					          //MAG_DO_80_Hz,
							  //// Initialize magnetic field full scale to +/-16 gauss
							  //MAG_FS_4_Ga,
							  //MAG_FS_8_Ga,
							  //MAG_FS_12_Ga,
							   MAG_FS_16_Ga,
							  //// Enabling block data updating
							  MAG_BDU_ENABLE,
							  //MAG_BDU_DISABLE,
							  //// Initialize magnetometer X/Y axes ouput data rate to high-perf mode
							  //MAG_OMXY_LOW_POWER,
							  //MAG_OMXY_MEDIUM_PERFORMANCE,
							  MAG_OMXY_HIGH_PERFORMANCE,
							  //MAG_OMXY_ULTRA_HIGH_PERFORMANCE,
							  //// Initialize magnetometer Z axis performance mode
							  //MAG_OMZ_LOW_PW,
							  //MAG_OMZ_MEDIUM_PERFORMANCE,
							  MAG_OMZ_HIGH_PERFORMANCE,
							  //MAG_OMZ_ULTRA_HIGH_PERFORMANCE,
							  //// Initialize magnetometer run mode. Also enables I2C (bit 7 = 0)
							  MAG_MD_CONTINUOUS,
							  //MAG_MD_SINGLE,
							  //MAG_MD_POWER_DOWN_1,
							  //MAG_MD_POWER_DOWN_2,
							  //// Initialize acceleration full scale to +/-2g
							  ACC_FS_2g,
							  //ACC_FS_4g,
							  //ACC_FS_8g,
							  //// Enable block data updating
							  ACC_BDU_ENABLE,
							  //ACC_BDU_DISABLE,
							  //// Enable X, Y, and Z accelerometer axes
							  //ACC_DISABLE_ALL,
							  //ACC_X_ENABLE,
							  //ACC_Y_ENABLE,
							  //ACC_Z_ENABLE,
							  //ACC_X_ENABLE|ACC_Y_ENABLE,
							  //ACC_X_ENABLE|ACC_Z_ENABLE,
							  //ACC_Y_ENABLE|ACC_Z_ENABLE,
							  ACC_X_ENABLE|ACC_Y_ENABLE|ACC_Z_ENABLE,
							  //// Initialize accelerometer output data rate to 100 Hz (turn on device)
							  //ACC_ODR_POWER_DOWN
							  //ACC_ODR_10_Hz
							  //ACC_ODR_50_Hz
							  ACC_ODR_100_Hz
							  //ACC_ODR_200_Hz
							  //ACC_ODR_400_Hz
							  //ACC_ODR_800_Hz
	          ) == IMU_SUCCESS) return true;
	else return false;

}


status_t LSM303C_SPI_WriteByte_HW(CHIP_t chip, uint8_t reg, uint8_t data)
{
	uint8_t twoBytes[2];
	// Clear the read/write bit (bit 7) to do a write
	  reg &= ~_BV(7);
	  //twoBytes = reg << 8 | data;
	  twoBytes[0] = reg;
	  twoBytes[1] = data;
	  __disable_irq();

	  // Select the chip & deselect the other
	    switch (chip)
	    {
	    case MAG:
	      //bitClear(CSPORT_MAG, CSBIT_MAG);
	      HAL_GPIO_WritePin(CSPORT_MAG, CSBIT_MAG, GPIO_PIN_RESET);
	      //bitSet(CSPORT_XL, CSBIT_XL);
	      HAL_GPIO_WritePin(CSPORT_XL, CSBIT_XL, GPIO_PIN_SET);
	      break;
	    case ACC:
	      //bitClear(CSPORT_XL, CSBIT_XL);
	      HAL_GPIO_WritePin(CSPORT_XL, CSBIT_XL, GPIO_PIN_RESET);
	      //bitSet(CSPORT_MAG, CSBIT_MAG);
	      HAL_GPIO_WritePin(CSPORT_MAG, CSBIT_MAG, GPIO_PIN_SET);
	      break;
	    }
	    SPI_1LINE_TX(spi_handle);
	   // __HAL_SPI_ENABLE(spi_handle);
	    spi_handle->Instance->CR1 |= (1<<15);

	    HAL_SPI_Transmit(spi_handle, &twoBytes[0], 2, SPI_TIMEOUT);

	    // Unselect chip
	     switch (chip)
	     {
	     case MAG:
	       //bitSet(CSPORT_MAG, CSBIT_MAG);
	   	  HAL_GPIO_WritePin(CSPORT_MAG, CSBIT_MAG, GPIO_PIN_SET);
	       break;
	     case ACC:
	       //bitSet(CSPORT_XL, CSBIT_XL);
	   	  HAL_GPIO_WritePin(CSPORT_XL, CSBIT_XL, GPIO_PIN_SET);
	       break;
	     }

	     __enable_irq();

	     return IMU_SUCCESS;
}

uint8_t LSM303C_SPI_ReadByte_HW(CHIP_t chip, uint8_t data)
{
	// Set the read/write bit (bit 7) to do a read
	  data |= _BV(7);

	  __disable_irq();

	    // Select the chip & deselect the other
	    switch (chip)
	     {
	     case MAG:
	       //bitClear(CSPORT_MAG, CSBIT_MAG);
	       HAL_GPIO_WritePin(CSPORT_MAG, CSBIT_MAG, GPIO_PIN_RESET);
	       //bitSet(CSPORT_XL, CSBIT_XL);
	       HAL_GPIO_WritePin(CSPORT_XL, CSBIT_XL, GPIO_PIN_SET);
	       break;
	     case ACC:
	       //bitClear(CSPORT_XL, CSBIT_XL);
	       HAL_GPIO_WritePin(CSPORT_XL, CSBIT_XL, GPIO_PIN_RESET);
	       //bitSet(CSPORT_MAG, CSBIT_MAG);
	       HAL_GPIO_WritePin(CSPORT_MAG, CSBIT_MAG, GPIO_PIN_SET);
	       break;
	     }
	    spi_handle->Instance->CR1 |= (1<<15);
	    SPI_1LINE_TX(spi_handle);
	    HAL_SPI_Transmit(spi_handle, &data, 1, SPI_TIMEOUT);
	    SPI_1LINE_RX(spi_handle);
	    HAL_SPI_Receive(spi_handle, &data, 1, SPI_TIMEOUT);

	    // Unselect chip
	        switch (chip)
	        {
	        case MAG:
	          //bitSet(CSPORT_MAG, CSBIT_MAG);
	      	  HAL_GPIO_WritePin(CSPORT_MAG, CSBIT_MAG, GPIO_PIN_SET);
	          break;
	        case ACC:
	          //bitSet(CSPORT_XL, CSBIT_XL);
	      	  HAL_GPIO_WritePin(CSPORT_XL, CSBIT_XL, GPIO_PIN_SET);
	          break;
	        }

	      __enable_irq();

	      return(data);
}

#endif

#ifdef I2C_INTERFACE
bool LSM303C_init(I2C_HandleTypeDef *h_i2c) {
	i2c_handle = h_i2c;
	if (LSM303C_begin(// Default to I2C bus
							  ///// Interface mode options

							  //MODE_SPI,
							  MODE_I2C,
							  // Initialize magnetometer output data rate to 0.625 Hz (turn on device)
							  ///// Magnetometer output data rate options
							  //MAG_DO_0_625_Hz,
					          //MAG_DO_1_25_Hz,
					          //MAG_DO_2_5_Hz,
					          //MAG_DO_5_Hz,
					          //MAG_DO_10_Hz,
					          //MAG_DO_20_Hz,
					          MAG_DO_40_Hz,
					          //MAG_DO_80_Hz,
							  //// Initialize magnetic field full scale to +/-16 gauss
							  //MAG_FS_4_Ga,
							  //MAG_FS_8_Ga,
							  //MAG_FS_12_Ga,
							   MAG_FS_16_Ga,
							  //// Enabling block data updating
							  MAG_BDU_ENABLE,
							  //MAG_BDU_DISABLE,
							  //// Initialize magnetometer X/Y axes ouput data rate to high-perf mode
							  //MAG_OMXY_LOW_POWER,
							  //MAG_OMXY_MEDIUM_PERFORMANCE,
							  MAG_OMXY_HIGH_PERFORMANCE,
							  //MAG_OMXY_ULTRA_HIGH_PERFORMANCE,
							  //// Initialize magnetometer Z axis performance mode
							  //MAG_OMZ_LOW_PW,
							  //MAG_OMZ_MEDIUM_PERFORMANCE,
							  MAG_OMZ_HIGH_PERFORMANCE,
							  //MAG_OMZ_ULTRA_HIGH_PERFORMANCE,
							  //// Initialize magnetometer run mode. Also enables I2C (bit 7 = 0)
							  MAG_MD_CONTINUOUS,
							  //MAG_MD_SINGLE,
							  //MAG_MD_POWER_DOWN_1,
							  //MAG_MD_POWER_DOWN_2,
							  //// Initialize acceleration full scale to +/-2g
							  ACC_FS_2g,
							  //ACC_FS_4g,
							  //ACC_FS_8g,
							  //// Enable block data updating
							  ACC_BDU_ENABLE,
							  //ACC_BDU_DISABLE,
							  //// Enable X, Y, and Z accelerometer axes
							  //ACC_DISABLE_ALL,
							  //ACC_X_ENABLE,
							  //ACC_Y_ENABLE,
							  //ACC_Z_ENABLE,
							  //ACC_X_ENABLE|ACC_Y_ENABLE,
							  //ACC_X_ENABLE|ACC_Z_ENABLE,
							  //ACC_Y_ENABLE|ACC_Z_ENABLE,
							  ACC_X_ENABLE|ACC_Y_ENABLE|ACC_Z_ENABLE,
							  //// Initialize accelerometer output data rate to 100 Hz (turn on device)
							  //ACC_ODR_POWER_DOWN
							  //ACC_ODR_10_Hz
							  //ACC_ODR_50_Hz
							  ACC_ODR_100_Hz
							  //ACC_ODR_200_Hz
							  //ACC_ODR_400_Hz
							  //ACC_ODR_800_Hz
	          ) == IMU_SUCCESS) return true;
	else return false;

}

uint8_t  LSM303C_I2C_ByteWrite(I2C_ADDR_t slaveAddress, uint8_t reg,
    uint8_t data)
{
	uint8_t transmit_addr = slaveAddress << 1;
  uint8_t ret = IMU_GENERIC_ERROR;
  uint8_t temp_data[2] = {reg, data};

  if (HAL_I2C_Master_Transmit(i2c_handle, transmit_addr, &temp_data[0], 2, I2C_Timeout) == HAL_OK) ret = IMU_SUCCESS;
  else ret = IMU_HW_ERROR;

  return ret;
}

status_t LSM303C_I2C_ByteRead(I2C_ADDR_t slaveAddress, uint8_t reg,
    uint8_t *data)
{
	uint8_t temp_data = 0;
	uint8_t transmit_addr = slaveAddress << 1;
	uint8_t receive_addr = transmit_addr | 0x01;

  status_t ret = IMU_GENERIC_ERROR;
  //debug_print("Reading from I2C address: 0x");
  //debug_prints(slaveAddress, HEX);
  //debug_prints(", register 0x");
  //debug_printlns(reg, HEX);
 // Wire.beginTransmission(slaveAddress); // Initialize the Tx buffer

    if (HAL_I2C_Master_Transmit(i2c_handle, transmit_addr, &reg, 1, I2C_Timeout) != HAL_OK)  // Send Tx, send restart to keep alive
    {
      //debug_println("Error: I2C buffer didn't get sent!");
      //debug_print("Slave address: 0x");
      //debug_printlns(slaveAddress, HEX);
      //debug_print("Register: 0x");
      //debug_printlns(reg, HEX);
      ret = IMU_HW_ERROR;
    }
    else if (HAL_I2C_Master_Receive(i2c_handle, receive_addr, &temp_data, 1, I2C_Timeout) == HAL_OK)
    {
      *data = temp_data;
      //debug_print("Read: 0x");
      //debug_printlns(data, HEX);
      ret = IMU_SUCCESS;
    }
    else
    {
      //debug_println("IMU_HW_ERROR");
      ret = IMU_HW_ERROR;
    }


  return ret;
}


#endif


status_t LSM303C_MAG_ReadReg(MAG_REG_t reg, uint8_t *data)
{

  status_t ret = IMU_GENERIC_ERROR;

  if (interfaceMode == MODE_I2C)
  {
#ifdef I2C_INTERFACE
    ret = LSM303C_I2C_ByteRead(MAG_I2C_ADDR, reg, data);
#endif
  }
  else if (interfaceMode == MODE_SPI)
  {
#ifdef SPI_SOFTWARE
    *data = LSM303C_SPI_ReadByte(MAG, reg);
#endif

#ifdef SPI_HARDWARE
    *data = LSM303C_SPI_ReadByte_HW(MAG, reg);
#endif
    ret = IMU_SUCCESS;
  }
  else
  {
    ret = IMU_GENERIC_ERROR; // Shouldn't get here
  }

  return ret;
}

uint8_t  LSM303C_MAG_WriteReg(MAG_REG_t reg, uint8_t data)
{

  uint8_t ret;

  if (interfaceMode == MODE_I2C)
  {
#ifdef I2C_INTERFACE
    ret = LSM303C_I2C_ByteWrite(MAG_I2C_ADDR, reg, data);
#endif
  }
  else if (interfaceMode == MODE_SPI)
  {
#ifdef SPI_SOFTWARE
    ret = LSM303C_SPI_WriteByte(MAG, reg, data);
#endif

#ifdef SPI_HARDWARE
    ret = LSM303C_SPI_WriteByte_HW(MAG, reg, data);
#endif
  }
  else
  {
    ret = IMU_GENERIC_ERROR;
  }

  return ret;
}

status_t LSM303C_ACC_ReadReg(ACC_REG_t reg, uint8_t *data)
{

  status_t ret;

  if (interfaceMode == MODE_I2C)
  {
#ifdef I2C_INTERFACE
    ret = LSM303C_I2C_ByteRead(ACC_I2C_ADDR, reg, data);
#endif
  }
  else if (interfaceMode == MODE_SPI)
  {
#ifdef SPI_SOFTWARE
    *data = LSM303C_SPI_ReadByte(ACC, reg);
#endif

#ifdef SPI_HARDWARE
    *data = LSM303C_SPI_ReadByte_HW(ACC, reg);
#endif
    ret = IMU_SUCCESS;
  }
  else
  {
    ret = IMU_HW_ERROR;
  }

  return ret;
}





uint8_t  LSM303C_ACC_WriteReg(ACC_REG_t reg, uint8_t data)
{

  uint8_t ret;

  if (interfaceMode == MODE_I2C)
  {
#ifdef I2C_INTERFACE
    ret = LSM303C_I2C_ByteWrite(ACC_I2C_ADDR, reg, data);
#endif
  }
  else if (interfaceMode == MODE_SPI)
  {
#ifdef SPI_SOFTWARE
    ret = LSM303C_SPI_WriteByte(ACC, reg, data);
#endif

#ifdef SPI_HARDWARE
    ret = LSM303C_SPI_WriteByte_HW(ACC, reg, data);
#endif
  }
  else
  {
    ret = IMU_GENERIC_ERROR;
  }

  return ret;
}

float LSM303C_readTempC()
{
  uint8_t valueL = 0;
  uint8_t valueH = 0;
  float temperature;

  // Make sure temperature sensor is enabled
  if( LSM303C_MAG_TemperatureEN(MAG_TEMP_EN_ENABLE))
  {
    return false;
  }

	if( LSM303C_MAG_ReadReg(MAG_TEMP_OUT_L, &valueL) )
  {
    return false;
  }

  if( LSM303C_MAG_ReadReg(MAG_TEMP_OUT_H, &valueH) )
  {
    return false;
  }

  temperature = (float)( (valueH << 8) | valueL );
  temperature /= 8; // 8 digits/˚C
  temperature += 25;// Reads 0 @ 25˚C

  return temperature;
}

status_t LSM303C_MAG_TemperatureEN(MAG_TEMP_EN_t val){
  uint8_t value = 0;

  if( LSM303C_MAG_ReadReg(MAG_CTRL_REG1, &value) )
  {
    return IMU_HW_ERROR;
  }

  value &= ~MAG_TEMP_EN_ENABLE; //mask
  value |= val;

  if( LSM303C_MAG_WriteReg(MAG_CTRL_REG1, value) )
  {
    return IMU_HW_ERROR;
  }

  return IMU_SUCCESS;
}

status_t LSM303C_begin(InterfaceMode_t im, MAG_DO_t modr, MAG_FS_t mfs,
    MAG_BDU_t mbu, MAG_OMXY_t mxyodr, MAG_OMZ_t mzodr, MAG_MD_t mm,
    ACC_FS_t afs, ACC_BDU_t abu, uint8_t aea, ACC_ODR_t aodr)
{
  uint8_t successes = 0;
  // Select I2C or SPI
  interfaceMode = im;

  if (interfaceMode == MODE_SPI)
  {
    //debug_println("Setting up SPI");
    // Setup pins for SPI
    // CS & CLK must be outputs DDRxn = 1
    //bitSet(DIR_REG, CSBIT_MAG);
    //bitSet(DIR_REG, CSBIT_XL);
    //bitSet(DIR_REG, CLKBIT);
    // Deselect SPI chips
    //bitSet(CSPORT_MAG, CSBIT_MAG);
    //bitSet(CSPORT_XL, CSBIT_XL);
    // Clock polarity (CPOL) = 1
    //bitSet(CLKPORT, CLKBIT);
    // SPI Serial Interface Mode (SIM) bits must be set
#ifdef SPI_SOFTWARE
	LSM303C_SPI_WriteByte(ACC, ACC_CTRL4, 0b111);
	LSM303C_SPI_WriteByte(MAG, MAG_CTRL_REG3, _BV(2));
	who_is_mag = LSM303C_SPI_ReadByte(MAG, MAG_WHO_AM_I);
	who_is_acc = LSM303C_SPI_ReadByte(ACC, ACC_WHO_AM_I);
#endif
#ifdef SPI_HARDWARE
	LSM303C_SPI_WriteByte_HW(ACC, ACC_CTRL4, 0b111);
	LSM303C_SPI_WriteByte_HW(MAG, MAG_CTRL_REG3, _BV(2));
	who_is_mag = LSM303C_SPI_ReadByte_HW(MAG, MAG_WHO_AM_I);
	who_is_acc = LSM303C_SPI_ReadByte_HW(ACC, ACC_WHO_AM_I);
#endif
	//LSM303C_softReset();

	//ACCELERO_IO_Write(0x23, 0x5);
	//who_is_mag = ACCELERO_IO_Read(MAG_WHO_AM_I);

  }
  else
  {
    //I2C Mode
    //initialize I2C bus and clock stretch in the setup()
  }
  ////////// Initialize Magnetometer //////////
  // Initialize magnetometer output data rate
  successes += LSM303C_MAG_SetODR(modr);
  // Initialize magnetic field full scale
  successes += LSM303C_MAG_SetFullScale(mfs);
  // Enabling block data updating
  successes += LSM303C_MAG_BlockDataUpdate(mbu);
  // Initialize magnetometer X/Y axes ouput data rate
  successes += LSM303C_MAG_XY_AxOperativeMode(mxyodr);
  // Initialize magnetometer Z axis performance mode
  successes += LSM303C_MAG_Z_AxOperativeMode(mzodr);
  // Initialize magnetometer run mode.
  successes += LSM303C_MAG_SetMode(mm);

  ////////// Initialize Accelerometer //////////
  // Initialize acceleration full scale
  successes += LSM303C_ACC_SetFullScale(afs);
  // Enable block data updating
  successes += LSM303C_ACC_BlockDataUpdate(abu);
  // Enable X, Y, and Z accelerometer axes
  successes += LSM303C_ACC_EnableAxis(aea);
  // Initialize accelerometer output data rate
  successes += LSM303C_ACC_SetODR(aodr);

  return (successes == IMU_SUCCESS) ? IMU_SUCCESS : IMU_HW_ERROR;
}

// Methods required to get device up and running
status_t LSM303C_MAG_SetODR(MAG_DO_t val)
{
  //debug_print(EMPTY);
  uint8_t value;

  if(LSM303C_MAG_ReadReg(MAG_CTRL_REG1, &value))
  {
    //debug_printlns("Failed Read from MAG_CTRL_REG1");
    return IMU_HW_ERROR;
  }

  // Mask and only change DO0 bits (4:2) of MAG_CTRL_REG1
  value &= ~MAG_DO_80_Hz;
  value |= val;

  if(LSM303C_MAG_WriteReg(MAG_CTRL_REG1, value))
  {
    return IMU_HW_ERROR;
  }

  return IMU_SUCCESS;
}

status_t LSM303C_MAG_SetFullScale(MAG_FS_t val)
{
  //debug_print(EMPTY);
  uint8_t value;

  if ( LSM303C_MAG_ReadReg(MAG_CTRL_REG2, &value) )
  {
    return IMU_HW_ERROR;
  }

  value &= ~MAG_FS_16_Ga; //mask
  value |= val;

  if ( LSM303C_MAG_WriteReg(MAG_CTRL_REG2, value) )
  {
    return IMU_HW_ERROR;
  }

  return IMU_SUCCESS;
}

status_t LSM303C_MAG_BlockDataUpdate(MAG_BDU_t val)
{
  //debug_print(EMPTY);
  uint8_t value;

  if ( LSM303C_MAG_ReadReg(MAG_CTRL_REG5, &value) )
  {
    return IMU_HW_ERROR;
  }


  value &= ~MAG_BDU_ENABLE; //mask
  value |= val;

  if ( LSM303C_MAG_WriteReg(MAG_CTRL_REG5, value) )
  {
    return IMU_HW_ERROR;
  }

  return IMU_SUCCESS;
}

status_t LSM303C_MAG_XYZ_AxDataAvailable(uint8_t *value)
{
	uint8_t val = 0, tmp1 = 0;
  if ( LSM303C_MAG_ReadReg(MAG_STATUS_REG, &val) )
  {
    return IMU_HW_ERROR;
  }
  tmp1 = val & 0x08;
  *value = tmp1;

  return IMU_SUCCESS;
}

status_t LSM303C_MAG_XY_AxOperativeMode(MAG_OMXY_t val)
{
  //debug_print(EMPTY);

  uint8_t value;

  if ( LSM303C_MAG_ReadReg(MAG_CTRL_REG1, &value) )
  {
    return IMU_HW_ERROR;
  }

  value &= ~MAG_OMXY_ULTRA_HIGH_PERFORMANCE; //mask
  value |= val;

  if ( LSM303C_MAG_WriteReg(MAG_CTRL_REG1, value) )
  {
    return IMU_HW_ERROR;
  }

  return IMU_SUCCESS;
}

status_t LSM303C_MAG_Z_AxOperativeMode(MAG_OMZ_t val)
{
  //debug_print(EMPTY);
  uint8_t value;

  if ( LSM303C_MAG_ReadReg(MAG_CTRL_REG4, &value) )
  {
    return IMU_HW_ERROR;
  }

  value &= ~MAG_OMZ_ULTRA_HIGH_PERFORMANCE; //mask
  value |= val;

  if ( LSM303C_MAG_WriteReg(MAG_CTRL_REG4, value) )
  {
    return IMU_HW_ERROR;
  }

  return IMU_SUCCESS;
}

status_t LSM303C_MAG_SetMode(MAG_MD_t val)
{
  //debug_print(EMPTY);
  uint8_t value = 0;

  if ( LSM303C_MAG_ReadReg(MAG_CTRL_REG3, &value) )
  {
    //debug_print("Failed to read MAG_CTRL_REG3. 'Read': 0x");
    //debug_printlns(value, HEX);
    return IMU_HW_ERROR;
  }

  value &= ~MAG_MD_POWER_DOWN_2;
  value |= val;

  if ( LSM303C_MAG_WriteReg(MAG_CTRL_REG3, value) )
  {
    return IMU_HW_ERROR;
  }

  return IMU_SUCCESS;
}

status_t LSM303C_ACC_SetFullScale(ACC_FS_t val)
{
  //debug_print(EMPTY);
  uint8_t value = 0;

  if ( LSM303C_ACC_ReadReg(ACC_CTRL4, &value) )
  {
    //debug_printlns("Failed ACC read");
    return IMU_HW_ERROR;
  }

  value &= ~ACC_FS_8g;
  value |= val;


  if ( LSM303C_ACC_WriteReg(ACC_CTRL4, value) )
  {
    return IMU_HW_ERROR;
  }

  return IMU_SUCCESS;
}

status_t LSM303C_ACC_BlockDataUpdate(ACC_BDU_t val)
{
  //debug_print(EMPTY);
  uint8_t value = 0;

  if ( LSM303C_ACC_ReadReg(ACC_CTRL1, &value) )
  {
    return IMU_HW_ERROR;
  }

  value &= ~ACC_BDU_ENABLE;
  value |= val;

  if ( LSM303C_ACC_WriteReg(ACC_CTRL1, value) )
  {
    return IMU_HW_ERROR;
  }

  return IMU_SUCCESS;
}

status_t LSM303C_ACC_EnableAxis(uint8_t val)
{
  //debug_print(EMPTY);
  uint8_t value = 0;

  if ( LSM303C_ACC_ReadReg(ACC_CTRL1, &value) )
  {
    //debug_println(AERROR);
    return IMU_HW_ERROR;
  }

  value &= ~0x07;
  value |= val;

  if ( LSM303C_ACC_WriteReg(ACC_CTRL1, value) )
  {
    return IMU_HW_ERROR;
  }

  return IMU_SUCCESS;
}

status_t LSM303C_ACC_SetODR(ACC_ODR_t val)
{
  //debug_print(EMPTY);
  uint8_t value = 0;

  if ( LSM303C_ACC_ReadReg(ACC_CTRL1, &value) )
  {
    return IMU_HW_ERROR;
  }

  value &= ~ACC_ODR_MASK;
  value |= val;

  if ( LSM303C_ACC_WriteReg(ACC_CTRL1, value) )
  {
    return IMU_HW_ERROR;
  }

  return IMU_SUCCESS;
}

void  LSM303C_softReset()
{
	LSM303C_MAG_WriteReg(MAG_CTRL_REG5, 0x40);
	LSM303C_ACC_WriteReg(ACC_CTRL5, 0x40);
}

float LSM303C_readMagX()
{
  return LSM303C_readMag(xAxis);
}

float LSM303C_readMagY()
{
  return LSM303C_readMag(yAxis);
}

float LSM303C_readMagZ()
{
  return LSM303C_readMag(zAxis);
}


float LSM303C_readMag(AXIS_t dir)
{
	MAG_XYZDA_t flag_MAG_XYZDA = 0;
  status_t response = LSM303C_MAG_XYZ_AxDataAvailable(&flag_MAG_XYZDA);

  if (response != IMU_SUCCESS)
  {
    //debug_println(MERROR);
    return false;
  }

  // Check for new data in the status flags with a mask
  if (flag_MAG_XYZDA & MAG_XYZDA_YES)
  {
    response = LSM303C_MAG_GetMagRaw(&magData);
    //debug_println("Fresh raw data");
  }
  //convert from LSB to Gauss
  switch (dir)
  {
  case xAxis:
    return magData.xAxis * SENSITIVITY_MAG;
    break;
  case yAxis:
    return magData.yAxis * SENSITIVITY_MAG;
    break;
  case zAxis:
    return magData.zAxis * SENSITIVITY_MAG;
    break;
  default:
    return false;
  }

  // Should never get here
  //debug_println("Returning NAN");
  return false;
}

status_t LSM303C_MAG_GetMagRaw(AxesRaw_t *buff)
{
  //debug_print(EMPTY);
  uint8_t valueL = 0;
  uint8_t valueH = 0;

  //debug_println("& was false");
  if( LSM303C_MAG_ReadReg(MAG_OUTX_L, &valueL) )
  {
    return IMU_HW_ERROR;
  }

  if( LSM303C_MAG_ReadReg(MAG_OUTX_H, &valueH) )
  {
    return IMU_HW_ERROR;
  }

  buff->xAxis = (int16_t)( (valueH << 8) | valueL );

  if( LSM303C_MAG_ReadReg(MAG_OUTY_L, &valueL) )
  {
    return IMU_HW_ERROR;
  }

  if( LSM303C_MAG_ReadReg(MAG_OUTY_H, &valueH) )
  {
    return IMU_HW_ERROR;
  }

  buff->yAxis = (int16_t)( (valueH << 8) | valueL );

  if( LSM303C_MAG_ReadReg(MAG_OUTZ_L, &valueL) )
  {
    return IMU_HW_ERROR;
  }

  if( LSM303C_MAG_ReadReg(MAG_OUTZ_H, &valueH) )
  {
    return IMU_HW_ERROR;
  }

  buff->zAxis = (int16_t)( (valueH << 8) | valueL );

  return IMU_SUCCESS;
}

status_t LSM303C_ACC_Status_Flags(uint8_t *val)
{
  //debug_println("Getting accel status");
  if( LSM303C_ACC_ReadReg(ACC_STATUS, val) )
  {
    //debug_println(AERROR);
    return IMU_HW_ERROR;
  }

  return IMU_SUCCESS;
}

status_t LSM303C_ACC_GetAccRaw(AxesRaw_t *buff)
{
  uint8_t valueL;
  uint8_t valueH;

  if ( LSM303C_ACC_ReadReg(ACC_OUT_X_H, &valueH) )
  {
	  return IMU_HW_ERROR;
  }

  if ( LSM303C_ACC_ReadReg(ACC_OUT_X_L, &valueL) )
  {
	  return IMU_HW_ERROR;
  }

  buff->xAxis = (int16_t)( (valueH << 8) | valueL );

  if ( LSM303C_ACC_ReadReg(ACC_OUT_Y_H, &valueH) )
  {
	  return IMU_HW_ERROR;
  }

  if ( LSM303C_ACC_ReadReg(ACC_OUT_Y_L, &valueL) )
  {
	  return IMU_HW_ERROR;
  }

  buff->yAxis = (int16_t)( (valueH << 8) | valueL );

  if ( LSM303C_ACC_ReadReg(ACC_OUT_Z_H, &valueH) )
  {
	  return IMU_HW_ERROR;
  }

  if ( LSM303C_ACC_ReadReg(ACC_OUT_Z_L, &valueL) )
  {
	  return IMU_HW_ERROR;
  }

  buff->zAxis = (int16_t)( (valueH << 8) | valueL );

  return IMU_SUCCESS;
}

float LSM303C_readAccel(AXIS_t dir)
{
  uint8_t flag_ACC_STATUS_FLAGS;
  status_t response = LSM303C_ACC_Status_Flags(&flag_ACC_STATUS_FLAGS);

  if (response != IMU_SUCCESS)
  {
    //debug_println(AERROR);
    return false;
  }

  // Check for new data in the status flags with a mask
  // If there isn't new data use the last data read.
  // There are valid cases for this, like reading faster than refresh rate.
  if (flag_ACC_STATUS_FLAGS & ACC_ZYX_NEW_DATA_AVAILABLE)
  {
    response = LSM303C_ACC_GetAccRaw(&accelData);
    //debug_println("Fresh raw data");
  }
  //convert from LSB to mg
  switch (dir)
  {
  case xAxis:
    return accelData.xAxis * SENSITIVITY_ACC;
    break;
  case yAxis:
    return accelData.yAxis * SENSITIVITY_ACC;
    break;
  case zAxis:
    return accelData.zAxis * SENSITIVITY_ACC;
    break;
  default:
    return false;
  }
  // Should never get here
  //debug_println("Returning NAN");
  return false;
}


float LSM303C_readAccelX()
{
  uint8_t flag_ACC_STATUS_FLAGS;
  status_t response = LSM303C_ACC_Status_Flags(&flag_ACC_STATUS_FLAGS);

  if (response != IMU_SUCCESS)
  {
    //debug_println(AERROR);
    return false;
  }

  // Check for new data in the status flags with a mask
  // If there isn't new data use the last data read.
  // There are valid cases for this, like reading faster than refresh rate.
  if (flag_ACC_STATUS_FLAGS & ACC_X_NEW_DATA_AVAILABLE)
  {
    uint8_t valueL;
    uint8_t valueH;

    if ( LSM303C_ACC_ReadReg(ACC_OUT_X_H, &valueH) )
    {
	    return IMU_HW_ERROR;
    }

    if ( LSM303C_ACC_ReadReg(ACC_OUT_X_L, &valueL) )
    {
	    return IMU_HW_ERROR;
    }

    //debug_println("Fresh raw data");

    //convert from LSB to mg
    return (int16_t)(( (valueH << 8) | valueL )) * SENSITIVITY_ACC;
  }

  // Should never get here
  //debug_println("Returning NAN");
  return false;
}

float LSM303C_readAccelY()
{
  uint8_t flag_ACC_STATUS_FLAGS;
  status_t response = LSM303C_ACC_Status_Flags(&flag_ACC_STATUS_FLAGS);

  if (response != IMU_SUCCESS)
  {
    //debug_println(AERROR);
    return false;
  }

  // Check for new data in the status flags with a mask
  // If there isn't new data use the last data read.
  // There are valid cases for this, like reading faster than refresh rate.
  if (flag_ACC_STATUS_FLAGS & ACC_Y_NEW_DATA_AVAILABLE)
  {
    uint8_t valueL;
    uint8_t valueH;

    if ( LSM303C_ACC_ReadReg(ACC_OUT_Y_H, &valueH) )
    {
	    return IMU_HW_ERROR;
    }

    if ( LSM303C_ACC_ReadReg(ACC_OUT_Y_L, &valueL) )
    {
	    return IMU_HW_ERROR;
    }

    //debug_println("Fresh raw data");

    //convert from LSB to mg
    return (int16_t)(( (valueH << 8) | valueL )) * SENSITIVITY_ACC;
  }

  // Should never get here
  //debug_println("Returning NAN");
  return false;
}

float LSM303C_readAccelZ()
{
  uint8_t flag_ACC_STATUS_FLAGS;
  status_t response = LSM303C_ACC_Status_Flags(&flag_ACC_STATUS_FLAGS);

  if (response != IMU_SUCCESS)
  {
    //debug_println(AERROR);
    return false;
  }

  // Check for new data in the status flags with a mask
  // If there isn't new data use the last data read.
  // There are valid cases for this, like reading faster than refresh rate.
  if (flag_ACC_STATUS_FLAGS & ACC_Z_NEW_DATA_AVAILABLE)
  {
    uint8_t valueL;
    uint8_t valueH;

    if ( LSM303C_ACC_ReadReg(ACC_OUT_Z_H, &valueH) )
    {
	    return IMU_HW_ERROR;
    }

    if ( LSM303C_ACC_ReadReg(ACC_OUT_Z_L, &valueL) )
    {
	    return IMU_HW_ERROR;
    }

    //debug_println("Fresh raw data");

    //convert from LSB to mg
    return (int16_t)(( (valueH << 8) | valueL )) * SENSITIVITY_ACC;
  }

  // Should never get here
  //debug_println("Returning NAN");
  return false;
}

/**
	GET AZIMUTH
	Calculate the azimuth (in degrees);

	@since v0.1;
	@return int azimuth
**/
uint16_t LSM303C_getAzimuth(){
	int a = atan2( LSM303C_readMagY(), LSM303C_readMagX() ) * 180.0 / M_PI;
	return a < 0 ? 360 + a : a;
}


/**
	GET BEARING
	Divide the 360 degree circle into 16 equal parts and then return the a value of 0-15
	based on where the azimuth is currently pointing.

	@since v1.0.1 - function now requires azimuth parameter.
	@since v0.2.0 - initial creation

	@return byte direction of bearing
*/
uint8_t LSM303C_getBearing(int azimuth){
	unsigned long a = azimuth / 22.5;
	unsigned long r = a - (int)a;
	uint8_t sexdec = 0;
	sexdec = ( r >= .5 ) ? ceil(a) : floor(a);
	return sexdec;
}


/**
	This will take the location of the azimuth as calculated in getBearing() and then
	produce an array of chars as a text representation of the direction.

	NOTE: This function does not return anything since it is not possible to return an array.
	Values must be passed by reference back to your sketch.

	Example:

	( if direction is in 1 / NNE)

	char myArray[3];
	compass.getDirection(myArray, azimuth);

	Serial.print(myArray[0]); // N
	Serial.print(myArray[1]); // N
	Serial.print(myArray[2]); // E


	@see getBearing();

	@since v1.0.1 - function now requires azimuth parameter.
	@since v0.2.0 - initial creation
*/
void LSM303C_getDirection(char* myArray, int azimuth){
	int d = LSM303C_getBearing(azimuth);
	myArray[0] = _bearings[d][0];
	myArray[1] = _bearings[d][1];
	myArray[2] = _bearings[d][2];
}



