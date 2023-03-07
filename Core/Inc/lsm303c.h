/*
 * lsm303c.h
 *
 *  Created on: May 17, 2022
 *      Author: Shalimov
 */

#ifndef INC_LSM303C_H_
#define INC_LSM303C_H_

#include "stdbool.h"
#include "stdint.h"
#include "math.h"
#include "stm32wbxx_hal.h"
//#include "stm32f4xx_hal.h"

#define SPI_SOFTWARE
//#define SPI_HARDWARE
//#define I2C_INTERFACE

#define _BV(n) (1 << n)

#define CSPORT_MAG	GPIOA
#define CSBIT_MAG	GPIO_PIN_6
#define CSPORT_XL	GPIOA
#define CSBIT_XL	GPIO_PIN_7
#define CLKPORT		GPIOA
#define CLKBIT		GPIO_PIN_1
#define DATAPORTO	GPIOA
#define DATABIT		GPIO_PIN_5


#define SENSITIVITY_ACC   0.06103515625   // LSB/mg
#define SENSITIVITY_MAG   0.00048828125   // LSB/Ga

#define SPI_TIMEOUT 10
#define I2C_Timeout 10

// Return values
typedef enum
{
  IMU_SUCCESS,
  IMU_HW_ERROR,
  IMU_NOT_SUPPORTED,
  IMU_GENERIC_ERROR,
  IMU_OUT_OF_BOUNDS,
  //...
} status_t;

typedef enum
{
  MAG,
  ACC
} CHIP_t;

////////////////////////////////////
// LSM303C Magnetometer Registers //
////////////////////////////////////
typedef enum
{
  MAG_WHO_AM_I   = 0x0F,
  MAG_CTRL_REG1  = 0x20,
  MAG_CTRL_REG2  = 0x21,
  MAG_CTRL_REG3  = 0x22,
  MAG_CTRL_REG4  = 0x23,
  MAG_CTRL_REG5  = 0x24,
  MAG_STATUS_REG = 0x27,
  MAG_OUTX_L     = 0x28,
  MAG_OUTX_H     = 0x29,
  MAG_OUTY_L     = 0x2A,
  MAG_OUTY_H     = 0x2B,
  MAG_OUTZ_L     = 0x2C,
  MAG_OUTZ_H     = 0x2D,
  MAG_TEMP_OUT_L = 0x2E,
  MAG_TEMP_OUT_H = 0x2F,
  MAG_INT_CFG    = 0x30,
  MAG_INT_SRC    = 0x31,
  MAG_INT_THS_L  = 0x32,
  MAG_INT_THS_H  = 0x33
} MAG_REG_t;

/////////////////////////////////////
// LSM303C Accelerometer Registers //
/////////////////////////////////////
typedef enum
{
  ACC_TEMP_L       = 0x0B,
  ACC_TEMP_H       = 0x0C,
  ACC_ACT_TSH      = 0x1E,
  ACC_ACT_DUR      = 0x1F,
  ACC_WHO_AM_I     = 0x0F,
  ACC_CTRL1        = 0x20,
  ACC_CTRL2        = 0x21,
  ACC_CTRL3        = 0x22,
  ACC_CTRL4        = 0x23,
  ACC_CTRL5        = 0x24,
  ACC_CTRL6        = 0x25,
  ACC_CTRL7        = 0x26,
  ACC_STATUS       = 0x27,
  ACC_OUT_X_L      = 0x28,
  ACC_OUT_X_H      = 0x29,
  ACC_OUT_Y_L      = 0x2A,
  ACC_OUT_Y_H      = 0x2B,
  ACC_OUT_Z_L      = 0x2C,
  ACC_OUT_Z_H      = 0x2D,
  ACC_FIFO_CTRL    = 0x2E,
  ACC_FIFO_SRC     = 0x2F,
  ACC_IG_CFG1      = 0x30,
  ACC_IG_SRC1      = 0x31,
  ACC_IG_THS_X1    = 0x32,
  ACC_IG_THS_Y1    = 0x33,
  ACC_IG_THS_Z1    = 0x34,
  ACC_IG_DUR1      = 0x35,
  ACC_IG_CFG2      = 0x36,
  ACC_IG_SRC2      = 0x37,
  ACC_IG_THS2      = 0x38,
  ACC_IG_DUR2      = 0x39,
  ACC_XL_REFERENCE = 0x3A,
  ACC_XH_REFERENCE = 0x3B,
  ACC_YL_REFERENCE = 0x3C,
  ACC_YH_REFERENCE = 0x3D,
  ACC_ZL_REFERENCE = 0x3E,
  ACC_ZH_REFERENCE = 0x3F
} ACC_REG_t;

typedef enum
{
  MODE_SPI,
  MODE_I2C,
} InterfaceMode_t;

typedef enum
{
  ACC_I2C_ADDR = 0x1D,
  MAG_I2C_ADDR = 0x1E
} I2C_ADDR_t;

typedef enum
{
  MAG_TEMP_EN_DISABLE = 0x00,
  MAG_TEMP_EN_ENABLE  = 0x80
} MAG_TEMP_EN_t;

typedef enum
{
  MAG_DO_0_625_Hz = 0x00,
  MAG_DO_1_25_Hz  = 0x04,
  MAG_DO_2_5_Hz   = 0x08,
  MAG_DO_5_Hz     = 0x0C,
  MAG_DO_10_Hz    = 0x10,
  MAG_DO_20_Hz    = 0x14,
  MAG_DO_40_Hz    = 0x18,
  MAG_DO_80_Hz    = 0x1C
} MAG_DO_t;

typedef enum
{
  MAG_FS_4_Ga   =  0x00,
  MAG_FS_8_Ga   =  0x20,
  MAG_FS_12_Ga  =  0x40,
  MAG_FS_16_Ga  =  0x60
} MAG_FS_t;

typedef enum
{
  MAG_BDU_DISABLE = 0x00,
  MAG_BDU_ENABLE  = 0x40
} MAG_BDU_t;

typedef enum
{
  MAG_OMXY_LOW_POWER              = 0x00,
  MAG_OMXY_MEDIUM_PERFORMANCE     = 0x20,
  MAG_OMXY_HIGH_PERFORMANCE       = 0x40,
  MAG_OMXY_ULTRA_HIGH_PERFORMANCE = 0x60
} MAG_OMXY_t;

typedef enum
{
  MAG_OMZ_LOW_PW                  =  0x00,
  MAG_OMZ_MEDIUM_PERFORMANCE      =  0x04,
  MAG_OMZ_HIGH_PERFORMANCE        =  0x08,
  MAG_OMZ_ULTRA_HIGH_PERFORMANCE  =  0x0C
} MAG_OMZ_t;

typedef enum
{
  MAG_MD_CONTINUOUS   = 0x00,
  MAG_MD_SINGLE       = 0x01,
  MAG_MD_POWER_DOWN_1 = 0x02,
  MAG_MD_POWER_DOWN_2 = 0x03
} MAG_MD_t;

typedef enum
{
  ACC_FS_2g = 0x00,
  ACC_FS_4g = 0x20,
  ACC_FS_8g = 0x30
} ACC_FS_t;

typedef enum
{
  ACC_BDU_DISABLE = 0x00,
  ACC_BDU_ENABLE  = 0x08
} ACC_BDU_t;

typedef enum
{
  ACC_ODR_POWER_DOWN  = 0x00,
  ACC_ODR_10_Hz       = 0x10,
  ACC_ODR_50_Hz       = 0x20,
  ACC_ODR_100_Hz      = 0x30,
  ACC_ODR_200_Hz      = 0x40,
  ACC_ODR_400_Hz      = 0x50,
  ACC_ODR_800_Hz      = 0x60,
  ACC_ODR_MASK        = 0x60
} ACC_ODR_t;

typedef enum
{
  ACC_DISABLE_ALL = 0x00,
  ACC_X_ENABLE    = 0x01,
  ACC_Y_ENABLE    = 0x02,
  ACC_Z_ENABLE    = 0x04
} ACC_AXIS_EN_t;

typedef enum
{
  ACC_X_NEW_DATA_AVAILABLE    = 0x01,
  ACC_Y_NEW_DATA_AVAILABLE    = 0x02,
  ACC_Z_NEW_DATA_AVAILABLE    = 0x04,
  ACC_ZYX_NEW_DATA_AVAILABLE  = 0x08,
  ACC_X_OVERRUN               = 0x10,
  ACC_Y_OVERRUN               = 0x20,
  ACC_Z_OVERRUN               = 0x40,
  ACC_ZYX_OVERRUN             = 0x80
} ACC_STATUS_FLAGS_t;

typedef enum
{
  MAG_XYZDA_NO  = 0x00,
  MAG_XYZDA_YES = 0x08
} MAG_XYZDA_t;

typedef struct
{
  int16_t xAxis;
  int16_t yAxis;
  int16_t zAxis;
} AxesRaw_t;

typedef enum
{
  xAxis,
  yAxis,
  zAxis
} AXIS_t;



status_t LSM303C_MAG_ReadReg(MAG_REG_t reg, uint8_t *data);
uint8_t  LSM303C_MAG_WriteReg(MAG_REG_t reg, uint8_t data);
status_t LSM303C_ACC_ReadReg(ACC_REG_t reg, uint8_t *data);
uint8_t  LSM303C_ACC_WriteReg(ACC_REG_t reg, uint8_t data);
float LSM303C_readTempC();
status_t LSM303C_MAG_TemperatureEN(MAG_TEMP_EN_t val);
status_t LSM303C_begin(InterfaceMode_t im, MAG_DO_t modr, MAG_FS_t mfs,
    MAG_BDU_t mbu, MAG_OMXY_t mxyodr, MAG_OMZ_t mzodr, MAG_MD_t mm,
    ACC_FS_t afs, ACC_BDU_t abu, uint8_t aea, ACC_ODR_t aodr);
status_t LSM303C_ACC_SetODR(ACC_ODR_t val);
status_t LSM303C_ACC_EnableAxis(uint8_t val);
status_t LSM303C_ACC_BlockDataUpdate(ACC_BDU_t val);
status_t LSM303C_ACC_SetFullScale(ACC_FS_t val);
status_t LSM303C_MAG_SetMode(MAG_MD_t val);
status_t LSM303C_MAG_Z_AxOperativeMode(MAG_OMZ_t val);
status_t LSM303C_MAG_XY_AxOperativeMode(MAG_OMXY_t val);
status_t LSM303C_MAG_XYZ_AxDataAvailable(uint8_t *value);
status_t LSM303C_MAG_BlockDataUpdate(MAG_BDU_t val);
status_t LSM303C_MAG_SetFullScale(MAG_FS_t val);
status_t LSM303C_MAG_SetODR(MAG_DO_t val);
float LSM303C_readMag(AXIS_t dir);
status_t LSM303C_MAG_GetMagRaw(AxesRaw_t *buff);
float LSM303C_readAccel(AXIS_t dir);
float LSM303C_readMagX();
float LSM303C_readMagY();
float LSM303C_readMagZ();
float LSM303C_readAccelX();
float LSM303C_readAccelY();
float LSM303C_readAccelZ();
void  LSM303C_softReset();
uint16_t LSM303C_getAzimuth();
uint8_t LSM303C_getBearing(int azimuth);
void LSM303C_getDirection(char* myArray, int azimuth);

#ifdef SPI_SOFTWARE
bool LSM303C_init();
status_t LSM303C_SPI_WriteByte(CHIP_t chip, uint8_t reg, uint8_t data);
uint8_t LSM303C_SPI_ReadByte(CHIP_t chip, uint8_t data);
#endif

#ifdef SPI_HARDWARE
bool LSM303C_init(SPI_HandleTypeDef *h_spi);
status_t LSM303C_SPI_WriteByte_HW(CHIP_t chip, uint8_t reg, uint8_t data);
uint8_t LSM303C_SPI_ReadByte_HW(CHIP_t chip, uint8_t data);
#endif

#ifdef I2C_INTERFACE
bool LSM303C_init(I2C_HandleTypeDef *h_i2c);
uint8_t  LSM303C_I2C_ByteWrite(I2C_ADDR_t slaveAddress, uint8_t reg, uint8_t data);
status_t LSM303C_I2C_ByteRead(I2C_ADDR_t slaveAddress, uint8_t reg, uint8_t *data);
#endif


#endif /* INC_LSM303C_H_ */
