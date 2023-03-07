/*******************************************************************************

 Bare Conductive MPR121 library
 ------------------------------

 MPR121.cpp - MPR121 class implementation file

 Based on code by Jim Lindblom and plenty of inspiration from the Freescale
 Semiconductor datasheets and application notes.

 Bare Conductive code written by Stefan Dzisiewski-Smith, Peter Krige
 and Szymon Kaliski.

 This work is licensed under a MIT license https://opensource.org/licenses/MIT

 Copyright (c) 2016, Bare Conductive

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.

*******************************************************************************/


#include <stdlib.h>
#include <string.h>
#include <inttypes.h>


#include "MPR121.h"
#include "cmsis_os.h"

#define NOT_INITED_BIT 			0
#define ADDRESS_UNKNOWN_BIT 	1
#define READBACK_FAIL_BIT 		2
#define OVERCURRENT_FLAG_BIT 	3
#define OUT_OF_RANGE_BIT 		4
#define I2C_TIMEOUT				2

uint8_t address = 0x5A;
MPR121_settings_type defaultSettings;
uint8_t ECR_backup = 0x00; // so we can re-enable the correct number of electrodes
                  // when recovering from stop mode
uint8_t error = 1<<NOT_INITED_BIT;
bool running = false;
uint8_t interruptPin;
bool write_fail = false;

int16_t filteredData[13];
int16_t baselineData[13];
uint16_t touchData = 0;
uint16_t lastTouchData = 0;

bool autoTouchStatusFlag = false; // we use this to catch touch / release events that happen
                                        // during other update calls

I2C_HandleTypeDef *i2c_handle;

//extern MPR121_type MPR121;


void MPR121_setRegister(uint8_t reg, uint8_t value){

  bool wasRunning = false;
  uint8_t temp_reg = 0;

  if(reg==MPR121_ECR){  // if we are modding MPR121_ECR, update our internal running status
    if(value&0x3F){
      running = true;
    } else {
      running = false;
    }
  } else if(reg<MPR121_CTL0){
    wasRunning = running;
    if(wasRunning) MPR121_stop();  // we should ALWAYS be in stop mode for this
                // unless modding MPR121_ECR or GPIO / LED register
  }

  taskENTER_CRITICAL();
    if(HAL_I2C_Mem_Write(i2c_handle, address << 1, reg, 1, &value, 1, I2C_TIMEOUT) != HAL_OK){
      error |= 1<<ADDRESS_UNKNOWN_BIT; // set address unknown bit
    } else {
      error &= ~(1<<ADDRESS_UNKNOWN_BIT);
      if (HAL_I2C_Mem_Read(i2c_handle, address << 1, reg, 1, &temp_reg, 1, I2C_TIMEOUT) == HAL_OK) {
    	  if (value == temp_reg) write_fail = false;
    	  else {
    		  HAL_I2C_Mem_Read(i2c_handle, address << 1, MPR121_TS2, 1, &temp_reg, 1, I2C_TIMEOUT);
    		  if((temp_reg&0x80)!=0){
    		        error |= 1<<OVERCURRENT_FLAG_BIT;
    		  }
    		  write_fail = true;
    	  }
      }
    }
    taskEXIT_CRITICAL();
    if(wasRunning) MPR121_run();   // restore run mode if necessary
}

uint8_t MPR121_getRegister(uint8_t reg){
	uint8_t scratch = 0;

	taskENTER_CRITICAL();
    if(HAL_I2C_Mem_Read(i2c_handle, address << 1, reg, 1, &scratch, 1, I2C_TIMEOUT) == HAL_OK){  // just a single byte
      error &= ~(1<<ADDRESS_UNKNOWN_BIT); // all good, clear the bit
    } else {
      error |= 1<<ADDRESS_UNKNOWN_BIT; //set the bit - something went wrong
    }
    taskEXIT_CRITICAL();
    // auto update errors for registers with error data
    if(reg == MPR121_TS2 && ((scratch&0x80)!=0)){
      error |= 1<<OVERCURRENT_FLAG_BIT;
    } else {
      error &= ~(1<<OVERCURRENT_FLAG_BIT);
    }
    if((reg == MPR121_OORS1 || reg == MPR121_OORS2) && (scratch!=0)){
      error |= 1<<OUT_OF_RANGE_BIT;
    } else {
      error &= ~(1<<OUT_OF_RANGE_BIT);
    }
    return scratch;
}

bool MPR121_begin(I2C_HandleTypeDef *h_i2c, uint8_t touchThreshold, uint8_t releaseThreshold){

  // SDA and SCL should idle high, but MPR121 can get stuck waiting to complete a transaction
  // this code detects this state and releases us from it
	i2c_handle = h_i2c;

  // addresses only valid 0x5A to 0x5D - if we don't change the address it stays at default
  if(address>=0x5A && address<=0x5D)
  {
    address = address; // need to be specific here
  }

  error &= ~(1<<NOT_INITED_BIT); // clear NOT_INITED error flag

  if( MPR121_reset() ){
    // default values...
	  // default values in initialisation list
	  	defaultSettings.TTHRESH = 40;
	  	defaultSettings.RTHRESH = 20;
	  	defaultSettings.INTERRUPT = 4;   // note that this is not a hardware interrupt, just the digital
	                    // pin that the MPR121 ~INT pin is connected to
	  	defaultSettings.MHDR = 		0x01;
	  	defaultSettings.NHDR = 		0x01;
	  	defaultSettings.NCLR = 		0x10;
	  	defaultSettings.FDLR = 		0x20;
	  	defaultSettings.MHDF = 		0x01;
	  	defaultSettings.NHDF = 		0x01;
	  	defaultSettings.NCLF =		0x10;
	  	defaultSettings.FDLF = 		0x20;
	  	defaultSettings.NHDT = 		0x01;
	  	defaultSettings.NCLT = 		0x10;
	  	defaultSettings.FDLT = 		0xFF;
	  	defaultSettings.MHDPROXR = 	0x0F;
	  	defaultSettings.NHDPROXR = 	0x0F;
	  	defaultSettings.NCLPROXR = 	0x00;
	  	defaultSettings.FDLPROXR = 	0x00;
	  	defaultSettings.MHDPROXF = 	0x01;
	  	defaultSettings.NHDPROXF = 	0x01;
	  	defaultSettings.NCLPROXF = 	0xFF;
	  	defaultSettings.FDLPROXF = 	0xFF;
	  	defaultSettings.NHDPROXT = 	0x00;
	  	defaultSettings.NCLPROXT = 	0x00;
	  	defaultSettings.FDLPROXT = 	0x00;
	  	defaultSettings.DTR = 		0x11;
	  	defaultSettings.AFE1 = 		0xFF;
	  	defaultSettings.AFE2 = 		0x30;
	  	defaultSettings.ECR = 		0xCC; // default to fast baseline startup and 12 electrodes enabled, no prox
	  	defaultSettings.ACCR0 = 	0x00;
	  	defaultSettings.ACCR1 = 	0x00;
	  	defaultSettings.USL =		0x00;
	  	defaultSettings.LSL = 		0x00;
	  	defaultSettings.TL = 		0x00;

	  MPR121_applySettings( &defaultSettings );

    // only apply thresholds if they differ from existing defaults
    if( touchThreshold != defaultSettings.TTHRESH ){
    	MPR121_setTouchThreshold_all( touchThreshold );
    }

    if( releaseThreshold != defaultSettings.RTHRESH ){
    	MPR121_setReleaseThreshold_all( releaseThreshold );
    }

    return true;

  } else {
    return false;
  }
}

void MPR121_clearSavedThresholds() {
  #ifdef ARDUINO_ARCH_AVR
    uint8_t maxElectrodes = 12;
    int len = E2END;

    for(uint8_t i=0; i<13; i++){
      EEPROM.write(len - (i + 1), 255);
      EEPROM.write(len - (i + 1) - maxElectrodes, 255);
    }
  #endif
}

void MPR121_restoreSavedThresholds() {
  #ifdef ARDUINO_ARCH_AVR
    uint8_t maxElectrodes = 12;
    int len = E2END;

    for(uint8_t i=0; i<13; i++){
      uint8_t releaseThreshold = EEPROM.read(len - (i + 1));
      uint8_t touchThreshold = EEPROM.read(len - (i + 1) - maxElectrodes);

      if (touchThreshold < 255) {
        setTouchThreshold(i, touchThreshold + 1); // EEPROM values are saved off-by-one
      }
      else {
        setTouchThreshold(i, defaultSettings.TTHRESH);
      }

      if (releaseThreshold < 255) {
        setReleaseThreshold(i, releaseThreshold + 1); // EEPROM values are saved off-by-one
      }
      else {
        setReleaseThreshold(i, defaultSettings.RTHRESH);
      }
    }
  #endif
}

void MPR121_goSlow(){
  //Wire.setClock(100000L); // set I2C clock to 100kHz
}

void MPR121_goFast(){
    //Wire.setClock(400000L); // set I2C clock to 400kHz
}

void MPR121_run(){
  if(!MPR121_isInited()) return;
  MPR121_setRegister(MPR121_ECR, ECR_backup); // restore backup to return to run mode
}

void MPR121_stop(){
  if(!MPR121_isInited()) return;
  ECR_backup = MPR121_getRegister(MPR121_ECR); // backup MPR121_ECR to restore when we enter run
  MPR121_setRegister(MPR121_ECR, ECR_backup & 0xC0); // turn off all electrodes to stop
}

bool MPR121_reset(){
  // return true if we successfully reset a device at the
  // address we are expecting

  // MPR121_AFE2 is one of the few registers that defaults to a non-zero value -
  // checking it is sensible as reading back an incorrect value implies
  // something went wrong - we also check MPR121_TS2 bit 7 to see if we have an
  // overcurrent flag set

	MPR121_setRegister(MPR121_SRST, 0x63); // soft reset

  if(MPR121_getRegister(MPR121_AFE2)!=0x24){
    error |= 1<<READBACK_FAIL_BIT;
  } else {
    error &= ~(1<<READBACK_FAIL_BIT);
  }

  if((MPR121_getRegister(MPR121_TS2)&0x80)!=0){
    error |= 1<<OVERCURRENT_FLAG_BIT;
  } else {
    error &= ~(1<<OVERCURRENT_FLAG_BIT);
  }

  if(MPR121_getError()==NOT_INITED || MPR121_getError()==NO_ERROR){ // if our only error is that we are not inited...
    return true;
  } else {
    return false;
  }
}

void MPR121_applySettings(MPR121_settings_type *settings){
  bool wasRunning = running;
  if(wasRunning) MPR121_stop();  // can't change most regs when running - checking
              // here avoids multiple stop() / run() calls

  MPR121_setRegister(MPR121_MHDR,settings->MHDR);
  MPR121_setRegister(MPR121_NHDR,settings->NHDR);
  MPR121_setRegister(MPR121_NCLR,settings->NCLR);
  MPR121_setRegister(MPR121_FDLR,settings->FDLR);
  MPR121_setRegister(MPR121_MHDF,settings->MHDF);
  MPR121_setRegister(MPR121_NHDF,settings->NHDF);
  MPR121_setRegister(MPR121_NCLF,settings->NCLF);
  MPR121_setRegister(MPR121_FDLF,settings->FDLF);
  MPR121_setRegister(MPR121_NHDT,settings->NHDT);
  MPR121_setRegister(MPR121_NCLT,settings->NCLT);
  MPR121_setRegister(MPR121_FDLT,settings->FDLT);
  MPR121_setRegister(MPR121_MHDPROXR,settings->MHDPROXR);
  MPR121_setRegister(MPR121_NHDPROXR,settings->NHDPROXR);
  MPR121_setRegister(MPR121_NCLPROXR,settings->NCLPROXR);
  MPR121_setRegister(MPR121_FDLPROXR,settings->FDLPROXR);
  MPR121_setRegister(MPR121_MHDPROXF,settings->MHDPROXF);
  MPR121_setRegister(MPR121_NHDPROXF,settings->NHDPROXF);
  MPR121_setRegister(MPR121_NCLPROXF,settings->NCLPROXF);
  MPR121_setRegister(MPR121_FDLPROXF,settings->FDLPROXF);
  MPR121_setRegister(MPR121_NHDPROXT,settings->NHDPROXT);
  MPR121_setRegister(MPR121_NCLPROXT,settings->NCLPROXT);
  MPR121_setRegister(MPR121_FDLPROXT,settings->FDLPROXT);
  MPR121_setRegister(MPR121_DTR, settings->DTR);
  MPR121_setRegister(MPR121_AFE1, settings->AFE1);
  MPR121_setRegister(MPR121_AFE2, settings->AFE2);
  MPR121_setRegister(MPR121_ACCR0, settings->ACCR0);
  MPR121_setRegister(MPR121_ACCR1, settings->ACCR1);
  MPR121_setRegister(MPR121_USL, settings->USL);
  MPR121_setRegister(MPR121_LSL, settings->LSL);
  MPR121_setRegister(MPR121_TL, settings->TL);

  MPR121_setRegister(MPR121_ECR, settings->ECR);

  error &= ~(1<<NOT_INITED_BIT); // clear not inited error as we have just inited!
  MPR121_setTouchThreshold_all(settings->TTHRESH);
  MPR121_setReleaseThreshold_all(settings->RTHRESH);


  if(wasRunning) MPR121_run();
}

mpr121_error_type MPR121_getError(){

  // important - this resets the IRQ pin - as does any I2C comms

	MPR121_getRegister(MPR121_OORS1);  // OOR registers - we may not have read them yet,
	MPR121_getRegister(MPR121_OORS2);  // whereas the other errors should have been caught

  // order of error precedence is determined in this logic block

  if(!MPR121_isInited()) return NOT_INITED; // this has its own checker function

  if((error & (1<<ADDRESS_UNKNOWN_BIT)) != 0){
    return ADDRESS_UNKNOWN;
  } else if((error & (1<<READBACK_FAIL_BIT)) != 0){
    return READBACK_FAIL;
  } else if((error & (1<<OVERCURRENT_FLAG_BIT)) != 0){
    return OVERCURRENT_FLAG;
  } else if((error & (1<<OUT_OF_RANGE_BIT)) != 0){
    return OUT_OF_RANGE;
  } else return NO_ERROR;

}

void MPR121_clearError(){
  error = 0;
}

bool MPR121_isRunning(){
  return running;
}

bool MPR121_isInited(){
  return (error & (1<<NOT_INITED_BIT)) == 0;
}

void MPR121_updateTouchData(){
  if(!MPR121_isInited()) return;

  autoTouchStatusFlag = false;

  lastTouchData = touchData;
  touchData = (unsigned int)MPR121_getRegister(MPR121_TS1) + ((unsigned int)MPR121_getRegister(MPR121_TS2)<<8);
}

bool MPR121_getTouchData(uint8_t electrode){
  if(electrode>12 || !MPR121_isInited()) return false; // avoid out of bounds behaviour

  return((touchData>>electrode)&1);
}

uint8_t MPR121_getNumTouches(){
  if(!MPR121_isInited()) return(0xFF);

  uint8_t scratch = 0;
  for(uint8_t i=0; i<13; i++){
    if(MPR121_getTouchData(i)) scratch++;
  }

  return(scratch);
}

bool MPR121_getLastTouchData(uint8_t electrode){
  if(electrode>12 || !MPR121_isInited()) return false; // avoid out of bounds behaviour

  return((lastTouchData>>electrode)&1);
}

bool MPR121_updateFilteredData(){
  if(!MPR121_isInited()) return(false);


  uint8_t MSB = 0, LSB = 0, index = 0, reg_addr = MPR121_E0FDL;
  taskENTER_CRITICAL();
  do {
	  if (HAL_I2C_Mem_Read(i2c_handle, address << 1, reg_addr, 1, &LSB, 1, I2C_TIMEOUT) != HAL_OK) return false;
	  reg_addr++;
	  if (HAL_I2C_Mem_Read(i2c_handle, address << 1, reg_addr, 1, &MSB, 1, I2C_TIMEOUT) != HAL_OK) return false;
	  reg_addr++;
	  filteredData[index] = ((MSB << 8) | LSB);
	  index++;
  } while (reg_addr <= MPR121_E12FDH);
  taskEXIT_CRITICAL();
return true;
}

int MPR121_getFilteredData(uint8_t electrode){
  if(electrode>12 || !MPR121_isInited()) return(0xFFFF); // avoid out of bounds behaviour

  return(filteredData[electrode]);
}

bool MPR121_updateBaselineData(){
  if(!MPR121_isInited()) return(false);
  uint8_t index = 0, reg_data = 0;
  taskENTER_CRITICAL();
  for (uint8_t reg_addr = MPR121_E0BV; reg_addr <= MPR121_E12BV; reg_addr++) {
	  if (HAL_I2C_Mem_Read(i2c_handle, address << 1, reg_addr, 1, &reg_data, 1, I2C_TIMEOUT) != HAL_OK) return false;
	  baselineData[index] = reg_data;
	  index++;
  }
  taskEXIT_CRITICAL();
return true;
}

int MPR121_getBaselineData(uint8_t electrode){
  if(electrode>12 || !MPR121_isInited()) return(0xFFFF); // avoid out of bounds behaviour

  return(baselineData[electrode]);
}

bool MPR121_isNewTouch(uint8_t electrode){
  if(electrode>12 || !MPR121_isInited()) return(false); // avoid out of bounds behaviour
  return((MPR121_getLastTouchData(electrode) == false) && (MPR121_getTouchData(electrode) == true));
}

bool MPR121_isNewRelease(uint8_t electrode){
  if(electrode>12 || !MPR121_isInited()) return(false); // avoid out of bounds behaviour
  return((MPR121_getLastTouchData(electrode) == true) && (MPR121_getTouchData(electrode) == false));
}

void MPR121_updateAll(){
	MPR121_updateTouchData();
	MPR121_updateBaselineData();
	MPR121_updateFilteredData();
}

void MPR121_setTouchThreshold_all(uint8_t val){
  if(!MPR121_isInited()) return;
  bool wasRunning = running;

  if(wasRunning) MPR121_stop();  // can only change thresholds when not running
              // checking here avoids multiple stop() / run()
              // calls

  for(uint8_t i=0; i<13; i++){
	  MPR121_setTouchThreshold(i, val);
  }

  if(wasRunning) MPR121_run();
}

void MPR121_saveTouchThreshold(uint8_t electrode, uint8_t val){
  #ifdef ARDUINO_ARCH_AVR
    if(electrode>12 || !isInited()) return; // avoid out of bounds behaviour

    setTouchThreshold(electrode, val);

    // store to EEPROM
    uint8_t maxElectrodes = 12;
    int len = E2END;
    int addr = len - maxElectrodes - (electrode + 1);
    EEPROM.write(addr, val - 1); // val - 1 so 255 stays as never-written-to
  #endif
}

void MPR121_setTouchThreshold(uint8_t electrode, uint8_t val){
  if(electrode>12 || !MPR121_isInited()) return; // avoid out of bounds behaviour

  // this relies on the internal register map of the MPR121
  MPR121_setRegister(MPR121_E0TTH + (electrode<<1), val);
}

void MPR121_setReleaseThreshold_all(uint8_t val){
  if(!MPR121_isInited()) return;
  bool wasRunning = running;

  if(wasRunning) MPR121_stop();  // can only change thresholds when not running
              // checking here avoids multiple stop / starts

  for(uint8_t i=0; i<13; i++){
	  MPR121_setReleaseThreshold(i,val);
  }

  if(wasRunning) MPR121_run();
}

void MPR121_setReleaseThreshold(uint8_t electrode, uint8_t val){
  if(electrode>12 || !MPR121_isInited()) return; // avoid out of bounds behaviour

  // this relies on the internal register map of the MPR121
  MPR121_setRegister(MPR121_E0RTH + (electrode<<1), val);
}

void MPR121_saveReleaseThreshold(uint8_t electrode, uint8_t val){
  #ifdef ARDUINO_ARCH_AVR
    if(electrode>12 || !isInited()) return; // avoid out of bounds behaviour

    setReleaseThreshold(electrode, val);

    // store to EEPROM
    int len = E2END;
    int addr = len - (electrode + 1);
    EEPROM.write(addr, val - 1); // val - 1 so 255 stays as never-written-to
  #endif
}

uint8_t MPR121_getTouchThreshold(uint8_t electrode){
  if(electrode>12 || !MPR121_isInited()) return(0xFF); // avoid out of bounds behaviour
  return(MPR121_getRegister(MPR121_E0TTH+(electrode<<1))); // "255" issue is in here somewhere
  //return(101);
}

uint8_t MPR121_getReleaseThreshold(uint8_t electrode){
  if(electrode>12 || !MPR121_isInited()) return(0xFF); // avoid out of bounds behaviour
  return(MPR121_getRegister(MPR121_E0RTH+(electrode<<1))); // "255" issue is in here somewhere
  //return(51);
}


void MPR121_setProxMode(mpr121_proxmode_type mode){

  if(!MPR121_isInited()) return;

  bool wasRunning = running;

  if(wasRunning) MPR121_stop();

  switch(mode){
    case PROX_DISABLED:
      ECR_backup &= ~(3<<4);  // ELEPROX_EN[1:0] = 00
      break;
    case PROX_0_1:
      ECR_backup |=  (1<<4);  // ELEPROX_EN[1:0] = 01
      ECR_backup &= ~(1<<5);
      break;
    case PROX_0_3:
      ECR_backup &= ~(1<<4);  // ELEPROX_EN[1:0] = 10
      ECR_backup |=  (1<<5);
      break;
    case PROX_0_11:
      ECR_backup |=  (3<<4);  // ELEPROX_EN[1:0] = 11
      break;
  }

  if(wasRunning) MPR121_run();
}

void MPR121_setCalibrationLock(mpr121_cal_lock_type lock){

  if(!MPR121_isInited()) return;

  bool wasRunning = running;

  if(wasRunning) MPR121_stop();

  switch(lock){
    case CAL_LOCK_ENABLED:
      ECR_backup &= ~(3<<6);  // CL[1:0] = 00
      break;
    case CAL_LOCK_DISABLED:
      ECR_backup |=  (1<<6);  // CL[1:0] = 01
      ECR_backup &= ~(1<<7);
      break;
    case CAL_LOCK_ENABLED_5_BIT_COPY:
      ECR_backup &= ~(1<<6);  // CL[1:0] = 10
      ECR_backup |=  (1<<7);
      break;
    case CAL_LOCK_ENABLED_10_BIT_COPY:
      ECR_backup |=  (3<<4);  // CL[1:0] = 11
      break;
  }

  if(wasRunning) MPR121_run();
}

void MPR121_setGlobalCDC(uint8_t CDC){
  if(CDC > 63) return; // current is only valid 0..63uA

  MPR121_setRegister(MPR121_AFE1, (MPR121_getRegister(MPR121_AFE1) & 0xC0) | CDC);
}

void MPR121_setElectrodeCDC(uint8_t electrode, uint8_t CDC){
  if(CDC > 63 || electrode > 12) return; // current is only valid 0..63uA, electrode only valid 0..12

  MPR121_setRegister(MPR121_CDC0 + electrode, CDC);
}

void MPR121_setGlobalCDT(mpr121_CDT_type CDT){
	MPR121_setRegister(MPR121_AFE2, (MPR121_getRegister(MPR121_AFE2) & 0x1F) | (CDT << 5));
}

void MPR121_setElectrodeCDT(uint8_t electrode, mpr121_CDT_type CDT){
	MPR121_setRegister(MPR121_CDT01 + (electrode >> 1), (MPR121_getRegister(MPR121_CDT01 + (electrode >> 1)) & (0x0F << (((electrode + 1) % 2)<<2))) | (CDT << ((electrode % 2)<<2)));
}

bool MPR121_autoSetElectrodeCDC(uint8_t electrode, uint16_t VCC_mV) {
  uint16_t upper_limit_FDAT = (uint16_t)((((uint32_t)VCC_mV - 700)*256)/VCC_mV) << 2;
  uint16_t target_FDAT = (uint16_t)(((uint32_t)upper_limit_FDAT * 90) / 100);
  uint16_t lower_limit_FDAT = (uint16_t)(((uint32_t)upper_limit_FDAT * 65) / 100);

  uint16_t this_value;
  int16_t last_distance;
  int16_t this_distance;

  const uint8_t max_num_delay_loops = 100;
  uint8_t num_delay_loops;

  bool scratch = false; // default to failure
  uint8_t saved_num_enabled_electrodes = MPR121_getNumEnabledElectrodes();

  MPR121_setNumEnabledElectrodes(electrode + 1); // reducing the number of running electrodes to a minimum speeds things up
  if(!running) MPR121_run();

  for(uint8_t CDC = 1; CDC < 63; CDC ++){
	  MPR121_setElectrodeCDC(electrode, CDC);
    num_delay_loops = 0;

    do{
    	MPR121_updateFilteredData();
    } while((MPR121_getFilteredData(electrode) == 0) && (num_delay_loops++ < max_num_delay_loops));

    this_value = MPR121_getFilteredData(electrode);

    this_distance = (uint16_t)(abs((int16_t)this_value - (int16_t)target_FDAT)); // TODO: tidy up signed / unsigned types here
    if(CDC > 1){ // only need to see if we need to quit once we have at least two measurements to compare
      if(this_distance > last_distance){ // if we got further away from our target this setting should work (slightly prefer higher values)
    	  MPR121_setElectrodeCDC(electrode, CDC);
        if((this_value >= lower_limit_FDAT) && (this_value <= upper_limit_FDAT)){
          scratch = true; // success
        }
        break;
      } else if(CDC == 63){ // or if we're at the end of the available adjustment, see if we're close enough
    	  MPR121_setElectrodeCDC(electrode, CDC);
        if((this_value >= lower_limit_FDAT) && (this_value <= upper_limit_FDAT)){
          scratch = true; // success
        }
        break;
      }
    }
    last_distance = this_distance;
  }

  MPR121_setRegister(MPR121_ECR, ECR_backup);
  MPR121_setNumEnabledElectrodes(saved_num_enabled_electrodes); // have to do this separately as ECR_backup gets invalidated by setNumEnabledElectrodes(electrode + 1);

  return(scratch);
}

bool MPR121_autoSetElectrodeCDC_default(uint8_t electrode){
  // default to 3.3V VCC if not explicitly stated
  return(MPR121_autoSetElectrodeCDC(electrode, 3300));
}

bool MPR121_autoSetElectrodeCDC_all(){
  bool scratch = true;
  for(uint8_t i=0; i < MPR121_getNumEnabledElectrodes(); i++){
    scratch = MPR121_autoSetElectrodeCDC_default(i) ? scratch : false;
  }

  return(scratch);
}

bool MPR121_autoSetElectrodes(uint16_t VCC_mV, bool fixedChargeTime){
  uint8_t USL = (uint8_t)((((uint32_t)VCC_mV - 700)*256)/VCC_mV);
  uint8_t T_L = (uint8_t)(((uint16_t)USL * 90) / 100);
  uint8_t LSL = (uint8_t)(((uint16_t)USL * 65) / 100);
  bool wasRunning = running;

  MPR121_stop();

  MPR121_setRegister(MPR121_USL, USL);
  MPR121_setRegister(MPR121_TL, T_L);
  MPR121_setRegister(MPR121_LSL, LSL);

  // don't enable retry, copy other settings from elsewhere
  MPR121_setRegister(MPR121_ACCR0, 1 | ((ECR_backup & 0xC0) >> 4) | (MPR121_getRegister(MPR121_AFE1) & 0xC0));
  // fixed charge time is useful for designs with higher lead-in resistance - e.g. using Bare Electric Paint
  MPR121_setRegister(MPR121_ACCR1, fixedChargeTime ? 1 << 7 : 0);

  if(wasRunning){
	  MPR121_run();
  }

  return(!(MPR121_getRegister(MPR121_OORS2) & 0xC0));
}

bool MPR121_autoSetElectrodes_all(bool fixedChargeTime){
  return(MPR121_autoSetElectrodes(3300, fixedChargeTime));
}

void MPR121_setNumDigPins(uint8_t numPins){
  if(!MPR121_isInited()) return;
  bool wasRunning = running;

  if(numPins>8) numPins = 8; // maximum number of GPIO pins is 8 out of 12

  if(wasRunning){
	  MPR121_stop(); // have to stop to change MPR121_ECR
  }
  ECR_backup = (0x0F&(12-numPins)) | (ECR_backup&0xF0);
  if(wasRunning){
	  MPR121_run();
  }
}

void MPR121_setNumEnabledElectrodes(uint8_t numElectrodes){
  if(!MPR121_isInited()) return;
  bool wasRunning = running;

  if(numElectrodes>12) numElectrodes = 12; // avoid out-of-bounds behaviour

  if(wasRunning){
	  MPR121_stop(); // have to stop to change MPR121_ECR
  }
  ECR_backup = (0x0F&numElectrodes) | (ECR_backup&0xF0);
  if(wasRunning){
	  MPR121_run();
  }
}

uint8_t MPR121_getNumEnabledElectrodes(){
  if(!MPR121_isInited()) return(0xFF);

  return(MPR121_getRegister(MPR121_ECR) & 0x0F);
}



void MPR121_pinMode(uint8_t electrode, int mode){
  // this is to catch the fact that Arduino prefers its definitions of
  // INPUT, OUTPUT and INPUT_PULLUP to ours...

  // only valid for ELE4..ELE11
  if(electrode<4 || electrode >11 || !MPR121_isInited()) return;

  uint8_t bitmask = 1<<(electrode-4);

  switch(mode){
    case OUTPUT:
      // MPR121_EN = 1
      // MPR121_DIR = 1
      // MPR121_CTL0 = 0
      // MPR121_CTL1 = 0
    	MPR121_setRegister(MPR121_EN, MPR121_getRegister(MPR121_EN) | bitmask);
    	MPR121_setRegister(MPR121_DIR, MPR121_getRegister(MPR121_DIR) | bitmask);
    	MPR121_setRegister(MPR121_CTL0, MPR121_getRegister(MPR121_CTL0) & ~bitmask);
    	MPR121_setRegister(MPR121_CTL1, MPR121_getRegister(MPR121_CTL1) & ~bitmask);
      break;

    case INPUT:
      // MPR121_EN = 1
      // MPR121_DIR = 0
      // MPR121_CTL0 = 0
      // MPR121_CTL1 = 0
    	MPR121_setRegister(MPR121_EN, MPR121_getRegister(MPR121_EN) | bitmask);
    	MPR121_setRegister(MPR121_DIR, MPR121_getRegister(MPR121_DIR) & ~bitmask);
    	MPR121_setRegister(MPR121_CTL0, MPR121_getRegister(MPR121_CTL0) & ~bitmask);
    	MPR121_setRegister(MPR121_CTL1, MPR121_getRegister(MPR121_CTL1) & ~bitmask);
      break;

    case INPUT_PULLUP:
      // MPR121_EN = 1
      // MPR121_DIR = 0
      // MPR121_CTL0 = 1
      // MPR121_CTL1 = 1
    	MPR121_setRegister(MPR121_EN, MPR121_getRegister(MPR121_EN) | bitmask);
    	MPR121_setRegister(MPR121_DIR, MPR121_getRegister(MPR121_DIR) & ~bitmask);
    	MPR121_setRegister(MPR121_CTL0, MPR121_getRegister(MPR121_CTL0) | bitmask);
    	MPR121_setRegister(MPR121_CTL1, MPR121_getRegister(MPR121_CTL1) | bitmask);
      break;

    case INPUT_PULLDOWN:
          // MPR121_EN = 1
          // MPR121_DIR = 0
          // MPR121_CTL0 = 1
          // MPR121_CTL1 = 0
    	MPR121_setRegister(MPR121_EN, MPR121_getRegister(MPR121_EN) | bitmask);
    	MPR121_setRegister(MPR121_DIR, MPR121_getRegister(MPR121_DIR) & ~bitmask);
    	MPR121_setRegister(MPR121_CTL0, MPR121_getRegister(MPR121_CTL0) | bitmask);
    	MPR121_setRegister(MPR121_CTL1, MPR121_getRegister(MPR121_CTL1) & ~bitmask);
          break;

        case OUTPUT_HIGHSIDE:
          // MPR121_EN = 1
          // MPR121_DIR = 1
          // MPR121_CTL0 = 1
          // MPR121_CTL1 = 1
        	MPR121_setRegister(MPR121_EN, MPR121_getRegister(MPR121_EN) | bitmask);
        	MPR121_setRegister(MPR121_DIR, MPR121_getRegister(MPR121_DIR) | bitmask);
        	MPR121_setRegister(MPR121_CTL0, MPR121_getRegister(MPR121_CTL0) | bitmask);
        	MPR121_setRegister(MPR121_CTL1, MPR121_getRegister(MPR121_CTL1) | bitmask);
          break;

        case OUTPUT_LOWSIDE:
          // MPR121_EN = 1
          // MPR121_DIR = 1
          // MPR121_CTL0 = 1
          // MPR121_CTL1 = 0
        	MPR121_setRegister(MPR121_EN, MPR121_getRegister(MPR121_EN) | bitmask);
        	MPR121_setRegister(MPR121_DIR, MPR121_getRegister(MPR121_DIR) | bitmask);
        	MPR121_setRegister(MPR121_CTL0, MPR121_getRegister(MPR121_CTL0) | bitmask);
        	MPR121_setRegister(MPR121_CTL1, MPR121_getRegister(MPR121_CTL1) & ~bitmask);
          break;


    default:
      break;
  }
}

void MPR121_digitalWrite(uint8_t electrode, uint8_t val){

  // avoid out of bounds behaviour

  if(electrode<4 || electrode>11 || !MPR121_isInited()) return;

  if(val){
	  MPR121_setRegister(MPR121_SET, 1<<(electrode-4));
  } else {
	  MPR121_setRegister(MPR121_CLR, 1<<(electrode-4));
  }
}

void MPR121_digitalToggle(uint8_t electrode){

  // avoid out of bounds behaviour

  if(electrode<4 || electrode>11 || !MPR121_isInited()) return;

  MPR121_setRegister(MPR121_TOG, 1<<(electrode-4));
}

bool MPR121_digitalRead(uint8_t electrode){

  // avoid out of bounds behaviour

  if(electrode<4 || electrode>11 || !MPR121_isInited()) return false;

  return(((MPR121_getRegister(MPR121_DAT)>>(electrode-4))&1)==1);
}

void MPR121_analogWrite(uint8_t electrode, uint8_t value){
  // LED output 5 (ELE9) and output 6 (ELE10) have a PWM bug
  // https://community.nxp.com/thread/305474

  // avoid out of bounds behaviour

  if(electrode<4 || electrode>11 || !MPR121_isInited()) return;

  uint8_t shiftedVal = value>>4;

  if(shiftedVal > 0){
	  MPR121_setRegister(MPR121_SET, 1<<(electrode-4)); // normal PWM operation
  } else {
    // this make a 0 PWM setting turn off the output
	  MPR121_setRegister(MPR121_CLR, 1<<(electrode-4));
  }

  switch(electrode-4){

  case 0:
	  MPR121_setRegister(MPR121_PWM0, (shiftedVal & 0x0F) | (MPR121_getRegister(MPR121_PWM0) & 0xF0));
    break;
  case 1:
	  MPR121_setRegister(MPR121_PWM0, ((shiftedVal & 0x0F)<<4) | (MPR121_getRegister(MPR121_PWM0) & 0x0F));
    break;
  case 2:
	  MPR121_setRegister(MPR121_PWM1, (shiftedVal & 0x0F) | (MPR121_getRegister(MPR121_PWM1) & 0xF0));
    break;
  case 3:
	  MPR121_setRegister(MPR121_PWM1, ((shiftedVal & 0x0F)<<4) | (MPR121_getRegister(MPR121_PWM1) & 0x0F));
    break;
  case 4:
	  MPR121_setRegister(MPR121_PWM2, (shiftedVal & 0x0F) | (MPR121_getRegister(MPR121_PWM2) & 0xF0));
    break;
  case 5:
	  MPR121_setRegister(MPR121_PWM2, ((shiftedVal & 0x0F)<<4) | (MPR121_getRegister(MPR121_PWM2) & 0x0F));
    break;
  case 6:
	  MPR121_setRegister(MPR121_PWM3, (shiftedVal & 0x0F) | (MPR121_getRegister(MPR121_PWM3) & 0xF0));
    break;
  case 7:
	  MPR121_setRegister(MPR121_PWM3, ((shiftedVal & 0x0F)<<4) | (MPR121_getRegister(MPR121_PWM3) & 0x0F));
    break;
  }
}

void MPR121_setSamplePeriod(mpr121_sample_interval_type period){
	MPR121_setRegister(MPR121_AFE2, (MPR121_getRegister(MPR121_AFE2) & 0xF8) | (period & 0x07));
}

void MPR121_setFFI(mpr121_FFI_type FFI){
	MPR121_setRegister(MPR121_AFE1, (MPR121_getRegister(MPR121_AFE1) & 0x3F) | ((FFI & 0x03) << 6));
}

void MPR121_setSFI(mpr121_SFI_type SFI){
	MPR121_setRegister(MPR121_AFE2, (MPR121_getRegister(MPR121_AFE2) & 0xE7) | ((SFI & 0x03) << 3));
}

//MPR121_type MPR121 = MPR121_type();


