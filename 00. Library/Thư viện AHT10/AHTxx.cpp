/***************************************************************************************************/
/*
   This is an Arduino library for Aosong ASAIR AHT10/AHT15/AHT20/AHT21/AHT25/AM2301B/AM2311B
   Digital Humidity & Temperature Sensor

   written by : enjoyneering
   sourse code: https://github.com/enjoyneering/
   modifications for Soft-I2C by karsten

   Aosong ASAIR AHT1x/AHT2x features:
   - AHT1x +1.8v..+3.6v, AHT2x +2.2v..+5.5v
   - AHT1x 0.25uA..320uA, AHT2x 0.25uA..980uA
   - temperature range -40C..+85C
   - humidity range 0%..100%
   - typical accuracy T +-0.3C, RH +-2%
   - typical resolution T 0.01C, RH 0.024%
   - normal operating range T -20C..+60C, RH 10%..80%
   - maximum operating rage T -40C..+80C, RH 0%..100%
   - response time 8..30sec*
   - I2C bus speed 100KHz..400KHz, 10KHz recommended minimum
     *measurement with high frequency leads to heating
      of the sensor, must be > 2 seconds apart to keep
      self-heating below 0.1C

   This device uses I2C bus to communicate, specials pins are required to interface
   Board:                                    SDA              SCL              Level
   Uno, Mini, Pro, ATmega168, ATmega328..... A4               A5               5v
   Mega2560................................. 20               21               5v
   Due, SAM3X8E............................. 20               21               3.3v
   Leonardo, Micro, ATmega32U4.............. 2                3                5v
   Digistump, Trinket, ATtiny85............. PB0              PB2              5v
   Blue Pill, STM32F103xxxx boards.......... PB9/PB7*         PB8/PB6*         3.3v/5v
   ESP8266 ESP-01........................... GPIO0**          GPIO2**          3.3v/5v
   NodeMCU 1.0, WeMos D1 Mini............... GPIO4/D2         GPIO5/D1         3.3v/5v
   ESP32.................................... GPIO21/D21       GPIO22/D22       3.3v
                                             *hardware I2C Wire mapped to Wire1 in stm32duino
                                              see https://github.com/stm32duino/wiki/wiki/API#i2c
                                            **most boards has 10K..12K pullup-up resistor
                                              on GPIO0/D3, GPIO2/D4/LED & pullup-down on
                                              GPIO15/D8 for flash & boot

   Frameworks & Libraries:
   ATtiny  Core - https://github.com/SpenceKonde/ATTinyCore
   ESP32   Core - https://github.com/espressif/arduino-esp32
   ESP8266 Core - https://github.com/esp8266/Arduino
   STM32   Core - https://github.com/stm32duino/Arduino_Core_STM32


   GNU GPL license, all text above must be included in any redistribution,
   see link for details  - https://www.gnu.org/licenses/licenses.html
*/
/***************************************************************************************************/

#include "AHTxx.h"

#include <SlowSoftI2CMaster.h>
// create new instance with SDA, SCL, enable internal pullups
SlowSoftI2CMaster si2c = SlowSoftI2CMaster(PIN_SOFT_SDA, PIN_SOFT_SCL, false);

/**************************************************************************/
/*
    Constructor
*/
/**************************************************************************/
AHTxx::AHTxx(uint8_t address, AHTXX_I2C_SENSOR sensorType)
{
  _address    = address << 1;
  _sensorType = sensorType;
  _status     = AHTXX_NO_ERROR;
}

/**************************************************************************/
/*
    begin()

    Initialize I2C & sensor

    NOTE:
    - call this function before doing anything else!!!

    - returned value by "Wire2.endTransmission()":
      - 0 success
      - 1 data too long to fit in transmit data buffer
      - 2 received NACK on transmit of address
      - 3 received NACK on transmit of data
      - 4 other error
*/
/**************************************************************************/
#if defined(__AVR__)
bool AHTxx::begin(uint32_t speed, uint32_t stretch) {
  si2c.i2c_init();
#elif defined(ESP8266) || defined(ESP32)
bool AHTxx::begin(uint8_t sda, uint8_t scl, uint32_t speed, uint32_t stretch) {
  si2c.i2c_init();
#elif defined(_VARIANT_ARDUINO_STM32_)
bool AHTxx::begin(uint8_t sda, uint8_t scl, uint32_t speed) {
  si2c.i2c_init();
#else
bool AHTxx::begin() {
  si2c.i2c_init();
#endif

  delay(AHT2X_POWER_ON_DELAY); //wait for sensor to initialize

  return softReset();          //soft reset is recommended at start (reset, set normal mode,
                               //set calibration bit & check calibration bit)
}


/**************************************************************************/
/*
    readHumidity()

    Read relative humidity, in %

    NOTE:
    - relative humidity range........ 0%..100%
    - relative humidity resolution... 0.024%
    - relative humidity accuracy..... +-2%
    - response time.................. 8sec

    - sensors data structure:
      - {status, RH, RH, RH+T, T, T, CRC*}, *CRC for AHT2x only

    - long-term exposure for 60 hours outside the normal range
      (humidity > 80%) can lead to a temporary drift of the
      signal +3%, sensor slowly returns to the calibrated state at normal
      operating conditions

    - normal operating range T -20C..+60C, RH 10%..80%
    - maximum operating rage T -40C..+80C, RH 0%..100%
*/
/**************************************************************************/
float AHTxx::readHumidity(bool readI2C)
{
  if (readI2C == AHTXX_FORCE_READ_DATA) {_readMeasurement();} //force to read data via I2C & update "_rawData[]" buffer
  if (_status != AHTXX_NO_ERROR)        {return AHTXX_ERROR;} //no reason to continue, call "getStatus()" for error description

  uint32_t humidity   = _rawData[1];                          //20-bit raw humidity data
           humidity <<= 8;
           humidity  |= _rawData[2];
           humidity <<= 4;
           humidity  |= _rawData[3] >> 4;

  if (humidity > 0x100000) {humidity = 0x100000;}             //check if RH>100, no need to check for RH<0 since "humidity" is "uint"

  return ((float)humidity / 0x100000) * 100;
}


/**************************************************************************/
/*
    readTemperature()

    Read temperature, in C 

    NOTE:
    - temperature range........ -40C..+85C
    - temperature resolution... 0.01C
    - temperature accuracy..... +-0.3C
    - response time............ 5..30sec*
      *measurement with high frequency leads to heating of the
       sensor, to detect +-0.1C time between measurements
       should be > 2 seconds

    - sensors data structure:
      - {status, RH, RH, RH+T, T, T, CRC*}, *CRC for AHT2x only
*/
/**************************************************************************/
float AHTxx::readTemperature(bool readAHT)
{
  if (readAHT == AHTXX_FORCE_READ_DATA) {_readMeasurement();} //force to read data via I2C & update "_rawData[]" buffer
  if (_status != AHTXX_NO_ERROR)        {return AHTXX_ERROR;} //no reason to continue, call "getStatus()" for error description

  uint32_t temperature   = _rawData[3] & 0x0F;                //20-bit raw temperature data
           temperature <<= 8;
           temperature  |= _rawData[4];
           temperature <<= 8;
           temperature  |= _rawData[5];

  return ((float)temperature / 0x100000) * 200 - 50;
}


/**************************************************************************/
/*
    setNormalMode()  
 
    Set normal measurement mode

    NOTE:
    - no info in datasheet, suspect this is one measurement & power down
    - true=success, false=I2C error
*/
/**************************************************************************/
bool AHTxx::setNormalMode()
{
  return _setInitializationRegister(AHTXX_INIT_CTRL_CAL_ON | AHT1X_INIT_CTRL_NORMAL_MODE);
}


/**************************************************************************/
/*
    setCycleMode()  
 
    Set cycle measurement mode

    NOTE:
    - no info in datasheet, suspect this is continuous measurement
    - true=success, false=I2C error
*/
/**************************************************************************/
bool AHTxx::setCycleMode()
{
  return _setInitializationRegister(AHTXX_INIT_CTRL_CAL_ON | AHT1X_INIT_CTRL_CYCLE_MODE);
}


/**************************************************************************/
/*
    setComandMode()  
 
    Set command measurement mode

    NOTE:
    - no info in datasheet
    - true=success, false=I2C error
*/
/**************************************************************************/
bool AHTxx::setComandMode()
{
  return _setInitializationRegister(AHTXX_INIT_CTRL_CAL_ON | AHT1X_INIT_CTRL_CMD_MODE);
}


/**************************************************************************/
/*
    softReset()  
 
    Restart sensor, without power off

    NOTE:
    - takes 20ms
    - all registers set to default
*/
/**************************************************************************/
bool AHTxx::softReset()
{
  if (! si2c.i2c_start(_address | I2C_READ)) return false;

//   if (! si2c.i2c_write(AHTXX_SOFT_RESET_REG)) return false;
  si2c.i2c_write(AHTXX_SOFT_RESET_REG);
  si2c.i2c_stop();

  delay(AHTXX_SOFT_RESET_DELAY);

  return ((setNormalMode() == true) && (_getCalibration() == AHTXX_STATUS_CTRL_CAL_ON)); //set mode & check calibration bit
}


/**************************************************************************/
/*
    getStatus()  
 
    Return sensor status

    NOTE:
    - returned statuse:
      - AHTXX_NO_ERROR   = 0x00, success, no errors
      - AHTXX_BUSY_ERROR = 0x01, sensor is busy
      - AHTXX_ACK_ERROR  = 0x02, sensor didn't return ACK
      - AHTXX_DATA_ERROR = 0x03, received data smaller than expected
      - AHTXX_CRC8_ERROR = 0x04, computed CRC8 not match received CRC8, for AHT2x only
*/
/**************************************************************************/
uint8_t AHTxx::getStatus()
{
  return _status;
}


/**************************************************************************/
/*
    setType()  
 
    Set sensor type

    NOTE:
    - AHT1x vs AHT2x:
      - AHT1x +1.8v..+3.6v, AHT2x 2.2v..5.5v
      - AHT1x 0.25uA..320uA, AHT2x 0.25uA..980uA
      - AHT2x support CRC8 check
*/
/**************************************************************************/
void AHTxx::setType(AHTXX_I2C_SENSOR sensorType)
{
  _sensorType = sensorType;
}





/**************************************************************************/
/*
    _readMeasurement()

    Start new measurement, read sensor data to buffer & collect errors

    NOTE:
    - sensors data structure:
      - {status, RH, RH, RH+T, T, T, CRC*}, *CRC for AHT2x only & for
        status description see "_readStatusRegister()" NOTE
*/
/**************************************************************************/
void AHTxx::_readMeasurement()
{
  /* send measurement command */
  si2c.i2c_start(_address | I2C_WRITE);
  
  si2c.i2c_write(AHTXX_START_MEASUREMENT_REG);
  si2c.i2c_write(AHTXX_START_MEASUREMENT_CTRL);
  si2c.i2c_write(AHTXX_START_MEASUREMENT_CTRL_NOP);
  si2c.i2c_stop();

  /* check busy bit */
  _status = _getBusy(AHTXX_FORCE_READ_DATA);                                                //update status byte, read status byte & check busy bit

  if      (_status == AHTXX_BUSY_ERROR) {delay(AHTXX_MEASUREMENT_DELAY - AHTXX_CMD_DELAY);}
  else if (_status != AHTXX_NO_ERROR)   {return;}                                           //no reason to continue, received data smaller than expected

  /* read data from sensor */
  uint8_t dataSize;

  if   (_sensorType == AHT1x_SENSOR) {dataSize = 6;}   //{status, RH, RH, RH+T, T, T, CRC*}, *CRC for AHT2x only
  else                               {dataSize = 7;}

  si2c.i2c_start(_address | I2C_READ);
  /* read n-bytes from "wire.h" rxBuffer */
  for (uint8_t i = 0; i < dataSize; i++)
  {
    if (i == dataSize - 1) {
        _rawData[i] = si2c.i2c_read(1);
    } else {
        _rawData[i] = si2c.i2c_read(0);
    }
  }
  si2c.i2c_stop();

  /* check busy bit after measurement dalay */
  _status = _getBusy(AHTXX_USE_READ_DATA); //update status byte, read status byte & check busy bit

  if (_status != AHTXX_NO_ERROR) {return;} //no reason to continue, sensor is busy

  /* check CRC8, for AHT2x only */
  if ((_sensorType == AHT2x_SENSOR) && (_checkCRC8() != true)) {_status = AHTXX_CRC8_ERROR;} //update status byte
}


/**************************************************************************/
/*
    _setInitializationRegister()
 
    Set initialization register

    NOTE:
    - true=success, false=I2C error
*/
/**************************************************************************/
bool AHTxx::_setInitializationRegister(uint8_t value)
{
  delay(AHTXX_CMD_DELAY);

  if (! si2c.i2c_start(_address | I2C_WRITE)) return false;

  if   (_sensorType == AHT1x_SENSOR) {si2c.i2c_write(AHT1X_INIT_REG);} //send initialization command, for AHT1x only
  else                               {si2c.i2c_write(AHT2X_INIT_REG);} //send initialization command, for AHT2x only

  uint8_t st = si2c.i2c_write(value);                                  //send initialization register controls
  si2c.i2c_write(AHTXX_INIT_CTRL_NOP);                                 //send initialization register NOP control
  si2c.i2c_stop();

  return st;                        //true=success, false=I2C error
}


/**************************************************************************/
/*
    _readStatusRegister()

    Read status register

    NOTE:
    - AHT1x status register controls:
      7    6    5    4   3    2   1   0
      BSY, MOD, MOD, xx, CAL, xx, xx, xx
      - BSY:
        - 1, sensor busy/measuring
        - 0, sensor idle/sleeping
      - MOD:
        - 00, normal mode
        - 01, cycle mode
        - 1x, comand mode
      - CAL:
        - 1, calibration on
        - 0, calibration off

    - AHT2x status register controls:
      7    6   5   4   3    2   1  0
      BSY, xx, xx, xx, CAL, xx, xx, xx

    - under normal conditions status is 0x18 & 0x80 if the sensor is busy
*/
/**************************************************************************/
uint8_t AHTxx::_readStatusRegister()
{
  delay(AHTXX_CMD_DELAY);
  if (! si2c.i2c_start(_address | I2C_WRITE)) return false;

  if (! si2c.i2c_write(AHTXX_STATUS_REG)) return false;
  if (! si2c.i2c_rep_start(_address | I2C_READ)) return false;

  uint8_t sibyte = si2c.i2c_read(1);
  si2c.i2c_stop();
//   Serial.print(F("sibyte "));
//   Serial.print(sibyte);
  return sibyte;
}


/**************************************************************************/
/*
    _getCalibration()

    Read calibration bits from status register

    NOTE:
    - 0x08=loaded, 0x00=not loaded, 0xFF=I2C error
    - calibration status check should only be performed at power-up,
      rechecking is not required during data collection
*/
/**************************************************************************/
uint8_t AHTxx::_getCalibration()
{
  uint8_t value = _readStatusRegister();

  if (value != AHTXX_ERROR) {return (value & AHTXX_STATUS_CTRL_CAL_ON);} //0x08=loaded, 0x00=not loaded
  return AHTXX_ERROR;                       						   //collision on I2C bus, sensor didn't return ACK
}


/**************************************************************************/
/*
    _getBusy()

    Read/check busy bit after measurement command

    NOTE:
    - part of "readRawMeasurement()" function!!!
    - 0x80=busy, 0x00=measurement completed, etc
*/
/**************************************************************************/
uint8_t AHTxx::_getBusy(bool readAHT)
{
  if (readAHT == AHTXX_FORCE_READ_DATA)                    //force to read data via I2C & update "_rawData[]" buffer
  {
    delay(AHTXX_CMD_DELAY);

    if (! si2c.i2c_start(_address | I2C_READ)) return AHTXX_DATA_ERROR;
  
    _rawData[0] = si2c.i2c_read(1);
    si2c.i2c_stop();
  }

  if   ((_rawData[0] & AHTXX_STATUS_CTRL_BUSY) == AHTXX_STATUS_CTRL_BUSY) {_status = AHTXX_BUSY_ERROR;} //0x80=busy, 0x00=measurement completed
  else                                                                    {_status = AHTXX_NO_ERROR;}

  return _status;
}


/**************************************************************************/
/*
    _checkCRC8()

    Check CRC-8-Maxim of AHT2X measured data

    NOTE:
    - part of "readRawMeasurement()" function!!!
    - only AHT2x sensors have CRC
    - initial value=0xFF, polynomial=(x8 + x5 + x4 + 1) ie 0x31 CRC [7:0] = 1+X4+X5+X8
*/
/**************************************************************************/
bool AHTxx::_checkCRC8()
{
  if (_sensorType == AHT2x_SENSOR)
  {
    uint8_t crc = 0xFF;                                      //initial value

    for (uint8_t byteIndex = 0; byteIndex < 6; byteIndex ++) //6-bytes in data, {status, RH, RH, RH+T, T, T, CRC}
    {
      crc ^= _rawData[byteIndex];

      for(uint8_t bitIndex = 8; bitIndex > 0; --bitIndex)    //8-bits in byte
      {
        if   (crc & 0x80) {crc = (crc << 1) ^ 0x31;}         //0x31=CRC seed/polynomial 
        else              {crc = (crc << 1);}
      }
    }

    return (crc == _rawData[6]);
  }

  return true;
}


/******************************************************//**
 * Soft I2C Bus nach devices scannen
 **********************************************************/
void AHTxx::scan_si2c(void) {
	uint8_t add = 0;
	uint8_t found = 0;
	
	// try read
	do {
		if (si2c.i2c_start(add | I2C_READ)) {
			found ++;
			si2c.i2c_read(1);
			si2c.i2c_stop();
			Serial.print(F("Read:  0x"));
			if (add < 0x0F) Serial.print(0, HEX);
			Serial.print(add + I2C_READ, HEX);
			Serial.print(F(" at 0x"));
			if (add >> 1 < 0x0F) Serial.print(0, HEX);
			Serial.println(add >> 1, HEX);
		} else {
			si2c.i2c_stop();
		}
		add += 2;
	} while (add);

	// try write
	add = 0;
	do {
		if (si2c.i2c_start(add | I2C_WRITE)) {
			found ++;
			si2c.i2c_stop();
			Serial.print(F("Write: 0x"));
			if (add < 0x0F) Serial.print(0, HEX);  
			Serial.print(add + I2C_WRITE, HEX);
			Serial.print(F(" at 0x"));
			if (add >> 1 < 0x0F) Serial.print(0, HEX);
			Serial.println(add >> 1, HEX);
		} else {
			si2c.i2c_stop();
		}
		si2c.i2c_stop();
		add += 2;
	} while (add);
	Serial.print(found);
	Serial.println(F(" I2C response"));
}
