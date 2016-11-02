/**************************************************************************/
/*!
    @file     Adafruit_MPL3115A2.cpp
    @author   K.Townsend (Adafruit Industries)
    @license  BSD (see license.txt)

    Driver for the MPL3115A2 barometric pressure sensor

    This is a library for the Adafruit MPL3115A2 breakout
    ----> https://www.adafruit.com/products/1893

    Adafruit invests time and resources providing this open source code,
    please support Adafruit and open-source hardware by purchasing
    products from Adafruit!

    @section  HISTORY

    v1.0 - First release
*/
/**************************************************************************/
#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#ifdef __AVR_ATtiny85__
 #include <TinyWireM.h>
 #define Wire TinyWireM
#else
 //#include <Wire.h>
 #include "DueWire.h"
#endif

#include "Adafruit_MPL3115A2.h"

/**************************************************************************/
/*!
    @brief  Instantiates a new MPL3115A2 class
*/
/**************************************************************************/
Adafruit_MPL3115A2::Adafruit_MPL3115A2() {

}

/**************************************************************************/
/*!
    @brief  Setups the HW (reads coefficients values, etc.)
*/
/**************************************************************************/
boolean Adafruit_MPL3115A2::begin() {

  DueWire.begin();
  uint8_t whoami = read8(MPL3115A2_WHOAMI);
  //Serial.print("whoami: ");
  //Serial.println(whoami, HEX);
  
  //SerialUSB.print("whoami: ");
  //SerialUSB.println(whoami, HEX);
  
  if (whoami != 0xC4) {
    return false;
  }

  write8(MPL3115A2_CTRL_REG1,
	 MPL3115A2_CTRL_REG1_SBYB |
	 MPL3115A2_CTRL_REG1_OS128 |
	 MPL3115A2_CTRL_REG1_ALT);
  write8(MPL3115A2_PT_DATA_CFG, 
	 MPL3115A2_PT_DATA_CFG_TDEFE |
	 MPL3115A2_PT_DATA_CFG_PDEFE |
	 MPL3115A2_PT_DATA_CFG_DREM);
  return true;
}

void Adafruit_MPL3115A2::zero() {
  zeroAltitude = 0;
  zeroAltitude = getAltitude(true);
}

void Adafruit_MPL3115A2::setReadTimeout(int timeoutToSet) {
	readTimeout = timeoutToSet; 
}

/**************************************************************************/
/*!
    @brief  Gets the floating-point pressure level in kPa
*/
/**************************************************************************/
float Adafruit_MPL3115A2::getPressure() {
  uint32_t pressure;

  write8(MPL3115A2_CTRL_REG1, 
	 MPL3115A2_CTRL_REG1_SBYB |
	 MPL3115A2_CTRL_REG1_OS128 |
	 MPL3115A2_CTRL_REG1_BAR);

  uint8_t sta = 0;
  while (! (sta & MPL3115A2_REGISTER_STATUS_PDR)) {
    sta = read8(MPL3115A2_REGISTER_STATUS);
    delay(10);
  }
  
  /*
  DueWire.beginTransmission(MPL3115A2_ADDRESS); // start transmission to device 
  DueWire.write(MPL3115A2_REGISTER_PRESSURE_MSB); 
  DueWire.endTransmission(false); // end transmission
  
  DueWire.requestFrom((uint8_t)MPL3115A2_ADDRESS, (uint8_t)3);// send data n-bytes read
  */
  
  DueWire.requestFrom((uint8_t) MPL3115A2_ADDRESS, (uint8_t) 3, (uint32_t) MPL3115A2_WHOAMI, (uint8_t) 3);
  
  pressure = DueWire.read(); // receive DATA
  pressure <<= 8;
  pressure |= DueWire.read(); // receive DATA
  pressure <<= 8;
  pressure |= DueWire.read(); // receive DATA
  pressure >>= 4;

  float baro = pressure;
  baro /= 4.0;
  return baro;
}

float Adafruit_MPL3115A2::getAltitude(boolean ignoreTimeout) {
  int32_t alt;

  write8(MPL3115A2_CTRL_REG1, 
	 MPL3115A2_CTRL_REG1_SBYB |
	 MPL3115A2_CTRL_REG1_OS128 |
	 MPL3115A2_CTRL_REG1_ALT);

  uint8_t sta = 0;
  unsigned int timeSinceStartOfReading = millis();
  while (! (sta & MPL3115A2_REGISTER_STATUS_PDR)) {
	  if(millis() - timeSinceStartOfReading >= readTimeout && !ignoreTimeout) {
	  	return -999;
	  }
    sta = read8(MPL3115A2_REGISTER_STATUS);
    delay(10);
  }
  
  /*
  DueWire.beginTransmission(MPL3115A2_ADDRESS); // start transmission to device 
  DueWire.write(MPL3115A2_REGISTER_PRESSURE_MSB); 
  DueWire.endTransmission(false); // end transmission
  
  DueWire.requestFrom((uint8_t)MPL3115A2_ADDRESS, (uint8_t)3);// send data n-bytes read
  */
  
  DueWire.requestFrom((uint8_t) MPL3115A2_ADDRESS, (uint8_t) 3, (uint32_t) MPL3115A2_WHOAMI, (uint8_t) 3);
  
  alt = DueWire.read(); // receive DATA
  alt <<= 8;
  alt |= DueWire.read(); // receive DATA
  alt <<= 8;
  alt |= DueWire.read(); // receive DATA
  alt >>= 4;

  if (alt & 0x80000) {
    alt |= 0xFFF00000;
  }

  float altitude = alt;
  altitude /= 16.0;
  return (altitude / DEC_TO_FEET) - zeroAltitude;
}

/**************************************************************************/
/*!
    @brief  Gets the floating-point temperature in Centigrade
*/
/**************************************************************************/
float Adafruit_MPL3115A2::getTemperature() {
  int16_t t;

  uint8_t sta = 0;
  while (! (sta & MPL3115A2_REGISTER_STATUS_TDR)) {
    sta = read8(MPL3115A2_REGISTER_STATUS);
    delay(10);
  }
  
  /*
  DueWire.beginTransmission(MPL3115A2_ADDRESS); // start transmission to device 
  DueWire.write(MPL3115A2_REGISTER_TEMP_MSB); 
  DueWire.endTransmission(false); // end transmission
  
  DueWire.requestFrom((uint8_t)MPL3115A2_ADDRESS, (uint8_t)2);// send data n-bytes read
  */
  
  DueWire.requestFrom((uint8_t) MPL3115A2_ADDRESS, (uint8_t) 2, (uint32_t) MPL3115A2_WHOAMI, (uint8_t) 2);
  
  t = DueWire.read(); // receive DATA
  t <<= 8;
  t |= DueWire.read(); // receive DATA
  t >>= 4;

  float temp = t;
  temp /= 16.0;
  return temp;
}




/*********************************************************************/

uint8_t Adafruit_MPL3115A2::read8(uint8_t a) {
	
  // Fix for the Due!
  DueWire.requestFrom((uint8_t) MPL3115A2_ADDRESS, (uint8_t) 1, (uint32_t) a, (uint8_t) 1);
	
	
  // The following doesn't work on the Due as endTransmission((sendStop) false) isn't respected.
  /*
  DueWire.beginTransmission(MPL3115A2_ADDRESS); // start transmission to device 
  DueWire.write(a); // sends register address to read from
    
  /*
  SerialUSB.print(F("Connection: "));
  SerialUSB.println(DueWire.endTransmission(false)); // end transmission
  
  SerialUSB.print(F("Request: "));
  SerialUSB.println(DueWire.requestFrom((uint8_t)MPL3115A2_ADDRESS, (uint8_t)1)); // send data n-bytes read
  
  */
  
  uint8_t response = DueWire.read(); // receive DATA
  //SerialUSB.print("Response: ");
  //SerialUSB.println(response);
  
  //SerialUSB.println(F("----------------------------------------------"));
  
  return response;

}

void Adafruit_MPL3115A2::write8(uint8_t a, uint8_t d) {
  DueWire.beginTransmission(MPL3115A2_ADDRESS); // start transmission to device 
  DueWire.write(a); // sends register address to write to
  DueWire.write(d); // sends register data
  DueWire.endTransmission(false); // end transmission
}
