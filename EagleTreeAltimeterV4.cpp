#include "EagleTreeAltimeterV4.h"

//SDA: pin A4 / yellow wire
//SCL pin A5 / brown wire

EagleTreeAltimeterV4::EagleTreeAltimeterV4() {

}
void EagleTreeAltimeterV4::initialize(){
  altimeterAddress = 0x76;
  filter = new IIR_doubleFilter(0.5);
  zero();   
}

void EagleTreeAltimeterV4::zero () {
  int data = 0;
  for (int i = 0; i < 10; i++)
    data = readRawData();
  filter->setValue(data);
  zeroVal = data;
}

int EagleTreeAltimeterV4::readRawData() {
  int data;
  byte a, b;
  //send "read" command to sensor
  Wire.beginTransmission(altimeterAddress);
  Wire.write(0x07);
  Wire.endTransmission();
  //request 2 bytes
  Wire.beginTransmission(altimeterAddress);
  Wire.requestFrom(altimeterAddress, 2);
  while(Wire.available()<2);
  a = Wire.read();
  b = Wire.read();
  Wire.endTransmission();
  //combine the bytes to an integer
  data = (a) | (b << 8);
  return data;
} 

double EagleTreeAltimeterV4::readAltitude() {
  int data = readRawData();
  filter->addValue(data);
  return (filter->getCurrentValue() - zeroVal) * DEC_TO_FEET;
}

    /*byte data[2];
    signed short reading = 0xFFFF;
    i2c_start();
    
    // select sensor in write mode
    if (!(i2c_write(SENSOR_ADDRESS | I2C_WRITE_BIT))) {
      
        // send "read data" command to sensor
        if (!i2c_write(0x07)) {
            i2c_restart(); // perform I2C restart
            
            // select sensor in read mode
            if (!i2c_write(| SENSOR_ADDRESS | I2C_READ_BIT)) {
                // read two bytes of sensor data
                data[0] = i2c_read(1);
                data[1] = i2c_read(0);
                reading = *((signed short *)(&data[0]));
            }
        }
    }
    i2c_stop();
    */

  /*Serial.print("bin raw: ");
  Serial.print(data, BIN);
  Serial.print(", dec raw:");
  Serial.print(data);
  Serial.print(", val in metres: ");*/
