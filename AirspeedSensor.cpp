#pragma once
#include "AirspeedSensor.h"
#include <Arduino.h>

//information about converting pressure to airspeed can be found here:
//http://www.grc.nasa.gov/WWW/K-12/airplane/pitot.html


//to convert ADC(Analog to Digital Converter) output to voltage:
//voltage = (10 or 12 bit number) / (2^10 or 2^12) * (supply voltage) * (voltage divider ratio)

AirspeedSensor::AirspeedSensor(){
}

AirspeedSensor::~AirspeedSensor(void){
}

void AirspeedSensor::initialize(){
  #ifdef ARDUINO_TYPE_DUE
  analogReadResolution(12);  //arduino due has 12 bit ADC capability, comment out this line if using UNO
  #endif

  //define input filters
  //define IIR low pass filter with digital cut off frequency = 0.0713
  //barometer is updated in slow loop every 250ms
  //update frequency = 1/(250e-3) = 4Hz5
  //Real cut off frequency is 5 * 0.0713 = 0.285Hz
  //Airpseed sensor also has a analog RC low pass filter with a cut off frequency of 1.59Hz
  airspeedFilter = new IIR_doubleFilter(0.5);

  //Set up initial calibration settings
  zeroVoltage = 2.6;
  voltagePressureScale = 1.0;

  //read initial values
  for(int i = 0; i < 5; i++){
    double d = readAirSpeed();
  }
  
  unitType = METRES;  
}

double AirspeedSensor::readAirSpeed(){        
  //analogRead() returns a value between 0 and 4095 or 0 and 1023 depending on arduino type

  #ifdef ARDUINO_TYPE_DUE
  double v=5*2; // because of voltage divider: voltage divider diveds the input voltage by 2
  double inputVoltage = analogRead(AIRSPEED_SENSOR_PIN);
  inputVoltage = inputVoltage / 4095 *v ;//12 bit -> 4095 steps   
  #endif

  #ifdef ARDUINO_TYPE_UNO
  double v=5;
  double inputVoltage = analogRead(AIRSPEED_SENSOR_PIN);
  inputVoltage = inputVoltage / 1023 * v;//10 bit -> 1024 steps
  #endif


  double density = 1.18;
  //sensor zero input voltage is 2.5V
  //sensitivity is 1V/kPa

  //do calculations

  double pressure = (inputVoltage - zeroVoltage) * voltagePressureScale; //kPa
  double temp = 2.0 * pressure * 100 / density;

  double airspeed = 0;
  
  if(temp >= 0){
    airspeed = sqrt(temp) * 2.23694;  //miles per hour
  }
  else{
    airspeed = 0;  //don't display negative speeds, these are just confusing (caused by wind blowing sideways to the tube)
  }
  if(unitType==METRES){ //  m/sM
      airspeed=airspeed*0.44704;
  }  
  else if(unitType==FEET){//    ft/s
      airspeed=airspeed*1.46667;
  }
  
  //add to filter
  airspeedFilter->addValue(airspeed);
  return airspeedFilter->getCurrentValue();
  
}

void AirspeedSensor::setType(int unit)
{  
     unitType=unit;
}

void AirspeedSensor::reset(){
  
  //zero();
  
  airspeedFilter->reset();
   for(int i = 0; i < 50; i++){  //5 times as many times as filter order
          double d =readAirSpeed();
          delay(10); 
    }     
}

