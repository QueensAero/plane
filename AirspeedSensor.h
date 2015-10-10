 #pragma once
#include "filter.h"

//To tell the complier what version of arduino we are using, only have one of these uncommented at a time!
//#define ARDUINO_TYPE_UNO
#define ARDUINO_TYPE_DUE

#define METRES 1
#define FEET 2



#define AIRSPEED_SENSOR_PIN A0

//http://www.grc.nasa.gov/WWW/K-12/airplane/pitot.html

class AirspeedSensor{
public:
	AirspeedSensor();
	~AirspeedSensor(void);
        void initialize();

	//input functions
	double readAirSpeed();		
        void setType(int);

	void reset();   //resets filters, callss zero

private:
	//filters for roll and pitch - staibilize input
	IIR_doubleFilter *airspeedFilter;

	//calibration settings
	double zeroVoltage;
	double voltagePressureScale;
        int unitType;

};



