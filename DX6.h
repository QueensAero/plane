#pragma once
#include "filter.h"
#define PID_MODE_ON 1
#define PID_MODE_OFF 0
#define AILERON_FLAP_MODE 1
#define AILERON_NORMAL_MODE 0

//#define CALIBRATEING_DX6  //if this line is not commented, the plane will print out the DX7 base values

//X axis points forwards
//Y axis points left
//Z axis points up

//roll is rotation about x axis
//roll to right is positive

//pitch is rotation about y axis
//pitch up is positive

//data structure too keep track of the timing for each pin
typedef struct {
      unsigned long riseTime;
      unsigned long fallTime;
      int  lastGoodWidth;
} pinTimingData;

class DX6{
public:
	DX6();
	~DX6(void);
        void initialize(volatile int *_input_roll, volatile int *_input_pitch, volatile int *_input_PID, volatile int *_inputAileron);

	//input functions
	void readPWM();		//reads the PWM values from the DX6, converts to angle and stores in filter
	double getRoll();	//outputs current value from roll filter
	double getPitch();	//outputs current value from pitch filter
        double getPIDmode();
        double getAileronMode();

	//functions to calibrate the DX6
	void calibrateRoll(int min, int zero, int max);			//set range of roll PWM values the DX6 outputs
	void calibratePitch(int min, int zero, int max);			//set range of pitch PWM values the DX6 outputs
	void setRollRange(double min, double max);		//set corisponding roll angle range
	void setPitchRange(double min, double max);		//set corisponding pitch angle range

	//reads PWM and updates filters (calls readPWM(); ) multiple times
	void readSample(int n);

private:
	//filters for roll and pitch - staibilize input
	IIR_doubleFilter *rollFilter;
	IIR_doubleFilter *pitchFilter;

        int PID_mode;
        int aileron_mode;

	volatile int *input_roll;
	volatile int *input_pitch;
        volatile int *input_PID;
        volatile int *input_Aileron;

	//calibration settings
	int rollMinReading;
	int rollMaxReading;
	int rollZeroReading;

	int pitchMinReading;
	int pitchMaxReading;
	int pitchZeroReading;

	int rollVal;
	int pitchVal;

	double min_roll;
	double max_roll;
	double min_pitch;
	double max_pitch;

	double roll_scale;		//used to convert between PWM value and angle for roll
	double pitch_scale;		//used to convert between PWM value and angle for pitch       
        
};



