#pragma once
#include "DX6.h"
#include <Arduino.h>

//X axis points forwards
//Y axis points left
//Z axis points up

//roll is rotation about x axis
//roll to right is positive

//pitch is rotation about y axis
//pitch up is positive

DX6::DX6()
{

}


DX6::~DX6(void)
{
}

void DX6::initialize(volatile int *_input_roll, volatile int *_input_pitch, volatile int *_input_PID, volatile int *_input_Aileron){
        input_roll = _input_roll;
        input_pitch = _input_pitch;
        input_PID = _input_PID;
        input_Aileron = _input_Aileron;
        
	//define input filters
	rollFilter = new IIR_doubleFilter(0.25);
	pitchFilter = new IIR_doubleFilter(0.25);

        PID_mode = PID_MODE_OFF;
        aileron_mode = AILERON_NORMAL_MODE;

	//Set up intial calibration settings

	rollMinReading = 900;
	rollMaxReading = 2100;
	pitchMinReading = 900;
	pitchMaxReading = 2100;

	//set initial scale and range
 
	max_roll = 20;
	min_roll = -20;

	max_pitch = 20;
	min_pitch = -20;

        //calculate the scale needed to convert the PWM reading to angle
	roll_scale = (max_roll - min_roll) / (rollMaxReading - rollMinReading);
	pitch_scale = (max_pitch - min_pitch) / (pitchMaxReading - pitchMinReading);  
}

void DX6::readSample(int n){
	//read PWM values multiple times
	for(int i = 0; i < n; i++){
		readPWM();
	}
}
double DX6::getRoll(){
    //update PWM filters and return roll value
    return rollVal; //rollFilter->getCurrentValue();
}

double DX6::getPitch(){
    //update PWM filters and return pitch value
    return pitchVal; //pitchFilter->getCurrentValue();
}
double DX6::getPIDmode(){
    return PID_mode;
}
double DX6::getAileronMode(){
    return aileron_mode;
}

void DX6::readPWM(){        
        //check to make sure input values are within range before adding them to filter
        //extraeous values can cause serious problems for the PIDs
        
        rollVal = (*input_roll - rollZeroReading) * roll_scale;
        pitchVal = (*input_pitch - pitchZeroReading) * pitch_scale * -1; //XXX get rid of -1

        //update PID mode and aileron flap mode
        if(*input_PID < 1300) PID_mode = PID_MODE_OFF;
        if(*input_PID > 1700) PID_mode = PID_MODE_ON;
      
        PID_mode = PID_MODE_OFF; //for now!
        
        if(*input_Aileron < 1300) aileron_mode = AILERON_NORMAL_MODE;
        if(*input_Aileron > 1700) aileron_mode = AILERON_FLAP_MODE;
        
        /*
        Serial.print("PID: ");
        Serial.print(PID_mode);
        Serial.print("    Aileron: ");
        Serial.println(aileron_mode);
        */
        
      #ifdef CALIBRATEING_DX6
      Serial.println("Roll    Pitch    PID Mode    Aileron Flap Mode");
      Serial.print(*input_roll);
      Serial.print("      ");
      Serial.print(*input_pitch);
      Serial.print("      ");
      Serial.print(*input_PID);
      Serial.print("      ");
      Serial.print(*input_Aileron);
      Serial.println("      ");  
      #endif
      
}

void DX6::calibrateRoll(int minR, int zeroR, int maxR){
	rollMinReading = minR;
	rollMaxReading = maxR;
	rollZeroReading = zeroR;

	//update roll scale
	roll_scale = (max_roll - min_roll) / (rollMaxReading - rollMinReading);
}

void DX6::calibratePitch(int minR, int zeroR, int maxR){
	pitchMinReading = minR;
	pitchMaxReading = maxR;
	pitchZeroReading = zeroR;

	//update pitch scale
	pitch_scale = (max_pitch - min_pitch) / (pitchMaxReading - pitchMinReading);
}

void DX6::setRollRange(double minR, double maxR){
	min_roll = minR;
	max_roll = maxR;

	//update roll scale
	roll_scale = (max_roll - min_roll) / (rollMaxReading - rollMinReading);
}

void DX6::setPitchRange(double minR, double maxR){
	min_pitch = minR;
	max_pitch = maxR;

	//update pitch scale
	pitch_scale = (max_pitch - min_pitch) / (pitchMaxReading - pitchMinReading);
}



