#pragma once
#ifndef PID_H
#define PID_H

#include <Arduino.h>
#include "filter.h"

//#define DEBUG_PID

#define INTEGRAL_ATTENUATION_CONSTANT 0.08    
#define OUTPUT_LOW_PASS_FILTER_CONSTANT 0.002 //this controls how fast the PID output can change (small value = slowly changing)
#define DEFAULT_TIME_STEP 0.0025 //SECONDSS

//NEED TO CHANGE THIS VALUE WHEN PROGRAM SPEED CHANGES!
//we want it so that if the leaky integrator gets a unit step, it stable integral value will be 1
//the attenuation constant changes the stable value
//for some reason, this formula works!
#define DIRECT  0
#define REVERSE  1

class PID_inputFilter{
public:
	PID_inputFilter(double a);
	~PID_inputFilter(void);

	void add(double d);
	double computeIntegral();

	void print();

private:        
        double integral;
        double old_factor;           
};

class PID
{
public:
	PID();
	~PID(void);

        void initialize(double *in, double * out, double *setp, double p, double i, double d, double f);
	
	bool update(); 
        unsigned long getRunTime();
	//controler operation functions
	void SetOutputLimits(double Min, double Max);
	void SetTuningParameters(double Kp, double kI, double kd, double kf); 
	void SetControllerDirection(int direction);
	void SetSampleTime(int t);	//milliseconds

	//accessor functions
	double getKp();
	double getKi();
	double getKd();
	int getDirection();

private:
        unsigned long timeChange;  
	//constants for weighted sum
	double Kp, Ki, Kd, Kf;
	//variable to reverse the output direction - set to 1 for 2013-2014
	int controllerDirection;

        double prev_input;
        

	//pointers for input and output
	double *input, *output, *setpoint;

	//variables for calculating integral
	unsigned long sampleTime;
	unsigned long lastTime;
	
	//output bounds
	double outMin, outMax;

	//filters for integral and derivative 
	PID_inputFilter *integralFilter;	
        IIR_doubleFilter *outputFilter;
};

#endif
