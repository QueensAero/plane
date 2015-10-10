#include "PID.h"
#include "filter.h"

//PID constructor
PID::PID(){

}


PID::~PID(void){

}

void PID::initialize(double *in, double * out, double *setp, double p, double i, double d, double f){
   input = in;
   output = out;
   setpoint = setp;
  
    //declare filters that are used to calculate integral and average derivative
    integralFilter = new PID_inputFilter(INTEGRAL_ATTENUATION_CONSTANT);
    prev_input = 0;

    outputFilter = new IIR_doubleFilter(OUTPUT_LOW_PASS_FILTER_CONSTANT);

    //set temporary output limits for now
    SetOutputLimits(0,255);

    //set temporary sample time
    sampleTime = 100;
    lastTime = micros() - sampleTime;  //want Compute() to execute first time it is called
	
    //set up PID 
    SetTuningParameters(p,i,d,f);
    SetControllerDirection(DIRECT);  
}

unsigned long PID::getRunTime(){
     return timeChange;
}

bool PID::update(){
    //check system time
        
	unsigned long now = micros();
	timeChange = (now - lastTime);
        //Serial.println(timeChange);
	if(timeChange >= sampleTime){

		//determine current input and error
                double time_step = timeChange * 1e-6;  //convert to seconds
                
		double current_input = *input; //this comes from motion sensor

                //take derivative of sensor signal in the feedback path
                double derivative = (current_input - prev_input) / time_step * Kd;
                if(derivative > 180) derivative = 180;
                if(derivative < -180) derivative = -180;
                
                //Serial.print(derivative);
                //Serial.print("   ");
                
                //add derivative to the current input
                double feedback_term = derivative + current_input;
                
                double current_setpoint = *setpoint;
 
		double current_error = current_setpoint - feedback_term;

                if(current_error > 180) current_error = 180;
                if(current_error < -180) current_error = -180;
                
                //check for error
                if(isnan(current_error)){
                    return false;
                }                
                
		//compute integral using a low pass filter - this acts as a "leaky integrator"
                //this increases the stability of the system and allows for higher integral constants
                //a pure integral adds a pole at the origine, which pulls the root locus closer to the right and plane - NOT GOOD
                //a low pass filter adds a pole on the negative real axis, further away from the origine - MORE STABLE
                integralFilter->add(current_error); //don't keep track of time step assume constant
  		double integral = integralFilter->computeIntegral();


                prev_input = current_input;

		
		//calculate output
		double output_pre_filter = Kp * current_error + Ki * integral;
                
                //add output to the output filter
                //this is MUCH more stable than filtering the measured angles in the feedback path!!!
                outputFilter->addValue(output_pre_filter);
                double current_output = outputFilter->getCurrentValue();
                
                 #ifdef DEBUG_PID
                 Serial.print(now);
                 Serial.print("  ");
                 Serial.print(current_error);
                 Serial.print("  ");
                 Serial.print(integral);
                 Serial.print("  ");
                 Serial.print(derivative);              
                 Serial.print("  ");
                 Serial.println(current_output);
                #endif

                //check to ensure that bounds are not outside min / max range
		if(current_output < outMin){
			current_output = outMin;
                }
                
		if(current_output > outMax){
			current_output = outMax;
                }
                //Serial.println(current_output);
                //return output
		*output = current_output + Kf * current_setpoint;  //add current PID output to feedforward term
        
		//update last time
		lastTime = now;
		return true;
	}
	else
		return false;
}

//controler operation functions
void PID::SetOutputLimits(double Min, double Max){
	if(Min >= Max) return;

	//if the limits are in the correct order update
	outMin = Min;
	outMax = Max;

	//check to make sure current output is still in range
	if(*output > outMax) * output = outMax;
	else if(*output < outMin) * output = outMin;

}

void PID::SetTuningParameters(double p, double i, double d, double f){
   if (p<0 || i<0 || d<0) return;
 
   //double SampleTimeInSec = ((double)sampleTime)/1000; 

   Kp = p;
   Ki = i;
   Kd = d;
   Kf = f;
 
  if(controllerDirection == REVERSE)
   {
      Kp = (0 - Kp);
      Ki = (0 - Ki);
      Kd = (0 - Kd);
      Kf = (0 - Kf);
   }
}

void PID::SetControllerDirection(int direction){ 
	if(direction == 0 || direction == 1)
		controllerDirection = direction; 
	if(controllerDirection == REVERSE){
		Kp = 0 - Kp;
		Ki = 0 - Ki;
		Kd = 0 - Kd;
                Kf = 0 - Kf;
	}
}
void PID::SetSampleTime(int t){
	if(t > 0){
		double ratio = (double)t / (double) sampleTime;

		Ki *= ratio;
		Kd /= ratio;

		sampleTime = (unsigned long) t;

	}
}

//accessor functions
double PID::getKp(){ return Kp; }

double PID::getKi(){ return Ki; }

double PID::getKd(){ return Kd; }

int PID::getDirection(){ return controllerDirection; }



PID_inputFilter::PID_inputFilter(double d)
{
      old_factor = 1 - d;
      integral = 0;
}

PID_inputFilter::~PID_inputFilter(void){
}

void PID_inputFilter::add(double d){
        integral = integral * old_factor + d;
}

double PID_inputFilter::computeIntegral(){
        return integral;
}

