
//main plane program

//http://students.sae.org/cds/aerodesign/rules/2015_aero_rules.pdf

#include "plane.h" //all definitions

//includes
//#include <Wire.h>
#include "Servo.h"

//hardware includes
#include "Communicator.h"
#include "Adafruit_MPL3115A2.h"
//#include "SparkFunMPL3115A2.h"
#include "filter.h"


double current_pitch, current_roll;
double base_pitch, base_roll;

//roll and pitch filters - these are used only to zero the sensor, not for PIDs!
//filtering in the feedback loop of a control system adds unwanted dynamics and delay - causes instability!
IIR_doubleFilter *roll_filter = new IIR_doubleFilter(0.1);
IIR_doubleFilter *pitch_filter = new IIR_doubleFilter(0.1);

// Altitude
double altitude;
//#define numberOfAltitudeReadsBeforeZero 2
//byte numberOfAltitudeReadsPassed = 0;
boolean didGetZero = false;

//system variables
byte blinkState;

//Servo declarations
Servo wheel_servo, left_aileron_servo, right_aileron_servo, left_Vtail_servo, right_Vtail_servo;
//#define PID_MODE_INPUT 7   #define PITCH_INPUT 8   #define ROLL_INPUT 9  #define FLAP_MODE_INPUT 11   #define YAW_INPUT 12
volatile int pwm_high_time[5];  //In the order of: PID, pitch, roll, flap, yaw
volatile unsigned long pwm_high_start[5];
volatile unsigned long pwm_high_end[5];



//this is the best way to declare objects!
//other ways work but are not robust... sometimes they fail because Arduino compiler is stupid
//this method does not call the constructor
//instead, you have to call an initialize() methode later on
Communicator comm;

//sensor declerations
Adafruit_MPL3115A2 altimeter = Adafruit_MPL3115A2();
//MPL3115A2 altimeter;

unsigned long current_time;
unsigned long prev_slow_time, prev_medium_time, prev_long_time;




void setup() {

  //do this first so servos are good regardless
  initializeServos();


  //start up serial communicator
  delay(3000);  //give XBee time to start
  SerialUSB.begin(115200);
  //Wire.begin();

  comm.initialize();

  comm.sendMessage(MESSAGE_START);

  //attach servos that are controlled directly by communicator
  comm.attachDropBay(DROP_PIN);

  //initialize Data Acquisition System (which also preforms a DAS reset)
  initializeDAS();

  //setup hardware that is controlled directly in the main loop
  blinkState = 0;
  pinMode(STATUS_LED_PIN, OUTPUT);


  //start system time
  prev_medium_time = millis();
  prev_slow_time = prev_medium_time;
  prev_long_time = prev_medium_time;

  //send message to ground station saying everything is ready
  comm.sendMessage(MESSAGE_READY);
}

void loop() {

  current_time = millis();

  unsigned long medium_time_diff = current_time - prev_medium_time;
  unsigned long slow_time_diff = current_time - prev_slow_time;
  unsigned long long_time_diff = current_time - prev_long_time;


  //call loop functions in slow->fast order
  //this way, new commands are received in slow loop and implemented in faster loops
  if (long_time_diff > LONG_LOOP_TIME) {
    prev_long_time = current_time;
    longLoop();
  }
  if (slow_time_diff > SLOW_LOOP_TIME) {
    prev_slow_time = current_time;
    slowLoop();
  }
  if (medium_time_diff > MEDIUM_LOOP_TIME ) {
    prev_medium_time = current_time;
    mediumLoop();
  }



  //check if commands received. This function executes quickly even if a command is received
  comm.recieveCommands();

  //check if incoming data from GPS. If a full string is received, this function automatically parses it. Shouldn't take >1ms even when parsing required (which is 5x per second)
  comm.getSerialDataFromGPS();

}


//TODO: figure out which pwm_high_time indexes correspond to which signals. Also add any offsets or negative values (if not doing that on controller)
void mediumLoop() {

  //these lines write the values obtained through isr for each PWM signal
  // 0 = Left or right aileron
  // 1 = Left or right aileron
  // 2 = Mixed
  // 3 =


  //wheel servo (requires demixing the two V tail signals (note: these two should also be used below for left and right VTail
  if ((pwm_high_time[0] != 0) && (pwm_high_time[1] != 0))
    wheel_servo.writeMicroseconds(tail_wheel_demixing(3, 2)); //tail_wheel_demixing arguements are leftPin, rightPin.  returns value to write to servo

  //left Vtail
  if (pwm_high_time[0] != 0)
    left_Vtail_servo.writeMicroseconds(pwm_high_time[0]);

  //right Vtail
  if (pwm_high_time[2] != 0)
    right_Vtail_servo.writeMicroseconds(pwm_high_time[2]);

  //left Aileron  (actually right vtail)
  if (pwm_high_time[1] != 0)
    left_aileron_servo.writeMicroseconds(pwm_high_time[1]);

  //right Aileron
  if (pwm_high_time[3] != 0)
    right_aileron_servo.writeMicroseconds(pwm_high_time[3]);

  /*
    SerialUSB.print(tail_wheel_demixing(2,3));
    SerialUSB.print('\t');
    SerialUSB.print(pwm_high_time[0]);
    SerialUSB.print('\t');
    SerialUSB.print(pwm_high_time[1]);
    SerialUSB.print('\t');
    SerialUSB.print(pwm_high_time[2]);
    SerialUSB.print('\t');
    SerialUSB.print(pwm_high_time[3]);
    SerialUSB.println('\t');
    //SerialUSB.print(pwm_high_time[4]);
    //SerialUSB.println('\t');  */

}

//preforme serial communication in the slow loop - this needs to hapen less often
//slow loop was timed to take between 1.3 and 1.7ms.
void slowLoop() {
  //unsigned long loopStartTime = millis(); //for testing timing
  //get latest telementary data
  double altitudeReadIn = altimeter.getAltitude(false);
  
  if (altitudeReadIn > -990) {

  //SerialUSB.print("Raw in: ");
  //SerialUSB.println(altitudeReadIn);
    
    if (!didGetZero) {
      didGetZero = true;
      altimeter.zero();
    }
    else{
      //if(numberOfAltitudeReadsPassed >= numberOfAltitudeReadsBeforeZero){
        altitude = altitudeReadIn;
      //}
      //numberOfAltitudeReadsPassed++;
    }

  }

  //SerialUSB.print("Altitude: ");
  //SerialUSB.println(altitude);

  //altitude = altimeter.readAltitudeFt();
  //unsigned long loopEndTime = millis(); //for testing timing
  

  //pass new data to serial communicator
  comm.roll = roll_filter->getCurrentValue() - base_roll;
  comm.pitch = pitch_filter->getCurrentValue() - base_pitch;
  comm.altitude = altitude;

  //send data to XBee
  comm.sendData();


  //check for reset flag
  if (comm.reset == true) {
    comm.reset = false;
    resetDAS();
    comm.sendMessage(MESSAGE_RESET_AKN);
  }

  //check for restart flag
  if (comm.restart == true) {
    comm.restart = false;

    resetDAS();

    //restart servos, PIDs, everything!

    comm.sendMessage(MESSAGE_RESTART_AKN);
  }



  //SerialUSB.print("Reading time: ");
  //SerialUSB.println(loopEndTime - loopStartTime);

}

void longLoop() {
  blinkState = !blinkState;
  digitalWrite(STATUS_LED_PIN, blinkState);
}

//TODO: which pwm_high_time values correspond to the left/right signals? (in first two lines in function)
int tail_wheel_demixing(int leftPin, int rightPin) {

  int left_signal = pwm_high_time[leftPin] - LEFT_SIGNAL_OFFSET; //currently defined as 0 offset. may be subject to change
  int right_signal = pwm_high_time[rightPin] - RIGHT_SIGNAL_OFFSET; //same as above


  //These constants are from plane.h -> switch to true if required based on servo. WILL THIS AFFECT THE OFFSET SIGN? (ie. should it be done after?)
  if (FLIP_LEFT_SIGNAL)
    left_signal = map(left_signal, 1000, 2000, 2000, 1000);
  if (FLIP_RIGHT_SIGNAL)
    right_signal = map(right_signal, 1000, 2000, 2000, 1000);


  //Use following code to make sure servo is straight. Should obtain 1500 for both left and right signals at rest.
  //  SerialUSB.print(left_signal);   SerialUSB.print(" ");  SerialUSB.println(right_signal);


  return constrain(map((left_signal + right_signal) / 2, 1000, 2000, 2000, 1000), 1000, 2000); //average left and right values, flip them, then constrain to between 1000-2000

  // was the below beofre, which returns a degree position instead of microsecond value
  //return constrain(map(left_signal + right_signal,2000,4000,0,255),0,255);
}


//initialize servo locations
void initializeServos() {
  //set pin modes for incoming PWM signals
  pinMode(PID_MODE_INPUT, INPUT);
  pinMode(FLAP_MODE_INPUT, INPUT);
  pinMode(ROLL_INPUT, INPUT);
  pinMode(YAW_INPUT, INPUT);
  pinMode(PITCH_INPUT, INPUT);

  //attach each of the servos (before the interrupts to make sure it's initialized properly).
  wheel_servo.attach(RUDDER_OUT_PIN);
  left_aileron_servo.attach(L_AILERON_OUT_PIN);
  right_aileron_servo.attach(R_AILERON_OUT_PIN);
  left_Vtail_servo.attach(L_TAIL_OUT_PIN);
  right_Vtail_servo.attach(R_TAIL_OUT_PIN);

  //apply 1500us neutral position (if drop bay is ever here, this neutral would likely no be ok for it
  wheel_servo.writeMicroseconds(1500);
  left_aileron_servo.writeMicroseconds(1500);
  right_aileron_servo.writeMicroseconds(1500);
  left_Vtail_servo.writeMicroseconds(1500);
  right_Vtail_servo.writeMicroseconds(1500);

  //attach interrupts on the rising edge of each input signal
  attachInterrupt(PID_MODE_INPUT, isr_rising_pid, RISING);
  attachInterrupt(FLAP_MODE_INPUT, isr_rising_flapMode, RISING);
  attachInterrupt(ROLL_INPUT, isr_rising_roll, RISING);
  attachInterrupt(YAW_INPUT, isr_rising_yaw, RISING);
  attachInterrupt(PITCH_INPUT, isr_rising_pitch, RISING);


}

void initializeDAS() {
  //Wire.begin();
  //initialize sensors

  altimeter.begin();
  altimeter.setReadTimeout(10);

  //altimeter.begin(); // Get sensor online
  //altimeter.setOversampleRate(7);

  //Configure the sensor
  //altimeter.setModeAltimeter(); // Measure altitude above sea level in meters
  //altimeter.setOversampleRate(7); // Set Oversample to the recommended 128
  //altimeter.enableEventFlags(); // Enable all three pressure and temp event flags

  //preforme DAS reset
  resetDAS();

  base_pitch = 0;
  base_roll = 0;
}

//function to reset Data Acquisition System when resent command is sent via serial
void resetDAS() {
  //turn off LED when reseting DAS
  //when it resumes blinking, we know the reset has finished
  digitalWrite(STATUS_LED_PIN, LOW);

  //call individual reset functions for all the sensors

  base_pitch = pitch_filter->getCurrentValue();
  base_roll = roll_filter->getCurrentValue();
  //GPS doesn't retain status information (so don't need to reset). If we filter heading/speed in the future will need to do this

}



/*6 input signals as labelled on PCB: PID_MODE_INPUT 7, FLAP_MODE_INPUT 11, ROLL_INPUT 9, YAW_INPUT 12, PITCH_INPUT 8
  The mappings between the labelled inputs to outputs are:
  PID_MODE_INPUT  ->
  FLAP_MODE_INPUT ->
  ROLL_INPUT ->
  YAW_INPUT ->
  PITCH_INPUT ->

  NOTE: one of these is not needed (there are 4 outputs directly from these signals, the throttle is shorted, tail wheel comes from demixing L/R Tail signals, 1 is unused and 1 is for dropbay
    controlled in communicator class.  This brings us to the total of 8 output slots on PCB.

  Only the first set is commented, and all follow those same comments with a different array index
*/

void isr_rising_pid()
{
  //find current time in microseconds
  pwm_high_start[0] = micros();

  //find difference between this rising edge and last falling edge
  int time_difference = pwm_high_start[0] - pwm_high_end[0];

  //if it's been long enough since last falling edge (ie. isn't just debouncing) then attach falling interrupt
  if (time_difference > 1000)
    attachInterrupt(PID_MODE_INPUT, isr_falling_pid, FALLING);



}

void isr_falling_pid()
{
  //find current time in microseconds
  pwm_high_end[0] = micros();

  //find how long it was high for (in us)
  int time_difference = pwm_high_end[0] - pwm_high_start[0];

  //if it was high for correct amount of time (in range of servo commands) then save value
  if (time_difference >= 400 && time_difference <= 2500)
    pwm_high_time[0] = time_difference;

  //regardless if time ok, attach rising interrupt, to get next duty cycle
  attachInterrupt(PID_MODE_INPUT, isr_rising_pid, RISING);

}

void isr_rising_flapMode()
{
  pwm_high_start[1] = micros();

  int time_difference = pwm_high_start[1] - pwm_high_end[1];

  if (time_difference > 1000)
    attachInterrupt(FLAP_MODE_INPUT, isr_falling_flapMode, FALLING);


}

void isr_falling_flapMode()
{
  pwm_high_end[1] = micros();

  int time_difference = pwm_high_end[1] - pwm_high_start[1];

  if (time_difference >= 400 && time_difference <= 2500)
    pwm_high_time[1] = time_difference;

  attachInterrupt(FLAP_MODE_INPUT, isr_rising_flapMode, RISING);

}

void isr_rising_roll()
{
  pwm_high_start[2] = micros();

  int time_difference = pwm_high_start[2] - pwm_high_end[2];

  if (time_difference > 1000)
    attachInterrupt(ROLL_INPUT, isr_falling_roll, FALLING);

}

void isr_falling_roll()
{
  pwm_high_end[2] = micros();

  int time_difference = pwm_high_end[2] - pwm_high_start[2];

  if (time_difference >= 400 && time_difference <= 2500)
    pwm_high_time[2] = time_difference;

  attachInterrupt(ROLL_INPUT, isr_rising_roll, RISING);

}

void isr_rising_yaw()
{
  pwm_high_start[3] = micros();

  int time_difference = pwm_high_start[3] - pwm_high_end[3];

  if (time_difference > 1000)
    attachInterrupt(YAW_INPUT, isr_falling_yaw, FALLING);


}

void isr_falling_yaw()
{
  pwm_high_end[3] = micros();

  int time_difference = pwm_high_end[3] - pwm_high_start[3];

  if (time_difference >= 400 && time_difference <= 2500)
    pwm_high_time[3] = time_difference;

  attachInterrupt(YAW_INPUT, isr_rising_yaw, RISING);

}

void isr_rising_pitch()
{
  pwm_high_start[4] = micros();

  int time_difference = pwm_high_start[4] - pwm_high_end[4];

  if (time_difference > 1000)
    attachInterrupt(PITCH_INPUT, isr_falling_pitch, FALLING);


}

void isr_falling_pitch()
{
  pwm_high_end[4] = micros();

  int time_difference = pwm_high_end[4] - pwm_high_start[4];

  if (time_difference >= 400 && time_difference <= 2500)
    pwm_high_time[4] = time_difference;

  attachInterrupt(PITCH_INPUT, isr_rising_pitch, RISING);

}





//Interrupt routines for de-mixing of signal. Error checking to ensure the signal is valid.
/* These are encorporated into the above isr's
  //interupt service routine called on the rising edge of the pulse
  void isr_rising_elevator()
  {
  //record current time.  this is the rise time
  prev_time[0] = micros();

    //calculate the time since the last falling edge was detected
    //only count this as a valid rising edge if time difference is more than 1000 microseconds
    if(prev_time[0] > 1000){
       //if it is a valid rising edge, set the interupt for the falling edge on the same pin
       attachInterrupt(ELEVATOR_INPUT_PIN, isr_falling_elevator, FALLING);
    }

  }

  //interrupt service routine called on the falling edge of the pulse
  void isr_falling_elevator() {
  //record current time, this is the fall time of the pulse
  pwm_value[0] = micros()-prev_time[0];

  //calculate the time difference between the rising and falling edge
  //only a valid pulse if the time difference is between 400 and 2500 microseconds
  if (pwm_value[0] >= 400 && pwm_value[0] <= 2500){
    attachInterrupt(ELEVATOR_INPUT_PIN, isr_rising_elevator, RISING);
  }

  }

  //interupt service routine called on the rising edge of the pulse
  void isr_rising_rudder()
  {
  //record current time.  this is the rise time
  prev_time[1] = micros();

  //calculate the time since the last falling edge was detected
  //only count this as a valid rising edge if time difference is more than 1000 microseconds
  if(prev_time[1] >= 1000){
    //if it is a valid rising edge, set the interupt for the falling edge on the same pin
    attachInterrupt(RUDDER_INPUT_PIN, isr_falling_rudder, FALLING);
  }

  }

  //interrupt service routine called on the falling edge of the pulse
  void isr_falling_rudder() {
  //record current time, this is the fall time of the pulse
  pwm_value[1] = micros()-prev_time[1];

  //calculate the time difference between the rising and falling edge
  //only a valid pulse if the time difference is between 400 and 2500 microseconds
  if (pwm_value[1] >= 400 && pwm_value[1] <= 2500){
    attachInterrupt(RUDDER_INPUT_PIN, isr_rising_rudder, RISING);
  }

  }   */
  
