//main plane program

//http://students.sae.org/cds/aerodesign/rules/2015_aero_rules.pdf

#include "plane.h" //all definitions

//includes
#include <Wire.h>
#include "Servo.h"
#include "MPU6050_6Axis_MotionApps20.h"

//hardware includes
#include "Communicator.h"
#include "EagleTreeAltimeterV4.h"

//ROLL PITCH AND YAW CONTROLL
//SDA pin 4
//SCL pin 5
MPU6050 mpu;
#define OUTPUT_READABLE_mpu
int MPU6050status;

//MPU6050 stuff
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
uint8_t fifoBackup[64]; //backup buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

double current_pitch, current_roll;
double base_pitch, base_roll;

//roll and pitch filters - these are used only to zero the sensor, not for PIDs!
//filtering in the feedback loop of a control system adds unwanted dynamics and delay - causes instability!
IIR_doubleFilter *roll_filter = new IIR_doubleFilter(0.1);
IIR_doubleFilter *pitch_filter = new IIR_doubleFilter(0.1);

//system variables
double altitude;
double altitude_at_drop;
byte blinkState;

//Servo declarations
Servo wheel_servo;
volatile int pwm_value[ ] = {0,0};
volatile int prev_time[ ] = {0,0};

//this is the best way to declare objects! 
//other ways work but are not robust... sometimes they fail because Arduino compiler is stupid
//this method does not call the constructor 
//instead, you have to call an initialize() methode later on
Communicator comm;

//sensor declerations
EagleTreeAltimeterV4 altimeter;

unsigned long current_time;
unsigned long prev_slow_time, prev_medium_time, prev_fast_time, prev_long_time;

//interupt routine
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void setup(){
  Serial.begin(57600);  // tested to be about 0.17ms to print one double
                        //9600 was 1.02ms
                        //115200 was 0.084ms - this is very fast, if there is interference could get errors
  
  //start up serial communicator                      
  comm.initialize();
  comm.sendMessage(MESSAGE_START);

  //attach servos that are controlled directly by communicator
  comm.attachDropBay(DROP_PIN);

  //initialize Data Acquisition System (which also preforms a DAS reset)
  initializeDAS();

 

  //setup hardware that is controlled directly in the main loop
  blinkState = 0;
  pinMode(STATUS_LED_PIN, OUTPUT);

  initializeServos();

  //start system time
  prev_fast_time = millis();
  prev_medium_time = prev_fast_time;
  prev_slow_time = prev_fast_time;
  prev_long_time = prev_fast_time;
  
  //send message to ground station saying everything is ready
  comm.sendMessage(MESSAGE_READY);
}

void loop(){

  current_time = millis();

  unsigned long fast_time_diff = current_time - prev_fast_time;
  unsigned long medium_time_diff = current_time - prev_medium_time;
  unsigned long slow_time_diff = current_time - prev_slow_time;
  unsigned long long_time_diff = current_time - prev_long_time;

  
  //call loop functions in slow->fast order
  //this way, new commands are received in slow loop and implemented in faster loops
  if(long_time_diff > LONG_LOOP_TIME){
    prev_long_time = current_time;
    longLoop();
  }  
  if(slow_time_diff > SLOW_LOOP_TIME){
    prev_slow_time = current_time;
    slowLoop();
  }  
  if(medium_time_diff > MEDIUM_LOOP_TIME ){
    prev_medium_time = current_time;
    mediumLoop();
  }
  
  if(fast_time_diff > FAST_LOOP_TIME){
    prev_fast_time = current_time;
    fastLoop();
  }

}

void fastLoop(){
  
  //update current angle
  //unsigned long t1 = micros();  //used for testing timing

  //reading angles take 5.7ms.
  if(mpuInterrupt || fifoCount > packetSize){
        // reset interrupt flag and get INT_STATUS byte
        mpuInterrupt = false;
        mpuIntStatus = mpu.getIntStatus();
    
        // get current FIFO count
        fifoCount = mpu.getFIFOCount();
    
        // check for overflow
        if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
            // reset so we can continue cleanly
            mpu.resetFIFO();
            //Serial.println(F("FIFO overflow!"));
    
        // otherwise, check for DMP data ready interrupt (this should happen frequently)
        } else if (mpuIntStatus & 0x02) {
            // wait for correct available data length, should be a VERY short wait
            while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    
            // read a packet from FIFO
            mpu.getFIFOBytes(fifoBuffer, packetSize);
            
            // track FIFO count here in case there is > 1 packet available
            // (this lets us immediately read more without waiting for an interrupt)
            fifoCount -= packetSize;
            
            // calculate yaw pitch and roll
            mpu.dmpGetQuaternion(&q, fifoBuffer);        //get measured angle (in Quaternions) from buffer 
            mpu.dmpGetGravity(&gravity, &q);             //compute gravity vector from quaternion mesurment
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);   //with that information, compute yaw pitch and roll
    
    
            double pitch = ypr[1] * 180 / 3.14159;
            double roll = ypr[2] * 180 / 3.14159;
    
            if(!isnan(pitch) && !isnan(roll)){  
              roll_filter->addValue(roll);
              pitch_filter->addValue(pitch);
              current_pitch = pitch - base_pitch;
              current_roll = roll - base_roll;      
            }
        }

    }    
}

void mediumLoop(){
  
  //De-mixing of signal for tail wheel
  int signal_wheel;
  
  if ((pwm_value[0] != 0) && (pwm_value[1] != 0)){
    signal_wheel = tail_wheel_demixing();
    wheel_servo.write(signal_wheel);
  }
  delay(10);
  
}

//preforme serial communication in the slow loop - this needs to hapen less often
//slow loop was timed to take between 1.3 and 1.7ms.
void slowLoop(){
  //unsigned long t1 = micros(); //for testing timing
 //get latest telementary data 
  altitude = altimeter.readAltitude();
  
   //pass new data to serial communicator
   
  comm.roll = roll_filter->getCurrentValue() - base_roll;
  comm.pitch = pitch_filter->getCurrentValue() - base_pitch;
  comm.altitude = altitude;
  
  //send the next set of data
  //only one variable is sent with each call - 4 calls necessary to send everything
  #ifndef CALIBRATEING_DX6
      comm.sendData();  
  #endif
  
  //receive commands from ground station
  //these are caried out inside communicator class
  comm.recieveCommands();

  //check for reset flag
  if(comm.reset == true){
     comm.reset = false;
     resetDAS();
     comm.sendMessage(MESSAGE_RESET_AKN);
  }
  
  //check for restart flag
  if(comm.restart == true){
       comm.restart = false;
       
       resetDAS();
       
       //restart servos, PIDs, everything!
    
       comm.sendMessage(MESSAGE_RESTART_AKN);
  } 
  while(comm.calibration_flag){     
      comm.calibrate();
  }
  /*
  unsigned long t2 = micros(); //for testing timing
  
  Serial.print("  ");
  Serial.println(t2 - t1);
  */
}

void longLoop(){
  blinkState = !blinkState;
  digitalWrite(STATUS_LED_PIN, blinkState);  
}

int tail_wheel_demixing(){
  int signal_wheel;
  int left_signal = pwm_value[0]-100; //100 offset. may be subject to change
  int right_signal = pwm_value[1]-100; //100 offset. may be subect to change
  
//  Switch to true if the servos are installed in the opposite direction
  boolean flip_left_signal = false;
  boolean flip_right_signal = false;
  
  if (flip_left_signal == true)
    left_signal = map(left_signal,1000,2000,2000,1000);
  if (flip_right_signal == true)
    right_signal = map(right_signal,1000,2000,2000,1000);
  
//  int left_signal = pwm_value[0]-100; //100 offset. may be subject to change
//  int right_signal = pwm_value[1]-100; //100 offset. may be subect to change
    
//  Use following code to make sure servo is straight. Should obtain 1500 for both left and right signals at rest.
//  SerialUSB.print(left_signal); //uncomment to show values to know how much offset is needed
//  SerialUSB.print(" ");
//  SerialUSB.println(right_signal);
  
  signal_wheel = left_signal + right_signal;
  signal_wheel = constrain(map(signal_wheel,2000,4000,0,255),0,255);
  
  return(signal_wheel);
}

//initialize servo locations
void initializeServos(){
  //De-mixing of signal to tail wheel
  pinMode(ELEVATOR_INPUT_PIN, INPUT); 
  pinMode(RUDDER_INPUT_PIN, INPUT); 
  
  attachInterrupt(ELEVATOR_INPUT_PIN, rising_elevator, RISING);
  attachInterrupt(RUDDER_INPUT_PIN, rising_rudder, RISING);

  wheel_servo.attach(TAIL_WHEEL_PIN);
}

void initializeDAS(){
  Wire.begin();
  //initialize sensors
  
  initializeMPU6050();
  
   //Serial.println("here");
  altimeter.initialize();
   //Serial.println("here1");
  //preforme DAS reset
  resetDAS();
   //Serial.println("here2");
  
  base_pitch = 0;
  base_roll = 0;  
}

//function to reset Data Acquisition System when resent command is sent via serial
void resetDAS(){
  //turn off LED when reseting DAS
  //when it resumes blinking, we know the reset has finished
  digitalWrite(STATUS_LED_PIN, LOW);
  
  //call individual reset functions for all the sensors
  altimeter.zero();
  
  base_pitch = pitch_filter->getCurrentValue();
  base_roll = roll_filter->getCurrentValue();
  
}


void initializeMPU6050(){
	//MPU6050
	#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
		Wire.begin();
	#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
		Fastwire::setup(400, true);
	#endif

	// initialize device
        
	//Serial.println("Initializing I2C devices...");
        comm.sendMessage(MPU6050_INITIALIZING);

	mpu.initialize();

        // verify connection
        //Serial.println(F("Testing device connections..."));
        comm.sendMessage(mpu.testConnection() ? MPU6050_READY : MPU6050_FAILED);
        
        // load and configure the DMP
        //Serial.println(F("Initializing DMP..."));
        devStatus = mpu.dmpInitialize();
        //DMP = digital motion processor!
        
        //set offsets
        /*
        mpu.setXGyroOffset(220);
        mpu.setYGyroOffset(76);
        mpu.setZGyroOffset(-85);
        mpu.setXAccelOffset(548);
        mpu.setYAccelOffset(-108);
        mpu.setZAccelOffset(1480);        
*/
/*
        mpu.setXAccelOffset(10);
        mpu.setYAccelOffset(-180);
        mpu.setZAccelOffset(1480);  
*/
      // make sure it worked (returns 0 if so)
      if (devStatus == 0) {
          // turn on the DMP, now that it's ready        
          //Serial.println(F("Enabling DMP..."));
          mpu.setDMPEnabled(true);
  
          // enable Arduino interrupt detection
          //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
          attachInterrupt(52, dmpDataReady, RISING);
          mpuIntStatus = mpu.getIntStatus();
  
          // set our DMP Ready flag so the main loop() function knows it's okay to use it
          //Serial.println(F("DMP ready! Waiting for first interrupt..."));
          dmpReady = true;
          comm.sendMessage(MPU6050_DMP_READY);
  
          // get expected DMP packet size for later comparison
          packetSize = mpu.dmpGetFIFOPacketSize();
      } else {
          // ERROR!
          comm.sendMessage(MPU6050_DMP_FAILED);
          // 1 = initial memory load failed
          // 2 = DMP configuration updates failed
          // (if it's going to break, usually the code will be 1)
          //Serial.print(F("DMP Initialization failed (code "));
          //Serial.print(devStatus);
          //Serial.println(F(")"));
      }
}
 
 //PWM STUFF has to stay in main page because Arduino...

//Interrupt routines for de-mixing of signal. Error checking to ensure the signal is valid.

//interupt service routine called on the rising edge of the pulse
void rising_elevator()
{
  //record current time.  this is the rise time
  prev_time[0] = micros();
  
    //calculate the time since the last falling edge was detected
    //only count this as a valid rising edge if time difference is more than 1000 microseconds
    if(prev_time[0] > 1000){
       //if it is a valid rising edge, set the interupt for the falling edge on the same pin
       attachInterrupt(ELEVATOR_INPUT_PIN, falling_elevator, FALLING);
    }
    
  //SerialUSB.println("rising elevator"); //For testing purposes only
}

//interrupt service routine called on the falling edge of the pulse
void falling_elevator() {
  //record current time, this is the fall time of the pulse
  pwm_value[0] = micros()-prev_time[0];
  
  //calculate the time difference between the rising and falling edge
  //only a valid pulse if the time difference is between 400 and 2500 microseconds
  if (pwm_value[0] >= 400 && pwm_value[0] <= 2500){
    attachInterrupt(ELEVATOR_INPUT_PIN, rising_elevator, RISING);
  }
  
  //SerialUSB.println(pwm_value[0]);//For testing purposes only
}

//interupt service routine called on the rising edge of the pulse
void rising_rudder()
{
  //record current time.  this is the rise time
  prev_time[1] = micros();
  
  //calculate the time since the last falling edge was detected
  //only count this as a valid rising edge if time difference is more than 1000 microseconds
  if(prev_time[1] >= 1000){
    //if it is a valid rising edge, set the interupt for the falling edge on the same pin
    attachInterrupt(RUDDER_INPUT_PIN, falling_rudder, FALLING);
  }
  
  //SerialUSB.println("rising rudder"); //For testing purposes only
}

//interrupt service routine called on the falling edge of the pulse
void falling_rudder() {
  //record current time, this is the fall time of the pulse
  pwm_value[1] = micros()-prev_time[1];
  
  //calculate the time difference between the rising and falling edge
  //only a valid pulse if the time difference is between 400 and 2500 microseconds
  if (pwm_value[1] >= 400 && pwm_value[1] <= 2500){
    attachInterrupt(RUDDER_INPUT_PIN, rising_rudder, RISING);
  }
  
  //SerialUSB.println(pwm_value[1]);//For testing purposes only
}
