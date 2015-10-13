//main plane program

//http://students.sae.org/cds/aerodesign/rules/2015_aero_rules.pdf

#include "plane.h" //all definitions

//includes
#include <Wire.h>
#include "Servo.h"
#include "MPU6050_6Axis_MotionApps20.h"

//hardware includes
#include "AirspeedSensor.h"
#include "Communicator.h"
#include "DX6.h"
#include "EagleTreeAltimeterV4.h"

//PWM interupt stuff
#define NUM_PWM 4
int PWM_pin[NUM_PWM] = {AILERON_INPUT_PIN, ELEVATOR_INPUT_PIN, PID_MODE_PIN, AILERON_MODE_PIN}; //roll, pitch, PID mode, flap mode
//array of data sturctures (on for each pin)
volatile static pinTimingData pinData[NUM_PWM];

//define sensors and measurment classes
DX6 RC_receiver;

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

//variables used by PIDs
double target_pitch, target_roll;
double current_pitch, current_roll;
double offset_pitch, offset_roll;
double base_pitch, base_roll;

//roll and pitch filters - these are used only to zero the sensor, not for PIDs!
//filtering in the feedback loop of a control system adds unwanted dynamics and delay - causes instability!
IIR_doubleFilter *roll_filter = new IIR_doubleFilter(0.1);
IIR_doubleFilter *pitch_filter = new IIR_doubleFilter(0.1);

//XXX Calibrate - DONE!
int elevator_base_val = 1450;
int right_aileron_base_val = 1500;
int left_aileron_base_val = 1500;

//arduino-controlled servos
Servo right_aileron_servo, left_aileron_servo, elevator_servo;

//system variables
double altitude;
double altitude_at_drop;
double airspeed;
byte blinkState;


//this is the best way to declare objects! 
//other ways work but are not robust... sometimes they fail because Arduino compiler is stupid
//this method does not call the constructor 
//instead, you have to call an initialize() methode later on
Communicator comm;

//sensor declerations
AirspeedSensor airspeedSensor;
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
  comm.attachCamera(CAMERA_PAN_PIN, CAMERA_TILT_PIN);
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
  
  initializeDX6();
  
  //send message to gournd station saying everything is ready
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

  //update servo positions
  //run PIDs
  //read DX6 for new target pitch and roll
  RC_receiver.readPWM();
  target_roll = (int) RC_receiver.getRoll();  //DX6's roll is opposite to MPU6050
  target_pitch = (int) RC_receiver.getPitch();
  
  
  //unsigned long t2 = micros();  //used for testing timing
  //Serial.print("  ");
  //Serial.println(t2 - t1);
  
}

void mediumLoop(){
  updateServos();
}

//preforme serial communication in the slow loop - this needs to hapen less often
//slow loop was timed to take between 1.3 and 1.7ms.
void slowLoop(){
  //unsigned long t1 = micros(); //for testing timing
 //get latest telementary data 
  airspeed = airspeedSensor.readAirSpeed(); 
  altitude = altimeter.readAltitude();
  
   //pass new data to serial communicator
   
  comm.roll = roll_filter->getCurrentValue() - base_roll;
  comm.pitch = pitch_filter->getCurrentValue() - base_pitch;
  comm.altitude = altitude;
  comm.airspeed = airspeed;   
  
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
      //write latest base values to servos
      elevator_base_val = comm.ElevatorVal_cal;
      right_aileron_base_val = comm.RightAileronVal_cal;
      left_aileron_base_val = comm.LeftAileronVal_cal;
      
      elevator_servo.writeMicroseconds(elevator_base_val);
      right_aileron_servo.writeMicroseconds(right_aileron_base_val);
      left_aileron_servo.writeMicroseconds(left_aileron_base_val);  
      
      comm.calibrate();
      //get new values
      elevator_base_val = comm.ElevatorVal_cal;
      right_aileron_base_val = comm.RightAileronVal_cal;
      left_aileron_base_val = comm.LeftAileronVal_cal;
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


//initialize servo locations
void initializeServos(){
  right_aileron_servo.attach(RIGHT_AILERON_SERVO_PIN);
  left_aileron_servo.attach(LEFT_AILERON_SERVO_PIN);
  elevator_servo.attach(ELEVATOR_SERVO_PIN);
}

//function to update servo positions
void updateServos(){
  
  //XXX Calibrate - DONE!
  if(RC_receiver.getPIDmode() == 2){ //PID_MODE_ON
    
  }
  else{
      //PID mode off
      //check flap mode
      if(RC_receiver.getAileronMode() == AILERON_NORMAL_MODE){
          right_aileron_servo.writeMicroseconds(right_aileron_base_val + target_roll); //calibrate +/-
          left_aileron_servo.writeMicroseconds(left_aileron_base_val - target_roll);  //calibrate +/-
      }
      else{
          right_aileron_servo.writeMicroseconds(right_aileron_base_val + target_roll - RIGHT_AILERON_FLAP_VAL);  //calibrate +/-
          left_aileron_servo.writeMicroseconds(left_aileron_base_val - target_roll - LEFT_AILERON_FLAP_VAL);    //calibrate +/-    
      }
      elevator_servo.writeMicroseconds(elevator_base_val - target_pitch);
    
  }
  
}

void initializeDAS(){
  Wire.begin();
  //initialize sensors
  pinMode(AIRSPEED_SENSOR_PIN, INPUT);
  airspeedSensor.initialize();  
  
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
  airspeedSensor.reset();
  
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

//function to initialize the DX6
void initializeDX6(){
         //initialize interupt stuff first 
        //start with pin 0

         //initialize data structure for each pin
        for(int i = 0; i < NUM_PWM; i++){
              pinData[i].riseTime = 0;
              pinData[i].fallTime = 0;
              pinData[i].lastGoodWidth = 0;
        }
        

	//define and calibrate DX6 controler
        //                     Roll                        Pitch                      PID Mode                  Aileron Flap Mode
	RC_receiver.initialize(&pinData[0].lastGoodWidth, &pinData[1].lastGoodWidth, &pinData[2].lastGoodWidth, &pinData[3].lastGoodWidth);

        //XXX Calibrate - DONE!
        //set min, zero, max readings for roll and pitch
	RC_receiver.calibrateRoll(1099,1494,1917);
	RC_receiver.calibratePitch(1096,1501,1918);

        //XXX Calibrate - DONE!
        //set the input roll and pitch ranges
        //these values reflect the maximum allowed roll and pich of the aircraft
	RC_receiver.setRollRange(-500, 500); //calibrate - these are servo angles in PWM
	RC_receiver.setPitchRange(-500, 500);//calibrate - these are servo angles in PWM
  
      //start interupt for first pin
      attachInterrupt(PWM_pin[0], PWM_RISING_ISR_0, RISING); 
      attachInterrupt(PWM_pin[1], PWM_RISING_ISR_1, RISING);  
      attachInterrupt(PWM_pin[2], PWM_RISING_ISR_2, RISING); 
      attachInterrupt(PWM_pin[3], PWM_RISING_ISR_3, RISING);  

}
 
 //PWM STUFF has to stay in main page cause Arduino sucks
//interupt service routine called on the rising edge of the pulse
void PWM_RISING_ISR_0(){
    //record current time.  this is the rise time
    pinData[0].riseTime = micros();
    //calculate the time sice the last falling edge was detected
    //only count this as a valid rising edge if time difference is more than 1000 microseconds
    unsigned long time_difference=  pinData[0].riseTime - pinData[0].fallTime;

    if(time_difference > 1000){
          //if it is a valid rising edge, set the interupt for the falling edge on the same pin
          attachInterrupt(PWM_pin[0], PWM_FALLING_ISR_0, FALLING);
    }
}

//interrupt service routine called on the falling edge of the pulse
void PWM_FALLING_ISR_0(){
    //record current time, this is the fall time of the pulse
    pinData[0].fallTime = micros();

    //calculate the time difference between the rising and falling edge
    //only a valid pulse if the time difference is between 400 and 2500 microseconds
     
    int time_difference = pinData[0].fallTime - pinData[0].riseTime;   
    if(time_difference >= 400 && time_difference <= 2500){
        //if it is a valid pulse, update the pin's last good width
        pinData[0].lastGoodWidth = time_difference;    
        //Serial.println(time_difference);
   }
   attachInterrupt(PWM_pin[0], PWM_RISING_ISR_0, RISING); 
}

//interupt service routine called on the rising edge of the pulse
void PWM_RISING_ISR_1(){
    //record current time.  this is the rise time
    pinData[1].riseTime = micros();
    //calculate the time sice the last falling edge was detected
    //only count this as a valid rising edge if time difference is more than 1000 microseconds
    unsigned long time_difference=  pinData[1].riseTime - pinData[1].fallTime;

    if(time_difference > 1000){
          //if it is a valid rising edge, set the interupt for the falling edge on the same pin
          attachInterrupt(PWM_pin[1], PWM_FALLING_ISR_1, FALLING);
    }
}

//interrupt service routine called on the falling edge of the pulse
void PWM_FALLING_ISR_1(){
    //record current time, this is the fall time of the pulse
    pinData[1].fallTime = micros();

    //calculate the time difference between the rising and falling edge
    //only a valid pulse if the time difference is between 400 and 2500 microseconds
     
    int time_difference = pinData[1].fallTime - pinData[1].riseTime;   
    if(time_difference >= 400 && time_difference <= 2500){
        //if it is a valid pulse, update the pin's last good width
        pinData[1].lastGoodWidth = time_difference;    
   }
   attachInterrupt(PWM_pin[1], PWM_RISING_ISR_1, RISING); 
}

//interupt service routine called on the rising edge of the pulse
void PWM_RISING_ISR_2(){
    //record current time.  this is the rise time
    pinData[2].riseTime = micros();
    //calculate the time sice the last falling edge was detected
    //only count this as a valid rising edge if time difference is more than 1000 microseconds
    unsigned long time_difference=  pinData[2].riseTime - pinData[2].fallTime;

    if(time_difference > 1000){
          //if it is a valid rising edge, set the interupt for the falling edge on the same pin
          attachInterrupt(PWM_pin[2], PWM_FALLING_ISR_2, FALLING);
    }
}

//interrupt service routine called on the falling edge of the pulse
void PWM_FALLING_ISR_2(){
    //record current time, this is the fall time of the pulse
    pinData[2].fallTime = micros();

    //calculate the time difference between the rising and falling edge
    //only a valid pulse if the time difference is between 400 and 2500 microseconds
     
    int time_difference = pinData[2].fallTime - pinData[2].riseTime;   
    if(time_difference >= 400 && time_difference <= 2500){
        //if it is a valid pulse, update the pin's last good width
        pinData[2].lastGoodWidth = time_difference;    
   }
   attachInterrupt(PWM_pin[2], PWM_RISING_ISR_2, RISING); 
}
//interupt service routine called on the rising edge of the pulse
void PWM_RISING_ISR_3(){
    //record current time.  this is the rise time
    pinData[3].riseTime = micros();
    //calculate the time sice the last falling edge was detected
    //only count this as a valid rising edge if time difference is more than 1000 microseconds
    unsigned long time_difference=  pinData[3].riseTime - pinData[3].fallTime;

    if(time_difference > 1000){
          //if it is a valid rising edge, set the interupt for the falling edge on the same pin
          attachInterrupt(PWM_pin[3], PWM_FALLING_ISR_3, FALLING);
    }
}

//interrupt service routine called on the falling edge of the pulse
void PWM_FALLING_ISR_3(){
    //record current time, this is the fall time of the pulse
    pinData[3].fallTime = micros();

    //calculate the time difference between the rising and falling edge
    //only a valid pulse if the time difference is between 400 and 2500 microseconds
     
    int time_difference = pinData[3].fallTime - pinData[3].riseTime;   
    if(time_difference >= 400 && time_difference <= 2500){
        //if it is a valid pulse, update the pin's last good width
        pinData[3].lastGoodWidth = time_difference;    
   }
   attachInterrupt(PWM_pin[3], PWM_RISING_ISR_3, RISING); 
}
