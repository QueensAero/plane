// Main plane program

//http://students.sae.org/cds/aerodesign/rules/2015_aero_rules.pdf

#include "plane.h" //all definitions

// #Includes
#include "Servo.h"

// Hardware #Includes
#include "Communicator.h"
#include "Adafruit_MPL3115A2.h"

double current_pitch, current_roll;
double base_pitch, base_roll;

// Altitude
double altitude;
boolean didGetZeroAltitudeLevel = false;

// System variables
byte blinkState;

// Servo declarations
Servo wheel_servo, left_aileron_servo, right_aileron_servo, left_Vtail_servo, right_Vtail_servo, flaps_servo; //L_flap_servo, R_flap_servo;

/*
volatile int pwm_high_time[5];  //In the order of: PID, pitch, roll, flap, yaw
volatile unsigned long pwm_high_start[5];
volatile unsigned long pwm_high_end[5];
*/

volatile int pwm_L_VTAIL_pid_mode, pwm_R_VTAIL_roll, pwm_L_AILERON_flap_mode, pwm_R_AILERON_yaw, pwm_FLAPS_pitch;
volatile unsigned long pwm_L_VTAIL_pid_mode_start, pwm_R_VTAIL_roll_start, pwm_L_AILERON_flap_mode_start, pwm_R_AILERON_yaw_start, pwm_FLAPS_pitch_start;
volatile unsigned long pwm_L_VTAIL_pid_mode_end, pwm_R_VTAIL_roll_end, pwm_L_AILERON_flap_mode_end, pwm_R_AILERON_yaw_end, pwm_FLAPS_pitch_end;

// This is the best way to declare objects!
// other ways work but are not robust... sometimes they fail because Arduino compiler is stupid
// this method does not call the constructor
// instead, you have to call an initialize() methode later on
Communicator comm;

// Sensor declerations
Adafruit_MPL3115A2 altimeter = Adafruit_MPL3115A2();

unsigned long current_time;
unsigned long prev_slow_time, prev_medium_time, prev_long_time;

void setup() {

  // Do this first so servos are good regardless
  initializeServos();

  // Start up serial communicator
  delay(3000);  // Give the XBee some time to start
  SerialUSB.begin(115200);

  comm.initialize();
  comm.sendMessage(MESSAGE_START);

  // Attach servos that are controlled directly by communicator
  comm.attachDropBay(DROP_PIN);

  // Initialize Data Acquisition System (which also preforms a DAS reset)
  initializeDAS();

  // Setup hardware that is controlled directly in the main loop
  blinkState = 0;
  pinMode(STATUS_LED_PIN, OUTPUT);

  // Start system time
  prev_medium_time = millis();
  prev_slow_time = prev_medium_time;
  prev_long_time = prev_medium_time;

  // Send message to ground station saying everything is ready
  comm.sendMessage(MESSAGE_READY);

}

void loop() {

  current_time = millis();

  unsigned long medium_time_diff = current_time - prev_medium_time;
  unsigned long slow_time_diff = current_time - prev_slow_time;
  unsigned long long_time_diff = current_time - prev_long_time;

  // Call loop functions in slow->fast order
  // this way, new commands are received in slow loop and implemented in faster loops
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

  // Check if commands received. This function executes quickly even if a command is received
  comm.recieveCommands();

  // Check if incoming data from GPS. If a full string is received, this function automatically parses it. Shouldn't take >1ms even when parsing required (which is 5x per second)
  comm.getSerialDataFromGPS();

}


// TODO: figure out which pwm_high_time indexes correspond to which signals. Also add any offsets or negative values (if not doing that on controller)
void mediumLoop() {

  // These lines write the values obtained through isr for each PWM signal
  // 0 = Left or right aileron
  // 1 = Left or right aileron
  // 2 = Mixed
  // 3 =

  // Wheel servo (requires demixing the two V tail signals (note: these two should also be used below for left and right VTail
  if ((pwm_L_VTAIL_pid_mode != 0) && (pwm_L_AILERON_flap_mode != 0)) {
    wheel_servo.writeMicroseconds(tail_wheel_demixing(pwm_R_AILERON_yaw, pwm_R_VTAIL_roll)); // Tail_wheel_demixing arguements are leftPin, rightPin.  returns value to write to servo
  }

  // Left Vtail
  if (pwm_L_VTAIL_pid_mode != 0) {
    left_Vtail_servo.writeMicroseconds(pwm_L_VTAIL_pid_mode);
  }

  // Right Vtail
  if (pwm_R_VTAIL_roll != 0) {
    right_Vtail_servo.writeMicroseconds(pwm_R_VTAIL_roll);
  }

  // Left Aileron  (actually right vtail)
  if (pwm_L_AILERON_flap_mode != 0) {
    left_aileron_servo.writeMicroseconds(pwm_L_AILERON_flap_mode);
  }

  // Right Aileron
  if (pwm_R_AILERON_yaw != 0) {
    right_aileron_servo.writeMicroseconds(pwm_R_AILERON_yaw);
  }
  //Flaps
  if(pwm_FLAPS_pitch != 0) {
    flaps_servo.writeMicroseconds(pwm_FLAPS_pitch);
  }
  
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

// Preforme serial communication in the slow loop - this needs to hapen less often
// slow loop was timed to take between 1.3 and 1.7ms.
void slowLoop() {

  //unsigned long loopStartTime = millis(); // For testing timing

  // Get latest telementary data
  double altitudeReadIn = altimeter.getAltitude(false);

  // Altitude = -999 means a timeout occured
  if (altitudeReadIn > -990) {

    //SerialUSB.print("Raw in: ");
    //SerialUSB.println(altitudeReadIn);

    if (!didGetZeroAltitudeLevel) {
      didGetZeroAltitudeLevel = true;
      altimeter.zero();
    }
    else {
      altitude = altitudeReadIn;
    }

  }

  //SerialUSB.print("Altitude: ");
  //SerialUSB.println(altitude);

  //unsigned long loopEndTime = millis(); //for testing timing

  // Pass new data to serial communicator
  comm.altitude = altitude;

  // Send data to XBee
  comm.sendData();

  // Check for reset flag
  if (comm.reset == true) {
    comm.reset = false;
    resetDAS();
    comm.sendMessage(MESSAGE_RESET_AKN);
  }

  // Check for restart flag
  if (comm.restart == true) {
    comm.restart = false;
    // Restart servos, PIDs, everything!
    resetDAS();
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
int tail_wheel_demixing(int leftSignal, int rightSignal) {

  leftSignal -= LEFT_SIGNAL_OFFSET;
  rightSignal -= RIGHT_SIGNAL_OFFSET;
  
  // These constants are from plane.h -> switch to true if required based on servo. WILL THIS AFFECT THE OFFSET SIGN? (ie. should it be done after?)
  if (FLIP_LEFT_SIGNAL) {
    leftSignal = map(leftSignal, 1000, 2000, 2000, 1000);
  }
  if (FLIP_RIGHT_SIGNAL) {
    rightSignal = map(rightSignal, 1000, 2000, 2000, 1000);
  }

  // Use following code to make sure servo is straight. Should obtain 1500 for both left and right signals at rest.
  //SerialUSB.print(leftSignal);   SerialUSB.print(" ");  SerialUSB.println(rightSignal);

  return constrain(map((leftSignal + rightSignal) / 2, 1000, 2000, 2000, 1000), 1000, 2000); //average left and right values, flip them, then constrain to between 1000-2000

}


// Initialize servo locations
void initializeServos() {

  // Set pin modes for incoming PWM signals
  pinMode(L_VTAIL_pid_mode_INPUT, INPUT);
  pinMode(R_VTAIL_roll_INPUT, INPUT);
  pinMode(L_AILERON_flap_mode_INPUT, INPUT);
  pinMode(R_AILERON_yaw_INPUT, INPUT);
  pinMode(FLAPS_pitch_INPUT, INPUT);

  // Attach each of the servos (before the interrupts to make sure it's initialized properly).
  wheel_servo.attach(RUDDER_OUT_PIN);
  left_aileron_servo.attach(L_AILERON_OUT_PIN);
  right_aileron_servo.attach(R_AILERON_OUT_PIN);
  left_Vtail_servo.attach(L_TAIL_OUT_PIN);
  right_Vtail_servo.attach(R_TAIL_OUT_PIN);
  flaps_servo.attach(FLAPS_OUT_PIN);
  //L_flap_servo.attach(L_FLAP_OUT_PIN);
  //R_flap_servo.attach(R_FLAP_OUT_PIN);

  // Apply 1500us neutral position (if drop bay is ever here, this neutral would likely no be ok for it
  wheel_servo.writeMicroseconds(1500);
  left_aileron_servo.writeMicroseconds(1500);
  right_aileron_servo.writeMicroseconds(1500);
  left_Vtail_servo.writeMicroseconds(1500);
  right_Vtail_servo.writeMicroseconds(1500);
  flaps_servo.writeMicroseconds(1500);

  // Attach interrupts on the rising edge of each input signal
  attachInterrupt(L_VTAIL_pid_mode_INPUT, isr_rising_L_VTAIL_pid_mode, RISING);
  attachInterrupt(L_AILERON_flap_mode_INPUT, isr_rising_L_AILERON_flap_mode, RISING);
  attachInterrupt(R_VTAIL_roll_INPUT, isr_rising_R_VTAIL_roll, RISING);
  attachInterrupt(R_AILERON_yaw_INPUT, isr_rising_R_AILERON_yaw, RISING);
  attachInterrupt(FLAPS_pitch_INPUT, isr_rising_FLAPS_pitch, RISING);

}

void initializeDAS() {

  // Initialize sensors
  altimeter.begin();
  altimeter.setReadTimeout(10);

  // Preform DAS reset
  resetDAS();

}

// Function to reset Data Acquisition System when resent command is sent via serial
void resetDAS() {

  // Turn off LED when reseting DAS
  // When it resumes blinking, we know the reset has finished
  digitalWrite(STATUS_LED_PIN, LOW);

  // Call individual reset functions for all the sensors
  // GPS doesn't retain status information (so don't need to reset). If we filter heading/speed in the future will need to do this

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

void isr_rising_L_VTAIL_pid_mode() {

  // Find current time in microseconds
  pwm_L_VTAIL_pid_mode_start = micros();

  // Find difference between this rising edge and last falling edge
  int time_low = pwm_L_VTAIL_pid_mode_start - pwm_L_VTAIL_pid_mode_end;

  // If it's been long enough since last falling edge (ie. isn't just debouncing) then attach falling interrupt
  if (time_low > 1000) {
    attachInterrupt(L_VTAIL_pid_mode_INPUT, isr_falling_L_VTAIL_pid_mode, FALLING);
  }

}

void isr_falling_L_VTAIL_pid_mode() {

  // Find current time in microseconds
  pwm_L_VTAIL_pid_mode_end = micros();

  // Find how long it was high for (in us)
  int time_high = pwm_L_VTAIL_pid_mode_end - pwm_L_VTAIL_pid_mode_start;

  // If it was high for correct amount of time (in range of servo commands) then save value
  if (time_high >= 400 && time_high <= 2500) {
    pwm_L_VTAIL_pid_mode = time_high;
  }

  // Regardless if time ok, attach rising interrupt, to get next duty cycle
  attachInterrupt(L_VTAIL_pid_mode_INPUT, isr_rising_L_VTAIL_pid_mode, RISING);

}

void isr_rising_L_AILERON_flap_mode() {

  // Find current time in microseconds
  pwm_L_AILERON_flap_mode_start = micros();

  // Find how long it was high for (in us)
  int time_low = pwm_L_AILERON_flap_mode_start - pwm_L_AILERON_flap_mode_end;

  // If it's been long enough since last falling edge (ie. isn't just debouncing) then attach falling interrupt
  if (time_low > 1000) {
    attachInterrupt(L_AILERON_flap_mode_INPUT, isr_falling_L_AILERON_flap_mode, FALLING);
  }

}

void isr_falling_L_AILERON_flap_mode() {

  // Find current time in microseconds
  pwm_L_AILERON_flap_mode_end = micros();

  // Find how long it was high for (in us)
  int time_high = pwm_L_AILERON_flap_mode_end - pwm_L_AILERON_flap_mode_start;

  // If it was high for correct amount of time (in range of servo commands) then save value
  if (time_high >= 400 && time_high <= 2500) {
    pwm_L_AILERON_flap_mode = time_high;
  }

  // Regardless if time ok, attach rising interrupt, to get next duty cycle
  attachInterrupt(L_AILERON_flap_mode_INPUT, isr_rising_L_AILERON_flap_mode, RISING);

}

void isr_rising_R_VTAIL_roll() {

  // Find current time in microseconds
  pwm_R_VTAIL_roll_start = micros();

  // Find how long it was high for (in us)
  int time_low = pwm_R_VTAIL_roll_start - pwm_R_VTAIL_roll_end;

  // If it's been long enough since last falling edge (ie. isn't just debouncing) then attach falling interrupt
  if (time_low > 1000) {
    attachInterrupt(R_VTAIL_roll_INPUT, isr_falling_R_VTAIL_roll, FALLING);
  }

}

void isr_falling_R_VTAIL_roll() {

  // Find current time in microseconds
  pwm_R_VTAIL_roll_end = micros();

  // Find how long it was high for (in us)
  int time_high = pwm_R_VTAIL_roll_end - pwm_R_VTAIL_roll_start;

  // If it was high for correct amount of time (in range of servo commands) then save value
  if (time_high >= 400 && time_high <= 2500) {
    pwm_R_VTAIL_roll = time_high;
  }

  // Regardless if time ok, attach rising interrupt, to get next duty cycle
  attachInterrupt(R_VTAIL_roll_INPUT, isr_rising_R_VTAIL_roll, RISING);

}

void isr_rising_R_AILERON_yaw() {

  // Find current time in microseconds
  pwm_R_AILERON_yaw_start = micros();

  // Find how long it was high for (in us)
  int time_low = pwm_R_AILERON_yaw_start - pwm_R_AILERON_yaw_end;

  // If it's been long enough since last falling edge (ie. isn't just debouncing) then attach falling interrupt
  if (time_low > 1000) {
    attachInterrupt(R_AILERON_yaw_INPUT, isr_falling_R_AILERON_yaw, FALLING);
  }

}

void isr_falling_R_AILERON_yaw() {

  // Find current time in microseconds
  pwm_R_AILERON_yaw_end = micros();

  // Find how long it was high for (in us)
  int time_high = pwm_R_AILERON_yaw_end - pwm_R_AILERON_yaw_start;

  // If it was high for correct amount of time (in range of servo commands) then save value
  if (time_high >= 400 && time_high <= 2500) {
    pwm_R_AILERON_yaw = time_high;
  }

  // Regardless if time ok, attach rising interrupt, to get next duty cycle
  attachInterrupt(R_AILERON_yaw_INPUT, isr_rising_R_AILERON_yaw, RISING);

}

void isr_rising_FLAPS_pitch() {

  // Find current time in microseconds
  pwm_FLAPS_pitch_start = micros();

  // Find how long it was high for (in us)
  int time_low = pwm_FLAPS_pitch_start - pwm_FLAPS_pitch_end;

  // If it's been long enough since last falling edge (ie. isn't just debouncing) then attach falling interrupt
  if (time_low > 1000) {
    attachInterrupt(FLAPS_pitch_INPUT, isr_falling_FLAPS_pitch, FALLING);
  }

}

void isr_falling_FLAPS_pitch() {

  // Find current time in microseconds
  pwm_FLAPS_pitch_end = micros();

  // Find how long it was high for (in us)
  int time_high = pwm_FLAPS_pitch_end - pwm_FLAPS_pitch_start;

  // If it was high for correct amount of time (in range of servo commands) then save value
  if (time_high >= 400 && time_high <= 2500) {
    pwm_FLAPS_pitch = time_high;
  }

  // Regardless if time ok, attach rising interrupt, to get next duty cycle
  attachInterrupt(FLAPS_pitch_INPUT, isr_rising_FLAPS_pitch, RISING);

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
