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
double altitudeFt;
boolean didGetZeroAltitudeLevel = false;

// System variables
byte blinkState;

// Servo declarations
Servo wheel_servo, l_aileron_servo, r_aileron_servo, l_Vtail_servo, r_Vtail_servo, l_flaps_servo, r_flaps_servo;

volatile int pw_l_vtail= 0, pw_r_vtail= 0, pw_l_aileron= 0, pw_r_aileron= 0, pw_flaps= 0;
volatile unsigned long pwm_l_vtail_start= 0, pwm_r_vtail_start= 0, pwm_l_aileron_start= 0, pwm_r_aileron_start= 0, pwm_flaps_start= 0;
volatile unsigned long pwm_l_vtail_end = 0, pwm_r_vtail_end = 0, pwm_l_aileron_end = 0, 
                       pwm_r_aileron_end = 0, pwm_flaps_end = 0;


//Control what the MUX output comes from
bool ctrl_sig_from_arduino = true;
bool flaps_sig_from_arduino = true;



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

  DEBUG_BEGIN(DEBUG_SERIAL_BAUD); // This is to computer (this is ok even if not connected to computer)
  TARGET_BEGIN(DEBUG_SERIAL_BAUD); //if either defined we start it (note it may restart if both defined)

  Serial.begin(115200);
  Serial.println("starting");

  // Do this first so servos are hopefully good regardless if below fails/times out/gets 'stuck'
  initializeServos();

  //Mux pins
  pinMode(MUX_CONTROL_SURFACE_PIN, OUTPUT);
  pinMode(MUX_FLAPS_PIN, OUTPUT);

  if(!ctrl_sig_from_arduino)
    digitalWrite(MUX_CONTROL_SURFACE_PIN, SKIP_ARDUINO);
  else
    digitalWrite(MUX_CONTROL_SURFACE_PIN, ARDUINO_CTRL);

   if(!flaps_sig_from_arduino)
    digitalWrite(MUX_FLAPS_PIN, SKIP_ARDUINO);
  else
    digitalWrite(MUX_FLAPS_PIN, ARDUINO_CTRL);


  //Pushbuttons
  pinMode(RESET_PUSHBUTTON_PIN, INPUT);
  pinMode(DROP_PUSHBUTTON_PIN, INPUT);

  //Battery Voltage
  pinMode(BATTERY_VOLTAGE_PIN, INPUT);
  analogReadResolution(12);  //use 12 bit analog read
  comm.sendMessage(MESSAGE_BATTERY_V, (float)analogRead(BATTERY_VOLTAGE_PIN)*ANALOG_READ_CONV);
  

  // Start up serial communicator
  delay(3000);  // Give the XBee some time to start - SUPPOSEDLY IT CAN CAUSE ERRORS IF SENT DATA BEFORE XBEE READY
  SerialUSB.begin(115200);

  comm.initialize();
  comm.sendMessage(MESSAGE_START);

  // Initialize Data Acquisition System (which also preforms a DAS reset)
  initializeDAS();

  // Setup hardware that is controlled directly in the main loop
  blinkState = 0;
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);

  pinMode(HEARTBEAT_LED_PIN, OUTPUT);
  digitalWrite(HEARTBEAT_LED_PIN, LOW);

  // Start system time
  prev_medium_time = millis();
  prev_slow_time = prev_medium_time;
  prev_long_time = prev_medium_time;

  //Setup interrupts for pushbuttons
  attachInterrupt(DROP_PUSHBUTTON_PIN,isr_drop_pushbutton, RISING);
  attachInterrupt(RESET_PUSHBUTTON_PIN,isr_reset_pushbutton, RISING);

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
  comm.recieveCommands(current_time);

  // Check if incoming data from GPS. If a full string is received, this function automatically parses it. Shouldn't take >1ms even when parsing required (which is 5x per second)
  comm.getSerialDataFromGPS();

}



// TODO: possibly flaps swtiching (if down is +ve on one and -ve on other), and find optimal position
//TODO: Tail wheel demixing
void mediumLoop() {


  Serial.print("LAil: "); Serial.print(pw_l_aileron); 
  Serial.print("\tRAil - is flaps: "); Serial.print(pw_r_aileron); //Not needed/used -> aileron are just opposite signal
  Serial.print("\tLVTail: "); Serial.print(pw_l_vtail); 
  Serial.print("\tRVTtail: "); Serial.print(pw_r_vtail);  //nothing/not needed
  Serial.print("\tFlaps: "); Serial.println(pw_flaps);

  //Recalculate targeting
  comm.recalculateTargettingNow(false); //(with non-new data - projects forward with time since received GPS data

  //L aileron output
  if(pw_l_aileron != 0)
  {
      l_aileron_servo.writeMicroseconds(pw_l_aileron);
  }

  //R aileron output
  if(pw_l_aileron != 0)
  {
      r_aileron_servo.writeMicroseconds(pw_l_aileron);  //Signal is basically l_aileron and NOT(r_aileron) -> move eact opposite
  }

  if(pw_l_vtail != 0)
  {
      l_Vtail_servo.writeMicroseconds(pw_l_vtail);
  }

  if(pw_r_vtail != 0)
  {
      r_Vtail_servo.writeMicroseconds(pw_r_vtail);
  }

  //TODO - may need to flip signal for left/right flaps
  //THIS ONE IS WEIRD
  if(pw_r_aileron != 0)
  { 
      //TODO - flaps incoming signal can be thought of as switch (so program value for switch high/low, and program switch threshold)
      l_flaps_servo.writeMicroseconds(pw_r_aileron);
      r_flaps_servo.writeMicroseconds(pw_r_aileron);   //ie. arguement map(pw_flaps, 1000, 2000, 2000, 1000); if need to flip
  }

  if(pw_l_vtail != 0 && pw_r_vtail != 0)
  {
    wheel_servo.writeMicroseconds(tail_wheel_demixing(pw_l_vtail, pw_r_vtail));    
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

  // Get latest altitude data
  double altitudeReadInFt = altimeter.getAltitudeFt(false);

  // Altitude = -999 means a timeout occured
  if (altitudeReadInFt > -990) {

    //DEBUG_PRINT("Raw alt: ");
    //DEBUG_PRINTLN(altitudeReadIn);

    if (!didGetZeroAltitudeLevel) {
      didGetZeroAltitudeLevel = true;
      altimeter.zero();
      altitudeFt = altimeter.getAltitudeFt(false);  //now get a correctly zerod altitude
    }
    else {
      altitudeFt = altitudeReadInFt;
    }

  }

  #ifdef Targeter_Test
    comm.recalculateTargettingNow(true);
  #endif

  // Pass new data to serial communicator
  comm.altitudeFt = altitudeFt;

  // Send data to XBee
  comm.sendData();

  comm.checkToCloseDropBay();

  // Check for reset flag
  if (comm.reset == true) {
    comm.reset = false;
    resetDAS();
    comm.sendMessage(MESSAGE_RESET_AKN);
    didGetZeroAltitudeLevel = false; //re-zero
    comm.altitudeAtDropFt = -1;  //also reset the altitude at drop

  }

  // Check for restart flag
  if (comm.restart == true) {
    comm.restart = false;
    resetDAS();
    comm.sendMessage(MESSAGE_RESTART_AKN);
    didGetZeroAltitudeLevel = false; //re-zero
  }
}

void longLoop() {
  blinkState = !blinkState;
  digitalWrite(HEARTBEAT_LED_PIN, blinkState);
}



// Initialize servo locations
void initializeServos() {

  // Set pin modes for incoming PWM signals
  pinMode(LEFT_VTAIL_IN, INPUT);
  pinMode(RIGHT_VTAIL_IN, INPUT);
  pinMode(LEFT_AILERON_IN, INPUT);
  pinMode(RIGHT_AILERON_IN, INPUT);
  pinMode(FLAPS_IN, INPUT);

  // Attach each of the servos (before the interrupts to make sure it's initialized properly).
  wheel_servo.attach(TAIL_WHEEL_OUT);
  l_aileron_servo.attach(LEFT_AILERON_OUT);
  r_aileron_servo.attach(RIGHT_AILERON_OUT);
  l_Vtail_servo.attach(LEFT_VTAIL_OUT);
  r_Vtail_servo.attach(RIGHT_VTAIL_OUT);
  l_flaps_servo.attach(FLAPS_LEFT_OUT);
  r_flaps_servo.attach(FLAPS_RIGHT_OUT);


  // Apply 1500us neutral position (if drop bay is ever here, this neutral would likely no be ok for it
  wheel_servo.writeMicroseconds(1500);
  l_aileron_servo.writeMicroseconds(1500);
  r_aileron_servo.writeMicroseconds(1500);
  l_Vtail_servo.writeMicroseconds(1500);
  r_Vtail_servo.writeMicroseconds(1500);
  l_flaps_servo.writeMicroseconds(1500);
  r_flaps_servo.writeMicroseconds(1500);


  // Attach interrupts on the rising edge of each input signal
  attachInterrupt(LEFT_VTAIL_IN, isr_rising_l_vtail, RISING);
  attachInterrupt(RIGHT_VTAIL_IN, isr_rising_r_vtail, RISING);
  attachInterrupt(LEFT_AILERON_IN, isr_rising_l_aileron, RISING);
  attachInterrupt(RIGHT_AILERON_IN, isr_rising_r_aileron, RISING);
  attachInterrupt(FLAPS_IN, isr_rising_flaps, RISING);


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
  digitalWrite(HEARTBEAT_LED_PIN, LOW);

  // Call individual reset functions for all the sensors
  // GPS doesn't retain status information (so don't need to reset). If we filter heading/speed in the future will need to do this

}



int tail_wheel_demixing(int leftSignal, int rightSignal) {

  leftSignal -= LEFT_SIGNAL_OFFSET; // Currently defined as 0 offset. may be subject to change
  rightSignal -= RIGHT_SIGNAL_OFFSET; //Same as above

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

/************ ISR's *****************/

//Pusbuttons
void isr_drop_pushbutton()
{
  /*
  //TODO - is guard below (for not in air) ok?  Have default disabled?
  if(comm.dropBayServoPos == DROP_BAY_OPEN)
  {
    comm.setDropBayState(MANUAL_CMD,DROPBAY_CLOSE);  //close drop bay    
  }
  else if(comm.altitudeFt < 10)
  { 
    //Only drop if near ground (prevent accidental activation in air??  
    comm.setDropBayState(MANUAL_CMD,DROPBAY_OPEN);
  }
  */
  //TODO - ensure doesn't become active on first push
  //TODO - decide on altitude filtering
  //TODO - debouncing (only accept trigger if >5s after previous push

    DEBUG_PRINTLN("Drop Pushbutton Pressed");

}

void isr_reset_pushbutton()
{

  DEBUG_PRINTLN("Reset Pushbutton Pressed");
  //TODO - have this do something
  //TODO - ensure doesn't become active on first push
  //TODO - decide on altitude filtering
  //TODO - debouncing (only accept trigger if >5s after previous push
  
  
}



/*************** RISING ****************/

void isr_rising_r_vtail()
{
  // Find current time in microseconds
  pwm_r_vtail_start = micros();

  // Find difference between this rising edge and last falling edge
  int low_time = pwm_r_vtail_start - pwm_r_vtail_end;

  // If it's been long enough since last falling edge (ie. isn't just debouncing) then attach falling interrupt
  if (low_time > 1000) {
    attachInterrupt(RIGHT_VTAIL_IN, isr_falling_r_vtail, FALLING);
  }

    DEBUG_PRINTLN("RVTAIL RISING ISR"); 

  
}

void isr_rising_l_vtail()
{
  // Find current time in microseconds
  pwm_l_vtail_start = micros();

  // Find difference between this rising edge and last falling edge
  int low_time = pwm_l_vtail_start - pwm_l_vtail_end;

  // If it's been long enough since last falling edge (ie. isn't just debouncing) then attach falling interrupt
  if (low_time > 1000) {
    attachInterrupt(LEFT_VTAIL_IN, isr_falling_l_vtail, FALLING);
  }

      DEBUG_PRINTLN("LVTAIL RISING ISR"); 

  
}

void isr_rising_r_aileron()
{
  // Find current time in microseconds
  pwm_r_aileron_start = micros();

  // Find difference between this rising edge and last falling edge
  int low_time = pwm_r_aileron_start - pwm_r_aileron_end;

  // If it's been long enough since last falling edge (ie. isn't just debouncing) then attach falling interrupt
  if (low_time > 1000) {
    attachInterrupt(RIGHT_AILERON_IN, isr_falling_r_aileron, FALLING);
  }

      DEBUG_PRINTLN("RAILERON RISING ISR"); 
  
}

void isr_rising_l_aileron()
{
  // Find current time in microseconds
  pwm_l_aileron_start = micros();

  // Find difference between this rising edge and last falling edge
  int low_time = pwm_l_aileron_start - pwm_l_aileron_end;

  // If it's been long enough since last falling edge (ie. isn't just debouncing) then attach falling interrupt
  if (low_time > 1000) {
    attachInterrupt(LEFT_AILERON_IN, isr_falling_l_aileron, FALLING);
  }

    DEBUG_PRINTLN("LAILERON RISING ISR");   
}

void isr_rising_flaps()
{
  // Find current time in microseconds
  pwm_flaps_start = micros();

  // Find difference between this rising edge and last falling edge
  int low_time = pwm_flaps_start - pwm_flaps_end;

  // If it's been long enough since last falling edge (ie. isn't just debouncing) then attach falling interrupt
  if (low_time > 1000) {
    attachInterrupt(FLAPS_IN, isr_falling_flaps, FALLING);
  }

   DEBUG_PRINTLN("FLAPS RISING ISR");   
  
}


/*************** FALLING ****************/
void isr_falling_r_vtail()
{
  // Find current time in microseconds
  pwm_r_vtail_end = micros();

  // Find difference between this rising edge and last falling edge
  int high_time = pwm_r_vtail_end - pwm_r_vtail_start;

  // If it's been long enough since last falling edge (ie. isn't just debouncing) then attach falling interrupt
  if (high_time > 800 && high_time < 2200) {
    pw_r_vtail = high_time;
  }

  attachInterrupt(RIGHT_VTAIL_IN, isr_rising_r_vtail, RISING);

   DEBUG_PRINTLN("RVTAIL FALLING ISR");   

  
}


void isr_falling_l_vtail()
{

    // Find current time in microseconds
  pwm_l_vtail_end = micros();

  // Find difference between this rising edge and last falling edge
  int high_time = pwm_l_vtail_end - pwm_l_vtail_start;

  // If it's been long enough since last falling edge (ie. isn't just debouncing) then attach falling interrupt
  if (high_time > 800 && high_time < 2200) {
    pw_l_vtail = high_time;
  }

  attachInterrupt(LEFT_VTAIL_IN, isr_rising_l_vtail, RISING);

     DEBUG_PRINTLN("LVTAIL FALLING ISR");   

  
}

void isr_falling_r_aileron()
{

    // Find current time in microseconds
  pwm_r_aileron_end = micros();

  // Find difference between this rising edge and last falling edge
  int high_time = pwm_r_aileron_end - pwm_r_aileron_start;

  // If it's been long enough since last falling edge (ie. isn't just debouncing) then attach falling interrupt
  if (high_time > 800 && high_time < 2200) {
    pw_r_aileron = high_time;
  }

  attachInterrupt(RIGHT_AILERON_IN, isr_rising_r_aileron, RISING);

   DEBUG_PRINTLN("RAILERON FALLING ISR");    
}

void isr_falling_l_aileron()
{
    // Find current time in microseconds
  pwm_l_aileron_end = micros();

  // Find difference between this rising edge and last falling edge
  int high_time = pwm_l_aileron_end - pwm_l_aileron_start;

  // If it's been long enough since last falling edge (ie. isn't just debouncing) then attach falling interrupt
  if (high_time > 800 && high_time < 2200) {
    pw_l_aileron = high_time;
  }

  attachInterrupt(LEFT_AILERON_IN, isr_rising_l_aileron, RISING);

 DEBUG_PRINTLN("LAILERON FALLING ISR");     
}

void isr_falling_flaps()
{

    // Find current time in microseconds
  pwm_flaps_end = micros();

  // Find difference between this rising edge and last falling edge
  int high_time = pwm_flaps_end - pwm_flaps_start;

  // If it's been long enough since last falling edge (ie. isn't just debouncing) then attach falling interrupt
  if (high_time > 800 && high_time < 2200) {
    pw_flaps = high_time;
  }

  attachInterrupt(FLAPS_IN, isr_rising_flaps, RISING);

   DEBUG_PRINTLN("FLAPS FALLING ISR");   
}
