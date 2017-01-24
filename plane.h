#ifndef _PLANE_H
#define _PLANE_H

// OUTPUT pins (as labelled on PCB)
#define RUDDER_OUT_PIN 2
#define L_TAIL_OUT_PIN 3
#define R_TAIL_OUT_PIN 4
#define R_AILERON_OUT_PIN 5
#define L_AILERON_OUT_PIN 6
#define SPARE_OUT_PIN 13
//#define DROP_OUT_PIN 10  // Note this is done by communicator class

// INPUT pins (as labelled on PCB). These labels are misleading/innaccurate
#define PID_MODE_INPUT 7
#define PITCH_INPUT 8
#define ROLL_INPUT 9
#define FLAP_MODE_INPUT 11
#define YAW_INPUT 12

// Define Demixing constants
#define FLIP_LEFT_SIGNAL false
#define FLIP_RIGHT_SIGNAL true
#define LEFT_SIGNAL_OFFSET 0
#define RIGHT_SIGNAL_OFFSET -150

// System timing variables in microseconds
#define SLOW_LOOP_TIME 250    //Xbee send packets of data 
#define MEDIUM_LOOP_TIME 50   // Servo updating
#define FAST_LOOP_TIME 1  	  // MPU updating, if PID's then compute new servo values
#define LONG_LOOP_TIME 500 	  // LED blinking

// Hardware declerations
#define HEARTBEAT_LED_PIN 50
#define STATUS_LED_PIN 13

// ------------------------------------ DROP BAY ------------------------------------

const int closeDropBayTimeout = 10000;

// ------------------------------------ TARGETING ------------------------------------

// Format: dd° mm.mmmm'
const double targetLatitude = 4413.682;
const double targetLongitude = -7629.518;
// 4413.682, -77629.518 = Passage between ILC and Walter Light Hall

const double targetAltitude = 0; // meters
const double targetRaduis = 20; // meters

// -------------------------------------------- DEBUG --------------------------------------------

// During testing we might want to send over USB to computer. Instead of commenting out a lot of  'SerialUSB.print(...)' statements we can define a macro as below
// If the line directly below is NOT commented out, then DEGUB_PRINT(...) will send to computer. If it is commented out, the macro DEBUG_PRINT/LN will be empty and
// the compiler will optimize it out of the code automatically
#define DEBUG_COMMUNICATOR

#ifdef DEBUG_COMMUNICATOR
#define DEBUG_SERIAL Serial
#define DEBUG_PRINT(x) DEBUG_SERIAL.print(x)
#define DEBUG_PRINTLN(x) DEBUG_SERIAL.println(x)
#define DEBUG_BEGIN(x) DEBUG_SERIAL.begin(x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#define DEBUG_BEGIN(x)
#endif

//#define Targeter_Test
// Tests the targeting system with pre-defined GPS datapoints

#endif //_PLANE_H
