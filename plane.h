#ifndef _PLANE_H
#define _PLANE_H

// OUTPUT pins (as labelled on PCB)
#define TAIL_WHEEL_OUT 2
#define LEFT_VTAIL_OUT 3
#define RIGHT_VTAIL_OUT 4
#define LEFT_AILERON_OUT 5
#define RIGHT_AILERON_OUT 6
#define FLAPS_LEFT_OUT 7
#define FLAPS_RIGHT_OUT 8
//DROP SERVO DEFINED IN COMMUNICATOR.H
#define SPARE_OUT 9


// INPUT pins (as labelled on PCB).
#define RIGHT_VTAIL_IN 12
#define LEFT_VTAIL_IN 11
#define LEFT_AILERON_IN 40
#define RIGHT_AILERON_IN 42
#define FLAPS_IN 50

// Define Demixing constants
#define FLIP_LEFT_SIGNAL false
#define FLIP_RIGHT_SIGNAL true
#define LEFT_SIGNAL_OFFSET 0
#define RIGHT_SIGNAL_OFFSET 0


//MUX Control Pins
#define MUX_CONTROL_SURFACE_PIN 44
#define MUX_FLAPS_PIN 46
#define ARDUINO_CTRL HIGH
#define SKIP_ARDUINO LOW

//Pushbuttons and Voltage
#define RESET_PUSHBUTTON_PIN 48
#define DROP_PUSHBUTTON_PIN 13
#define BATTERY_VOLTAGE_PIN A4
#define ANALOG_READ_CONV 3.3/4095.0 * 4.01204819  //last value: 1 / 332K /(332K + 1000K) -> voltage divider used to lower battery voltage to readable range
//TODO - test battery voltage in charger, then check this returns correct value (resistors may be off?)

// System timing variables in microseconds
#define SLOW_LOOP_TIME  250  //250    //Xbee send packets of data 
#define MEDIUM_LOOP_TIME 30  //50   // Servo updating
#define FAST_LOOP_TIME 1  	  // MPU updating, if PID's then compute new servo values
#define LONG_LOOP_TIME 500 	  // LED blinking

// Hardware declerations
#define HEARTBEAT_LED_PIN 50
#define STATUS_LED_PIN 13

// ------------------------------------ DROP BAY ------------------------------------

const int closeDropBayTimeout = 10000;

// ------------------------------------ TARGETING ------------------------------------

// Format: dd° mm.mmmm'
#define TARGET_LATT 4413.724
#define TARGET_LONG -7629.492
#define TARGET_ALTITUDE_M 0
// 4413.682, -77629.518 = Passage between ILC and Walter Light Hall
#define TARGET_RADIUS 20 //meters


// -------------------------------------------- DEBUG --------------------------------------------

// During testing we might want to send over USB to computer. Instead of commenting out a lot of  'SerialUSB.print(...)' statements we can define a macro as below
// If the line directly below is NOT commented out, then DEGUB_PRINT(...) will send to computer. If it is commented out, the macro DEBUG_PRINT/LN will be empty and
// the compiler will optimize it out of the code automatically
//#define DEBUG_COMMUNICATOR
#define DEBUG_SERIAL_BAUD 115200  // This actually doesn't matter - over USB it defaults to some high baudrate


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

// Tests the targeting system with pre-defined GPS datapoints
//#define Targeter_Test
//#define Targeter_Debug_Print  //ONLY WANT MINIMUM STUFF
//same thing with targeting debugging (Note - includes 'drop' and 'closeDropBay' functions
#if defined(Targeter_Test) || defined(Targeter_Debug_Print)
  #define TARGET_SERIAL Serial
  #define TARGET_PRINT(x) TARGET_SERIAL.print(x)
  #define TARGET_PRINTLN(x) TARGET_SERIAL.println(x)
  #define TARGET_BEGIN(x) TARGET_SERIAL.begin(x)
 #else
  #define TARGET_PRINT(x)
  #define TARGET_PRINTLN(x)
  #define TARGET_BEGIN(x)
#endif



#endif //_PLANE_H
