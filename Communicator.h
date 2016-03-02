#include "Arduino.h"
#include <Servo.h>

//Drop Bay Servo Details. UPDATE THESE FOR 2016!!
#define DROP_BAY_CLOSED 1100
#define DROP_BAY_OPEN 1900



//MESSAGE CONSTANTS
#define MESSAGE_START       's' 
#define MESSAGE_READY       'r'
#define MESSAGE_DROP        'd'
#define MESSAGE_RESET_AKN   'k'
#define MESSAGE_RESTART_AKN 'q'
#define MESSAGE_CAM_RESET   'x'
#define MESSAGE_ERROR       'e'
#define MESSAGE_DROP_ACK    'y'

//MPU6050 messages
#define MPU6050_READY         '1' 
#define MPU6050_FAILED        '2'
#define MPU6050_DMP_READY     '3'
#define MPU6050_DMP_FAILED    '4'
#define MPU6050_INITIALIZING  '5'


//define Servo pins 
#define DROP_PIN 2


//For testing by sending over USB to computer
//#define DEBUG_COMMUNICATOR
#ifdef DEBUG_COMMUNICATOR
  #define DEBUG_PRINT(x) SerialUSB.print(x)
  #define DEBUG_PRINTLN(x) SerialUSB.println(x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)  
#endif


#define XBEE_BAUD 57600
#define SERIAL_USB_BAUD 9600   //this actually doesn't matter - over USB it defaults to some high baudrate


class Communicator {
private:
	int dropServoPin;
	int dropVal;


	double altitudeAtDrop;

	Servo dropServo;

    int dropBayAttached;
       
    boolean dropBayOpen;
    unsigned long closeDropBayTime;
    unsigned long dropBayDelayTime;
	
	//These functions help accomplish the enterBypass mode function
	void sendBypassCommand();
	boolean checkInBypassMode();
	void flushInput();
	int flushInputUntilCurrentTime();
	boolean delayUntilSerialData(unsigned long ms_timeout);
        
        
public:

    double airspeed, altitude, roll, pitch, lattitude, longitude, heading;

	Communicator();
    ~Communicator();
    void initialize();

	int getDropPin();
    int waiting_for_message = false;
    int calibration_flag = false;

    boolean reset;
    boolean restart;
	
	//functions called by main program each loop
	void recieveCommands();  //when drop command is received set altitude at drop
	void sendData();  //send current altitude, altitude at drop, roll, pitch, airspeed
	
	//functions to attach sensors and motors	
    void attachDropBay(int _dropServoPin);
        
    //function to send standard message to ground station
    //examples: START, READY, RESET AKNOLAGED, DROP
    void sendMessage(char message); //takes single standard character that is a code for standard message - see definitions above
    
	//Ensure the XBee enters Bypass mode. If this fails, ALL communication will be non-functional
	boolean enterBypass();    
        
	//Calibration code  (no longer used but left for reference)
	//void calibrate();

};
