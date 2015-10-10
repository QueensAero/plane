#include "Arduino.h"
#include <Servo.h>

//SERVO VALUES
//XXX Calibrate
#define CAMERA_PAN_START_VAL 1500
#define CAMERA_TILT_START_VAL 1500

//XXX Calibrate
#define DROP_BAY_CLOSED 1100
#define DROP_BAY_OPEN 1900
#define TILT_SERVO_MIN -400
#define PAN_SERVO_MIN  -400
#define TILT_SERVO_MAX 400
#define PAN_SERVO_MAX  400
#define SERVO_STEP_SIZE  100


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
#define CAMERA_PAN_PIN 3

#define ELEVATOR_PIN 4
#define RIGHT_AILERON_PIN 5
#define LEFT_AILERON_PIN 6

class Communicator {
private:
	int cameraTiltPin;
	int cameraPanPin;
	int dropServoPin;
	
	int cameraTiltVal;
	int cameraPanVal;
	int dropVal;

        int cameraTiltBaseVal;
        int cameraPanBaseVal;

	double altitudeAtDrop;

	Servo cameraTiltServo;
	Servo cameraPanServo;
	Servo dropServo;

        int cameraAttached;
        int dropBayAttached;
       
        boolean dropBayOpen;
        unsigned long closeDropBayTime;
        unsigned long dropBayDelayTime;
        
        int sequence_step;
        
public:

        double airspeed, altitude, roll, pitch;

	Communicator();
        ~Communicator();
        void initialize();

	int getCameraTiltPin();
	int getCameraPanPin();
	int getDropPin();
        int waiting_for_message = false;
        int calibration_flag = false;

        boolean reset;
        boolean restart;
	
	//functions called by main program each loop
	void recieveCommands();  //when drop command is received set altitude at drop
	void sendData();  //send current altitude, altitude at drop, roll, pitch, airspeed
	
	//functions to attach sensors and motors	
        void attachCamera(int _cameraPanPin, int _cameraTiltPin);
        void attachDropBay(int _dropServoPin);
        
        //function to send standard message to ground station
        //examples: START, READY, RESET AKNOLAGED, DROP
        
        void sendMessage(char message); //takes single standard character that is a code for standard message - see definitions above
        
        
        //Calibration code
        void calibrate();
        int TiltVal_cal;
        int PanVal_cal;
        int dropVal_cal;
        int LeftAileronVal_cal;
        int RightAileronVal_cal;
        int ElevatorVal_cal;
        void initalizeCalibration();

};
