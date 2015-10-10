//DX6 RC receiver
#define PID_MODE_PIN 10
#define AILERON_INPUT_PIN 9
#define ELEVATOR_INPUT_PIN 8
#define AILERON_MODE_PIN 7

//system timing variables in microseconds
#define SLOW_LOOP_TIME 250 //100 miliseconds
#define MEDIUM_LOOP_TIME 50 //50 miliseconds
#define FAST_LOOP_TIME 1  //1 milisecond
#define LONG_LOOP_TIME 500 //0.5 seconds

//define Servo pins 
#define DROP_PIN 2 //actually on tilt pin, I accidently left out a physical drop pin on the PCB...
#define CAMERA_TILT_PIN 11 //this is not physically used on the PCB...
#define CAMERA_PAN_PIN 3

#define RIGHT_AILERON_SERVO_PIN  5
#define LEFT_AILERON_SERVO_PIN 6
#define ELEVATOR_SERVO_PIN 4

//hardware declerations
#define PS_PIN 48 //used for IntersemaBaro, not Eagle Tree
#define CS_PIN 46 //used for IntersemaBaro, not Eagle Tree
#define STATUS_LED_PIN 50


//XXX Calibrate - DONE
//#define CALIBRATE_ROLL_PITCH //if this line is not commented, the plane will print out desiered vs current roll / pitch
#define LEFT_AILERON_FLAP_VAL 350
#define RIGHT_AILERON_FLAP_VAL 350

