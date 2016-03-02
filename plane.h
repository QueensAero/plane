//DX6 RC receiver
#define ELEVATOR_INPUT_PIN 7
#define RUDDER_INPUT_PIN 6

//system timing variables in microseconds
#define SLOW_LOOP_TIME 250 //100 miliseconds
#define MEDIUM_LOOP_TIME 50 //50 miliseconds
#define FAST_LOOP_TIME 1  //1 milisecond
#define LONG_LOOP_TIME 500 //0.5 seconds

//define Servo pins 
#define DROP_PIN 2 //actually on tilt pin, I accidently left out a physical drop pin on the PCB...
#define TAIL_WHEEL_PIN 12

//hardware declerations
#define STATUS_LED_PIN 50
