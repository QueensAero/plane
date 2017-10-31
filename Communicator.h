#ifndef _COMMUNICATOR_H
#define _COMMUNICATOR_H

#include "Arduino.h"
#include <Servo.h>
#include "Adafruit_GPS.h"
#include "plane.h"
#include "Targeter.h"

// Drop Bay Servo Details.


// MESSAGE CONSTANTS -- RECEIVE
//Used characters: a,b,c,d,g,i,l,n,o,q,r,t,u
#define INCOME_AUTO_ON		 	 'a'
#define INCOME_AUTO_OFF      'n'
#define INCOME_RESET		     'r'
#define INCOME_RESTART		   'q'
#define INCOME_DROP_ALT		   'g'
#define INCOME_DROP_OPEN     'o'
#define INCOME_DROP_CLOSE  	 'c'
#define INCOME_BATTERY_V     'b'
#define INCOME_NEW_TARGET_START 't'
#define INCOME_CAM_TILT_UP   'u'
#define INCOME_CAM_TILT_DOWN 'd'
#define INCOME_CAM_PAN_LEFT  'l'
#define INCOME_CAM_PAN_RIGHT 'i'


// MESSAGE CONSTANTS -- SEND
//Used characters: a,b,c,d,k,o,p,q,r,s,w,x,y
#define DATA_PACKET         'p'
#define MESSAGE_START       's'
#define MESSAGE_READY       'r'
#define MESSAGE_DROP_OPEN   'o'
#define MESSAGE_DROP_CLOSE  'c'
#define MESSAGE_RESET_AKN   'k'
#define MESSAGE_RESTART_AKN 'q'
#define MESSAGE_CAM_RESET   'x'
#define MESSAGE_DROP_ACK    'y'
#define MESSAGE_AUTO_ON		  'b'
#define MESSAGE_AUTO_OFF	  'd'
#define MESSAGE_BATTERY_V   'w'
#define MESSAGE_ALT_AT_DROP 'a'

//Drop Bay Details
#define DROP_PIN 10
#define DROP_BAY_CLOSED 1500
#define DROP_BAY_OPEN 2100
#define DROPBAY_OPEN 1
#define DROPBAY_CLOSE 0
#define AUTOMATIC_CMD 1
#define MANUAL_CMD 0

//Camera Movement Details
#define TILT_PIN 5
#define PAN_PIN 6
#define TILT_INCREMENT 100
#define PAN_INCREMENT 100

//XBee
#define XBEE_BAUD 115200
#define XBEE_SERIAL Serial3

// GPS constants
#define MAXLINELENGTH 120
#define GPS_BAUD 9600
#define GPS_SERIAL Serial1

class Communicator {

  private:
    Servo dropServo;
    Servo panServo;
    Servo tiltServo;

    unsigned long timeAtDrop;

    // For receiving new GPS target
    byte targetLat[8];
    byte targetLon[8];
    double targetLatDoub;
    double targetLonDoub;
    unsigned long transmitStartTime;
    unsigned int bufferIndex; // Current position in received Target GPS position update message

    //Initialize XBee by starting communication and putting in transparent mode
    bool initXBee();
    bool sendCmdAndWaitForOK(String cmd, int timeout = 3000); //3 second as default timeout

    // Sending data
    void sendFloat(float toSend);
    void sendInt(int toSend);
    void sendUint16_t(uint16_t toSend);
    void sendUint8_t(uint8_t toSend);


    //GPS and Autotargeting
    boolean autoDrop = true;  //TODO TEMPORARY
    int nmeaBufInd = 0;
    boolean newParsedData = false;
    void setupGPS();
    void flushGPSSerial();
    bool checkReturnString(int commandNum);
    bool sendGPSConfigureCommands();

  public:

    int currentTargeterDataPoint = -1;  //used for testing targeter
    int dropBayServoPos;
    int panServoPos = 1800;
    int tiltServoPos = 1800;

    double altitudeFt, altitudeAtDropFt;

    Communicator();
    ~Communicator();
    void initialize();
    void setDropBayState(int src, int state);
    void moveCamera(char orientation);

    boolean reset;
    boolean restart;

    //gps variables and functions
    char nmeaBuf[MAXLINELENGTH];  //needs to be public so GPS class can access
    void getSerialDataFromGPS();  //needs to be public since called from plane


    // Functions called by main program each loop
    void recieveCommands(unsigned long curTime);  // When drop command is received set altitude at drop
    void sendData();  // Send current altitude, altitude at drop, roll, pitch, airspeed
    void checkToCloseDropBay(void);  //Close drop bay after 10 seconds of being open
    void recalculateTargettingNow(boolean withNewData);  //Check GPS data vs. target to see if we should drop

    // Function to send standard message to ground station
    // examples: START, READY, RESET ACKNOLEGED
    void sendMessage(char message); // Takes single standard character that is a code for standard message - see definitions above
    void sendMessage(char message, float value); // Messages with associated floats


};

#endif //_COMMUNICATOR_H
