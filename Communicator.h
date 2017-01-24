#ifndef _COMMUNICATOR_H
#define _COMMUNICATOR_H

#include "Arduino.h"
#include <Servo.h>
#include "Adafruit_GPS.h"
#include "plane.h"
#include "Targeter.h"

// Drop Bay Servo Details.
#define DROP_BAY_CLOSED 1100
#define DROP_BAY_OPEN 1900

// MESSAGE CONSTANTS -- RECEIVE
#define INCOME_DROP			'P'
#define INCOME_AUTO_ON			'a'
#define INCOME_AUTO_OFF      'n'
#define INCOME_RESET		'r'
#define INCOME_RESTART		'q'
#define INCOME_DROP_ALT		'g'
#define INCOME_DROP_OPEN    'o'
#define INCOME_DROP_CLOSE  	'c'

// MESSAGE CONSTANTS -- SEND
#define DATA_PACKET 'p'
#define MESSAGE_START       's'
#define MESSAGE_READY       'r'
#define MESSAGE_DROP_OPEN   'o'
#define MESSAGE_DROP_CLOSE  'c'
#define MESSAGE_RESET_AKN   'k'
#define MESSAGE_RESTART_AKN 'q'
#define MESSAGE_CAM_RESET   'x'
#define MESSAGE_DROP_ACK    'y'
#define MESSAGE_AUTO_ON		'b'
#define MESSAGE_AUTO_OFF	'd'

// #define Servo pins
#define DROP_PIN 10

#define XBEE_BAUD 115200
#define XBEE_SERIAL Serial3   //Serial3

// GPS constants
#define MAXLINELENGTH 120
#define GPS_BAUD 9600
//Below GPS vary based on the new PCB or Old PCB
#define NEW_PCB
#ifdef NEW_PCB
    #define TX_TO_DISCONNECT 28 //These pins are unused anyways
    #define RX_TO_DISCONNECT 29
    #define GPS_SERIAL Serial1
#else
    #define TX_TO_DISCONNECT 18  // These are where the PCB traces go to. Due to a PCB error, TX went to TX, meaning nothing could to communicate. So we wired to Serial2 pins. But we need to makes sure these get disconnected
    #define RX_TO_DISCONNECT 19
    #define GPS_SERIAL Serial2   
#endif



class Communicator {

  private:
    int dropBayServoPos; 
    Servo dropServo;

    double altitudeAtDrop;
    unsigned long timeAtDrop;

    //Initialize XBee by starting communication and putting in transparent mode
    bool initXBee();
    bool sendCmdAndWaitForOK(String cmd, int timeout = 3000); //3 second as default timeout

    // Sending data
    void sendFloat(float toSend);
    void sendInt(int toSend);
    void sendUint16_t(uint16_t toSend);
    void sendUint8_t(uint8_t toSend);


  public:

    int currentTargeterDataPoint = -1;  //used for testing targeter


    double altitude, roll, pitch;

    Communicator();
    ~Communicator();
    void initialize();
    void dropNow(int src, int state);
    
    

    boolean reset;
    boolean restart;

    //gps variables and functions
    char nmeaBuf[MAXLINELENGTH];
    int nmeaBufInd = 0;
    boolean newParsedData = false;
    void getSerialDataFromGPS();
    void setupGPS();
    boolean autoDrop = true;  //TEMPORARY


    // Functions called by main program each loop
    void recieveCommands();  // When drop command is received set altitude at drop
    void sendData();  // Send current altitude, altitude at drop, roll, pitch, airspeed
    void checkToCloseDropBay(void);  //Close drop bay after 10 seconds of being open
    void recalculateTargettingNow(boolean withNewData);  //Check GPS data vs. target to see if we should drop

    // Function to send standard message to ground station
    // examples: START, READY, RESET ACKNOLEGED
    void sendMessage(char message); // Takes single standard character that is a code for standard message - see definitions above

};

#endif //_COMMUNICATOR_H
