#include "Arduino.h"
#include <Servo.h>
#include "Adafruit_GPS.h"

// Drop Bay Servo Details.
#define DROP_BAY_CLOSED 1100
#define DROP_BAY_OPEN 1900

// MESSAGE CONSTANTS
#define MESSAGE_START       's'
#define MESSAGE_READY       'r'
#define MESSAGE_DROP        'd'
#define MESSAGE_RESET_AKN   'k'
#define MESSAGE_RESTART_AKN 'q'
#define MESSAGE_CAM_RESET   'x'
#define MESSAGE_ERROR       'e'
#define MESSAGE_DROP_ACK    'y'
// Note (Not used anymore): currently in EagleTreeAltimeter the letter 't' is used to indicate an altimeter I2C timeout

// MPU6050 messages
#define MPU6050_READY         '1'
#define MPU6050_FAILED        '2'
#define MPU6050_DMP_READY     '3'
#define MPU6050_DMP_FAILED    '4'
#define MPU6050_INITIALIZING  '5'

// #define Servo pins
#define DROP_PIN 10

#define XBEE_BAUD 57600
#define SERIAL_USB_BAUD 9600   // This actually doesn't matter - over USB it defaults to some high baudrate
#define XBEE_SERIAL Serial3

// GPS constants
#define MAXLINELENGTH 120
#define TX_TO_DISCONNECT 18  // These are where the PCB traces go to. Due to a PCB error, TX went to TX, meaning nothing could to communicate. So we wired to Serial2 pins. But we need to makes sure these get disconnected
#define RX_TO_DISCONNECT 19
#define GPS_SERIAL Serial2   // Based on re-wiring of PCB
#define GPS_BAUD 9600

class Communicator {

  private:
    int dropServoPin;
    int dropBayServoPos;

    double altitudeAtDrop;

    Servo dropServo;

    int dropBayAttached;

    // These functions help accomplish the enterBypass mode function
    void sendBypassCommand();
    boolean checkInBypassMode();
    void flushInput();
    int flushInputUntilCurrentTime();
    boolean delayUntilSerialData(unsigned long ms_timeout);

    // Sending data
    void sendFloat(float toSend);
    void sendInt(int toSend);
    void sendUint16_t(uint16_t toSend);
    void sendUint8_t(uint8_t toSend);


  public:

    double altitude, roll, pitch;

    Communicator();
    ~Communicator();
    void initialize();


    int getDropPin();
    int waiting_for_message = false;
    int calibration_flag = false;

    boolean reset;
    boolean restart;

    //gps variables and functions
    char nmeaBuf[MAXLINELENGTH];
    int nmeaBufInd = 0;
    boolean newParsedData = false;
    void getSerialDataFromGPS();
    void setupGPS();


    // Functions called by main program each loop
    void recieveCommands();  // When drop command is received set altitude at drop
    void sendData();  // Send current altitude, altitude at drop, roll, pitch, airspeed

    // Functions to attach sensors and motors
    void attachDropBay(int _dropServoPin);

    // Function to send standard message to ground station
    // examples: START, READY, RESET AKNOLAGED, DROP
    void sendMessage(char message); // Takes single standard character that is a code for standard message - see definitions above

    // Ensure the XBee enters Bypass mode. If this fails, ALL communication will be non-functional
    boolean enterBypass();

    // Calibration code  (no longer used but left for reference)
    //void calibrate();

};
