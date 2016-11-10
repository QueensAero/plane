/*

  This class deals with all serial communication. This includes sending and receiving data from the XBee, as well as sending commands and receiving
  data from the GPS. The GPS class is only called from here
  It also controls the servos for the dropbay, since the "drop payload" command will be received directly by this class and it's easier then having another class.

*/

#include "Arduino.h"
#include "Communicator.h"
#include <Servo.h>

// During testing we might want to send over USB to computer. Instead of commenting out a lot of  'SerialUSB.print(...)' statements we can define a macro as below
// If the line directly below is NOT commented out, then DEGUB_PRINT(...) will send to computer. If it is commented out, the macro DEBUG_PRINT/LN will be empty and
// the compiler will optimize it out of the code automatically
#define DEBUG_COMMUNICATOR

#ifdef DEBUG_COMMUNICATOR
#define DEBUG_PRINT(x) SerialUSB.print(x)
#define DEBUG_PRINTLN(x) SerialUSB.println(x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif

Adafruit_GPS GPS;

//Constructor
Communicator::Communicator() {}
Communicator::~Communicator() {}

// Function called in void setup() that instantiates all the variables, attaches pins, ect
// This funciton needs to be called before anything else will work
void Communicator::initialize() {

  pinMode(TX_TO_DISCONNECT, INPUT);  // Ensure it's in high impedance state
  pinMode(RX_TO_DISCONNECT, INPUT);  // Ensure it's in high impedance state

#ifdef DEBUG_COMMUNICATOR
  SerialUSB.begin(SERIAL_USB_BAUD); // This is to computer
  DEBUG_PRINTLN("at start of comm initialize");
#endif

  // Set initial values to 0
  altitude = 0;
  roll = 0;
  pitch = 0;
  altitudeAtDrop = 0;

  // Start without hardware attached
  dropBayAttached = 0;

  // Initialize serial commuication to Xbee.  While loop is to enter bypass
  // If this fails all communication will fail, and the control will not work at all (I believe)
  XBEE_SERIAL.begin(XBEE_BAUD);  //this is to Xbee
  int tries = 0;
  while (!enterBypass() && ++tries < 5); // While until successfully entered bypass. This should only take 1 try

  //Setup the GPS
  setupGPS();

}

//Function called by main program and receiveCommands function. Toggles Drop Bay
//src == 1 corresponds to the automatic drop function.
void Communicator::dropNow(int src) {
	if(src == 1 && autoDrop == false) //AutoDrop Protection
		return;
	
	//Toggle Drop Bay
	if (dropBayServoPos == DROP_BAY_OPEN)
          dropBayServoPos = DROP_BAY_CLOSED;
	else
	  dropBayServoPos = DROP_BAY_OPEN;

	sendMessage(MESSAGE_DROP_ACK);
	dropServo.writeMicroseconds(dropBayServoPos);
	altitudeAtDrop = altitude;
}

// Function that is called from main program to receive incoming serial commands from ground station
// Commands are one byte long, represented as characters for easy reading
void Communicator::recieveCommands() {

  // Look for new byte from serial buffer
  if (XBEE_SERIAL.available() > 0) {
    // New command detected, parse and execute
    byte incomingByte = XBEE_SERIAL.read();
    DEBUG_PRINT("Received a command");

    // Drop bay (Manual Drop)
    if (dropBayAttached) {
      if (incomingByte == 'P')
		  dropNow(0); 
    }

    //Auto drop (Toggle)
    if(incomingByte == 's') {
      if(autoDrop == false)
        autoDrop = true;
      else
        autoDrop = false;
    }

    // Reset
    if (incomingByte == 'r') {  //RESET FUNCTION.
      sendData();  // Flush current data packets
      reset = true;
    }

    if (incomingByte == 'q') {  //RESTART FUNCTION.
      sendData();  //Flush current data packets
      restart = true;
      dropBayServoPos = DROP_BAY_CLOSED;
      dropServo.writeMicroseconds(dropBayServoPos);
    }

    if (incomingByte == 'g') {  //SEND ALTITUDE_AT_DROP
      XBEE_SERIAL.print("*a");
      XBEE_SERIAL.print(altitudeAtDrop);
      XBEE_SERIAL.print("%ee");
    }

  }
}


void Communicator::getSerialDataFromGPS() {

  while (GPS_SERIAL.available()) {

    nmeaBuf[nmeaBufInd] = GPS_SERIAL.read();
    //DEBUG_PRINT(nmeaBuf[nmeaBufInd]);
    if (nmeaBuf[nmeaBufInd++] == '\n') { // Increment index after checking if current character signifies the end of a string
      nmeaBuf[nmeaBufInd - 1] = '\0'; // Add null terminating character (note: -1 is because nmeaBufInd is incremented in if statement)
      newParsedData = GPS.parse(nmeaBuf); 	// This parses the string, and updates the values of GPS.lattitude, GPS.longitude etc.
      nmeaBufInd = 0;  // Regardless of it parsing sucessful, we want to reset position back to zero
    }

    if (nmeaBufInd >= MAXLINELENGTH) { // Should never happen. Means a corrupted packed and the newline was missed. Good to have just in case
      nmeaBufInd = 0;  // Note the next packet will then have been corrupted as well. Can't really recover until the next-next packet
    }
    //DEBUG_PRINTLN("");

  }

}

void Communicator::setupGPS() {

  // Initialize the variables in GPS class object
  GPS.init();

  // Start the serial communication
  GPS_SERIAL.begin(GPS_BAUD);

  // Commands to configure GPS:
  GPS_SERIAL.println(PMTK_SET_NMEA_OUTPUT_RMCONLY); 		// Set to only output GPRMC (has all the info we need),
  GPS_SERIAL.println(SET_NMEA_UPDATE_RATE_5HZ);			// Increase rate strings sent over serial
  GPS_SERIAL.println(PMTK_API_SET_FIX_CTL_5HZ);			// Increase rate GPS 'connects' and syncs with satellites
  GPS_SERIAL.println(ENABLE_SBAB_SATELLITES);				// Enable using a more accurate type of satellite
  GPS_SERIAL.println(ENABLE_USING_WAAS_WITH_SBSB_SATS); 	// Enable the above satellites in 'fix' mode (I think)
  delay(3000);  //Not really sure if needed.

  // Flush the GPS input (still unsure if the GPS sends response to the above commands)
  while (GPS_SERIAL.available()) {
    //GPS_SERIAL.read();
    DEBUG_PRINT("FLUSH RESPONSE: ");
    DEBUG_PRINT(GPS_SERIAL.read());
    DEBUG_PRINTLN("");
  }

  DEBUG_PRINTLN("DONE FLUSHING");

}

void Communicator::attachDropBay(int _dropServoPin) {
  dropBayAttached = 1;
  dropBayServoPos = DROP_BAY_CLOSED;
  dropServoPin = _dropServoPin;
  dropServo.attach(dropServoPin);
  dropServo.writeMicroseconds(dropBayServoPos);
}

int Communicator::getDropPin () {
  return dropServoPin;
}

// Data is sent via wireless serial link to ground station
// data packet format:  *p%ROLL%PITCH%ALTITUDE%AIRSPEED%LATTITUDE%LONGITUDE%HEADING%ms%secondee
// Not anymore: now is bytewise transmission of floats
// Total number of bytes: 11 (*p% type) + ~ 50 (data) = 61 bytes *4x/second = 244 bytes/s.  Each transmission is under outgoing buffer (128 bytes) and baud
// No other serial communication can be done in other classes!!!
void Communicator::sendData() {

  /*
    XBEE_SERIAL.print("*p%");
    XBEE_SERIAL.print(roll);  //6?
    XBEE_SERIAL.print('%');
    XBEE_SERIAL.print(pitch);  //6?
    XBEE_SERIAL.print('%');
    XBEE_SERIAL.print(altitude);  //6
    XBEE_SERIAL.print('%');
    XBEE_SERIAL.print(GPS.speed);  //5
    XBEE_SERIAL.print('%');
    XBEE_SERIAL.print(GPS.latitude, 4);  //8	//second argument is number of decimal places
    XBEE_SERIAL.print('%');
    XBEE_SERIAL.print(GPS.longitude, 4);  //8
    XBEE_SERIAL.print('%');
    XBEE_SERIAL.print(GPS.angle, 2);  //6
    XBEE_SERIAL.print('%');
    XBEE_SERIAL.print(GPS.milliseconds);  //2
    XBEE_SERIAL.print('%');
    XBEE_SERIAL.print(GPS.seconds);	//3
    XBEE_SERIAL.print("ee"); */

  // Change to:  (total of 35 bytes)
  XBEE_SERIAL.print("*p");
  sendFloat((float)roll);
  sendFloat((float)pitch);
  sendFloat((float)altitude);
  sendFloat(GPS.speed);
  sendFloat(GPS.latitude);
  sendFloat(GPS.longitude);
  sendFloat(GPS.angle);
  sendUint16_t(GPS.milliseconds);
  sendUint8_t(GPS.seconds);
  XBEE_SERIAL.print("ee");

}

void Communicator::sendMessage(char message) {
  XBEE_SERIAL.print("*");
  XBEE_SERIAL.print(message);
  XBEE_SERIAL.print("ee");
}

void Communicator::sendUint8_t(uint8_t toSend) {
  byte *data = (byte*)&toSend; //cast address of input to byte array
  XBEE_SERIAL.write(data, sizeof(toSend));
}

void Communicator::sendUint16_t(uint16_t toSend) {
  byte *data = (byte*)&toSend; //cast address of input to byte array
  XBEE_SERIAL.write(data, sizeof(toSend));
}

void Communicator::sendInt(int toSend) {
  byte *data = (byte*)&toSend; //cast address of input to byte array
  XBEE_SERIAL.write(data, sizeof(toSend));
}

void Communicator::sendFloat(float toSend) {
  byte *data = (byte*)&toSend; //cast address of float to byte array
  XBEE_SERIAL.write(data, sizeof(toSend));  //send float as 4 bytes
}

// Send 'B' to enter bypass mode
boolean Communicator::enterBypass() {

  boolean success = false;
  success = checkInBypassMode();  // Check before changing baud rate

  // Needs to be at 9600 baud to enter bypass mode
  XBEE_SERIAL.end();
  XBEE_SERIAL.begin(9600);

  int numTries = 0, maxNumTries = 3;
  if (!success) {
    while (++numTries <= maxNumTries) {
      sendBypassCommand();
      success = checkInBypassMode();

      if (success) {
        break;
      }
    }
  }

  flushInput();

  if (!success) {
    DEBUG_PRINT("Failed to enter bypass, numtries = ");
  }
  else {
    DEBUG_PRINT("Entered bypass, numtries = ");
  }

  DEBUG_PRINTLN(numTries);

  XBEE_SERIAL.end();
  XBEE_SERIAL.begin(XBEE_BAUD);  //restart serial in the set xbee baudrate

  return success;

}

// Max time = 1s
void Communicator::sendBypassCommand() {
  flushInput();
  XBEE_SERIAL.print("B");
}

// Max time = 1.5s
// The idea behind this: if in non-bypass mode, every character sent to the xbee is echoed back.
// So the test is to send a character, and wait. If nothing is received, the test is true (ie. we are in bypass mode)
// Which is what we want to be in.  If something is received, then it indicates that we need to keep trying to enter bypass mode
boolean Communicator::checkInBypassMode() {

  flushInput();
  XBEE_SERIAL.print('@');

  if (delayUntilSerialData(1500)) { //true indicates it got data, which means it's NOT in bypass mode
    return false;
  }
  else {
    return true;
  }

}


// We could receive more data while flushing the input. So instead of using just 1 call to 'flushInputUntilCurrentTime'  we repeatedly call it until nothing has been flushedthen
// it's likely very rare this has any effect, but it doesn't hurt
void Communicator::flushInput() {
  while (flushInputUntilCurrentTime() > 0);
}

// Time should be negligible.  returns number of values discarded
int Communicator::flushInputUntilCurrentTime() {

  int valuesDiscarded = 0;
  //flush input
  while (XBEE_SERIAL.available())
  { XBEE_SERIAL.read();
    valuesDiscarded++;
  }
  return valuesDiscarded;

}

//delay until Serial.available() >0, or until timeout in ms reached. false = did not get data
//Max time = ms_timeout arguement
boolean Communicator::delayUntilSerialData(unsigned long ms_timeout) {

  unsigned long startTime = millis();

  while (XBEE_SERIAL.available() <= 0 && (millis() - startTime) < ms_timeout)
    delay(2);

  if ((millis() - startTime) >= ms_timeout  && !XBEE_SERIAL.available()) {//if no data was received before timeout
    return false;
  }
  else {
    return true;
  }

}

/*  No longer used.  Leave in case needed for later years
  void Communicator::calibrate(){
  //set inital calibration values
  waiting_for_message = true;


  //serial buffer - defined outside of loop
  char serial_buffer[16] = "";
  int serial_index = 0;

  while(waiting_for_message){
    if (Serial.available() > 0) {
      //check for new bytes and add them to the buffer
      // cli();  // disable global interrupts for serial read
      byte incoming_byte = Serial.read();
      //sei(); //inable interups again
      if (incoming_byte == '~'){ //end of calibration session, stop and set flags
        waiting_for_message=false;
        calibration_flag=false;
     }
     else if (incoming_byte == '&') { //this is the character sent at the start of the communication
        Serial.print('&');  //send a '&' back to confirm connection
     }
     else {
        //add character to buffer
        serial_buffer[serial_index] = incoming_byte;
        serial_index ++;

        if (serial_index == 16) serial_index = 0; //catch overflow - should not happen, 16 bytes is enough

        //if message is complete, update DAC_values
        if (incoming_byte == '*') {
          // termanating character found, message received
          waiting_for_message = false;

          //determine SERVO_target number by converting ASCII character to number
          int SERVO_target_number = serial_buffer[0] - 48;

          //transfer value part of message into new char array
          int message_size = serial_index - 3;   //message starts on character # 3 and serial_index has been incrememted by 1 at this point
          char* message = (char*) malloc(sizeof(char*) * message_size); //get space for message char array

          for (int i = 0; i < message_size; i++) {
            message[i] = serial_buffer[i + 2];
          }

          //convert char array into float value
          int val = atoi(message);
          if (val > 2000) val = 2000;
          if (val < 0) val = 0;

          //check to make sure that the values are in the correct range
          if (SERVO_target_number > 7) SERVO_target_number = 7;
          if (SERVO_target_number < 0) SERVO_target_number = 0;

          //Convert val to match servo range
          int servoVal;

          Serial.print(SERVO_target_number);
          Serial.print("/");
          Serial.print(val);
          Serial.print("*");

          if (SERVO_target_number == 0){
            cameraTiltBaseVal = val;
            cameraTiltServo.writeMicroseconds(cameraTiltBaseVal);
          }
          if (SERVO_target_number == 1){
            cameraPanBaseVal = val;
            cameraPanServo.writeMicroseconds(cameraPanBaseVal);
          }
          if (SERVO_target_number == 2){
            LeftAileronVal_cal = val;
          }
          if (SERVO_target_number == 3){
            RightAileronVal_cal = val;
          }
          if (SERVO_target_number == 4){
            ElevatorVal_cal = val;
          }

          Serial.flush();
          serial_index = 0; //start over for next message
          free(message);
        }
      }
    }
  }

  }*/
