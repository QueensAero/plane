#include "Arduino.h"
#include "Communicator.h"
#include <Servo.h>


//constructor
Communicator::Communicator() {
}
Communicator::~Communicator() {}

//function called in void setup() that instantiates all the variables, attaches pins, ect
//this funciton needs to be called before anything else will work
void Communicator::initialize() {


  //set initial values to 0
  airspeed = 0;
  altitude = 0;
  roll = 0;
  pitch = 0;
  altitudeAtDrop = 0;
  lattitude = 0;
  longitude = 0;
  heading = 0;

  //start without hardware attached
  dropBayAttached = 0;
  dropBayOpen = 0;
  dropBayDelayTime = 5000;

  //initialize serial commuication to Xbee.  Infinite while loop is to enter bypass
  //If this fails all communication will fair and this is essentially useless
  Serial.begin(XBEE_BAUD);  //this is to Xbee
  int tries = 0;
  while(!enterBypass() && ++tries < 5);  //infinite while until successfully entered bypass. This should only take 1 try

  #ifdef DEBUG_COMMUNICATOR
      SerialUSB.begin(SERIAL_USB_BAUD); //this is to computer
  #endif

}

//function that is called from main program to receive incoming serial commands from ground station
//commands are one byte long, represented as characters for easy reading
void Communicator::recieveCommands() {
  //command syntax:
  //u=up  d=down l=left r=right x=drop

  //look for new byte from serial buffer
  byte incomingByte;
  if (Serial.available() > 0) {
    //new command detected, parse and execute

    incomingByte = Serial.read();
    //check to make sure hardware is connected before atempting to write values

    //drop bay
    if (dropBayAttached) {
      if (incomingByte == 'P') {
        dropVal = DROP_BAY_OPEN;
        sendMessage(MESSAGE_DROP_ACK);
        dropServo.writeMicroseconds(dropVal);
        altitudeAtDrop = altitude;
        dropBayOpen = 1;
        closeDropBayTime = millis() + dropBayDelayTime;
      }
    }

    //reset
    if (incomingByte == 'r') {  //RESET FUNCTION.
      sendData();  //flush current data packets
      reset = true; 
    }

    if (incomingByte == 'q') {  //RESTART FUNCTION.
      sendData();  //flush current data packets
      restart = true;
      dropVal = DROP_BAY_CLOSED;
      dropServo.writeMicroseconds(dropVal);
    }
    
  }
}

void Communicator::attachDropBay(int _dropServoPin) {
  dropBayAttached = 1;
  dropVal = DROP_BAY_CLOSED;
  dropServoPin = _dropServoPin;
  dropServo.attach(dropServoPin);
  dropServo.writeMicroseconds(dropVal);
}

int Communicator::getDropPin () {
  return dropServoPin;
}

//data is sent via wireless serial link to ground station
//data packet format:  *p%ROLL%PITCH%ALTITUDE%AIRSPEED%LATTITUDE%LONGITUDE%HEADING&
//Total number of bytes: 10 (*p% type) + ~ 50 (data) = 60 bytes *4x/second = 240 bytes/s.  This is under outgoing buffer (128 bytes) and baud
//no other serial communication can be done in other classes!!!
void Communicator:: sendData() {
    Serial.print("*p%");
    Serial.print(roll);
    Serial.print('%');
    Serial.print(pitch);
    Serial.print('%');
    Serial.print(altitude);
    Serial.print('%');
    Serial.print(airspeed);
	Serial.print('%');
	Serial.print(lattitude, 4);  //second argument is number of decimal places
	Serial.print('%');
	Serial.print(longitude, 4);
	Serial.print('%');
	Serial.print(heading, 2);
    Serial.print('&');
}

void Communicator::sendMessage(char message) {
  Serial.print("*");
  Serial.print(message);
  Serial.print("&");
}




//open bootloader (by sending newline), then send 'B' to enter bypass mode
boolean Communicator::enterBypass(){

    boolean success = false;
    success = checkInBypassMode();  //check before changing baud rate 

    //needs to be at 9600 baud to enter bypass mode
    Serial.end();
    Serial.begin(9600);  
        
    int numTries = 0, maxNumTries = 3;
    if(!success)
    {
      while(++numTries <= maxNumTries)  
      {
        sendBypassCommand();          
        success = checkInBypassMode();
        
        if(success)
          break;        
      }
    }

    flushInput();

    if(!success)
        DEBUG_PRINT("Failed to enter bypass, numtries = ");
    else
        DEBUG_PRINT("Entered bypass, numtries = ");
    
     DEBUG_PRINTLN(numTries);

     Serial.end();
     Serial.begin(XBEE_BAUD);  //restart serial in the set xbee baudrate
     
     return success;
    
}

//max time = 1s
void Communicator::sendBypassCommand(){

     flushInput();
     Serial.print("B");  
}

//Max time = 1.5s 
boolean Communicator::checkInBypassMode(){

  flushInput();
  Serial.print('@');
  
  if(delayUntilSerialData(1500))  //true indicates it got data, which means it's NOT in bypass mode 
    return false;
  else
    return true;
 
}


//if receive data after first flush started, this will attempt to flush again
void Communicator::flushInput(){

  while(flushInputUntilCurrentTime() >0);
  
}

//time should be negligible.  returns number of values discarded
int Communicator::flushInputUntilCurrentTime()
{
  int valuesDiscarded = 0;
  //flush input
  while(Serial.available())
  {    Serial.read();  
       valuesDiscarded++;
  }
  return valuesDiscarded;
}

//delay until Serial.available() >0, or until timeout in ms reached. false = did not get data
//Max time = ms_timeout arguement
boolean Communicator::delayUntilSerialData(unsigned long ms_timeout)
{ 
  unsigned long startTime = millis();

  while(Serial.available() <= 0 && (millis()-startTime) < ms_timeout)
      delay(2);  

  if((millis()-startTime) >= ms_timeout  && !Serial.available())  //if no data was received before timeout
      return false;
  else
      return true;
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
