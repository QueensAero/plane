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
  altitude = 0;
  roll = 0;
  pitch = 0;
  altitudeAtDrop = 0;

  //start without hardware attached
  dropBayAttached = 0;
  dropBayOpen = 0;
  dropBayDelayTime = 5000;

  sequence_step = 0;
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
    //check to make sure hardware is connected before atempting to write value

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
      while (sequence_step > 0) sendData();  //flush current data packets
      reset = true; 
    }

    if (incomingByte == 'q') {  //RESTART FUNCTION.
      while (sequence_step > 0) sendData();  //flush current data packets
      restart = true;
      dropVal = DROP_BAY_CLOSED;
      dropServo.writeMicroseconds(dropVal);
    }
    if (incomingByte == '$'){
      if(millis() < 10000){
          calibration_flag = true;        
      }
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
//data packet format:  *ROLL%PITCH%ALTITUDE%AIRSPEED&
//packet sent one variable at a time so it does not take too long
//no other serial communication can be done in other classes!!!
void Communicator:: sendData() {
  if (sequence_step == 0) {
    Serial.print("*p%");
    Serial.print(roll);
  }
  if (sequence_step == 1) {
    Serial.print('%');
    Serial.print(pitch);
  }
  if (sequence_step == 2) {
    Serial.print('%');
    Serial.print(altitude);
  }
  sequence_step++;
}
void Communicator::sendMessage(char message) {
  //need to flush data packet before sending message
  //otherwise, will cause terminal to misinterpret data packet
  while (sequence_step > 0) sendData();

  Serial.print("*");
  Serial.print(message);
  Serial.print("&");
}

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
          
          Serial.flush();
          serial_index = 0; //start over for next message
          free(message);
        }
      }
    }  
  } 
  
}
