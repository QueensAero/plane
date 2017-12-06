/*

  This class deals with all serial communication. This includes sending and receiving data from the XBee, as well as sending commands and receiving
  data from the GPS. The GPS class is only called from here
  It also controls the servos for the dropbay, since the "drop payload" command will be received directly by this class and it's easier then having another class.

*/

//TODO
//Code to receive a command to update target location based on ground station message (is this useful??)
//Code for commands to enable/disable in flight mode  (ie. disable pushbuttons once takeoff)
//Code to program flaps to 'brake' mode (Ask mech if useful??)



#include "Arduino.h"
#include "Communicator.h"
#include <Servo.h>
#include "Targeter.h"

// System variables
boolean noFixLedIsOn = true;

Adafruit_GPS GPS;
Targeter targeter;

//Constructor
Communicator::Communicator() {}
Communicator::~Communicator() {}

// -------------------------------------------- DEBUG TARGET DATA --------------------------------------------

#ifdef Targeter_Test

//static float GPSLatitudes[] = {4413.546, 4413.574, 4413.610, 4413.636, 4413.660};
//static float GPSLongitudes[] = { -7629.504, -7629.507, -7629.509, -7629.513, -7629.514};

static float GPSLatitudes[] = {4413.546, 4413.574, 4413.610, 4413.636, 4413.668, 4413.688, 4413.718, 4413.7225, 4413.7241};
static float GPSLongitudes[] = { -7629.504, -7629.507, -7629.509, -7629.513, -7629.509, -7629.500, -7629.496, -7629.490, -7629.490};
// Points start at south end of bioscience complex and move North along Arch street

static float altitudes[] = {100, 100, 100, 100, 100, 100, 100, 100, 100};  //FT
static float velocities[] = {20, 20, 20, 20, 20, 20, 20, 20, 20};
static float headings[] = {360, 360, 360, 360, 360, 360, 360, 360, 360};

#define NUM_TARGETER_DATAPTS sizeof(GPSLatitudes) / sizeof(GPSLatitudes[0])
#endif
// -------------------------------------------- END DEBUG TARGET DATA --------------------------------------------



// Function called in void setup() that instantiates all the variables, attaches pins, ect
// This funciton needs to be called before anything else will work
void Communicator::initialize() {

  DEBUG_PRINTLN("Initializing Communicator");

  // Set initial values to 0
  altitudeFt = 0;
  altitudeAtDropFt = 0;
  timeAtDrop = 0;
  bufferIndex = 0;

  //Attach servo, init position to closed
  dropServo.attach(DROP_PIN);
  dropBayServoPos = DROP_BAY_CLOSED;
  dropServo.writeMicroseconds(dropBayServoPos);

  tiltServoPos = 1800;
  panServoPos = 1800;
  
  tiltServo.attach(TILT_PIN);
  panServo.attach(PAN_PIN);
  tiltServo.writeMicroseconds(tiltServoPos);
  panServo.writeMicroseconds(panServoPos);
    
  int maxTries = 3, numTries = 0;
  while (!initXBee() && ++numTries < maxTries); //Keep trying to put into transparent mode until failure

  //Setup the GPS
  setupGPS();

  DEBUG_PRINTLN("Done Communicator Initialize (includes GPS)");

}


bool Communicator::initXBee()
{
  // Initialize serial commuication to Xbee.
  XBEE_SERIAL.begin(XBEE_BAUD);  //this is to Xbee
  while (!XBEE_SERIAL); //wait until it's ready

  //Put into command mode, switch to transparent (not API) mode, then exit command mode
  if (!sendCmdAndWaitForOK("+++"))
  {
    DEBUG_PRINTLN("Failed on '+++'");
    return false;
  }

  if (!sendCmdAndWaitForOK("ATAP0\r"))
  {
    DEBUG_PRINTLN("Failed on 'ATAP0'");
    return false;
  }

  //sendCmdAndWaitForOK("ATAP\r");  //Should return '0' indicating transparent mode
  //sendCmdAndWaitForOK("ATID\r");  //To check network ID

  if (!sendCmdAndWaitForOK("ATCN\r"))
  {
    DEBUG_PRINTLN("Failed on 'ATCN'");
    return false;
  }

  DEBUG_PRINTLN("Successfully put into transparent mode");
  return true;  //if reached here, was successful
}



bool Communicator::sendCmdAndWaitForOK(String cmd, int timeout)
{
  //Flush input
  while (XBEE_SERIAL.read() != -1);

  //Send Command
  XBEE_SERIAL.print(cmd);

  //readStringUntil reads from serial until it encounters the termination character OR timeout (set below) is reached
  XBEE_SERIAL.setTimeout(timeout);  //sets it for the XBee serial accross the board
  String response = XBEE_SERIAL.readStringUntil('\r');  //until the carriage return terminates
  //NOTE - it does not include the terminator (it is removed from the string)

  DEBUG_PRINT("Response: ");
  DEBUG_PRINTLN(response);

  if (response.endsWith("OK"))
    return true;
  else
    return false;
}



// Function that is called from main program to receive incoming serial commands from ground station
// Commands are one byte long, represented as characters for easy reading
void Communicator::recieveCommands(unsigned long curTime) {

  // Look for new byte from serial buffer
  while (XBEE_SERIAL.available() > 0) {
    // New command detected, parse and execute
    byte incomingByte = XBEE_SERIAL.read();
    DEBUG_PRINT("Received a command: ");
    DEBUG_PRINTLN(incomingByte);

    // If we are currently in the middle of receiving a new GPS target
    if(bufferIndex > 0)
    {
      /*Serial.print(bufferIndex);
      Serial.print(": ");
      Serial.println((char)incomingByte);
      */
      // Note: If you want to notify the groundstation of a failed transmission, this check should move outside of the encapsualting if-statement
      if((curTime - transmitStartTime) > 2000) {
        // If it has been 2 seconds since the start character and we still aren't done,
        // then there was probably a transmit error. Just give up.
        // This is very unlikely, but I don't want to get stuck waiting for something that isn't coming
        // and miss meaningful messages
        bufferIndex = 0;
      }
      if(bufferIndex < 9) {
        targetLat[bufferIndex++ - 1] = incomingByte;
        continue; // Don't check if current byte matches a command - it might - but that's just a coincidence
      } else if(bufferIndex == 9) {
        if(incomingByte == '%') {
          // We're on the right track
          bufferIndex++;
          continue; // Don't check if current byte matches a command
        } else {
          // There was a transmission error
          // Give up on this message
          bufferIndex = 0;
        }
      } else if(bufferIndex > 9 && bufferIndex < 18) {
        targetLon[bufferIndex++ - 10] = incomingByte;
      } else if(bufferIndex == 18) {
        if(incomingByte == 'e') {
          // We succesfully received the new target
          // Convert lat and lon to doubles
          memcpy(&targetLatDoub, targetLat, 8);
          memcpy(&targetLonDoub, targetLon, 8);
          /*Serial.println("Longitude Double Bytes: ");
          for(int i = 0; i < 8; i++)
          {
            Serial.print(i);
            Serial.print(": ");
            Serial.println(((char*)&targetLonDoub)[i]);
          }*/
          // Update targeter
          // Note: Currently, it is assumed that the altitude is 0. This is wrong if the altimeter is reset at an altitude different than the target altitude
          targeter.setTargetData(targetLatDoub, targetLonDoub, 0);
          /*Serial.print("Latitude: ");
          Serial.println(targetLatDoub);
          Serial.print("Longitude: ");
          Serial.println(targetLonDoub);
          */
          sendMessage('g');
          bufferIndex = 0;
          continue; // Don't bother checking if the incomingByte matches any of the other codes - it doesn't
        }
        bufferIndex = 0; // Last byte didn't match - something got screwed up
      } else {
        // Should never get here
        bufferIndex = 0;
      }
        
    } // End if(bufferIndex > 0)

    if (incomingByte == INCOME_DROP_OPEN) {
      // Drop bay (Manual Drop)
      setDropBayState(MANUAL_CMD, DROPBAY_OPEN);
    } else if (incomingByte == INCOME_DROP_CLOSE) {
      setDropBayState(MANUAL_CMD, DROPBAY_CLOSE);
    } else if (incomingByte == INCOME_AUTO_ON) {
      // Turn ON auto drop
      autoDrop = true;
      sendMessage(MESSAGE_AUTO_ON);
    } else if (incomingByte == INCOME_AUTO_OFF) {
      // Turn OFF auto drop
      autoDrop = false;
      sendMessage(MESSAGE_AUTO_OFF);
    } else if (incomingByte == INCOME_RESET) {  //RESET FUNCTION.
      // Reset (only sensors, not drop bay??)
      sendData();  // Flush current data packets
      reset = true;
    } else if (incomingByte == INCOME_RESTART) {  //RESTART FUNCTION.
      // Restart
      sendData();  //Flush current data packets
      restart = true;
      setDropBayState(MANUAL_CMD, DROPBAY_CLOSE); //close drop bay
    } else if (incomingByte == INCOME_DROP_ALT) {  //SEND ALTITUDE_AT_DROP
      // Return Altitude at Drop
      sendMessage(MESSAGE_ALT_AT_DROP, (float)altitudeAtDropFt);
    } else if(incomingByte == INCOME_BATTERY_V){
       // Send Battery Voltage
       sendMessage(MESSAGE_BATTERY_V, (float)analogRead(BATTERY_VOLTAGE_PIN)*ANALOG_READ_CONV);
    } else if(incomingByte == INCOME_NEW_TARGET_START){
      // Start of a gps target position update message
      bufferIndex = 1;
      transmitStartTime = curTime;
    } else if(incomingByte == INCOME_CAM_TILT_UP || incomingByte == INCOME_CAM_TILT_DOWN || incomingByte == INCOME_CAM_PAN_LEFT || incomingByte == INCOME_CAM_PAN_RIGHT){
      moveCamera(incomingByte);
    } 

  } // End while(XBEE_SERIAL.available() > 0) 
} // End recieveCommands()


// Data is sent via wireless serial link to ground station
// data packet format:  *pALTITUDE%AIRSPEED%LATTITUDE%LONGITUDE%HEADING%ms%secondee
// Total number of bytes: 11 (*p% type) + ~ 50 (data) = 61 bytes *4x/second = 244 bytes/s.  Each transmission is under outgoing buffer (128 bytes) and baud
// Not anymore: now is bytewise transmission of floats
// This form is: *pAAAABBBBCCCCDDDDEEEEFFGee  AAAA = altitude flot, BBBB = spd, CCCC = latt, DDDD = long, EEEE = heading, FF = ms (uint16), G = s (uint8)  ee = end sequence
// Total Bytes: 27 (a little under half)
// No other serial communication can be done in other classes!!!
void Communicator::sendData() {

  /* For testing
    long maxRand = 1000000;
    altitudeFt = random(0,maxRand)*(150.0-0.0)/maxRand + 0.0;
    GPS.speedMPS = random(0,maxRand)*(20.0-5.0)/maxRand  + 5.0;
    float GPSerr = 0.1;
    GPS.latitude = random(0,maxRand)*GPSerr/maxRand  + TARGET_LATT - GPSerr/2;
    GPS.longitude = random(0,maxRand)*GPSerr/maxRand + TARGET_LONG - GPSerr/2;   //NOTE - pay attend to signs, this is negative (as it should be)
    GPS.angle = random(0,maxRand)*(360.0-0.0)/maxRand + 0.0;
    GPS.milliseconds = random(0,1000);
    GPS.seconds = random(0,60);  */


  //Send to XBee
  XBEE_SERIAL.print("*");
  XBEE_SERIAL.print(DATA_PACKET);
  sendFloat((float)altitudeFt);
  sendFloat(GPS.speedMPS);
  sendFloat(GPS.latitude);
  sendFloat(GPS.longitude);
  sendFloat(GPS.angle);
  sendFloat(GPS.HDOP);
  sendFloat((float)GPS.msSinceValidHDOP);
  sendFloat(GPS.altitudeMeters);
  sendUint8_t(GPS.fixquality);
  XBEE_SERIAL.print("ee");


  //If Debugging, send to Serial Monitor (Note this doesn't use the bytewise representation of numbers)
  /*
  DEBUG_PRINT("Message:");
  DEBUG_PRINT("Alt: ");
  DEBUG_PRINT(altitudeFt);
  DEBUG_PRINT("  Spd: ");
  DEBUG_PRINT(GPS.speedMPS);
  DEBUG_PRINT("  Latt: ");
  DEBUG_PRINT_PRECISION(GPS.latitude,5);
  DEBUG_PRINT("  Long: ");
  DEBUG_PRINT_PRECISION(GPS.longitude,5);
  DEBUG_PRINT("  HDOP: ");
  DEBUG_PRINT(GPS.HDOP);
  DEBUG_PRINT("  FixQual: ");
  DEBUG_PRINTLN(GPS.fixquality);*/


}

void Communicator::sendMessage(char message) {
  XBEE_SERIAL.print("*");
  XBEE_SERIAL.print(message);
  XBEE_SERIAL.print("ee");
}


void Communicator::sendMessage(char message, float value) // Messages with associated floats
{
  XBEE_SERIAL.print("*");
  XBEE_SERIAL.print(message);
  sendFloat((float)value);
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




/*********************** TARGETING CONTROL  *********************/

//This is called:
//1) From medium loop (with false) to repeatedly check for drop condition
//2) From in this class, once we parse a new GPS string
//3) If testing the targeter, within slow loop, which simulates new data being received
void Communicator::recalculateTargettingNow(boolean withNewData) {

  bool isReadyToDrop = false;

  //If testing the targeting, use the simulated data
#ifdef Targeter_Test

  if (withNewData) {
#ifndef Targeter_Debug_Print
    TARGET_PRINTLN("\t Targeting Test: Recalaculating Targeting with New Data (Advancing a Point) \t");
#endif

    if (++currentTargeterDataPoint == NUM_TARGETER_DATAPTS) {
      currentTargeterDataPoint = 0;
    }

    isReadyToDrop = targeter.setAndCheckCurrentData(GPSLatitudes[currentTargeterDataPoint], GPSLongitudes[currentTargeterDataPoint], altitudes[currentTargeterDataPoint], velocities[currentTargeterDataPoint], headings[currentTargeterDataPoint], millis(), true);  //HDOPOK = true for testing purposes
  }
  else {
#ifndef Targeter_Debug_Print
    TARGET_PRINT("Targeting Test: Projecting Data Foward");
#endif
    isReadyToDrop = targeter.recalculate();
  }

#else  //What we do when it's real GPS data

  if (withNewData) {
#ifndef Targeter_Debug_Print
    TARGET_PRINTLN("\t Real GPS: Recalaculating Targeting with New Data \t");
#endif
    isReadyToDrop = targeter.setAndCheckCurrentData(GPS.latitude, -GPS.longitude, altitudeFt, GPS.speedMPS, GPS.angle, millis(), GPS.HDOP_OK);

  }
  else {
#ifndef Targeter_Debug_Print
    TARGET_PRINT("Real GPS: Projecting Data Foward");
#endif
    isReadyToDrop = targeter.recalculate();
  }

#endif

  //Interpret result the same, regardless of if it was a testing run
  if (!isReadyToDrop)  {
#ifndef Targeter_Debug_Print
    TARGET_PRINTLN("\n NOT READY FOR DROP\n\n");
#endif
  }
  //Check if it's already open (ie. don't want to update/change altitudeAtDropFt)
  else if (dropBayServoPos == DROP_BAY_OPEN) {
    TARGET_PRINTLN("Dropbay already open (otherwise wanted to drop)");
  }
  //Check if autoDrop is enabled
  else if (!autoDrop) {
    TARGET_PRINTLN("\n\n AUTODROP DISABLED PREVENTING A DROP\n\n\n\n");
    //If reach here, targeter wants a drop, dropbay is closed, and autodrop is enabled. Therefore, we open the drop bay!
  }
  else {
    setDropBayState(AUTOMATIC_CMD, DROPBAY_OPEN);
    TARGET_PRINTLN("\n\n ******************************* AUTOMATIC DROP *****************\n\n\n\n");
  }



}


/********************  DROP BAY FUNCTIONS **********************/
//Function called by main program and receiveCommands function. Toggles Drop Bay
//src == 1 corresponds to the automatic drop function.
//state == 0 closes drop bay. state == 1 opens drop bay.
void Communicator::setDropBayState(int src, int state) {

  if (src == 1 && autoDrop == false) { //AutoDrop Protection

    TARGET_PRINT("Autodrop is disabled, preventing drop (src = ");
    TARGET_PRINT(src);
    TARGET_PRINT("   autoDrop = ");
    TARGET_PRINT(autoDrop);
    TARGET_PRINTLN(")");

    return;
  }

  //Open/Close Drop Bay
  if (state == DROPBAY_CLOSE)  {

    TARGET_PRINT("Closed bay door (src = ");
    TARGET_PRINT(src);
    TARGET_PRINT("   autoDrop = ");
    TARGET_PRINT(autoDrop);
    TARGET_PRINTLN(")");

    digitalWrite(STATUS_LED_PIN, LOW);
    dropBayServoPos = DROP_BAY_CLOSED;
    sendMessage(MESSAGE_DROP_CLOSE);
  }
  else {
#ifdef Targeter_Test
    TARGET_PRINT("****** DROPPED ****** (src = ");
    TARGET_PRINT(src);
    TARGET_PRINT("   autoDrop = ");
    TARGET_PRINT(autoDrop);
    TARGET_PRINTLN(")");
#endif
    digitalWrite(STATUS_LED_PIN, HIGH);
    dropBayServoPos = DROP_BAY_OPEN;
    altitudeAtDropFt = altitudeFt;
    timeAtDrop = millis();
    sendMessage(MESSAGE_DROP_OPEN);
  }

  dropServo.writeMicroseconds(dropBayServoPos);
}

// Function called in slow loop. If the drop bay is currently open, checks if
// 10 seconds has passed since it was opened. If so, closes it.
void Communicator::checkToCloseDropBay() {

  if (dropBayServoPos == DROP_BAY_OPEN) {

    unsigned long currentMillis = millis();

    if (currentMillis - timeAtDrop >= closeDropBayTimeout && currentMillis - timeAtDrop < closeDropBayTimeout + 10000) {

      TARGET_PRINT("Auto closing bay door (time passed = ");
      TARGET_PRINT((currentMillis - timeAtDrop));
      TARGET_PRINTLN(")");

      //TEMPORARY / TODO - RE-ENABLE THIS
      //setDropBayState(AUTOMATIC_CMD, DROPBAY_CLOSE);
    }
  }
}

/************CAMERA MOVEMENT*********************/
//Function serves to move the camera on the gimble in the tilt or pan directions.
void Communicator::moveCamera(char orientation) {
  
  if (orientation == INCOME_CAM_TILT_UP) {          //Tilt up
    if (tiltServoPos < 2400) {
      tiltServoPos += TILT_INCREMENT;
    } 
    tiltServo.writeMicroseconds(tiltServoPos);
  } 
  else if (orientation == INCOME_CAM_TILT_DOWN) {  //Tilt down
    if (tiltServoPos > 600) {
      tiltServoPos -= TILT_INCREMENT;
    }
    tiltServo.writeMicroseconds(tiltServoPos);
  } 
  else if (orientation == INCOME_CAM_PAN_LEFT) {   //Pan left
    if (panServoPos > 600) {
      panServoPos -= PAN_INCREMENT;
    }
    panServo.writeMicroseconds(panServoPos);
  } 
  else if (orientation == INCOME_CAM_PAN_RIGHT) {  //Pan right
    if (panServoPos < 2400) {
      panServoPos += PAN_INCREMENT;
    }
    panServo.writeMicroseconds(panServoPos);
  }
  Serial.println(tiltServoPos);
  
}


/***********GPS FUNCTIONALITY  *******/
void Communicator::setupGPS() {

  DEBUG_PRINTLN("GPS Initilization...");

  // Initialize the variables in GPS class object
  GPS.init();

  // Start the serial communication
  GPS_SERIAL.begin(GPS_BAUD);
  DEBUG_PRINTLN("Begin Setting GPS:");

  
  //Settings should persist over power off, but safer to reset each time
  // Commands to configure GPS: (each involves setting, flushing out previous data, then checking a correct return string received
  if(!sendGPSConfigureCommands())
  {
    //Try again:
    if(!sendGPSConfigureCommands())
    {
      //TODO setup message to tell ground station
      
    }    
  }

}



void Communicator::getSerialDataFromGPS() {

  while (GPS_SERIAL.available()) {

    nmeaBuf[nmeaBufInd] = GPS_SERIAL.read();

    if (nmeaBuf[nmeaBufInd++] == '\n') { // Increment index after checking if current character signifies the end of a string
      nmeaBuf[nmeaBufInd - 1] = '\0'; // Add null terminating character (note: -1 is because nmeaBufInd is incremented in if statement)
      newParsedData = GPS.parse(nmeaBuf);   // This parses the string, and updates the values of GPS.lattitude, GPS.longitude etc.
      nmeaBufInd = 0;  // Regardless of it parsing sucessful, we want to reset position back to zero
      //Potential flaw - the string length is used in parsing. By only setting index to 0, it may keep null terminating character, giving false future readings?
    
      if (noFixLedIsOn == GPS.fix) {
        digitalWrite(NO_FIX_LED_PIN, !GPS.fix);
        noFixLedIsOn = !GPS.fix;
      }

#ifndef Targeter_Test  //Otherwise may confuse real data and simulated data
      recalculateTargettingNow(true);
#endif

    }

    if (nmeaBufInd >= MAXLINELENGTH) { // Should never happen. Means a corrupted packed and the newline was missed. Good to have just in case
      nmeaBufInd = 0;  // Note the next packet will then have been corrupted as well. Can't really recover until the next-next packet
    }

  }

}


bool Communicator::sendGPSConfigureCommands()
{
  
  // Stop updates (before this, cannot accurately receive responses to commands
  GPS_SERIAL.println(SET_SERIAL_UPDATE_RATE_0HZ);
  delay(1000);
  flushGPSSerial();
  int check = 2, errorLocation = 1;
  
  while(check) {
	  // Repeat send the "stop update" command. Only this time, we should be able to check it was successfull
	  if(errorLocation == 1) {
		  GPS_SERIAL.println(SET_SERIAL_UPDATE_RATE_0HZ);
		  if(!checkReturnString(SET_SERIAL_UPDATE_RATE_0HZ_COMMANDNUM)) {  
			check--;
			flushGPSSerial();
			continue; 
			} 
			else {
			check = 2;
			errorLocation++; 
			//delay(1000);
			flushGPSSerial();}
	  }

	  // Set the output to RMC and GGA
	  if(errorLocation == 2) {
		  GPS_SERIAL.println(PMTK_SET_NMEA_OUTPUT_RMCGGA);
		  if(!checkReturnString(PMTK_SET_NMEA_OUTPUT_RMCGGA_COMMANDNUM)) {  
			check--;
			continue; } 
			else {
			check = 2;
			errorLocation++; }
	  }
	  // Increase rate GPS 'connects' and syncs with satellites
	  if(errorLocation == 3)  {
		  GPS_SERIAL.println(SET_FIX_RATE_5HZ);     
		  if(!checkReturnString(SET_FIX_RATE_5HZ_COMMANDNUM)) {  
			check--;
			continue; } 
			else {
			check = 2;
			errorLocation++; }
	  }
	  // Enable using a more accurate type of satellite
	  if(errorLocation == 4) {
		  GPS_SERIAL.println(ENABLE_SBAS_SATELLITES);       
		  if(!checkReturnString(ENABLE_SBAS_SATELLITES_COMMANDNUM)) {  
			 check--;
			 continue;} 
			 else {
			 check = 2;
			 errorLocation++; }
	  }
	  // Enable using the more accurate satellite to get a better fix
	  if(errorLocation == 5) {
		  GPS_SERIAL.println(ENABLE_USING_WAAS_WITH_SBAS_SATS);   
		  if(!checkReturnString(ENABLE_USING_WAAS_WITH_SBAS_SATS_COMMANDNUM)) {  
			check--;
			continue;} 
			else {
			check = 2;
			errorLocation++; }
	  }	  
	  // Increase rate strings sent over serial (was previously set to 0Hz)
	  if(errorLocation == 6) {
		  GPS_SERIAL.println(SET_SERIAL_UPDATE_RATE_5HZ);     
		  if(!checkReturnString(SET_SERIAL_UPDATE_RATE_5HZ_COMMANDNUM)) {  
			check--;
			continue;} 
			else { break;}
	  }
  }
  if(!check)
	  return false;
  //If got here, successful
  return true;  
}


//$PMTK001,<commandNum>,<success?>*32<CR><LF> is format
//Success -> 0 = Invalid Command/Packet,  1 = Unsupported Command/packet,  2 = Valid Command, action failed,  3 = Success
//For this function, anything other than 3 is considered failure
bool Communicator::checkReturnString(int commandNum)
{
  //Get the return string
  unsigned long startT = millis();
  unsigned long maxT = 1000;  //1 second timeout
  int receivedIndex = 0, maxLength = 25;  //typical is 18 I believe
  char returnString[maxLength];
  int newChar;
  bool gotPacket = false;

  
  while((millis() - startT) < maxT && receivedIndex < maxLength)
  {   
    if(GPS_SERIAL.available() > 0)
    {
      returnString[receivedIndex] = GPS_SERIAL.read();

      //Check for end of string
      if (returnString[receivedIndex++] == '\n') // Increment index after checking if current character signifies the end of a string
      {
        returnString[receivedIndex - 1] = '\0'; // Add null terminating character (note: -1 is because nmeaBufInd is incremented in if statement)
        gotPacket = true;
        break;
      }
    }      
  }

  if(!gotPacket)  //If we didn't get a packet, don't bother trying below
  {
    DEBUG_PRINTLN("Did not get a packet when waiting for GPS response");
    return false;
  }
  
  // Check for valid checksum
  if (returnString[strlen(returnString) - 4] == '*') 
  {
    uint16_t sum = GPS.parseHex(returnString[strlen(returnString) - 3]) * 16;
    sum += GPS.parseHex(returnString[strlen(returnString) - 2]);

    
    // Check checksum
    // NOTE the starting at i=1!! based on how their code worked, the first the string actually begin as:  s[0] = '\n', s[1] = $, s[2] = 'G'  ie. the checksum characters start at i=2 index, not i=1 as would be expected
    // In the original library code this for loop began at i=2 to account for:they set the index back to 0, then immidately 'added' the current characer (the newline) at index 0, then the next string began with $ at s[1]
    // Be wary of how the string is sent!
    for (uint8_t i = 1; i < (strlen(returnString) - 4); i++) {
      sum ^= returnString[i];
    }

    if (sum != 0) { // Bad checksum
      
      DEBUG_PRINT("Bad Checksum -> sum != 0. (Sum = ");
      DEBUG_PRINTLN(sum);
      
      return false;
    }
  }
  else
  {
    DEBUG_PRINTLN("Asterix in wrong spot");
    return false;  //we don't have one
  }
  DEBUG_PRINTLN(returnString);
  //Valid string, check it's the right command
  char *ind = returnString;  //set pointer equal to start
  ind = strchr(ind, ',') + 1;  //Move pointer to just past first command

  //Extract command number and compare it
  int commandNumReturn = atoi(ind);
  if(commandNumReturn != commandNum)
  {
    DEBUG_PRINT("Command Num Wrong: ");
    DEBUG_PRINT(commandNumReturn);
    DEBUG_PRINT("\t(Should be ");
    DEBUG_PRINTLN(commandNum);
    return false;
  }


  //Extract "success" and check it's 3
  ind = strchr(ind, ',') + 1;  //Move pointer to past second comma
  int successReturn = atoi(ind);
  if(successReturn != 3)
  {
    DEBUG_PRINT("Success number = ");
    DEBUG_PRINTLN(successReturn);
    return false;
  }

  DEBUG_PRINT("GPS Successfull command - Number = "); DEBUG_PRINTLN(commandNum);

  //Getting here means we're good 
  return true;  
}

void Communicator::flushGPSSerial()
{
  delay(200);
  char hold;
  int numBytes = GPS_SERIAL.available();

  //DEBUG_PRINT("Flushed Bytes: ");
  for(int i=0;i<numBytes;i++)  {
    hold = (char)GPS_SERIAL.read();
    //DEBUG_PRINT(hold);
  }
  
  //DEBUG_PRINTLN("\t End Flushed Bytes");


}





