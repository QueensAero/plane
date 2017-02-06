/*

  This class deals with all serial communication. This includes sending and receiving data from the XBee, as well as sending commands and receiving
  data from the GPS. The GPS class is only called from here
  It also controls the servos for the dropbay, since the "drop payload" command will be received directly by this class and it's easier then having another class.

*/

//TODO
//Code to receive a command to update target location based on ground station message (is this useful??)
//Code for commands to enable/disable in flight mode  (ie. disable pushbuttons once takeoff)
//Code to program flaps to 'brake' mode (Ask mech if useful)



#include "Arduino.h"
#include "Communicator.h"
#include <Servo.h>
#include "Targeter.h"


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

  //Attach servo, init position to closed
  dropServo.attach(DROP_PIN);
  dropBayServoPos = DROP_BAY_CLOSED;
  dropServo.writeMicroseconds(dropBayServoPos);

  int maxTries = 3, numTries = 0;
  while (!initXBee() && ++numTries < maxTries); //Keep trying to put into transparent mode until failure

  //Setup the GPS
  setupGPS();

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
void Communicator::recieveCommands() {

  // Look for new byte from serial buffer
  if (XBEE_SERIAL.available() > 0) {
    // New command detected, parse and execute
    byte incomingByte = XBEE_SERIAL.read();
    DEBUG_PRINT("Received a command: ");
    DEBUG_PRINT(incomingByte);

    // Drop bay (Manual Drop)
    if (incomingByte == INCOME_DROP_OPEN)
      setDropBayState(MANUAL_CMD, DROPBAY_OPEN);
    
    if (incomingByte == INCOME_DROP_CLOSE)
      setDropBayState(MANUAL_CMD, DROPBAY_CLOSE);

    // Turn ON auto drop
    if (incomingByte == INCOME_AUTO_ON) {
      autoDrop = true;
      sendMessage(MESSAGE_AUTO_ON);
    }

    // Turn OFF auto drop
    if (incomingByte == INCOME_AUTO_OFF) {
      autoDrop = false;
      sendMessage(MESSAGE_AUTO_OFF);
    }
    
    // Reset (only sensors, not drop bay??)
    if (incomingByte == INCOME_RESET) {  //RESET FUNCTION.
      sendData();  // Flush current data packets
      reset = true;
    }

    //Restart
    if (incomingByte == INCOME_RESTART) {  //RESTART FUNCTION.
      sendData();  //Flush current data packets
      restart = true;
      setDropBayState(MANUAL_CMD, DROPBAY_CLOSE); //close drop bay
    }

    //Return Altitude at Drop
    if (incomingByte == INCOME_DROP_ALT) {  //SEND ALTITUDE_AT_DROP
      sendMessage(MESSAGE_ALT_AT_DROP, (float)altitudeAtDropFt);
    }

    //Send Battery Voltage
    if(incomingByte == INCOME_BATTERY_V){
       sendMessage(MESSAGE_BATTERY_V, (float)analogRead(BATTERY_VOLTAGE_PIN)*ANALOG_READ_CONV);
    }

  }
}


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
  sendUint16_t(GPS.milliseconds);
  sendUint8_t(GPS.seconds);
  XBEE_SERIAL.print("ee");

  
  //If Debugging, send to Serial Monitor (Note this doesn't use the bytewise representation of numbers)
  DEBUG_PRINT("Message:");
  DEBUG_PRINT("Alt: ");
  DEBUG_PRINT(altitudeFt);
  DEBUG_PRINT("  Spd: ");
  DEBUG_PRINT(GPS.speedMPS);
  DEBUG_PRINT("  Latt: ");
  DEBUG_PRINT(GPS.latitude);
  DEBUG_PRINT("  Long: ");
  DEBUG_PRINT(GPS.longitude);
  DEBUG_PRINT("  ms: ");
  DEBUG_PRINT(GPS.milliseconds);
  DEBUG_PRINT("  s: ");
  DEBUG_PRINTLN(GPS.seconds);

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

  if(withNewData) {
  #ifndef Targeter_Debug_Print
    TARGET_PRINTLN("\t Targeting Test: Recalaculating Targeting with New Data (Advancing a Point) \t");
  #endif
  
    if (++currentTargeterDataPoint == NUM_TARGETER_DATAPTS) {      currentTargeterDataPoint = 0;   }

    isReadyToDrop = targeter.setAndCheckCurrentData(GPSLatitudes[currentTargeterDataPoint], GPSLongitudes[currentTargeterDataPoint], altitudes[currentTargeterDataPoint], velocities[currentTargeterDataPoint], headings[currentTargeterDataPoint], millis());
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
    isReadyToDrop = targeter.setAndCheckCurrentData(GPS.latitude, -GPS.longitude, altitudeFt, GPS.speedMPS, GPS.angle, millis());

  }
  else {
    #ifndef Targeter_Debug_Print
      TARGET_PRINT("Real GPS: Projecting Data Foward");
    #endif
    isReadyToDrop = targeter.recalculate();   
  }

#endif 

  //Interpret result the same, regardless of if it was a testing run
  if(!isReadyToDrop)  {
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
  else{
    setDropBayState(AUTOMATIC_CMD, DROPBAY_OPEN);
    TARGET_PRINTLN("\n\n ******************************* AUTOMATIC DROP *****************\n\n\n\n");
  }



}


/********************  DROP BAY FUNCTIONS **********************/
//Function called by main program and receiveCommands function. Toggles Drop Bay
//src == 1 corresponds to the automatic drop function.
//state == 0 closes drop bay. state == 1 opens drop bay.
void Communicator::setDropBayState(int src, int state) {
  
  if (src == 1 && autoDrop == false){ //AutoDrop Protection  
    
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

      setDropBayState(AUTOMATIC_CMD, DROPBAY_CLOSE);
    }
  }
}




/***********GPS FUNCTIONALITY  *******/
void Communicator::setupGPS() {

  DEBUG_PRINTLN("GPS Initilization...");

  // Initialize the variables in GPS class object
  GPS.init();

  // Start the serial communication
  GPS_SERIAL.begin(GPS_BAUD);

  //Settings should persist over power off, but having problems with that - so set these
  //TODO - check for responses, not blindly hope it worked. Seems to not send response (ignore working on this for now...)
  // Commands to configure GPS:
  GPS_SERIAL.println(PMTK_SET_NMEA_OUTPUT_RMCONLY);     // Set to only output GPRMC (has all the info we need),
  GPS_SERIAL.println(SET_NMEA_UPDATE_RATE_5HZ);     // Increase rate strings sent over serial
  GPS_SERIAL.println(PMTK_API_SET_FIX_CTL_5HZ);     // Increase rate GPS 'connects' and syncs with satellites
  GPS_SERIAL.println(ENABLE_SBAB_SATELLITES);       // Enable using a more accurate type of satellite
  GPS_SERIAL.println(ENABLE_USING_WAAS_WITH_SBSB_SATS);   // Enable the above satellites in 'fix' mode (I think)
  delay(1500);  //Not really sure if needed.

}

void Communicator::getSerialDataFromGPS() {

  while (GPS_SERIAL.available()) {
     
    nmeaBuf[nmeaBufInd] = GPS_SERIAL.read();

    if (nmeaBuf[nmeaBufInd++] == '\n') { // Increment index after checking if current character signifies the end of a string
      nmeaBuf[nmeaBufInd - 1] = '\0'; // Add null terminating character (note: -1 is because nmeaBufInd is incremented in if statement)
      newParsedData = GPS.parse(nmeaBuf);   // This parses the string, and updates the values of GPS.lattitude, GPS.longitude etc.
      nmeaBufInd = 0;  // Regardless of it parsing sucessful, we want to reset position back to zero

      #ifndef Targeter_Test  //Otherwise may confuse real data and simulated data
        recalculateTargettingNow(true);
        //String str(nmeaBuf);
        //DEBUG_PRINTLN(str);
      #endif

    }

    if (nmeaBufInd >= MAXLINELENGTH) { // Should never happen. Means a corrupted packed and the newline was missed. Good to have just in case
      nmeaBufInd = 0;  // Note the next packet will then have been corrupted as well. Can't really recover until the next-next packet
    }

  }

}


