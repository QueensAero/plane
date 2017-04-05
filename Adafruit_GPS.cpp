/***********************************
  This is our GPS library

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, check license.txt for more information
  All text above must be included in any redistribution
****************************************/

#include "Adafruit_GPS.h"

/*General Operation Notes (Ryan Dowling)

  This is significantly changed from library implementation.  Summary of changes:
  - Moved all serial communication with GPS into communicator.cpp, for more flexibility
  - Removed parsing of strings other than GPRMC (since we only use GPRMC)
  - Altered parsing function - see comment in it about checksum now starting from buffer index 1 (instead 2)
  - The raw NMEA string is now housed in communicator
  - This class essentially only functions by being passed a raw NMEA string to be parsed. It takes care of checksum.

*/

Adafruit_GPS::Adafruit_GPS() {
  //empty constructor (all initialization done in the init function)
}


// Initialization code
void Adafruit_GPS::init() {

  // Initialize state variables
  hour = minute = seconds = year = month = day = fixquality = satellites = 0; // uint8_t
  lat = lon = mag = 0; // char
  fix = false; // boolean
  milliseconds = 0; // uint16_t
  latitude = longitude = geoidheight = altitudeMeters =	speedKnots = speedMPS = angle = magvariation = 0.0; // float
  lat = lon = '0';
  HDOP = 5.0; //Ensures the HDOP initializes to 'bad'

}

// Parse the incoming serial data from the GPS
boolean Adafruit_GPS::parse(char *nmea) {
  // Do checksum check

  // First look if we even have one
  if (nmea[strlen(nmea) - 4] == '*') {
    uint16_t sum = parseHex(nmea[strlen(nmea) - 3]) * 16;
    sum += parseHex(nmea[strlen(nmea) - 2]);

    // Check checksum
    // NOTE the starting at i=1!! based on how their code worked, the first the string actually begin as:  s[0] = '\n', s[1] = $, s[2] = 'G'  ie. the checksum characters start at i=2 index, not i=1 as would be expected
    // In the original library code this for loop began at i=2 to account for:they set the index back to 0, then immidately 'added' the current characer (the newline) at index 0, then the next string began with $ at s[1]
    // Be wary of how the string is sent!
    for (uint8_t i = 1; i < (strlen(nmea) - 4); i++) {
      sum ^= nmea[i];
    }

    if (sum != 0) { // Bad checksum
      return false;
    }
  }

  int32_t degree;
  long minutes;
  char degreebuff[10];

  

  //------------------------ GPRMC parsing ------------------ /
  if (strstr(nmea, "$GPRMC")) {
    // Found RMC
    char *p = nmea;

    // Get time
    p = strchr(p, ',') + 1;
    float timef = atof(p);
    uint32_t time = timef;
    hour = time / 10000;
    minute = (time % 10000) / 100;
    seconds = (time % 100);
    milliseconds = fmod(timef, 1.0) * 1000;

    p = strchr(p, ',') + 1;
    if (p[0] == 'A') {
      fix = true;
    }
    else if (p[0] == 'V') {
      fix = false;
    }
    else {
      return false;
    }

    // Parse out latitude
    p = strchr(p, ',') + 1;
    if (',' != *p) { // Double comma would indicate we don't have a lattitude
      strncpy(degreebuff, p, 2);
      p += 2;
      degreebuff[2] = '\0';
      long degree = atol(degreebuff) * 10000000;
      strncpy(degreebuff, p, 2); // minutes
      p += 3; // Skip decimal point
      strncpy(degreebuff + 2, p, 4);
      degreebuff[6] = '\0';
      long minutes = 50 * atol(degreebuff) / 3;

      // A couple ways of representing lattitude - pure degrees, degreeminutes and stuff.  Need to decide which format. Same below with Longitude
      //latitude_fixed = degree + minutes;
      latitude = degree / 100000 + minutes * 0.000006F;
      //latitudeDegrees = (latitude-100*int(latitude/100))/60.0;
      //latitudeDegrees += int(latitude/100);
    }

    p = strchr(p, ',') + 1;
    if (',' != *p) {
      //if (p[0] == 'S') latitudeDegrees *= -1.0;
      if (p[0] == 'N') {
        lat = 'N';
      }
      else if (p[0] == 'S') {
        lat = 'S';
      }
      else if (p[0] == ',') {
        lat = 0;
      }
      else {
        return false;
      }
    }

    // parse out longitude
    p = strchr(p, ',') + 1;
    if (',' != *p) {
      strncpy(degreebuff, p, 3);
      p += 3;
      degreebuff[3] = '\0';
      degree = atol(degreebuff) * 10000000;
      strncpy(degreebuff, p, 2); // minutes
      p += 3; // skip decimal point
      strncpy(degreebuff + 2, p, 4);
      degreebuff[6] = '\0';
      minutes = 50 * atol(degreebuff) / 3;
      //longitude_fixed = degree + minutes;
      longitude = degree / 100000 + minutes * 0.000006F;
    }

    p = strchr(p, ',') + 1;
    if (',' != *p) {
      if (p[0] == 'W') {
        lon = 'W';
        //longitudeDegrees *= -1.0;
      }
      else if (p[0] == 'E') {
        lon = 'E';
      }
      else if (p[0] == ',') {
        lon = 0;
      }
      else {
        return false;
      }
    }
    // speed
    p = strchr(p, ',') + 1;
    if (',' != *p) {
      speedKnots = atof(p);
      speedMPS = speedKnots*KNOTS_TO_METERS_PS;
    }

    // angle
    p = strchr(p, ',') + 1;
    if (',' != *p) {
      angle = atof(p);
    }

    p = strchr(p, ',') + 1;
    if (',' != *p) {
      uint32_t fulldate = atof(p);
      day = fulldate / 10000;
      month = (fulldate % 10000) / 100;
      year = (fulldate % 100);
    }
    // we dont parse the remaining, yet!
    return true;
  }  // End of  if (strstr(nmea, "$GPRMC"))




  
  //--------------------- GGA ----------------------------/
  if (strstr(nmea, "$GPGGA")) {
    //Serial.println("GGA exists.");
    // found GGA
    char *p = nmea;
    // get time
    p = strchr(p, ',') + 1;
    float timef = atof(p);
    uint32_t time = timef;
    hour = time / 10000;
    minute = (time % 10000) / 100;
    seconds = (time % 100);

    milliseconds = fmod(timef, 1.0) * 1000;

    // parse out latitude
    p = strchr(p, ',') + 1;
    if (',' != *p)
    {
      strncpy(degreebuff, p, 2);
      p += 2;
      degreebuff[2] = '\0';
      degree = atol(degreebuff) * 10000000;
      strncpy(degreebuff, p, 2); // minutes
      p += 3; // skip decimal point
      strncpy(degreebuff + 2, p, 4);
      degreebuff[6] = '\0';
      minutes = 50 * atol(degreebuff) / 3;
      latitude_fixed = degree + minutes;
      latitude = degree / 100000 + minutes * 0.000006F;
      latitudeDegrees = (latitude - 100 * int(latitude / 100)) / 60.0;
      latitudeDegrees += int(latitude / 100);
    }

    p = strchr(p, ',') + 1;
    if (',' != *p)
    {
      if (p[0] == 'S') latitudeDegrees *= -1.0;
      if (p[0] == 'N') lat = 'N';
      else if (p[0] == 'S') lat = 'S';
      else if (p[0] == ',') lat = 0;
      else return false;
    }

    // parse out longitude
    p = strchr(p, ',') + 1;
    if (',' != *p)
    {
      strncpy(degreebuff, p, 3);
      p += 3;
      degreebuff[3] = '\0';
      degree = atol(degreebuff) * 10000000;
      strncpy(degreebuff, p, 2); // minutes
      p += 3; // skip decimal point
      strncpy(degreebuff + 2, p, 4);
      degreebuff[6] = '\0';
      minutes = 50 * atol(degreebuff) / 3;
      longitude_fixed = degree + minutes;
      longitude = degree / 100000 + minutes * 0.000006F;
      longitudeDegrees = (longitude - 100 * int(longitude / 100)) / 60.0;
      longitudeDegrees += int(longitude / 100);
    }

    p = strchr(p, ',') + 1;
    if (',' != *p)
    {
      if (p[0] == 'W') longitudeDegrees *= -1.0;
      if (p[0] == 'W') lon = 'W';
      else if (p[0] == 'E') lon = 'E';
      else if (p[0] == ',') lon = 0;
      else return false;
    }

    p = strchr(p, ',') + 1;
    if (',' != *p)
    {
      fixquality = atoi(p);
    }

    p = strchr(p, ',') + 1;
    if (',' != *p)
    {
      satellites = atoi(p);
    }

    p = strchr(p, ',') + 1;
    if (',' != *p)
    {
      HDOP = atof(p);
    }

    p = strchr(p, ',') + 1;
    if (',' != *p)
    {
      altitudeMeters = atof(p);
    }

    p = strchr(p, ',') + 1;
    p = strchr(p, ',') + 1;
    if (',' != *p)
    {
      geoidheight = atof(p);
    }

    //Updates HDOP OK variable
    HDOPCheck();
    
    return true;
  }
  

  return false;
}

// Read a Hex value and return the decimal equivalent
uint8_t Adafruit_GPS::parseHex(char c) {
  if (c < '0') {
    return 0;
  }
  if (c <= '9') {
    return c - '0';
  }
  if (c < 'A') {
    return 0;
  }
  if (c <= 'F') {
    return (c - 'A') + 10;
  }
  // if (c > 'F')
  return 0;
}


//Checks whether the current fix is accurate enough to autodrop
void Adafruit_GPS::HDOPCheck() {
	if(HDOP > 3) { //If accuracy is not high enough
		msSinceValidHDOP = millis()-timeLastValidHDOP;
		HDOP_OK = msSinceValidHDOP < 5000;
	} else {
		timeLastValidHDOP = millis();
		msSinceValidHDOP = 0;
    HDOP_OK = true;
	}
}


