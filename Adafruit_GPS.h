/***********************************
  This is the Adafruit GPS library - the ultimate GPS library
  for the ultimate GPS module!

  Tested and works great with the Adafruit Ultimate GPS module
  using MTK33x9 chipset
    ------> http://www.adafruit.com/products/746
  Pick one up today at the Adafruit electronics shop
  and help support open source hardware & software! -ada

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada  for Adafruit Industries.
  BSD license, check license.txt for more information
  All text above must be included in any redistribution
****************************************/
// Fllybob added lines 34,35 and 40,41 to add 100mHz logging capability

#ifndef _ADAFRUIT_GPS_H
#define _ADAFRUIT_GPS_H


//----- USED COMMANDS --------/ (as of March 28, 2017)
//note print'ln' adds the <CR><LF> part automatically in sendCommand function
#define SET_SERIAL_UPDATE_RATE_0HZ "$PMTK220,10000*2F"  //Used during setting to pause output of the other strings
#define SET_SERIAL_UPDATE_RATE_0HZ_COMMANDNUM 220 

#define SET_SERIAL_UPDATE_RATE_5HZ "$PMTK220,200*2C"
#define SET_SERIAL_UPDATE_RATE_5HZ_COMMANDNUM 220 

#define SET_FIX_RATE_5HZ "$PMTK300,200,0,0,0,0*2F"    //Note 5Hz is max fix rate (so no point having higher serial update rate)
#define SET_FIX_RATE_5HZ_COMMANDNUM 300 

#define ENABLE_SBAS_SATELLITES "$PMTK313,1*2E"  //allow searching for more accurate SBAB sattelites
#define ENABLE_SBAS_SATELLITES_COMMANDNUM 313 

#define ENABLE_USING_WAAS_WITH_SBAS_SATS "$PMTK301,2*2E"  //Must be after ^. Allow using those sattelites to get 3d DGPS fix (using WAAS)
#define ENABLE_USING_WAAS_WITH_SBAS_SATS_COMMANDNUM 301 

#define PMTK_SET_NMEA_OUTPUT_RMCGGA "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
#define PMTK_SET_NMEA_OUTPUT_RMCGGA_COMMANDNUM 314 


//TO CREATE NEW COMMANDS
//Use the manuals to determine the command
//Use http://www.hhhh.org/wiml/proj/nmeaxor.html  to find checksum. Ensure leading '$' and trailing '*' are not in the space there


/* UNUSED (some may be duplicate to above, so commented out)
// different commands to set the update rate from once a second (1 Hz) to 10 times a second (10Hz)
// Note that these only control the rate at which the position is echoed, to actually speed up the
// position fix you must also send one of the position fix rate commands below too.
#define PMTK_SET_NMEA_UPDATE_1HZ  "$PMTK220,1000*1F"
#define PMTK_SET_NMEA_UPDATE_5HZ  "$PMTK220,200*2C"
#define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,100*2F"
// Position fix update rate commands.
#define PMTK_API_SET_FIX_CTL_1HZ  "$PMTK300,1000,0,0,0,0*1C"
#define PMTK_API_SET_FIX_CTL_5HZ  "$PMTK300,200,0,0,0,0*2F"
// Can't fix position faster than 5 times a second!

#define PMTK_SET_BAUD_115200 "$PMTK251,115200*1F"  
#define PMTK_SET_BAUD_57600 "$PMTK251,57600*2C"
#define PMTK_SET_BAUD_9600 "$PMTK251,9600*17"

// turn on only the second sentence (GPRMC)
#define PMTK_SET_NMEA_OUTPUT_RMCONLY "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
// turn on GPRMC and GGA
#define PMTK_SET_NMEA_OUTPUT_RMCGGA "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
// turn on ALL THE DATA
#define PMTK_SET_NMEA_OUTPUT_ALLDATA "$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
// turn off output
#define PMTK_SET_NMEA_OUTPUT_OFF "$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"

#define PMTK_LOCUS_STARTLOG  "$PMTK185,0*22"
#define PMTK_LOCUS_STOPLOG "$PMTK185,1*23"
#define PMTK_LOCUS_STARTSTOPACK "$PMTK001,185,3*3C"
#define PMTK_LOCUS_QUERY_STATUS "$PMTK183*38"
#define PMTK_LOCUS_ERASE_FLASH "$PMTK184,1*22"
#define LOCUS_OVERLAP 0
#define LOCUS_FULLSTOP 1

// standby command & boot successful message
#define PMTK_STANDBY "$PMTK161,0*28"
#define PMTK_STANDBY_SUCCESS "$PMTK001,161,3*36"  // Not needed currently
#define PMTK_AWAKE "$PMTK010,002*2D"

// ask for the release and version
#define PMTK_Q_RELEASE "$PMTK605*31"

// request for updates on antenna status
#define PGCMD_ANTENNA "$PGCMD,33,1*6C"
#define PGCMD_NOANTENNA "$PGCMD,33,0*6D"  
*/


#define KNOTS_TO_METERS_PS 0.514444


// how long to wait when we're looking for a response
#define MAXWAITSENTENCE 5

#include "Arduino.h"

class Adafruit_GPS {
  public:

    //constructors
    Adafruit_GPS();

    //functions still used
    void init();
    uint8_t parseHex(char c);
    boolean parse(char *);

    uint8_t hour, minute, seconds, year, month, day;
    uint16_t milliseconds;
    // Floating point latitude and longitude value in degrees.
    float latitude, longitude;
    // Fixed point latitude and longitude value with degrees stored in units of 1/100000 degrees,
    // and minutes stored in units of 1/100000 degrees.  See pull #13 for more details:
    //   https://github.com/adafruit/Adafruit-GPS-Library/pull/13
    int32_t latitude_fixed, longitude_fixed, msSinceValidHDOP, timeLastValidHDOP;
    float latitudeDegrees, longitudeDegrees;
    float geoidheight, altitude;
    float speedKnots, speedMPS, angle, magvariation, HDOP;
    char lat, lon, mag;
    boolean fix, HDOP_OK;
    uint8_t fixquality, satellites;
	


};

#endif
