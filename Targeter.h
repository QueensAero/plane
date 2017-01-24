#ifndef _TARGETER_H
#define _TARGETER_H

#include "Arduino.h"

class Targeter {
  
  public:
    Targeter();

    boolean recalculate(void);

    // ------------------------------------ GETTERS ------------------------------------

    double getLateralError(void);
    double getTimeToDrop(void);

    // ------------------------------------ SETTERS ------------------------------------

    boolean setCurrentData(double, double, double, double, double, double);
    void setTargetData(double, double, double);

  private:

    const double CORRECTION_FACTOR = 0.9;

    // ------------------------------------ CURRENT POSITION ------------------------------------

    // Format: dd° mm.mmmm'
    double currentLatitude = 0;
    double currentLongitude = 0;

    double currentAltitude = 0; // m

    double currentDataAge = 0; // millis

    double currentVelocity = 0;
    double currentHeading = 0; // In degrees (E = 0, N = 90, W = 180, S = 270)

    // ------------------------------------ TARGET POSITION ------------------------------------

    // Format: dd° mm.mmmm'
    double targetLatitude = 0;
    double targetLongitude = 0;

    double targetEasting;
    double targetNorthing;

    double targetAltitude = 0; // m

    // ------------------------------------ CALCULATED POSITION ------------------------------------

    // Calculated values
    double lateralError;
    double timeToDrop;

    // ------------------------------------ PHYSICAL CALCULATIONS ------------------------------------

    double calculateLateralError(void);
    double calculateDirectDistanceToTarget(void);
    double calculatePathDistanceToTarget(void);
    double calculateDropDistance(void);
    double calculateTimeToDrop(void);
    double dropDistanceToTarget(void);

    // ------------------------------------ CONVERT LAT/LON DEGREES TO UTM COORDINATES ------------------------------------
    
    double convertHeadingToMathAngle(double);
    double convertDecimalDegMinToDegree(double);
    void convertDeg2UTM(double, double, double &, double &);

};

#endif //_TARGETER_H
