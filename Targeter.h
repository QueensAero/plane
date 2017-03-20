#ifndef _TARGETER_H
#define _TARGETER_H

#include "Arduino.h"

#define FT_TO_METERS 0.3048

class Targeter {
  
  public:
    Targeter();
    boolean recalculate();
    boolean setAndCheckCurrentData(double _currentLatitude, double _currentLongitude, double _currentAltitudeFt, double _currentVelocityMPS, double _currentHeading, double _currentDataTimestamp);
    void setTargetData(double _targetLatitude, double _targetLongitude, double _targetAltitudeM);

  private:

    //Correct factors (these are not tuned/tested!!)
    #define CORRECTION_FACTOR = 0.9;  //for air resistance
    #define SERVO_OPEN_DELAY 250 //ms

    // ------------------------------------ CURRENT POSITION ------------------------------------

    // Format: dd° mm.mmmm'
    bool haveAPosition = false;  //Protect against calling 'recalculate' before having any data
    double currentLatitude = 0;
    double currentLongitude = 0;
    double currentAltitudeM = 0; // m
    double currentDataTimestamp = 0; // millis
    double currentVelocityMPS = 0;  //m/s
    double currentHeading = 0; // In degrees (E = 0, N = 90, W = 180, S = 270)
    double currentEasting = 0, currentNorthing = 0;
    double estDropEasting = 0, estDropNorthing = 0;


    // ------------------------------------ TARGET POSITION ------------------------------------

    // Format: dd° mm.mmmm' - These are initialized through 'setTargetData' function
    double targetLatitude;
    double targetLongitude;
    double targetAltitudeM; // m
    double targetEasting = -10000000, targetNorthing = -10000000;  //MAKE different than initial currentEast/North, or it may autodrop on startup


    // ------------------------------------ PHYSICAL CALCULATIONS ------------------------------------

    //ENCOMPASSING: call this to call 6 functions in order, evaluate results, and return true (ie. drop) or false (not yet)
    bool performTargetCalcsAndEvaluateResults();  //Call the following functions (in correct order) to update all targeting variables

    //None of the following should be called outside the above function
    //Step 1:
    void calculateLateralError();   //On current heading, what is the closest we come to target
    double lateralError; //m

    //Step 2:
    void calculateDirectDistanceToTarget();  //'As the crow flies' distance to target, not based on current heading
    double directDistanceToTarget;

    //Step 3:
    void calculateDistAlongPathToMinLateralErr();   //How far along our path until the point of min distance to target (third side of trianlge, with hyp = calculateDirectDistanceToTarget, opp = lateralError)
    double distAlongPathToMinLateralErr;

    //Step 4:
    void calculateHorizDistance();  //horizontal distance travelled from data age, servo open delay, and during the fall
    double horizDistance;

    //Step 5:
    void calculateDistFromEstDropPosToTarget();  //distance from where we are estimated to drop (currently) to the target. MUST be <ringRadius if we want to drop 
    double distFromEstDropPosToTarget;
                                
    //Step 6:
    void calculateTimeTillDrop();   //How long until we should drop
    double timeTillDrop;



    // ------------------------------------ CONVERSIONS ------------------------------------
    
    double convertHeadingToMathAngle(double);
    double convertDecimalDegMinToDegree(double);
    void convertDeg2UTM(double lattDegrees, double longDegrees, double &utmEasting, double &utmNorthing);

};

#endif //_TARGETER_H
