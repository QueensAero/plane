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

    boolean setAndCheckCurrentData(double, double, double, double, double, double);
    void setTargetData(double, double, double);

  private:

    const double CORRECTION_FACTOR = 0.9;
    #define SERVO_OPEN_DELAY 250 //ms

    // ------------------------------------ CURRENT POSITION ------------------------------------

    // Format: dd° mm.mmmm'
    bool haveAPosition = false;  //Protect against calling 'recalculate' before having any data
    double currentLatitude = 0;
    double currentLongitude = 0;
    double currentAltitude = 0; // m
    double currentDataAge = 0; // millis
    double currentVelocity = 0;
    double currentHeading = 0; // In degrees (E = 0, N = 90, W = 180, S = 270)

    double currentEasting = 0, currentNorthing = 0;
    double estDropEasting = 0, estDropNorthing = 0;


    // ------------------------------------ TARGET POSITION ------------------------------------

    // Format: dd° mm.mmmm' - These are initialized through 'setTargetData' function
    double targetLatitude;
    double targetLongitude;
    double targetAltitude; // m
    double targetEasting = -10000000, targetNorthing = -10000000;  //MAKE different than initial currentEast/North, or it may autodrop on startup

    // ------------------------------------ CALCULATED POSITION ------------------------------------

    // Calculated values
    double lateralError; //m
    double timeToDrop;  //seconds
    double dropHorizDistance;
    double estDropPosDistToTarget;

    // ------------------------------------ PHYSICAL CALCULATIONS ------------------------------------

    double calculateLateralError(void);   //On current heading, what is the closest we come to target
    double calculateDirectDistanceToTarget(void);  //'As the crow flies', not based on current heading
    double calculatePathDistanceToTarget(void);   //How far along our path until the point of min distance to target (third side of trianlge, with hyp = calculateDirectDistanceToTarget, opp = lateralError)
    double calculateDropDistance(void);  //horizontal distance travelled while falling
    double calculateTimeToDrop(void);   //How long until we should drop
    double dropDistanceToTarget(void);   //PathDistanceToTarget  -  DropDistance   (ie. If we dropped now, what is dist to point where lateral error is a minimum (and thus DirectDistToTarget is minimum). May be negative is passed 
                                         //Note this function ignores any 'stale' data assumtions/projecting plane position forward 
    void calculateEstDropPositionAndDist(void);
    bool evaluateDropCriteria();
                                         

    // ------------------------------------ CONVERT LAT/LON DEGREES TO UTM COORDINATES ------------------------------------
    
    double convertHeadingToMathAngle(double);
    double convertDecimalDegMinToDegree(double);
    void convertDeg2UTM(double, double, double &, double &);

};

#endif //_TARGETER_H
