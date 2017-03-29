#include "Targeter.h"
#include "Arduino.h"
#include "plane.h"

//TODO - for autodropping conditions, have altitude checked (make sure at 100 ft)

Targeter::Targeter() {

  //initialize target information (UTM coordinates), based on constants defined in plane.h
  setTargetData(TARGET_LATT, TARGET_LONG, TARGET_ALTITUDE_M);  //Target UTM is now never recalculated outside 'setTargetData' function

}

// ------------------------------------ METHODS OF CALLING THIS CLASS ------------------------------------
//(Either have new data, recalculate old data with new current time, or update target position)

//Recalculate old data, with new dataAge (ie. horizDist now has the project of plane forward included)
boolean Targeter::recalculate() {

  if (!haveAPosition) //If we haven't set any data, then we can't try this or it will fail
    return false;

  #ifndef Targeter_Debug_Print
    TARGET_PRINTLN("Recalculating Results: \n");
  #endif
  return performTargetCalcsAndEvaluateResults();

}

//Update the position with new data
boolean Targeter::setAndCheckCurrentData(double _currentLatitude, double _currentLongitude, double _currentAltitudeFt, double _currentVelocityMPS, double _currentHeading, double _currentDataTimestamp, boolean _hdopOk) {

  haveAPosition = true;

  currentLatitude = _currentLatitude;
  currentLongitude = _currentLongitude;
  currentAltitudeM = _currentAltitudeFt * FT_TO_METERS;
  currentVelocityMPS = _currentVelocityMPS;
  currentHeading = _currentHeading;
  currentDataTimestamp = _currentDataTimestamp;
  HDOP_OK = _hdopOk; 

  // Get current coordinates (saved in currentEasting/currentNorthing)
  convertDeg2UTM(convertDecimalDegMinToDegree(currentLatitude), convertDecimalDegMinToDegree(currentLongitude), currentEasting, currentNorthing);

  #ifndef Targeter_Debug_Print
    TARGET_PRINTLN("New Data Results: \n");
  #endif
  
  return performTargetCalcsAndEvaluateResults();

}

void Targeter::setTargetData(double _targetLatitude, double _targetLongitude, double _targetAltitudeM) {

  targetLatitude = _targetLatitude;
  targetLongitude = _targetLongitude;
  targetAltitudeM = _targetAltitudeM;

  //Update Target UTM Coordinates
  convertDeg2UTM(convertDecimalDegMinToDegree(targetLatitude), convertDecimalDegMinToDegree(targetLongitude), targetEasting, targetNorthing);

}



// ------------------------------------ PHYSICAL CALCULATIONS ------------------------------------

//The calls the 6 steps in correct order, then evaluates the results
bool Targeter::performTargetCalcsAndEvaluateResults()
{
  /*****Update all variables (by calling FN's in order) *****/
  calculateLateralError();
  calculateDirectDistanceToTarget();
  calculateDistAlongPathToMinLateralErr();
  calculateHorizDistance(); 
  calculateTimeTillDrop();
  calculateDistFromEstDropPosToTarget();
  

  /*****DEBUGGING - print results *****/
  #ifndef Targeter_Debug_Print
    TARGET_PRINT("Easting = ");  TARGET_PRINT(currentEasting);
    TARGET_PRINT("\t\tNorthing = ");  TARGET_PRINT(currentNorthing);
    TARGET_PRINT("\t\tTarget.easting = ");  TARGET_PRINT(targetEasting);
    TARGET_PRINT("\t\tTarget.northing = ");  TARGET_PRINTLN(targetNorthing);
    
    TARGET_PRINT("Alt (M) = ");  TARGET_PRINT(currentAltitudeM );
    TARGET_PRINT("\t\tVel = ");  TARGET_PRINT(currentVelocityMPS);
    TARGET_PRINT("\t\tHeading = ");  TARGET_PRINT(currentHeading);
    TARGET_PRINT("\t\tTimestamp = ");  TARGET_PRINTLN(currentDataTimestamp);
  
    TARGET_PRINT("Lateral error = ");  TARGET_PRINTLN(lateralError);
    TARGET_PRINT("Direct Dist to Target = "); TARGET_PRINTLN(directDistanceToTarget);
    TARGET_PRINT("Distance to min lateral err = ");  TARGET_PRINTLN(distAlongPathToMinLateralErr);
    TARGET_PRINT("Horizontal dist from drop, dataAge, dropDelay = "); TARGET_PRINTLN(horizDistance);
    TARGET_PRINT("Time until drop = ");  TARGET_PRINTLN(timeTillDrop);
    TARGET_PRINT("Dist from drop loc to target = ");  TARGET_PRINTLN(distFromEstDropPosToTarget);
    TARGET_PRINT("Target radius = ");  TARGET_PRINTLN(TARGET_RADIUS);
  #else
    TARGET_PRINT("DD = "); TARGET_PRINT(directDistanceToTarget);
    TARGET_PRINT("\tT = ");  TARGET_PRINT(timeTillDrop);
    TARGET_PRINT("\tTD = ");  TARGET_PRINTLN(distFromEstDropPosToTarget);
  #endif

  /***** Evaluate Results ******/

  //If altitude is below 100ft, do not drop regardless of other conditions
  if(currentAltitudeM < MINIMUM_DROP_ALTITUDE_M)
    return false;
  //else...
  
  // We need to have sufficient accuracy to drop
  if (!HDOP_OK)
	  return false;
  
  //If we are within 5m, drop regardless of other conditions
  if(distFromEstDropPosToTarget < 5)
    return true;
  //else...
  
  // 1. We update every 50ms, so if much higher than that we want to wait to drop closer to center
  if (timeTillDrop > 0.2) //200 ms * 15 m/s = 3 m, not really a worry of dropping early
    return false;

  // 2. We NEED be in the rings at the drop point
  if (distFromEstDropPosToTarget > TARGET_RADIUS)
    return false;

  // 3. Altitude???
  // We are now close enough to be in rings, and if under 0.2 we want to drop
  return true; 
}




/*
     Step 1
     Computes shortest distance from the line defined by the plane's path
     to the location of the target.
*/

void Targeter::calculateLateralError() {
  // See "Line defined by two points" at https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
  // From the wikipedia equation: P1 => prevPos, P2 => curPos, and (x0, y0) => targetPos

  // P1:
  // Generate an arbitrary point to define the line (far enough away to avoid rounding errors)
  double currentHeadingMathAngle = convertHeadingToMathAngle(currentHeading);
  double x1 = currentEasting + cos(currentHeadingMathAngle / 180 * PI) * 2000;
  double y1 = currentNorthing + sin(currentHeadingMathAngle / 180 * PI) * 2000;

  // P2:
  double x2 = currentEasting;
  double y2 = currentNorthing;

  // Target
  double x0 = targetEasting;
  double y0 = targetNorthing;

  // Calculate numerator in equation: (it's a big equation...)
  double numerator = (y2 - y1) * x0;
  numerator -= (x2 - x1) * y0;
  numerator += x2 * y1;
  numerator -= y2 * x1;
  numerator = abs(numerator);

  // Calculate denominator in equation:
  double denominator = pow(y2 - y1, 2.0);
  denominator += pow(x2 - x1, 2.0);
  denominator = sqrt(denominator);

  lateralError = numerator / denominator;
}

/*
   Step 2
   Calculates direct distance from current position to the target.
*/

void Targeter::calculateDirectDistanceToTarget() {

  // Use pythagorean theorem to calculate distance between curPos and targetPos:
  directDistanceToTarget =  sqrt(pow((targetEasting - currentEasting), 2) + pow((targetNorthing - currentNorthing), 2));
}


/*
   Step 3
   Calculates distance along current path until nearest point to target.
   Note: Relies on the fact that an accurate lateral error has already been calculated
*/

void Targeter::calculateDistAlongPathToMinLateralErr() {
  distAlongPathToMinLateralErr = sqrt(pow(directDistanceToTarget, 2) - pow(lateralError, 2));
}


/*
   Step 4
   Calculates how early (distance in m before being above the target) the
   package must be dropped based on current speed, altitude, DataTimestamp and servo opening delay.
   Note - this accounts for data age AND a delay for the servo opening


   *** Note: right now this picks a value for air resistance and is unrealistic.
   Some thoughts on air resistance: terminal falling velocity 35-45 m/s ish, which would be reached at 4.1s or 82m / 270ft
   It will accelerate slower once above low speeds
   @ 10 m/s estimated air resistance acceleration would be -0.6 m/s^2. If fall time on the order of 3s, this would
   only affect speed vi = 10, vf = 8.2, vavg = 9.1.  So not a lot, and is partially offset by reduced fall speed
   Approximate Equations: m = 2kg, cd =1 CSA = 0.02m^2, rho = 1.25   a = 0.5*cd*rho*CSA*v^2/m = 0.00625*v^2
   Overall approximation of 0.9 correction factor
   See MATLAB script for more details
*/
//#define CORRECTION_FACTOR 0.9

void Targeter::calculateHorizDistance() {

  // targetPos.getAltitude() will probably be 0, but I included it just in case:
  double heightm = (currentAltitudeM - targetAltitudeM);

  // Calculate time for payload to fall:
  if (heightm < 0) {
    heightm = 0;  //prevent NaN from sqrt (altimeter noise may make it less than 0 often on the ground)
  }

  double fallTime = sqrt(2 * heightm / 9.807); // time in seconds

  // Calculate horizontal distance that payload will travel in this time:
  double distanceDuringFall = currentVelocityMPS * fallTime * CORRECTION_FACTOR;
  double distanceFromDataAge = currentVelocityMPS*(millis() - currentDataTimestamp)/1000.0;
  double distanceFromServoOpenDelay = currentVelocityMPS*SERVO_OPEN_DELAY/1000.0;
  horizDistance = distanceDuringFall  + distanceFromDataAge  + distanceFromServoOpenDelay;
}


/*
  Step 5
  Calculate where we end up in relation to target 
 */
 
 void Targeter::calculateDistFromEstDropPosToTarget() {

  double currentHeadingMathAngle = convertHeadingToMathAngle(currentHeading);
  estDropEasting = currentEasting + cos(currentHeadingMathAngle / 180 * PI) * horizDistance;
  estDropNorthing = currentNorthing + sin(currentHeadingMathAngle / 180 * PI) * horizDistance;

  distFromEstDropPosToTarget = sqrt(pow(targetNorthing - estDropNorthing, 2) + pow(targetEasting - estDropEasting, 2));

}

/*
   Step 6
   Calculates time until optimal drop location.
*/
void Targeter::calculateTimeTillDrop() {

  
  //Special Case: Plane is 'before' target, and est drop position is 'after' target.
  //Test: If drop easting and current easting are on opposite sides of the target easting (and same for nothing) 
  //This is the first if (which is used to prevent this scenario from triggering 'Moving away from target'
    if(   estDropEasting > targetEasting && currentEasting < targetEasting      ||    estDropEasting < targetEasting && currentEasting > targetEasting  //Compare easting
      ||  estDropNorthing > targetNorthing && currentNorthing < targetNorthing  ||    estDropNorthing < targetNorthing && currentNorthing > targetNorthing) //Compare northing
    {
      //This actually works properly. distAlongPathToMinLateralErr is positive, and horizontal distance should be subtracted
      timeTillDrop = (distAlongPathToMinLateralErr - horizDistance) / currentVelocityMPS;
    }
    //Moving away from target
    else if(distFromEstDropPosToTarget > directDistanceToTarget)
    {
        timeTillDrop = (-distAlongPathToMinLateralErr - horizDistance) / currentVelocityMPS;   //now distAlongPathToMinLateralErr is a negative
    }
    else
    {
        timeTillDrop = (distAlongPathToMinLateralErr - horizDistance) / currentVelocityMPS;
    }
}





// ------------------------------------ CONVERSIONS -----------------------------------

//A heading (ie. N = 0 degrees) into a math angle (typical x/y origin, x axis = 0 degrees)
double Targeter::convertHeadingToMathAngle(double heading) {
  // Transform from compass degrees to typical math degrees
  double angle = -1 * (heading - 90);
  if (angle < 0) {
    angle = 360 + angle;
  }
  return angle;
}

//lat/long in decimal degree minutes ---> degrees
double Targeter::convertDecimalDegMinToDegree(double decimalDegreeMin) {  //accepts format AAAYY.ZZZZZ  AAA = degrees,, YY = minutes,  ZZZZZZ = decimal minutes
  // Assumes the N/S & E/W sign convention is followed (N = +ve, W = -ve).
  // This is currently done in MainWindow in analyzePacket

  int baseDegree = (int)(decimalDegreeMin / 100.0);   // Extracts AAA (the cast will remove the shifted digits /100.0 ->  AAA.YYZZZZZ, cast -> AAA)
  double baseDegMins = decimalDegreeMin - 100 * baseDegree;  // AAAYY.ZZZ - AAA*100 = AAAYY.ZZ - AAA00.0 = YY.ZZZZ
  return baseDegree + baseDegMins / 60.0; // 1 degree minute = 1/60 degrees
}


//Convert degrees GPS data into UTM
void Targeter::convertDeg2UTM(double latDegrees, double longDegrees, double &utmEasting, double &utmNorthing) {

  char utmLetter;
  int utmZone = (int) floor(longDegrees / 6 + 31);

  if (latDegrees < -72)
    utmLetter = 'C';
  else if (latDegrees < -64)
    utmLetter = 'D';
  else if (latDegrees < -56)
    utmLetter = 'E';
  else if (latDegrees < -48)
    utmLetter = 'F';
  else if (latDegrees < -40)
    utmLetter = 'G';
  else if (latDegrees < -32)
    utmLetter = 'H';
  else if (latDegrees < -24)
    utmLetter = 'J';
  else if (latDegrees < -16)
    utmLetter = 'K';
  else if (latDegrees < -8)
    utmLetter = 'L';
  else if (latDegrees < 0)
    utmLetter = 'M';
  else if (latDegrees < 8)
    utmLetter = 'N';
  else if (latDegrees < 16)
    utmLetter = 'P';
  else if (latDegrees < 24)
    utmLetter = 'Q';
  else if (latDegrees < 32)
    utmLetter = 'R';
  else if (latDegrees < 40)
    utmLetter = 'S';
  else if (latDegrees < 48)
    utmLetter = 'T';
  else if (latDegrees < 56)
    utmLetter = 'U';
  else if (latDegrees < 64)
    utmLetter = 'V';
  else if (latDegrees < 72)
    utmLetter = 'W';
  else
    utmLetter = 'X';

  utmEasting = 0.5 * log((1 + cos(latDegrees * PI / 180) * sin(longDegrees * PI / 180 - (6 * utmZone - 183) * PI / 180)) / (1 - cos(latDegrees * PI / 180) * sin(longDegrees * PI / 180 - (6 * utmZone - 183) * PI / 180))) * 0.9996 * 6399593.62 / pow((1 + pow(0.0820944379, 2) * pow(cos(latDegrees * PI / 180), 2)), 0.5) * (1 + pow(0.0820944379, 2) / 2 * pow((0.5 * log((1 + cos(latDegrees * PI / 180) * sin(longDegrees * PI / 180 - (6 * utmZone - 183) * PI / 180)) / (1 - cos(latDegrees * PI / 180) * sin(longDegrees * PI / 180 - (6 * utmZone - 183) * PI / 180)))), 2) * pow(cos(latDegrees * PI / 180), 2) / 3) + 500000;
  utmEasting = round(utmEasting * 100) * 0.01;
  utmNorthing = (atan(tan(latDegrees * PI / 180) / cos((longDegrees * PI / 180 - (6 * utmZone - 183) * PI / 180))) - latDegrees * PI / 180) * 0.9996 * 6399593.625 / sqrt(1 + 0.006739496742 * pow(cos(latDegrees * PI / 180), 2)) * (1 + 0.006739496742 / 2 * pow(0.5 * log((1 + cos(latDegrees * PI / 180) * sin((longDegrees * PI / 180 - (6 * utmZone - 183) * PI / 180))) / (1 - cos(latDegrees * PI / 180) * sin((longDegrees * PI / 180 - (6 * utmZone - 183) * PI / 180)))), 2) * pow(cos(latDegrees * PI / 180), 2)) + 0.9996 * 6399593.625 * (latDegrees * PI / 180 - 0.005054622556 * (latDegrees * PI / 180 + sin(2 * latDegrees * PI / 180) / 2) + 4.258201531e-05 * (3 * (latDegrees * PI / 180 + sin(2 * latDegrees * PI / 180) / 2) + sin(2 * latDegrees * PI / 180) * pow(cos(latDegrees * PI / 180), 2)) / 4 - 1.674057895e-07 * (5 * (3 * (latDegrees * PI / 180 + sin(2 * latDegrees * PI / 180) / 2) + sin(2 * latDegrees * PI / 180) * pow(cos(latDegrees * PI / 180), 2)) / 4 + sin(2 * latDegrees * PI / 180) * pow(cos(latDegrees * PI / 180), 2) * pow(cos(latDegrees * PI / 180), 2)) / 3);

  if (utmLetter < 'M') {
    utmNorthing = utmNorthing + 10000000;
  }
  utmNorthing = round(utmNorthing * 100) * 0.01;
}
