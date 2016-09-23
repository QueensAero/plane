#include "filter.h"
#include "I2Cdev.h"
#include <Wire.h>
#include <Arduino.h>

#define DEC_TO_FEET 0.32808399

class EagleTreeAltimeterV4 {
  
  public:
    EagleTreeAltimeterV4();
    void initialize();
    void zero();
    double readAltitude();
    int altimeterAddress; //76
    int readRawData();
/*Value displayed on sensor is either in meters of feet. 
 * To change between the two, disconnect sensor. Short yellow (data) and brown (clock) dots and connect to power.
 * This will not affect raw data value to Arduino.
 */
  private:
    IIR_doubleFilter *filter;
    int zeroVal;
};
