#include "filter.h"
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

  private:
    IIR_doubleFilter *filter;
    int zeroVal;
};
