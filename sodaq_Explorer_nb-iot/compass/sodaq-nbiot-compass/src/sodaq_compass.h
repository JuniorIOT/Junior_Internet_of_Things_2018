/*
  Sodaq_compass.h - Library for compass in sodaq nb_iot for sodaq explorer
  Created by Dennis Ruigrok, November 20, 2017.
  Released into the public domain.
*/
#ifndef NB_iot_Compass_h
#define NB_iot_Compass_h

#include "Arduino.h"

class NBIOT_Compass
{
  public:
    NBIOT_Compass();
    void setup();
    void getNewValues(); // always call to read new values
    float getXGauss();
    float getYGauss();
    float getZGauss();   
  private:
    void set_register(unsigned char r, unsigned char v);
    byte _address;
    int _tries;
    float _headingFiltered;
    float _xguass;
    float _yguass;
    float _zguass;
    int16_t _xbytes;
    int16_t _ybytes;
    int16_t _zbytes;

   
};

#endif
