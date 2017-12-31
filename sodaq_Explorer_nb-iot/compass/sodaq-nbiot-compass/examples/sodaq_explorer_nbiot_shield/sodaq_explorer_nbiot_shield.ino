#include <sodaq_compass.h>
#define DEBUG_STREAM SerialUSB
NBIOT_Compass compass;


void setup() {
  SerialUSB.begin(9600);
  compass.setup();  
}


float X,Y,Z;
float heading, headingDegrees, headingFiltered, geo_magnetic_declination_deg;

long readCompass() {
  float xguass, yguass, zguass;
  
  compass.getNewValues();
  xguass = compass.getXGauss();    
  yguass = compass.getYGauss();
  zguass = compass.getZGauss();
  
  X = xguass / 32768; // 2^15 because a two's complement 16 bits integer has 2^15 posibilities in positive and negative
  Y = yguass / 32768;
  Z = zguass / 32768;
  
  // Correcting the heading with the geo_magnetic_declination_deg angle depending on your location
  // You can find your geo_magnetic_declination_deg angle at: http://www.ngdc.noaa.gov/geomag-web/
  // At zzz location it's 4.2 degrees => 0.073 rad
  // Haarlem                    2017-10-20  1° 4' E  ± 0° 22'  changing by  0° 9' E per year
  // Amsterdam                  2017-10-20  1° 9' E  ± 0° 22'  changing by  0° 9' E per year
  // Alkmaar 52.6324 4.7534     2017-10-20  1.09° E  ± 0.38°  changing by  0.14° E per year
  geo_magnetic_declination_deg = 1.09; // for our location
  
  //Calculating Heading
  headingDegrees = 180*atan2(Y, Z)/PI;  // assume pitch, roll are 0
  headingDegrees -=10;
  
  if (headingDegrees <0)
    headingDegrees += 360;
  if (headingDegrees <0)
    headingDegrees += 360;
 
  
  //Sending the heading value through the Serial Port 
  
  DEBUG_STREAM.println(headingDegrees,6);
  
  return headingDegrees;
}

void loop() {
  
  SerialUSB.println(readCompass());  
  delay(1000);

}

