//////////////////////////////////////////////////////////
//// Compass LSM303_U i2c
////////////////////////////////////////////
void setupCompass() {
  compass.setup();  
}

double X,Y,Z;
float heading, headingDegrees, headingFiltered, geo_magnetic_declination_deg;

double YclosestToZero = 1; 
double ZclosestToZero = 1; 
unsigned int readCompass() {
  // because compass doesn't give accurate results if gps has a heading, then rather use that
  if(sodaq_gps.getCogHeading() != -1) {
    headingFiltered = sodaq_gps.getCogHeading();
    do_flash_led(LEDPIN);
  } else {
    headingFiltered = sodaq_gps.getCogHeading();
    /* for testing
    headingFiltered = compassOneValue();*/
  }
  /*for(int i = 0; i < 100; i++) {
    heading = compassOneValue();
    headingFiltered = (headingFiltered * 0.3) + (heading * 0.7);
  }
  //Sending the heading value through the Serial Port 
  */
  SerialUSB.println(headingFiltered,6);
  return headingFiltered;
}



unsigned int compassOneValue() {

  float xguass, yguass, zguass;
  
  compass.getNewValues();
  xguass = compass.getXGauss();    
  yguass = compass.getYGauss();
  zguass = compass.getZGauss();
  
  X = xguass / 32768; // 2^15 because a two's complement 16 bits integer has 2^15 posibilities in positive and negative
  Y = (yguass / 32768);
  Z = (zguass / 32768);

  /*lsm.read();
  float xh,yh,zh,ayf,axf,azf;
  axf = lsm.accelData.x/32768*PI;
  ayf = lsm.accelData.y/32768*PI;
  azf = lsm.accelData.z/32768*PI;
 
  zh=Z*cos(ayf)+Y*sin(ayf)*sin(azf)-X*cos(azf)*sin(ayf);

  yh=Y*cos(azf)+Z*sin(azf);

 */
/*
  SerialUSB.print("zh");
  SerialUSB.println(zh);
  SerialUSB.print("yh");
  SerialUSB.println(yh);
  /*if(abs(0-Y) < YclosestToZero) YclosestToZero = abs(0-Y);
  if(abs(0-Z) < ZclosestToZero) ZclosestToZero = abs(0-Z);
  SerialUSB.print("YclosestToZero");
  SerialUSB.println(YclosestToZero,30);
  SerialUSB.print("ZclosestToZero");
  SerialUSB.println(ZclosestToZero,30);*/
  
  // Correcting the heading with the geo_magnetic_declination_deg angle depending on your location
  // You can find your geo_magnetic_declination_deg angle at: http://www.ngdc.noaa.gov/geomag-web/
  // At zzz location it's 4.2 degrees => 0.073 rad
  // Haarlem                    2017-10-20  1° 4' E  ± 0° 22'  changing by  0° 9' E per year
  // Amsterdam                  2017-10-20  1° 9' E  ± 0° 22'  changing by  0° 9' E per year
  // Alkmaar 52.6324 4.7534     2017-10-20  1.09° E  ± 0.38°  changing by  0.14° E per year
  geo_magnetic_declination_deg = 1.09; // for our location
  
  //Calculating Heading
  headingDegrees = atan2(Y,Z)*(180/PI);  
  
  headingDegrees +=(2*360);
  headingDegrees = (unsigned int)headingDegrees % 360;
  return headingDegrees;
}


void put_Compass_and_Btn_into_sendbuffer() {
  unsigned int compass = readCompass(); // 0..360 deg
  uint8_t compass_bin = compass/3 ;  // rescale 0-360 deg into 0 - 120 values and make sure it is not bigger than one byte
  // now add a bit for BTN (not implemented)
  myLoraWanData[21] = compass_bin;
  #ifdef DEBUG
  DEBUG_STREAM.print(F("  compass=")); DEBUG_STREAM.print(compass); DEBUG_STREAM.print(F("  deg. compass_bin=")); DEBUG_STREAM.println(compass_bin);
  #endif
  if(buttonpressedForLoraWan) myLoraWanData[21] |= 0b10000000;
  else myLoraWanData[21] &= 0b01111111;
}

