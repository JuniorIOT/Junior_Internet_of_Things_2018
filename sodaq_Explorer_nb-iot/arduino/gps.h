//////////////////////////////////////////////////////////
//// JuniorIOTchallenge routines for gps
////////////////////////////////////////////

void gps_init() {
  sodaq_gps.init(6);
  #ifdef DEBUG
  sodaq_gps.setDiag(DEBUG_STREAM);
  #endif

  l_lat = 0; l_lon = 0; l_alt = 678; hdopNumber = 99999;   // the zero position
  l_lat_hist = 0; l_lon_hist = 0; l_lat_movement = 0; l_lon_movement = 0; // movement detection

  // First time finding a fix wait ?? seconds at most
  find_fix(1);  // quickly switch on and leave on so it can continue to find a fix
}

void doGPS(uint32_t delay_until) {
  find_fix(delay_until);
}

void find_fix(uint32_t delay_until)
{
    DEBUG_STREAM.println(String("find_fix started."));
    uint32_t start = millis();
    int hasfix=0;
    uint32_t timeout = delay_until * 1000; // timeout
    uint32_t mini_timeout = timeout / 1000; 
    if (mini_timeout < 1100 ) mini_timeout = 1100;  // somewhat longer than one second
    DEBUG_STREAM.println(String("  GPS scanning till fix, timeout=") + timeout/1000 + String("s"));
    
    while( !hasfix and (millis() - start) < (timeout) ) {
      if (sodaq_gps.scan(true, mini_timeout)) { // true == leave on
      
        hasfix = 1;
        
        l_lat = sodaq_gps.getLat() * 10000000;
        l_lon = sodaq_gps.getLon() * 10000000;
        hitlat1 = l_lat;
        hitlng1 = l_lon;
    
        // movement detection
        if(l_lat_hist==0) l_lat_hist=l_lat;
        if(l_lon_hist==0) l_lon_hist=l_lon;
        l_lat_movement = abs(l_lat_hist - l_lat);
        l_lon_movement = abs(l_lon_hist - l_lon);
        l_lat_hist= (3*l_lat_hist + l_lat)/4; 
        l_lon_hist= (3*l_lon_hist + l_lon)/4; 
          
        l_alt = sodaq_gps.getAlt();
        hdopNumber = sodaq_gps.getHDOP();
        DEBUG_STREAM.print("  Has found a Fix, "); 
        DEBUG_STREAM.print(" l_alt ");
        DEBUG_STREAM.print(l_alt);
        DEBUG_STREAM.print(" hdop ");
        DEBUG_STREAM.println(hdopNumber);
      }
      DEBUG_STREAM.print(".");
      led_toggle();
    }
    if (!hasfix) {
        DEBUG_STREAM.print("  No Fix");        
    }
    DEBUG_STREAM.println(""); 
}

void put_gpsvalues_into_lora_sendbuffer(bool savePrevious) {
  DEBUG_STREAM.println(F("Started: put_gpsvalues_into_sendbuffer"));
  
  //   GPS reading = Satellite time hh:mm:18, lat 526326595, lon 47384133, alt 21, hdop 990, sat count = 12
  // With the larger precision in LONG values in NMEAGPS, 
  //    when rescaling, the values exceed the range for type LONG  -2.147.483.648 .. 2.147.483.647
  //    so we need to use DOUBLE with 15 digits precision, not preferred is FLOAT with 7 digits
  //    our values such as 526326595 have 9 digits
    
  const double shift_lat     =    90. * 10000000.;                 // range shift from -90..90 into 0..180, note: 
                                                                 //      NMEAGPS long lat&lon are degree values * 10.000.000
                                                                 //      TynyGPS long lat&lon are degree values * 1.000.000
  const double max_old_lat   =   180. * 10000000.;                 // max value for lat is now 180
  const double max_3byte     =         16777215.;                   // max value that fits in 3 bytes
  double lat_DOUBLE         = l_lat;                              // put 4byte LONG into a more precise floating point to prevent rounding during calcs 
  lat_DOUBLE = (lat_DOUBLE + shift_lat) * max_3byte / max_old_lat; // rescale into 3 byte integer range
  uint32_t LatitudeBinary  = lat_DOUBLE;                          // clips off anything after the decimal point    
  const double shift_lon     =   180. * 10000000.;                 // range shift from -180..180 into 0..360
  const double max_old_lon   = 360. * 10000000.;                   // max value longitude is now 360, note the value is too big for Long type
  double lon_DOUBLE = l_lon;                                      // put the 4byte LONG into a precise floating point memory space
  lon_DOUBLE = (lon_DOUBLE + shift_lon) * max_3byte / max_old_lon; // rescale into 3 byte integer range
  uint32_t LongitudeBinary = lon_DOUBLE;                          // clips off anything after the decimal point  
  uint16_t altitudeBinary  = l_alt    ;                          // we want altitudeGps in meters, note:
                                                                 //      NMEAGPS alt.whole is meter value 
                                                                 //      TynyGPS long alt is meter value * 100
  if (l_alt<0) altitudeBinary=0;                                 // unsigned int wil not allow negative values and warps them to huge number  
  uint8_t HdopBinary = hdopNumber * 10;                           // we want horizontal dillution, good is 2..5, poor is >20. Note:
                                                                 
                                                                 //      from TinyGPS horizontal dilution of precision in 100ths? We succesfully divided by 10
                                                                 //      TinyGPSplus seems the same in 100ths as per MNEMA string. We succesfully divided by 10
  
//  DEBUG_STREAM.print(F("  shift_lat = "));//DEBUG_STREAM.println(shift_lat);
//  DEBUG_STREAM.print(F("  max_old_lat = "));//DEBUG_STREAM.println(max_old_lat);
//  DEBUG_STREAM.print(F("  max_3byte = "));//DEBUG_STREAM.println(max_3byte);
//  DEBUG_STREAM.print(F("  l_lat = "));//DEBUG_STREAM.println(l_lat);
//  DEBUG_STREAM.print(F("  lat_float = "));//DEBUG_STREAM.println(lat_float);
//  DEBUG_STREAM.print(F("  LatitudeBinary = "));//DEBUG_STREAM.println(LatitudeBinary);
//  
//  DEBUG_STREAM.print(F("\n  shift_lon = "));//DEBUG_STREAM.println(shift_lon);
//  DEBUG_STREAM.print(F("  max_old_lon = "));//DEBUG_STREAM.println(max_old_lon);
//  DEBUG_STREAM.print(F("  l_lon = "));//DEBUG_STREAM.println(l_lon);
//  DEBUG_STREAM.print(F("  lon_float = "));//DEBUG_STREAM.println(lon_float);
//  DEBUG_STREAM.print(F("  LongitudeBinary = "));//DEBUG_STREAM.println(LongitudeBinary);
//  
//  DEBUG_STREAM.print(F("\n  l_alt = "));//DEBUG_STREAM.println(l_alt);
//  DEBUG_STREAM.print(F("  altitudeBinary = "));//DEBUG_STREAM.println(altitudeBinary);
//  
//  //DEBUG_STREAM.print(F("\n  hdopNumber = "));//DEBUG_STREAM.println(hdopNumber);
//  //DEBUG_STREAM.print(F("  HdopBinary = "));//DEBUG_STREAM.println(HdopBinary);
  
  myLoraWanData[0] = ( LatitudeBinary >> 16 ) & 0xFF;
  myLoraWanData[1] = ( LatitudeBinary >> 8 ) & 0xFF;
  myLoraWanData[2] = LatitudeBinary & 0xFF;
  myLoraWanData[3] = ( LongitudeBinary >> 16 ) & 0xFF;
  myLoraWanData[4] = ( LongitudeBinary >> 8 ) & 0xFF;
  myLoraWanData[5] = LongitudeBinary & 0xFF;
  // altitudeGps in meters into unsigned int
  myLoraWanData[6] = ( altitudeBinary >> 8 ) & 0xFF;
  myLoraWanData[7] = altitudeBinary & 0xFF;
  // hdop in tenths of meter
  myLoraWanData[8] = HdopBinary & 0xFF;

  print_myLoraWanData();
    
  //  Dummy satellite values: time hh:mm:5, lat 52632400, lon 4738800, alt 678, hdop 2345, sat count = 12
  //       0  1  2  3  4  5  6  7  8   
  //     [87 7C 49 80 56 44 02 A6 17 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00  .. ]
  
  if(savePrevious) {
    DEBUG_STREAM.println("  Saving value into previous gps");
    myLoraWanData[9] = myLoraWanData[0];
    myLoraWanData[10] = myLoraWanData[1];
    myLoraWanData[11] = myLoraWanData[2];
    myLoraWanData[12] = myLoraWanData[3];
    myLoraWanData[13] = myLoraWanData[4];
    myLoraWanData[14] = myLoraWanData[5];
    myLoraWanData[15] = myLoraWanData[6];
    myLoraWanData[16] = myLoraWanData[7];
    myLoraWanData[17] = myLoraWanData[8];
  } else {    
    DEBUG_STREAM.println("  DO NOT safe value into previous gps");
  }
}

void doGPS_and_put_values_into_lora_sendbuffer() {
  DEBUG_STREAM.print(F("doGPS_and_put_values_into_sendbuffer started. milis=")); DEBUG_STREAM.println(millis());
  find_fix(20); // find fix in ?? seconds
  // put gps values into send buffer
  put_gpsvalues_into_lora_sendbuffer(false); // don't put this in previous
  
  DEBUG_STREAM.print(F("  doGPS_and_put_values_into_sendbuffer completed. milis=")); DEBUG_STREAM.println(millis());
  
}
