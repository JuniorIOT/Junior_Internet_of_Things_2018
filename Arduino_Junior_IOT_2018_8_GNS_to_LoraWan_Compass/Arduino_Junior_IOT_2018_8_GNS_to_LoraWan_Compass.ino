/*******************************************************************************
 * Modified By DenniZr & Marco van Schagen for Junior IOT - Smart City Challenge 2018 #juniorIOTchallenge2018
 * Modified By Marco van Schagen for Junior IOT Challenge 2018
 *******************************************************************************/ 
 
//#define DEBUG     // if DEBUG is defined, some code is added to display some basic debug info
//#define DEBUGLEVEL2
#define DEBUGRADIO

#define VBATPIN A9
#define LEDPIN 13 
#define LORAWAN_TX_INTERVAL 240  // seconds between LoraWan messages
   // 240 = every 4 minutes,  was original 10/26
   //  60 = once a minute, that is okay when war-driving with only few minutes daily within range
   
//////////////////////////////////////////////
// GPS libraries, mappings and things
//////////////////////////////////////////////
#define GPS_FIX_HDOP   // to prevent eror: 'const class gps_fix' has no member named 'hdop'
#define GPS_RXD_PIN 10  
#define GPS_TXD_PIN 11    // where we plugged in our GNSS GPS into Lora32u4

#include <NeoSWSerial.h>  //  We now use NeoSWSerial for lower footprint end better performance than SoftwareSerial
  // an issue with Leonardo-types is fixed in branch, yet to be merged into main version library. So you may need to remove all your NeoSWSerial libraries and add \libraries\NeoSWSerial-master-DamiaBranch.zip
NeoSWSerial ss(GPS_RXD_PIN, GPS_TXD_PIN);

// You must enable/change these lines in NMEAGPS_cfg.h:
//          #define NMEAGPS_PARSE_GLL  // juniorIOTchallenge2018
//          #define LAST_SENTENCE_IN_INTERVAL NMEAGPS::NMEA_GLL  // juniorIOTchallenge2018
// and in GPSfox_cfg.h:         
//          #define GPS_FIX_HDOP  // juniorIOTchallenge2018
#include <NMEAGPS.h>       // We now use NmeaGps (or NeoGps) as it understands newer GNSS (default reads only GGA, RMC)
static NMEAGPS gps;    // This parses the GPS characters

long gps_fix_count = 0;
long gps_nofix_count = 0;
unsigned long gps_last_time = millis();
unsigned long gps_gets_time = 5000;

//////////////////////////////////////////////
// LoraWan libraries, mappings and things
//////////////////////////////////////////////

//#include <SPI.h>  //MISO MOSI SCK stuff that was part of 2017 thing with rfm95
#define PAYLOADSIZE 23 // The size of the package to be sent
#include <avr/pgmspace.h>
#include <lmic_slim.h>     // the really cool micro-library, to replace our 2017 LMIC which filled 99% memory
#include "keys.h"          // the personal keys to identify our own nodes, in a file outside GITHUB

int TX_COMPLETE_was_triggered = 0;  // 20170220 added to allow full controll in main Loop
uint8_t  myLoraWanData[40];  // including byte[0]
unsigned long last_lora_time = millis(); // last time lorawan ran

//////////////////////////////////////////////////////////
//// Compass HMC5983
////////////////////////////////////////////
#include <Wire.h> //I2C Arduino Library

#define HMC5983_selector_for_X_byte0 0x03  
#define HMC5983_selector_for_X_byte1 0x04  
#define HMC5983_selector_for_Z_byte0 0x05  
#define HMC5983_selector_for_Z_byte1 0x06  
#define HMC5983_selector_for_Y_byte0 0x07  
#define HMC5983_selector_for_Y_byte1 0x08  
int HMC5983_X_byte0, HMC5983_X_byte1, HMC5983_X_value;
int HMC5983_Y_byte0, HMC5983_Y_byte1, HMC5983_Y_value;
int HMC5983_Z_byte0, HMC5983_Z_byte1, HMC5983_Z_value;
float X_milliGauss,Y_milliGauss,Z_milliGauss;
float heading, headingDegrees, headingFiltered, geo_magnetic_declination_deg;
#define IIC_HMC5983_address 0x1E //I2C 7bit address of HMC5883

//--------------Table of contents------------//
//////////////////////////////////////////////////////////
//// Kaasfabriek routines for gps
////////////////////////////////////////////
void doGPS();
void put_gpsvalues_into_lora_sendbuffer();
void gps_init();
//////////////////////////////////////////////////
// Kaasfabriek routines for LMIC_slim for LoraWan
///////////////////////////////////////////////
void setupLora();
void doOneLora();
//////////////////////////////////////////////////
// Kaasfabriek routines for RFM95 radio to radio 
///////////////////////////////////////////////
void formatRadioPackage(uint8_t *loopbackToData);
void doOneRadio();
void halt_stressed();
void setupRadio();
void decodeReply(uint8_t buf[], bool debugToSerial);
///////////////////////////////////////////////
//  some other measurements
///////////////////////////////////////////
double GetTemp(void);
long readVbat();
void put_TimeToFix_into_sendbuffer(int TimeToFix_Seconds);
//////////////////////////////////////////////////////////
//// Compass HMC5983
////////////////////////////////////////////
void setupCompass();
long readCompass();
void put_Compass_and_Btn_into_sendbuffer();
///////////////////////////////////////////////
//  arduino init and main
///////////////////////////////////////////
void setup();
void loop();
//--------------/Table of contents------------//



//////////////////////////////////////////////////////////
//// Kaasfabriek routines for gps
////////////////////////////////////////////
long l_lat, l_lon, l_alt;
int hdopNumber;

void put_gpsvalues_into_lora_sendbuffer() {
  #ifdef DEBUGLEVEL2
  Serial.println(F("Started: put_gpsvalues_into_sendbuffer"));
  #endif
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
  uint8_t HdopBinary = hdopNumber/100;                           // we want horizontal dillution, good is 2..5, poor is >20. Note:
                                                                 //      NMEAGPS outputs an indoor value of 600..1000. Let's divide by 100
                                                                 //      from TinyGPS horizontal dilution of precision in 100ths? We succesfully divided by 10
                                                                 //      TinyGPSplus seems the same in 100ths as per MNEMA string. We succesfully divided by 10
  
//  Serial.print(F("  shift_lat = "));//Serial.println(shift_lat);
//  Serial.print(F("  max_old_lat = "));//Serial.println(max_old_lat);
//  Serial.print(F("  max_3byte = "));//Serial.println(max_3byte);
//  Serial.print(F("  l_lat = "));//Serial.println(l_lat);
//  Serial.print(F("  lat_float = "));//Serial.println(lat_float);
//  Serial.print(F("  LatitudeBinary = "));//Serial.println(LatitudeBinary);
//  
//  Serial.print(F("\n  shift_lon = "));//Serial.println(shift_lon);
//  Serial.print(F("  max_old_lon = "));//Serial.println(max_old_lon);
//  Serial.print(F("  l_lon = "));//Serial.println(l_lon);
//  Serial.print(F("  lon_float = "));//Serial.println(lon_float);
//  Serial.print(F("  LongitudeBinary = "));//Serial.println(LongitudeBinary);
//  
//  Serial.print(F("\n  l_alt = "));//Serial.println(l_alt);
//  Serial.print(F("  altitudeBinary = "));//Serial.println(altitudeBinary);
//  
//  //Serial.print(F("\n  hdopNumber = "));//Serial.println(hdopNumber);
//  //Serial.print(F("  HdopBinary = "));//Serial.println(HdopBinary);
  
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

}

bool process_gps_datastream(const gps_fix & fix) {   // constant pointer to fix object
  //unsigned long age; 
  //bool GPS_values_are_valid = true;
    
  //if (fix.valid.location && fix.dateTime.seconds > 0) {   
  if (fix.valid.location ) {    
    l_lat = fix.latitudeL();
    l_lon = fix.longitudeL();
    l_alt = fix.alt.whole;
    //l_alt = fix.altitude_cm;
    hdopNumber = fix.hdop;
    
    // serial print commands will take time and may affect the gps read
    #ifdef DEBUGLEVEL2
    Serial.print("  Satellite time hh:mm:"); Serial.print( fix.dateTime.seconds ); Serial.print(", ");
    Serial.print("lat "); Serial.print( l_lat  ); Serial.print(", ");
    Serial.print("lon "); Serial.print( l_lon ); Serial.print(", ");
    Serial.print("alt "); Serial.print( l_alt ); Serial.print(", ");
    Serial.print("hdop "); Serial.print( hdopNumber ); Serial.print(", ");
    Serial.print("sat count = "); Serial.print( fix.satellites );
    //Serial.print( fix.dateTime ); Serial.print(" datetime, ");    
    //Serial.print( fix.latitude(), 6 ); // floating-point display
    //Serial.print( fix.longitude(), 6 ); // floating-point display
    //   Satellite time hh:mm:18, lat 526326595, lon 47384133, alt 21, hdop 990, sat count = 12
    
    Serial.println();
    #endif
    return true;
  } else {
    #ifdef DEBUGLEVEL2
    Serial.print( "(no valid location) " );
    #endif
    return false;
  }  
}

void doGPS(unsigned long gps_listen_timeout) {
  bool hasFix = false;
  #ifdef DEBUGLEVEL2
  Serial.print(F("\nStart: doGPS_and_put_values_into_sendbuffer. milis=")); Serial.println(millis());
  // first we want to know GPS coordinates - we do accept a long delay if needed, even before listening to radio
  #endif
  unsigned long gps_listen_startTime = millis(); 

  
  //now listen to gps till fix or time-out, once gps has a fix, the refresh should be ready within 2 data reads = less than 3 sec
  // gps read command:
  #ifdef DEBUGLEVEL2
  Serial.println(F("Listen to GPS stream:"));
  #endif
  while((millis() - gps_listen_startTime) < (gps_listen_timeout * 1000L) && !hasFix) {
    // for NMEAgps
    while (gps.available(ss) && !hasFix) {
      hasFix = process_gps_datastream(gps.read()); 
    }
  }    
  #ifdef DEBUGLEVEL2
  Serial.println(F("Completed listening to GPS."));
  #endif
}

void doGPS_and_put_values_into_lora_sendbuffer() {
  doGPS(3);
  // put gps values into send buffer
  put_gpsvalues_into_lora_sendbuffer();
  #ifdef DEBUGLEVEL2
  Serial.print(F("\Completed: doGPS_and_put_values_into_sendbuffer. milis=")); Serial.println(millis());
  #endif
}

void gps_init() {  
  #ifdef DEBUGLEVEL2
  Serial.print(F("\nGPS init. milis=")); Serial.println(millis());
  #endif

    
  // load the send buffer with dummy location 0,0. This location 0,0 is recognized as dummy by TTN Mapper and will be ignored
  //l_lat = 526324000; l_lon = 47388000; l_alt = 678; hdopNumber = 23459;   // Alkmaar
  l_lat = 0; l_lon = 0; l_alt = 678; hdopNumber = 99999;   // the zero position
  put_gpsvalues_into_lora_sendbuffer(); 
  
//  Serial.print( F("The NeoGps people are proud to show their smallest possible size:\n") );
//  Serial.print( F("NeoGps, fix object size = ") ); Serial.println( sizeof(gps.fix()) );
//  Serial.print( F("NeoGps, NMEAGPS object size = ") ); Serial.println( sizeof(gps) );

  #ifdef NMEAGPS_NO_MERGING
    #ifdef DEBUGLEVEL2
    Serial.println( F("Only displaying data from xxRMC sentences.\n Other sentences may be parsed, but their data will not be displayed.") );
    #endif
  #endif
  
  //Serial.flush();
  ss.begin(9600);
  
  //gps_requestColdStart();  // DO NOT USE: it seems this does a FACTORY RESET and delays getting a solid fix
//  gps_SetMode_gpsRfOn();
  gps_setStrings();
//  gps_setNavMode(7); // 2=stationary, 3=pedestrian, 4=auto, 5=Sea, 6=airborne 1g, 7=air 2g, 8=air 4g
  
//  gps_setPowerMode(1);  // 1=max power, 2=eco, 3=cyclic power save

  //assume fix was found, go to airborne
  // Lessons learned,
  //       my gps does not find fix while on the ground in mode 6
  //       does find fix in mode 7, but no re-fix after RF-off and RF-on
  //       so for now we keep in mode 4 OR experiment with powermodes instead of RF_off
  // gps_setNavMode(7); // 2=stationary, 3=pedestrian, 4=auto, 5=Sea, 6=airborne 1g, 7=air 2g, 8=air 4g -- with 6 no 2d fix supported
//   gps_read_until_fix_or_timeout(30 * 60);  // after factory reset, time to first fix can be 15 minutes (or multiple).  gps needs to acquire full data which is sent out once every 15 minutes; sat data sent out once every 5 minutes
//gps_read_chars(200);
  //gps_setPowerMode(2);
}

void gps_setStrings() {
  #ifdef DEBUGLEVEL2
  Serial.print(F("\nStart: gps_setStrings. milis=")); Serial.println(millis());
  #endif

  // Turning ON or OFF  GPS NMEA strings 
  // we need lat, lon, alt, HDOP  --> keep GGA

  // GLL = Lat/Lon time fix
  //ss.print(F("$PUBX,40,GLL,0,0,0,0*5C\r\n"));  // GLL OFF
  ss.print(F("$PUBX,40,GLL,1,1,1,0*5D\r\n"));  // GLL ON

  // ZDA = date, time
  ss.print(F("$PUBX,40,ZDA,0,0,0,0*44\r\n"));  // ZDA OFF
  //ss.print(F("$PUBX,40,ZDA,1,1,1,0*45\r\n"));  // ZDA ON 
  
  // VTG = Vector Track and speed over ground
  ss.print(F("$PUBX,40,VTG,0,0,0,0*5E\r\n"));  // VTG OFF
  //ss.print(F("$PUBX,40,VTG,1,1,1,0*5F\r\n"));  // VTG ON

  // GSV = Satellite in View. #sentences,sentence#,#sat,[sat PRN#, elev degr, azi degr, SNR,] *check
  ss.print(F("$PUBX,40,GSV,0,0,0,0*59\r\n"));  //GSV OFF
  //ss.print(F("$PUBX,40,GSV,1,1,1,0*58\r\n"));  //GSV ON

  // RMC = recommended minimum, no Alt
  // ss.print(F("$PUBX,40,RMC,0,0,0,0*47\r\n"));    // RMC OFF
  ss.print(F("$PUBX,40,RMC,1,1,1,0*46\r\n"));      // RMC ON 

  // GSA = Overall Satelite status. Auto/Manual,1/2/3 D fix, PRN1, ...PRN12 satt id, pdop,hdop,vdop,*check
  ss.print(F("$PUBX,40,GSA,0,0,0,0*4E\r\n"));  // GSA OFF
  //ss.print(F("$PUBX,40,GSA,1,1,1,0*4F\r\n"));  // GSA ON

  // GGA = Fix information. time,lat,N,lon,E,fix qual,num sat,hor dilution, alt,M,height geoid,M,time since DGPS,DGPS id, *check
  // ss.println(F("$PUBX,40,GGA,0,0,0,0*5A"));   // GGA OFF
  ss.println(F("$PUBX,40,GGA,1,1,1,0*5B"));   // GGA ON

  //GRS
  //$PUBX,40,GRS,0,0,0,0*5D // Turn OFF
  //$PUBX,40,GRS,1,1,1,0*5C // Turn ON

  //GST
  //$PUBX,40,GRS,0,0,0,0*5D // Turn OFF
  //$PUBX,40,GRS,1,1,1,0*5C // Turn ON

  //gps_read_chars(300);
  
  unsigned long gps_listen_startTime = millis(); 
  unsigned long gps_timeout = 3;  // sec
  
  int times_without_char=0;
  while((millis() - gps_listen_startTime) < (gps_timeout * 1000L)) {
    // for debugging
    if (ss.available()) {
      char c = ss.read();
      Serial.write(c);
    }
    if(times_without_char++>30 && times_without_char<60) Serial.write(".");
  }
  #ifdef DEBUGLEVEL2
  Serial.println();
  #endif
  // #define LAST_SENTENCE_IN_INTERVAL NMEAGPS::NMEA_RMC

  // our GN-801 works well
  //     used to output: $GNVTG, $GNGGA, $GNGSA, $GPGSV, $GLGSV, $GNGLL. 
  //     Changed to --> RMC, GGA, GLL
}

//////////////////////////////////////////////////
// Kaasfabriek routines for LMIC_slim for LoraWan
///////////////////////////////////////////////

void lmic_slim_init() {
  #ifdef DEBUGLEVEL2
  Serial.print("\nStart: lmic_slim_init. milis="); Serial.println(millis());  
  #endif
  spi_start();
  pinMode(SS_pin, OUTPUT);                                                                  
  pinMode(SCK_pin, OUTPUT);                                         
  pinMode(MOSI_pin, OUTPUT);
  digitalWrite(SCK_pin, LOW);            // SCK low
  digitalWrite(SS_pin, HIGH);            // NSS high
  delay(10);
  writeReg(0x01, 0x08);
  delay(10);
  radio_init();  // that is in the LMIC_slim library
  delay(10);
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession (DEVADDR, nwkskey, appskey);
  
  LMIC_LORARegModemConfig (0x72, 0b01110100, 0x04);  // LORARegModemConfig1, LORARegModemConfig2, LORARegModemConfig3
    // USE THESE SETS FOR CORRECT LORA SPEC:
    //        0x72, 0b11000100, 0x0C  // BW=125, Coding Rate=4/5, SF12, mobile
    //        0x72, 0b10110100, 0x0C  // BW=125, Coding Rate=4/5, SF11, mobile
    //        0x72, 0b10100100, 0x04  // BW=125, Coding Rate=4/5, SF10, static
    //        0x72, 0b10010100, 0x04  // BW=125, Coding Rate=4/5, SF9, static 
    //        0x72, 0b10000100, 0x04  // BW=125, Coding Rate=4/5, SF8, static
    //        0x72, 0b01110100, 0x04  // BW=125, Coding Rate=4/5, SF7, static -> this is preferred setting for generic nodes and is the TTNmapper default
    // LORARegModemConfig1
    //       0x72 is normal BW=125 en Coding Rate=4/5, all other messages I see on our TTN basestation have BW=125 en Coding Rate=4/5
    // LORARegModemConfig2 
    //       nnnn----   Spreading Factor (bit 7..4)
    //       ----0---   TxContinuousMode =0 normal mode (bit 3)  
    //       -----1--   RxPayloadCrcOn = 1 CRC ON (bit 2)  
    //       ------00   SymbTimeout(9:8)=00 default (bit 1..0)
    // LORARegModemConfig3, bit 3 van RegModemConfig3, spec van de RFM95 zegt: bit 3: MobileNode, type: rw, Default: 0x00, Value: 0 : Use for static node
    //       0x0C  originally by Rene Harte, IOT-partners for SF11, SF12  
    //       0x04  for SF7.....SF10 
    // Airtime voor 5 bytes payload = 13 x 2^(SF-6) ms. 
    //       Number of sec per message with 30-50 bytes: roughly SF12 = 2 seconds, SF10 = 0,5 sec, SF8 = 120 msec, SF7= 60 msec. One device is allowed a total of 30 seconds per day airtime.
    //       Message frequency allowed SF12 = 15 messages per day = 1 message every 90 minutes; SF11 = 1 per 45 min; SF10 = 1 per 24 min, SF9 = 1 per 12 min, SF8 = 1 per 6 min, SF7 = 1 per 3 min roughly
    // For your received messages:
    //       RSSI  Received signal strength indicator, as low as -120 is told to be workable, but I have seen it work even lower. I get -120 from my work table. Does not change with SF.
    //       SNR   Signal to noise ratio, as low as -20dB seems workable. I get -5 from my work table.
}

void print_myLoraWanData() {
  #ifdef DEBUGLEVEL2
  Serial.print(F("  myLoraWanData = [")); 
  //Serial.print((char*)myLoraWanData); Serial.println("]"); Serial.print(F("                  [ "));  
  #endif
  for(int i=0; i<30; i++) {  
    if (myLoraWanData[i] < 16) 
    #ifdef DEBUGLEVEL2 
    Serial.print("0") 
    #endif 
    ; 
    #ifdef DEBUGLEVEL2
    Serial.print(myLoraWanData[i], HEX); 
    Serial.print(F(" "));
    #endif  
  }  
  #ifdef DEBUGLEVEL2
  Serial.println(F(" .. ]"));
  #endif
}

void doOneLoraWan() {
  #ifdef DEBUGLEVEL2
  Serial.print("\nStart: Do one lora. milis="); Serial.println(millis());
  print_myLoraWanData();
  #endif
  
  LMIC_setTxData2(myLoraWanData, PAYLOADSIZE);
  radio_init();                                                       
  delay (10);
  //digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(LEDPIN, !digitalRead(LEDPIN));
  #ifdef DEBUGLEVEL2
  Serial.print("  txLora. milis="); Serial.println(millis());
  #endif
  txlora();
  #ifdef DEBUGLEVEL2
  Serial.print("  txLora completed. milis="); Serial.println(millis());
  #endif
  delay(200);           // this is a simple wait with no checking for TX Ready. Sdjust this for your SF.
                          // Airtime voor 5 bytes payload = 13 x 2^(SF-6) ms. 
                          // with 30-50 bytes: SF12 = 2 seconds, SF10 = 0,5 sec, SF8 = 120 msec, SF7= 70 msec. One device has 30 seconds per day airtime.
  //digitalWrite(LED_BUILTIN, LOW);
  #ifdef DEBUGLEVEL2
  Serial.print("  send time delay completed. milis="); Serial.println(millis());
  #endif
  digitalWrite(LEDPIN, !digitalRead(LEDPIN));
  setopmode(0x00);                     // opmode SLEEP; better not tell lorawan to go to sleep before message is done
  #ifdef DEBUGLEVEL2
  Serial.print("Completed: Do one lora. milis="); Serial.println(millis());
  #endif
}

//////////////////////////////////////////////////
// Kaasfabriek routines for RFM95 radio to radio 
///////////////////////////////////////////////

// radiohead radio 2 radio
#include <RH_RF95.h>  
/* for feather32u4 */  // this is what we use in Junior IOT Challenge 2018 by Dataschrift & Kaasfabriek Alkmaar 
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 7
// Change to 434.0 or other frequency, must match RX's freq!
// You can dial in the frequency you want the radio to communicate on, such as 915.0, 434.0 or 868.0 or any number really. Different countries/ITU Zones have different ISM bands so make sure you're using those or if you are licensed, those frequencies you may use
#define RF95_FREQ 868.0   // this is what we use in Junior IOT Challenge 2018 by Dataschrift & Kaasfabriek Alkmaar 
// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

int16_t packetnum = 0;  // packet counter, we increment per transmission
bool ReceivedFromRadio = false;
// radio buf
uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];

void doOneRadio() {
  #ifdef DEBUGLEVEL2
  Serial.print("\nStart: Do one radio. milis="); Serial.println(millis());  
  #endif
  
  uint8_t radiopacket[10];
  
  formatRadioPackage(&radiopacket[0]);
  #ifdef DEBUGLEVEL2
  Serial.println("Sending..."); delay(10);
  #endif
  rf95.send(radiopacket, 10);

  // now, sending is done. start listening
  //Serial.println("Waiting for radio packet to complete..."); delay(10);
  rf95.waitPacketSent();
  // Now wait for a reply  
  uint8_t len = sizeof(buf);
  
  #ifdef DEBUGLEVEL2
  Serial.println("Waiting for some other radio to reply..."); delay(10);
  #endif
  if (!rf95.waitAvailableTimeout(20000)) { Serial.println("No radio received in 20 sec, is there anyone around in same send settings?"); }
  else { 
    // a message was received
    if (!rf95.recv(buf, &len)) { Serial.println("Receive buffer is empty."); }
    else {
      // message has a length
      RH_RF95::printBuffer("Received this radio message: ", buf, len);
      #ifdef DEBUGLEVEL2
      Serial.println("Got reply:              [");
      #endif
      decodeReply(buf, 1/*debug*/);
      //Serial.print((char*)buf);
      #ifdef DEBUGLEVEL2
      Serial.print("] RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);
      #endif
      // RSSI values, indication for Wifi: http://www.metageek.com/training/resources/understanding-rssi.html
      //   -45 dBm  =  60 cm distance
      //   -110 dBm  =  probably 200 meter in streets
      //   -120 dBm  =  not good
      //   -125 dBm  =  unusable 
      ReceivedFromRadio = true; 
    }
  }

  // end loop
  #ifdef DEBUGLEVEL2
  Serial.print("\nCompleted: Do one radio. milis="); Serial.println(millis());
  #endif 
}

void halt_stressed() {  
  #ifdef DEBUGLEVEL2
  Serial.print("\nPanic. Halted. milis="); Serial.println(millis());  
  #endif
  while(1) {
    digitalWrite(LEDPIN, HIGH);   
    delay(150);
    digitalWrite(LEDPIN, LOW);  
    delay(50);
    Serial.print("x");  
  }
}   

void setupRadio() {  
  #ifdef DEBUGLEVEL2
  Serial.print("\nSetup radio. milis="); //Serial.println(millis());  
  #endif
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  // hard reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  if (!rf95.init()) {
    Serial.println("Radio init failed");
    //while (1);
    halt_stressed();
  }
  #ifdef DEBUGLEVEL2
  Serial.println("Radio init OK!");
  #endif

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    //while (1);
    halt_stressed();
  }
  #ifdef DEBUGLEVEL2
  Serial.print("Freq is set to: "); 
  Serial.println(RF95_FREQ);
  #endif
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  
  //rf95.setTxPower(23, false);
  rf95.setTxPower(13, false);   // is 13 TTN & TTNtracker intended spec?
}

/*
   byte 0          My ID      My ID and message type
        0b0000 0000            
          ---- nnnn MessType   
          ---- 0001 msg#1      Yelling out loud that I have fired
          ---- 0010 msg#2      You have fired and here is my answer 
          nnnn ---- MyID       
    byte 1, 2, 3    MyLat      
    byte 4, 5, 6    MyLon      
    byte 7          MyComp ++   
        0b0000 0000            
          -nnn nnnn MyComp     
          1--- ---- MyBtn#1       
    byte 8          RemoteID   Your ID, hey I am talkming to you
        0b0000 0000            
          ---- ---n WasIhit    Hit indicator
          nnnn ---- RemoteID   Value 0-31, Remote team ID
    byte 9          Validator  Hash (binary add) on message, GPS date, salt..
    */
void formatRadioPackage(uint8_t *loopbackToData) {
  bool didIFire = true;
  bool didSomeoneElseFire = false;
  
  bool shouldITalkBack = false;
  uint8_t MyID = 1;
  uint8_t buttonPressed = 0b10000000;
  uint8_t targetID = 0b00000000; // unknown
  
  if(didIFire) {
    loopbackToData[0] = 0b00000001;
    loopbackToData[8] = targetID; // send unknown    
  } else if(didSomeoneElseFire && shouldITalkBack) {
    loopbackToData[0] = 0b00000010;
    loopbackToData[8] |= whoWasItThatTalkedToMe() << 4;
    loopbackToData[8] |= wasIHit();
  }
  loopbackToData[0] |= MyID << 4;

  doGPS(10); // must have a gps - wait up to 10 seconds

  // maybe we should make a function for lat lng encoding that doesnt put them to lorawan
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

  loopbackToData[1] = ( LatitudeBinary >> 16 ) & 0xFF;
  loopbackToData[2] = ( LatitudeBinary >> 8 ) & 0xFF;
  loopbackToData[3] = LatitudeBinary & 0xFF;
  loopbackToData[4] = ( LongitudeBinary >> 16 ) & 0xFF;
  loopbackToData[5] = ( LongitudeBinary >> 8 ) & 0xFF;
  loopbackToData[6] = LongitudeBinary & 0xFF;

  // maybe we should make a function for compass encoding that doesnt put them to lorawan
  long compass = readCompass(); // 0..360 deg
  uint8_t compass_bin = compass/3 ;  // rescale 0-360 deg into 0 - 120 values and make sure it is not bigger than one byte
  // now add a bit for BTN (not implemented)
  loopbackToData[7] = compass_bin;
  #ifdef DEBUG
  Serial.print(F("  compass=")); Serial.print(compass); Serial.print(F("  deg. compass_bin=")); Serial.println(compass_bin);
  #endif

  loopbackToData[7] |= buttonPressed;

  loopbackToData[9] = 0x00; // What is this?
}
/*
   byte 0          My ID      My ID and message type
        0b0000 0000            
          ---- nnnn MessType   
          ---- 0001 msg#1      Yelling out loud that I have fired
          ---- 0010 msg#2      You have fired and here is my answer 
          nnnn ---- MyID       
    byte 1, 2, 3    MyLat      
    byte 4, 5, 6    MyLon      
    byte 7          MyComp ++   
        0b0000 0000            
          -nnn nnnn MyComp     
          1--- ---- MyBtn#1       
    byte 8          RemoteID   Your ID, hey I am talkming to you
        0b0000 0000            
          ---- ---n WasIhit    Hit indicator
          nnnn ---- RemoteID   Value 0-31, Remote team ID
    byte 9          Validator  Hash (binary add) on message, GPS date, salt..
    */
void decodeReply(uint8_t buf[], bool debugToSerial) {
  if(debugToSerial) {
    #ifdef DEBUGRADIO
    // bytes 0
    if((buf[0] & 0b00001111) == 0b00000001) {
      Serial.println("Radio: Someone says that he fired");
    } else if((buf[0] & 0b00001111) == 0b00000010) {
        Serial.println("Radio: Someone talkes back to someone who fired");
    }
    Serial.print("That someone has an id of:");
    uint8_t id = (buf[0] >> 4) & 0b00001111;
    Serial.println((int) id,DEC);
    
    // byte 1,2,3 and 4,5,6
    Serial.print("His location is: ");
    
    float _lat = ((((uint32_t)buf[1]) << 16) + (((uint32_t)buf[2]) << 8) + buf[3]) / 16777215.0 * 180.0 - 90;
    float _lng = ((((uint32_t)buf[4]) << 16) + (((uint32_t)buf[5]) << 8) + buf[6]) / 16777215.0 * 360.0 - 180;
    Serial.print("lat: ");
    Serial.print(_lat);
    Serial.print("lng: ");
    Serial.println(_lng);

    // byte 7
    uint8_t compass = buf[7] & 0b01111111; // don't want the hit indicator now
    Serial.print("His compass points to: ");
    int _compass = (compass & 127)*3;
    Serial.println(_compass);

    bool hePressedHisButton = ((buf[7] >> 7) & 0b00000001) == 0b00000001;
    if(hePressedHisButton) Serial.println("He pressed his button");
    else Serial.println("He did not press his button");

    // byte 8
    bool heWasHit = (buf[8] & 0b00000001) == 0b00000001;
    if(heWasHit) Serial.println("He was hit");
    else Serial.println("He was not hit - or doesn't know it yet");

    uint8_t remoteid = (buf[8] >> 4) & 0b00001111;
    Serial.println("He was talking to id: ");
    Serial.print((int)remoteid,DEC);

    // byte 9 - what is this?
    #endif
  }
}
uint8_t whoWasItThatTalkedToMe() {
  uint8_t who = 2; // 2 talks to me
  return who;
}
uint8_t wasIHit() {
  uint8_t hit = 0b00000001; // yes i was hit
  return hit;
}

///////////////////////////////////////////////
//  some other measurements
///////////////////////////////////////////

double GetTemp(void) { //http://playground.arduino.cc/Main/InternalTemperatureSens
  unsigned int wADC;
  double t;
  
  // The internal temperature has to be used with the internal reference of 1.1V. ATmega32U4 has 2.56V ref instead of 1.1?
  // Set the internal reference and mux.
  ADMUX = (_BV(REFS1) | _BV(REFS0) | _BV(MUX3));
  ADCSRA |= _BV(ADEN);  // enable the ADC
  delay(20);            // wait for voltages to become stable.
  ADCSRA |= _BV(ADSC);  // Start the ADC

  // Detect end-of-conversion
  while (bit_is_set(ADCSRA,ADSC));  
  // Reading register "ADCW" takes care of how to read ADCL and ADCH.
  wADC = ADCW;
  
  t = (wADC - 324.31 ) / 1.22;  // The offset of 324.31 could be wrong. It is just an indication.
  // The returned temperature is in degrees Celsius.
  //t = t - 10; // before this correction we had readings of 30 in a room which was 20 deg C, readings of 20 outdoors when it was 6 deg C

  return (t);
}

long readVccCPU() {  //http://dumbpcs.blogspot.nl/2013/07/arduino-secret-built-in-thermometer.html
  long result;
  // Read 1.1V reference against AVcc 
  //ATmega32U4 has 2.56V ref instead of 1.1?
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  return result;
}

long readVbat() {
  long result;
  float measuredvbat = analogRead(VBATPIN);
    // devide by 1024 to convert to voltage
    // we divided by 2 using 2x 100M Ohm, so multiply back
    // Multiply by 3.3V, our reference voltage
    // *1000 to get milliVolt
  measuredvbat *= 6600 / 1024;         
  result = measuredvbat;
  Serial.print(F("  Vbat=") ); 
  Serial.print(result);
  Serial.println(F(" milliVolt"));
  return result;
}

void put_Compass_and_Btn_into_sendbuffer() {
  long compass = readCompass(); // 0..360 deg
  uint8_t compass_bin = compass/3 ;  // rescale 0-360 deg into 0 - 120 values and make sure it is not bigger than one byte
  // now add a bit for BTN (not implemented)
  myLoraWanData[9] = compass_bin;
  #ifdef DEBUG
  Serial.print(F("  compass=")); Serial.print(compass); Serial.print(F("  deg. compass_bin=")); Serial.println(compass_bin);
  #endif
}

void put_Volts_and_Temp_into_sendbuffer() {

  long vcc = readVccCPU();
  //long vcc = 0;
  uint8_t vcc_bin = vcc/20 ;  // rescale 0-5100 milli volt into 0 - 255 values and make sure it is not bigger than one byte
  myLoraWanData[10] = vcc_bin;
  #ifdef DEBUG
  Serial.print(F("  Vcc=")); Serial.print(vcc); Serial.print(F(" mV. vcc_bin=")); Serial.println(vcc_bin);
  #endif
  
  double temperature = GetTemp();
  uint8_t temperature_bin = temperature + 100;   // rescale -100 to 155 into 0 - 255 values and make sure it is not bigger than one byte
  myLoraWanData[11] = temperature_bin;
  #ifdef DEBUG
  Serial.print(F("  Temp=")); Serial.print(temperature); Serial.print(F(" bin=")); Serial.println(temperature_bin);
  #endif
  
  long vbat = readVbat();
  uint8_t vbat_bin = vbat/20 ;  // rescale 0-5100 milli volt into 0 - 255 values and make sure it is not bigger than one byte
  myLoraWanData[12] = vbat_bin;
  #ifdef DEBUG
  Serial.print(F("  Vbat=")); Serial.print(vbat); Serial.print(F(" mV. vbat_bin=")); Serial.println(vbat_bin);
  #endif
}

void put_TimeToFix_into_sendbuffer(int TimeToFix_Seconds) {  // time to fix onto gps coordinates
//  int TimeToFix_Calculate;  // this helps to calculate but no round-off yet
//  if ( TimeToFix_Seconds < 0) {
//    TimeToFix_Calculate=0;
//  } else if ( TimeToFix_Seconds <= (1 * 60) ) {  
//    TimeToFix_Calculate = TimeToFix_Seconds;                  // 0..60 sec  at 1 sec interval <==> values 0 .. 60 
//  } else if ( TimeToFix_Seconds <= (10 * 60) ) {    
//    TimeToFix_Calculate = 60 + (TimeToFix_Seconds - (1* 60) )/5 ;   // 1..10 min at 5 sec interval  <==> values 60 ..  168 
//  } else if ( TimeToFix_Seconds <= (60 * 60) ) {  
//    TimeToFix_Calculate = 168 + (TimeToFix_Seconds - (10 * 60) )/60 ;   // 10..60 min at 1 min interval <==> values 168 .. 218     
//  } else {
//    TimeToFix_Calculate = 218 + (TimeToFix_Seconds - (60 * 60) )/600 ;    // 1..7:00 hour at 10 min interval <==> values 218 ..254   
//  }
//  if (TimeToFix_Calculate>255) TimeToFix_Calculate = 255 ;                  //  more than 7 hour = 255
//  
//  uint8_t TimeToFix_bin = TimeToFix_Calculate;  // this can contain the values 0..255,
//      
//  myLoraWanData[11] = TimeToFix_bin;
//  #ifdef DEBUG
//  Serial.print(F("TTF=")); Serial.print(TimeToFix_Seconds); Serial.print(F(" sec. bin=")); Serial.print(TimeToFix_bin);
//  #endif
}

//////////////////////////////////////////////////////////
//// Compass HMC5983
////////////////////////////////////////////
void setupCompass() {
  
  //  HMC5983 first 4 pins are soldered to first 4 pins on lora32u4
  //   Vin to pin 6
  //   GND to pin 5
  //   SCL to SCL = 3
  //   SDA to SDA = 2
  pinMode(6, OUTPUT);
  pinMode(5, OUTPUT);
  digitalWrite(6, HIGH);
  digitalWrite(5, LOW);
  
  Wire.begin();
  delay(100);
  
  Wire.beginTransmission(IIC_HMC5983_address); 
  Wire.write(0x02); // Select mode register
  Wire.write(0x00); // Continuous measurement mode
  Wire.endTransmission();
}

long readCompass() {
  //---- X-Axis
  Wire.beginTransmission(IIC_HMC5983_address); // transmit to device
  Wire.write(HMC5983_selector_for_X_byte1);
  Wire.endTransmission();
  Wire.requestFrom(IIC_HMC5983_address,1); 
  if(Wire.available()<=1)   
  {
    HMC5983_X_byte0 = Wire.read();
  }
  Wire.beginTransmission(IIC_HMC5983_address); // transmit to device
  Wire.write(HMC5983_selector_for_X_byte0);
  Wire.endTransmission();
  Wire.requestFrom(IIC_HMC5983_address,1); 
  if(Wire.available()<=1)   
  {
    HMC5983_X_byte1 = Wire.read();
  }
  //---- Y-Axis
  Wire.beginTransmission(IIC_HMC5983_address); // transmit to device
  Wire.write(HMC5983_selector_for_Y_byte1);
  Wire.endTransmission();
  Wire.requestFrom(IIC_HMC5983_address,1); 
  if(Wire.available()<=1)   
  {
    HMC5983_Y_byte0 = Wire.read();
  }
  Wire.beginTransmission(IIC_HMC5983_address); // transmit to device
  Wire.write(HMC5983_selector_for_Y_byte0);
  Wire.endTransmission();
  Wire.requestFrom(IIC_HMC5983_address,1); 
  if(Wire.available()<=1)   
  {
    HMC5983_Y_byte1 = Wire.read();
  }
  
  //---- Z-Axis
  Wire.beginTransmission(IIC_HMC5983_address); // transmit to device
  Wire.write(HMC5983_selector_for_Z_byte1);
  Wire.endTransmission();
  Wire.requestFrom(IIC_HMC5983_address,1); 
  if(Wire.available()<=1)   
  {
    HMC5983_Z_byte0 = Wire.read();
  }
  Wire.beginTransmission(IIC_HMC5983_address); // transmit to device
  Wire.write(HMC5983_selector_for_Z_byte0);
  Wire.endTransmission();
  Wire.requestFrom(IIC_HMC5983_address,1); 
  if(Wire.available()<=1)   
  {
    HMC5983_Z_byte1 = Wire.read();
  }
  
  //---- X-Axis
  HMC5983_X_byte1 = HMC5983_X_byte1 << 8;
  HMC5983_X_value = HMC5983_X_byte0 + HMC5983_X_byte1; // Raw data
  // From the datasheet: 0.92 mG/digit
  X_milliGauss = HMC5983_X_value * 0.00092; // Gauss unit
  //* Earth magnetic field ranges from 0.25 to 0.65 Gauss, so these are the values that we need to get approximately.
  
  //---- Y-Axis
  HMC5983_Y_byte1 = HMC5983_Y_byte1 << 8;
  HMC5983_Y_value = HMC5983_Y_byte0 + HMC5983_Y_byte1;
  Y_milliGauss = HMC5983_Y_value * 0.00092;
  
  //---- Z-Axis
  HMC5983_Z_byte1 = HMC5983_Z_byte1 << 8;
  HMC5983_Z_value = HMC5983_Z_byte0 + HMC5983_Z_byte1;
  Z_milliGauss = HMC5983_Z_value * 0.00092;
  // ==============================
 
  // Correcting the heading with the geo_magnetic_declination_deg angle depending on your location
  // You can find your geo_magnetic_declination_deg angle at: http://www.ngdc.noaa.gov/geomag-web/
  // At zzz location it's 4.2 degrees => 0.073 rad
  // Haarlem                    2017-10-20  1° 4' E  ± 0° 22'  changing by  0° 9' E per year
  // Amsterdam                  2017-10-20  1° 9' E  ± 0° 22'  changing by  0° 9' E per year
  // Alkmaar 52.6324 4.7534     2017-10-20  1.09° E  ± 0.38°  changing by  0.14° E per year
  geo_magnetic_declination_deg = 1.09; // for our location
  
  //Calculating Heading
  headingDegrees = atan2(Y_milliGauss, X_milliGauss)* 180/PI + geo_magnetic_declination_deg;  // heading in rad. 
  
  // Correcting when signs are reveresed or due to the addition of the geo_magnetic_declination_deg angle
  if(headingDegrees <0) headingDegrees += 2*180;
  if(headingDegrees > 2*180) headingDegrees -= 2*180;
  
  // Smoothing the output angle / Low pass filter --- to make changes apeare slower
  //headingFiltered = headingFiltered*0.85 + headingDegrees*0.15;
  headingFiltered = headingDegrees;
  // We can do this, but then we need to take multiple readings and it will still go wrong if we take readings from the previous buttonpress in account
  // Because If i pressed a button at 180 degrees. And I press it again at 0 degrees, 180 is not relevant to be taking into account.
  
  //Sending the heading value through the Serial Port 
  #ifdef DEBUGLEVEL2
  Serial.print(headingDegrees);
  Serial.print(" filtered ");
  Serial.println(headingFiltered);
  #endif
  return headingFiltered;
}

///////////////////////////////////////////////
//  arduino init and main
///////////////////////////////////////////
unsigned long device_startTime;
bool has_sent_allready = false; 

void setup() {
  pinMode(LEDPIN, OUTPUT);
  delay(1000);  // https://www.thethingsnetwork.org/forum/t/got-adafruit-feather-32u4-lora-radio-to-work-and-here-is-how/6863
  
  Serial.begin(115200);   // whether 9600 or 115200; the gps feed shows repeated char and cannot be interpreted, setting high value to release system time
  delay(100);

  #ifdef DEBUGLEVEL2
  Serial.print(F("\nStarting device: ")); Serial.println(DEVADDR); 
  #endif
  device_startTime = millis();

  gps_init(); 
  lmic_slim_init();
  setupCompass();
  
  #ifdef DEBUGLEVEL2
  Serial.print(F("\nInit values. milis=")); Serial.println(millis());
  #endif
  put_Volts_and_Temp_into_sendbuffer();
  put_Compass_and_Btn_into_sendbuffer();
  doGPS_and_put_values_into_lora_sendbuffer();   

  #ifdef DEBUGLEVEL2
  Serial.print(F("\nSend one lorawan message as part of system init. milis=")); Serial.println(millis());
  #endif
//  LMIC_setTxData2(myLoraWanData, sizeof(myLoraWanData)-1);
//  radio_init();                                                       
//  delay (10);
//  txlora();
//  delay(200);                    // wacht op TX ready. Airtime voor 5 bytes payload = 13 x 2^(SF-6) ms
//  setopmode(0x00);                // opmode SLEEP
//  last_lora_time = millis();
  last_lora_time = millis();
  lmic_slim_init();
  doOneLoraWan();    
  
  #ifdef DEBUGLEVEL2
  Serial.print(F("\nCompleted: Setup. milis=")); Serial.println(millis());
  #endif
}

boolean radioActive = true;  // this name is for radio, not LoraWan
boolean loraWannaBe = false;

void loop() {
  #ifdef DEBUGLEVEL2
  Serial.print(F("\n==== Loop starts. milis=")); Serial.println(millis());
  #endif
  digitalWrite(LEDPIN, !digitalRead(LEDPIN)); 

  ////// GPS pre-loop //////////////
  // Serial.println(F("\nNo lengthy GPS read-till-fix is needed, the GPS will find/keep a fix as log as power is on. "));

  ////////// Radio  ///////////
  #ifdef DEBUGLEVEL2
  Serial.print(F("\nRadio listen? milis=")); Serial.println(millis());
  #endif
  // now listen a long time for a radio message which we may want to act on, or for a keypress on our side 
  // time needs to be long enough not to miss a radio, we do not worry about GPS as it will keep fix as long as powered
  if(radioActive) {
    setupRadio();
    while((millis() - last_lora_time) < (LORAWAN_TX_INTERVAL * 1000L)) {
    // next command is not what we want to do
    doOneRadio();  // sends a radio message and will listen for return message for a certain time
    delay(5000);
    }
    /*if(ReceivedFromRadio) {
      // use the radio message content for Lora
      memcpy(myLoraWanData,buf,PAYLOADSIZE);
      ReceivedFromRadio = false;
    } else {
      sprintf(myLoraWanData,"xx geen radio ontvangen xx");
    } */ 
  } else {
    //not listening to radio at all, we may as well use delay for a bit 
    #ifdef DEBUGLEVEL2
    Serial.print(F("  No radio listen required, so instead just add a delay before lorawan: \n    ")); Serial.print(LORAWAN_TX_INTERVAL); Serial.print(F(" sec."));
    #endif
    while((millis() - last_lora_time) < (LORAWAN_TX_INTERVAL * 1000L)) {
      delay(5000);   
      Serial.print(F("."));
    }
    Serial.println();
  }
  // we keep doing this part until it is time to send one LORAWAN TX to the worl

  ////////// Collect data needed just before sending a LORAWAN update to the world  ///////////
  //Serial.println(F("\nCollect data needed just before sending a LORAWAN update. milis=")); //Serial.println(millis());
  put_Volts_and_Temp_into_sendbuffer();
  put_Compass_and_Btn_into_sendbuffer();

  // Clear gps buffer, or else it will retail old position if no fix is found. If no fix is found we want invalid location so TTNmapper does not get disturbed.
  l_lat = 0; l_lon = 0; l_alt = 678; hdopNumber = 99999;   // the zero position
  doGPS_and_put_values_into_lora_sendbuffer();

  ////////// Now we need to send a LORAWAN update to the world  ///////////
  // switch the LMIC antenna to LoraWan mode
  #ifdef DEBUGLEVEL2
  Serial.println(F("Time or button press tells us to send one LoraWan. milis=")); Serial.println(millis());
  #endif
  last_lora_time = millis();
  lmic_slim_init();
  doOneLoraWan();    
  
  /////////// Loop again  //////////////
  #ifdef DEBUGLEVEL2
  Serial.println(F("\nEnd of loop. milis=")); Serial.println(millis());
  #endif
}



