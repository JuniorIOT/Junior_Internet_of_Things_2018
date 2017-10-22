/*******************************************************************************
 * Modified By DenniZr & Marco van Schagen for Junior IOT - Smart City Challenge 2018 #juniorIOTchallenge2018
 * Modified By Marco van Schagen for Junior IOT Challenge 2018
 *******************************************************************************/ 
 
//#define DEBUG     // if DEBUG is defined, some code is added to display some basic debug info



#include <rn2xx3.h>
#include "keys.h"
#define mySerial Serial2
#define PAYLOADSIZE 9 // The size of the package to be sent

//create an instance of the rn2xx3 library,
//giving the software serial as port to use
rn2xx3 myLora(mySerial);

byte data[10] = {0xAA, 0xAA,0xAA, 0xAA,0xAA, 0xAA,0xAA, 0xAA,0xAA, 0xAA};

// the setup routine runs once when you press reset:

// Battery voltage sodaq one
/*#define ADC_AREF 3.3f
#define BATVOLT_R1 2.0f // One v1
#define BATVOLT_R2 2.0f // One v1
// #define BATVOLT_R1 4.7f // One v2
// #define BATVOLT_R2 10.0f // One v2
#define BATVOLT_PIN BAT_VOLT
*/

#define LEDPIN LED_BUILTIN

#define LORAWAN_TX_INTERVAL 240  // seconds between LoraWan messages

//////////////////////////////////////////////
// GPS libraries, mappings and things
//////////////////////////////////////////////
#define GPS_FIX_HDOP   // to prevent eror: 'const class gps_fix' has no member named 'hdop'
#define GPS_RXD_PIN D0  
#define GPS_TXD_PIN D1    // where we plugged in our GNSS GPS into Lora32u4

/*#include <NeoSWSerial.h>  //  We now use NeoSWSerial for lower footprint end better performance than SoftwareSerial
  // an issue with Leonardo-types is fixed in branch, yet to be merged into main version library. So you may need to remove all your NeoSWSerial libraries and add \libraries\NeoSWSerial-master-DamiaBranch.zip
NeoSWSerial ss(GPS_RXD_PIN, GPS_TXD_PIN);
// not supported for sodaq explorer
*/
#define ss Serial
/*
 * need gps hack
 *  .arduino15/packages/SODAQ/hardware/samd/1.6.14/cores/arduino/avr/pgmspace.h aanpassen. #define pgm_read_ptr(addr) (*(void * const *)(addr))//hack was *(const void *)(addr))
 */

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

int TX_COMPLETE_was_triggered = 0;  // 20170220 added to allow full controll in main Loop
uint8_t  myLoraWanData[40];  // including byte[0]
unsigned long last_lora_time = millis(); // last time lorawan ran

//--------------Table of contents------------//
//////////////////////////////////////////////////////////
//// Kaasfabriek routines for gps
////////////////////////////////////////////
//void put_gpsvalues_into_sendbuffer(long l_lat, long l_lon, long l_alt, int hdopNumber);
void put_gpsvalues_into_sendbuffer();
//void process_gps_datastream(const gps_fix & fix );
void gps_init();
//////////////////////////////////////////////////
// Kaasfabriek routines for RN2483 for LoraWan
///////////////////////////////////////////////
void rn2483_init();
void doOneLoraWan();
//////////////////////////////////////////////////
// Kaasfabriek routines for radio to radio 
///////////////////////////////////////////////
void doOneRadio();
void halt_stressed();
void setupRadio();
///////////////////////////////////////////////
//  some other measurements
///////////////////////////////////////////
double GetTemp(void);
long readVbat();
void put_TimeToFix_into_sendbuffer(int TimeToFix_Seconds);
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

void put_gpsvalues_into_sendbuffer() {
  SerialUSB.println(F("Started: put_gpsvalues_into_sendbuffer"));
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
  
//  SerialUSB.print(F("  shift_lat = "));SerialUSB.println(shift_lat);
//  SerialUSB.print(F("  max_old_lat = "));SerialUSB.println(max_old_lat);
//  SerialUSB.print(F("  max_3byte = "));SerialUSB.println(max_3byte);
//  SerialUSB.print(F("  l_lat = "));SerialUSB.println(l_lat);
//  SerialUSB.print(F("  lat_float = "));SerialUSB.println(lat_float);
//  SerialUSB.print(F("  LatitudeBinary = "));SerialUSB.println(LatitudeBinary);
//  
//  SerialUSB.print(F("\n  shift_lon = "));SerialUSB.println(shift_lon);
//  SerialUSB.print(F("  max_old_lon = "));SerialUSB.println(max_old_lon);
//  SerialUSB.print(F("  l_lon = "));SerialUSB.println(l_lon);
//  SerialUSB.print(F("  lon_float = "));SerialUSB.println(lon_float);
//  SerialUSB.print(F("  LongitudeBinary = "));SerialUSB.println(LongitudeBinary);
//  
//  SerialUSB.print(F("\n  l_alt = "));SerialUSB.println(l_alt);
//  SerialUSB.print(F("  altitudeBinary = "));SerialUSB.println(altitudeBinary);
//  
//  SerialUSB.print(F("\n  hdopNumber = "));SerialUSB.println(hdopNumber);
//  SerialUSB.print(F("  HdopBinary = "));SerialUSB.println(HdopBinary);
  
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

void process_gps_datastream(const gps_fix & fix) {   // constant pointer to fix object
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
    SerialUSB.print("  Satellite time hh:mm:"); SerialUSB.print( fix.dateTime.seconds ); SerialUSB.print(", ");
    SerialUSB.print("lat "); SerialUSB.print( l_lat  ); SerialUSB.print(", ");
    SerialUSB.print("lon "); SerialUSB.print( l_lon ); SerialUSB.print(", ");
    SerialUSB.print("alt "); SerialUSB.print( l_alt ); SerialUSB.print(", ");
    SerialUSB.print("hdop "); SerialUSB.print( hdopNumber ); SerialUSB.print(", ");
    SerialUSB.print("sat count = "); SerialUSB.print( fix.satellites );
    //SerialUSB.print( fix.dateTime ); SerialUSB.print(" datetime, ");    
    //SerialUSB.print( fix.latitude(), 6 ); // floating-point display
    // SerialUSB.print( fix.longitude(), 6 ); // floating-point display
    //   Satellite time hh:mm:18, lat 526326595, lon 47384133, alt 21, hdop 990, sat count = 12
    
    SerialUSB.println();
  } else {
    SerialUSB.print( "(no valid location) " );
  }  
}

void doGPS_and_put_values_into_sendbuffer() {
  SerialUSB.print(F("\nStart: doGPS_and_put_values_into_sendbuffer. milis=")); SerialUSB.println(millis());
  // first we want to know GPS coordinates - we do accept a long delay if needed, even before listening to radio
  unsigned long gps_listen_startTime = millis(); 
  unsigned long gps_listen_timeout = 3;  // sec
  
  //now listen to gps till fix or time-out, once gps has a fix, the refresh should be ready within 2 data reads = less than 3 sec
  // gps read command:
  SerialUSB.println(F("Listen to GPS stream:"));
  while((millis() - gps_listen_startTime) < (gps_listen_timeout * 1000L)) {
    // for NMEAgps
    while (gps.available(ss)) {
      process_gps_datastream(gps.read()); 
    }
  }    
  SerialUSB.println(F("Completed listening to GPS."));
  
  // put gps values into send buffer
  put_gpsvalues_into_sendbuffer();
  SerialUSB.print(F("\Completed: doGPS_and_put_values_into_sendbuffer. milis=")); SerialUSB.println(millis());
}

void gps_init() {  
  SerialUSB.print(F("\nGPS init. milis=")); SerialUSB.println(millis());
    
  // load the send buffer with dummy location 0,0. This location 0,0 is recognized as dummy by TTN Mapper and will be ignored
  //put_gpsvalues_into_sendbuffer( 0, 0, 0, 0);
  l_lat = 52632400; l_lon = 4738800; l_alt = 678; hdopNumber = 2345;   // Alkmaar
  put_gpsvalues_into_sendbuffer(); 
  
//  SerialUSB.print( F("The NeoGps people are proud to show their smallest possible size:\n") );
//  SerialUSB.print( F("NeoGps, fix object size = ") ); SerialUSB.println( sizeof(gps.fix()) );
//  SerialUSB.print( F("NeoGps, NMEAGPS object size = ") ); SerialUSB.println( sizeof(gps) );

  #ifdef NMEAGPS_NO_MERGING
    SerialUSB.println( F("Only displaying data from xxRMC sentences.\n Other sentences may be parsed, but their data will not be displayed.") );
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
  SerialUSB.print(F("\nStart: gps_setStrings. milis=")); SerialUSB.println(millis());

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
  SerialUSB.println();
  
  // #define LAST_SENTENCE_IN_INTERVAL NMEAGPS::NMEA_RMC

  // our GN-801 works well
  //     used to output: $GNVTG, $GNGGA, $GNGSA, $GPGSV, $GLGSV, $GNGLL. 
  //     Changed to --> RMC, GGA, GLL
}
//////////////////////////////////////////////////
// Kaasfabriek routines for RN2483 for LoraWan
///////////////////////////////////////////////

void rn2483_init()
{
  mySerial.begin(57600); //serial port to radio
  
  //reset rn2483
  pinMode(12, OUTPUT);
  digitalWrite(12, LOW);
  delay(500);
  digitalWrite(12, HIGH);

  delay(100); //wait for the RN2xx3's startup message
  mySerial.flush();

  //Autobaud the rn2483 module to 9600. The default would otherwise be 57600.
  myLora.autobaud();

  //check communication with radio
  String hweui = myLora.hweui();
  while(hweui.length() != 16)
  {
    SerialUSB.println("Communication with RN2xx3 unsuccessful. Power cycle the board.");
    SerialUSB.println(hweui);
    delay(10000);
    hweui = myLora.hweui();
  }

  //print out the HWEUI so that we can register it via ttnctl
  SerialUSB.println("When using OTAA, register this DevEUI: ");
  SerialUSB.println(myLora.hweui());
  SerialUSB.println("RN2xx3 firmware version:");
  SerialUSB.println(myLora.sysver());

  //configure your keys and join the network
  SerialUSB.println("Trying to join TTN");
  bool join_result = false;

  //ABP: initABP(String addr, String AppSKey, String NwkSKey);
  join_result = myLora.initABP(DEVADDR, APPSKEY, NWKSKEY);

  //OTAA: initOTAA(String AppEUI, String AppKey);
  //join_result = myLora.initOTAA("70B3D57ED00001A6", "A23C96EE13804963F8C2BD6285448198");

  while(!join_result)
  {
    SerialUSB.println("Unable to join. Are your keys correct, and do you have TTN coverage?");
    RED();
    delay(60000); //delay a minute before retry
    join_result = myLora.init();
  }
  SerialUSB.println("Successfully joined TTN");
  GREEN();

  // SF is not in the library yet - maybe add it
  
}

void print_myLoraWanData() {
  SerialUSB.print(F("  myLoraWanData = [")); 
  //SerialUSB.print((char*)myLoraWanData); SerialUSB.println("]"); SerialUSB.print(F("                  [ "));  
  for(int i=0; i<30; i++) {  
    if (myLoraWanData[i] < 16) SerialUSB.print("0"); 
    SerialUSB.print(myLoraWanData[i], HEX); 
    SerialUSB.print(F(" "));  
  }  
  SerialUSB.println(F(" .. ]"));
}

void doOneLoraWan() {
  SerialUSB.print("\nStart: Do one lora. milis="); SerialUSB.println(millis());
  print_myLoraWanData();
    led_on();

    
    SerialUSB.print("  txLora. milis="); SerialUSB.println(millis());
    myLora.txBytes(myLoraWanData, sizeof(PAYLOADSIZE));
    SerialUSB.print("  txLora completed. milis="); SerialUSB.println(millis());
    led_off();
   SerialUSB.print("  send time delay completed. milis="); SerialUSB.println(millis());
  SerialUSB.print("Completed: Do one lora. milis="); SerialUSB.println(millis());
}


void led_on()
{
  digitalWrite(LEDPIN, 1);
}

void led_off()
{
  digitalWrite(LEDPIN, 0);
}

void RED() {
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);
}

void GREEN() {
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_BLUE, HIGH);
}

void BLUE() {
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, LOW);
}
int16_t packetnum = 0;  // packet counter, we increment per transmission
bool ReceivedFromRadio = false;
// radio buf
uint8_t buf[40];

void doOneRadio() {}; // not yet implemented
void setupRadio() {}; // not yet implemented

///////////////////////////////////////////////
//  some other measurements
///////////////////////////////////////////
double GetTemp(void) {
  pinMode(TEMP_SENSOR, INPUT);
  //10mV per C, 0C is 500mV
  float mVolts = (float)analogRead(TEMP_SENSOR) * 3300.0 / 1023.0;
  float temp = (mVolts - 500.0) / 10.0;
  return (double)temp;
}
long readVbat()
{
    /*uint16_t voltage = (uint16_t)((ADC_AREF / 1.023) * (BATVOLT_R1 + BATVOLT_R2) / BATVOLT_R2 * (float)analogRead(BATVOLT_PIN));

    return (long)voltage;*/
    return 0;
}

long readVccCPU() {return 0;}// not implemented

void put_Compass_and_Btn_into_sendbuffer() {
  //long compass = readCompass();
  long compass = 123;  // 0..360 deg
  uint8_t compass_bin = compass/3 ;  // rescale 0-360 deg into 0 - 120 values and make sure it is not bigger than one byte
  // now add a bit for BTN (not implemented)
  myLoraWanData[9] = compass_bin;
  //#ifdef DEBUG
  SerialUSB.print(F("  compass=")); SerialUSB.print(compass); SerialUSB.print(F("  deg. compass_bin=")); SerialUSB.println(compass_bin);
  //#endif
}

void put_Volts_and_Temp_into_sendbuffer() {

  long vcc = readVccCPU();
  //long vcc = 0;
  uint8_t vcc_bin = vcc/20 ;  // rescale 0-5100 milli volt into 0 - 255 values and make sure it is not bigger than one byte
  myLoraWanData[10] = vcc_bin;
  //#ifdef DEBUG
  SerialUSB.print(F("  Vcc=")); SerialUSB.print(vcc); SerialUSB.print(F(" mV. vcc_bin=")); SerialUSB.println(vcc_bin);
  //#endif
  
  double temperature = GetTemp();
  uint8_t temperature_bin = temperature + 100;   // rescale -100 to 155 into 0 - 255 values and make sure it is not bigger than one byte
  myLoraWanData[11] = temperature_bin;
  //#ifdef DEBUG
  SerialUSB.print(F("  Temp=")); SerialUSB.print(temperature); SerialUSB.print(F(" bin=")); SerialUSB.println(temperature_bin);
  //#endif
  
  long vbat = readVbat();
  uint8_t vbat_bin = vbat/20 ;  // rescale 0-5100 milli volt into 0 - 255 values and make sure it is not bigger than one byte
  myLoraWanData[12] = vbat_bin;
  //#ifdef DEBUG
  SerialUSB.print(F("  Vbat=")); SerialUSB.print(vbat); SerialUSB.print(F(" mV. vbat_bin=")); SerialUSB.println(vbat_bin);
  //#endif
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
//  SerialUSB.print(F("TTF=")); SerialUSB.print(TimeToFix_Seconds); SerialUSB.print(F(" sec. bin=")); SerialUSB.print(TimeToFix_bin);
//  #endif
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

  SerialUSB.print(F("\nStarting device: ")); SerialUSB.println(DEVADDR); 
  device_startTime = millis();

  gps_init(); 
  rn2483_init();

  SerialUSB.print(F("\nInit values. milis=")); SerialUSB.println(millis());
  put_Volts_and_Temp_into_sendbuffer();
  put_Compass_and_Btn_into_sendbuffer();
  doGPS_and_put_values_into_sendbuffer();   

  SerialUSB.print(F("\nSend one lorawan message as part of system init. milis=")); SerialUSB.println(millis());

  last_lora_time = millis();
  doOneLoraWan();
  SerialUSB.print(F("\nCompleted: Setup. milis=")); SerialUSB.println(millis());
  
}
boolean radioActive = false;  // this name is for radio, not LoraWan
boolean loraWannaBe = false;

void loop() {
  SerialUSB.print(F("\n==== Loop starts. milis=")); SerialUSB.println(millis());
  ////// GPS pre-loop //////////////
  // SerialUSB.println(F("\nNo lengthy GPS read-till-fix is needed, the GPS will find/keep a fix as log as power is on. "));

  ////////// Radio  ///////////
  SerialUSB.print(F("\nRadio listen? milis=")); SerialUSB.println(millis());
  // now listen a long time for a radio message which we may want to act on, or for a keypress on our side 
  // time needs to be long enough not to miss a radio, we do not worry about GPS as it will keep fix as long as powered
  if(radioActive) {
    setupRadio();
//while((millis() - last_lora_time) < (LORAWAN_TX_INTERVAL * 1000L)) {
    // next command is not what we want to do
    doOneRadio();  // sends a radio message and will listen for return message for a certain time
    
    if(ReceivedFromRadio) {
      // use the radio message content for Lora
      memcpy(myLoraWanData,buf,PAYLOADSIZE);
      ReceivedFromRadio = false;
    } else {
      //sprintf(myLoraWanData,"xx geen radio ontvangen xx");
    }  
  } else {
    //not listening to radio at all, we may as well use delay for a bit 
    SerialUSB.print(F("  No radio listen required, so instead just add a delay before lorawan: \n    ")); SerialUSB.print(LORAWAN_TX_INTERVAL); SerialUSB.print(F(" sec."));
    while((millis() - last_lora_time) < (LORAWAN_TX_INTERVAL * 1000L)) {
      delay(5000);   
      SerialUSB.print(F("."));
    }
    SerialUSB.println();
  }
  // we keep doing this part until it is time to send one LORAWAN TX to the worl

  ////////// Collect data needed just before sending a LORAWAN update to the world  ///////////
  SerialUSB.println(F("\nCollect data needed just before sending a LORAWAN update. milis=")); SerialUSB.println(millis());
  put_Volts_and_Temp_into_sendbuffer();
  put_Compass_and_Btn_into_sendbuffer();
  doGPS_and_put_values_into_sendbuffer();

  ////////// Now we need to send a LORAWAN update to the world  ///////////
  // switch the LMIC antenna to LoraWan mode
  SerialUSB.println(F("Time or button press tells us to send one LoraWan. milis=")); SerialUSB.println(millis());
  last_lora_time = millis();
  rn2483_init();
  doOneLoraWan();    
  
  /////////// Loop again  //////////////
  SerialUSB.println(F("\nEnd of loop. milis=")); SerialUSB.println(millis());
}
