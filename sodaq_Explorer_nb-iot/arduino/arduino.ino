/*******************************************************************************
 * Modified By DenniZr & Marco van Schagen for Junior IOT - Smart City Challenge 2018 #juniorIOTchallenge2018
 * Modified By Marco van Schagen for Junior IOT Challenge 2018
 *******************************************************************************/ 
 
//#define DEBUG     // if DEBUG is defined, some code is added to display some basic debug info
#define DEBUG_STREAM SerialUSB

// lorawan
#include <rn2xx3.h>
#include "keys.h"
#define loraSerial Serial2
#define PAYLOADSIZE 12 // The size of the package to be sent
//create an instance of the rn2xx3 library,
//giving the software serial as port to use
rn2xx3 myLora(loraSerial);
#define LEDPIN LED_BUILTIN
#define LORAWAN_TX_INTERVAL 240  // seconds between LoraWan messages
uint8_t  myLoraWanData[40];  // including byte[0]
unsigned long last_lora_time = millis(); // last time lorawan ran

// GPS
#include <Arduino.h>
#include <Sodaq_nbIOT.h>
#include "Sodaq_UBlox_GPS.h"
Sodaq_nbIOT nbiot;



//////////////////////////////////////////////////
// Kaasfabriek routines for RN2483 for LoraWan
///////////////////////////////////////////////
void rn2483_init();
void doOneLoraWan();
///////////////////////////////////////////////
//  arduino init and main
///////////////////////////////////////////
void setup();
void loop();
//////////////////////////////////////////////////////////
//// Kaasfabriek routines for gps
////////////////////////////////////////////

void put_gpsvalues_into_sendbuffer();
void gps_init();
void find_fix(uint32_t delay_until);
void doGPS_and_put_values_into_lora_sendbuffer();
void put_gpsvalues_into_lora_sendbuffer();

//////////////////////////////////////////////////
// Kaasfabriek routines for RN2483 for LoraWan
///////////////////////////////////////////////
void rn2483_init()
{
  loraSerial.begin(57600); //serial port to radio
  
  //reset rn2483
  pinMode(12, OUTPUT);
  digitalWrite(12, LOW);
  delay(500);
  digitalWrite(12, HIGH);

  delay(100); //wait for the RN2xx3's startup message
  loraSerial.flush();

  //Autobaud the rn2483 module to 9600. The default would otherwise be 57600.
  myLora.autobaud();

  //check communication with radio
  String hweui = myLora.hweui();
  while(hweui.length() != 16)
  {
    DEBUG_STREAM.println("Communication with RN2xx3 unsuccessful. Power cycle the board.");
    DEBUG_STREAM.println(hweui);
    delay(10000);
    hweui = myLora.hweui();
  }

  //print out the HWEUI so that we can register it via ttnctl
  DEBUG_STREAM.println("When using OTAA, register this DevEUI: ");
  DEBUG_STREAM.println(myLora.hweui());
  DEBUG_STREAM.println("RN2xx3 firmware version:");
  DEBUG_STREAM.println(myLora.sysver());

  //configure your keys and join the network
  DEBUG_STREAM.println("Trying to join TTN");
  bool join_result = false;

  //ABP: initABP(String addr, String AppSKey, String NwkSKey);
  join_result = myLora.initABP(DEVADDR, APPSKEY, NWKSKEY);

  //OTAA: initOTAA(String AppEUI, String AppKey);
  //join_result = myLora.initOTAA("70B3D57ED00001A6", "A23C96EE13804963F8C2BD6285448198");

  while(!join_result)
  {
    DEBUG_STREAM.println("Unable to join. Are your keys correct, and do you have TTN coverage?");
    RED();
    delay(60000); //delay a minute before retry
    join_result = myLora.init();
  }
  DEBUG_STREAM.println("Successfully joined TTN");
  GREEN();

  // SF is not in the library yet - maybe add it
  
}

void print_myLoraWanData() {
  DEBUG_STREAM.print(F("  myLoraWanData = [")); 
  //DEBUG_STREAM.print((char*)myLoraWanData); DEBUG_STREAM.println("]"); DEBUG_STREAM.print(F("                  [ "));  
  for(int i=0; i<30; i++) {  
    if (myLoraWanData[i] < 16) DEBUG_STREAM.print("0"); 
    DEBUG_STREAM.print(myLoraWanData[i], HEX); 
    DEBUG_STREAM.print(F(" "));  
  }  
  DEBUG_STREAM.println(F(" .. ]"));
}

void doOneLoraWan() {
  DEBUG_STREAM.print("\nStart: Do one lora. milis="); DEBUG_STREAM.println(millis());
  print_myLoraWanData();
    led_on();

    
    DEBUG_STREAM.print("  txLora. milis="); DEBUG_STREAM.println(millis());
    myLora.txBytes(myLoraWanData, PAYLOADSIZE);
    DEBUG_STREAM.print("  txLora completed. milis="); DEBUG_STREAM.println(millis());
    led_off();
   DEBUG_STREAM.print("  send time delay completed. milis="); DEBUG_STREAM.println(millis());
  DEBUG_STREAM.print("Completed: Do one lora. milis="); DEBUG_STREAM.println(millis());
}


///////////////////////////////////////////////
//  arduino init and main
///////////////////////////////////////////

unsigned long device_startTime;
bool has_sent_allready = false;
int16_t packetnum = 0;  // packet counter, we increment per transmission
bool ReceivedFromRadio = false;
// radio buf
uint8_t buf[40];

void doOneRadio() {}; // not yet implemented
void setupRadio() {}; // not yet implemented

void setup() {
  pinMode(LEDPIN, OUTPUT);
  delay(1000);  // https://www.thethingsnetwork.org/forum/t/got-adafruit-feather-32u4-lora-radio-to-work-and-here-is-how/6863
  
  DEBUG_STREAM.begin(115200);   // whether 9600 or 115200; the gps feed shows repeated char and cannot be interpreted, setting high value to release system time
  delay(100);

  DEBUG_STREAM.print(F("\nStarting device: ")); DEBUG_STREAM.println(DEVADDR); 
  device_startTime = millis();

  gps_init(); 
  rn2483_init();

  DEBUG_STREAM.print(F("\nInit values. milis=")); DEBUG_STREAM.println(millis());
  /*TODO
   * put_Volts_and_Temp_into_sendbuffer();
  put_Compass_and_Btn_into_sendbuffer();
  */
  doGPS_and_put_values_into_lora_sendbuffer();   

  DEBUG_STREAM.print(F("\nSend one lorawan message as part of system init. milis=")); DEBUG_STREAM.println(millis());

  last_lora_time = millis();
  doOneLoraWan();
  DEBUG_STREAM.print(F("\nCompleted: Setup. milis=")); DEBUG_STREAM.println(millis());
  
}
boolean radioActive = false;  // this name is for radio, not LoraWan
boolean loraWannaBe = false;

void loop() {
  DEBUG_STREAM.print(F("\n==== Loop starts. milis=")); DEBUG_STREAM.println(millis());
  ////// GPS pre-loop //////////////
  // DEBUG_STREAM.println(F("\nNo lengthy GPS read-till-fix is needed, the GPS will find/keep a fix as log as power is on. "));

  ////////// Radio  ///////////
  DEBUG_STREAM.print(F("\nRadio listen? milis=")); DEBUG_STREAM.println(millis());
  // now listen a long time for a radio message which we may want to act on, or for a keypress on our side 
  // time needs to be long enough not to miss a radio, we do not worry about GPS as it will keep fix as long as powered
  if(radioActive) {
    //TODO:
//    setupRadio();
//while((millis() - last_lora_time) < (LORAWAN_TX_INTERVAL * 1000L)) {
    // next command is not what we want to do
    //doOneRadio();  // sends a radio message and will listen for return message for a certain time
    
    if(ReceivedFromRadio) {
      // use the radio message content for Lora
      memcpy(myLoraWanData,buf,PAYLOADSIZE);
      ReceivedFromRadio = false;
    } else {
      //sprintf(myLoraWanData,"xx geen radio ontvangen xx");
    }  
  } else {
    //not listening to radio at all, we may as well use delay for a bit 
    DEBUG_STREAM.print(F("  No radio listen required, so instead just add a delay before lorawan: \n    ")); DEBUG_STREAM.print(LORAWAN_TX_INTERVAL); DEBUG_STREAM.print(F(" sec."));
    while((millis() - last_lora_time) < (LORAWAN_TX_INTERVAL * 1000L)) {
      delay(5000);   
      DEBUG_STREAM.print(F("."));
    }
    DEBUG_STREAM.println();
  }
  // we keep doing this part until it is time to send one LORAWAN TX to the worl

  ////////// Collect data needed just before sending a LORAWAN update to the world  ///////////
  DEBUG_STREAM.println(F("\nCollect data needed just before sending a LORAWAN update. milis=")); DEBUG_STREAM.println(millis());
  
  /*
   * TODO
   * put_Volts_and_Temp_into_sendbuffer();
  put_Compass_and_Btn_into_sendbuffer();
  */
  doGPS_and_put_values_into_lora_sendbuffer();
  
  
  ////////// Now we need to send a LORAWAN update to the world  ///////////
  // switch the LMIC antenna to LoraWan mode
  DEBUG_STREAM.println(F("Time or button press tells us to send one LoraWan. milis=")); DEBUG_STREAM.println(millis());
  last_lora_time = millis();
  rn2483_init();
  doOneLoraWan();    
  
  /////////// Loop again  //////////////
  DEBUG_STREAM.println(F("\nEnd of loop. milis=")); DEBUG_STREAM.println(millis());
}

//////////////////////////////////////////////////////////
//// Kaasfabriek routines for gps
////////////////////////////////////////////
long l_lat, l_lon, l_alt;
int hdopNumber;

void gps_init() {
  sodaq_gps.init(6);
  #ifdef DEBUG
  sodaq_gps.setDiag(DEBUG_STREAM);
  #endif
  // First time finding a fix wait 60 seconds at most
  find_fix(60);
}


void find_fix(uint32_t delay_until)
{
    uint32_t start = millis();
    uint32_t timeout = delay_until * 1000; // timeout
    DEBUG_STREAM.println(String("waiting for fix ..., timeout=") + timeout + String("ms"));
    if (sodaq_gps.scan(false, timeout)) {
      
      l_lat = sodaq_gps.getLat();
      l_lon = sodaq_gps.getLon();
      l_alt = sodaq_gps.getAlt();
      hdopNumber = sodaq_gps.getHDOP();
      
    } else {
        DEBUG_STREAM.println("No Fix");        
    }
}

void put_gpsvalues_into_lora_sendbuffer() {
  
  Serial.println(F("Started: put_gpsvalues_into_sendbuffer"));
  
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

void doGPS_and_put_values_into_lora_sendbuffer() {
  find_fix(2); // find fix in 2 seconds
  // put gps values into send buffer
  put_gpsvalues_into_lora_sendbuffer();
  
  Serial.print(F("\Completed: doGPS_and_put_values_into_sendbuffer. milis=")); Serial.println(millis());
  
}



// Leds
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
