/*******************************************************************************
 * Modified By DenniZr & Marco van Schagen for Junior IOT - Smart City Challenge 2018 #juniorIOTchallenge2018
 * Modified By Marco van Schagen for Junior IOT Challenge 2018
 *******************************************************************************/ 
 
#define DEBUG     // if DEBUG is defined, some code is added to display some basic debug info
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
long l_lat, l_lon, l_alt;
int hdopNumber;

// compass - library in nb-iot compass folder
#include <sodaq_compass.h>
NBIOT_Compass compass;


// radio
int16_t packetnum = 0;  // packet counter, we increment per transmission
bool ReceivedFromRadio = false;
// radio buf
#define radioPacketSize 10
uint8_t buf[radioPacketSize];


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
void doGPS(unsigned long timeout);
//////////////////////////////////////////////////////////
//// Compass LSM303_U i2c
////////////////////////////////////////////
void setupCompass();
long readCompass();
void put_Compass_and_Btn_into_sendbuffer();


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



void doOneRadio() {
  String received = "";
  DEBUG_STREAM.print("\nStart: Do one radio. milis="); DEBUG_STREAM.println(millis());  
  
  
  uint8_t radiopacket[radioPacketSize];
  
  formatRadioPackage(&radiopacket[0]);
  
  DEBUG_STREAM.println("Sending..."); delay(10);
  

  switch(myLora.txBytes(radiopacket, radioPacketSize)) //one byte, blocking function
  {
    case TX_FAIL:
    {
      DEBUG_STREAM.println("TX unsuccessful or not acknowledged");
      break;
    }
    case TX_SUCCESS:
    {
      DEBUG_STREAM.println("TX successful and acknowledged");
      break;
    }
    case TX_WITH_RX:
    {
      received = myLora.getRx();
      received = myLora.base16decode(received);
      ReceivedFromRadio = true;
      received.getBytes(buf, received.length());
      DEBUG_STREAM.print("Received downlink immediately: " + received);
      break;
    }
    default:
    {
      DEBUG_STREAM.println("Unknown response from TX function");
    }
  }  

  
  // end loop
  
  DEBUG_STREAM.print("\nCompleted: Do one radio. milis="); DEBUG_STREAM.println(millis());
  
  
}; 
void setupRadio() {
  myLora.autobaud();

  DEBUG_STREAM.println("DevEUI? ");DEBUG_STREAM.print(F("> "));
  DEBUG_STREAM.println(myLora.hweui());
  DEBUG_STREAM.println("Version?");DEBUG_STREAM.print(F("> "));
  DEBUG_STREAM.println(myLora.sysver());
  DEBUG_STREAM.println(F("--------------------------------"));

  DEBUG_STREAM.println(F("Setting up for listening for another explorer"));
  bool join_result = false;


  // point to point
  if(join_result = myLora.initP2P())  
  DEBUG_STREAM.println("\u2713 Successfully Activated radio 2 radio");
};

void listenRadio() {
  DEBUG_STREAM.print("\nStart: Listen radio. milis="); DEBUG_STREAM.println(millis());  
  switch(myLora.listenP2P()) {
    case TX_WITH_RX:
    {
      String received = myLora.getRx();
      received = myLora.base16decode(received);
      ReceivedFromRadio = true;
      received.getBytes(buf, received.length());
      DEBUG_STREAM.print("Received downlink: " + received);
      break;
    }
    case RADIO_LISTEN_WITHOUT_RX:
    { 
      DEBUG_STREAM.println("Listened timeout but no downlink");
      break;
    }
    
 
  }  
  DEBUG_STREAM.print("\nCompleted: Listen radio. milis="); DEBUG_STREAM.println(millis());

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
  DEBUG_STREAM.print(F("  compass=")); DEBUG_STREAM.print(compass); DEBUG_STREAM.print(F("  deg. compass_bin=")); Serial.println(compass_bin);
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
      DEBUG_STREAM.println("Radio: Someone says that he fired");
    } else if((buf[0] & 0b00001111) == 0b00000010) {
        DEBUG_STREAM.println("Radio: Someone talkes back to someone who fired");
    }
    DEBUG_STREAM.print("That someone has an id of:");
    uint8_t id = (buf[0] >> 4) & 0b00001111;
    DEBUG_STREAM.println((int) id,DEC);
    
    // byte 1,2,3 and 4,5,6
    DEBUG_STREAM.print("His location is: ");
    
    float _lat = ((((uint32_t)buf[1]) << 16) + (((uint32_t)buf[2]) << 8) + buf[3]) / 16777215.0 * 180.0 - 90;
    float _lng = ((((uint32_t)buf[4]) << 16) + (((uint32_t)buf[5]) << 8) + buf[6]) / 16777215.0 * 360.0 - 180;
    DEBUG_STREAM.print("lat: ");
    DEBUG_STREAM.print(_lat);
    DEBUG_STREAM.print("lng: ");
    DEBUG_STREAM.println(_lng);

    // byte 7
    uint8_t compass = buf[7] & 0b01111111; // don't want the hit indicator now
    DEBUG_STREAM.print("His compass points to: ");
    int _compass = (compass & 127)*3;
    DEBUG_STREAM.println(_compass);

    bool hePressedHisButton = ((buf[7] >> 7) & 0b00000001) == 0b00000001;
    if(hePressedHisButton) DEBUG_STREAM.println("He pressed his button");
    else DEBUG_STREAM.println("He did not press his button");

    // byte 8
    bool heWasHit = (buf[8] & 0b00000001) == 0b00000001;
    if(heWasHit) DEBUG_STREAM.println("He was hit");
    else DEBUG_STREAM.println("He was not hit - or doesn't know it yet");

    uint8_t remoteid = (buf[8] >> 4) & 0b00001111;
    DEBUG_STREAM.println("He was talking to id: ");
    DEBUG_STREAM.print((int)remoteid,DEC);

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


void setup() {
  pinMode(LEDPIN, OUTPUT);
  delay(1000);  // https://www.thethingsnetwork.org/forum/t/got-adafruit-feather-32u4-lora-radio-to-work-and-here-is-how/6863
  
  DEBUG_STREAM.begin(115200);   // whether 9600 or 115200; the gps feed shows repeated char and cannot be interpreted, setting high value to release system time
  delay(100);

  DEBUG_STREAM.print(F("\nStarting device: ")); DEBUG_STREAM.println(DEVADDR); 
  device_startTime = millis();

  gps_init(); 
  rn2483_init();
  setupCompass();

  DEBUG_STREAM.print(F("\nInit values. milis=")); DEBUG_STREAM.println(millis());
  /*TODO
   * put_Volts_and_Temp_into_sendbuffer();
  */
  
  put_Compass_and_Btn_into_sendbuffer();
  
  doGPS_and_put_values_into_lora_sendbuffer();   

  DEBUG_STREAM.print(F("\nSend one lorawan message as part of system init. milis=")); DEBUG_STREAM.println(millis());

  last_lora_time = millis();
  doOneLoraWan();
  DEBUG_STREAM.print(F("\nCompleted: Setup. milis=")); DEBUG_STREAM.println(millis());
  
}
boolean radioActive = true;  // this name is for radio, not LoraWan
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
    setupRadio();
    while((millis() - last_lora_time) < (LORAWAN_TX_INTERVAL * 1000L)) {
    // next command is not what we want to do
    doOneRadio();  // sends a radio message and will listen for return message for a certain time
    delay(5000);
    }
    
  } else {
    //not listening to radio at all, we may as well use delay for a bit 
    DEBUG_STREAM.print(F("  No radio listen required, so instead just add a delay before lorawan: \n    ")); DEBUG_STREAM.print(LORAWAN_TX_INTERVAL); DEBUG_STREAM.print(F(" sec."));
    while((millis() - last_lora_time) < (LORAWAN_TX_INTERVAL * 1000L)) {
      /*delay(5000);   
      DEBUG_STREAM.print(F("."));*/
      // better listen to radio if nothing to do
      setupRadio();
      listenRadio();

      if(digitalRead(buttonpin) == HIGH) {
        
      }
    }
    DEBUG_STREAM.println();
  }
  // we keep doing this part until it is time to send one LORAWAN TX to the worl

  ////////// Collect data needed just before sending a LORAWAN update to the world  ///////////
  DEBUG_STREAM.println(F("\nCollect data needed just before sending a LORAWAN update. milis=")); DEBUG_STREAM.println(millis());
  
  /*
   * TODO
   * put_Volts_and_Temp_into_sendbuffer();
  */
  
  doGPS_and_put_values_into_lora_sendbuffer();
  put_Compass_and_Btn_into_sendbuffer();
  
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

void gps_init() {
  sodaq_gps.init(6);
  #ifdef DEBUG
  sodaq_gps.setDiag(DEBUG_STREAM);
  #endif
  // First time finding a fix wait 60 seconds at most
  find_fix(60);
}

void doGPS(uint32_t delay_until) {
  find_fix(delay_until);
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
  uint8_t HdopBinary = hdopNumber/100;                           // we want horizontal dillution, good is 2..5, poor is >20. Note:
                                                                 //      NMEAGPS outputs an indoor value of 600..1000. Let's divide by 100
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

}

void doGPS_and_put_values_into_lora_sendbuffer() {
  find_fix(2); // find fix in 2 seconds
  // put gps values into send buffer
  put_gpsvalues_into_lora_sendbuffer();
  
  DEBUG_STREAM.print(F("\Completed: doGPS_and_put_values_into_sendbuffer. milis=")); DEBUG_STREAM.println(millis());
  
}

//////////////////////////////////////////////////////////
//// Compass LSM303_U i2c
////////////////////////////////////////////
void setupCompass() {
  compass.setup();  
}

float X_milliGauss,Y_milliGauss,Z_milliGauss;
float heading, headingDegrees, headingFiltered, geo_magnetic_declination_deg;

long readCompass() {
  compass.getNewValues();
  X_milliGauss = compass.getXGauss() * 1000;
  Y_milliGauss = compass.getYGauss() * 1000;
  Z_milliGauss = compass.getZGauss() * 1000;
  // TODO Marco?? this is not millisguass anymore
  
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
  
  DEBUG_STREAM.print(headingDegrees);
  DEBUG_STREAM.print(" filtered ");
  DEBUG_STREAM.println(headingFiltered);
  
  return headingFiltered;
}


void put_Compass_and_Btn_into_sendbuffer() {
  long compass = readCompass(); // 0..360 deg
  uint8_t compass_bin = compass/3 ;  // rescale 0-360 deg into 0 - 120 values and make sure it is not bigger than one byte
  // now add a bit for BTN (not implemented)
  myLoraWanData[9] = compass_bin;
  #ifdef DEBUG
  DEBUG_STREAM.print(F("  compass=")); DEBUG_STREAM.print(compass); DEBUG_STREAM.print(F("  deg. compass_bin=")); DEBUG_STREAM.println(compass_bin);
  #endif
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
