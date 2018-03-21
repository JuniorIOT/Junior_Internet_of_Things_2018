
/*******************************************************************************
 * Modified By DenniZr & Marco van Schagen for Junior IOT - Smart City Challenge 2018 #juniorIOTchallenge2018
 * Modified By Marco van Schagen for Junior IOT Challenge 2018
 * Modified By Roel Drost Calculates the disatance and bearing given two GPS location near each other.
 *******************************************************************************/ 


// build options: 
//  Sodaq explorer board --> compile for Sodaq Explorer

// Libraries to UNINSTALL standard libraries: 
//      UNINSTALL  your existing RN2xx3 library by JPmeijers ==> delete folder Documents\Arduino\libraries\RN2483-Arduino-Library-master
// Libraries to add standard libraries: 
//      <Sodaq_nbIOT.h>
//      <Sodaq_wdt.h>
// Libraries to add from our libraries subfolder: 
//      <rn2xx3.h>          --> C:\arduino_port\GitHub\Junior_Internet_of_Things_2018\sodaq_Explorer_nb-iot\libraries\RN2483-Arduino-Library-master_kaasfabriek.zip
//      <sodaq_compass.h>   --> C:\arduino_port\GitHub\Junior_Internet_of_Things_2018\sodaq_Explorer_nb-iot\libraries\sodaq-nbiot-compass.zip


//   Make a file keys.h in the project folder and insert your variables which you copy from your device definition in TTN console
//     static const PROGMEM String NWKSKEY = "C6F8438D6E06B46.....1FCA829654B";
//     static const PROGMEM String APPSKEY = "F16C600925A36A1D1.....103A2D8D308";
//     static const String DEVADDR = "26....BD";
//      --> use strings, not arrays OR YOU WILL GET ERROR IN FUNCTION void rn2483_init()

// TODO wire your buttons, see schematic on our github
// define wirings
#define pin_button 8  
#define gnd_button 9         // pin of the push button
#define pin_buzzer 10
#define gnd_buzzer 11
#define led_directionfound 12
#define gnd_directionfound 13
#define pin_PM_TXD_rx 2
#define pin_PM_RXD_tx 3

 
#define DEBUG     // if DEBUG is defined, some code is added to display some basic debug info
#define DEBUG_STREAM SerialUSB

// lorawan
#include <rn2xx3.h>
#include "keys.h"
#define loraSerial Serial2
#define PAYLOADSIZE 34 // The size of the package to be sent
//create an instance of the rn2xx3 library,
//giving the software serial as port to use
rn2xx3 myLora(loraSerial);
#define LEDPIN LED_BUILTIN
#define LORAWAN_TX_INTERVAL_MIN 15  // seconds between LoraWan messages
#define LORAWAN_TX_INTERVAL_MAX 240  
#define TXTRIGGER_gps_movement 5000 // will send if GPS movement compared to avg( last 3 points) > value
    // gps coordinates in Long value means 52.6324510 translates into 526324510
    // 5000 trigger value means 25-30 meters
uint8_t  myLoraWanData[40];  // including byte[0]
unsigned long last_lora_time = millis(); // last time lorawan ran
unsigned long last_check_time = millis();  // last time we did movement detection 
uint16_t packagecounter;

// GPS
#include <Arduino.h>
#include <Sodaq_nbIOT.h>  // Standard library to be installed into Arduino IDE
#include "Sodaq_UBlox_GPS.h"  // this is just a good reference to a code file in your folder
Sodaq_nbIOT nbiot;            // declares an object to handle specific stuff for you
long l_lat, l_lon, l_alt, l_lat_hist, l_lon_hist, l_lat_movement, l_lon_movement;
int hdopNumber;

// compass - library in nb-iot compass folder
#include <sodaq_compass.h>  // library to be added from Zip in this project
NBIOT_Compass compass;      // declares an object to handle specific stuff for you

// radio
int16_t packetnum = 0;  // packet counter, we increment per transmission
bool ReceivedFromRadio = false;
// radio buf + 4 for radiohead compatibility
#define radioPacketSize (10+4)
uint8_t buf[radioPacketSize];
uint8_t *decoded;

// game
int negotiateState = 0;
float hitlat1, hitlng1, hitlat2, hitlng2, hitcompass;
boolean radioActive = true;  // this name is for radio, not LoraWan
boolean loraNeedsSendNow = false;  // an explanation could be given
#include "starwars.h"
uint8_t MyTeamID = 1;
uint8_t MyID = 2;
uint8_t buttonPressed;

// GPS heading formula
#include "roeldrost.h"

// sensors
#include "SoftwareSerial.h"  // samd version; constructor: SoftwareSerial
#include "temperature.h"
#include "wh-z19b.h"
#include "sds021.h"

// led on is used to signal to the user that a loop is completing
void led_on();
void led_off();

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
void put_gpsvalues_into_lora_sendbuffer(bool savePrevious);
void doGPS(unsigned long timeout);
//////////////////////////////////////////////////////////
//// Compass LSM303_U i2c
////////////////////////////////////////////
void setupCompass();
unsigned int readCompass();
void put_Compass_and_Btn_into_sendbuffer();
//////////////////////////////////////////////////////////
//// Game
////////////////////////////////////////////
uint8_t whoTalkedToMe();
uint8_t wasIHit();
void IHitSomeone();
//////////////////////////////////////////////////////////
//// Sensors lora
////////////////////////////////////////////
void put_Volts_and_Temp_into_sendbuffer();
void loraDatasetByte();
bool setupTemperature();
int readTemperatureFromShield();
double readHumidityFromShield();
//////////////////////////////////////////////////
// Kaasfabriek routines for RN2483 radio 2 radio
///////////////////////////////////////////////
void doOneRadio();
void setupRadio();
void listenRadio();
void formatRadioPackage(uint8_t *loopbackToData);
void decodeReply();


#include <math.h>
// GPS intersection
int bearing (double lat1, double lng1, double lat2, double lng2);
float _toDeg (float rad);
float _toRad (float deg);

//////////////////////////////////////////////////
// Kaasfabriek routines for RN2483 for LoraWan
///////////////////////////////////////////////
#include "lorawan.h"


unsigned long device_startTime;
bool has_sent_allready = false;

//////////////////////////////////////////////////
// Kaasfabriek routines for RN2483 radio 2 radio
///////////////////////////////////////////////
#include "loraradio.h"

uint8_t whoTalkedToMe() {  
  return who;
}

// compare compass and gps with the data from the other who just shot
uint8_t wasIHit() {
  SerialUSB.println("Checking if i was hit");
  uint8_t hit = 0b00000001; // yes i was hit
  uint8_t nothit = 0b00000000; 
  int inaccuracy = 30; //degrees
  SerialUSB.print("His compass was: ");
  SerialUSB.println(hitcompass);
  float heading = bearing(hitlat2, hitlng2,hitlat1 / 10000000, hitlng1 / 10000000);
  SerialUSB.print("The heading between us based on both our coordinates is: ");
  SerialUSB.println(heading);
  SerialUSB.print("Inaccuracy allowed: ");
  SerialUSB.println(inaccuracy);
  if( ((int)abs((hitcompass - heading)) % 360) <  inaccuracy) {
    SerialUSB.println("So i was HIT!");
    myLoraWanData[33] = 0b10000000;
    return hit;    
  }else {
    SerialUSB.println("So i was not hit");
    myLoraWanData[33] = 0b00000000;
    return nothit;    
  }
}

void IHitSomeone() {
  // yes!
  soundLoopOnce();
}

void loraDatasetByte() {
  // todo
  if(therewasaradioreceived) {
    myLoraWanData[22] = 0b00001000;
    therewasaradioreceived = false;
  }
  myLoraWanData[22] |= (MyTeamID & 0b00001111) << 4;
  myLoraWanData[23] = (packagecounter >> 8) & 0xFF;
  myLoraWanData[24] = packagecounter & 0xFF;
}

///////////////////////////////////////////////
//  arduino init and main
///////////////////////////////////////////

void setup() {
  delay(1000);  // https://www.thethingsnetwork.org/forum/t/got-adafruit-feather-32u4-lora-radio-to-work-and-here-is-how/6863
  
  // enable debug stream
  DEBUG_STREAM.begin(115200);   // whether 9600 or 115200; the gps feed shows repeated char and cannot be interpreted, setting high value to release system time
  delay(100);
  DEBUG_STREAM.print(F("Starting device: ")); DEBUG_STREAM.println(DEVADDR); 
  device_startTime = millis();

  // first activate GPS and leave on so it can find a fix quicker
  SerialUSB.println("setup: GPS_init");
  gps_init(); 
  
  // setup and test leds
  DEBUG_STREAM.print(F("setup: Wake up and test leds. milis=")); DEBUG_STREAM.println(millis());
  pinMode(LEDPIN, OUTPUT);
  
  pinMode(led_directionfound, OUTPUT);
  pinMode(gnd_directionfound, OUTPUT);
  digitalWrite(led_directionfound, HIGH);
  digitalWrite(gnd_directionfound, LOW);
  
  led_on(); do_flash_led(led_directionfound); delay(2000); led_off(); do_flash_led(led_directionfound); delay(2000);
  led_on(); do_flash_led(led_directionfound); delay(1000); led_off(); do_flash_led(led_directionfound); delay(1000);
  led_on(); do_flash_led(led_directionfound); delay(500); led_off(); do_flash_led(led_directionfound); delay(500); 
  led_on(); do_flash_led(led_directionfound); delay(500); led_off(); do_flash_led(led_directionfound); delay(500); 
  led_on(); do_flash_led(led_directionfound); delay(200); led_off(); do_flash_led(led_directionfound); delay(200); 
  led_on(); do_flash_led(led_directionfound); delay(200); led_off(); do_flash_led(led_directionfound); delay(200);
  DEBUG_STREAM.print(F("setup: Leds tested. milis=")); DEBUG_STREAM.println(millis());
  
  // setup and test sound
  pinMode(gnd_buzzer, OUTPUT);
  digitalWrite(gnd_buzzer, LOW);
  soundLoopOnce();
  DEBUG_STREAM.print(F("setup: Played sound. milis=")); DEBUG_STREAM.println(millis());

  // enable pushbutton
  pinMode(pin_button, INPUT_PULLUP);
  pinMode(gnd_button, OUTPUT);
  digitalWrite(gnd_button, LOW);

  // lora package
  packagecounter = 0;
  
  //sensors
  setup_pm();
  pm_getFirmwareVersion();
  pm_measure();
//  pm_goToSleep();
//  delay(1000);
//  pm_wakeUp();
//  while(1){pm_measure();delay(5);};  // this line for testing PM only
  
  // game parameters
  negotiateState = 0;
  didSomeoneElseFire = false;
  didIFire = false;

  SerialUSB.println("setup: rn2483-init");
  rn2483_init();
  SerialUSB.println("setup: setupCompass");
  setupCompass();
  if(!setupTemperature()) {
    DEBUG_STREAM.println("setup: Could not get temperature sensor");
  } else  {
    DEBUG_STREAM.println("setup: temperature sensor found");
  }
  
  DEBUG_STREAM.print(F("setup: get first measurements. milis=")); DEBUG_STREAM.println(millis());
  put_Volts_and_Temp_into_sendbuffer();
  put_Compass_and_Btn_into_sendbuffer();
  DEBUG_STREAM.print(F("setup: doGPS_and_put_values_into_lora_sendbuffer. milis=")); DEBUG_STREAM.println(millis());
  doGPS_and_put_values_into_lora_sendbuffer();   

  DEBUG_STREAM.print(F("setup: Send one lorawan message as part of system init. milis=")); DEBUG_STREAM.println(millis());
  last_lora_time = millis();
  last_check_time = millis();
  doOneLoraWan();
  digitalWrite(led_directionfound, LOW);
  DEBUG_STREAM.print(F("\nsetup: Completed. milis=")); DEBUG_STREAM.println(millis());
}

void loop() {
  DEBUG_STREAM.print(F("\n==== Loop starts. milis=")); DEBUG_STREAM.println(millis());
  ////// GPS pre-loop //////////////
  // DEBUG_STREAM.println(F("\nNo lengthy GPS read-till-fix is needed, the GPS will find/keep a fix as log as power is on. "));

  ////////// Radio  ///////////
  DEBUG_STREAM.print(F("\nloop: Radio listen? milis=")); DEBUG_STREAM.println(millis());
  // now listen a long time for a radio message which we may want to act on, or for a keypress on our side 
  // time needs to be long enough not to miss a radio, we do not worry about GPS as it will keep fix as long as powered
  if(radioActive) {
    bool loraNeedsSendNow = false;
    while((millis() - last_check_time) < (LORAWAN_TX_INTERVAL_MIN * 1000L) && !loraNeedsSendNow) {
      
      DEBUG_STREAM.println(F("loop: 1. setupRadio. milis=")); DEBUG_STREAM.println(millis());
      // if negotiateState == 1 then check if the shot was a hit
      // listen if someone else fired
      setupRadio();
      DEBUG_STREAM.print(F("loop: 2. listenRadio. milis=")); DEBUG_STREAM.println(millis());
      listenRadio();                // does this have a hardcoded time in library?
      DEBUG_STREAM.print(F("loop: 3. readCompass. milis=")); DEBUG_STREAM.println(millis());
      readCompass();      
      DEBUG_STREAM.print(F("loop: 4. check button press. milis=")); DEBUG_STREAM.println(millis());

      if(digitalRead(pin_button) == LOW) { // input pullup with ground
        didIFire = true;
        negotiateState = 1;
        buttonpressedForLoraWan = true;        
        buttonPressed = 0b10000000;
      } else {
        buttonPressed = 0b00000000;
      }

      if(didIFire && negotiateState == 1) {
        // tell other person I fired
        tone(pin_buzzer, 261, 250);
        setupRadio();
        doOneRadio();      
        
        // if you fired you wait 3 times for someone to say something back
        for(int i = 0; i < 3; i++) {
          DEBUG_STREAM.println("loop: Waiting for the other person to say he was hit");
          tone(pin_buzzer, 349, 250);
          setupRadio();
          listenRadio();          
        }
        
        loraNeedsSendNow = true;
        // reset fire but keep negotiateState to know someone might reply
        didIFire = false;
      }            
      
      DEBUG_STREAM.print(F("loop: 5. check if fired or hit. milis=")); DEBUG_STREAM.println(millis());
      
      if(didSomeoneElseFire && shouldITalkBack) {
        // tell other person i was hit
        DEBUG_STREAM.println("loop: Telling other person I was hit");
        setupRadio();
        doOneRadio();
        didSomeoneElseFire = false;
        shouldITalkBack = false;
      }      
      DEBUG_STREAM.print(F("loop: 6."));
    }
    
    DEBUG_STREAM.println();
  }
  // we keep doing this part until it is time to send one LORAWAN TX to the worl

  ////////// Collect data needed just before sending a LORAWAN update to the world  ///////////
  DEBUG_STREAM.print(F("loop: Collect data needed just before sending a LORAWAN update. milis=")); DEBUG_STREAM.println(millis());
  
  DEBUG_STREAM.print(F("loop: get first measurements. milis=")); DEBUG_STREAM.println(millis());
  put_Volts_and_Temp_into_sendbuffer();
  put_Compass_and_Btn_into_sendbuffer();
  DEBUG_STREAM.print(F("loop: doGPS_and_put_values_into_lora_sendbuffer. milis=")); DEBUG_STREAM.println(millis());
  doGPS_and_put_values_into_lora_sendbuffer();   
  
  DEBUG_STREAM.print(F("loop: pm_measure. milis=")); DEBUG_STREAM.println(millis());
  pm_measure(); //hier moeten we nog wat met de resultaten doen

  loraDatasetByte(); // what am i doing with the extra bytes - sending radio to lora or sending extra sensors
  
  ////////// Now CHECK IF we need to send a LORAWAN update to the world  ///////////
  
  DEBUG_STREAM.print(F("loop: Collect data needed just before sending a LORAWAN update. milis=")); DEBUG_STREAM.println(millis());
  if (loraNeedsSendNow
        or (millis() - last_lora_time) > (LORAWAN_TX_INTERVAL_MAX * 1000L)
        or ((abs(l_lat_movement) + abs(l_lon_movement)) > TXTRIGGER_gps_movement) ) {
    // switch the LMIC antenna to LoraWan mode
    DEBUG_STREAM.print(F("loop: Time or button press or movement detection tells us to send one LoraWan. milis=")); DEBUG_STREAM.println(millis());
    DEBUG_STREAM.print(F("loop: trigger distance=")); DEBUG_STREAM.println(TXTRIGGER_gps_movement);
    DEBUG_STREAM.print(F("loop: measured distance=")); DEBUG_STREAM.println((abs(l_lat_movement) + abs(l_lon_movement)));
    last_lora_time = millis();
    rn2483_init();
    doOneLoraWan();    
  } else {
    DEBUG_STREAM.println(F("loop: Lora does not need sending at this time (no movement detected, no button press). ")); 
  }
  last_check_time = millis();
  
  // save previous
  put_gpsvalues_into_lora_sendbuffer(true);

  // reset game
  negotiateState = 0; // timeout no one replied to me firing.
  buttonpressedForLoraWan = false;
  
  /////////// Loop again  //////////////
  DEBUG_STREAM.println(F("\nEnd of loop. milis=")); DEBUG_STREAM.println(millis());
}

//////////////////////////////////////////////////////////
//// Kaasfabriek routines for gps
////////////////////////////////////////////
#include "gps.h"

//////////////////////////////////////////////////////////
//// Compass LSM303_U i2c
////////////////////////////////////////////
#include "compass.h"

// Leds
void led_on()
{
  digitalWrite(LEDPIN, 1);
}

void led_off()
{
  digitalWrite(LEDPIN, 0);
}
void led_toggle()
{
  digitalWrite(LEDPIN, !digitalRead(LEDPIN));
}

void do_flash_led(int pin)
{
    for (size_t i = 0; i < 2; ++i) {
        delay(100);
        digitalWrite(pin, LOW);
        delay(100);
        digitalWrite(pin, HIGH);
    }
}


// gps intersection
/**
     * Calculate the bearing between two positions as a value from 0-360
     *
     * @param lat1 - The latitude of the first position
     * @param lng1 - The longitude of the first position
     * @param lat2 - The latitude of the second position
     * @param lng2 - The longitude of the second position
     *
     * @return int - The bearing between 0 and 360
     */

     /*
      * new formula from https://www.movable-type.co.uk/scripts/latlong.html
      * untested
      */
      // phi = lat
      
    int bearing (double lat1, double lng1, double lat2, double lng2) {
      
      /*
      
      SerialUSB.println("Bearing inputs");
      SerialUSB.print("lat1: ");
      SerialUSB.println(lat1,8);
      SerialUSB.print("lng1: ");
      SerialUSB.println(lng1,8);
      SerialUSB.print("lat2: ");
      SerialUSB.println(lat2,8);
      SerialUSB.print("lng2: ");
      SerialUSB.println(lng2,8);
      
        /*float dLon = (lng2-lng1);
        float y = sin(dLon) * cos(lat2);
        float x = (cos(lat1)*sin(lat2)) - ((sin(lat1)*cos(lat2))*cos(dLon));
        float brng = _toDeg(atan2(y, x));
        return 360 - (((int)brng + 360) % 360);
        *//*
        double y = sin(lng2-lng1) * cos(lat2);
        double x = (cos(lat1)*sin(lat2)) - (sin(lat1)*cos(lat2)*cos(lng2-lng1));
        double brng = atan2(y, x);
        brng = 180.0*brng/PI;   
        if (brng <0)
          brng += 360;
*/
/*      lat1 = _toRad(lat1);
        lat2 = _toRad(lat2);
        double y = sin(_toRad(lng2-lng1)) * cos(lat2);
        double x = (cos(lat1)*sin(lat2)) - (sin(lat1)*cos(lat2)*cos(_toRad(lng2-lng1)));
        double brng = _toDeg(atan2(y, x));
        brng = (int)(brng+360) % 360;
        return brng;
*//*
    lat1 = _toRad(lat1);
    lat2 = _toRad(lat2);
    
    double deltaLon = _toRad(lng2 - lng1);

    
    if (deltaLon >  PI) deltaLon -= 2*PI;
    if (deltaLon < -PI) deltaLon += 2*PI;

    double a = log(tan(PI/4 + lat2/2)/tan(PI/4+lat1/2));

    double b = atan2(deltaLon, a);

    double returns =  ((signed int)_toDeg(b)+360) % 360;
    */
    static const testLocation_t currentLocationFrom = {"From", {lat1, lng1}};
    static const testLocation_t currentLocationTo = {"To", {lat2, lng2}};
    
    const testLocation_t& from = currentLocationFrom;
    const testLocation_t& to   = currentLocationTo;
      
      SerialUSB.print("From \""); SerialUSB.print(from.name);
      SerialUSB.print("\" to \""); SerialUSB.print(to.name);
      SerialUSB.println("\"");

      double distance, bearing;
      calculateNearAlkmaar(from.location, to.location, distance, bearing);

      SerialUSB.print("\tDistance: ");
      SerialUSB.print(distance);
      SerialUSB.println("m");
      
      SerialUSB.print("\tBearing : ");
      SerialUSB.print(bearing);
      SerialUSB.println("°");
    return bearing;
    
}
   /**
     * Since not all browsers implement this we have our own utility that will
     * convert from degrees into radians
     *
     * @param deg - The degrees to be converted into radians
     * @return radians
     */
     
    double _toRad (double deg) {
         return deg * PI / 180.0F;
    }

    /**
     * Since not all browsers implement this we have our own utility that will
     * convert from radians into degrees
     *
     * @param rad - The radians to be converted into degrees
     * @return degrees
     */
    double _toDeg (double rad) {
        return rad * (180.0 / PI);
    }

  
