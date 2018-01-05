/*******************************************************************************
 * Modified By DenniZr & Marco van Schagen for Junior IOT - Smart City Challenge 2018 #juniorIOTchallenge2018
 * Modified By Marco van Schagen for Junior IOT Challenge 2018
 * Modified By Roel Drost Calculates the disatance and bearing given two GPS location near each other.
 *******************************************************************************/ 

// TODO prepare your Arduino IDE:
//   install some standard libraries, get the generic version:
//      - Sodaq_nbIOT
//   install adjusted version of libraries, YOU NEED TO GET THESE FROM YOUR PROJECT FOLDERS
//      - sodaq_compass
//      - RN2483-Arduino-Library-master_kaasfabriek.zip  --> you need to uninstall your existing RN2xx3 library by JPmeijers ==> delete folder Documents\Arduino\libraries\RN2483-Arduino-Library-master
//          --> OR YOU WILL GET ERROR  case TX_FAIL: 'TX_FAIL' was not declared in this scope

//   Make a file keys.h in the project folder and insert your variables which you copy from your device definition in TTN console
//     static const PROGMEM String NWKSKEY = "C6F8438D6E06B46.....1FCA829654B";
//     static const PROGMEM String APPSKEY = "F16C600925A36A1D1.....103A2D8D308";
//     static const String DEVADDR = "26....BD";
//      --> use strings, not arrays OR YOU WILL GET ERROR IN FUNCTION void rn2483_init()

// TODO wire your buttons and tie up your shoelaces
//     D8 --> pushbutton --> ground
//     D9 --> speaker --> ground
[//    D10 --> led + resistor --> ground 

 
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
#define LORAWAN_TX_INTERVAL 240  // seconds between LoraWan messages
uint8_t  myLoraWanData[40];  // including byte[0]
unsigned long last_lora_time = millis(); // last time lorawan ran
uint16_t packagecounter;

// GPS
#include <Arduino.h>
#include <Sodaq_nbIOT.h>  // Standard library to be installed into Arduino IDE
#include "Sodaq_UBlox_GPS.h"  // this is just a good reference to a code file in your folder
Sodaq_nbIOT nbiot;            // declares an object to handle specific stuff for you
long l_lat, l_lon, l_alt;
int hdopNumber;

// compass - library in nb-iot compass folder
#include <sodaq_compass.h>  // library to be added from Zip in this project
NBIOT_Compass compass;      // declares an object to handle specific stuff for you
#define hasWalkingDirectionLED 10

// radio
int16_t packetnum = 0;  // packet counter, we increment per transmission
bool ReceivedFromRadio = false;
// radio buf + 4 for radiohead compatibility
#define radioPacketSize (10+4)
uint8_t buf[radioPacketSize];
uint8_t *decoded;

// game
int negotiateState = 0;
int buttonpin = 8; // pin of the gun button
float hitlat1, hitlng1, hitlat2, hitlng2, hitcompass;
boolean radioActive = true;  // this name is for radio, not LoraWan
boolean loraWannaBeNow = false;
int buzzerPin = 9;
#include "starwars.h"
uint8_t MyTeamID = 1;
uint8_t MyID = 2;
uint8_t buttonPressed;

// GPS heading formula
#include "roeldrost.h"

// nonsense
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
uint8_t whoWasItThatTalkedToMe();
uint8_t wasIHit();
void IHitSomeone();
//////////////////////////////////////////////////////////
//// Sensors lora
////////////////////////////////////////////
void put_Volts_and_Temp_into_sendbuffer();
void loraDatasetByte();
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

uint8_t whoWasItThatTalkedToMe() {
  
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

void put_Volts_and_Temp_into_sendbuffer() {
/* TODO: 
 *   -- now our 'regular' values
    byte 18         VCC        byte, 50ths, 0 - 5.10 volt -- secret voltmeter
    byte 19         CPUtemp    byte, -100 - 155 deg C     -- secret thermometer
    byte 20         Vbat       byte, 50ths, 0 - 5.10 volt -- hardwired Lora32u4
*/

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
  pinMode(LEDPIN, OUTPUT);
  pinMode(hasWalkingDirectionLED, OUTPUT);
  digitalWrite(hasWalkingDirectionLED, HIGH);
  pinMode(buttonpin, INPUT_PULLUP);
  delay(1000);  // https://www.thethingsnetwork.org/forum/t/got-adafruit-feather-32u4-lora-radio-to-work-and-here-is-how/6863
  
  DEBUG_STREAM.begin(115200);   // whether 9600 or 115200; the gps feed shows repeated char and cannot be interpreted, setting high value to release system time
  delay(100);

  DEBUG_STREAM.print(F("\nStarting device: ")); DEBUG_STREAM.println(DEVADDR); 
  device_startTime = millis();
  packagecounter = 0;
  
  // game parameters
  negotiateState = 0;
  didSomeoneElseFire = false;
  didIFire = false;
  
  gps_init(); 
  rn2483_init();
  setupCompass();

  DEBUG_STREAM.print(F("\nInit values. milis=")); DEBUG_STREAM.println(millis());

  put_Volts_and_Temp_into_sendbuffer();
  put_Compass_and_Btn_into_sendbuffer();
  doGPS_and_put_values_into_lora_sendbuffer();   

  DEBUG_STREAM.print(F("\nSend one lorawan message as part of system init. milis=")); DEBUG_STREAM.println(millis());

  last_lora_time = millis();
  doOneLoraWan();
  DEBUG_STREAM.print(F("\nCompleted: Setup. milis=")); DEBUG_STREAM.println(millis());
  digitalWrite(hasWalkingDirectionLED, LOW);
}

void loop() {
  DEBUG_STREAM.print(F("\n==== Loop starts. milis=")); DEBUG_STREAM.println(millis());
  ////// GPS pre-loop //////////////
  // DEBUG_STREAM.println(F("\nNo lengthy GPS read-till-fix is needed, the GPS will find/keep a fix as log as power is on. "));

  ////////// Radio  ///////////
  DEBUG_STREAM.print(F("\nRadio listen? milis=")); DEBUG_STREAM.println(millis());
  // now listen a long time for a radio message which we may want to act on, or for a keypress on our side 
  // time needs to be long enough not to miss a radio, we do not worry about GPS as it will keep fix as long as powered
  if(radioActive) {
    bool loraWannaBeNow = false;
    while((millis() - last_lora_time) < (LORAWAN_TX_INTERVAL * 1000L) && !loraWannaBeNow) {
      
      
      // better listen to radio
      // if negotiateState == 1 then check if the shot was a hit
      // listen if someone else fired
      setupRadio();
      listenRadio();
      readCompass();
      
      DEBUG_STREAM.print(F("."));

      if(digitalRead(buttonpin) == LOW) { // input pullup with ground
        didIFire = true;
        negotiateState = 1;
        buttonpressedForLoraWan = true;        
        buttonPressed = 0b10000000;
      } else {
        buttonPressed = 0b00000000;
      }

      if(didIFire && negotiateState == 1) {
        // tell other person I fired
        tone(buzzerPin, 261, 250);
        setupRadio();
        doOneRadio();      
        

        // if you fired you wait 3 times for someone to say something back
        for(int i = 0; i < 3; i++) {
          DEBUG_STREAM.println("Waiting for the other person to say he was hit");
          tone(buzzerPin, 349, 250);
          setupRadio();
          listenRadio();          
        }
        
        loraWannaBeNow = true;
        // reset fire but keep negotiateState to know someone might reply
        didIFire = false;
      }
      
      if(didSomeoneElseFire && shouldITalkBack) {
        // tell other person i was hit
        DEBUG_STREAM.println("Telling other person I was hit");
        setupRadio();
        doOneRadio();
        didSomeoneElseFire = false;
        shouldITalkBack = false;
      }
      
    }
    
    
    DEBUG_STREAM.println();
  }
  // we keep doing this part until it is time to send one LORAWAN TX to the worl

  ////////// Collect data needed just before sending a LORAWAN update to the world  ///////////
  DEBUG_STREAM.println(F("\nCollect data needed just before sending a LORAWAN update. milis=")); DEBUG_STREAM.println(millis());
  
  put_Volts_and_Temp_into_sendbuffer();
  put_Compass_and_Btn_into_sendbuffer();
  doGPS_and_put_values_into_lora_sendbuffer();   

  loraDatasetByte(); // what am i doing with the extra bytes - sending radio to lora or sending extra sensors
  
  ////////// Now we need to send a LORAWAN update to the world  ///////////
  // switch the LMIC antenna to LoraWan mode
  DEBUG_STREAM.println(F("Time or button press tells us to send one LoraWan. milis=")); DEBUG_STREAM.println(millis());
  last_lora_time = millis();
  rn2483_init();
  doOneLoraWan();    

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

  
