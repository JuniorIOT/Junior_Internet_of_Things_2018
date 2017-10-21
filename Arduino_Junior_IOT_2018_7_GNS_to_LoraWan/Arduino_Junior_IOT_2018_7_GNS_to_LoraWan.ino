/*******************************************************************************
 * Modified By DenniZr & Marco van Schagen for Junior IOT - Smart City Challenge 2018 #juniorIOTchallenge2018
 * Modified By Marco van Schagen for Junior IOT Challenge 2018
 *******************************************************************************/ 
 
//#define DEBUG     // if DEBUG is defined, some code is added to display some basic debug info


#define VBATPIN A9
#define LEDPIN 13 
#define TX_INTERVAL 60  // seconds between LoraWan messages

//////////////////////////////////////////////
// GPS libraries, mappings and things
//////////////////////////////////////////////
#define GPS_FIX_HDOP
#define GPS_TXD_PIN 11    // where we plugged in our GNSS GPS into Lora32u4
#define GPS_RXD_PIN 10  
#ifndef GPS_FIX_HDOP
  #error GPS_FIX_HDOP must be defined in GPSfix_cfg.h!
#endif

#include <NeoSWSerial.h>  //  We now use NeoSWSerial for lower footprint end better performance than SoftwareSerial
  // an issue with Leonardo-types is fixed in branch, yet to be merged into main version library. So you may need to remove all your NeoSWSerial libraries and add \libraries\NeoSWSerial-master-DamiaBranch.zip
NeoSWSerial ss(GPS_RXD_PIN, GPS_TXD_PIN);

#include <NMEAGPS.h>       // We now use NmeaGps (or NeoGps) as it understands newer GNSS
static NMEAGPS gps;    // This parses the GPS characters


long gps_fix_count = 0;
long gps_nofix_count = 0;
unsigned long gps_last_time = millis();
unsigned long gps_gets_time = 5000;

//////////////////////////////////////////////
// LoraWan libraries, mappings and things
//////////////////////////////////////////////

//#include <SPI.h>  //MISO MOSI SCK stuff that was part of 2017 thing with rfm95
#define PAYLOADSIZE 9 // The size of the package to be sent
#include <avr/pgmspace.h>
#include <lmic_slim.h>     // the really cool micro-library, to replace our 2017 LMIC which filled 99% memory
#include "keys.h"          // the personal keys to identify our own nodes, in a file outside GITHUB

int TX_COMPLETE_was_triggered = 0;  // 20170220 added to allow full controll in main Loop
uint8_t  myLoraWanData[40];  // including byte[0]
unsigned long last_lora_time = millis(); // last time lorawan ran
//--------------Table of contents------------//
//////////////////////////////////////////////////////////
//// Kaasfabriek routines for gps
////////////////////////////////////////////
void put_gpsvalues_into_sendbuffer(long l_lat, long l_lon, long l_alt, int hdopNumber);
//void process_gps_values(const gps_fix & fix );
void gps_init();
//////////////////////////////////////////////////
// Kaasfabriek routines for LMIC_slim for LoraWan
///////////////////////////////////////////////
void setupLora();
void doOneLora();
//////////////////////////////////////////////////
// Kaasfabriek routines for RFM95 radio to radio 
///////////////////////////////////////////////
void doOneRadio();
void halt_stressed();
void setupRadio();
///////////////////////////////////////////////
//  some other measurements
///////////////////////////////////////////
double GetTemp(void);
long readVcc();
void put_TimeToFix_into_sendbuffer(int TimeToFix_Seconds);
///////////////////////////////////////////////
//  arduino init and main
///////////////////////////////////////////
void setup();
void loop();
//--------------Table of contents------------//



//////////////////////////////////////////////////////////
//// Kaasfabriek routines for gps
////////////////////////////////////////////

void put_gpsvalues_into_sendbuffer(long l_lat, long l_lon, long l_alt, int hdopNumber) {
  const double shift_lat =    90 * 1000000;        // range shift from -90M..90M into 0..180M
  const double max_old_lat = 180 * 1000000;        // max value for lat is now 180M
  const double max_3byte =        16777215;        // max value that fits in 3 bytes
  double lat_float = l_lat;                        // put the 4byte LONG into a more precise floating point to prevent round-off effect in calculation
  lat_float = (lat_float + shift_lat) * max_3byte / max_old_lat; // rescale into 3 byte integer range
  uint32_t LatitudeBinary = lat_float;             // clips off anything after the decimal point    
  const double shift_lon =   180 * 1000000;        // range shift from -180M..180M into 0..360M
  const double max_old_lon = 360 * 1000000;        // max value longitude is now 360M
  double lon_float = l_lon;                        // put the 4byte LONG into a precise floating point memory space
  lon_float = (lon_float + shift_lon) * max_3byte / max_old_lon; // rescale into 3 byte integer range
  uint32_t LongitudeBinary = lon_float;             // clips off anything after the decimal point  
  uint16_t altitudeGps = l_alt/100;         // altitudeGps in meters, l_alt from tinyGPS is integer in centimeters
  if (l_alt<0) altitudeGps=0;               // unsigned int wil not allow negative values and warps them to huge number, needs to be zero'ed  
  uint8_t accuracy = hdopNumber/10;   // from TinyGPS horizontal dilution of precision in 100ths, TinyGPSplus seems the same in 100ths as per MNEMA string
  
  myLoraWanData[0] = ( LatitudeBinary >> 16 ) & 0xFF;
  myLoraWanData[1] = ( LatitudeBinary >> 8 ) & 0xFF;
  myLoraWanData[2] = LatitudeBinary & 0xFF;
  myLoraWanData[3] = ( LongitudeBinary >> 16 ) & 0xFF;
  myLoraWanData[4] = ( LongitudeBinary >> 8 ) & 0xFF;
  myLoraWanData[5] = LongitudeBinary & 0xFF;
  // altitudeGps in meters into unsigned int
  myLoraWanData[6] = ( altitudeGps >> 8 ) & 0xFF;
  myLoraWanData[7] = altitudeGps & 0xFF;
  // hdop in tenths of meter
  myLoraWanData[8] = accuracy & 0xFF;
//  Serial.print(F(" myLoraWanData[]=[ "));  //  for(int i=0; i<message_size; i++) {  //    Serial.print(myLoraWanData[i], HEX); Serial.print(F(" "));  //  }  //  Serial.println(F("]"));
}

// 04 datetime sec, 561398704 datetime, 526326337 lat, 47384373 lon, 8, 1.067000 kn = 1.227881 mph
void process_gps_values(const gps_fix & fix ) {   // constant pointer to fix object
  //  This is the best place to do your time-consuming work, right after the RMC sentence was received.  If you do anything in "loop()",
  //     you could cause GPS characters to be lost, and you will not get a good lat/lon.
  //  For this example, we just print the lat/lon.  If you print too much, this routine will not get back to "loop()" in time to process
  //     the next set of GPS data.
  long l_lat, l_lon, l_alt;
  unsigned long age; 
  int hdopNumber;  
  bool GPS_values_are_valid = true;
  
  if (fix.valid.location && fix.dateTime.seconds > 0) {
    if ( fix.dateTime.seconds < 10 )
      Serial.print( "0" );
    Serial.print( fix.dateTime.seconds ); Serial.print(" datetime sec, ");
    Serial.print( fix.dateTime ); Serial.print(" datetime, ");
    
    Serial.print( fix.latitude(), 6 ); // floating-point display
    l_lat = fix.latitudeL();
    Serial.print( l_lat  ); Serial.print(" lat, ");
    // Serial.print( fix.longitude(), 6 ); // floating-point display
    l_lon = fix.longitudeL();
    Serial.print( l_lon ); Serial.print(" lon, ");
    if (fix.valid.satellites)
      Serial.print( fix.satellites );
    Serial.print(", ");
    Serial.print( fix.speed(), 6 );
    Serial.print( F(" kn = ") );
    hdopNumber = fix.hdop;
    l_alt = fix.alt.whole;
    Serial.println();
  } else {
    // No valid location data yet!
    //Serial.println( "No valid location data yet!" );
  }
  
  
  put_gpsvalues_into_sendbuffer( l_lat, l_lon, l_alt, hdopNumber);
  
}

void gps_init() {  
  Serial.print(F("GPS init"));
    
  // load the send buffer with dummy location 0,0. This location 0,0 is recognized as dummy by TTN Mapper and will be ignored
  //put_gpsvalues_into_sendbuffer( 0, 0, 0, 0);
  put_gpsvalues_into_sendbuffer( 52632400, 4738800, 678, 2345); // Alkmaar
  
  // GPS serial starting
  Serial.print( F("The NeoGps people are prowd to show their smallest possible size:\n") );
  Serial.print( F("NeoGps, fix object size = ") ); Serial.println( sizeof(gps.fix()) );
  Serial.print( F("NeoGps, NMEAGPS object size = ") ); Serial.println( sizeof(gps) );

  #ifdef NMEAGPS_NO_MERGING
    Serial.println( F("Only displaying data from xxRMC sentences.\n  Other sentences may be parsed, but their data will not be displayed.") );
  #endif
  
  Serial.flush();
  ss.begin(9600);
  
  //gps_requestColdStart();  // DO NOT USE: it seems this does a FACTORY RESET and delays getting a solid fix
//  gps_SetMode_gpsRfOn();
//  gps_setStrings();
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
    
//////////////////////////////////////////////////
// Kaasfabriek routines for LMIC_slim for LoraWan
///////////////////////////////////////////////

void lmic_slim_init() {  
  Serial.println("\nlmic_slim_init");   
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
  LMIC_LORARegModemConfig2 (0b11000100);  
// LORARegModemConfig2 0b11000100  // SF12 works
// LORARegModemConfig2 0b10110100  // SF11 works
// LORARegModemConfig2 0b10100100  // SF10 
// LORARegModemConfig2 0b10010100  // SF9 
// LORARegModemConfig2 0b10000100  // SF8 
// LORARegModemConfig2 0b01110100  // SF7 official setting
       // Register LORARegModemConfig2 
       //          0b0000 0000
       //            nnnn ----   Spreading Factor (bit 7..4)
       //            0111 ----     7 = SF7    is the TTNmapper default
       //            1011 ----    11 = SF11   tested
       //            1100 ----    12 = SF12   tested & working
       //            ---- 0---   TxContinuousMode =0 normal mode (bit 3)  
       //            ---- -1--   RxPayloadCrcOn = 1 CRC ON (bit 2)  
       //            ---- --00   SymbTimeout(9:8)=00 default (bit 1..0)
      // Airtime voor 5 bytes payload = 13 x 2^(SF-6) ms. 
      // with 30-50 bytes: SF12 = 2 seconds, SF10 = 0,5 sec, SF8 = 120 msec, SF7= 70 msec. One device has 30 seconds per day airtime.

  //LMIC_setDrTxpow(DR_SF7,14);   // void LMIC_setDrTxpow (dr_t dr, s1_t txpow)... Set data rate and transmit power. Should only be used if data rate adaptation is disabled.
}

void doOneLoraWan() {
  Serial.println("\nDo one lora");
  Serial.print("Send buffer:              [");
  Serial.print((char*)myLoraWanData);
  Serial.println("]");
  LMIC_setTxData2(myLoraWanData, sizeof(myLoraWanData)-1);
  radio_init();                                                       
  delay (10);
  digitalWrite(LED_BUILTIN, HIGH);
  txlora();
                          // this is a simple wait with no checking for TX Ready. 
                          // Airtime voor 5 bytes payload = 13 x 2^(SF-6) ms. 
                          // with 30-50 bytes: SF12 = 2 seconds, SF10 = 0,5 sec, SF8 = 120 msec, SF7= 70 msec. One device has 30 seconds per day airtime.
  digitalWrite(LED_BUILTIN, LOW);
  setopmode(0x00);                     // opmode SLEEP
  Serial.println("Done one lora");
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
  Serial.println("\nDo one radio");  
    
  // preparing to send a message to everyone  
  float measuredvbat = analogRead(VBATPIN);
  measuredvbat *= 2;                  // we divided by 2, so multiply back
  measuredvbat *= 3.3;                // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024;               // convert to voltage
  int vbat = measuredvbat * 1000;     // convert to mV
  Serial.print("VBat: " ); 
  Serial.print(vbat);
  Serial.println(" miliVolt");
  
  char radiopacket[40] = "Hello World #       Vbatt= #       mV  ";
  int radiopacket_strlen=sprintf(radiopacket, "Radio message #%d 'Vbatt= %d mV' ",packetnum++,vbat);
  Serial.print("radiopacket: "); Serial.println(radiopacket);
  radiopacket[radiopacket_strlen] = 0; // last char was nulled by sprintf?
  
  Serial.println("Sending..."); delay(10);
  rf95.send((uint8_t *)radiopacket, 40);

  // now, sending is done. start listening
  Serial.println("Waiting for radio packet to complete..."); delay(10);
  rf95.waitPacketSent();
  // Now wait for a reply  
  uint8_t len = sizeof(buf);
  
  Serial.println("Waiting for some other radio to reply..."); delay(10);
  if (!rf95.waitAvailableTimeout(20000)) { Serial.println("No radio received in 20 sec, is there anyone around in same send settings?"); }
  else { 
    // a message was received
    if (!rf95.recv(buf, &len)) { Serial.println("Receive buffer is empty."); }
    else {
      // message has a length
      RH_RF95::printBuffer("Received this radio message: ", buf, len);
      Serial.print("Got reply:              [");
      Serial.print((char*)buf);
      Serial.print("] RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);    
      // RSSI values, indication for Wifi: http://www.metageek.com/training/resources/understanding-rssi.html
      //   -45 dBm  =  60 cm distance
      //   -110 dBm  =  probably 200 meter in streets
      //   -120 dBm  =  not good
      //   -125 dBm  =  unusable 
      ReceivedFromRadio = true; 
    }
  }

  // end loop
  Serial.println("Done one radio");  
}

void halt_stressed() {  
  Serial.println("\nPanic. Halted.");  
  while(1) {
    digitalWrite(LEDPIN, HIGH);   
    delay(150);
    digitalWrite(LEDPIN, LOW);  
    delay(50);
    Serial.print("x");  
  }
}   

void setupRadio() {  
  Serial.println("\nSetup radio");  
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
  Serial.println("Radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    //while (1);
    halt_stressed();
  }
  Serial.print("Freq is set to: "); 
  Serial.println(RF95_FREQ);
  
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  
  //rf95.setTxPower(23, false);
  rf95.setTxPower(13, false);   // is 13 TTN & TTNtracker intended spec?
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

//long readVccCPU() {  //http://dumbpcs.blogspot.nl/2013/07/arduino-secret-built-in-thermometer.html
//  long result;
//  // Read 1.1V reference against AVcc 
//  //ATmega32U4 has 2.56V ref instead of 1.1?
//  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
//  delay(2); // Wait for Vref to settle
//  ADCSRA |= _BV(ADSC); // Convert
//  while (bit_is_set(ADCSRA,ADSC));
//  result = ADCL;
//  result |= ADCH<<8;
//  result = 1126400L / result; // Back-calculate AVcc in mV
//  return result;
//}

long readVcc() {
  long result;
  float measuredvbat = analogRead(VBATPIN);
    // devide by 1024 to convert to voltage
    // we divided by 2 using 2x 100M Ohm, so multiply back
    // Multiply by 3.3V, our reference voltage
    // *1000 to get milliVolt
  measuredvbat *= 6600 / 1024;         
  result = measuredvbat;
//  Serial.print(F("Vcc=") ); 
//  Serial.print(result);
//  Serial.println(F("mV"));
  return result;
}

void put_VCC_and_Temp_into_sendbuffer() {
  long vcc = readVcc();
  uint8_t vcc_bin = vcc /20 ;  // rescale 0-5100 milli volt into 0 - 255 values
  myLoraWanData[9] = vcc_bin;
  #ifdef DEBUG
  Serial.print(F("Vcc=")); Serial.print(vcc); Serial.print(F(" mV. vcc_bin=")); Serial.print(vcc_bin);
  #endif

  double temperature = GetTemp();
  uint8_t temperature_bin = temperature + 100;   // rescale -100 to 155 into 0 - 255 values
  myLoraWanData[10] = temperature_bin;
  #ifdef DEBUG
  Serial.print(F(" Temp=")); Serial.print(temperature); Serial.print(F(" bin=")); Serial.println(temperature_bin);
  #endif
}

void put_TimeToFix_into_sendbuffer(int TimeToFix_Seconds) {  // time to fix onto gps coordinates
  int TimeToFix_Calculate;  // this helps to calculate but no round-off yet
  if ( TimeToFix_Seconds < 0) {
    TimeToFix_Calculate=0;
  } else if ( TimeToFix_Seconds <= (1 * 60) ) {  
    TimeToFix_Calculate = TimeToFix_Seconds;                  // 0..60 sec  at 1 sec interval <==> values 0 .. 60 
  } else if ( TimeToFix_Seconds <= (10 * 60) ) {    
    TimeToFix_Calculate = 60 + (TimeToFix_Seconds - (1* 60) )/5 ;   // 1..10 min at 5 sec interval  <==> values 60 ..  168 
  } else if ( TimeToFix_Seconds <= (60 * 60) ) {  
    TimeToFix_Calculate = 168 + (TimeToFix_Seconds - (10 * 60) )/60 ;   // 10..60 min at 1 min interval <==> values 168 .. 218     
  } else {
    TimeToFix_Calculate = 218 + (TimeToFix_Seconds - (60 * 60) )/600 ;    // 1..7:00 hour at 10 min interval <==> values 218 ..254   
  }
  if (TimeToFix_Calculate>255) TimeToFix_Calculate = 255 ;                  //  more than 7 hour = 255
  
  uint8_t TimeToFix_bin = TimeToFix_Calculate;  // this can contain the values 0..255,
      
  myLoraWanData[11] = TimeToFix_bin;
  #ifdef DEBUG
  Serial.print(F("TTF=")); Serial.print(TimeToFix_Seconds); Serial.print(F(" sec. bin=")); Serial.print(TimeToFix_bin);
  #endif
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

  Serial.print(F("\n Starting\ndevice:")); Serial.println(DEVADDR); Serial.println();
  device_startTime = millis();

  gps_init(); put_gpsvalues_into_sendbuffer( 52632400, 4738800, 678, 2345);
  
  Serial.println(F("\nlmic init"));
  lmic_slim_init();  

  Serial.println(F("\ninit values"));
  put_VCC_and_Temp_into_sendbuffer();    

  Serial.println(F("\nSend one lorawan message as part of system init"));
  LMIC_setTxData2(myLoraWanData, sizeof(myLoraWanData)-1);
  radio_init();                                                       
  delay (10);
  txlora();
  delay(1000);                    // wacht op TX ready. Airtime voor 5 bytes payload = 13 x 2^(SF-6) ms
  setopmode(0x00);                // opmode SLEEP
  last_lora_time = millis();
  //gps_read_until_fix_or_timeout(60 * 60);  // after factory reset, time to first fix can be 15 minutes (or multiple).  gps needs to acquire full data which is sent out once every 15 minutes; sat data sent out once every 5 minutes
}

boolean radioActive = false;  // this name is for radio, not LoraWan
boolean loraWannaBe = false;

void loop() {
  digitalWrite(LEDPIN, !digitalRead(LEDPIN)); 
  unsigned long startTime = millis();
  
  //Serial.println(F("\nValues"));
  put_VCC_and_Temp_into_sendbuffer();
  
  int Time_till_now = (millis() - startTime) / 1000 ; 
  if (!has_sent_allready) {
    has_sent_allready = true;
    Time_till_now = (millis() - device_startTime) / 1000 ; // only the first message tells the world the device boot time till first fix/send
  }
  put_TimeToFix_into_sendbuffer( Time_till_now );

  // some radio  
  if(radioActive) {
    setupRadio();
    doOneRadio();  // sends a radio message and will listen for return message for a certain time
    if(ReceivedFromRadio) {
      // use the radio message content for Lora
      memcpy(myLoraWanData,buf,PAYLOADSIZE);
      ReceivedFromRadio = false;
    } else {
      sprintf(myLoraWanData,"xx geen radio ontvangen xx");
    }  
  }
  if (ss.available()) {
    gps.available(ss);
    process_gps_values( gps.read()); 
  }
    
 
  
  if((millis() - last_lora_time) > (TX_INTERVAL * 1000L)) {
    last_lora_time = millis();
    Serial.println(F("\nSend one LoraWan"));
    lmic_slim_init();
    doOneLoraWan();    
  }
  
 
}



