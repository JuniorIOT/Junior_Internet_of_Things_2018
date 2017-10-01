/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 *
 * Modified By DenniZr & Marco van Schagen for Junior IOT Challenge 2017
 * Modified By Marco van Schagen for Junior IOT Challenge 2018
 *******************************************************************************/ 
 // TODO: We will create a new TTN account 'Kaasfabriek' at some later day where our teams wil be building their stuff.
 
#define DEBUG     // if DEBUG is defined, some code is added to display some basic debug info

#define VBATPIN A9
#define LEDPIN 13


//#include "LowPower.h"   // help to do power save on the arduino  https://github.com/rocketscream/Low-Power
//#include <JeeLib.h>  // Include library containing low power functions
//ISR(WDT_vect) { Sleepy::watchdogEvent(); } // Setup for low power waiting

//////////////////////////////////////////////
// GPS libraries, mappings and things
//////////////////////////////////////////////
//#include <SoftwareSerial.h> 
//#include <TinyGPS.h>
//
//SoftwareSerial ss(GPS_TXD_PIN, GPS_RXD_PIN);  // ss RX, TX --> gps TXD, RXD
//TinyGPS gps;
//long gps_fix_count = 0;
//long gps_nofix_count = 0;

//////////////////////////////////////////////
// LMIC and RFM95 mapping and things
//////////////////////////////////////////////
#include <lmic.h>
#include <hal/hal.h>
const unsigned  TX_INTERVAL = 120;  // previously tested with 300=5 minutes; 15 hours battery time but many gps losses // transmit interval, 5 minutes is healthy according to TTN rules; however 60 sec is still very well possible (risk is getting the device blacklisted rest of day)
const dr_t LMIC_DR_sequence[] = {DR_SF10, DR_SF7, DR_SF7, DR_SF7, DR_SF7, DR_SF7, DR_SF9, DR_SF7, DR_SF7, DR_SF7, DR_SF7, DR_SF7 };      //void LMIC_setDrTxpow (dr_t dr, s1_t txpow)
const int  LMIC_DR_sequence_count = 12;
int  LMIC_DR_sequence_index = 0;

// changed for feather_32u4_lora https://www.thethingsnetwork.org/forum/t/got-adafruit-feather-32u4-lora-radio-to-work-and-here-is-how/6863
//const  lmic_pinmap lmic_pins = { .nss = 8, .rxtx = LMIC_UNUSED_PIN, .rst = 4, .dio = {7, 6, LMIC_UNUSED_PIN}, };
const  lmic_pinmap lmic_pins = { .nss = 8, .rxtx = LMIC_UNUSED_PIN, .rst = 4, .dio = {7, 1, LMIC_UNUSED_PIN}, };

const unsigned message_size = 12;  // including byte[0]
uint8_t  mydata[message_size];  // including byte[0]
// byte 0, 1, 2      Latitude       3 bytes: -90 to +90 degrees rescaled to 0 - 16777215
// byte 3, 4, 5      Longitude      3 bytes: -180 to + 180 degrees rescaled to 0 - 16777215
// byte 6, 7         Altitude       2 bytes: in meters. 0 - 65025 meter
// byte 8            GPS DoP        1 byte: in 0.1 values.  0 - 25.5 DoP
// byte 9            Arduino VCC    1 byte in 50ths Volt. 0 - 5.10 volt
// byte 10           cpu temp       1 byte  value -100. -100 - 155 deg C
// byte 11           time to fix    1 byte: special finction, 10% accuracy 1 sec - 7 hours
      // 0..60 sec  at 1 sec interval <==> values 0 .. 60 
      // 1..10 min at 5 sec interval  <==> values 60 ..  168
      // 10..60 min at 1 min interval <==> values 168 .. 218
      // 1..7 hour at 10 min interval <==> values 218 ..254; 255 is "more than 7 hours"

// THIS BYTE STRING NEEDS A DECODER FUNCTION IN TTN:
/* * function Decoder (bytes) {
  var _lat = ((bytes[0] << 16) + (bytes[1] << 8) + bytes[2]) / 16777215.0 * 180.0 - 90;
  var _lng = ((bytes[3] << 16) + (bytes[4] << 8) + bytes[5]) / 16777215.0 * 360.0 - 180;
  var _alt = (bytes[6] << 8) + bytes[7];
  var _acc = bytes[8] / 10.0;
  var _VCC = bytes[9] / 50;
  var _tempCPU = bytes[10] -100;
  var _time_to_fix_bin = bytes[11];
  var _time_to_fix;
  if (_time_to_fix_bin>=218) { _time_to_fix = 60*60+(_time_to_fix_bin-218)*600 }
  else if (_time_to_fix_bin>=168) { _time_to_fix = 10*60+(_time_to_fix_bin-168)*60 }
  else if (_time_to_fix_bin>=60) {  _time_to_fix = 60+(_time_to_fix_bin-60)*5 }
  else  {_time_to_fix = _time_to_fix_bin }
  
      // 0..60 sec  at 1 sec interval <==> values 0 .. 60 
      // 1..10 min at 5 sec interval  <==> values 60 ..  168
      // 10..60 min at 1 min interval <==> values 168 .. 218
      // 1..7 hour at 10 min interval <==> values 218 ..254; 255 means "more than 7 hours"
      
  var _inputHEX = bytes[0].toString(16)+' '+bytes[1].toString(16)+' '+bytes[2].toString(16)
                  +' '+bytes[3].toString(16)+' '+bytes[4].toString(16)+' '+bytes[5].toString(16)
                  +' '+bytes[6].toString(16)+' '+bytes[7].toString(16)+' '+bytes[8].toString(16)
                  +' / '+bytes[9].toString(16)+' '+bytes[10].toString(16)+' '+bytes[11].toString(16);
  return {
    gps_lat: _lat,
    gps_lng: _lng,
    gps_alt: _alt,
    gps_prec: _acc,
    arduino_VCC: _VCC,
    arduino_temp: _tempCPU,
    time_to_fix: _time_to_fix,
    payload: _inputHEX
  };
}
*/

// // do not keep radio active to listen to return message in RX2. see https://github.com/matthijskooijman/arduino-lmic/blob/master/src/lmic/config.h
// #define DISABLE_JOIN     // Uncomment this to disable all code related to joining
#define DISABLE_PING     // Uncomment this to disable all code related to ping
#define DISABLE_BEACONS  // Uncomment this to disable all code related to beacon tracking.// Requires ping to be disabled too 

#include <SPI.h>  //MISO MOSI SCK stuff
#include "keys.h"  // the personal keys to identify our own nodes

static  osjob_t sendjob;

// os_ interfaces for callbacks only used in over-the-air activation, so functions can be left empty here
void  os_getArtEui (u1_t* buf) { }
void  os_getDevEui (u1_t* buf) { }
void  os_getDevKey (u1_t* buf) { } 

int TX_COMPLETE_was_triggered = 0;  // 20170220 added to allow full controll in main Loop

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
  
  Serial.println(F("\n\nInto send buffer "));
    
  const double shift_lon =   180 * 1000000;        // range shift from -180M..180M into 0..360M
  const double max_old_lon = 360 * 1000000;        // max value longitude is now 360M
  double lon_float = l_lon;                        // put the 4byte LONG into a precise floating point memory space
  lon_float = (lon_float + shift_lon) * max_3byte / max_old_lon; // rescale into 3 byte integer range
  uint32_t LongitudeBinary = lon_float;             // clips off anything after the decimal point
  
  uint16_t altitudeGps = l_alt/100;         // altitudeGps in meters, l_alt from tinyGPS is integer in centimeters
  if (l_alt<0) altitudeGps=0;               // unsigned int wil not allow negative values and warps them to huge number, needs to be zero'ed
  
  uint8_t accuracy = hdopNumber/10;   // from TinyGPS horizontal dilution of precision in 100ths, TinyGPSplus seems the same in 100ths as per MNEMA string
  
  mydata[0] = ( LatitudeBinary >> 16 ) & 0xFF;
  mydata[1] = ( LatitudeBinary >> 8 ) & 0xFF;
  mydata[2] = LatitudeBinary & 0xFF;

  mydata[3] = ( LongitudeBinary >> 16 ) & 0xFF;
  mydata[4] = ( LongitudeBinary >> 8 ) & 0xFF;
  mydata[5] = LongitudeBinary & 0xFF;

  // altitudeGps in meters into unsigned int
  mydata[6] = ( altitudeGps >> 8 ) & 0xFF;
  mydata[7] = altitudeGps & 0xFF;

  // hdop in tenths of meter
  mydata[8] = accuracy & 0xFF;
  
  Serial.print(F(" Mydata[]=[ "));
  for(int i=0; i<message_size; i++) {
    Serial.print(mydata[i], HEX); Serial.print(F(" "));
  }
  Serial.println(F("]"));
}

//////////////////////////////////////////////////
// Kaasfabriek routines for rfm95
///////////////////////////////////////////////

void do_send(){  
  // starting vesion was same as https://github.com/tijnonlijn/RFM-node/blob/master/template%20ttnmapper%20node%20-%20scheduling%20removed.ino
    
    Serial.println(F("\ndo_send "));
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.

        #ifdef DEBUG
        Serial.println(F("  expected CA DA F? 83 5E 9? 0 ?? ??" ));  
        Serial.println(F("    dummy 7F FF FF 7F FF FF 0 0 0" ));    
        #endif  
        Serial.print(F(" Mydata[]=[ "));
        for(int i=0; i<message_size; i++) {
          Serial.print(mydata[i], HEX);  Serial.print(F(" "));
        }
        Serial.print(F("]"));
        
        Serial.print(F(" DR="));
        if ( LMIC_DR_sequence[LMIC_DR_sequence_index]==DR_SF7) Serial.print(F("DR_SF7")); 
        if ( LMIC_DR_sequence[LMIC_DR_sequence_index]==DR_SF8) Serial.print(F("DR_SF8")); 
        if ( LMIC_DR_sequence[LMIC_DR_sequence_index]==DR_SF9) Serial.print(F("DR_SF9")); 
        if ( LMIC_DR_sequence[LMIC_DR_sequence_index]==DR_SF10) Serial.print(F("DR_SF10")); 
        if ( LMIC_DR_sequence[LMIC_DR_sequence_index]==DR_SF11) Serial.print(F("DR_SF11")); 
        if ( LMIC_DR_sequence[LMIC_DR_sequence_index]==DR_SF12) Serial.print(F("DR_SF12")); 
        
        // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
        // for the ttn mapper always use SF7. For Balloon, up to SF12 can be used, however that will require 60 minutes quiet time
        LMIC_setDrTxpow(LMIC_DR_sequence[LMIC_DR_sequence_index],14);   // void LMIC_setDrTxpow (dr_t dr, s1_t txpow)... Set data rate and transmit power. Should only be used if data rate adaptation is disabled.
        
        LMIC_DR_sequence_index = LMIC_DR_sequence_index + 1;
        if (LMIC_DR_sequence_index >= LMIC_DR_sequence_count) LMIC_DR_sequence_index=0;

        // NOW SEND SOME DATA OUT
        //  LMIC_setTxData2( LORAWAN_APP_PORT, LMIC.frame, LORAWAN_APP_DATA_SIZE, LORAWAN_CONFIRMED_MSG_ON );
        LMIC_setTxData2(1, mydata, message_size, 0);   
        Serial.println(F(" - Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

// event gets hooked into the system
void onEvent (ev_t ev) {
    Serial.print(F("\n\nonEvent: "));
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACK"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAIL"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAIL"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            TX_COMPLETE_was_triggered = 1;  // 20170220 our custom code see https://github.com/tijnonlijn/RFM-node/blob/master/template%20ttnmapper%20node%20-%20scheduling%20removed.ino
            
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes"));
            }
       //     // Schedule next transmission   20170220 disabled the interrupt chain, now all controll in main Loop
       //     os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPL"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
         default:
            Serial.println(F("Unknown event"));
            break;
    }
}

void lmic_init() {
    // LMIC init
    os_init();
    
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();
    LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100); // https://www.thethingsnetwork.org/forum/t/got-adafruit-feather-32u4-lora-radio-to-work-and-here-is-how/6863

    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
    #ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
    #else
    // If not running an AVR with PROGMEM, just use the arrays directly
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
    #endif

    #if defined(CFG_eu868)
    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.
    // NA-US channels 0-71 are configured automatically
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.
    #elif defined(CFG_us915)
    // NA-US channels 0-71 are configured automatically
    // but only one group of 8 should (a subband) should be active
    // TTN recommends the second sub band, 1 in a zero based count.
    // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
    LMIC_selectSubBand(1);
    #endif

    // Disable data rate adaptation - per http://platformio.org/lib/show/842/IBM%20LMIC%20framework%20v1.51%20for%20Arduino
    //      and http://www.developpez.net/forums/attachments/p195381d1450200851/environnements-developpement/delphi/web-reseau/reseau-objet-connecte-lorawan-delphi/lmic-v1.5.pdf/
    //LMIC_setAdrMode(0);     // Enable or disable data rate adaptation. Should be turned off if the device is mobile
    // Disable link check validation
    LMIC_setLinkCheckMode(0);  //Enable/disable link check validation. Link check mode is enabled by default and is used to periodically verify network connectivity. Must be called only if a session is established.
    // Disable beacon tracking
    //LMIC_disableTracking ();  // Disable beacon tracking. The beacon will be no longer tracked and, therefore, also pinging will be disabled.
    // Stop listening for downstream data (periodical reception)
    //LMIC_stopPingable();  //Stop listening for downstream data. Periodical reception is disabled, but beacons will still be tracked. In order to stop tracking, the beacon a call to LMIC_disableTracking() is required

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7,14);   // void LMIC_setDrTxpow (dr_t dr, s1_t txpow)... Set data rate and transmit power. Should only be used if data rate adaptation is disabled.
}


///////////////////////////////////////////////
//  some other measurements
///////////////////////////////////////////

double GetTemp(void) { //http://playground.arduino.cc/Main/InternalTemperatureSensor

  unsigned int wADC;
  double t;
  
  // The internal temperature has to be used with the internal reference of 1.1V.
  //ATmega32U4 has 2.56V ref instead of 1.1?
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

long readVcc() { 
  long result;
  float measuredvbat = analogRead(VBATPIN);
  measuredvbat *= 2;                  // we divided by 2, so multiply back
  measuredvbat *= 3.3;                // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024;               // convert to voltage
  result = measuredvbat * 1000;     // convert to mV
  Serial.print("VBat: " ); 
  Serial.print(result);
  Serial.println(" miliVolt");
  return result;
}

void put_VCC_and_Temp_into_sendbuffer() {
  long vcc = readVcc();
  uint8_t vcc_bin = vcc /20 ;  // rescale 0-5100 milli volt into 0 - 255 values
  mydata[9] = vcc_bin;
  #ifdef DEBUG
  Serial.print(F("Vcc="));
  Serial.print(vcc);
  Serial.print(F(" mVolt. vcc_bin="));
  Serial.print(vcc_bin);
  #endif

  double temperature = GetTemp();
  uint8_t temperature_bin = temperature + 100;   // rescale -100 to 155 into 0 - 255 values
  mydata[10] = temperature_bin;
  #ifdef DEBUG
  Serial.print(F(" Temperature="));
  Serial.print(temperature);
  Serial.print(F(" temperature_bin="));
  Serial.println(temperature_bin);
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
      
  mydata[11] = TimeToFix_bin;
  #ifdef DEBUG
  Serial.print(F("TTF="));
  Serial.print(TimeToFix_Seconds);
  Serial.print(F(" sec. TimeToFix_bin="));
  Serial.print(TimeToFix_bin);
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
    
    //Serial.begin(115200);   // whether 9600 or 115200; the gps feed shows repeated char and cannot be interpreted, setting high value to release system time
    delay(100);
    Serial.begin(9600);
    delay(100);
  
    Serial.print(F("\n\n*** Starting ***\ndevice:")); Serial.println(myDeviceName); 
    Serial.println();
    device_startTime = millis();

    //gps_init();
    put_gpsvalues_into_sendbuffer( 52632400, 4738800, 678, 2345);
    
    Serial.println(F("\nto lmic init"));
    lmic_init();  
 
    Serial.println(F("\ninit read values"));
    put_VCC_and_Temp_into_sendbuffer();
    Serial.println(F("\nSending one empty message"));
    do_send();
    Serial.println(F("Waiting.."));  
    while (TX_COMPLETE_was_triggered == 0) {
      os_runloop_once();     // system picks up just the first job from all scheduled jobs, needed for the scheduled and interrupt tasks
    }
    TX_COMPLETE_was_triggered = 0;
    Serial.println(F("TX_COMPL in init"));

    //gps_read_until_fix_or_timeout(60 * 60);  // after factory reset, time to first fix can be 15 minutes (or multiple).  gps needs to acquire full data which is sent out once every 15 minutes; sat data sent out once every 5 minutes

}

void loop() {
  digitalWrite(LEDPIN, !digitalRead(LEDPIN)); 
  unsigned long startTime = millis();
  
  Serial.println(F("\nRead values"));
  put_VCC_and_Temp_into_sendbuffer();
  
  int Time_till_now = (millis() - startTime) / 1000 ; 
  if (!has_sent_allready) {
    has_sent_allready = true;
    Time_till_now = (millis() - device_startTime) / 1000 ; // only the first message tells the world the device boot time till first fix/send
  }
  put_TimeToFix_into_sendbuffer( Time_till_now );
  
  Serial.println(F("\nSending"));
  do_send();
  Serial.println(F("Waiting.."));  
  while (TX_COMPLETE_was_triggered == 0) {
    os_runloop_once();     // system picks up just the first job from all scheduled jobs, needed for the scheduled and interrupt tasks
  }
  TX_COMPLETE_was_triggered = 0;
  Serial.println(F("TX_COMPL"));
  
 
  Serial.print(F("\nSleep all "));
  //=--=-=---=--=-=--=-=--=  START SLEEP HERE -=-=--=-=-=-=-==-=-=-

  unsigned long processedTime = millis() - startTime;
  long sleeptime = TX_INTERVAL  - (processedTime / 1000);
  if ( sleeptime < 0 ) sleeptime = 0;
//  Serial.print(" TX_INTERVAL=" );
//  Serial.print(TX_INTERVAL);
//  Serial.print(" processedTime=" );
//  Serial.print(processedTime);
//  Serial.print(" sleeptime=" );
  Serial.print(sleeptime );
  Serial.println(F(" sec"));
  
  //LowPower.powerDown(SLEEP_4S, ADC_OFF, BOD_OFF);    // this kind of sleep does not work
  //Serial.println(F("sleep2 "));
  //Sleepy::loseSomeTime(8000);  // max 60.000 (60 sec)  // this kind of sleep does not work
  //Serial.println(F("delay "));
  
  delay(sleeptime * 1000);
  
  //=--=-=---=--=-=--=-=--=  SLEEP IS COMPLETED HERE -=-=--=-=-=-=-==-=-=-
  
  Serial.println(F("Sleep done"));
}


