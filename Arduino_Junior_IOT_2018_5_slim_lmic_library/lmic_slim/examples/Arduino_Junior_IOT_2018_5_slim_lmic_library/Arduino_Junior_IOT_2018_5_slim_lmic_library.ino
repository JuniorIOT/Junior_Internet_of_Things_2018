#include <avr/pgmspace.h>

// LoRa credentials
#include "keys.h"
#define PAYLOADSIZE 6

#include <lmic_slim.h>

void setup() {
    delay (5000);
    spi_start();
    pinMode(SS_pin, OUTPUT);                                                                  
    pinMode(SCK_pin, OUTPUT);                                         
    pinMode(MOSI_pin, OUTPUT);
    digitalWrite(SCK_pin, LOW);                                                   // SCK low
    digitalWrite(SS_pin, HIGH);                                                   // NSS high
    delay(10);
    writeReg(0x01, 0x08);
    delay(10);
    radio_init ();
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
}

void loop() {
      LMIC_setTxData2(mydata, sizeof(mydata)-1);
      radio_init ();                                                       
      delay (10);
      txlora ();
      delay(1000);                                                                 // wacht op TX ready. Airtime voor 5 bytes payload = 13 x 2^(SF-6) ms
      setopmode(0x00);                                                             // opmode SLEEP
      delay (60000);                                                               // Wacht 1 minuut
}

