#include <avr/pgmspace.h>

// LoRa credentials
#include "keys.h"

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

