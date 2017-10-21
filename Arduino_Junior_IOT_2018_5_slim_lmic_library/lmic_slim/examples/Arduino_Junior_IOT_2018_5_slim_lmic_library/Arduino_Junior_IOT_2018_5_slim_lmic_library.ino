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
    LMIC_LORARegModemConfig (0x72, 0b01110100, 0x04);  // LORARegModemConfig1, LORARegModemConfig2, LORARegModemConfig3
// LORARegModemConfig1
//    0x72 is normal BW=125 en Coding Rate=4/5, all other messages I see on our TTN basestation have BW=125 en Coding Rate=4/5
// LORARegModemConfig2 
//      nnnn----   Spreading Factor (bit 7..4)
//      ----0---   TxContinuousMode =0 normal mode (bit 3)  
//      -----1--   RxPayloadCrcOn = 1 CRC ON (bit 2)  
//      ------00   SymbTimeout(9:8)=00 default (bit 1..0)
      //    Airtime voor 5 bytes payload = 13 x 2^(SF-6) ms. 
      //    Sec per message with 30-50 bytes: roughly SF12 = 2 seconds, SF10 = 0,5 sec, SF8 = 120 msec, SF7= 60 msec. One device has 30 seconds per day airtime.
      //    Message frequency allowed SF12 = 15 messages per day = 1 message every 90 minutes; SF11 = 1 per 45 min; SF10 = 1 per 24 min, SF9 = 1 per 12 min, SF8 = 1 per 6 min, SF7 = 1 per 3 min roughly
// LORARegModemConfig3
//    0x0C  originally by Rene Harte, IOT-partners for SF11, SF12 
//    0x04  for SF7.....SF10 
//  Het gaat dus om bit 3 van RegModemConfig3, spec van de RFM95 zegt: bit 3: MobileNode, type: rw, Default: 0x00, Value: 0 : Use for static node
 
//    0x72, 0b11000100, 0x0C  //  BW=125, Coding Rate=4/5, SF12, mobile
//              21/10 works "time": "2017-10-21T05:37:14.753250054Z", "frequency": 867.9, "modulation": "LORA", "data_rate": "SF12BW125", "coding_rate": "4/5", "gtw_id": "eui-0000024b080e0bb6", "rssi": -120, "snr": -10
//    0x72, 0b10110100, 0x0C  // BW=125, Coding Rate=4/5, SF11, mobile
//              21/10 works "time": "2017-10-21T05:43:24.228573917Z", "frequency": 868.5, "modulation": "LORA", "data_rate": "SF11BW125", "coding_rate": "4/5", "gtw_id": "eui-0000024b080e0bb6", "channel": 2, "rssi": -120, "snr": -12.8
//              21/10 works on node 5+2
//    0x72, 0b10100100, 0x04  // BW=125, Coding Rate=4/5, SF10, static
//              21/10 works on node 2
//    0x72, 0b10010100, 0x04  // BW=125, Coding Rate=4/5, SF9 
//              21/10 works on node 2
//    0x72, 0b10000100, 0x04  // BW=125, Coding Rate=4/5, SF8 
//              21/10 works on node 2
//    0x72, 0b01110100, 0x04  // BW=125, Coding Rate=4/5, SF7 - this is official setting and is the TTNmapper default
//              21/10 cannot get a message 

// testing from my house: other sensor sending SF9 and SF10 is delivered, all SF7 and SF8 are ignored


  //2017: LMIC_setDrTxpow(DR_SF7,14);   // void LMIC_setDrTxpow (dr_t dr, s1_t txpow)... Set data rate and transmit power. Should only be used if data rate adaptation is disabled.
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

