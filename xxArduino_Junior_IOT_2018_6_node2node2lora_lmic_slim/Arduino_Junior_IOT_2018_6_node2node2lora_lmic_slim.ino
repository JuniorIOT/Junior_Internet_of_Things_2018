#include <avr/pgmspace.h>

// LoRa credentials & lmic_slim(lora)
#include <SPI.h>
#include "keys.h"

#include <lmic_slim.h>

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

// example with battery
#define VBATPIN A9
#define LEDPIN LED_BUILTIN
int16_t packetnum = 0;  // packet counter, we increment per transmission
bool ReceivedFromRadio = false;
// radio buf
uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  
  //while (!Serial); // this blocks execute if not connected
  delay(100);
  Serial.begin(9600);
  delay(100);

  delay(3000);  // tijdelijk even delay in de opstart om de monitor te openen - om error bij opstarten te vinden
  Serial.println("Feather and Lora23u4 LoRa TX Test!");  
  digitalWrite(LEDPIN, !digitalRead(LEDPIN)); 
}

void loop() {
  digitalWrite(LEDPIN, !digitalRead(LEDPIN)); 
  
  setupRadio();
  doOneRadio();  // sends a radio messag eand will listen for return message for a certain time
  
  if(ReceivedFromRadio) {
    // use the radio message content for Lora
    memcpy(mydata,buf,40);
    ReceivedFromRadio = false;
  } else {
    sprintf(mydata,"xx geen radio ontvangen xx");
  }
  
  //delay (5000);
  setupLora();
  doOneLora();
  delay (20000); // officially each 3 to 5 minutes one SF7 lora with 10-30 bytes payload
}

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

void setupLora() {
  Serial.println("\nDSetup Lora");  
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

void doOneLora() {
  Serial.println("\nDo one lora");
      Serial.print("Send buffer:              [");
      Serial.print((char*)mydata);
      Serial.println("]");
  LMIC_setTxData2(mydata, sizeof(mydata)-1);
  radio_init ();                                                       
  delay (10);
  //digitalWrite(LED_BUILTIN, HIGH);
  txlora ();
  delay(1000);                        // this is a simple wait with no checking for TX Ready. Airtime voor 5 bytes payload = 13 x 2^(SF-6) ms
  //digitalWrite(LED_BUILTIN, LOW);
  setopmode(0x00);                     // opmode SLEEP
  Serial.println("Done one lora");
}

