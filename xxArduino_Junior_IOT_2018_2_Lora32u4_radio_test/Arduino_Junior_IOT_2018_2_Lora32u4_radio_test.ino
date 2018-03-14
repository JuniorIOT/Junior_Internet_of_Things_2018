//
// from https://learn.adafruit.com/adafruit-feather-32u4-radio-with-lora-radio-module/downloads?view=all#transmitter-example-code
//

// Feather9x_TX
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messaging client (transmitter)
// with the RH_RF95 class. RH_RF95 class does not provide for addressing or
// reliability, so you should only use RH_RF95 if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example Feather9x_RX

#include <SPI.h>
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

#define VBATPIN A9
#define LEDPIN 13

void setup() 
{
  pinMode(LEDPIN, OUTPUT);
  
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  //while (!Serial); // this blocks execute if not connected
  delay(100);
  Serial.begin(9600);
  delay(100);

  Serial.println("Feather LoRa TX Test!");

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  if (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    //while (1);
    halt_stressed();
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    //while (1);
    halt_stressed();
  }
  Serial.print("Set Freq to: "); 
  Serial.println(RF95_FREQ);
  
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);
}

int16_t packetnum = 0;  // packet counter, we increment per xmission

void loop() {  
  digitalWrite(LEDPIN, !digitalRead(LEDPIN)); 
  
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
  int radiopacket_strlen=sprintf(radiopacket, "Hello World #%d Vbatt= %d mV ",packetnum++,vbat);
  Serial.print("Sending "); Serial.println(radiopacket);
  radiopacket[radiopacket_strlen] = 0; // last char was nulled by sprintf?
  
  Serial.println("Sending..."); delay(10);
  rf95.send((uint8_t *)radiopacket, 40);

  // now, sending is done. start listening
  Serial.println("Waiting for packet to complete..."); delay(10);
  rf95.waitPacketSent();
  // Now wait for a reply
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  
  Serial.println("Waiting for reply..."); delay(10);
  if (!rf95.waitAvailableTimeout(5000)) { Serial.println("Nothing received, is there anyone around?"); }
  else { 
    // a message was received
    if (!rf95.recv(buf, &len)) { Serial.println("Receive buffer is empty."); }
    else {
      // message has a length
      RH_RF95::printBuffer("Received: ", buf, len);
      Serial.print("Got reply:                                 [");
      Serial.print((char*)buf);
      Serial.print("] RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);    
      // RSSI values, indication for Wifi: http://www.metageek.com/training/resources/understanding-rssi.html
      //   -45 dBm  =  60 cm distance
      //   -110 dBm  =  probably 200 meter in streets
      //   -120 dBm  =  not good
      //   -125 dBm  =  unusable  
    }
  }

  // end loop
}

void halt_stressed() {  
  while(1) {
    digitalWrite(LEDPIN, HIGH);   
    delay(150);
    digitalWrite(LEDPIN, LOW);  
    delay(50);
  }
}

    

