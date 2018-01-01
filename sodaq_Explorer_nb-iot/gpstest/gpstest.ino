#include <Arduino.h>
#include <Sodaq_nbIOT.h>
#include "Sodaq_UBlox_GPS.h"

#define DEBUG_STREAM SerialUSB

#define ARRAY_DIM(arr)  (sizeof(arr) / sizeof(arr[0]))

Sodaq_nbIOT nbiot;

void find_fix(uint32_t delay_until);
void do_flash_led(int pin);

void setup()
{
   while ((!DEBUG_STREAM) && (millis() < 10000)) {
     // Wait for serial monitor for 10 seconds
   }

    DEBUG_STREAM.begin(57600);
    digitalWrite(13, HIGH);
    pinMode(13, OUTPUT);
    
    sodaq_gps.init(6);

    //sodaq_gps.setMinNumOfLines(10);

    //sodaq_gps.setDiag(DEBUG_STREAM);

    find_fix(0);
}

void loop()
{
    find_fix(60000);
    if(sodaq_gps.getMagHeading() != -1) while(1) do_flash_led(13);
}

/*!
 * Find a GPS fix, but first wait a while
 */
void find_fix(uint32_t delay_until)
{
    
    uint32_t start = millis();
    uint32_t timeout = 900L * 1000;
    DEBUG_STREAM.println(String("waiting for fix ..., timeout=") + timeout + String("ms"));
    if (sodaq_gps.scan(true, timeout)) {
      
        
    } else {
        DEBUG_STREAM.println("No Fix");
    }
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

