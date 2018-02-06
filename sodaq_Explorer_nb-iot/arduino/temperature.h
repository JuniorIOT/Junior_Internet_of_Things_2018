//JuniorIOTchallenge

#include "Sodaq_HTS221.h"

bool setupTemperature() {
  return hts221.begin();  
}

int readTemperatureFromShield() {
  return (int)hts221.readTemperature();
}

void put_Volts_and_Temp_into_sendbuffer() {
/* TODO: 
 *   -- now our 'regular' values
    byte 18         VCC        byte, 50ths, 0 - 5.10 volt -- secret voltmeter
    byte 19         CPUtemp    byte, -100 - 155 deg C     -- secret thermometer
    byte 20         Vbat       byte, 50ths, 0 - 5.10 volt -- hardwired Lora32u4
*/
int temperature = readTemperatureFromShield();
uint8_t temperature_bin = temperature + 100; 
myLoraWanData[19] = temperature_bin;
}

double readHumidityFromShield() {
  hts221.readHumidity();
}

