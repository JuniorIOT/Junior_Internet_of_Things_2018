//JuniorIOTchallenge

#include "Sodaq_HTS221.h"

bool setupHTS221() {
  DEBUG_STREAM.print(F("setupHTS221 started. milis=")); DEBUG_STREAM.println(millis());
  
  // Sodaq explorer temp sensor
  pinMode(TEMP_SENSOR, INPUT);

  // Shield
  return hts221.begin();  
}

int readTemperatureFromShield() {
  return (int)hts221.readTemperature();
}

double readHumidityFromShield() {
  hts221.readHumidity();
}

void put_Temp_into_sendbuffer() {
  DEBUG_STREAM.print(F("put_Temp_into_sendbuffer started. milis=")); DEBUG_STREAM.println(millis());
  /* TODO: 
   *   -- now our 'regular' values
      byte 18         VCC        byte, 50ths, 0 - 5.10 volt -- secret voltmeter
      byte 19         CPUtemp    byte, -100 - 155 deg C     -- secret thermometer
      byte 20         Vbat       byte, 50ths, 0 - 5.10 volt -- hardwired Lora32u4
      
      byte 23,24      Temperature 2 bytes  (was: counter, 2 bytes)
      byte 34, 35     Moisture   2 bytes, AD measurement directly from AD port
  */
  
  //10mV per C, 0C is 500mV
  float mVolts = (float)analogRead(TEMP_SENSOR) * 3300.0 / 1023.0;
  float tempExplorer = (mVolts - 500.0) / 10.0;
  
  double temperatureFromShield = readTemperatureFromShield();  
  double HumidityFromShield = readHumidityFromShield();
  
  uint8_t tempExplorer_bin = tempExplorer + 100; 
  unsigned int temperatureFromShield_bin = temperatureFromShield * 100;
  unsigned int HumidityFromShield_bin = HumidityFromShield * 500;  // reading = 0 --> error?
  
  DEBUG_STREAM.print("    Temp from SodaqExplorer "); DEBUG_STREAM.print(tempExplorer); DEBUG_STREAM.print(" C,  bin="); DEBUG_STREAM.println(tempExplorer_bin); 
  DEBUG_STREAM.print("    Temp from Shield "); DEBUG_STREAM.print(temperatureFromShield); DEBUG_STREAM.print(" C,  bin="); DEBUG_STREAM.print(temperatureFromShield_bin); DEBUG_STREAM.print(" "); DEBUG_STREAM.print((temperatureFromShield_bin >> 8) & 0xFF); DEBUG_STREAM.print(" "); DEBUG_STREAM.print((temperatureFromShield_bin ) & 0xFF);
  DEBUG_STREAM.print("    Humidity from Shield "); DEBUG_STREAM.print(HumidityFromShield); DEBUG_STREAM.print(",  bin="); DEBUG_STREAM.println(HumidityFromShield_bin);
  
  myLoraWanData[19] = tempExplorer_bin;

  myLoraWanData[23] = ( temperatureFromShield_bin >> 8 ) & 0xFF;
  myLoraWanData[24] = temperatureFromShield_bin & 0xFF;
  
  myLoraWanData[34] = ( HumidityFromShield_bin >> 8 ) & 0xFF;
  myLoraWanData[35] = HumidityFromShield_bin & 0xFF;
  

}

