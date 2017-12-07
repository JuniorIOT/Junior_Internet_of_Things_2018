#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303.h>

Adafruit_LSM303 lsm;

void setup() 
{
  Wire.begin();
  SerialUSB.begin(9600);
  
  // Try to initialise and warn if we couldn't detect the chip
  if (!lsm.begin())
  {
    SerialUSB.println("Oops ... unable to initialize the LSM303. Check your wiring!");
    while (1);
  }
}

void loop() 
{
  lsm.read();
  SerialUSB.print("Accel X: "); SerialUSB.print((int)lsm.accelData.x); SerialUSB.print(" ");
  SerialUSB.print("Y: "); SerialUSB.print((int)lsm.accelData.y);       SerialUSB.print(" ");
  SerialUSB.print("Z: "); SerialUSB.println((int)lsm.accelData.z);     SerialUSB.print(" ");
  
  delay(1000);
}
