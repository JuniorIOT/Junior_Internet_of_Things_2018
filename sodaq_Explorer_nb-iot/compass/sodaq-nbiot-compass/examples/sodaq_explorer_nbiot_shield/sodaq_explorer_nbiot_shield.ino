#include <sodaq_compass.h>

NBIOT_Compass compass();


void setup() {
  SerialUSB.begin(9600);
  compass.setup();  
}

void loop() {
  compass.getNewValues();
  float xguass = compass.getXGauss();
  float yguass = compass.getYGauss();
  float zguass = compass.getZGauss();

  SerialUSB.println("The fieldstrengths are:");
  SerialUSB.print("X: ");
  SerialUSB.print(xguass, 6);
  SerialUSB.println(" Guass.");
  SerialUSB.print("Y: ");
  SerialUSB.print(yguass, 6);
  SerialUSB.println(" Guass.");
  SerialUSB.print("Z: ");
  SerialUSB.print(zguass, 6);
  SerialUSB.println(" Guass.");
  
  delay(5000);
}


