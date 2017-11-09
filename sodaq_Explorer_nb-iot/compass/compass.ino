// Wire Master Writer
// by Nicholas Zambetti <http://www.zambetti.com>

// Demonstrates use of the Wire library
// Writes data to an I2C/TWI slave device
// Refer to the "Wire Slave Receiver" example for use with this

// Created 29 March 2006

// This example code is in the public domain.


#include <Wire.h>
#define compass_device (0x1E)
void setup()
{
  SerialUSB.begin(9600);
  Wire.begin(); // join i2c bus (address optional for master)
}

byte x = 0;

void loop()
{

Wire.beginTransmission(compass_device);    // Get the slave's attention, tell it we're sending a command byte
Wire.write(0x0F);                               //  The command byte, sets pointer to register with address of 0x32
Wire.endTransmission();
Wire.requestFrom(compass_device,1);          // Tell slave we need to read 1byte from the current register
byte slaveByte2 = Wire.read();        // read that byte into 'slaveByte2' variable
Wire.endTransmission();
  SerialUSB.print("Device is: ");
  SerialUSB.println(slaveByte2, HEX);
;
//CTRL_REG1_M
SerialUSB.println("Writing [temp on] [xy low power] [xy low power] [10hz] [10hz] [10h] [self test off] [0]");
SerialUSB.println("0b10010000");
SerialUSB.println("To address 0x20 CTRL_REG1_M");
//[temp on] [xy low power] [xy low power] [10hz] [10hz] [10h] [self test off] [0] 
set_register(0x20, 0b10010000); 

Wire.beginTransmission(compass_device);    // Get the slave's attention, tell it we're sending a command byte
Wire.write(0x20);                               //  The command byte, sets pointer to register with address of 0x32
Wire.endTransmission();
Wire.requestFrom(compass_device,1);          // Tell slave we need to read 1byte from the current register
slaveByte2 = Wire.read();        // read that byte into 'slaveByte2' variable

  SerialUSB.print("Register 0x20 is: ");
  SerialUSB.println(slaveByte2, HEX);
;

SerialUSB.println("Writing to address 0x21 CTRL_REG2_M");
// CTRL_REG2_M
// [always 0] [standard resolution] [standard resolution] [always 0] [do not reboot memory] [do not soft reset] [always 0] [always 0]
set_register(0x21, 0b00000000); 
SerialUSB.println("[always 0] [standard resolution] [standard resolution] [always 0] [do not reboot memory] [do not soft reset] [always 0] [always 0]");
SerialUSB.println("0b00000000");
;

Wire.beginTransmission(compass_device);    // Get the slave's attention, tell it we're sending a command byte
Wire.write(0x21);                               //  The command byte, sets pointer to register with address of 0x32
Wire.endTransmission();
Wire.requestFrom(compass_device,1);          // Tell slave we need to read 1byte from the current register
slaveByte2 = Wire.read();        // read that byte into 'slaveByte2' variable

  SerialUSB.print("Register 0x21 is: ");
  SerialUSB.println(slaveByte2, HEX);
;

//CTRL_REG3_M
SerialUSB.println("Writing to address 0x22 CTRL_REG3_M");
SerialUSB.println("[i2c enable] [always 0] [low power] [always 0] [always 0][spi only write - but not using this] [Continuous-conversion mode] [Continuous-conversion mode]");
SerialUSB.println("0b00000000");
// doubting [Continuous-conversion mode]
set_register(0x22, 0b00000000);
;
Wire.beginTransmission(compass_device);    // Get the slave's attention, tell it we're sending a command byte
Wire.write(0x22);                               //  The command byte, sets pointer to register with address of 0x32
Wire.endTransmission();
Wire.requestFrom(compass_device,1);          // Tell slave we need to read 1byte from the current register
slaveByte2 = Wire.read();        // read that byte into 'slaveByte2' variable

  SerialUSB.print("Register 0x22 is: ");
  SerialUSB.println(slaveByte2, HEX);
;

// CTRL_REG4_M
SerialUSB.println("Writing to address 0x23 CTRL_REG4_M");
SerialUSB.println("[always 0] [always 0] [always 0] [always 0] [z low power] [z low power] [data LSb at lower address] [always 0]");
set_register(0x23, 0b00000000);
;
Wire.beginTransmission(compass_device);    // Get the slave's attention, tell it we're sending a command byte
Wire.write(0x23);                               //  The command byte, sets pointer to register with address of 0x32
Wire.endTransmission();
Wire.requestFrom(compass_device,1);          // Tell slave we need to read 1byte from the current register
slaveByte2 = Wire.read();        // read that byte into 'slaveByte2' variable

  SerialUSB.print("Register 0x23 is: ");
  SerialUSB.println(slaveByte2, HEX);
;

// CTRL_REG5_M
SerialUSB.println("Writing to address 0x24 CTRL_REG5_M");
SerialUSB.println("[always 0] [after byte read]"); 
SerialUSB.println("0b01000000");
set_register(0x24, 0b01000000);
;
Wire.beginTransmission(compass_device);    // Get the slave's attention, tell it we're sending a command byte
Wire.write(0x24);                               //  The command byte, sets pointer to register with address of 0x32
Wire.endTransmission();
Wire.requestFrom(compass_device,1);          // Tell slave we need to read 1byte from the current register
slaveByte2 = Wire.read();        // read that byte into 'slaveByte2' variable

  SerialUSB.print("Register 0x24 is: ");
  SerialUSB.println(slaveByte2, HEX);
;




// OUT_X_L_M 28
// OUT_X_H_M 29
Wire.beginTransmission(compass_device);    // Get the slave's attention, tell it we're sending a command byte
Wire.write(0x28);                               //  The command byte, sets pointer to register with address of 0x32
Wire.endTransmission();
Wire.requestFrom(compass_device,1);          // Tell slave we need to read 1byte from the current register
byte OUT_X_L_M = Wire.read();        // read that byte into 'slaveByte2' variable
Wire.beginTransmission(compass_device);    // Get the slave's attention, tell it we're sending a command byte
Wire.write(0x29);                               //  The command byte, sets pointer to register with address of 0x32
Wire.endTransmission();
Wire.requestFrom(compass_device,1);          // Tell slave we need to read 1byte from the current register
byte OUT_X_H_M = Wire.read();        // read that byte into 'slaveByte2' variable

// OUT_Y_L_M 2A
// OUT_Y_H_M 2B
Wire.beginTransmission(compass_device);    // Get the slave's attention, tell it we're sending a command byte
Wire.write(0x2A);                               //  The command byte, sets pointer to register with address of 0x32
Wire.endTransmission();
Wire.requestFrom(compass_device,1);          // Tell slave we need to read 1byte from the current register
byte OUT_Y_L_M = Wire.read();        // read that byte into 'slaveByte2' variable
Wire.beginTransmission(compass_device);    // Get the slave's attention, tell it we're sending a command byte
Wire.write(0x2B);                               //  The command byte, sets pointer to register with address of 0x32
Wire.endTransmission();
Wire.requestFrom(compass_device,1);          // Tell slave we need to read 1byte from the current register
byte OUT_Y_H_M = Wire.read();        // read that byte into 'slaveByte2' variable

// OUT_Z_L_M 2C
// OUT_Z_H_M 2D
Wire.beginTransmission(compass_device);    // Get the slave's attention, tell it we're sending a command byte
Wire.write(0x2C);                               //  The command byte, sets pointer to register with address of 0x32
Wire.endTransmission();
Wire.requestFrom(compass_device,1);          // Tell slave we need to read 1byte from the current register
byte OUT_Z_L_M = Wire.read();        // read that byte into 'slaveByte2' variable
Wire.beginTransmission(compass_device);    // Get the slave's attention, tell it we're sending a command byte
Wire.write(0x2D);                               //  The command byte, sets pointer to register with address of 0x32
Wire.endTransmission();
Wire.requestFrom(compass_device,1);          // Tell slave we need to read 1byte from the current register
byte OUT_Z_H_M = Wire.read();        // read that byte into 'slaveByte2' variable

// TEMP_L_M 2E
// TEMP_H_M 2F
Wire.beginTransmission(compass_device);    // Get the slave's attention, tell it we're sending a command byte
Wire.write(0x2E);                               //  The command byte, sets pointer to register with address of 0x32
Wire.endTransmission();
Wire.requestFrom(compass_device,1);          // Tell slave we need to read 1byte from the current register
byte TEMP_L_M = Wire.read();        // read that byte into 'slaveByte2' variable
Wire.beginTransmission(compass_device);    // Get the slave's attention, tell it we're sending a command byte
Wire.write(0x2F);                               //  The command byte, sets pointer to register with address of 0x32
Wire.endTransmission();
Wire.requestFrom(compass_device,1);          // Tell slave we need to read 1byte from the current register
byte TEMP_H_M = Wire.read();        // read that byte into 'slaveByte2' variable

SerialUSB.print("OUT_X_H_M: ");
SerialUSB.print(OUT_X_H_M,HEX);
SerialUSB.print("OUT_X_L_M: ");
SerialUSB.println(OUT_X_L_M,HEX);

int x = ((OUT_X_H_M << 8) + OUT_X_L_M);
if(OUT_X_H_M >= 0b10000000) {
  SerialUSB.println("Negative");
  OUT_X_H_M -= 0b10000000;
  x = ((OUT_X_H_M << 8) + OUT_X_L_M);
  x -= 32768;
} else {
  SerialUSB.println("Positive");
}

int y = ((OUT_Y_H_M << 8) + OUT_Y_L_M);
if(OUT_Y_H_M >= 0b10000000) {
  OUT_Y_H_M -= 0b10000000;
  y = ((OUT_Y_H_M << 8) + OUT_Y_L_M);
  y -= 32768;
}
int z = ((OUT_Z_H_M << 8) + OUT_Z_L_M);
if(OUT_Z_H_M >= 0b10000000) {
  OUT_Z_H_M -= 0b10000000;
  z = ((OUT_Z_H_M << 8) + OUT_Z_L_M);
  z -= 32768;
}
int temp = ((TEMP_H_M << 8) + TEMP_L_M);
if(TEMP_H_M >= 0b10000000) {
  TEMP_H_M -= 0b10000000;
  temp = ((TEMP_H_M << 8) + TEMP_L_M);
  temp -= 32768;
}

float xgauss = (float)x/32768.0F * 16.0F * 1000;
float ygauss = (float)y/32768.0F * 16.0F * 1000;
float zgauss = (float)z/32768.0F * 16.0F * 1000;


SerialUSB.print("x mgauss");
SerialUSB.println(xgauss, 6);
SerialUSB.print("y mgauss: ");
SerialUSB.println(ygauss, 6);
SerialUSB.print("z mgauss: ");
SerialUSB.println(zgauss, 6);
SerialUSB.print("temp byte: ");
SerialUSB.println(temp, DEC);
float X_milliGauss = xgauss;
float Y_milliGauss = ygauss;
float Z_milliGauss = zgauss;

float heading, headingDegrees, headingFiltered, geo_magnetic_declination_deg;
geo_magnetic_declination_deg = 1.09; // for our location
  
  //Calculating Heading
  headingDegrees = atan2(Y_milliGauss, X_milliGauss)* 180/PI + geo_magnetic_declination_deg;  // heading in rad. 
  
  // Correcting when signs are reveresed or due to the addition of the geo_magnetic_declination_deg angle
  if(headingDegrees <0) headingDegrees += 2*180;
if(headingDegrees > 2*180) headingDegrees -= 2*180;
SerialUSB.print("heading: ");
SerialUSB.println(headingDegrees);
delay(2 *1000);
}



void set_register(unsigned char r, unsigned char v) {
    Wire.beginTransmission(compass_device);
    Wire.write(r);
    Wire.write(v);
    Wire.endTransmission();
}
