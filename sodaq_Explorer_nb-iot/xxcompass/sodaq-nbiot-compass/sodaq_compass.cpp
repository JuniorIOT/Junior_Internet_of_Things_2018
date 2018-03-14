#include "Arduino.h"
#include "sodaq_compass.h"
#include <Wire.h>

NBIOT_Compass::NBIOT_Compass()
{
  _address = (0x1E);
  _tries = 0;
  _headingFiltered = 0;
}

void NBIOT_Compass::setup() {
  Wire.begin();
  Wire.beginTransmission(_address);    // Get the slave's attention, tell it we're sending a command byte
  Wire.write(0x0F);                               //  The command byte, sets pointer to register with address of 0x32
  Wire.endTransmission();
  Wire.requestFrom(_address,1);          // Tell slave we need to read 1byte from the current register
  byte slaveByte2 = Wire.read();        // read that byte into 'slaveByte2' variable
  Wire.endTransmission();
    /*SerialUSB.print("Device is: ");
    SerialUSB.println(slaveByte2, HEX);
  ;*/
  //CTRL_REG1_M
  /*SerialUSB.println("Writing [temp on] [xy low power] [xy low power] [10hz] [10hz] [10h] [self test off] [0]");
  SerialUSB.println("0b10010000");
  SerialUSB.println("To address 0x20 CTRL_REG1_M");*/
  //[temp off] [xy low power] [xy low power] [10hz] [10hz] [10h] [self test off] [0] 
  set_register(0x20, 0b00010000); 
  
  Wire.beginTransmission(_address);    // Get the slave's attention, tell it we're sending a command byte
  Wire.write(0x20);                               //  The command byte, sets pointer to register with address of 0x32
  Wire.endTransmission();
  Wire.requestFrom(_address,1);          // Tell slave we need to read 1byte from the current register
  slaveByte2 = Wire.read();        // read that byte into 'slaveByte2' variable
  /*
    SerialUSB.print("Register 0x20 is: ");
    SerialUSB.println(slaveByte2, HEX);
  ;
  
  SerialUSB.println("Writing to address 0x21 CTRL_REG2_M");*/
  // CTRL_REG2_M
  // [always 0] [16 guass full scale used] [16 guass full scale used] [always 0] [do not reboot memory] [do not soft reset] [always 0] [always 0]
  set_register(0x21, 0b01100000); 
  /*SerialUSB.println("[always 0] [16 guass full scale used] [16 guass full scale used] [always 0] [do not reboot memory] [do not soft reset] [always 0] [always 0]");
  SerialUSB.println("0b01100000");
  ;*/
  
  Wire.beginTransmission(_address);    // Get the slave's attention, tell it we're sending a command byte
  Wire.write(0x21);                               //  The command byte, sets pointer to register with address of 0x32
  Wire.endTransmission();
  Wire.requestFrom(_address,1);          // Tell slave we need to read 1byte from the current register
  slaveByte2 = Wire.read();        // read that byte into 'slaveByte2' variable
  
    /*SerialUSB.print("Register 0x21 is: ");
    SerialUSB.println(slaveByte2, HEX);
  ;*/
  
  //CTRL_REG3_M
  /*SerialUSB.println("Writing to address 0x22 CTRL_REG3_M");
  SerialUSB.println("[i2c enable] [always 0] [low power] [always 0] [always 0][spi only write - but not using this] [continues conversion][continues conversion]");
  SerialUSB.println("0b00000000");
  */
  set_register(0x22, 0b00000000);
  ;
  Wire.beginTransmission(_address);    // Get the slave's attention, tell it we're sending a command byte
  Wire.write(0x22);                               //  The command byte, sets pointer to register with address of 0x32
  Wire.endTransmission();
  Wire.requestFrom(_address,1);          // Tell slave we need to read 1byte from the current register
  slaveByte2 = Wire.read();        // read that byte into 'slaveByte2' variable
  
    /*SerialUSB.print("Register 0x22 is: ");
    SerialUSB.println(slaveByte2, HEX);
  ;*/
  
  // CTRL_REG4_M
  /*SerialUSB.println("Writing to address 0x23 CTRL_REG4_M");
  SerialUSB.println("[always 0] [always 0] [always 0] [always 0] [z low power] [z low power] [data LSb at lower address] [always 0]");*/
  set_register(0x23, 0b00000000);
  
  Wire.beginTransmission(_address);    // Get the slave's attention, tell it we're sending a command byte
  Wire.write(0x23);                               //  The command byte, sets pointer to register with address of 0x32
  Wire.endTransmission();
  Wire.requestFrom(_address,1);          // Tell slave we need to read 1byte from the current register
  slaveByte2 = Wire.read();        // read that byte into 'slaveByte2' variable
  
  /*  SerialUSB.print("Register 0x23 is: ");
    SerialUSB.println(slaveByte2, HEX);
  ;*/
  
  // CTRL_REG5_M
  /*SerialUSB.println("Writing to address 0x24 CTRL_REG5_M");
  SerialUSB.println("[always 0] [after byte read]"); 
  SerialUSB.println("0b01000000");*/
  set_register(0x24, 0b01000000);
  
  Wire.beginTransmission(_address);    // Get the slave's attention, tell it we're sending a command byte
  Wire.write(0x24);                               //  The command byte, sets pointer to register with address of 0x32
  Wire.endTransmission();
  Wire.requestFrom(_address,1);          // Tell slave we need to read 1byte from the current register
  slaveByte2 = Wire.read();        // read that byte into 'slaveByte2' variable
  
  /*  SerialUSB.print("Register 0x24 is: ");
    SerialUSB.println(slaveByte2, HEX);
  ;*/


}

void NBIOT_Compass::getNewValues() {
  setup();
}

float NBIOT_Compass::getXGauss() {
  Wire.endTransmission();
  Wire.flush();
  Wire.begin();
  // OUT_X_L_M 28
  // OUT_X_H_M 29
  Wire.beginTransmission(_address);    // Get the slave's attention, tell it we're sending a command byte
  Wire.write(0x28);                               //  The command byte, sets pointer to register with address of 0x32
  Wire.endTransmission();
  Wire.requestFrom(_address,1);          // Tell slave we need to read 1byte from the current register
  byte OUT_X_L_M = Wire.read();        // read that byte into 'slaveByte2' variable
  Wire.beginTransmission(_address);    // Get the slave's attention, tell it we're sending a command byte
  Wire.write(0x29);                               //  The command byte, sets pointer to register with address of 0x32
  Wire.endTransmission();
  Wire.requestFrom(_address,1);          // Tell slave we need to read 1byte from the current register
  uint16_t OUT_X_H_M = Wire.read();        // read that byte into 'slaveByte2' variable
  _xbytes = ((OUT_X_H_M << 8) | OUT_X_L_M);
  _xguass = ((float)_xbytes) /** 0.58/1000*/;
  return _xguass;
}

float NBIOT_Compass::getYGauss() {
  // OUT_Y_L_M 2A
  // OUT_Y_H_M 2B
  Wire.beginTransmission(_address);    // Get the slave's attention, tell it we're sending a command byte
  Wire.write(0x2A);                               //  The command byte, sets pointer to register with address of 0x32
  Wire.endTransmission();
  Wire.requestFrom(_address,1);          // Tell slave we need to read 1byte from the current register
  byte OUT_Y_L_M = Wire.read();        // read that byte into 'slaveByte2' variable
  Wire.beginTransmission(_address);    // Get the slave's attention, tell it we're sending a command byte
  Wire.write(0x2B);                               //  The command byte, sets pointer to register with address of 0x32
  Wire.endTransmission();
  Wire.requestFrom(_address,1);          // Tell slave we need to read 1byte from the current register
  uint16_t OUT_Y_H_M = Wire.read();        // read that byte into 'slaveByte2' variable
  _ybytes = ((OUT_Y_H_M << 8) | OUT_Y_L_M);
  _yguass = ((float)_ybytes) /** 0.58/1000*/;
  return _yguass;
}

float NBIOT_Compass::getZGauss() {
  // OUT_Z_L_M 2C
  // OUT_Z_H_M 2D
  Wire.beginTransmission(_address);    // Get the slave's attention, tell it we're sending a command byte
  Wire.write(0x2C);                               //  The command byte, sets pointer to register with address of 0x32
  Wire.endTransmission();
  Wire.requestFrom(_address,1);          // Tell slave we need to read 1byte from the current register
  byte OUT_Z_L_M = Wire.read();        // read that byte into 'slaveByte2' variable
  Wire.beginTransmission(_address);    // Get the slave's attention, tell it we're sending a command byte
  Wire.write(0x2D);                               //  The command byte, sets pointer to register with address of 0x32
  Wire.endTransmission();
  Wire.requestFrom(_address,1);          // Tell slave we need to read 1byte from the current register
  uint16_t OUT_Z_H_M = Wire.read();        // read that byte into 'slaveByte2' variable
  _zbytes = ((OUT_Z_H_M << 8) | OUT_Z_L_M);
  _zguass = ((float)_zbytes) /** 0.58/1000*/;
  return _zguass;
}

//SerialUSB.println("0.25–0.60 gauss – the Earth's magnetic field at its surface");
void NBIOT_Compass::set_register(unsigned char r, unsigned char v) {
    Wire.beginTransmission(_address);
    Wire.write(r);
    Wire.write(v);
    Wire.endTransmission();
}
