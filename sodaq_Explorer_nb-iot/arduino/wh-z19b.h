//////////////////////////////////////////////////////////
//// JuniorIOTchallenge routines for MH-Z19B 0-5000ppm
////////////////////////////////////////////


SoftwareSerial co2_serial(pin_co2_TXD_rx, pin_co2_RXD_tx); // RX, TX  --> samd SoftwareSerial werkt niet op Sodaq Explorer







uint8_t getCheckSum(uint8_t *packet) {
  byte i;
  unsigned char checksum = 0;
  for (i = 1; i < 8; i++) {
    checksum += packet[i];
  }
  checksum = 0xff - checksum;
  checksum += 1;
  return checksum;
}

int readCO2PWM() {
  unsigned long th, tl, ppm_pwm = 0;
  do {
    th = pulseIn(9, HIGH, 1004000) / 1000;
    tl = 1004 - th;
    ppm_pwm = 5000 * (th-2)/(th+tl-4);
  } while (th == 0);
  DEBUG_STREAM.print("PPM PWM: ");
  DEBUG_STREAM.println(ppm_pwm);
  return ppm_pwm;  
}

 
//void zzloop() {
//  DEBUG_STREAM.println("------------------------------");
//  DEBUG_STREAM.print("Time from start: ");
//  DEBUG_STREAM.print((millis() - startTime) / 1000);
//  DEBUG_STREAM.println(" s");
//  int ppm_uart = readco2_ppm_serial();
//  int ppm_pwm = readCO2PWM();
//  delay(5000);
//}

int readco2_ppm_serial(){
  uint8_t cmd[9] = {0xFF,0x01,0x86,0x00,0x00,0x00,0x00,0x00,0x79};
  uint8_t response[9]; // for answer

  DEBUG_STREAM.println("Sending CO2 request...");
  co2_serial.write(cmd, 9); //request PPM CO2

  // clear the buffer
  memset(response, 0, 9);
  int i = 0;
  while (co2_serial.available() == 0) {
    DEBUG_STREAM.print("Waiting for response ");
    DEBUG_STREAM.print(i);
    DEBUG_STREAM.println(" s");
    delay(1000);
    i++;
  }
  if (co2_serial.available() > 0) {
      co2_serial.readBytes(response, 9);
  }
  // print out the response in hexa
  for (int i = 0; i < 9; i++) {
    DEBUG_STREAM.print(String(response[i], HEX));
    DEBUG_STREAM.print("   ");
  }
  DEBUG_STREAM.println("");

  // checksum
  uint8_t check = getCheckSum(response);
  if (response[8] != check) {
    DEBUG_STREAM.println("Checksum not OK!");
    DEBUG_STREAM.print("Received: ");
    DEBUG_STREAM.println(response[8]);
    DEBUG_STREAM.print("Should be: ");
    DEBUG_STREAM.println(check);
  }
  
  // ppm
  int co2_ppm_serial = 256 * (int)response[2] + response[3];
  DEBUG_STREAM.print("PPM co2_ppm_serial: ");
  DEBUG_STREAM.println(co2_ppm_serial);

  // temp
  uint8_t temp = response[4] - 40;
  DEBUG_STREAM.print("Temperature? ");
  DEBUG_STREAM.println(temp);

  // status
  uint8_t status = response[5];
  DEBUG_STREAM.print("Status? ");
  DEBUG_STREAM.println(status); 
  if (status == 0x40) {
    DEBUG_STREAM.println("Status OK"); 
  }
  
  return co2_ppm_serial;
}








void setup_co2()
{
  DEBUG_STREAM.println("  co2 Setup sensor");
  co2_serial.begin(9600);         
}

void co2_measure()
{
  DEBUG_STREAM.println("  co2_measure started.");
  while(1) {
    
    int ppm_uart = readco2_ppm_serial();
    delay(5000);
  }
}

void put_co2_into_sendbuffer() {
  DEBUG_STREAM.print(F("put_co2_into_sendbuffer started. milis=")); DEBUG_STREAM.println(millis());
//  // pm25_bin and pm10_bin are 2 byte values
//  //   byte 40, 41     PPM 2.5    2 bytes, AD measurement directly from AD port put_PM_into_sendbuffer
//  //   byte 42, 43     PPM 10     2 bytes, AD measurement directly from AD port
//  
//  myLoraWanData[40] = ( pm25_bin >> 8 ) & 0xFF;
//  myLoraWanData[41] = pm25_bin & 0xFF;
//  
//  myLoraWanData[42] = ( pm10_bin >> 8 ) & 0xFF;
//  myLoraWanData[43] = pm10_bin & 0xFF;
//  
//  #ifdef DEBUG
//  DEBUG_STREAM.print(F("  pm25_bin=")); DEBUG_STREAM.print(pm25_bin); DEBUG_STREAM.print(F("  pm10_bin=")); DEBUG_STREAM.println(pm10_bin);
//  #endif
}




