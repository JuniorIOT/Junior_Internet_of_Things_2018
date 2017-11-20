#define loraSerial Serial2

String str;


void setup() {
  
  SerialUSB.begin(9600);
  
  loraSerial.begin(9600);
  loraSerial.setTimeout(1000);
  lora_autobaud();
  
  delay(1000);
  
  SerialUSB.println("Initing LoRa");

  
  str = loraSerial.readStringUntil('\n');
  SerialUSB.println(str);
  loraSerial.println("sys get ver");
  str = loraSerial.readStringUntil('\n');
  SerialUSB.println(str);

  loraSerial.println("mac pause");
  str = loraSerial.readStringUntil('\n');
  SerialUSB.println(str);

  loraSerial.println("radio set mod lora");
  str = loraSerial.readStringUntil('\n');
  SerialUSB.println(str);
  
  loraSerial.println("radio set freq 869100000");
  str = loraSerial.readStringUntil('\n');
  SerialUSB.println(str);
  
  loraSerial.println("radio set pwr 14");
  str = loraSerial.readStringUntil('\n');
  SerialUSB.println(str);

  loraSerial.println("radio set sf sf7");
  str = loraSerial.readStringUntil('\n');
  SerialUSB.println(str);
  
  loraSerial.println("radio set afcbw 41.7");
  str = loraSerial.readStringUntil('\n');
  SerialUSB.println(str);
  
  loraSerial.println("radio set rxbw 125");
  str = loraSerial.readStringUntil('\n');
  SerialUSB.println(str);

  loraSerial.println("radio set prlen 8");
  str = loraSerial.readStringUntil('\n');
  SerialUSB.println(str);
  
  loraSerial.println("radio set crc on");
  str = loraSerial.readStringUntil('\n');
  SerialUSB.println(str);
  
  loraSerial.println("radio set iqi off");
  str = loraSerial.readStringUntil('\n');
  SerialUSB.println(str);
  
  loraSerial.println("radio set cr 4/5");
  str = loraSerial.readStringUntil('\n');
  SerialUSB.println(str);
  
  loraSerial.println("radio set wdt 60000"); //disable for continuous reception
  str = loraSerial.readStringUntil('\n');
  SerialUSB.println(str);
  
  loraSerial.println("radio set sync 12");
  str = loraSerial.readStringUntil('\n');
  SerialUSB.println(str);
  
  loraSerial.println("radio set bw 125");
  str = loraSerial.readStringUntil('\n');
  SerialUSB.println(str);

  SerialUSB.println("starting loop");

}

void loop() {
  loraSerial.println("radio tx 20");
  str = loraSerial.readStringUntil('\n');
  SerialUSB.println(str);
  str = loraSerial.readStringUntil('\n');
  SerialUSB.println(str);
  delay(200);

}

void lora_autobaud()
{
  String response = "";
  while (response=="")
  {
    delay(1000);
    loraSerial.write((byte)0x00);
    loraSerial.write(0x55);
    loraSerial.println();
    loraSerial.println("sys get ver");
    response = loraSerial.readStringUntil('\n');
  }
}

/*
 * This function blocks until the word "ok\n" is received on the UART,
 * or until a timeout of 3*5 seconds.
 */
int wait_for_ok()
{
  str = loraSerial.readStringUntil('\n');
  if ( str.indexOf("ok") == 0 ) {
    return 1;
  }
  else return 0;
}


