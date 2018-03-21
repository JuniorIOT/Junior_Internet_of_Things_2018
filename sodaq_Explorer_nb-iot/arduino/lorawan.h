//////////////////////////////////////////////////
// JuniorIOTchallenge routines for RN2483 for LoraWan
///////////////////////////////////////////////
void rn2483_init()
{
  DEBUG_STREAM.println("rn2483_init");
  loraSerial.begin(57600); //serial port to radio
  DEBUG_STREAM.println("  rn baudrate");
  //reset rn2483
  DEBUG_STREAM.println("  rn 12 reset");
  pinMode(LORA_RESET, OUTPUT);
  digitalWrite(LORA_RESET, LOW);
  delay(500);
  digitalWrite(LORA_RESET, HIGH);

  delay(100); //wait for the RN2xx3's startup message
  //DEBUG_STREAM.println("flush");
  //loraSerial.flush();

  //Autobaud the rn2483 module to 9600. The default would otherwise be 57600.
  DEBUG_STREAM.println("  rn autobaud");
  myLora.autobaud();

  //check communication with radio
  DEBUG_STREAM.println("  rn hweui");
  String hweui = myLora.hweui();
  while(hweui.length() != 16)
  {
    DEBUG_STREAM.println("  rn Communication with RN2xx3 unsuccessful. After programming unplug the usb cable and reconnect.");
    DEBUG_STREAM.println(hweui);
    delay(10000);
    hweui = myLora.hweui();
  }

  //print out the HWEUI so that we can register it via ttnctl  
  DEBUG_STREAM.print("  rn DevEUI: ");
  DEBUG_STREAM.print(myLora.hweui());
  DEBUG_STREAM.println(" (register this when using OTAA)");
  DEBUG_STREAM.print("  rn RN2xx3 firmware version: ");
  DEBUG_STREAM.println(myLora.sysver());

  //configure your keys and join the network
  DEBUG_STREAM.println("  rn Trying to join TTN");
  bool join_result = false;

  //ABP: initABP(String addr, String AppSKey, String NwkSKey);
  DEBUG_STREAM.println("  rn myLora.initABP");
  myLora.init(); // come back from radio2radio
  join_result = myLora.initABP(DEVADDR, APPSKEY, NWKSKEY);

  //OTAA: initOTAA(String AppEUI, String AppKey);
  //join_result = myLora.initOTAA("70B3D57ED00001A6", "A23C96EE13804963F8C2BD6285448198");

  while(!join_result)
  {
    DEBUG_STREAM.println("  rn Unable to join. Are your keys correct, and do you have TTN coverage?");
    
    delay(60000); //delay a minute before retry
    join_result = myLora.init();
  }
  DEBUG_STREAM.println("  rn Successfully joined TTN");
  

  // SF is not in the library yet - maybe add it
  DEBUG_STREAM.println("  rn Completed rn2483_init");
}

void print_myLoraWanData() {
  
  DEBUG_STREAM.print(F("                   ")); 
  for(int ii=0; ii<48; ii++) {  
    if (ii < 10) DEBUG_STREAM.print("0"); 
    DEBUG_STREAM.print(ii); 
    DEBUG_STREAM.print(F(" "));  
  }  
  DEBUG_STREAM.println(F(" "));
  
  DEBUG_STREAM.print(F("  myLoraWanData = [")); 
  for(int i=0; i<48; i++) {  
    if (myLoraWanData[i] < 16) DEBUG_STREAM.print("0"); 
    DEBUG_STREAM.print(myLoraWanData[i], HEX); 
    DEBUG_STREAM.print(F(" "));  
  }  
  DEBUG_STREAM.println(F(" ]"));
  
}

void doOneLoraWan() {
  DEBUG_STREAM.print("doOneLoraWan started. milis="); DEBUG_STREAM.println(millis());
  print_myLoraWanData();
    led_on();
    
    DEBUG_STREAM.print("  txLora. milis="); DEBUG_STREAM.println(millis());
    myLora.txBytes(myLoraWanData, PAYLOADSIZE);
    DEBUG_STREAM.print("  txLora completed. milis="); DEBUG_STREAM.println(millis());
    led_off();
    packagecounter++;
   DEBUG_STREAM.print("  send time delay completed. milis="); DEBUG_STREAM.println(millis());
  DEBUG_STREAM.print("  doOneLoraWan completed. milis="); DEBUG_STREAM.println(millis());
}

void resetSomeLoraValues() {
  /*for(int i = 18; i < PAYLOADSIZE; i++) {
    if(!(i == 33))myLoraWanData[i] = 0x00;
  }*/
}

