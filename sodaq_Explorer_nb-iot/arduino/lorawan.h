//////////////////////////////////////////////////
// Kaasfabriek routines for RN2483 for LoraWan
///////////////////////////////////////////////
void rn2483_init()
{
  loraSerial.begin(57600); //serial port to radio
  
  //reset rn2483
  pinMode(12, OUTPUT);
  digitalWrite(12, LOW);
  delay(500);
  digitalWrite(12, HIGH);

  delay(100); //wait for the RN2xx3's startup message
  loraSerial.flush();

  //Autobaud the rn2483 module to 9600. The default would otherwise be 57600.
  myLora.autobaud();

  //check communication with radio
  String hweui = myLora.hweui();
  while(hweui.length() != 16)
  {
    DEBUG_STREAM.println("Communication with RN2xx3 unsuccessful. Power cycle the board.");
    DEBUG_STREAM.println(hweui);
    delay(10000);
    hweui = myLora.hweui();
  }

  //print out the HWEUI so that we can register it via ttnctl
  DEBUG_STREAM.println("When using OTAA, register this DevEUI: ");
  DEBUG_STREAM.println(myLora.hweui());
  DEBUG_STREAM.println("RN2xx3 firmware version:");
  DEBUG_STREAM.println(myLora.sysver());

  //configure your keys and join the network
  DEBUG_STREAM.println("Trying to join TTN");
  bool join_result = false;

  //ABP: initABP(String addr, String AppSKey, String NwkSKey);
  myLora.init(); // come back from radio2radio
  join_result = myLora.initABP(DEVADDR, APPSKEY, NWKSKEY);

  //OTAA: initOTAA(String AppEUI, String AppKey);
  //join_result = myLora.initOTAA("70B3D57ED00001A6", "A23C96EE13804963F8C2BD6285448198");

  while(!join_result)
  {
    DEBUG_STREAM.println("Unable to join. Are your keys correct, and do you have TTN coverage?");
    
    delay(60000); //delay a minute before retry
    join_result = myLora.init();
  }
  DEBUG_STREAM.println("Successfully joined TTN");
  

  // SF is not in the library yet - maybe add it
  
}

void print_myLoraWanData() {
  DEBUG_STREAM.print(F("  myLoraWanData = [")); 
  //DEBUG_STREAM.print((char*)myLoraWanData); DEBUG_STREAM.println("]"); DEBUG_STREAM.print(F("                  [ "));  
  for(int i=0; i<30; i++) {  
    if (myLoraWanData[i] < 16) DEBUG_STREAM.print("0"); 
    DEBUG_STREAM.print(myLoraWanData[i], HEX); 
    DEBUG_STREAM.print(F(" "));  
  }  
  DEBUG_STREAM.println(F(" .. ]"));
}

void doOneLoraWan() {
  DEBUG_STREAM.print("\nStart: Do one lora. milis="); DEBUG_STREAM.println(millis());
  print_myLoraWanData();
    led_on();

    
    DEBUG_STREAM.print("  txLora. milis="); DEBUG_STREAM.println(millis());
    myLora.txBytes(myLoraWanData, PAYLOADSIZE);
    DEBUG_STREAM.print("  txLora completed. milis="); DEBUG_STREAM.println(millis());
    led_off();
    packagecounter++;
   DEBUG_STREAM.print("  send time delay completed. milis="); DEBUG_STREAM.println(millis());
  DEBUG_STREAM.print("Completed: Do one lora. milis="); DEBUG_STREAM.println(millis());
}


