//////////////////////////////////////////////////
// Kaasfabriek routines for RN2483 radio 2 radio
///////////////////////////////////////////////
void doOneRadio() {
  String received = "";
  DEBUG_STREAM.print("\nStart: Do one radio. milis="); DEBUG_STREAM.println(millis());  
  
  
  uint8_t radiopacket[radioPacketSize];
  for(int i = 0; i < radioPacketSize; i++) {
    radiopacket[i] = 0b00000000;
  }
  formatRadioPackage(&radiopacket[0]);
  
  DEBUG_STREAM.println("Sending..."); delay(10);
  

  switch(myLora.txBytes(radiopacket, radioPacketSize)) //one byte, blocking function
  {
    case TX_FAIL:
    {
      DEBUG_STREAM.println("TX unsuccessful or not acknowledged");
      break;
    }
    case TX_SUCCESS:
    {
      DEBUG_STREAM.println("TX successful and acknowledged");
      break;
    }
    case TX_WITH_RX:
    {
      received = myLora.getRx();
      DEBUG_STREAM.print("Received downlink immediately: " + received);
      
      ReceivedFromRadio = true;
      decoded = myLora.base16decodeBytes(received);
      SerialUSB.print("base16: ");
      for(int i = 0; i < radioPacketSize; i++) SerialUSB.print(decoded[i], HEX);
      SerialUSB.println(".");
      
      decodeReply();
      break;
    }
    default:
    {
      DEBUG_STREAM.println("Unknown response from TX function");
    }
  }  

  
  // end loop
  
  DEBUG_STREAM.print("\nCompleted: Do one radio. milis="); DEBUG_STREAM.println(millis());
  
  
}; 
void setupRadio() {
  myLora.autobaud();

  DEBUG_STREAM.println("DevEUI? ");DEBUG_STREAM.print(F("> "));
  DEBUG_STREAM.println(myLora.hweui());
  DEBUG_STREAM.println("Version?");DEBUG_STREAM.print(F("> "));
  DEBUG_STREAM.println(myLora.sysver());
  DEBUG_STREAM.println(F("--------------------------------"));

  DEBUG_STREAM.println(F("Setting up for listening for another explorer"));
  bool join_result = false;


  // point to point
  if(join_result = myLora.initP2P())  
  DEBUG_STREAM.println("\u2713 Successfully Activated radio 2 radio");
};

void listenRadio() {
  DEBUG_STREAM.print("\nStart: Listen radio. milis="); DEBUG_STREAM.println(millis());  
  switch(myLora.listenP2P()) {
    case TX_WITH_RX:
    {
      String received = myLora.getRx();
      DEBUG_STREAM.print("Received downlink: " + received);
      
      ReceivedFromRadio = true;
      decoded = myLora.base16decodeBytes(received);
      SerialUSB.print("base16: ");
      for(int i = 0; i < radioPacketSize; i++) SerialUSB.print(decoded[i], HEX);
      SerialUSB.println(".");
      
      decodeReply();
      
      break;
    }
    case RADIO_LISTEN_WITHOUT_RX:
    { 
      DEBUG_STREAM.println("Listened timeout but no downlink");
      break;
    }
    
 
  }  
  DEBUG_STREAM.print("\nCompleted: Listen radio. milis="); DEBUG_STREAM.println(millis());

}

/*
   byte 0          My ID      My ID and message type
        0b0000 0000            
          ---- nnnn MessType   
          ---- 0001 msg#1      Yelling out loud that I have fired
          ---- 0010 msg#2      You have fired and here is my answer 
          nnnn ---- MyID       
    byte 1, 2, 3    MyLat      
    byte 4, 5, 6    MyLon      
    byte 7          MyComp ++   
        0b0000 0000            
          -nnn nnnn MyComp     
          1--- ---- MyBtn#1       
    byte 8          RemoteID   Your ID, hey I am talkming to you
        0b0000 0000            
          ---- ---n WasIhit    Hit indicator
          nnnn ---- RemoteID   Value 0-31, Remote team ID
    byte 9          Validator  Hash (binary add) on message, GPS date, salt..
    */
    bool didIFire = false;
    bool didSomeoneElseFire = false;
    bool shouldITalkBack = false;
    uint8_t who = 0b00000000;    
    bool buttonpressedForLoraWan = false;
void formatRadioPackage(uint8_t *loopbackToData) {  
    
  uint8_t targetID = 0b00000000; // unknown
  
  
  if(didIFire) {    
    loopbackToData[0] = 0b00000001;
    loopbackToData[8] = targetID; // send unknown    
  } else if(didSomeoneElseFire && shouldITalkBack) {
    loopbackToData[0] = 0b00000010;
    SerialUSB.print("Target: ");
    SerialUSB.println(whoWasItThatTalkedToMe());
    loopbackToData[8] |= whoWasItThatTalkedToMe() << 4;
    SerialUSB.println(loopbackToData[8], BIN);
    loopbackToData[8] |= (myLoraWanData[33] >> 7) & 0b00000001;// better not call wasIHit(); because then it keeps repeating
    SerialUSB.println(loopbackToData[8], BIN);
  }
  loopbackToData[0] |= MyID << 4;
  SerialUSB.print("MyID: ");
  SerialUSB.println(loopbackToData[0], BIN);

  
  doGPS(60); // must have a gps - wait up to 10 seconds

  // maybe we should make a function for lat lng encoding that doesnt put them to lorawan
  const double shift_lat     =    90. * 10000000.;                 // range shift from -90..90 into 0..180, note: 
                                                                 //      NMEAGPS long lat&lon are degree values * 10.000.000
                                                                 //      TynyGPS long lat&lon are degree values * 1.000.000
  const double max_old_lat   =   180. * 10000000.;                 // max value for lat is now 180
  const double max_3byte     =         16777215.;                   // max value that fits in 3 bytes
  double lat_DOUBLE         = l_lat;                              // put 4byte LONG into a more precise floating point to prevent rounding during calcs 
  lat_DOUBLE = (lat_DOUBLE + shift_lat) * max_3byte / max_old_lat; // rescale into 3 byte integer range
  uint32_t LatitudeBinary  = lat_DOUBLE;                          // clips off anything after the decimal point    
  const double shift_lon     =   180. * 10000000.;                 // range shift from -180..180 into 0..360
  const double max_old_lon   = 360. * 10000000.;                   // max value longitude is now 360, note the value is too big for Long type
  double lon_DOUBLE = l_lon;                                      // put the 4byte LONG into a precise floating point memory space
  lon_DOUBLE = (lon_DOUBLE + shift_lon) * max_3byte / max_old_lon; // rescale into 3 byte integer range
  uint32_t LongitudeBinary = lon_DOUBLE;                          // clips off anything after the decimal point  

  loopbackToData[1] = ( LatitudeBinary >> 16 ) & 0xFF;
  loopbackToData[2] = ( LatitudeBinary >> 8 ) & 0xFF;
  loopbackToData[3] = LatitudeBinary & 0xFF;
  loopbackToData[4] = ( LongitudeBinary >> 16 ) & 0xFF;
  loopbackToData[5] = ( LongitudeBinary >> 8 ) & 0xFF;
  loopbackToData[6] = LongitudeBinary & 0xFF;

  // maybe we should make a function for compass encoding that doesnt put them to lorawan
  unsigned int compass = readCompass(); // 0..360 deg
  uint8_t compass_bin = compass/3 ;  // rescale 0-360 deg into 0 - 120 values and make sure it is not bigger than one byte
  // now add a bit for BTN (not implemented)
  loopbackToData[7] = compass_bin;
  #ifdef DEBUG
  DEBUG_STREAM.print(F("  compass=")); DEBUG_STREAM.print(compass); DEBUG_STREAM.print(F("  deg. compass_bin=")); DEBUG_STREAM.println(compass_bin);
  #endif

  loopbackToData[7] |= buttonPressed;

  loopbackToData[9] = 0x00; // What is this?
  // now add radiohead compatibility
  for(int i = (radioPacketSize-4) - 1; i >= 0; i--) loopbackToData[i+4] = loopbackToData[i];
  loopbackToData[0]=255; // to all
  loopbackToData[1]=MyID; // from
  loopbackToData[2]=packagecounter%0xFF; // headerid
  loopbackToData[3]=0x00; // flags
}
/*
   byte 0          My ID      My ID and message type
        0b0000 0000            
          ---- nnnn MessType   
          ---- 0001 msg#1      Yelling out loud that I have fired
          ---- 0010 msg#2      You have fired and here is my answer 
          nnnn ---- MyID       
    byte 1, 2, 3    MyLat      
    byte 4, 5, 6    MyLon      
    byte 7          MyComp ++   
        0b0000 0000            
          -nnn nnnn MyComp     
          1--- ---- MyBtn#1       
    byte 8          RemoteID   Your ID, hey I am talkming to you
        0b0000 0000            
          ---- ---n WasIhit    Hit indicator
          nnnn ---- RemoteID   Value 0-31, Remote team ID
    byte 9          Validator  Hash (binary add) on message, GPS date, salt..
    */
    bool therewasaradioreceived = false;
void decodeReply() {
  // radiohead compatibility
  for(int i = 0; i < (radioPacketSize-4); i++) decoded[i] = decoded[i+4];
  
  therewasaradioreceived = true;
  SerialUSB.print("BUF HEX: ");
  for(byte b=0; b<10; b++)
   {
      SerialUSB.print(decoded[b], HEX);
   }
   SerialUSB.println(".");
  bool someoneIsTalkingBackToSomeoneWhoFired = false;
    
  // bytes 0
  if((decoded[0] & 0b00001111) == 0b00000001) {
    DEBUG_STREAM.println("Radio: Someone says that he fired");
    didSomeoneElseFire = true;      
  } else if((decoded[0] & 0b00001111) == 0b00000010) {
      DEBUG_STREAM.println("Radio: Someone talkes back to someone who fired");
      didSomeoneElseFire = false;
      // did i fire and is he talking to me
      someoneIsTalkingBackToSomeoneWhoFired = true; 
  } else{
    didSomeoneElseFire = false;
  }
  DEBUG_STREAM.print("That someone has an id of:");
  SerialUSB.print("MyID: ");
  SerialUSB.println(decoded[0], BIN);
  who = (decoded[0] >> 4) & 0b00001111;
  DEBUG_STREAM.println((unsigned int) who,DEC);
  
  // byte 1,2,3 and 4,5,6
  DEBUG_STREAM.print("His location is: ");

  // tell lorawan
  myLoraWanData[26] = decoded[1];
  myLoraWanData[27] = decoded[2];
  myLoraWanData[28] = decoded[3];
  myLoraWanData[29] = decoded[4];
  myLoraWanData[30] = decoded[5];
  myLoraWanData[31] = decoded[6];
  
  float _lat = ((((uint32_t)decoded[1]) << 16) + (((uint32_t)decoded[2]) << 8) + decoded[3]) / 16777215.0 * 180.0 - 90;
  float _lng = ((((uint32_t)decoded[4]) << 16) + (((uint32_t)decoded[5]) << 8) + decoded[6]) / 16777215.0 * 360.0 - 180;
  DEBUG_STREAM.print("lat: ");
  DEBUG_STREAM.print(_lat, 6);
  DEBUG_STREAM.print("lng: ");
  DEBUG_STREAM.println(_lng,6);
  hitlat2 = _lat;
  hitlng2 = _lng;
  

  
  // byte 7
  myLoraWanData[32] = decoded[7];
  uint8_t compass = decoded[7] & 0b01111111; // don't want the hit indicator now    
  DEBUG_STREAM.print("His compass points to: ");
  unsigned int _compass = compass*3;
  DEBUG_STREAM.println(_compass);
  hitcompass = _compass;
  
  bool hePressedHisButton = ((decoded[7] >> 7) & 0b00000001) == 0b00000001;
  if(hePressedHisButton) DEBUG_STREAM.println("He pressed his button");
  else DEBUG_STREAM.println("He did not press his button");
  
  // byte 8
  bool heWasHit = (decoded[8] & 0b00000001) == 0b00000001;
  if(heWasHit) DEBUG_STREAM.println("He was hit");
  else DEBUG_STREAM.println("He was not hit - or doesn't know it yet");
  
  uint8_t remoteid = (decoded[8] >> 4) & 0b00001111;
  DEBUG_STREAM.println("He was talking to id: ");
  DEBUG_STREAM.print((int)remoteid,DEC);

  myLoraWanData[25] = remoteid << 4;
  
  // byte 9 - what is this?
  
  if(didSomeoneElseFire) {
    SerialUSB.println("Checking was i hit on receive");
    shouldITalkBack = wasIHit();      
  }

  SerialUSB.println("someoneIsTalkingBackToSomeoneWhoFired?");
  if(someoneIsTalkingBackToSomeoneWhoFired && (negotiateState == 1)) {
    SerialUSB.println("someoneIsTalkingBackToSomeoneWhoFired!");
    SerialUSB.print("remoteid: ");
    SerialUSB.println(remoteid);
    SerialUSB.print("MyID: ");
    SerialUSB.println(MyID & 0b00001111);
    if((remoteid&0b00001111) == (MyID&0b00001111)) {
      // i fired and i hit!
      IHitSomeone();
    }
  }
  free(decoded);
}
