/*******************************************************************************
 * Modified By DenniZr & Marco van Schagen for Junior IOT - Smart City Challenge 2018 #juniorIOTchallenge2018
 * Modified By Marco van Schagen for Junior IOT Challenge 2018
 *******************************************************************************/ 
 
//#define DEBUG     // if DEBUG is defined, some code is added to display some basic debug info


// lorawan
#include <rn2xx3.h>
#include "keys.h"
#define loraSerial Serial2
#define PAYLOADSIZE 12 // The size of the package to be sent
//create an instance of the rn2xx3 library,
//giving the software serial as port to use
rn2xx3 myLora(loraSerial);
#define LEDPIN LED_BUILTIN
#define LORAWAN_TX_INTERVAL 240  // seconds between LoraWan messages
uint8_t  myLoraWanData[40];  // including byte[0]
unsigned long last_lora_time = millis(); // last time lorawan ran

//////////////////////////////////////////////////
// Kaasfabriek routines for RN2483 for LoraWan
///////////////////////////////////////////////
void rn2483_init();
void doOneLoraWan();
///////////////////////////////////////////////
//  arduino init and main
///////////////////////////////////////////
void setup();
void loop();


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
    SerialUSB.println("Communication with RN2xx3 unsuccessful. Power cycle the board.");
    SerialUSB.println(hweui);
    delay(10000);
    hweui = myLora.hweui();
  }

  //print out the HWEUI so that we can register it via ttnctl
  SerialUSB.println("When using OTAA, register this DevEUI: ");
  SerialUSB.println(myLora.hweui());
  SerialUSB.println("RN2xx3 firmware version:");
  SerialUSB.println(myLora.sysver());

  //configure your keys and join the network
  SerialUSB.println("Trying to join TTN");
  bool join_result = false;

  //ABP: initABP(String addr, String AppSKey, String NwkSKey);
  join_result = myLora.initABP(DEVADDR, APPSKEY, NWKSKEY);

  //OTAA: initOTAA(String AppEUI, String AppKey);
  //join_result = myLora.initOTAA("70B3D57ED00001A6", "A23C96EE13804963F8C2BD6285448198");

  while(!join_result)
  {
    SerialUSB.println("Unable to join. Are your keys correct, and do you have TTN coverage?");
    RED();
    delay(60000); //delay a minute before retry
    join_result = myLora.init();
  }
  SerialUSB.println("Successfully joined TTN");
  GREEN();

  // SF is not in the library yet - maybe add it
  
}

void print_myLoraWanData() {
  SerialUSB.print(F("  myLoraWanData = [")); 
  //SerialUSB.print((char*)myLoraWanData); SerialUSB.println("]"); SerialUSB.print(F("                  [ "));  
  for(int i=0; i<30; i++) {  
    if (myLoraWanData[i] < 16) SerialUSB.print("0"); 
    SerialUSB.print(myLoraWanData[i], HEX); 
    SerialUSB.print(F(" "));  
  }  
  SerialUSB.println(F(" .. ]"));
}

void doOneLoraWan() {
  SerialUSB.print("\nStart: Do one lora. milis="); SerialUSB.println(millis());
  print_myLoraWanData();
    led_on();

    
    SerialUSB.print("  txLora. milis="); SerialUSB.println(millis());
    myLora.txBytes(myLoraWanData, PAYLOADSIZE);
    SerialUSB.print("  txLora completed. milis="); SerialUSB.println(millis());
    led_off();
   SerialUSB.print("  send time delay completed. milis="); SerialUSB.println(millis());
  SerialUSB.print("Completed: Do one lora. milis="); SerialUSB.println(millis());
}


///////////////////////////////////////////////
//  arduino init and main
///////////////////////////////////////////

unsigned long device_startTime;
bool has_sent_allready = false;
int16_t packetnum = 0;  // packet counter, we increment per transmission
bool ReceivedFromRadio = false;
// radio buf
uint8_t buf[40];

void doOneRadio() {}; // not yet implemented
void setupRadio() {}; // not yet implemented

void setup() {
  pinMode(LEDPIN, OUTPUT);
  delay(1000);  // https://www.thethingsnetwork.org/forum/t/got-adafruit-feather-32u4-lora-radio-to-work-and-here-is-how/6863
  
  SerialUSB.begin(115200);   // whether 9600 or 115200; the gps feed shows repeated char and cannot be interpreted, setting high value to release system time
  delay(100);

  SerialUSB.print(F("\nStarting device: ")); SerialUSB.println(DEVADDR); 
  device_startTime = millis();

  //todo   gps_init(); 
  rn2483_init();

  SerialUSB.print(F("\nInit values. milis=")); SerialUSB.println(millis());
  /*TODO
   * put_Volts_and_Temp_into_sendbuffer();
  put_Compass_and_Btn_into_sendbuffer();
  doGPS_and_put_values_into_sendbuffer();   
*/
  SerialUSB.print(F("\nSend one lorawan message as part of system init. milis=")); SerialUSB.println(millis());

  last_lora_time = millis();
  doOneLoraWan();
  SerialUSB.print(F("\nCompleted: Setup. milis=")); SerialUSB.println(millis());
  
}
boolean radioActive = false;  // this name is for radio, not LoraWan
boolean loraWannaBe = false;

void loop() {
  SerialUSB.print(F("\n==== Loop starts. milis=")); SerialUSB.println(millis());
  ////// GPS pre-loop //////////////
  // SerialUSB.println(F("\nNo lengthy GPS read-till-fix is needed, the GPS will find/keep a fix as log as power is on. "));

  ////////// Radio  ///////////
  SerialUSB.print(F("\nRadio listen? milis=")); SerialUSB.println(millis());
  // now listen a long time for a radio message which we may want to act on, or for a keypress on our side 
  // time needs to be long enough not to miss a radio, we do not worry about GPS as it will keep fix as long as powered
  if(radioActive) {
    //TODO:
//    setupRadio();
//while((millis() - last_lora_time) < (LORAWAN_TX_INTERVAL * 1000L)) {
    // next command is not what we want to do
    //doOneRadio();  // sends a radio message and will listen for return message for a certain time
    
    if(ReceivedFromRadio) {
      // use the radio message content for Lora
      memcpy(myLoraWanData,buf,PAYLOADSIZE);
      ReceivedFromRadio = false;
    } else {
      //sprintf(myLoraWanData,"xx geen radio ontvangen xx");
    }  
  } else {
    //not listening to radio at all, we may as well use delay for a bit 
    SerialUSB.print(F("  No radio listen required, so instead just add a delay before lorawan: \n    ")); SerialUSB.print(LORAWAN_TX_INTERVAL); SerialUSB.print(F(" sec."));
    while((millis() - last_lora_time) < (LORAWAN_TX_INTERVAL * 1000L)) {
      delay(5000);   
      SerialUSB.print(F("."));
    }
    SerialUSB.println();
  }
  // we keep doing this part until it is time to send one LORAWAN TX to the worl

  ////////// Collect data needed just before sending a LORAWAN update to the world  ///////////
  SerialUSB.println(F("\nCollect data needed just before sending a LORAWAN update. milis=")); SerialUSB.println(millis());
  
  /*
   * TODO
   * put_Volts_and_Temp_into_sendbuffer();
  put_Compass_and_Btn_into_sendbuffer();
  doGPS_and_put_values_into_sendbuffer();
  */
  
  ////////// Now we need to send a LORAWAN update to the world  ///////////
  // switch the LMIC antenna to LoraWan mode
  SerialUSB.println(F("Time or button press tells us to send one LoraWan. milis=")); SerialUSB.println(millis());
  last_lora_time = millis();
  rn2483_init();
  doOneLoraWan();    
  
  /////////// Loop again  //////////////
  SerialUSB.println(F("\nEnd of loop. milis=")); SerialUSB.println(millis());
}

// Leds
void led_on()
{
  digitalWrite(LEDPIN, 1);
}

void led_off()
{
  digitalWrite(LEDPIN, 0);
}

void RED() {
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);
}

void GREEN() {
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_BLUE, HIGH);
}

void BLUE() {
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, LOW);
}
