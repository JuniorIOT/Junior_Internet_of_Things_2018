
# Junior IOT - Smart City Challenge 2018 - Fablab de Kaasfabriek
Inexpensive low-quality Lora32u4 GPS tracker for #juniorIOTchallenge 2018

## Inexpensive low-quality Lora32u4 GPS tracker
```
    this set of folders is for #JuniorIOTchallenge - Smart City Challenge 2018
             met leerling teams bij fablab de Kaasfabriek in Alkmaar
                     deze software op het internet is natuurlijk geheim...
                                  ...anders kunnen ze makkelijk afkijken!
                                  
  previous iteration is Junior IoT Baloon Challenge february 2017
  so for ora location transmittor with GPS and rfm95 see 
  https://github.com/Kaasfabriek/GPS-Lora-Balloon-rfm95-TinyGPS/tree/master/Balloon-rfm95


```
Credits:
- Software plakker: Dennis --> dennis.ruigrok@gmail.com
- Educatie kletser: Marco --> marco@kaasfabriek.nl
- Regie en inspiratie: marco@marcovanschagen.nl
- Junior IOT Challenges: marco@dataschrift.nl

## Important
Some practical soldering tips in nice pictures:
https://www.thethingsnetwork.org/labs/story/build-the-cheapest-possible-node-yourself
 --> you will need to use our pin mapping instead

## Parts list
- Lora32u4 with antenna, frequency 868 Mhz - 12 euro on Ebay and AliExpress
optional: 
- GPS
- Compass
- Solar Powerbank + lipo protect strip
- Lipo 380 mAh

## Important files and folders
- juniorIOTchallenge_Lora32u4_gpsTracker_with_extras (folder) 
   this is the GPS tracker, with demo code added got compass and peer-to-peer radio
   
## Libraries required
- tbd1       https://github.com/tbd1
- tb21       https://github.com/tbd2

## Pin mapping for Lora32u4
```
    -----------------------------------------------------------------------------

                                  2A usb charge cable
      ┌────────────────────────────────────────┐  │   one or more I2c modules:
      │ solar      ┌──────────────────────────┐│  │  ┌───────────────┐
      │ powerbank  │ controll board    (Mini USB)─┘  │ Oled display  │
      │ enclosure  │     B- B+   S- S+        ││     │Vin GND SCL SDA│
      │            └──────┼─┼────┼─┼──────────┘│     └─┬───┬───┬───┬─┘
      │┌────────────────┐ │ │  ┌─│─│─────────┐ │       │   │   │I2c│chained 
      ││ 1-5x 1800mAh  +┼─│─┤  │ - +  200 mA │ │     ┌────────────────────┐
      ││ Li-Ion 18650 - ┼─┤ │  │  solar panel│ │     │BME/BMP280 or BME680│
      │└────────────────┘ │ │  └─────────────┘ │     │Vin GND SCL SDA     │
      └───────────────────│─│──────────────────┘     └─┬───┬───┬───┬──────┘
   ┌──────────────────────┼─┼──┐                       │   │   │I2c│chained 
   │ Lipo protect    - + B- B+ │                     ┌────────────────────────┐
   └─────────────────┼─┼───────┘                     │ GY-91 (BMP280+MPU9250) │
                     │ │ ┌───────────────────────┐   │Vin GND SCL SDA 3 SD A B│
                     │ │ │       GPS GN-801      │   └─┬───┬───┬───┬──┬─┬──┬─┬┘
                     │ │ │or BN-180/BN-200/BN-220│     │   │   │I2c│chained
                     │ │ │        3V3 GND RXD TXD│   ┌────────────────────────┐
       ┌─────────┐   │ │ └─┬───┬───┬───┬───┬───┬─┘   │    compass HMC5983     │
       │   LiPo  │   ext          3V3 GND  │   │     │Vin GND SCL SDA CS SD DR│
       │ 380 mAh │   - +        <50mA      │   │     └─┬───┬───┬───┬──┬──┬──┬─┘         
       │protected│   │ │                   │   │       │   │   │   │         
       └────┬──┬─┘ ┌─┘ │                 tx│ rx│ Vbat/2│   │   │I2c│        ant
   ╔════════│══│═══│═══╬═══X═══╬═══X═══X═══╬═══╬═══╬═══X═══X═══╬═══╬═══════╗ │
   ║        -  +   │  BAT EN  5V  13  12  11  10   9   6   5   3   2       ║ │
   ║    (LIPO CONN)│             LED  A1      A10  A9  A7     SCL SDA      ║ │
   │               │           ┌──────┐            ┌────────────────┐ DIO3 R │
   │(USB CONN)     │  LORA32U4 │ATMEGA│            │ RFM95 / HDP13  │ DIO2 R │
   │               │           │ 32U4 │            │4=rst 7=irq 8=cs│      ║ │
   ║   (RST BTN)   │           └──────┘            └────────────────┘ ant(0)─┘
   ║               │                          15  16                       ║ 
   ║  RST 3V3 REF GND  A0  A1  A2  A3  A4 A5 SCK MOSI MISO 0   1 DIO1  ANT ╬
   ╚═══╬═══╬═══╬═══╬═══╬═══╬═══╬═══X═══X═══X═══R═══R═══R═══╬═══╬═══R═══════╝
                   │ __│   ?   ?              xxx xxx xxx  RX TX
                   └─  ┘   │   │               SPI-RFM95  serial1                                              
                pushbutton │   │                           ?   ?      18mA
     GND BAT               │   │  GND 5V                   │   │  GND 5V
    ┌─┼───┼─┐            ┌─┴───┴───┴───┴──────┴┴┴┐       ┌─┴───┴───┴───┴──┴┴┴┐
    │gnd Vin│            │RXD TXD GND 5V     misc│       │TXD RXD GND 5V misc│
    │step-up│            │    PM dust sensor     │       │     Co2 sensor    │
    │GND 5V │            │SDS021 42x32x24mm 60mA │       │       MH-Z19      │
    └─┼───┼─┘            │SDS011 71x70x23mm 220mA│       └───────────────────┘
                         └───────────────────────┘
                    
   * R = pins connected to RFM95
   * X = avoid main function on these pins for compatibility with 32u4 Beetle
   * for pushbutton to work, need to enable the internal pull-up
   * pin-out for GPS BN-180/BN-200/BN-220 ==> (led) GND TX RX VCC (batt)
   * pin-out for SDS021 ==> (hole) (1) 5V NC GND Rx Tx (5)
   * pin-out for SDS011 ==> (1) CTL 1umPWM 5V 25umPWM GND Rx Tx (7) (hole)
   * pin-out for MH-Z19 ==> 2 rows:
                      (HD)                
                      (SR)               PWM
                        Tx               (AoT)
                        Rx               GND
         (Vout 3.3V 10 mA)               Vin 5V
                            side window

```
## Pin mapping for Xtra small implementation (project X) on 3.3V SS Beetle
```
    -----------------------------------------------------------------------------
  
                  ┌───────────┐
                  │  │  │  │  │              
                  │  │  │  │  │              
                  │  │  │  │  │                 ╔═══════════════╗
           ╔══════│  │  │  │  │══════╗   ┌────┐ ╬GND        DIO2╬
     PM TXD╬A2                     MO╬───│──┐ └─╬MISO       DIO1╬
     PM RDX╬A1                    SCK╬───│─┐└───╬MOSI       DIOO╬
   GND─BTN─╬A0          SS         MI╬───┘ └────╬SCK  RFM95 3.3V╬─3V3
   I2c sda ╬A10 D9    BEETLE      RST╬          ╬NSS        DIO4╬
     CO2 tx╬A11 D10                5V╬          ╬RESET      DIO3╬    ant
     CO2 rx╬D11   RX  TX SDA SCL  GND╬          ╬DIO5        GND╬     │
           ║  D6  D0  D1  D2  D3 3V3 ║       gnd╬GND         ANA╬─────┘
           ╚══╬═══╬═══╬═══╬═══╬═══╬══╝          ╚═══════════════╝
                 RXD TXD I2C I2C RFM               
                 CO2 CO2 SDA SCL 3V3
                                   + LiPo?
```
## Pin mapping for Xtra small on 5V CJMCU Beetle
```
    -----------------------------------------------------------------------------
  
                                
                  ┌───────────┐
                  │  │  │  │  │              
                  │  │  │  │  │              
                  │  │  │  │  │                 ╔═══════════════╗
           ╔══════│  │  │  │  │══════╗          ╬GND        DIO2╬
           ║                       MI╬──────────╬MISO       DIO1╬
           ║                       MO╬──────────╬MOSI       DIOO╬
   I2c sca ╬SCA       CJMCU       SCK╬──────────╬SCK  RFM95 3.3V╬─diode─5V
   I2c sda ╬SDA       BEETLE      RES╬          ╬NSS        DIO4╬
     CO2 tx╬RX                    GND╬          ╬RESET      DIO3╬    ant
     CO2 rx╬TX                     5V╬          ╬DIO5        GND╬     │
           ║ D11 D10  D9  A0  A1  A2 ║       gnd╬GND         ANA╬─────┘
           ╚══╬═══╬═══╬═══╬═══╬═══╬══╝          ╚═══════════════╝
              rx  tx      │  RXD TXD                      
             GPS GPS    push  PM  PM
                         btn
                          │
                         gnd
                          
```  
## Pin mapping for Sodaq Explorer & NB-IOT gps shield
```

                                                                     ┌───────────────────────┐
                                                                     │    PM dust sensor     │
                                                                     │SDS021 42x32x24mm 60mA │
                                                                     │SDS011 71x70x23mm 220mA│
                                                                     │RXD TXD GND 5V     misc│
                             ┌───┐           ┌───┐                   └─┬───┬───┬───┬────┬─┬─┬┘
                             │  led  ┌───┐   │  push                   │   │   │   │
   SODAQ                     │   │   │  spk  │  btn                    │   │  gnd  5v
   EXPLORER   I2c            │   │   │   │   │   │                     │   │    
   ╔═════════╬═══╬═══╬═══╬═══╬═══╬═══╬═══╬═══╬═══╬═════╬═══╬═══╬═══╬═══╬═══╬═══╬═══╬═╗═══════════════╗ 
   ║RSTBTN  SCL SDA  X  GND D13 D12 D11 D10  D9 D8  - D7  D6  D5  D4  D3  D2  D1  D0 ║           lora║ 
   ║                        SCK MI  MO  SS                            tx  rx  TX  RX ║            ant║
   ║    LED-BUILTIN                                                           Serial ║               ║
   ║                   TEMP_SENSOR       LED_RED                                     \\              ║
   │USBCONN             analog           LED_GREEN                                    \\             ║
   │SerialUsb                            LED_BLUE      < ATSAMD21 >                    ║             ║
   │                                                                                   ║             ║
   │               A7  A8                                                              ║             ║
   ║       GND 3V D11 D12  o o+                                                        ║             ║
   ║         o  o  o  o    batt      o int          [ lora RN2483   ]                  ║             ║
   ║BTN                    o o+      o              [   Serial2     ]                  ║             ║
   ║       GND 3V SCA SCL  solr      o ext                                             ║             ║
   ║BLE      o  o  o  o                                                     SDA1 SCL1 //             ║
   ║Serial1                  X   X  RES 3.3 5V  GND GND BAT - A0  A1  A2  A3  A4  A5 //              ║
   ╚═════════════════════════╬═══╬═══╬═══╬═══╬═══╬═══╬═══╬═════╬═══╬═══╬═══╬═══╬═══╬═╝═══════════════╝
   
   NB-IOT
   SHIELD     I2c
   ╔═════════╬═══╬═══╬═══╬═══╬═══╬═══╬═══╬═══╬═══╬═════╬═══╬═══╬═══╬═══╬═══╬═══╬═══╬═╗ 
   ║        SCL SDA  X  GND D13 D12 D11 D10  D9 D8  - D7  D6  D5  D4  D3  D2  D1  D0 ║ 
   ║                                                                          TX  RX ║
   ║                               LSM303AGR/LSM303C ACCEL & MAGNETO          Serial ║ 
   ║         [                  ]  I2C-address ACCEL 0x1D, MAG 0x1E                   \
   ║nb-iot   [      NB-IOT      ]                                                      \
   ║ant      [ UBLOX SARA N200  ]  HTS221 TEMP&HUM                                     ║ 
   ║         [                  ]  I2C-address 0x5F            [                   ]   ║ 
   ║           Serial                                          [        GPS        ]   ║ 
   ║                               LPS22HB BAROMETRIC          [ UBLOX SAM-M8Q-0-10]   ║ 
   ║                               I2C-address is 0x5D         [                   ]   ║   
   ║                                                              I2C-address 0x42     ║   
   ║  SD CARD                                    RSTBTN                                ║ 
   ║                                                                                   /
   ║                         X   X  RES 3.3 5V  GND GND BAT - A0  A1  A2  A3  A4  A5  /
   ╚═════════════════════════╬═══╬═══╬═══╬═══╬═══╬═══╬═══╬═════╬═══╬═══╬═══╬═══╬═══╬═╝
   
   	   LPS22HB BAROMETRIC 
       I2C-address is 0x5D       
       
       HTS221 TEMP&HUM   
       I2C-address 0x5F
       
       
       

```
## IOT TTN message format
```
    Important: bytes 0 to 8 (nine bytes) are agreed format for TTNmapper.org 
      who will ignore any further bytes. They expect SF7 signal, which is our 
      default choice also for other reasons.
    
    -- start with TTNmapper defined format
    byte 0, 1, 2    Latitude   3 bytes, -90 to +90 degr, scaled to 0..16777215
    byte 3, 4, 5    Longitude  3 bytes, -180..+180 degrees, scaled 0..16777215
       note: earth circumfence is 40.075 km; data is 40075*1000/16777215 = 2.5 m
             2.5 m is about the GPS accuracy of 2..3 meters
    byte 6, 7       Altitude   2 bytes, in meters. 0..65025 meter
    byte 8          GPS DoP    byte, in 0.1 values. 0.25.5 DoP 
    
    -- then a section to detect dark spots in the coverage map
    byte 9, 10, 11  Prev Latit 
    byte 12, 13, 14 Prev Longi 
    byte 15, 16     Prev Altit 
    byte 17         Prev DoP    
                               
    -- now our 'regular' values
    byte 18         VCC        byte, 50ths, 0 - 5.10 volt -- secret voltmeter
    byte 19         CPUtemp    byte, -100 - 155 deg C     -- secret thermometer
    byte 20         Vbat       byte, 50ths, 0 - 5.10 volt -- hardwired Lora32u4
                               
    byte 21         
        0b0000 0000            
          -nnn nnnn Compass    0-120, My compass in 3 degree precision 0..360
                               Value=127: no compass value
          1--- ---- MyBtn#1    bit, is my button pressed
          
    byte 22         myID, dataset:
        0b0000 0000            
          ---- -nnn Dataset    Select Value 0-7 to tell which dataset
          ---- 0000 None       No additional data, this is just a GPS bleep. 
          ---- 0001 Set#1      Supplying our set#1 'environmental sensor' 
          ---- 1000 Radio      I have received a radio, sending radio values
          nnnn ---- MyTeamID   Value 0-31 my team ID
    byte 23,24         Counter    LoraWan Message counter. Last 2 bytes.
                               
    -- OPTIONAL set#1 environmental sensors values (not finalized)
    byte 25, 26     Moisture   2 bytes, AD measurement directly from AD port
    byte 27, 28     AirPress   2 bytes, AD measurement directly from AD port
    byte 29, 30     CO2        2 bytes, AD measurement directly from AD port
    byte 31, 32     PPM 2.5    2 bytes, AD measurement directly from AD port
    byte 33, 34     PPM 10     2 bytes, AD measurement directly from AD port
    byte 35, 36     Audio 1    2 bytes
    byte 37, 38     Audio 2    2 bytes
                      
    -- OPTIONAL radio values 
    byte 25         RemoteID   ID of remote team (who shot me)
        0b0000 0000            
          ---- nnnn RadioSSN   Received radio strength 1 
          nnnn ---- RemoteID   Value 0-31, Remote team ID
    byte 26, 27, 28 RemoteLat  3 bytes, -90 to +90 degrees scaled 0..16777215
    byte 29, 30, 31 RemoteLon  3 bytes, -180..+180 degrees scaled 0..16777215
    byte 32         R comp ++
        0b0000 0000            
          -nnn nnnn RemoteComp 0-120, Remote Compass 3 degree precision 0..360
                               Value=127: no compass value
          1--- ---- RemBtn#1   bit, is remote button pressed
    byte 33         distance ++
        0b0000 0000
          -nnn nnnn distance   0-100, Distance in meters        0..100
                               101-120,  100+(x-100)*20     for 120..500
                               121-126,  500+(x-120)*100     for 600..1100
                               Value=127: no distance value 
          1--- ---- Am I Hit   My hit status (I was hit?)
                               
    Game rule: add 2 degrees on each side of this 3 degree segment 
    A hit is when target is within this range and within 20 meters +/-3 meters

    THIS BYTE STRING NEEDS A DECODER FUNCTION IN TTN:
    /* * 
    
    function Decoder (bytes) {
      var _lat = (bytes[0] << 16 | bytes[1] << 8 | bytes[2]) / 16777215.0 * 180.0 - 90;
      var _lng = (bytes[3] << 16 | bytes[4] << 8 | bytes[5]) / 16777215.0 * 360.0 - 180;
      var _alt = (bytes[6] << 8) + bytes[7];
      var _hdop = bytes[8] / 10.0;

      var _prev_lat = (bytes[9] << 16 | bytes[10] << 8 | bytes[11]) / 16777215.0 * 180.0 - 90;
      var _prev_lng = (bytes[12] << 16 | bytes[13] << 8 | bytes[14]) / 16777215.0 * 360.0 - 180;
      var _prev_alt = (bytes[15] << 8) + bytes[16];
      var _prev_hdop = bytes[17] / 10.0;

      var _VCC = bytes[18] / 50;
      var _tempCPU = bytes[19] -100;
      var _Vbat = bytes[20] / 50;
      var _compass = (bytes[21] & 127) * 3;
      var _myBtn = bytes[21] >> 7;

      var _myID = bytes[22] >> 4;
      var _dataSetType = bytes[22] & 15;
      var _counter =  bytes[23] << 8 | bytes[24];
         
      var _inputHEX = bytes.map(function(b) { return ('0' + b.toString(16)).substr(-2);}).join(' ');

    // if _dataSetType = 0001 my game data
      var _remoteID = bytes[25] & 15;
      var _remote_radioSSN = bytes[25] >> 4;
      var _remote_lat = (bytes[26] << 16 | bytes[27] << 8 | bytes[28]) / 16777215.0 * 180.0 - 90;
      var _remote_lng = (bytes[29] << 16 | bytes[30] << 8 | bytes[31]) / 16777215.0 * 360.0 - 180;
      var _remote_compass = (bytes[32] & 127) * 3;
      var _remote_Btn = bytes[32] >> 7;
      var _remote_distance = bytes[33] & 127;
      var _remote_DidHitMe = bytes[33] >> 7;

      return {
        arduino_VCC: _VCC,
        arduino_Vbat: _Vbat,
        arduino_tempCPU: _tempCPU,
        compass: _compass,
        myBtn: _myBtn,
        myID: _myID,
        dataSetType: _dataSetType,
        gps_lat: _lat,
        gps_lng: _lng,
        gps_alt: _alt,
        gps_hdop: _hdop,
        gps_prev_lat: _prev_lat,
        gps_prev_lng: _prev_lng,
        gps_prev_alt: _prev_alt,
        gps_prev_hdop: _prev_hdop,
        counter: _counter,
        payload: _inputHEX,
        remoteID: _remoteID,
        remote_radioSSN: _remote_radioSSN,
        remote_lat: _remote_lat, 
        remote_lng: _remote_lng,
        remote_compass: _remote_compass,
        remote_Btn: _remote_Btn, 
        remote_distance: _remote_distance,
        remote_DidHitMe: _remote_DidHitMe 
      };
    }
    
    */
    
```

## p2p message format
```
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

```

