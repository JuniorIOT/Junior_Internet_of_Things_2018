
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

## Pin mapping
```
    Suggested pin mapping:
    -----------------------------------------------------------------------------


      ┌──────────────────────────────────────────┐
      │ solar      ┌────────────────────────────┐│
      │ powerbank  │ controll board           (Mini USB) ---> 2A usb charge cable  
      │ enclosure  │     B- B+   S- S+          ││
      │            └──────┼─┼─────┼─┼───────────┘│
      │┌────────────────┐ │ │  ┌──│─│──────────┐ │
      ││ 1-5x 1800mAh  +┼─│─┤  │  - +   200 mA │ │
      ││ Li-Ion 18650 - ┼─┤ │  │    solar panel│ │
      │└────────────────┘ │ │  └───────────────┘ │
      └───────────────────│─│────────────────────┘
   ┌──────────────────────┼─┼──┐ 
   │ Lipo protect    - + B- B+ │
   └─────────────────┼─┼───────┘
                     │ │ ┌───────────────────────┐   ┌───────────────┐
                     │ │ │          GPS          │   │    compass    │
                     │ │ │                       │   │    HMC5983    │
                     │ │ │                       │   │               │
          ┌──────┐   │ │ └─┬───┬───┬───┬───┬───┬─┘   └─┬───┬───┬───┬─┘   
          │ LIPO │   4V   blue wht │blk│red│grn│yel    │   │   │   │
          │  380 │   ext          3V3 GND  │   │       │   │   │   │       ant
          │  mAh │   - +                   │   │       │   │   │   │        │
          └─┬──┬─┘ ┌─┘ │                   │   │       │   │   │   │        │
   ╔════════│══│═══│═══╬═══╬═══╬═══╬═══╬═══╬═══╬═══╬═══╬═══╬═══╬═══╬══════╗ │
   ║        -  +   │  BAT EN  5V  13  12  11  10   9   6   5   3   2  DIO3╬ │
   ║    (LIPO CONN)│                               ┌─────────────┐    DIO2╬ │
   │               │                               │             │        ║ │
   │(USB CONN)     │   LORA32U4                    │   (RFM95)   │        ║ │
   │               │                               │             │        ║ │
   ║   (RST BTN)   │                               └─────────────┘     (0)──┘
   ║  RST 3V3 ARF GND  A0  A1  A2  A3  A4 A5 SCK MOSI MISO 0   1 DIO1  ANT╬
   ╚═══╬═══╬═══╬═══╬═══╬═══╬═══╬═══╬═══╬═══╬═══╬═══╬═══╬═══╬═══╬═══╬══════╝
                                              xxx xxx xxx       
   
```
## IOT TTN message format
```
    Important: bytes 0 to 8 (nine bytes) are agreed format for TTNmapper.org 
      who will ignore any further bytes. They expect SF7 signal, which is our 
      default choice also for other reasons.
    
    -- start with TTNmapper defined format
    byte 0, 1, 2    Latitude   3 bytes, -90 to +90 degr, scaled to 0..16777215
       note: earth circumfence is 6371 km; data is 6371*1000/16777215 = 0.4 m
             0.4 m is much better than the GPS accuracy of 2..3 meters
    byte 3, 4, 5    Longitude  3 bytes, -180..+180 degrees, scaled 0..16777215
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
                               
    -- OPTIONAL set#1 environmental sensors values (not finalized)
    byte 23, 24     Moisture   2 bytes, AD measurement directly from AD port
    byte 25, 26     AirPress   2 bytes, AD measurement directly from AD port
    byte 27, 28     CO2        2 bytes, AD measurement directly from AD port
    byte 29, 30     PPM 2.5    2 bytes, AD measurement directly from AD port
    byte 31, 32     PPM 10     2 bytes, AD measurement directly from AD port
    byte 33, 34     Audio 1    2 bytes
    byte 35, 36     Audio 2    2 bytes
                      
    -- OPTIONAL radio values 
    byte 23         RemoteID   ID of remote team (who shot me)
        0b0000 0000            
          ---- nnnn RadioSSN   Received radio strength 1 
          nnnn ---- RemoteID   Value 0-31, Remote team ID
    byte 24, 25, 26 RemoteLat  3 bytes, -90 to +90 degrees scaled 0..16777215
    byte 27, 28, 29 RemoteLon  3 bytes, -180..+180 degrees scaled 0..16777215
    byte 30         R comp ++
        0b0000 0000            
          -nnn nnnn RemoteComp 0-120, Remote Compass 3 degree precision 0..360
                               Value=127: no compass value
          1--- ---- RemBtn#1   bit, is remote button pressed
    byte 31         distance ++
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
         
      var _inputHEX = bytes.map(function(b) { return ('0' + b.toString(16)).substr(-2);}).join(' ');

    // if _dataSetType = my game data
      var _remoteID = bytes[23] & 15;
      var _remote_radioSSN = bytes[23] >> 4;
      var _remote_lat = (bytes[24] << 16 | bytes[25] << 8 | bytes[26]) / 16777215.0 * 180.0 - 90;
      var _remote_lng = (bytes[27] << 16 | bytes[28] << 8 | bytes[29]) / 16777215.0 * 360.0 - 180;
      var _remote_compass = (bytes[30] & 127) * 3;
      var _remote_Btn = bytes[30] >> 7;
      var _remote_distance = bytes[31] & 127;
      var _remote_DidHitMe = bytes[31] >> 7;

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

