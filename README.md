
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

## Libraries required
- tbd1       https://github.com/tbd1
- tb21       https://github.com/tbd2

## Pin mapping
```
    Suggested pin mapping:
    -----------------------------------------------------------------------------

          ┌──────┐
          │ LIPO │
          └─┬──┬─┘                                         
            │  │                                        
   ╔════════│══│═══════╬═══╬═══╬═══╬═══╬═══╬═══╬═══╬═══╬═══╬═══╬═══╬══════╗
   ║        -  +      BAT EN  5V  13  12  11  10   9   6   5   3   2  DIO3╬
   ║    (LIPO CONN)                                ┌─────────────┐    DIO2╬
   ║                                               │             │        ║
   ║(USB CONN)         LORA32U4                    │   (RFM95)   │        ║
   ║                                               │             │        ║
   ║   (RST BTN)                                   └─────────────┘        ║
   ║  RST 3V3 ARF GND  A0  A1  A2  A3  A4 A5 SCK MOSI MISO 0   1 DIO1  ANT╬
   ╚═══╬═══╬═══╬═══╬═══╬═══╬═══╬═══╬═══╬═══╬═══╬═══╬═══╬═══╬═══╬═══╬══════╝
   
   
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
    byte 9, 10, 11  Prev Latit 3 bytes, -90 to +90 degr, scaled to 0..16777215
    byte 12, 13, 14 Prev Longi 3 bytes, -180..+180 degrees, scaled 0..16777215
    byte 15, 16     Prev Altit 2 bytes, in meters. 0..65025 meter
    byte 8          GPS DoP    byte, in 0.1 values. 0.25.5 DoP 
                               
    -- now our 'regular' values
    byte 9                        
        0b0000 0000            
          -nnn nnnn Compass    0-120, My compass in 3 degree precision 0..360
                               Value=127: no compass value
          1--- ---- MyBtn#1    bit, is my button pressed
    byte 10         VCC        byte, 50ths, 0 - 5.10 volt -- secret voltmeter
    byte 11         CPUtemp    byte, -100 - 155 deg C     -- secret thermometer
    byte 12         Vbat       byte, 50ths, 0 - 5.10 volt -- hardwired Lora32u4
                               
    byte 13, 14, 15 prevLat    3 bytes, ...
    byte 16, 17, 18 prevLon    3 bytes,    ... to detect dark spots
    byte 19         myID, dataset:
        0b0000 0000            
          ---- -nnn Dataset    Select Value 0-7 to tell which dataset
          ---- 0000 None       No additional data, this is just a GPS bleep. 
          ---- 0001 Set#1      Supplying our set#1 'environmental sensor' 
          ---- 1000 Radio      I have received a radio, sending radio values
          nnnn ---- MyTeamID   Value 0-31 my team ID
                               
    -- OPTIONAL set#1 environmental sensors values (not finalized)
    byte 21, 22     CO2        2 bytes, AD measurement directly from AD port
    byte 23, 24     Moisture   2 bytes, AD measurement directly from AD port
    byte 25, 26     AirPress   2 bytes, AD measurement directly from AD port
    byte 27, 28     O3         2 bytes, AD measurement directly from AD port
    byte 29         spare
                  
    -- OPTIONAL radio values 
    byte 21         RemoteID   ID of remote team (who shot me)
        0b0000 0000            
          ---- nnnn RadioSSN   Received radio strength 1 
          nnnn ---- RemoteID   Value 0-31, Remote team ID
    byte 22, 23, 24 RemoteLat  3 bytes, -90 to +90 degrees scaled 0..16777215
    byte 25, 26, 27 RemoteLon  3 bytes, -180..+180 degrees scaled 0..16777215
    byte 28         R comp ++
        0b0000 0000            
          -nnn nnnn RemoteComp 0-120, Remote Compass 3 degree precision 0..360
                               Value=127: no compass value
          1--- ---- RemBtn#1   bit, is remote button pressed
    byte 29         distance ++
        0b0000 0000
          -nnn nnnn distance   0-100, Distance in meters        0..100
                               101-120,  100+(x-100)*20     for 120..500
                               121-126,  500+(x-120)*100     for 600..1100
                               Value=127: no distance value 
          1--- ---- Am I Hit   My hit status (I was hit?)
                               
    Game rule: add 2 degrees on each side of this 3 degree segment 
    A hit is when target is within this range and within 20 meters 



    THIS BYTE STRING NEEDS A DECODER FUNCTION IN TTN:
    /* * function Decoder (bytes) {
      var _lat = ((bytes[0] << 16) + (bytes[1] << 8) + bytes[2]) / 16777215.0 * 180.0 - 90;
      var _lng = ((bytes[3] << 16) + (bytes[4] << 8) + bytes[5]) / 16777215.0 * 360.0 - 180;
      var _alt = (bytes[6] << 8) + bytes[7];
      var _acc = bytes[8] / 10.0;
      var _VCC = bytes[9] / 50;
      var _tempCPU = bytes[10] -100;
     ...
      var _inputHEX = bytes[0].toString(16)+' '+bytes[1].toString(16)+' '+bytes[2].toString(16)
                      +' '+bytes[3].toString(16)+' '+bytes[4].toString(16)+' '+bytes[5].toString(16)
                      +' '+bytes[6].toString(16)+' '+bytes[7].toString(16)+' '+bytes[8].toString(16)
                      +' / '+bytes[9].toString(16)+' '+bytes[10].toString(16)+' '+bytes[11].toString(16);
      return {
        gps_lat: _lat,
        gps_lng: _lng,
        gps_alt: _alt,
        gps_prec: _acc,
        arduino_VCC: _VCC,
        arduino_temp: _tempCPU,
      ...
      payload: _inputHEX
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

