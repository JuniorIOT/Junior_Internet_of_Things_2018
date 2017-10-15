
# GPS-Lora-Balloon-rfm95-TinyGPS

## Libraries required
- tbd1       https://github.com/tbd1
- tb21       https://github.com/tbd2

## Lora location transmittor with GPS and rfm95 for Kaasfabriek project 2018
```
  previous iteration is located at the project 
        for fablab Kaasfabriek Junior IoT Baloon Challenge february 2017

  this set of folders is for #JuniorIOTchallenge2018
             met leerling teams bij fablab de Kaasfabriek in Alkmaar
                     deze software op het internet is natuurlijk geheim...
                                  ...anders kunnen ze makkelijk afkijken!
```
See https://www.thethingsnetwork.org/labs/story/junior-iot-ballonnen-challenge

- Software plakker: Dennis --> dennis.ruigrok@gmail.com
- Educatie kletser: Marco --> marco@kaasfabriek.nl
- Regie en inspiratie: Kaasfabriek --> info at kaasfabriek punt nl

## Important
how to build your first node, practical soldering tips in nice pictures,
https://www.thethingsnetwork.org/labs/story/build-the-cheapest-possible-node-yourself
 --> you will need to use our pin mapping instead

## Pin mapping
```
    Suggested pin mapping:
    this new mapping puts the boards next to each other as shown in schema below,
          can be folded into compact stack or embedded in a business card size
    -----------------------------------------------------------------------------
                            ║                 ║ USB to serial programmer
                            ║3.3 TX RX GND 5V ║ while programming, power comes
                            ╚══╬══╬══╬══╬══╬══╝ from external supply to get
                               x  │  │  │  x    enough current for GPS
                               ┌──│──┘  │
                               │  │  x  │      ARDUINO pro mini
                          ╔═╬══╬══╬══╬══╬══╬═╗ 8Mhz 32kb                 ///////
      GPS unit            ║ G TX RX VCC GND B║ 3.3 volt                  helical
      NEO-6M              ║                  ║            gnd            868Mhz
      3.3 volt            ╬ TXD          RAW ╬x            │ 3v          antenna
     ╔═══════╗            ╬ RXD          GND ╬─black─60mm──┘ │           ///////
     ║       ║            ╬ RST          RST ╬x              │               │
     ║ (PPS) ╬x           ╬ GND          VCC ╬─red─48mm──────┘    3v   gnd   │
      
      NOPE, the old is no longer used here. Up to tree new designs are needed 
         (1) based on Lora32u4
         (2) based on SodaQ ExPloRer
         (3) based on Sodaq One

      └─────────────┘                                 ║    │  │  │  │  │       ║
                                                      ╚═╬══╬══╬══╬══╬══╬══╬══╬═╝
                                                      GND MI MO SCK NS RE D5 GND

```
## IOT TTN message format
```
    Important: bytes 0 to 8 (nine bytes) are agreed format for TTNmapper.org who will ignore 
    any further bytes. They expect SF7 signal, which for more reasons is our default choise.

    const unsigned message_size = 12;  // including byte[0]
    uint8_t  mydata[message_size];  // including byte[0]
    -- start with TTNmapper defined format
    // byte 0, 1, 2      Latitude       3 bytes, -90 to +90 degrees, scaled to 0 - 16777215
    // byte 3, 4, 5      Longitude      3 bytes, -180 to + 180 degrees, scaled to 0 - 16777215
    // byte 6, 7         Altitude       2 bytes, in meters. 0 - 65025 meter
    // byte 8            GPS DoP        byte, in 0.1 values.  0 - 25.5 DoP
    // -- now our 'usual' values
    // byte 9           My Compass     byte, degrees 0-180
          //  0b0000 0000
          //  0b00nn nnnn - value 0-63, My compass*64/180; compass in approx 3 degree precision 
                            Game rule: add 1.5 degree on each side of this 3 degree segment 
                            A hit is when target is within this larger range and within 20 meters 
          //  0b1000      - My button #1 
          //  0b0100      - spare
    // byte 10           Arduino VCC    byte, 50ths, 0 - 5.10 volt -- secret atmel voltmeter
    // byte 11           cpu temp       byte, -100 - 155 deg C     -- secret atmel thermometer
    // byte 12           Charging V     byte, 50ths, 0 - 5.10 volt -- hard-wired into Lora32u4
    // byte 13           myID
          //  0b0000 0000
          //          nnn -   value 0-7 to tell which dataset
          //         0000 -   no additional data, this is just a GPS bleep. message ends here
          //         0001 -   our set#1 'environmental sensors' values
          //         1000 -   I have received a radio, will send remote gameplay values
          //  0bnnnn      - value 0-31 my team ID
          
    // -- OPTIONAL set#1 environmental sensors values (not finalized)
    // byte 14, 15        CO2           2 bytes, AD measurement directly from AD port
    // byte 16, 17        Moisture      2 bytes, AD measurement directly from AD port
    // byte 18, 19        Air Pressure  2 bytes, AD measurement directly from AD port
    // byte 20, 21        O3            2 bytes, AD measurement directly from AD port

    // -- OPTIONAL remote gameplay values
    // byte 14           Remote ID
          //  0b0000 0000
          //         nnnn - received radio strength 1 
          //  0bnnnn      - value 0-31 Remote team ID
    // byte 15, 16, 17   Remote Lat     3 bytes, -90 to +90 degrees, scaled to 0 - 16777215
    // byte 18, 19, 20   Remote Longit  3 bytes, -180 to + 180 degrees, scaled to 0 - 16777215
    // byte 21           Remote Compass     byte, degrees 0-180
          //  0b0000 0000
          //  0b00nn nnnn - value 0-63, remote compass*64/180; compass in approx 3 degree precision 
          //  0b1000      - Remote button #1 
          //  0b0100      - My hit status (I was hit, have told remote, sounding 'ouch' for 60 sec)


    // THIS BYTE STRING NEEDS A DECODER FUNCTION IN TTN:
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
    // byte 0            My ID, message type
          //  0b0000 0000
          //         nnnn - message type
          //         0001 - handshake #1, I am yelling out loud that I have fired
          //         0010 - handshake #2, you have fired and here is my answer 
          //  0bnnnn      - value 0-31 My team ID
    // byte 1, 2, 3      My Lat         3 bytes, -90 to +90 degrees, scaled to 0 - 16777215
    // byte 4, 5, 6      My Longit      3 bytes, -180 to + 180 degrees, scaled to 0 - 16777215
    // byte 7            My Compass     byte, degrees 0-180
          //  0b0000 0000
          //  0b00nn nnnn - value 0-63, my compass*64/180; compass in approx 3 degree precision 
          //  0b1000      - my button #1 
          //  0b0100      - My hit status (I think I was hit, now tell remote, sounding 'ouch' for 60 sec)
    // byte 8            Your ID, hey I am talkming to you
    // byte 9            Validator, secret hash (binary add) based on message content, GPS date, application secret salt from keys.h

```

