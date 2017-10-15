
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
Important: bytes 0 to 8 (nine bytes) are agreed format for TTNmapper.org who will ignore 
any further bytes. They expect SF7 signal, which for more reasons is our default choise.

const unsigned message_size = 12;  // including byte[0]
uint8_t  mydata[message_size];  // including byte[0]
-- start with TTNmapper defined format
// byte 0, 1, 2      Latitude       3 bytes, -90 to +90 degrees, scaled to 0 - 16777215
// byte 3, 4, 5      Longitude      3 bytes, -180 to + 180 degrees, scaled to 0 - 16777215
// byte 6, 7         Altitude       2 bytes, in meters. 0 - 65025 meter
// byte 8            GPS DoP        byte, in 0.1 values.  0 - 25.5 DoP
// --- now our 'usual' values
// byte 9            Arduino VCC    byte, 50ths, 0 - 5.10 volt -- secret atmel voltmeter
// byte 10           cpu temp       byte, -100 - 155 deg C     -- secret atmel thermometer
// byte 11           Charging V     byte, 50ths, 0 - 5.10 volt -- hard-wired into Lora32u4
// -- OPTIONAL now our gameplay values
// byte 12           Who am I       byte, my unique game ID 0 - 255
// byte 13           action select  byte, multiple bit info
      //  0b1000 0000 - I have received a radio 
      //  0b0100 0000 - remote button #1 was pressed
      //  0b0010 0000 - remote button #2 was pressed
      //  0b0001 0000 - I have interpreted this as a hit type 1
      //  0b0000 1000 - I have interpreted this as a hit type 2
      //  0b0000



      0000 - 
      //  0b1000 0000 - my button #1 was pressed
      //  0b0100 0000 - my button #2 was pressed
// byte 11           

// byte xx           time to fix    1 byte: any time with approx 10% accuracy 1 sec - 7 hours
      // 0..60 sec  at 1 sec interval <==> values 0 .. 60 
      // 1..10 min at 5 sec interval  <==> values 60 ..  168
      // 10..60 min at 1 min interval <==> values 168 .. 218
      // 1..7 hour at 10 min interval <==> values 218 ..254; 255 is "more than 7 hours"

// THIS BYTE STRING NEEDS A DECODER FUNCTION IN TTN:
/* * function Decoder (bytes) {
  var _lat = ((bytes[0] << 16) + (bytes[1] << 8) + bytes[2]) / 16777215.0 * 180.0 - 90;
  var _lng = ((bytes[3] << 16) + (bytes[4] << 8) + bytes[5]) / 16777215.0 * 360.0 - 180;
  var _alt = (bytes[6] << 8) + bytes[7];
  var _acc = bytes[8] / 10.0;
  var _VCC = bytes[9] / 50;
  var _tempCPU = bytes[10] -100;
  var _time_to_fix_bin = bytes[11];
  var _time_to_fix;
  if (_time_to_fix_bin>=218) { _time_to_fix = 60*60+(_time_to_fix_bin-218)*600 }
  else if (_time_to_fix_bin>=168) { _time_to_fix = 10*60+(_time_to_fix_bin-168)*60 }
  else if (_time_to_fix_bin>=60) {  _time_to_fix = 60+(_time_to_fix_bin-60)*5 }
  else  {_time_to_fix = _time_to_fix_bin }
  
      // 0..60 sec  at 1 sec interval <==> values 0 .. 60 
      // 1..10 min at 5 sec interval  <==> values 60 ..  168
      // 10..60 min at 1 min interval <==> values 168 .. 218
      // 1..7 hour at 10 min interval <==> values 218 ..254; 255 means "more than 7 hours"
      
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
    time_to_fix: _time_to_fix,
    payload: _inputHEX
  };
}
*/

## p2p message format
