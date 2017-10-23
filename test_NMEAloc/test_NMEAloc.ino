//#include <SoftwareSerialUSB.h>  // better use NeoSWSerialUSB, they claim it is much lighter, misses less characters and that would be nice
//SoftwareSerialUSB ss(10, 11);
//#define SerialUSB1 ss

#define ss Serial

#include <NMEAGPS.h>
static NMEAGPS  gps; // This parses the GPS characters

// output ==> 04 datetime sec, 561398704 datetime, 526326337 lat, 47384373 lon, 8, 1.067000 kn = 1.227881 mph

//static void doSomeWork( const gps_fix & fix );
static void doSomeWork( const gps_fix & fix )
{
  //  This is the best place to do your time-consuming work, right after
  //     the RMC sentence was received.  If you do anything in "loop()",
  //     you could cause GPS characters to be lost, and you will not
  //     get a good lat/lon.
  //  For this example, we just print the lat/lon.  If you print too much,
  //     this routine will not get back to "loop()" in time to process
  //     the next set of GPS data.

  if (fix.valid.location) {
    if ( fix.dateTime.seconds < 10 )
      SerialUSB.print( "0" );
    SerialUSB.print( fix.dateTime.seconds );
    SerialUSB.print(" datetime sec, ");
    
    SerialUSB.print( fix.dateTime );
    SerialUSB.print(" datetime, ");
    

    // SerialUSB.print( fix.latitude(), 6 ); // floating-point display
    SerialUSB.print( fix.latitudeL() ); // integer display
    SerialUSB.print(" lat, ");
    // SerialUSB.print( fix.longitude(), 6 ); // floating-point display
    SerialUSB.print( fix.longitudeL() );  // integer display
    
    SerialUSB.print(" lon, ");
    if (fix.valid.satellites)
      SerialUSB.print( fix.satellites );

    SerialUSB.print(", ");
    SerialUSB.print( fix.speed(), 6 );
    SerialUSB.print( F(" kn = ") );
    SerialUSB.print( fix.speed_mph(), 6 );
    SerialUSB.print( F(" mph") );
  } else {
    // No valid location data yet!
    SerialUSB.print( 'No valid location data yet!' );
  }
  SerialUSB.println();
} 

void setup()
{
  
  SerialUSB.begin(9600);

  SerialUSB.print( F("NMEAloc.INO: started\n") );
  SerialUSB.print( F("fix object size = ") );
  SerialUSB.println( sizeof(gps.fix()) );
  SerialUSB.print( F("NMEAGPS object size = ") );
  SerialUSB.println( sizeof(gps) );

  #ifdef NMEAGPS_NO_MERGING
    SerialUSB.println( F("Only displaying data from xxRMC sentences.\n  Other sentences may be parsed, but their data will not be displayed.") );
  #endif
  SerialUSB.flush();

  ss.begin(9600);
}

void loop()
{
//  // NMEAGPS
  while (gps.available( ss )) {
    doSomeWork( gps.read() );
  }

/*  if (ss.available()) {
    char c = ss.read();
    SerialUSB.write(c);
  }
  */
}
