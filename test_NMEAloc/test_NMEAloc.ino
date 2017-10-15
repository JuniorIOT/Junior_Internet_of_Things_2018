//#include <SoftwareSerial.h>  // better use NeoSWSerial, they claim it is much lighter, misses less characters and that would be nice
//SoftwareSerial ss(10, 11);
//#define Serial1 ss

#include <NeoSWSerial.h>  // an issue with Leonardo-types is fixed in branch, yet to be merged. So you may need to remove all your NeoSWSerial libraries and add \libraries\NeoSWSerial-master-DamiaBranch.zip
NeoSWSerial ss( 10, 11 );

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
      Serial.print( "0" );
    Serial.print( fix.dateTime.seconds );
    Serial.print(" datetime sec, ");
    
    Serial.print( fix.dateTime );
    Serial.print(" datetime, ");
    

    // Serial.print( fix.latitude(), 6 ); // floating-point display
    Serial.print( fix.latitudeL() ); // integer display
    Serial.print(" lat, ");
    // Serial.print( fix.longitude(), 6 ); // floating-point display
    Serial.print( fix.longitudeL() );  // integer display
    
    Serial.print(" lon, ");
    if (fix.valid.satellites)
      Serial.print( fix.satellites );

    Serial.print(", ");
    Serial.print( fix.speed(), 6 );
    Serial.print( F(" kn = ") );
    Serial.print( fix.speed_mph(), 6 );
    Serial.print( F(" mph") );
  } else {
    // No valid location data yet!
    Serial.print( 'No valid location data yet!' );
  }
  Serial.println();
} 

void setup()
{
  
  Serial.begin(9600);

  Serial.print( F("NMEAloc.INO: started\n") );
  Serial.print( F("fix object size = ") );
  Serial.println( sizeof(gps.fix()) );
  Serial.print( F("NMEAGPS object size = ") );
  Serial.println( sizeof(gps) );

  #ifdef NMEAGPS_NO_MERGING
    Serial.println( F("Only displaying data from xxRMC sentences.\n  Other sentences may be parsed, but their data will not be displayed.") );
  #endif
  Serial.flush();

  ss.begin(9600);
}

void loop()
{
  while (gps.available( ss )) {
    doSomeWork( gps.read() );

    //char c = gps.read();
    ///Serial.print(c);
    ///doSomeWork( c );
  }
}
