#include <NMEAGPS.h>

//#include <SoftwareSerial.h>
//SoftwareSerial ss(10, 11);
//#define Serial1 ss

             #include <NeoSWSerial.h>
             NeoSWSerial gpsPort(10, 11);
//---------------------------------------------------------------------
//#define INTER_CHAR_TIME 50
#define INTER_CHAR_TIME 0

             #define GPS_PORT_NAME "NeoSWSerial(3,2)"
//             #define DEBUG_PORT Serial
////#include <GPSport.h>

static NMEAGPS  gps; // This parses the GPS characters

//----------------------------------------------------------------
//  Print the 32-bit integer degrees *as if* they were high-precision floats

static void printL( Print & outs, int32_t degE7 );
static void printL( Print & outs, int32_t degE7 )
{
  // Extract and print negative sign
  if (degE7 < 0) {
    degE7 = -degE7;
    outs.print( '-' );
  }

  // Whole degrees
  int32_t deg = degE7 / 10000000L;
  outs.print( deg );
  outs.print( '.' );

  // Get fractional degrees
  degE7 -= deg*10000000L;

  // Print leading zeroes, if needed
  int32_t factor = 1000000L;
  while ((degE7 < factor) && (factor > 1L)){
    outs.print( '0' );
    factor /= 10L;
  }
  
  // Print fractional degrees
  outs.print( degE7 );
}

static void doSomeWork( const gps_fix & fix );
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
      Serial.print( '0' );
    Serial.print( fix.dateTime.seconds );
    Serial.print( ',' );

    // Serial.print( fix.latitude(), 6 ); // floating-point display
    // Serial.print( fix.latitudeL() ); // integer display
    printL( Serial, fix.latitudeL() ); // prints int like a float
    Serial.print( ',' );
    // Serial.print( fix.longitude(), 6 ); // floating-point display
    // Serial.print( fix.longitudeL() );  // integer display
    printL( Serial, fix.longitudeL() ); // prints int like a float

    Serial.print( ',' );
    if (fix.valid.satellites)
      Serial.print( fix.satellites );

    Serial.print( ',' );
    Serial.print( fix.speed(), 6 );
    Serial.print( F(" kn = ") );
    Serial.print( fix.speed_mph(), 6 );
    Serial.print( F(" mph") );

  } else {
    // No valid location data yet!
    Serial.print( '?' );
  }

  Serial.println();

} // doSomeWork

//------------------------------------

void setup()
{
  
  Serial.begin(9600);

  Serial.print( F("NMEAloc.INO: started\n") );
  Serial.print( F("fix object size = ") );
  Serial.println( sizeof(gps.fix()) );
  Serial.print( F("NMEAGPS object size = ") );
  Serial.println( sizeof(gps) );
  Serial.println( F("Looking for GPS device on " GPS_PORT_NAME) );

  #ifdef NMEAGPS_NO_MERGING
    Serial.println( F("Only displaying data from xxRMC sentences.\n  Other sentences may be parsed, but their data will not be displayed.") );
  #endif

  Serial.flush();

  gpsPort.begin(9600);
}

//--------------------------

void loop()
{
  
  while (gps.available( gpsPort ))
    doSomeWork( gps.read() );

}
