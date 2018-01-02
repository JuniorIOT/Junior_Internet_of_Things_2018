  /****************************************************************
bearing.ino

Calculates the disatance and bearing given two GPS location near
each other.

Author: Roel Drost
  Date: 2017-12-30

****************************************************************/

//------------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------------
#include <math.h>

//------------------------------------------------------------------------------
// Types
//------------------------------------------------------------------------------
typedef struct {
   double latitude;  // (assumes North)
   double longitude; // (assumes West)
} coordinate_t;

typedef struct {
  char*        name;
  coordinate_t location;
} testLocation_t;

//------------------------------------------------------------------------------
// Constants
//------------------------------------------------------------------------------

/**
 * Some test vectors
 */
static const testLocation_t testLocations[] = {
  "Kaasfabriek", {52.6384638, 4.7497695},
  "Watertoren" , {52.6365654, 4.7363240},
  "Waagplein"  , {52.6314069, 4.7503715},
};

/**
 * The distance to the middle point of the earth in meters for 'Alkmaar'.
 * Note that this is different relative to the latitude.
 */
static const double earthRadius = 6364670;

//------------------------------------------------------------------------------
// Functions
//------------------------------------------------------------------------------

static inline double radToDeg(const double& rad) {
  return rad * 180.0 / PI;
}
//------------------------------------------------------------------------------

static inline double degToRad(const double& deg) {
  return deg * PI / 180.0;
}
//------------------------------------------------------------------------------

/**
 * Calculates the distance and bearing between two GPS coordinates.
 * WARNING this function is a rough simplification!!! It assumes the area where
 * the two points are can be considered as flat! It also assumes the radius of
 * earth is that of 'Alkmaar'.
 * @param    from
 *             in: the GPS location 'from'
 * @param    to
 *             in: the GPS locaton 'to'
 * @param    distance
 *             out: the calculated distance
 * @param    bearing
 *             out: the calculated bearing
 */
static void calculateNearAlkmaar(
  const coordinate_t& from,
  const coordinate_t& to,
  double&             distance,
  double&             bearing)
{
  const double deltaLatitude  = to.latitude  - from.latitude;
  const double deltaLongitude = to.longitude - from.longitude;

  const double aspectRatio = cos(degToRad(from.latitude));
  const double distanceX   = (deltaLongitude / 360) * 2 * PI * earthRadius * aspectRatio;
  const double distanceY   = (deltaLatitude  / 360) * 2 * PI * earthRadius;

  distance = sqrt(distanceX*distanceX + distanceY*distanceY);

  if (distanceY > 0.0) {
    bearing = (radToDeg(atan(distanceX/distanceY)) + 360.0);
  } else if (distanceY < 0.0) {
    bearing = (radToDeg(atan(distanceX/distanceY)) + 180.0);
  } else {
    if (distanceX > 0.0) {
      bearing = 90;
    } else if (distanceX < 0.0) {
      bearing = 270;
    } else {
      bearing = NAN;
    }
  }
  if (bearing >= 360) {
    bearing -= 360;
  }
}
