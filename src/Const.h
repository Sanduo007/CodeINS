#ifndef CONST_H
#define CONST_H

#include <math.h>

#define pi 3.1415926535897932384626433832795
#define pi2 (2.0*pi)                    /* 2pi */
#define DEG2RAD (pi/180.0)                  /* Radians per degree */
#define RAD2DEG (180.0/pi)                  /* Degrees per radian */
#define Arcs (3600.0*180.0/pi)          /* Arcseconds per radian */


#define C_Light 299792458.0      /* Speed of light  [m/s]; IAU 1976  */


#define a_WGS84  6378137.0          /* long Radius of Earth [m]; WGS-84  */
#define b_WGS84  6356752.3142       /* short Radius of Earth [m]; WGS-84  */
#define e_WGS84  (sqrt(a_WGS84*a_WGS84 - b_WGS84*b_WGS84)) / a_WGS84   /*first eccentricity*/
#define F_WGS84  1.0/298.257223563  /* Flattening; WGS-84  ±âÂÊ */
#define Omega_WGS 7.2921151467e-5   /*[rad/s], the earth rotation rate */
#define GM_Earth   398600.5e+9     /* [m^3/s^2]; WGS-84 */


const float Ante_ca_Lever[3] = {0};
#endif // CONST_H
