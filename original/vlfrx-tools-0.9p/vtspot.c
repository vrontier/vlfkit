//
// Copyright (c) 2010 Paul Nicholson
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
// IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
// OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
// IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
// NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
// THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//

#include "config.h"
#include "vtport.h"
#include "vtlib.h"

#define EARTH_RAD 6371.2

#define ERAD_a 6378.137     // WGS84 equatorial radius
#define ERAD_b 6356.7523142 // WGS84 polar radius

#define ERAD_a2 (ERAD_a * ERAD_a)
#define ERAD_b2 (ERAD_b * ERAD_b)

// Command options
static int OPT_b = FALSE;        // -b option: calculate bearing
static int OPT_d = FALSE;        // -d option: calculate destination point
static int OPT_g = FALSE;        // -g option: output great circle
static int OPT_q = FALSE;        // -q option: query velocity factor
static int OPT_s = FALSE;        // -s option: calculate sun position
static int OPT_M = FALSE;        // -M option: calculate magnetic field
static int OPT_C = FALSE;        // -C option: compute centroid
static int OPT_p = FALSE;        // -p option: perpendicular distance to path
static int OPT_P = FALSE;        // -P option: test bounding polygon

static int ncpu = 1;             // -U option: number of worker threads to use
static int OMODE_ISO = FALSE;    // Set by -o iso: ISO output format timestamps
static int OMODE_EXT = FALSE;    // Set by -o ext: extended output
static int OMODE_BRC = FALSE;    // Set by -o brc: output .brc files

static double filter_envelop = 0;  // -f envelop= option
static double filter_nearmax = 0;  // -f nearmax= option
static double maxatd = 0;          // -f maxatd= option
static int shortsun = FALSE;       // TRUE if -f shortsun given

static char *pointlist = NULL;   // Argument of -S option
static char *opt_radec = NULL;   // Argument of -R option

static char *vfmodel_loadname = NULL;  // From -c option when model data file
                                       // is given

static int illum_type = 0;        // 0 == above horizon fraction
                                  // 1 == integrated vertical component
#define BASELINE_EXCESS_VLF 1.05
#define BASELINE_EXCESS_ELF 2.0

static double baseline_excess = BASELINE_EXCESS_VLF;

#define MAXSITES 200

//
// If a site or measurement doesn't provide a sigma for its TOGAs or bearings,
// then these are the values used.
//

static double DEFAULT_AT_SIGMA = 20e-6;    // Arrival time sigma, seconds
static double DEFAULT_AZ_SIGMA = 5;        // Azimuth sigma, degrees

static double max_residual = 1; // Set by -r option:  units of sigma

//
//
//

#define MODEL_FIXED 0
#define MODEL_ELF1 1
#define MODEL_ELF2 2
#define MODEL_ELF3 3
#define MODEL_ELF4 4
#define MODEL_VLF0 5
#define MODEL_VLF1 6
#define MODEL_VLF2 7

#define DEFAULT_VF 0.9922

static double CVLF = 300e3 * DEFAULT_VF;
static double fixed_VF = DEFAULT_VF;

static int cvlf_given = FALSE;
static int copt_model = MODEL_FIXED;  // Model selected by -c option

//
// Some little utilities.
//

static void usage( void)
{
   fprintf( stderr,
     "usage:  vtspot [options] [measurement1 [measurement2 ...]]\n"
     "\n"
     "measurements:\n"
     "\n"
     "    T/location/timestamp[/sigma]     (sigma in seconds, default 20e-6)\n"
     "    B/location/bearing[/sigma]       (sigma in degrees, default, 5.0)\n"
     "\n"
     "options:\n"
     "  -v            Increase verbosity\n"
     "  -L name       Specify logfile\n"
     "  -U ncpu       Number of worker threads to use (default 1)\n"
     "\n"
     "  -c factor     Fixed velocity factor (default 0.9922)\n"
     "  -c modelname  Use a built-in propagation model for velocity factor\n"
     "\n"
     "matching:\n"
     " -m location=file    Specify TOGA file for the receiver location\n"
     " -n minsites         Minimum number of sites to attempt solution\n"
     "                     (default is all sites)\n"
     " -r residual         Max residual to accept solution (default 1.0)\n"
     " -f envelop=degrees  Apply surrounding filter rule\n"
     " -f nearmax=km       Apply distance limit to nearest receiver\n"
     " -f maxatd=seconds   Max arrival time difference to consider\n"
     " -o ext              Extended output records\n"
     " -o iso              Output ISO timestamps (default numeric)\n"
     " -a t=seconds        Timing measurement sigma (default 20e-6)\n"
     " -a z=degrees        Azimuth measurement sigma (default 5.0\n"
     "\n"
     "distances, bearings, paths:\n"
     " -b standpoint forepoint              Range and bearing to forepoint\n"
     " -d standpoint bearing range_km       Compute forepoint\n"
     " -g point1 point2 [angle_step]        Great circle path points\n"
     " -p standpoint forepoint crosspoint   Cross track distance, position\n"
     "\n"
     "utility functions:\n"
     " -s standpoint timestamp            Calculate Sun azimuth, elevation\n"
     " -M1 standpoint timestamp           Calculate magnetic field\n"
     " -M2 standpoint timestamp           Compute conjugate and L shell\n"
     " -CS \"lat1,lon1 lat2,lon2 ...\"      Compute centroid of set of points\n"
     " -S \"point1 point2 point3 ...\" -P standpoint  Test bounding polygon\n"
     "\n"
     "diagnostic:\n"
     " -q1                                  Print velocity factor table\n"
     " -q2 standpoint forepoint timestamp   Velocity factor for GC path\n"
     " -q3 distance illumination            Velocity factor\n"
   );

   exit( 1);
}

static double constrain( double a, double low, double high)
{
   double r = high - low;
   while (a < low) a += r;
   while (a >= high) a -= r;
   return a;
}

///////////////////////////////////////////////////////////////////////////////
// N-vector Operations                                                       //
///////////////////////////////////////////////////////////////////////////////

// https://www.ffi.no/en/research/n-vector/n-vector-explained
// https://www.ffi.no/en/research/n-vector
// https://www.navlab.net/Publications/A_Nonsingular_Horizontal_Position_Representation.pdf
// https://nvector.readthedocs.io/en/latest/

typedef double V3[3];
typedef double A3[3][3]; 

static V3 V3north = { 0, 0, 1 };
// static A3 A0 = { { 0, 0, 0}, {0, 0, 0}, {0, 0, 0} };
static A3 AI = { { 1, 0, 0}, {0, 1, 0}, {0, 0, 1} };

static inline int v3_equal( V3 const v1, V3 const v2)
{
   return v1[0] == v2[0] && v1[1] == v2[1] && v1[2] == v2[2];
}

static inline void v3_add( V3 const v1, V3 const v2, V3 vr)
{
   vr[0] = v1[0] + v2[0];
   vr[1] = v1[1] + v2[1];
   vr[2] = v1[2] + v2[2];
}

#if 0   // Not used right now
static void v3_sub( V3 const v1, V3 const v2, V3 vr)
{
   vr[0] = v1[0] - v2[0];
   vr[1] = v1[1] - v2[1];
   vr[2] = v1[2] - v2[2];
}
#endif

// Dot product
static inline double v3_dot( V3 const v1, V3 const v2)
{
   return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

// Cross product, v1 x v2 -> vc
static inline void v3_cross( V3 const v1, V3 const v2, V3 vc)
{
   vc[0] = v1[1] * v2[2] - v1[2] * v2[1];
   vc[1] = v1[2] * v2[0] - v1[0] * v2[2];
   vc[2] = v1[0] * v2[1] - v1[1] * v2[0];
}

// Magnitude
static inline double v3_mag( V3 const v)
{
   return sqrt( v3_dot( v, v));
}

// Multiply by a scalar
static inline void v3_scale( V3 const v, double s, V3 r)
{
   int i;
   for (i = 0; i < 3; i++) r[i] = v[i] * s;
}

// Make into unit vector
static inline void v3_normalise( V3 v)
{
   v3_scale( v, 1/v3_mag( v), v);
}

// Random normalised vector
static inline void v3_random( V3 v)
{
   v[0] = VT_rand(); v[1] = VT_rand(); v[2] = VT_rand();
   v3_normalise( v);
}

// Cross product matrix from vector
static inline void a3_cpmat( V3 const v, A3 a)
{
   a[0][0] = 0;     a[0][1] = -v[2];      a[0][2] = v[1];
   a[1][0] = v[2];  a[1][1] = 0;          a[1][2] = -v[0];
   a[2][0] = -v[1]; a[2][1] = v[0];       a[2][2] = 0;
}

static void a3_scalar_mul( A3 a, double s, A3 r)
{
   int i, j;
   for (i = 0; i < 3; i++) for (j = 0; j < 3; j++) r[i][j] = a[i][j] * s;
}

#if 0   // Not used right now
static void a3_copy( A3 s, A3 d)
{
   memcpy( d, s, sizeof( A3));
}
#endif

static void v3_copy( V3 const s, V3 d)
{
   memcpy( d, s, sizeof( V3));
}

// Angle between two vectors, radians
static inline double v3_angle( V3 const v1, V3 const v2)
{
   V3 xp; v3_cross( v1, v2, xp);
   return atan2( v3_mag( xp), v3_dot( v1, v2));
}

// Matrix sum, a + b -> r and r can be the same matrix as a and/or b
static void a3_sum( A3 a, A3 b, A3 r)
{
   int i, j;
   for (i = 0; i < 3; i++) for (j = 0; j < 3; j++) r[i][j] = a[i][j] + b[i][j];
}

// Matrix multiplication a x b -> r
static void a3_mul( A3 a, A3 b, A3 r)
{
   int i, j, k;

   for (i = 0; i < 3; i++)
      for (j = 0; j < 3; j++)
      {
         r[i][j] = 0;
         for (k = 0; k < 3; k++) r[i][j] += a[i][k] * b[k][j];
      }
}

#if 0   // Not used right now
static double a3_det( A3 X)
{
   return
    X[0][0]*X[1][1]*X[2][2] + X[0][1]*X[1][2]*X[2][0] +
    X[0][2]*X[1][0]*X[2][1] - X[0][2]*X[1][1]*X[2][0] -
    X[0][1]*X[1][0]*X[2][2] - X[0][0]*X[1][2]*X[2][1];
}
#endif

// Compute the unit vector normal to v1 and v2
static inline void v3_unit_normal_to( V3 const v1, V3 const v2, V3 vn)
{
   v3_cross( v1, v2, vn);
   v3_normalise( vn);
}

#if 0   // Not used right now
static void v3_outer_prod( V3 const v1, V3 const v2, A3 r)
{
   int i, j;
   for (i = 0; i < 3; i++) for (j = 0; j < 3; j++) r[i][j] = v1[i] * v2[j];
}
#endif

static void v3_transform( V3 const v, A3 rot, V3 r)
{
   int i, j;

   for (i = 0; i < 3; i++)
   {
      r[i] = 0;
      for (j = 0; j < 3; j++) r[i] += rot[i][j] * v[j];
   }
}

// Compute rotation matrix R from unit axis vector v and angle phi
static void a3_rot( V3 const v, double phi, A3 r)
{
   A3 k;  a3_cpmat( v, k);
   A3 t;  a3_scalar_mul( k, sin(phi), t);  a3_sum( AI, t, r);
   a3_mul( k, k, t);  a3_scalar_mul( t, 1 - cos(phi), t); a3_sum( r, t, r);
}

///////////////////////////////////////////////////////////////////////////////
// N-vector Calculations                                                     //
///////////////////////////////////////////////////////////////////////////////

static inline void v3_latlon( V3 const v, double *lat, double *lon)
{
   *lat = atan2( v[2], sqrt(v[0] * v[0] + v[1] * v[1]));
   *lon = atan2( v[1], v[0]);

   if (*lat > M_PI/2)
   {
      *lat = M_PI - *lat;
      *lon += M_PI;
   }
   else
   if (*lat < -M_PI/2)
   {
      *lat = -M_PI - *lat;
      *lon += M_PI;
   }

   *lon = constrain( *lon, -M_PI, M_PI);
}

//
// Convert N-vector to latitude/longitude and format into a string.
//
// If 's' is supplied, it forms the destination and return value.
// With NULL 's', a pointer to a static string is returned, so should be used
// no more than once in a printf argument list.
//

static char * v3_string( V3 const v, char *s)
{
   static char temp[50];

   if (!s) s = temp;
   double lat, lon;

   v3_latlon( v, &lat, &lon);

   if (lat > M_PI/2)
   {
      lat = M_PI - lat;
      lon += M_PI;
   }
   else
   if (lat < -M_PI/2)
   {
      lat = -M_PI - lat;
      lon += M_PI;
   }

   lon = constrain( lon, -M_PI, M_PI);
   sprintf( s, "%.3f,%.3f", lat*180/M_PI, lon*180/M_PI);
   return s;
}

//
// Convert latitude/longitude to N-vector.
//

static void v3_make( double lat, double lon, V3 v)
{
   v[0] = cos( lat) * cos( lon);
   v[1] = cos( lat) * sin( lon);
   v[2] = sin( lat);
}

//
// Range in km between two points v1 and v2.
//

static inline double v3_range( V3 const v1, V3 const v2)
{
   V3 xp; v3_cross( v1, v2, xp);
   return atan2( v3_mag( xp), v3_dot( v1, v2)) * EARTH_RAD;
}

//
// Bearing (radians) of forepoint v2 from standpoint v1.
//

static double v3_bearing( V3 const v1, V3 const v2)
{
   V3 x1;  v3_unit_normal_to( V3north, v1, x1);
   V3 x2;  v3_unit_normal_to( v2, v1, x2);

   if (x1[1] > 0.99999 && x2[1] > 0.99999) return 0;
   if (x1[1] > 0.99999 && x2[1] < -0.99999) return M_PI;

   V3 xp;  v3_cross( x1, x2, xp);
   V3 xpn; v3_copy( xp, xpn); v3_normalise( xpn);

   return -v3_dot( v1, xpn) * atan2( v3_mag( xp), v3_dot( x1, x2));
}

//
// Compute forepoint vf from standpoint vs along initial bearing b radians.
// Distance 'a' is in units of Earth radii.
//

static void destination_point( V3 const vs, double b, double a, V3 vf)
{
   A3 r0;  a3_rot( vs, -b, r0);
   V3 v0;  v3_transform( V3north, r0, v0);
   V3 n0;  v3_unit_normal_to( vs, v0, n0);
   A3 r1;  a3_rot( n0, a, r1);
   v3_transform( vs, r1, vf);
}

//
// Parse a latitude or longitude string and return the angle in radians.
//

static double parse_coord( char const *str)
{
   double a;
   char temp[50], *p;

   strcpy( temp, str);
   int sign = 1;

   if ((p = strchr( temp, 'S')) ||
       (p = strchr( temp, 'W')) ||
       (p = strchr( temp, 's')) ||
       (p = strchr( temp, 'w')))
   {
      *p = 0;  sign = -sign;
   }

   if ((p = strchr( temp, 'N')) ||
       (p = strchr( temp, 'E')) ||
       (p = strchr( temp, 'n')) ||
       (p = strchr( temp, 'e')))
   {
      *p = 0;
   }

   int fd, fm;
   double fs;

   if (sscanf( temp, "%d:%d:%lf", &fd, &fm, &fs) == 3 ||
       sscanf( temp, "%d:%d", &fd, &fm) == 2)
      a = fd + fm/60.0 + fs/3600.0;
   else
   if (sscanf( temp, "%lf", &a) != 1)
      VT_bailout( "bad coordinate [%s]", temp);

   return a * sign * M_PI/180;
}

///////////////////////////////////////////////////////////////////////////////
// Sun Position & Path Illumination                                          //
///////////////////////////////////////////////////////////////////////////////

//
// Parse opt_radec (given by -R option) and convert RA/Dec to radians.
//

static void parse_radec( double *ra, double *dec)
{
   if (strlen( opt_radec) > 49) VT_bailout( "invalid -R argument");
   char temp[50];
   strcpy( temp, opt_radec);

   char *p = strchr( temp, ',');
   if (!p) VT_bailout( "invalid -R argument");

   *p++ = 0;    // Pointing to dec

   *ra = atof( temp) * M_PI/180;
   *dec = atof( p) * M_PI/180;
}

//
// Equatorial coordinates of the Sun.
//

struct RADEC {
   double ra;               // Right ascension, radians
   double dec;              // Declination, Radians
   double sindec;
   double cosdec;
   double day;
   double hour;
};

//
// Compute equatorial Ra/Dec for the Sun or whatever target is given by
// opt_radec, at the given UT second.
//
// This can be called just once at the start of a stroke solution and the
// radec then used for Sun position as needed.
//

static void init_radec( uint32_t ut, struct RADEC *p)
{
   p->day = (ut - 946728000)/86400.0;
   p->hour = (ut % 86400)/3600.0;

   // If opt_radec has been set by a -R option, just parse and use that
   if (opt_radec)
   {
      parse_radec( &p->ra, &p->dec);
      p->sindec = sin( p->dec);
      p->cosdec = cos( p->dec);
      return;
   }

   double JD = ut/86400.0 + 2440587.5;   // Julian day
   double TU = JD - 2451545.0;

   double e_of_e = 23.43691 * M_PI/180;  // Obliquity of the ecliptic, radians

   double ERA = 2 * M_PI * (0.779057273 + 1.002737812 * TU);

   double GST = ERA; // - EPREC(t)   // Radians

   GST = GST - 2 * M_PI * (int)(GST/(2*M_PI));

   //
   // RA and DEC of the Sun.
   //

   double mean_longitude = 280.46 + 0.9856474 * TU;   // Degrees

   double mean_anomaly = 357.528 + 0.9856003 * TU;    // Degrees

   double ecliptic_longitude =
        mean_longitude +
        1.915 * sin( mean_anomaly * M_PI/180) +
        0.020 * sin( 2 * mean_anomaly * M_PI/180);     // Degrees

   // RA and DEC in radians

   p->ra = atan2( cos( e_of_e) * sin( ecliptic_longitude * M_PI/180),
                      cos( ecliptic_longitude * M_PI/180));

   p->dec = asin( sin( e_of_e) * sin( ecliptic_longitude * M_PI/180));

   p->sindec = sin( p->dec);
   p->cosdec = cos( p->dec);
}

//
// Compute the horizontal coordinates of the Sun or whatever target is given
// by radec, from the standpoint 'v'.
//

static void astropos0( V3 const v, struct RADEC const *radec,
                     double *az, double *el)
{
   //
   // Local coordinates.
   //

   double lat = atan2( v[2], sqrt(v[0] * v[0] + v[1] * v[1]));
   double lon = atan2( v[1], v[0]);

   if (lat > M_PI/2)
   {
      lat = M_PI - lat;
      lon += M_PI;
   }
   else
   if (lat < -M_PI/2)
   {
      lat = -M_PI - lat;
      lon += M_PI;
   }

   lon = constrain( lon, -M_PI, M_PI);
  
   //
   // Local time and hour angle.
   //
 
   double LST = 100.46 * M_PI/180 +
                0.985647 * radec->day * M_PI/180 +
                lon +
                radec->hour * M_PI/12;

   double LHA = LST - radec->ra;   // Radians

   // http://www.stargazing.net/kepler/altaz.html

   double sinlat = sin( lat);
   double coslat = cos( lat);

   double alt = asin( radec->sindec * sinlat +
                      radec->cosdec * coslat * cos(LHA));
   if (el) *el = alt;

   if (az)
   {
      double a = acos( (radec->sindec - sin( alt) * sinlat)/cos( alt)/coslat);
      if (sin(LHA) > 0) a = 2 * M_PI - a;
      *az = a;
   }
}

//
// Compute Sun azimuth and elevation from the standpoint 'v' at time 'T'.
//

static void astropos1( V3 const v, timestamp T, double *solaz, double *solel)
{
   struct RADEC radec;
   init_radec( timestamp_secs( T), &radec);

   astropos0( v, &radec, solaz, solel);
}

//
// Compute the illumination factor of the path from v1 to v2 with the Sun
// at the given radec.
//
// Return value is the fraction 0.0 to 1.0 of the path which has the Sun
// above the horizon.   Return value is quantised in 0.05 steps.
//
// If the shortsun flag is set, a path which is dark at both ends, or light
// at both ends, is assumed the same throughout.
//

static double illum_horizon( V3 const v1, V3 const v2,
                             struct RADEC const *radec)
{
   int n, k = 0;
   double el;

   astropos0( v1, radec, NULL, &el);
   if (el > 0) k++;
   astropos0( v2, radec, NULL, &el);
   if (el > 0) k++;

   if (shortsun)
   {
      if (k == 2) return 1;
      if (k == 0) return 0;
   }

   for (n = 1; n < 19; n++)
   {
      double s = n/19.0;

      V3 t1; v3_scale( v1, 1 - s, t1);
      V3 t2; v3_scale( v2, s, t2);

      V3 t3; v3_add( t1, t2, t3); v3_normalise( t3);
      
      astropos0( t3, radec, NULL, &el);
      if (el > 0) k++;
   }

   return k/20.0;
}
 
static double illum_vertical( V3 const v1, V3 const v2,
                              struct RADEC const *radec)
{
   int n;
   double el;

   double sum = 0;

   for (n = 0; n < 20; n++)
   {
      double s = (n + 0.5)/20.0;

      V3 t1; v3_scale( v1, 1 - s, t1);
      V3 t2; v3_scale( v2, s, t2);

      V3 t3; v3_add( t1, t2, t3); v3_normalise( t3);
      
      astropos0( t3, radec, NULL, &el);
      if (el > 0) sum += sin( el);
   }

   return sum/20.0;
}
 
static double path_illumination0( V3 const v1, V3 const v2,
                                  struct RADEC const *radec)
{
   return illum_type ? illum_vertical( v1, v2, radec) :
                       illum_horizon( v1, v2, radec);
}

static double path_illumination1( V3 const v1, V3 const v2, timestamp T)
{
   struct RADEC radec;
   init_radec( timestamp_secs( T), &radec);

   return path_illumination0( v1, v2, &radec);
}

///////////////////////////////////////////////////////////////////////////////
// Spots File                                                                //
///////////////////////////////////////////////////////////////////////////////

//
// Load the 'spots' file into memory.  The spots file is a plain text file
// which maps symbolic site names into geographic coordinates.
//

static struct SPOT {
   char *name;
   V3 v;
}
 *spots = NULL;

static int nspots = 0;

static void alloc_spot( char const *name, V3 v)
{
   VT_report( 3, "alloc spot [%s] %s", name, v3_string( v, NULL));

   spots = VT_realloc( spots, (nspots+1) * sizeof( struct SPOT));
   spots[nspots].name = strdup( name);
   v3_copy( v, spots[nspots].v);
   nspots++;
}

static int load_spots_file( char const *filename)
{
   FILE *f = fopen( filename, "r");
   if (!f) return FALSE;

   int lino = 0;

   char temp[500], *p, *q, *s;
   while (fgets( temp, 500, f))
   {
      lino++;

      p = strchr( temp, '\r'); if (p) *p = 0;
      p = strchr( temp, '\n'); if (p) *p = 0;
      p = strchr( temp, ';'); if (p) *p = 0;

      p = temp;
      while (isspace( *p)) p++;
      if (!*p) continue;

      if (isalpha( *p))
      {
         VT_report( 0, "error in %s, line %d", filename, lino);
         continue;
      }

      for (q = p; *q && !isspace( *q); ) q++;
      if (*q) *q++ = 0;
      if ((s = strchr( p, ',')) == NULL)
      {
         VT_report( 0, "error in %s, line %d", filename, lino);
         continue;
      }

      *s++ = 0;
      double lat = parse_coord( p);
      double lon = parse_coord( s);
      V3 v;
      v3_make( lat, lon, v);

      p = q;
      int n = 0;
      while (1)
      {
         while (isspace( *p)) p++;
         if (!*p) break;
         
         for (q = p; *q && !isspace( *q); ) q++;
         if (*q) *q++ = 0;

         alloc_spot( p, v);
         n++;
         p = q;
      }

      if (!n)
      {
         VT_report( 0, "error in %s, line %d", filename, lino);
         continue;
      }
   }

   fclose( f);
   VT_report( 2, "loaded spots file [%s], %d entries", filename, nspots);
   return TRUE;
}

//
// Attempt to load 'spots' file from the current directory.  If that fails,
// try from the user's home directory.
//

static void load_spots( void)
{
   if (nspots) return;

   if (load_spots_file( "./spots")) return;
 
   char *home = getenv( "HOME");
   if (!home) return;

   char *path;
   if (asprintf( &path, "%s/spots", home) < 0 || !path) return;
   load_spots_file( path);
   free( path);
}

//
// A case insensitive search for a symbolic site name in the spots table,
// returning TRUE if found, with 'v' filled in with the location.
//

static int lookup_spot( char const *s, V3 v)
{
   int i;

   for (i = 0; i < nspots; i++)
      if (!strcasecmp( spots[i].name, s))
      {
         v3_copy( spots[i].v, v);
         return TRUE;
      }

   return FALSE;
}

///////////////////////////////////////////////////////////////////////////////
// Sites                                                                     //
///////////////////////////////////////////////////////////////////////////////

//
// A list of sites referred to in measurement input records.
//

static struct SITE {
   V3 v;                 // Location of the site
   char *name;           // Symbolic name of the site.  If none, then the
                         // string representation of the lat/lon.
   char *file;           // Source file for sferic TOGAs when matching

   int n_sferic;         // Number of sferics supplied for matching
   int n_matched;        // Number of sferics successfully matched

   double tetotal;       // Overall sum of timing residuals, seconds
}
 sites[MAXSITES];

static int nsites = 0;

static int minsites = 0;     // Set by -n option
static int site_width = 0;   // Max length of site names

//
// Add a site to the list, if not already listed.  'name' is optional.
//

static struct SITE *add_site( V3 const v, char const *name)
{
   int i;
   for (i = 0; i < nsites; i++) if (v3_equal( sites[i].v, v)) return sites + i;

   if (nsites == MAXSITES) VT_bailout( "too many sites, max %d", MAXSITES);

   v3_copy( v, sites[nsites].v);
   sites[nsites].name = strdup( name);
   if (strlen( name) > site_width) site_width = strlen( name);

   return sites + nsites++;
}

static void clear_sites( void)
{
   int i;
   for (i = 0; i < nsites; i++)
      if (sites[i].name) free( sites[i].name);
   nsites = 0;
}

//
// Parse a location.  This can be a symbolic place name and the coordinates
// are looked up in the spots file, or a latitude/longitude pair.
//
// Returns a pointer into the sites[] array.
//

static struct SITE *parse_latlon( char const *s)
{
   char temp[150], *p;

   strcpy( temp, s);

   V3 v;
   if (isalpha( s[0]))
   {
      if (lookup_spot( s, v)) return add_site( v, temp);
      VT_bailout( "no definition in spots file for [%s]", s);
   }

   if ((p = strchr( temp, ',')) == NULL)
       VT_bailout( "bad lat/long [%s]", s);
   *p++ = 0;

   double lat = parse_coord( temp);
   double lon = parse_coord( p);

   v3_make( lat, lon, v);
   return add_site( v, temp);
}

///////////////////////////////////////////////////////////////////////////////
// Input Measurements                                                        //
///////////////////////////////////////////////////////////////////////////////

//
// Measurement set structure.
//

struct MSET {

   //
   // Table of locations and arrival times, usually TOGAs but could also be
   // trigger times.
   //
   
   struct AT {
      struct SITE *site;
      timestamp toga;
      double sigma;
   } ats[MAXSITES];
   
   int nats;    // Number of elements of ats[]
   
   //
   // Table of locations and bearings.
   //
   
   struct BR {
      struct SITE *site;
      double bearing;
      double sigma;
   } brs[MAXSITES];
   
   int nbrs;    // Number of elements of brs[]
   
   //
   // Table of arrival time differences.
   //
   
   struct ATD {
      struct SITE *site1, *site2;
      double atd;                     // Arrival time difference, seconds
      double sigma;                   // Combined sigma of the two arrival times
      double base;                    // Baseline distance, km
   }
    atds[MAXSITES];
   
   int natd;    // Number of elements of atd[]
   
   char *ident;     // User-supplied ident for the measurement set

   struct RADEC radec;
   int use_illum;
};
 
static void init_measurement_set( struct MSET *m)
{
   memset( m, 0, sizeof( struct MSET));
}

static void reset_measurement_set( struct MSET *m)
{
   m->nats = 0;
   m->nbrs = 0;
   m->natd = 0;
   m->use_illum = 0;
   if (m->ident) { free( m->ident); m->ident = NULL; }
}

#if 0   // Diagnostic use only

static void print_mset( struct MSET *m)
{
   int i;

   VT_printf( "nats=%d\n", m->nats);
   for (i = 0; i < m->nats; i++)
      VT_printf( "  %s %.6Lf %.6f\n",
                 m->ats[i].site->name, m->ats[i].toga, m->ats[i].sigma);
   VT_printf( "natd=%d\n", m->natd);
   for (i = 0; i < m->natd; i++)
      VT_printf( " %s->%s %.6f\n",
                  m->atds[i].site1->name, m->atds[i].site2->name, 
                  m->atds[i].atd);
} 

#endif

static void parse_measurement( struct MSET *m, char *arg)
{
   char *p1 = strdup( arg), *p2, *p3, *p4 = NULL, *p5 = NULL;

   if (p1[1] != '/') VT_bailout( "cannot parse measurement [%s]", arg);
   p2 = p1 + 2;
   p3 = strchr( p2, '/');
   if (p3)
   {
      *p3++ = 0;
      p4 = strchr( p3, '/');
      if (p4)
      {
         *p4++ = 0;
         p5 = strchr( p4, '/');
         if (p5) *p5++ = 0;
      }
   }

   switch (*p1)
   {
      case 'T':  m->ats[m->nats].site = parse_latlon( p2);
                 if (!p3) VT_bailout( "missing timestamp in [%s]", arg);
                 m->ats[m->nats].toga = VT_parse_timestamp( p3);
                 m->ats[m->nats].sigma = p4 ? atof( p4) : DEFAULT_AT_SIGMA;
                 m->nats++;
                 break;

      case 'B':  m->brs[m->nbrs].site = parse_latlon( p2);
                 if (!p3) VT_bailout( "missing bearing in [%s]", arg);
                 m->brs[m->nbrs].bearing = atof( p3) * M_PI/180;
                 m->brs[m->nbrs].sigma = (p4 ? atof( p4) : 5) * M_PI/180;
                 m->nbrs++;
                 break;

      case 'A':  m->atds[m->natd].site1 = parse_latlon( p2);
                 if (!p3) VT_bailout( "missing location in [%s]", arg);
                 m->atds[m->natd].site2 = parse_latlon( p3);
                 if (!p4) VT_bailout( "missing ATD in [%s]", arg);
                 m->atds[m->natd].atd = atof( p4);
                 m->atds[m->natd].sigma = p5 ? atof( p5) : DEFAULT_AT_SIGMA;
                 m->natd++;
                 break;
                 
      case 'I':  if (m->ident) free( m->ident);
                 m->ident = strdup( p2);
                 break;

      default: VT_bailout( "unknown measurement type [%s]", arg);
   }

   free( p1);
}

///////////////////////////////////////////////////////////////////////////////
// Propagation MODEL_FIXED                                                   //
///////////////////////////////////////////////////////////////////////////////

//
// Applies a fixed velocity factor to all paths.  This is the default model.
//  

static double vfmodel_fixed( double azimuth, double range, double illum)
{
   return fixed_VF;
}

///////////////////////////////////////////////////////////////////////////////
// Propagation MODEL_ELF1                                                    //
///////////////////////////////////////////////////////////////////////////////

//
// Return velocity factor for an ELF path with the given illumination.
//

static double model_elf1_vftable[21] = {
  0.8834,  0.8761,  0.8752,  0.8735,  0.8706,  0.8671,  0.8631,  0.8546,
  0.8461,  0.8432,  0.8287,  0.8231,  0.8117,  0.8064,  0.8043,  0.7972,
  0.7933,  0.7933,  0.7896,  0.7905 };

static float model_elf1_offsets[21] = {
  2.586e-03,  2.964e-03,  3.071e-03,  3.197e-03,  3.307e-03,  3.398e-03,
  3.559e-03,  3.487e-03,  3.496e-03,  3.755e-03,  3.480e-03,  3.530e-03,
  3.270e-03,  3.276e-03,  3.455e-03,  3.289e-03,  3.246e-03,  3.409e-03,
  3.206e-03,  3.242e-03 };
                               

static double vfmodel_elf1( double azimuth, double range, double illum)
{
   int ib = round( illum * 20);
   if (ib > 20) ib = 20;

   return model_elf1_vftable[ib];
}

static double vfmodel_elf1_offset( double azimuth, double range, double illum)
{
   int ib = round( illum * 20);
   if (ib > 20) ib = 20;

   return model_elf1_offsets[ib];
}

///////////////////////////////////////////////////////////////////////////////
// Propagation MODEL_ELF2                                                    //
///////////////////////////////////////////////////////////////////////////////

//
// A 2D velocity factor lookup with 20 range bands of 0 to 19 Mm and
// 11 illumination bands 0.0 to 1.0 in 0.1 steps
//

static float model_elf2_vftable[20][11] = {
  { 0.416,0.430,0.457,0.479,0.406,0.367,0.373,0.393,0.345,0.622,0.800 },  // 0
  { 0.606,0.686,0.731,0.706,0.714,0.722,0.731,0.725,0.734,0.800,0.838 },  // 1
  { 0.699,0.730,0.756,0.759,0.760,0.761,0.769,0.772,0.777,0.796,0.800 },  // 2
  { 0.764,0.758,0.779,0.789,0.793,0.790,0.798,0.796,0.800,0.815,0.800 },  // 3
  { 0.794,0.778,0.783,0.790,0.799,0.797,0.804,0.801,0.807,0.817,0.800 },  // 4
  { 0.803,0.775,0.772,0.780,0.787,0.787,0.791,0.794,0.796,0.800,0.800 },  // 5
  { 0.808,0.767,0.763,0.760,0.769,0.776,0.778,0.781,0.795,0.796,0.800 },  // 6
  { 0.821,0.779,0.766,0.767,0.767,0.778,0.787,0.791,0.798,0.807,0.800 },  // 7
  { 0.831,0.784,0.774,0.765,0.766,0.773,0.782,0.786,0.794,0.803,0.800 },  // 8
  { 0.832,0.794,0.784,0.775,0.770,0.777,0.785,0.786,0.794,0.814,0.800 },  // 9
  { 0.835,0.802,0.790,0.779,0.776,0.779,0.783,0.781,0.786,0.800,0.800 },  // 10
  { 0.834,0.800,0.787,0.780,0.776,0.778,0.781,0.783,0.787,0.800,0.800 },  // 11
  { 0.836,0.809,0.797,0.788,0.785,0.781,0.782,0.783,0.801,0.800,0.800 },  // 12
  { 0.839,0.812,0.797,0.790,0.785,0.783,0.782,0.782,0.800,0.800,0.800 },  // 13
  { 0.837,0.811,0.797,0.788,0.786,0.787,0.785,0.797,0.800,0.800,0.800 },  // 14
  { 0.834,0.807,0.798,0.787,0.789,0.782,0.787,0.800,0.800,0.800,0.800 },  // 15
  { 0.827,0.809,0.802,0.794,0.787,0.778,0.784,0.800,0.800,0.800,0.800 },  // 16
  { 0.818,0.808,0.812,0.804,0.806,0.803,0.800,0.800,0.800,0.800,0.800 },  // 17
  { 0.830,0.821,0.817,0.821,0.811,0.800,0.800,0.800,0.800,0.800,0.800 },  // 18
  { 0.831,0.811,0.808,0.794,0.820,0.800,0.800,0.800,0.800,0.800,0.800 }   // 19
};

static double vfmodel_elf2( double azimuth, double range, double illum)
{
   double mm = range / 1000;
   int mi = floor( mm);
   double mx = mm - mi;

   int ii = floor( illum * 10);
   double ix = illum * 10 - ii;

   double v0, v1;

   // Corner and edge cases

   if (mi >= 19 && ii >= 10) return model_elf2_vftable[19][10];

   if (mi >= 19)
   {
      v0 = model_elf2_vftable[19][ii];
      v1 = model_elf2_vftable[19][ii+1];
      return v0 + (v1 - v0) * ix;
   }

   if (ii >= 10)
   {
      v0 = model_elf2_vftable[mi][10];
      v1 = model_elf2_vftable[mi+1][10];
      return v0 + (v1 - v0) * mx;
   }

   // 2D interpolation

   v0 = model_elf2_vftable[mi][ii];
   v1 = model_elf2_vftable[mi+1][ii];
   double va = v0 + (v1 - v0) * mx;
 
   v0 = model_elf2_vftable[mi][ii+1];
   v1 = model_elf2_vftable[mi+1][ii+1];
   double vb = v0 + (v1 - v0) * mx;

   return va + (vb - va) * ix;
}

static void vfmodel_elf2_dump( void)
{
   int mi, ii;
   for (mi = 0; mi < 20; mi++)
      for (ii = 0; ii <= 10; ii++)
         VT_printf( "%d %.1f %.3f\n", mi, ii/10.0, model_elf2_vftable[mi][ii]);
}

static void vfmodel_elf2_load( void)
{
   if (!vfmodel_loadname) return;   

   FILE *f = fopen( vfmodel_loadname, "r");
   if (!f) VT_bailout( "cannot read %s", vfmodel_loadname);

   memset( model_elf2_vftable, 0, sizeof( model_elf2_vftable));

   int mm, lino=0;
   double illum, vf;

   while( fscanf( f, "%d %lg %lg\n", &mm, &illum, &vf) == 3)
   {
      lino++;

      if (mm < 0 || mm > 19)
         VT_bailout( "invalid Mm %d in %s at line %d",
                      mm, vfmodel_loadname, lino);
     
      int ni = floor( illum * 10);
      if (ni < 0 || ni > 10)
         VT_bailout( "invalid illumination factor in %s at line %d",
                      vfmodel_loadname, lino);

      if (vf <= 0 || vf > 1.0)
         VT_bailout( "invalid vf %.4f in %s at line %d",
                vf, vfmodel_loadname, lino);

      model_elf2_vftable[mm][ni] = vf;
   }

   fclose( f);

   VT_report( 1, "loaded %d lines from %s", lino, vfmodel_loadname);

   for (mm = 0; mm <= 19; mm++)
   {
      int ni;
      for( ni = 0; ni <= 10; ni++)
         if (!model_elf2_vftable[mm][ni])
            VT_bailout( "vfmodel_loadname missing Mm=%d illum=%.1f",
                       mm, ni/10.0);
   }
}

///////////////////////////////////////////////////////////////////////////////
// Propagation MODEL_ELF3                                                    //
///////////////////////////////////////////////////////////////////////////////

//
// A 2D model using illumination factor and propagation direction.
//

static float model_elf3_vftable[21][4] = { 
  //   North     East    South      West    Illumination
   {  0.8648,  0.8881,  0.8459,  0.9198 },  //  0.00
   {  0.8621,  0.8810,  0.8445,  0.9084 },  //  0.05
   {  0.8684,  0.8811,  0.8418,  0.9014 },  //  0.10
   {  0.8675,  0.8808,  0.8402,  0.8987 },  //  0.15
   {  0.8719,  0.8737,  0.8383,  0.8927 },  //  0.20
   {  0.8656,  0.8745,  0.8449,  0.8909 },  //  0.25
   {  0.8656,  0.8714,  0.8440,  0.8854 },  //  0.30
   {  0.8632,  0.8610,  0.8387,  0.8781 },  //  0.35
   {  0.8581,  0.8544,  0.8339,  0.8710 },  //  0.40
   {  0.8535,  0.8435,  0.8361,  0.8639 },  //  0.45
   {  0.8393,  0.8347,  0.8207,  0.8409 },  //  0.50
   {  0.8322,  0.8272,  0.8018,  0.8283 },  //  0.55
   {  0.8227,  0.8142,  0.7727,  0.8182 },  //  0.60
   {  0.8170,  0.8031,  0.7663,  0.8098 },  //  0.65
   {  0.8105,  0.8009,  0.7632,  0.8099 },  //  0.70
   {  0.8041,  0.7941,  0.7589,  0.8007 },  //  0.75
   {  0.7967,  0.7957,  0.7583,  0.7985 },  //  0.80
   {  0.7980,  0.7945,  0.7583,  0.7952 },  //  0.85
   {  0.7886,  0.7962,  0.7608,  0.7942 },  //  0.90
   {  0.7902,  0.7948,  0.7608,  0.7958 },  //  0.95
   {  0.7744,  0.7871,  0.7723,  0.7934 }   //  1.00
};

static double vfmodel_elf3( double azimuth, double range, double illum)
{
   int ib = round( illum * 20);
   if (ib > 20) ib = 20;

   if (azimuth < 0) azimuth += 2 * M_PI;

   int ab1 = floor( azimuth / (M_PI/2));
   if (ab1 < 0) ab1 = 0;

   int ab2 = (ab1 + 1) % 4;

   double q1 = model_elf3_vftable[ib][ab1];
   double q2 = model_elf3_vftable[ib][ab2]; 

   double f = (azimuth - ab1 * M_PI/2) / (M_PI/2);
   return q1 + (q2 - q1) * f;
}

static float model_elf3_offsets[21][4] = {
  //   North     East    South      West    Illumination
   {  2.211e-03,  2.651e-03,  1.907e-03,  3.283e-03 },  //  0.00
   {  2.831e-03,  2.867e-03,  2.679e-03,  3.535e-03 },  //  0.05
   {  3.228e-03,  3.076e-03,  2.619e-03,  3.407e-03 },  //  0.10
   {  3.389e-03,  3.275e-03,  2.554e-03,  3.540e-03 },  //  0.15
   {  3.841e-03,  3.172e-03,  2.546e-03,  3.499e-03 },  //  0.20
   {  3.775e-03,  3.508e-03,  3.065e-03,  3.680e-03 },  //  0.25
   {  4.103e-03,  3.666e-03,  3.403e-03,  3.805e-03 },  //  0.30
   {  4.301e-03,  3.472e-03,  3.328e-03,  3.833e-03 },  //  0.35
   {  4.616e-03,  3.445e-03,  3.420e-03,  3.846e-03 },  //  0.40
   {  4.809e-03,  3.263e-03,  3.931e-03,  3.934e-03 },  //  0.45
   {  4.563e-03,  3.211e-03,  3.565e-03,  3.303e-03 },  //  0.50
   {  4.541e-03,  3.202e-03,  2.808e-03,  3.031e-03 },  //  0.55
   {  4.323e-03,  2.882e-03,  1.358e-03,  2.971e-03 },  //  0.60
   {  4.316e-03,  2.582e-03,  1.421e-03,  2.813e-03 },  //  0.65
   {  4.214e-03,  2.767e-03,  1.538e-03,  3.186e-03 },  //  0.70
   {  4.107e-03,  2.602e-03,  1.448e-03,  2.876e-03 },  //  0.75
   {  3.855e-03,  2.930e-03,  1.315e-03,  3.035e-03 },  //  0.80
   {  4.071e-03,  3.072e-03,  1.576e-03,  3.086e-03 },  //  0.85
   {  3.507e-03,  3.201e-03,  1.779e-03,  3.114e-03 },  //  0.90
   {  3.609e-03,  3.113e-03,  1.771e-03,  3.160e-03 },  //  0.95
   {  2.521e-03,  2.691e-03,  2.301e-03,  2.890e-03 }   //  1.00
};

static double vfmodel_elf3_offset( double azimuth, double range, double illum)
{
   int ib = round( illum * 20);
   if (ib > 20) ib = 20;

   if (azimuth < 0) azimuth += 2 * M_PI;

   int ab1 = floor( azimuth / (M_PI/2));
   if (ab1 < 0) ab1 = 0;

   int ab2 = (ab1 + 1) % 4;

   double q1 = model_elf3_offsets[ib][ab1];
   double q2 = model_elf3_offsets[ib][ab2]; 

   double f = (azimuth - ab1 * M_PI/2) / (M_PI/2);
   return q1 + (q2 - q1) * f;
}

///////////////////////////////////////////////////////////////////////////////
// Propagation MODEL_ELF4                                                    //
///////////////////////////////////////////////////////////////////////////////

#if 0  // +10 deg elevation

static double model_elf4_day[36] = {
   0.7757, 0.7663, 0.7632, 0.7617, 0.7660, 0.7857, 0.7901, 0.7822, 0.7276,
   0.7479, 0.7401, 0.7605, 0.8090, 0.7656, 0.7631, 0.7458, 0.7661, 0.7662,
   0.7706, 0.7595, 0.7497, 0.7630, 0.7488, 0.7795, 0.7883, 0.7890, 0.7345,
   0.7482, 0.7768, 0.8056, 0.7916, 0.7737, 0.7609, 0.7739, 0.7738, 0.7741
};

static double model_elf4_night[36] = {
   0.8445, 0.8435, 0.8387, 0.8387, 0.8572, 0.8699, 0.8879, 0.9065, 0.8301,
   0.8570, 0.8351, 0.8492, 0.9022, 0.8365, 0.8359, 0.8141, 0.8278, 0.8392,
   0.8326, 0.8359, 0.8270, 0.8199, 0.8277, 0.8691, 0.9160, 0.8841, 0.8271,
   0.8197, 0.8381, 0.9048, 0.9055, 0.8567, 0.8454, 0.8676, 0.8624, 0.8609
};

static double model_elf4_offset = 2.554e-3;
static int model_elf4_sunel = +10 * M_PI/180;
#endif

#if 1  // +5 deg elevation

static double model_elf4_day[36] = {
   0.7779, 0.7694, 0.7662, 0.7654, 0.7702, 0.7885, 0.7928, 0.7841, 0.7299,
   0.7515, 0.7433, 0.7631, 0.8106, 0.7681, 0.7654, 0.7492, 0.7675, 0.7678,
   0.7722, 0.7611, 0.7515, 0.7645, 0.7507, 0.7820, 0.7913, 0.7916, 0.7391,
   0.7480, 0.7800, 0.8069, 0.7944, 0.7750, 0.7637, 0.7771, 0.7765, 0.7767
};

static double model_elf4_night[36] = {
   0.8512, 0.8523, 0.8463, 0.8463, 0.8632, 0.8773, 0.8905, 0.9117, 0.8403,
   0.8663, 0.8426, 0.8544, 0.9049, 0.8417, 0.8413, 0.8220, 0.8371, 0.8458,
   0.8368, 0.8417, 0.8396, 0.8286, 0.8366, 0.8745, 0.9224, 0.8916, 0.8307,
   0.8281, 0.8418, 0.9149, 0.9111, 0.8623, 0.8516, 0.8756, 0.8716, 0.8691
};

static double model_elf4_offset = 2.578e-3;
static int model_elf4_sunel = +5 * M_PI/180;
#endif

#if 0  // Zero deg elevation

static double model_elf4_day[36] = {
   0.7800, 0.7728, 0.7693, 0.7698, 0.7742, 0.7918, 0.7941, 0.7877, 0.7307,
   0.7550, 0.7451, 0.7659, 0.8121, 0.7704, 0.7675, 0.7536, 0.7706, 0.7699,
   0.7741, 0.7628, 0.7546, 0.7664, 0.7532, 0.7841, 0.7946, 0.7933, 0.7436,
   0.7491, 0.7806, 0.8093, 0.7968, 0.7765, 0.7663, 0.7805, 0.7799, 0.7798
};

static double model_elf4_night[36] = {
   0.8582, 0.8605, 0.8547, 0.8559, 0.8699, 0.8850, 0.8940, 0.9163, 0.8554,
   0.8749, 0.8537, 0.8592, 0.9081, 0.8478, 0.8469, 0.8315, 0.8481, 0.8527,
   0.8428, 0.8480, 0.8523, 0.8398, 0.8451, 0.8803, 0.9291, 0.8988, 0.8347,
   0.8399, 0.8464, 0.9258, 0.9165, 0.8684, 0.8583, 0.8834, 0.8812, 0.8780
};

static double model_elf4_offset = 2.600e-3;
static int model_elf4_sunel = 0 * M_PI/180;
#endif

#if 0   // -10 deg elevation

static double model_elf4_day[36] = {
   0.7868, 0.7794, 0.7760, 0.7808, 0.7837, 0.7979, 0.7959, 0.7974, 0.7426,
   0.7630, 0.7512, 0.7722, 0.8145, 0.7757, 0.7722, 0.7645, 0.7823, 0.7763,
   0.7811, 0.7684, 0.7614, 0.7714, 0.7589, 0.7889, 0.8003, 0.8001, 0.7478,
   0.7542, 0.7822, 0.8139, 0.8024, 0.7805, 0.7729, 0.7879, 0.7890, 0.7874
};

static double model_elf4_night[36] = {
   0.8709, 0.8768, 0.8708, 0.8761, 0.8854, 0.9035, 0.9040, 0.9223, 0.8838,
   0.9008, 0.8738, 0.8714, 0.9159, 0.8610, 0.8607, 0.8498, 0.8649, 0.8670,
   0.8544, 0.8625, 0.8749, 0.8634, 0.8598, 0.8936, 0.9415, 0.9152, 0.8537,
   0.8604, 0.8682, 0.9462, 0.9283, 0.8827, 0.8740, 0.8978, 0.8982, 0.8930
};

static double model_elf4_offset = 2.658e-3;
static int model_elf4_sunel = -10 * M_PI/180;
#endif

#if 0   // -20 deg elevation

static double model_elf4_day[36] = {
   0.7926, 0.7869, 0.7834, 0.7864, 0.7918, 0.8060, 0.7999, 0.8075, 0.7589,
   0.7710, 0.7604, 0.7802, 0.8166, 0.7820, 0.7799, 0.7742, 0.7908, 0.7849,
   0.7900, 0.7759, 0.7683, 0.7770, 0.7648, 0.7949, 0.8054, 0.8070, 0.7570,
   0.7598, 0.7855, 0.8183, 0.8081, 0.7868, 0.7814, 0.7939, 0.7992, 0.7970
};

static double model_elf4_night[36] = {
   0.8824, 0.8921, 0.8874, 0.8946, 0.9040, 0.9206, 0.9165, 0.9311, 0.9061,
   0.9340, 0.8901, 0.8881, 0.9250, 0.8749, 0.8752, 0.8650, 0.8761, 0.8774,
   0.8672, 0.8774, 0.8964, 0.8901, 0.8737, 0.9068, 0.9549, 0.9370, 0.8737,
   0.8903, 0.8975, 0.9704, 0.9410, 0.9007, 0.8915, 0.9140, 0.9135, 0.9097
};

static double model_elf4_offset = 2.727e-3;
static int model_elf4_sunel = -20 * M_PI/180;
#endif

static double model_elf4_tprop( V3 v1, V3 v2, struct RADEC const *radec)
{
   double angle = v3_angle( v1, v2);
   double step = 4.4966 * M_PI/180;   // Radians for a 500km step

   // Adjust step size to give an integer number of points on the GC

   int istep = 0.5 + angle/step;
   step = angle/istep;
   double seglen = step * EARTH_RAD;   // Segment length in km

   // The great circle between the two sites.

   V3 gc; v3_unit_normal_to( v1, v2, gc);

   // Loop over the segments, adding up the propagation delay

   double tprop = model_elf4_offset;

   int i;
   for (i = 0; i < istep; i++)
   {
      A3 r;
      a3_rot( gc, i * step, r);
      V3 p1;  v3_transform( v1, r, p1);
      a3_rot( gc, (i + 1) * step, r);
      V3 p2;  v3_transform( v1, r, p2);

      double azim = v3_bearing( p1, p2);
      if (azim < 0) azim += 2 * M_PI;
      int ab = floor( (azim * 180/M_PI)/10 + 0.5);
      if (ab >= 36) ab = 0;

      double solel;
      V3 pmid;  v3_add( p1, p2, pmid); v3_normalise( pmid);
      astropos0( pmid, radec, NULL, &solel);
      
      double vf = solel >= model_elf4_sunel ? model_elf4_day[ab]
                                            : model_elf4_night[ab];
      tprop += seglen / (vf * 300e3);
   }

   return tprop;
}

///////////////////////////////////////////////////////////////////////////////
// Propagation MODEL_VLF0                                                    //
///////////////////////////////////////////////////////////////////////////////

static double vfmodel_vlf0( double azimuth, double range, double illum)
{ 
   double vf;

   if (range > 2000) vf = 0.985;
   else
   if (range > 1000) vf = 0.975;
   else
   if (range > 750) vf = 0.965;
   else
   if (range > 500) vf = 0.93;
   else vf = 0.90;

   return vf;
}

///////////////////////////////////////////////////////////////////////////////
// Propagation MODEL_VLF1                                                    //
///////////////////////////////////////////////////////////////////////////////

static double vfmodel_vlf1( double azimuth, double range, double illum)
{
   double h = 70 + (95 - 70) * (1 - illum);
 
   int hop = 1;
   double rmax = 2 * hop * EARTH_RAD * acos( 1/(1 + h/EARTH_RAD));

   double r1 = range > rmax ? rmax : range;       // Range up to rmax
   double r2 = range > rmax ? range - rmax : 0;   // Extra range beyond rmax

   double xi = r1/(2 * hop * EARTH_RAD);
   double phi = atan2( EARTH_RAD * sin( xi), h + EARTH_RAD * (1 - cos(xi)));

   double R = 2 * hop * EARTH_RAD * sin( xi)/sin( phi) + r2 / 0.995;

   return range/R;
}

///////////////////////////////////////////////////////////////////////////////
// Propagation MODEL_VLF2                                                    //
///////////////////////////////////////////////////////////////////////////////

//
// Loads day and night VF tables.   Intermediate illumination interpolated.
// Each table file is ASCII 2-column: range_km and VF
// Rows must be in 10km steps.
//

static float model2_day[2000];
static float model2_night[2000];
static int model2_cnt = 0;

static void vfmodel_vlf2_dump( void)
{
   int k;
   for (k = 1; k < 2000; k++)
      VT_printf( "%d %.4f %.4f\n", k * 10, model2_day[k], model2_night[k]);
}

static void vfmodel_vlf2_load( void)
{
   if (!vfmodel_loadname) VT_bailout( "model data file needed for model_vlf2");

   FILE *f = fopen( vfmodel_loadname, "r");
   if (!f) VT_bailout( "cannot read %s", vfmodel_loadname);

   int km, lino = 0;
   double vfday, vfnight;

   while( fscanf( f, "%d %lg %lg\n", &km, &vfday, &vfnight) == 3)
   {
      lino++;

      if (km < 10 || km >= 20000 || km % 10 != 0)
         VT_bailout( "invalid km %d in %s line %d",
                      km, vfmodel_loadname, lino);

      if (km != lino * 10)
         VT_bailout( "out of step %d km in %s line %d",
                      km, vfmodel_loadname, lino);

      if (vfday < 0 || vfday > 1)
         VT_bailout( "invalid day vf %.4f in %s at line %d",
                    vfday, vfmodel_loadname, lino);
      if (vfnight < 0 || vfnight > 1)
         VT_bailout( "invalid night vf %.4f in %s at line %d",
                    vfnight, vfmodel_loadname, lino);

      model2_day[km/10] = vfday;
      model2_night[km/10] = vfnight;
   }

   fclose( f);

   // Fill in any missing entries, from the immediately previous.
   // XXX should interpolate here

   int k;
   for (k = 2; k < 2000; k++)
   {
      if (!model2_day[k]) model2_day[k] = model2_day[k-1];
      if (!model2_night[k]) model2_night[k] = model2_night[k-1];
   }

   VT_report( 1, "loaded %d lines from %s", lino, vfmodel_loadname);

   model2_cnt = 2000;
}

//
// Return velocity factor using MODEL_VLF2 table lookup and interpolation.
//

static double vfmodel_vlf2( double azimuth, double range, double illum)
{
   int k = floor( range/10);
   if (k < 1) k = 1;
   if (k >= model2_cnt) k = model2_cnt - 1;

   float dayvf = model2_day[k];
   float nightvf = model2_night[k];

   return dayvf * illum + nightvf * (1 - illum);
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

//
// Return group velocity in km/sec for the given azimuth, range and
// illumination factor.
//
// This is where we branch to the particular VF model in use.
//

static double (*vf_hook)( double azim, double range, double illum) = NULL;

static void setup_vfmodel( void)
{
   switch( copt_model)
   {
      case MODEL_FIXED: vf_hook = vfmodel_fixed;
                        break;
      case MODEL_ELF1:  vf_hook = vfmodel_elf1;
                        baseline_excess = BASELINE_EXCESS_ELF;
                        break;
      case MODEL_ELF2:  vf_hook = vfmodel_elf2;
                        vfmodel_elf2_load();
                        baseline_excess = BASELINE_EXCESS_ELF;
                        break;
      case MODEL_ELF3:  vf_hook = vfmodel_elf3;
                        baseline_excess = BASELINE_EXCESS_ELF;
                        break;
      case MODEL_VLF0:  vf_hook = vfmodel_vlf0;
                        break;
      case MODEL_VLF1:  vf_hook = vfmodel_vlf1;
                        break;
      case MODEL_VLF2:  vf_hook = vfmodel_vlf2;
                        vfmodel_vlf2_load();
                        break;

      case MODEL_ELF4: break;

      default:  VT_bailout( "vf_hook not defined for model %d", copt_model);
   }
}

static inline double group_velocity( double azim, double range, double illum)
{
   return 300e3 * vf_hook( azim, range, illum);
}

//
// Compute path-specific group delay for the given illumination factor.
//

static inline double group_delay1( V3 v1, V3 v2, double illum)
{
   double azimuth = v3_bearing( v1, v2);
   double range = v3_range( v1, v2);

   double t = range / group_velocity( azimuth, range, illum);

   if (copt_model == MODEL_ELF1)
      t += vfmodel_elf1_offset( azimuth, range, illum);
   else
   if (copt_model == MODEL_ELF3)
      t += vfmodel_elf3_offset( azimuth, range, illum);

   return t;
}

//
// Compute expected group delay over a path from v1 to v2 at the given
// UT time.
//
// The function computes the path illumination factor and then
// calls group_delay1()
//

static inline double group_delay2( V3 v1, V3 v2, timestamp T)
{
   return group_delay1( v1, v2, !timestamp_is_ZERO( T) ?
                                    path_illumination1( v1, v2, T) : 0.5);
}

//
// Compute expected group delay over a path from v1 to v2 with the Sun
// position given by radac.
//
// The function computes the path illumination factor and then
// calls group_delay1()
//
// Called with use_illum FALSE during sferic matching, as there just isn't
// time to compute a path illumination when trialing so many combinations
// of sferics.
//

static inline double group_delay3( V3 v1, V3 v2,
                                   struct RADEC const *radec, int use_illum)
{
   if (copt_model == MODEL_ELF4)
      return model_elf4_tprop( v1, v2, radec);
   return group_delay1( v1, v2,
                        use_illum ? path_illumination0( v1, v2, radec) : 0.5);
}

///////////////////////////////////////////////////////////////////////////////
// Cost Function                                                             //
///////////////////////////////////////////////////////////////////////////////

//
// Compute a cost for a proposed position 'v' relative to the current
// measurement set.  The cost is measured in units of standard deviations.
// Each measurement in the set is compared with the value it should have if
// the source is at location v and the result expressed in standard deviations
// of that measurement type.
//
// We want to emphasise the worst-case deviation so that we can eliminate
// sferics which don't belong to the intended stroke.  But returning just the
// cost_max can sometimes cause the Nelder-Mead to hang up or converge
// very slowly.  As a compromise, cost() returns a combination of mean
// and worst case cost.
//

static double cost( struct MSET const *m, V3 v)
{
   double cost_max = 0;
   double cost_sum = 0;

   int i;

   for (i = 0; i < m->natd; i++)
   {
      struct ATD const *a = m->atds + i;
      double t1 = group_delay3( v, a->site1->v, &m->radec, m->use_illum);
      double t2 = group_delay3( v, a->site2->v, &m->radec, m->use_illum);
      double e = fabs(t1 - t2 - a->atd);
      double s = e/a->sigma;
      if (s > cost_max) cost_max = s;
      cost_sum += s;
   }

   for (i = 0; i < m->nbrs; i++)
   {
      struct BR const *b = m->brs + i;
      double e = v3_bearing( b->site->v, v) - b->bearing;
      if (e > M_PI) e -= 2*M_PI;
      if (e < -M_PI) e += 2*M_PI;

      // XXX Alternate for bearings mod 180
      // if (e > M_PI/2) e -= M_PI;
      // if (e < -M_PI/2) e += M_PI;

      double s = fabs(e) / b->sigma;
      if (s > cost_max) cost_max = s;
      cost_sum += s;
   }

   //
   // Return combination of mean cost and max cost, weighted to emphasise
   // the worst case by a factor of 3.
   //

   double mean_cost = cost_sum / (m->natd + m->nbrs);
   return (mean_cost + 3 * cost_max)/4;
}

///////////////////////////////////////////////////////////////////////////////
//                                                                           //
///////////////////////////////////////////////////////////////////////////////

//
// Estimate the timestamp of a source, given a location and a set of
// arrival times.
//

static void source_time( struct MSET const *m,
                         V3 location, timestamp *tp, double *stddev)
{
   *tp = timestamp_ZERO;
   if (stddev) *stddev = 0;

   if (!m->nats) return;

   timestamp T_base = m->ats[0].toga;
   int i;
   double sum = 0;
   double sumsq = 0;
   for (i = 0; i < m->nats; i++)
   {
      double measured = timestamp_diff( m->ats[i].toga, T_base);
      double expected =
                   group_delay3( location, m->ats[i].site->v, &m->radec, TRUE);
      double diff = measured - expected;
      sum += diff;
      sumsq += diff * diff;
   }

   double mean = sum/m->nats;
   *tp = timestamp_add( T_base, mean);
   if (stddev) *stddev = sqrt( sumsq/m->nats - mean*mean);
}

///////////////////////////////////////////////////////////////////////////////
// Stroke Solution Output                                                    //
///////////////////////////////////////////////////////////////////////////////

//
// Output a solution location, v.  'n' indicates which solution (0, 1, ...)
// if there is more than one solution for the measurement set.  'rc' is the
// residual cost for the solution. 'ns' is the number of sites used for
// the trilateration.
//
// stddev is the estimated RMS timing error, if known, otherwise zero.
//

static void output_A( struct MSET const *m,
                      V3 location, timestamp T, double stddev,
                      double rc, int idx, int ns)
{
   // Prefix with an 'A' record identifier if using extended output

   if (OMODE_EXT) VT_printf( "A ");

   // If the measurement set contained I/ident then start the output record
   // with the ident token.

   if (m->ident) VT_printf( "%s ", m->ident);

   // Timestamp, either numeric or ISO format string
   char tstring[30];
   if (OMODE_ISO) VT_format_timestamp( tstring, T);
   else  timestamp_string6( T, tstring);

   VT_printf( "%d %s %s %.2f %.2f %d\n",
              idx, v3_string( location, NULL), tstring, rc, stddev * 1e6, ns);
}

static void output_R( char const *sitename,
                      double range,            // km
                      double azimuth,          // Degrees
                      double terr,             // Seconds, -ve is early arrival
                      double amplitude,        // Arbitrary
                      double illum,            
                      timestamp T)             // Arrival time
{
   // Timestamp, either numeric or ISO format string

   char tstring[30];
   if (OMODE_ISO) VT_format_timestamp( tstring, T);
   else timestamp_string6( T, tstring);

   VT_printf( "R %-*s %7.1f %6.1f %5.1f %.3e %.2f %s\n",
               site_width, sitename, range,
               azimuth, terr * 1e6, amplitude, illum, tstring);
}

static void output_E( void)
{
   VT_printf( "E\n");
}

///////////////////////////////////////////////////////////////////////////////
//                                                                           //
///////////////////////////////////////////////////////////////////////////////

//
// Parametric function of an ATD hyperbola.  'param' is the input parameter,
// IS1 and IS2 are the two mirror image points associated with param.
//
// Ra: Radius of circle centered on a->v2;
// Rb: Radius of circle centered on a->v1;
// Ra and Rb in radians subtended at Earth center;
//
// Difference between Ra and Rb equals the ATD angle so that the circles
// intersect on the ATD hyperbola.
//
// Compute the two points IS1, IS2, at which the circles intersect.
//

static int hyperbola_point( struct ATD const *a, double param, V3 IS1, V3 IS2)
{
   double Ra = param + (a->base - a->atd * CVLF)/2/EARTH_RAD;
   double Rb = param + (a->base + a->atd * CVLF)/2/EARTH_RAD;

   double CA = cos(Ra);
   double CB = cos(Rb);

   V3 Va;  v3_copy( a->site2->v, Va);
   V3 Vb;  v3_copy( a->site1->v, Vb);

   double Ax = Va[0], Ay = Va[1], Az = Va[2];
   double Bx = Vb[0], By = Vb[1], Bz = Vb[2];

   V3 VX; v3_cross( Va, Vb, VX);

   double SQRT = sqrt( -v3_dot( Va, Va) * CB * CB -
                        v3_dot( Vb, Vb) * CA * CA +
                        2*v3_dot( Va, Vb) * CA * CB +
                        v3_dot( VX, VX) );

   // Sometimes, limited numeric precision can give a slight -ve argument to
   // sqrt(), when actually it should be zero or nearly zero.
   if (isnan( SQRT))
   {
      if (param) return FALSE;
      SQRT = 0;
   }

   A3 Ar = {
              { +Az * Az + Ay * Ay, -Ax * Ay,          -Ax * Az},
              { -Ax * Ay,           Az * Az + Ax * Ax, -Ay * Az},
              { -Ax * Az,           -Ay*Az,            Ay * Ay + Ax * Ax}
           };

   A3 Br = {
             { Bz * Bz + By * By,   -Bx * By,           -Bx * Bz},
             { -Bx * By,            +Bz * Bz + Bx * Bx, -By * Bz},
             { -Bx * Bz,            -By * Bz,           By * By + Bx * Bx}
           };

   V3 FB;   v3_transform( Vb, Ar, FB);
   V3 FA;   v3_transform( Va, Br, FA);

   v3_scale( FA, CA, FA);
   v3_scale( FB, CB, FB);
   V3 F;   v3_add( FA, FB, F);

   v3_scale( VX, +SQRT, IS1);
   v3_add( IS1, F, IS1);

   v3_scale( VX, -SQRT, IS2);
   v3_add( IS2, F, IS2);

   v3_normalise( IS1);
   v3_normalise( IS2);
   return TRUE;
}

///////////////////////////////////////////////////////////////////////////////
// Pattern Search                                                            //
///////////////////////////////////////////////////////////////////////////////

//
// The pattern search is used in cases where the Nelder-Mead downhill simplex
// cannot be used.  For example when there are only three arrival times, or
// other combinations where there are two exact solutions.
//
// Called with an approximate location 'v' for a source, pattern_search()
// tries to improves the solution with reference to the cost() function.
//

#define MING (0.001 * M_PI/180)    // Convergence resolution (pattern search)

static double pattern_search( struct MSET const *m, V3 v, double g)
{
   double best_cost = cost( m, v);
   VT_report( 2, "begin pattern_search with %.3f at %s",
                  best_cost, v3_string( v, NULL));

   int N = 0;

   //
   // Set up a pair of orthogonal axes, a priori there is no preferred
   // orientation so we just pick something at random.
   //

   V3 vr;   v3_random( vr);
   V3 vx;   v3_unit_normal_to( v, vr, vx);
   V3 vy;   v3_unit_normal_to( v, vx, vy);
 
   while (g >= MING)
   { 
      N++;

      A3 rx1;   a3_rot( vx, +g, rx1);
      A3 ry1;   a3_rot( vy, +g, ry1);
      A3 rx2;   a3_rot( vx, -g, rx2);
      A3 ry2;   a3_rot( vy, -g, ry2);

      // Evaluate points in orthogonal directions away from the current
      // point, continue until no further improvement at this scale.

      int n;  // Number of improvements made
      do {
         n = 0;

         V3 vt;
         double s;

         v3_transform( v, rx1, vt); 
         s = cost( m, vt);
         if (s < best_cost) { v3_copy( vt, v); best_cost = s; n++; }
      
         v3_transform( v, rx2, vt); 
         s = cost( m, vt);
         if (s < best_cost) { v3_copy( vt, v); best_cost = s; n++; }
      
         v3_transform( v, ry1, vt); 
         s = cost( m, vt);
         if (s < best_cost) { v3_copy( vt, v); best_cost = s; n++; }
      
         v3_transform( v, ry2, vt); 
         s = cost( m, vt);
         if (s < best_cost) { v3_copy( vt, v); best_cost = s; n++; }
      }
       while (n);

      // No further improvement so reduce the scale factor
      g *= 0.8;
   }

   VT_report( 2, "pattern search iterations: %d", N);
   return best_cost;
}

///////////////////////////////////////////////////////////////////////////////
// Solution Search Functions                                                 //
///////////////////////////////////////////////////////////////////////////////

//
// Walk along a bearing line in steps of 'ps' radians, looking for the
// minimum score.  Call pattern_search() to polish the position.
//

static int walk_bearing( struct MSET const *m, double ps)
{
   A3 r0; a3_rot( m->brs[0].site->v, -m->brs[0].bearing, r0);
   V3 v0; v3_transform( V3north, r0, v0);

   V3 n0;  v3_unit_normal_to( m->brs[0].site->v, v0, n0);

   double r;
   double sc = 1e99;
   V3 v;
   for (r = 0.01 * M_PI/180; r < 2 * M_PI - 2 * ps; r += ps)
   {
      A3 r1; a3_rot( n0, r, r1);
      V3 vt; v3_transform( m->brs[0].site->v, r1, vt);

      double st = cost( m, vt);
      if (st < sc) { v3_copy( vt, v); sc = st; }
   } 

   double c = pattern_search( m, v, ps);
   if (c < max_residual)
   {
      output_A( m, v, timestamp_ZERO, 0, c, 0, nsites);
      return TRUE;
   }

   return FALSE;
}

//
// Walk along an ATD hyperbola in steps of 'ps' radians.  Find the minimum
// score positions on each side of the ATD baseline and converge each one.
//

static int walk_atd( struct MSET const *m, double ps)
{
   double sc1 = 1e99, sc2 = 1e99;
   int n = 2 * (int)(M_PI/ps); // Number of points to examine on the hyperbola

   static V3 *plist = NULL;
   static double *costs = NULL;

   if (!plist)
   {
      plist = VT_malloc( sizeof( V3) * n);
      costs = VT_malloc( sizeof( double) * n);
   }

   int i;
   for (i = 0; i < n/2; i++)
   {
      double r = ps/2 + i * ps;
      if (!hyperbola_point( m->atds+0, r, plist[n/2 - i], plist[n/2 + i]))
         costs[n/2 - i] = costs[n/2 + i] = 1e99; 
      else
         costs[n/2 - i] = cost( m, plist[n/2 - i]), 
         costs[n/2 + i] = cost( m, plist[n/2 + i]);
   }

   int i1 = -1, i2 = -1;
   for (i = 1; i < n-1; i++)
   {
      if (costs[i] > costs[i-1] ||
          costs[i] > costs[i+1]) continue;

      if (sc1 > sc2 && costs[i] < sc1) { sc1 = costs[i]; i1 = i; }     
      else
      if (sc2 >= sc1 && costs[i] < sc2) { sc2 = costs[i]; i2 = i; }     
   }

   timestamp T;
   double sdev;

   int r = FALSE;
   if (i1 >= 0)
   {
      double c1 = pattern_search( m, plist[i1], ps);
      if (c1 < max_residual)
      {
         source_time( m, plist[i1], &T, &sdev);
         output_A( m, plist[i1], T, sdev, c1, 0, nsites);
         r = TRUE;
      }
   }

   if (i2 >= 0)
   {
      double c2 = pattern_search( m, plist[i2], ps);
      if (c2 < max_residual)
      {
         source_time( m, plist[i2], &T, &sdev);
         output_A( m, plist[i2], T, sdev, c2, 1, nsites);
         r = TRUE;
      }
   }

   return r;
}

//
// Scan an ATD hyperbola and to find an approximate location for the
// minimum cost.
//

static void scan_atd( struct MSET const *m,
                      struct ATD const *a,
                      V3 best_location)
{
   double ps = 1.0 * M_PI/180;
   double best_cost = 1e99;

   int n = M_PI/ps;       // Number of points to examine on the hyperbola

   int i;
   for (i = 0; i < n; i++)
   {
      double r = ps/2 + i * ps;
      V3 is1, is2;
      if (!hyperbola_point( a, r, is1, is2)) continue;

      double c;

      if ((c = cost( m, is1)) < best_cost)
      {
         best_cost = c;
         v3_copy( is1, best_location);
      }

      if ((c = cost( m, is2)) < best_cost)
      {
         best_cost = c;
         v3_copy( is2, best_location);
      }
   }
}

//
// Scan a bearing line in steps of 'ps' radians, looking for the
// minimum cost.
//

static void scan_bearing( struct MSET const *m,
                          struct BR const *br,
                          V3 best_location)
{
   double ps = 1.0 * M_PI/180;
   A3 r0; a3_rot( br->site->v, -br->bearing, r0);
   V3 v0; v3_transform( V3north, r0, v0);

   V3 n0;  v3_unit_normal_to( br->site->v, v0, n0);

   double r;
   double sc = 1e99;
   for (r = 0.01 * M_PI/180; r<2*M_PI - 2*ps; r += ps)
   {
      A3 r1; a3_rot( n0, r, r1);
      V3 vt; v3_transform( br->site->v, r1, vt);

      double st = cost( m, vt);
      if (st < sc) { v3_copy( vt, best_location); sc = st; }
   }
}

///////////////////////////////////////////////////////////////////////////////
// Downhill Simplex Algorithm                                                //
///////////////////////////////////////////////////////////////////////////////

struct SIMPLEX {
   V3 v;
   double cost;
};

//
// Initialise the simplex by scanning the ATD hyperbolas of the first
// three ATDs, finding the minimum cost location of each.  I have tried
// several other methods to choose initial simplex corners and this one
// is by far the most robust.
//
// If there are not enough arrival time differences, then use the available
// bearings in the same way.
//

static int init_simplex( struct MSET *m, struct SIMPLEX *P)
{
   int i, ni;

   for (ni = 0; ni < 3 && ni < m->natd; ni++)
      scan_atd( m, m->atds + ni, P[ni].v);
   for (i = 0; ni < 3 && i < 3 && i < m->nbrs; i++, ni++)
      scan_bearing( m, m->brs + i, P[ni].v);

   return TRUE;
}


//
// This method works with arrival time differences and bearings.  Therefore
// it can perform a 2-D minimising of the cost function over the lightning
// position without considering the lightning timestamp.  The timestamp is
// estimated separately once the location has been determined.
//

static int downhill_simplex( struct SIMPLEX *P,
                             struct MSET *m, V3 location, double *costp)
{
#if 0
   print_mset( m);   // XXX Diagnostic use only
#endif
   int i;

   //
   // Three point simplex and cost sorting function.
   //

   int cmp_cost( const void *a1, const void *a2)
   {
      struct SIMPLEX *p1 = (struct SIMPLEX *) a1;
      struct SIMPLEX *p2 = (struct SIMPLEX *) a2;
      if (p1->cost < p2->cost) return -1;
      if (p1->cost > p2->cost) return +1;
      return 0;
   }

   //
   // A completely standard Nelder-Mead.
   //

   double alpha = 1.0, gamma = 2.0, rho = 0.5, sigma = 0.5;

   for (i = 0; i < 3; i++) P[i].cost = cost( m, P[i].v);

   int NI = 0;    // Number of iterations
   double previous_cost = 0;

   while (1)
   {
      NI++;
      qsort( P, 3, sizeof( struct SIMPLEX), cmp_cost);

      // Termination conditions 
      if (NI > 50 && previous_cost - P[0].cost < 0.001) break;
      if (NI > 200) break;

      previous_cost = P[0].cost;

      V3 x0;  v3_add( P[0].v, P[1].v, x0);  v3_normalise( x0);

      // 
      // Reflection.
      //

      V3 xr;
      xr[0] = x0[0] + alpha * (x0[0] - P[2].v[0]);
      xr[1] = x0[1] + alpha * (x0[1] - P[2].v[1]);
      xr[2] = x0[2] + alpha * (x0[2] - P[2].v[2]);

      double cr = cost( m, xr);
      if (cr >= P[0].cost && cr < P[1].cost)
      {
         v3_copy( xr, P[2].v);
         P[2].cost = cr;
         continue;
      }

      //
      // Expansion.
      //

      if (cr < P[0].cost)
      {
         V3 xe;
         xe[0] = x0[0] + gamma * (xr[0] - x0[0]);
         xe[1] = x0[1] + gamma * (xr[1] - x0[1]);
         xe[2] = x0[2] + gamma * (xr[2] - x0[2]);
         double ce = cost( m, xe);
         if (ce < cr)
         {
            v3_copy( xe, P[2].v);
            P[2].cost = ce;
         }
         else
         {
            v3_copy( xr, P[2].v);
            P[2].cost = cr;
         }
         continue;
      }

      //
      // Contraction.
      //

      V3 xc;
      xc[0] = x0[0] + rho * (P[2].v[0] - x0[0]);
      xc[1] = x0[1] + rho * (P[2].v[1] - x0[1]);
      xc[2] = x0[2] + rho * (P[2].v[2] - x0[2]);
      double cc = cost( m, xc);

      if (cc < P[2].cost)
      {
         v3_copy( xc, P[2].v);
         P[2].cost = cc;
         continue;
      }

      //
      // Shrink.
      //

      for (i = 1; i <= 2; i++)
      {
         P[i].v[0] = P[0].v[0] + sigma * (P[i].v[0] - P[0].v[0]);
         P[i].v[1] = P[0].v[1] + sigma * (P[i].v[1] - P[0].v[1]);
         P[i].v[2] = P[0].v[2] + sigma * (P[i].v[2] - P[0].v[2]);
      }
   }

   if (P[0].cost < max_residual)   // Successful?
   {
      if (costp) *costp = P[0].cost;
      v3_copy( P[0].v, location);
      VT_report( 3, "downhill simplex completed NI %d cost %.4f %.4f",
                  NI, P[0].cost, previous_cost - P[0].cost);
      return TRUE;
   }

   return FALSE;
}

//
// Solve a measurement set, provided on the command line or via stdin.
//
// Returns FALSE if any ATD exceeds nightime baseline delay.
//

static int process_measurement_set( struct MSET *m)
{
   int i;

   init_radec( timestamp_secs( m->ats[0].toga), &m->radec);

   //
   // For diagnostics, report arrival times and bearings.
   //

   for (i = 0; i < m->nats; i++)
   {
      char temp[30];
      timestamp_string6( m->ats[i].toga, temp);
      VT_report( 2, "at %s %s (%.6f)",
                      v3_string( m->ats[i].site->v, NULL),
                      temp, m->ats[i].sigma);
   }

   for (i = 0; i < m->nbrs; i++)
      VT_report( 2, "br %s %.1f (%.1f)",
                      v3_string( m->brs[i].site->v, NULL),
                      m->brs[i].bearing * 180/M_PI,
                      m->brs[i].sigma * 180/M_PI);

   //
   // Take the arrival times ats[] of the measurement set and build a list of
   // all the arrival time differences.   If any ATD exceeds the baseline
   // this function returns without attempting a solution.
   //

   for (i = 0; i < m->nats - 1; i++)
   {
      struct ATD *a = m->atds + m->natd;
      a->site1 = m->ats[i+0].site;
      a->site2 = m->ats[i+1].site;
      a->atd = timestamp_diff( m->ats[i+0].toga, m->ats[i+1].toga);
      a->base = v3_range( a->site1->v, a->site2->v);

      double tbase = a->base/300e3 * baseline_excess;
      if (fabs( a->atd) > tbase)
      {
         VT_report( 1, "ATD out of range %s %s %.6f limit %.6f",
                           a->site1->name, a->site2->name, a->atd, tbase);
         return FALSE;
      }

      a->sigma = m->ats[i].sigma;
      if (m->ats[i+1].sigma > a->sigma) a->sigma = m->ats[i+1].sigma;

      VT_report( 1, "ATD %s %s %.6f", a->site1->name, a->site2->name, a->atd);
      m->natd++;
   }

   if (m->nats == 0 && m->nbrs == 2)
   {
      // Special case: intersection of two bearings.  Calculate the exact
      // solutions without iteration.

      // Make 2nd point on each GC by rotating V3north around each point
      A3 r0; a3_rot(  m->brs[0].site->v, -m->brs[0].bearing, r0);
      V3 v0; v3_transform( V3north, r0, v0);

      A3 r1; a3_rot(  m->brs[1].site->v, -m->brs[1].bearing, r1);
      V3 v1; v3_transform( V3north, r1, v1);

      // Define GCs by their normal unit vector
      V3 n0;  v3_unit_normal_to( m->brs[0].site->v, v0, n0);
      V3 n1;  v3_unit_normal_to( m->brs[1].site->v, v1, n1);

      V3 it1;  v3_unit_normal_to( n0, n1, it1);
      V3 it2;  v3_scale( it1, -1, it2);

      output_A( m, it1, timestamp_ZERO, 0, 0, 0, 2);
      output_A( m, it2, timestamp_ZERO, 0, 0, 1, 2);
      return TRUE;
   }

   //
   // If enough arrival time differences and bearings, use Nelder-Mead to find
   // the single best location.
   //

   if (m->natd + m->nbrs >= 3)
   {
      V3 v;
      double c;
      struct SIMPLEX P[3];
      init_simplex( m, P);
      if (downhill_simplex( P, m, v, &c))
      {
         timestamp T = timestamp_ZERO;
         double sdev = 0;
         source_time( m, v, &T, &sdev);

         output_A( m, v, T, sdev, c, 0, 0);

         if (OMODE_EXT)    // Extended output requested?
         {
            // Output an R record for each sferic
            int i;
            for( i = 0; i < m->nats; i++)
            {
               double illum =
                  path_illumination1( v, m->ats[i].site->v, T);

               double range = v3_range( v, m->ats[i].site->v);
               double terr =
                  timestamp_diff( m->ats[i].toga, T) -
                          group_delay3( v, m->ats[i].site->v, &m->radec, TRUE);

               output_R( m->ats[i].site->name,
                         range,
                         v3_bearing( v, m->ats[i].site->v) * 180/M_PI,
                         terr,
                         0.0, illum,  m->ats[i].toga);
            }

            output_E();
         }

         return TRUE;   // Successful downhill simplex
      }

      return FALSE;     // No solution from downhill simplex
   }

   double ps = 1.0 * M_PI/180;

   // If we have any arrival time differences, then pick the first ATD
   // baseline and scan that for approximate solutions.   Otherwise scan
   // the first bearing line of the measurement set.

   if (m->nats) init_radec( timestamp_secs( m->ats[0].toga), &m->radec);

   if (m->natd)
   {
      return walk_atd( m, ps);
   }
   else
   if (m->nbrs)
   {
      return walk_bearing( m, ps);
   }

   return FALSE;
}

///////////////////////////////////////////////////////////////////////////////
// Sferic Matching                                                           //
///////////////////////////////////////////////////////////////////////////////

//
// Set up tables for quick reference to baseline ranges and ATD limit.
//

struct BASE {
   double secs;
   double km;
} *bases = NULL;

static inline struct BASE * baseline( struct SITE const *s1,
                                      struct SITE const *s2)
{
   return bases + (s1 - sites) * nsites + (s2 - sites);
}

static void setup_tables( void)
{
   int i, j;

   bases = calloc( nsites * nsites, sizeof( struct BASE));

   for (i = 0; i < nsites - 1; i++)
   {
      struct SITE *s1 = sites + i;

      for (j = i + 1; j < nsites; j++)
      {
         struct SITE *s2 = sites + j;

         struct BASE *bp1 = baseline( s1, s2);
         struct BASE *bp2 = baseline( s2, s1);
         bp1->km = bp2->km = v3_range( s1->v, s2->v);
         bp1->secs = bp2->secs = bp1->km/300e3 * baseline_excess;
      }
   }
}

static void parse_matching( char *arg)
{
   char *s = arg;

   // -m site=file

   char *p = strchr( s, '=');
   if (!p) VT_bailout( "invalid -m argument [%s]", arg);
   *p++ = 0;

   struct SITE *site = parse_latlon( s);
   site->file = strdup( p);
   VT_report( 2, "matching: %s from %s", site->name, site->file);
}

//
// An aggregate array of all the sferics to be matched and solved.
//

typedef struct SFERIC {
   struct SITE *site; // Link to the site 

   timestamp ref;     // Reference time for this sferic
   timestamp toga;    // The timestamp from vttoga
   timestamp edge;    // Timestamp of leading edge, or timestamp_ZERO

   float rms;         // RMS amplitude reported by vttoga
   int16_t azimuth;   // Radians

   int16_t flags;     // Bit flags
} SFERIC;

static SFERIC *sfericlist = NULL;

#define HAS_AZIMUTH (1 << 0)
#define USED (1 << 1)

static int nmatch = 0;        // Number of entries in sfericlist[]
static int match_alloc = 0;   // Allocated entries in sfericlist[]

static void add_sfericlist( struct SITE *site,
                            timestamp ref,
                            timestamp toga,
                            timestamp edge,
                            double rms,
                            int has_az, double azimuth)
{
   // Extend sfericlist[] in blocks of 1000
   if (nmatch == match_alloc)
   {
      match_alloc += 1000;
      sfericlist = VT_realloc( sfericlist, sizeof( SFERIC) * match_alloc);
   }

   SFERIC *p = sfericlist + nmatch++;

   p->site = site;
   p->ref = ref;
   p->toga = toga;
   p->edge = edge;
   p->rms = rms;
   p->flags = has_az ? HAS_AZIMUTH : 0;
   p->azimuth = azimuth;
}

static int filter_solution( SFERIC **list, int nlist, uint64_t u,
                            V3 stroke)
{
   struct S5 {
      struct SITE *site;
      double bearing;
      double range;
   } s[64];

   int ns = 0;

   int cmp_bearing( const void *s1, const void *s2)
   {
      struct S5 *p1 = (struct S5 *) s1;
      struct S5 *p2 = (struct S5 *) s2;
      if (p1->bearing < p2->bearing) return -1;
      if (p1->bearing > p2->bearing) return +1;
      return 0;
   }

   if (__builtin_popcountl( u) < minsites) return FALSE;

   //
   // Link the sferics selected by the bit pattern 'u', into s[].
   // Compute the range and bearing of the receiver, from the standpoint
   // of the lightning stroke.
   //

   int i;
   for (i = 0; i < nlist; i++)
      if (u & (1L << i))
      {
         s[ns].site = list[i]->site;
         s[ns].range = v3_range( stroke, s[ns].site->v);
         s[ns].bearing = v3_bearing( stroke, s[ns].site->v);
         ns++;
      }

   if (filter_envelop)
   {
      // Sort into bearing order so that the receivers are ordered clockwise
      // around the stroke.

      qsort( s, ns, sizeof( struct S5), cmp_bearing);

      //
      // Look for any interval greater than filter_envelop degrees.
      //

      for (i = 0; i < ns; i++)
      {
         double db = i < ns-1 ? s[i+1].bearing - s[i].bearing
                              : s[0].bearing - s[i].bearing;

         if (db < 0) db += 2*M_PI;
         if (db > filter_envelop * M_PI/180) return FALSE;
      }
   }

   if (filter_nearmax)
   {
      //
      // Discard the solution if the nearest receiver is more than
      // filter_nearmax km distant from the stroke.
      //

      for (i = 0; i < ns; i++) if (s[i].range < filter_nearmax) break;
      if (i == ns) return FALSE;
   }

   //
   // XXX Other filtering options go in here.
   //

   return TRUE;
}

static int prepare_measurement_set( struct MSET *m, 
                                    SFERIC **list, int n,
                                    uint64_t mask)
{
   int i, j;

   reset_measurement_set( m);

   for (i = 0; i < n; i++)
   {
      if ((mask & (1L << i)) == 0) continue;  // This sferic not selected?
    
      m->ats[m->nats].site = list[i]->site;
      m->ats[m->nats].toga = list[i]->toga;
      m->ats[m->nats].sigma = DEFAULT_AT_SIGMA;
      m->nats++;

      if (list[i]->flags & HAS_AZIMUTH)
      {
         m->brs[m->nbrs].site = list[i]->site;
         m->brs[m->nbrs].bearing = list[i]->azimuth;
         m->brs[m->nbrs].sigma = DEFAULT_AZ_SIGMA * M_PI/180;
         m->nbrs++;
      }
   }

   //
   // Check baseline ATD limits and build list of independent ATDs.
   //

   for (i = 0; i < m->nats - 1; i++)
      for (j = i+1; j < m->nats; j++)
      {
         // Discard if any site has contributed more than one sferic
         if (m->ats[i].site == m->ats[j].site) return FALSE;

         // All baselines must be checked, to help remove sferics
         // that come from different strokes.

         double atd = timestamp_diff( m->ats[i].toga, m->ats[j].toga);
   
         struct BASE *bp = baseline( m->ats[i].site, m->ats[j].site);
  
         if (fabs(atd) > bp->secs) return FALSE;

         // Add independent ATDs to atds[]

         if (j == i+1)
         {
            struct ATD *a = m->atds + m->natd;
      
            a->site1 = m->ats[i].site;
            a->site2 = m->ats[j].site;

            a->sigma = m->ats[i].sigma;
            if (m->ats[j].sigma > a->sigma) a->sigma = m->ats[j].sigma;
      
            a->base = bp->km;
            a->atd = atd;
            m->natd++;
         }
      }

   return TRUE;
}

//
// Attempt to find a stroke solution from the array of up to 64 sferic.
// All sferics earlier than this list have already been dealt with, either
// used or exhaustively tried.
//
// The list is in timestamp order.
//
// Attempt to find a solution which involves the sferic at the head of the
// list, plus some number of other later sferics from the list.
//
// The strategy is to find a solution amongst the first eight sferics by
// trying all combinations involving the first sferic and four or more others.
// Then add additional later sferics one at a time, discarding those that
// don't fit.
//

static pthread_mutex_t output_lock = PTHREAD_MUTEX_INITIALIZER;

static int n_strokes = 0;      // Total number of solutions generated. Doubles
                               // as an index number for output records

static void process_matchlist( SFERIC **list, int nlist)
{
   int i;
   struct MSET m;
   init_measurement_set( &m);

   init_radec( timestamp_secs( list[0]->toga), &m.radec);

   // Can only consider up to 64 sferics at a time because we use a 64 bit
   // word to keep track of the selections

   if (nlist > 64) nlist = 64;

   //
   // Search permutations of sferics at the head of the list, until some
   // solution is found.  Permutations of up to max_initial sferics are tried.
   // We want at least min_initial number of sites in the initial solution.
   //

   int max_initial = 9;
   if (max_initial > nlist) max_initial = nlist;

   int min_initial = 6;
   if (min_initial > minsites) min_initial = minsites;

   V3 best_location;      // The stroke location for the best pattern
   uint64_t best_u = 0;   // The bit pattern which selects the best sferics in
                          // the list
   double best_cost = 0;  // The cost() of the best pattern
   int best_weight = 0;   // Number of sferics in the best pattern: the hamming
                          // weight of best_u

   uint64_t u;  // Bit pattern selecting the list entries currently being tried
 
   uint32_t npu = 1 << max_initial;
   uint32_t npm = (1 << min_initial) - 1;

   m.use_illum = FALSE;  // Turn off use of path illumination factor for the
                         // duration of the matching, the cost function will
                         // use a factor 0.5 for all paths.

   struct SIMPLEX P[3];

   for (u = npm; u < npu; u += 2) // Only odd patterns are tried because the
                                  // first entry in the sferics list must
                                  // always be present
   {
      int weight = __builtin_popcountl( u);
      if (weight < min_initial) continue;  // We want a minimum number of
                                           // sferics in the initial solution

      //
      // prepare_measurement_set() will return FALSE if any ATDs exceed a
      // baseline - this also removes combinations where a site has more than
      // one sferic in the selection.
      //

      if (prepare_measurement_set( &m, list, max_initial, u) &&
          init_simplex( &m, P) &&
          downhill_simplex( P, &m, best_location, &best_cost))
      {
         best_u = u;
         best_weight = weight;
         break;
      }
   }

   if (!best_weight) // No initial solution found?
   {
      reset_measurement_set( &m);
      return;   
   }

   //
   // Recompute initial solution using a better model.
   //

   m.use_illum = TRUE;       // Enable use of illumination factor as the
                             // solution is to be extended and refined

   if (!prepare_measurement_set( &m, list, nlist, best_u) ||
       !init_simplex( &m, P))
   {
      reset_measurement_set( &m);
      return;   
   }

   struct SIMPLEX save_P[3];
   memcpy( save_P, P, 3 * sizeof( struct SIMPLEX));

   if (!downhill_simplex( P, &m, best_location, &best_cost))
   {
      reset_measurement_set( &m);
      return;   
   }

   best_weight = __builtin_popcountl( best_u);

   //
   // Extend the solution by adding one additional sferic at a time,
   // keeping those that still solve.
   //

   for (i = 0; i < nlist; i++)
   {
      if (best_u & (1L << i)) continue;  // Sferic already in the list?

      u = best_u | (1L << i);    // Add sferic 'i' to the pattern

      struct SIMPLEX PT[3];      // Temporary copy of the best simplex
      memcpy( PT, save_P, 3 * sizeof( struct SIMPLEX));

      if (prepare_measurement_set( &m, list, nlist, u) &&
          downhill_simplex( PT, &m, best_location, &best_cost))
      {
         best_weight = __builtin_popcountl( u);
         best_u = u;
// XXX        memcpy( save_P, PT, 3 * sizeof( struct SIMPLEX));
      }
   }

   if (best_weight < minsites)
   {
      reset_measurement_set( &m);
      return;
   }
   
   //
   // Apply filtering rules to the solution.  If a rule fails, the solution
   // is discarded and the sferics in list[], except the first one, remain
   // available for further attempts.
   //

   if (!filter_solution( list, nlist, best_u, best_location))
   {
      reset_measurement_set( &m);
      return;
   }

   //
   // Mark the successful sferics as used in sfericlist[].  Reload ats[] with
   // the final selection of sferics - needed by source_time().
   //

   prepare_measurement_set( &m, list, nlist, best_u);

   for (i = 0; i < nlist; i++) if (best_u & (1L << i)) list[i]->flags |= USED;

   timestamp T = timestamp_ZERO;
   double sdev = 0;
   source_time( &m, best_location, &T, &sdev);

   pthread_mutex_lock( &output_lock);

   for (i = 0; i < nlist; i++)
      if (best_u & (1L << i))
      {
         double diff = timestamp_diff( list[i]->toga, T)
                  - group_delay2( best_location, list[i]->site->v, T);
         list[i]->site->tetotal += diff;
         list[i]->site->n_matched++;
      }

   //
   // Output solution record.
   //

   output_A( &m, best_location, T, sdev, best_cost, n_strokes, best_weight);

   if (OMODE_EXT) // Extended output requested?
   { 
      // Output an R record for every sferic used

      for (i = 0; i < nlist; i++)
         if (best_u & (1L << i))
         {
            double illum =
                path_illumination1( best_location, list[i]->site->v, T);

            double range = v3_range( best_location, list[i]->site->v);
            double terr =
                timestamp_diff( list[i]->toga, T) -
                 group_delay1( best_location, list[i]->site->v, illum);

            output_R( list[i]->site->name,
                      range,
                      v3_bearing( best_location, list[i]->site->v) * 180/M_PI,
                      terr,
                      list[i]->rms, illum, list[i]->ref);
         }

      output_E();
   }

   //
   // Output a .brc file for map plotting.
   //

   if (OMODE_BRC)
   {
      char filename[100];
      sprintf( filename, "brc/%d.brc", n_strokes);
      FILE *f = fopen( filename, "w");
      if (f)
      {
         for (i = 0; i < nlist; i++)
            if (u & (1L << i))
               fprintf( f, "spot %s 0.005\n",
                 v3_string( list[i]->site->v, NULL));
         fprintf( f, "spot %s 0.005 S\n", v3_string( best_location, NULL));
         fclose( f);
      }
   }

   n_strokes++;

   pthread_mutex_unlock( &output_lock);
}

//
// Load all the sferic data from the files list with -m options.
//
// The data goes into a single big array sfericlist[] which is then sorted
// into timestamp order.
//

static double batch_duration = 0;  // Overall time spanned by the batch of 
                                   // TOGA files.

static void load_matching_files( void)
{
   int i;

   for (i = 0; i < nsites; i++)
   {
      struct SITE *site = sites + i;

      FILE *f = fopen( site->file, "r");
      if (!f) VT_bailout( "cannot open %s: %s", site->file, strerror( errno));

      char buff[256];
      while (fgets( buff, 255, f))
      {
         if (!strncmp( buff, "H ", 2))
         {
            long double T;
            double rms;
            double dummy;
           
            int n;            // Number of chars used by the first three fields
            if (sscanf( buff+2, "%Lf %lg %lg%n",
                                &T, &rms, &dummy, &n) != 3)
               VT_bailout( "format error in [%s]", site->file);
   
            // Optional bearing supplied by the site?
            double azimuth;
            int has_az = sscanf( buff + 2 + n, "%lg", &azimuth) == 1;
               
            timestamp TS = timestamp_compose( (int)T, T - (int)T);
   
            add_sfericlist( site, TS, TS, timestamp_ZERO,
                            rms, has_az, azimuth * M_PI/180);
            site->n_sferic++;
         }
         else
         if (!strncmp( buff, "M ", 2))
         {
            long double Tref;
            double toga_offset;
            double edge_offset;
            double rms;
            double dummy;
           
            int n;            // Number of chars used by the first five fields
            if (sscanf( buff+2, "%Lf %lg %lg %lg %lg%n",
                                &Tref, &toga_offset, &edge_offset,
                                &rms, &dummy, &n) != 5)
               VT_bailout( "format error in [%s] [%s]", site->file, buff);
   
            // Optional bearing supplied by the site?
            double azimuth;
            int has_az = sscanf( buff + 2 + n, "%lg", &azimuth) == 1;
               
            timestamp TS_ref = timestamp_compose( (int)Tref, Tref - (int)Tref);
           
            timestamp TS_toga = timestamp_add( TS_ref, toga_offset);
            
            timestamp TS_edge = edge_offset >= 0 ? 
                                    timestamp_add( TS_ref, edge_offset) :
                                    timestamp_ZERO;
   
            add_sfericlist( site, TS_ref, TS_toga, TS_edge,
                            rms, has_az, azimuth * M_PI/180);
            site->n_sferic++;
         }
         else
            VT_bailout( "error in matching input [%s] [%s]", site->file, buff);
      }

      fclose( f);
   }

   if (!nmatch) VT_bailout( "no sferics found to match");

   //
   // Sort the sferics into timestamp order.
   //

   int cmp_sfericlist( const void *a1, const void *a2)
   {
      SFERIC *p1 = (SFERIC *) a1;
      SFERIC *p2 = (SFERIC *) a2;
      if (timestamp_LT( p1->toga, p2->toga)) return -1;
      if (timestamp_GT( p1->toga, p2->toga)) return +1;
      return 0;
   }
   
   qsort( sfericlist, nmatch, sizeof( SFERIC), cmp_sfericlist);

   VT_report( 1, "loaded %d sferics from %d sites", nmatch, nsites);

   batch_duration =
               timestamp_diff( sfericlist[nmatch-1].toga, sfericlist[0].toga);
   VT_report( 1, "duration %.1f seconds", batch_duration);
   VT_report( 1, "average rate %.1f sferics/second", nmatch/batch_duration);
}

//
// The sferic matching work will be spread across multiple threads.  This
// little structure is shared by the dispatcher and worker threads.
//

struct MATCH_LAUNCH {
   SFERIC *sfericlist;
   int nmatch;
   int done;
};

//
// Sferic matching worker task.
//

static void *task_matching( void *arg)
{
   struct MATCH_LAUNCH *L = (struct MATCH_LAUNCH *)arg;
   int used[MAXSITES];

   int i;

   //
   // All the sferics from all the sites are merged into sfericlist[] in
   // timestamp order.   Work sequentially through the list, trying to find
   // sferics which belong to the same stroke as the sfericlist[base] sferic.
   //

   double list_window = maxatd ? maxatd :
             M_PI * EARTH_RAD/group_velocity( M_PI, M_PI * EARTH_RAD, 1.0);

   int base;
   for (base = 0; base < L->nmatch - 3; base++)
   {
      //
      // All sferics sfericlist[0] to sfericlist[base-1] have been either used
      // or exhaustively tried.   Try to find a solution involving
      // sfericlist[base], with some number of later sferics.
      //

      SFERIC *bs = L->sfericlist + base;

      if (bs->flags & USED) continue;  // This sferic was successful in a
                                       // previous matching, so is not to
                                       // be used again.

      //
      // Build a list of TOGAs over which to attempt matching.  Up to 64
      // sferics can be considered, this limit set by the word size.
      //

      for (i = 0; i < nsites; i++) used[i] = FALSE;
      
      SFERIC *matchlist[64];    // Empty list of TOGAs
      int nmatchlist = 0;

      matchlist[nmatchlist++] = bs;  // Add our base entry
      int sites_in_matchlist = 1;    // Number of sites in the matching list
      used[bs->site - sites] = TRUE;

      int j;
      for (j = base+1; j < L->nmatch && nmatchlist < 64; j++)
      {
         if (L->sfericlist[j].flags & USED) // This TOGA was successful in a
            continue;                       // previous measurement set, so is
                                            // not to be used again

         // Don't include further sferics from the base site
         if (L->sfericlist[j].site == bs->site) continue; 

         double atd = timestamp_diff( L->sfericlist[j].toga, bs->toga);
         if (atd > list_window) break;

         struct BASE *bp = baseline( bs->site, L->sfericlist[j].site);
         if (fabs(atd) > bp->secs) continue;

         matchlist[nmatchlist++] = L->sfericlist + j;   // Accept into the list

         // Keep a count of sites in the list
         if (!used[L->sfericlist[j].site - sites]) 
         {
            used[L->sfericlist[j].site - sites] = TRUE;
            sites_in_matchlist++;
         }
      }

      //
      // Now matchlist[] contains a list of sferics which can plausibly belong
      // to the same stroke.   A site may contribute more than one sferic
      // into matchlist[].
      // 

      if (sites_in_matchlist >= minsites)
         process_matchlist( matchlist, nmatchlist);
   }

   L->done = TRUE;
   return 0;
}

//
// Entry point to sferic matching.  Work is allocated to CPUs and the
// actual matching occurs in task_matching().
//

static void run_matching( void)
{
   pthread_t pids[ncpu];
   memset( pids, 0, sizeof( pids));

   struct MATCH_LAUNCH L[ncpu];
   double mingap = maxatd ? maxatd :
         M_PI * EARTH_RAD/group_velocity( M_PI, M_PI * EARTH_RAD, 1.0);

   int partition_base = 0;
   int cpu = 0;

   if (ncpu == 1)
   {
      //
      // Only one CPU available so no point in partitioning.
      //

      L[0].sfericlist = sfericlist;
      L[0].nmatch = nmatch;
      task_matching( (void *)&L[0]);
   }
   else
   {
      pthread_attr_t pa;
      pthread_attr_init( &pa);
      pthread_attr_setdetachstate( &pa, PTHREAD_CREATE_DETACHED);

      //
      // Partition sfericlist[] where we find gaps in the time series of TOGAs.
      // Run each partition in a separate thread.
      // To avoid nibbling, ensure at least 500 sferics in a partition.
      //

      while (partition_base < nmatch - 1)
      {
         // Search forward through the sfericlist[] from partition_base to
         // find a gap in the TOGAs exceeding mingap seconds.
   
         int n = partition_base + 500;
         if (n > nmatch) n = nmatch;
         for (; n < nmatch; n++)
         {
            double dt = timestamp_diff( sfericlist[n].toga,
                                        sfericlist[n-1].toga);
            if (dt > mingap) break;
         }
   
         // No point in running this partition if it has less than minsites
         // sferics
         if (n - partition_base >= minsites)
         {
            // Find a spare CPU
            while (1)
            {
               for (cpu = 0; cpu < ncpu; cpu++)
                  if (!pids[cpu] || L[cpu].done) break;
      
               if (cpu < ncpu) break;   // Found an expired thread?
      
               usleep( 1000);
            }
      
            L[cpu].sfericlist = sfericlist + partition_base;
            L[cpu].nmatch = n - partition_base;
            L[cpu].done = FALSE;
            pthread_create( &pids[cpu], &pa, task_matching, (void *)&L[cpu]);
         }
   
         partition_base = n;
      }
   
      for (cpu = 0; cpu < ncpu; cpu++)
         while (pids[cpu] && !L[cpu].done) usleep( 1000);
      pthread_attr_destroy( &pa);
   }

   int n_used = 0;
   int i;
   for (i = 0; i < nmatch; i++) if (sfericlist[i].flags & USED) n_used++;

   VT_report( 1, "sferics successfully used: %d (%.1f%%)",
                  n_used, n_used * 100.0/nmatch);
   VT_report( 1, "strokes found: %d", n_strokes);

   if (n_strokes)
      VT_report( 1, "average sferics per stroke: %.1f",
                     n_used/(double) n_strokes);

   if (batch_duration)
      VT_report( 1, "average strokes per second: %.2f",
                        n_strokes/batch_duration);

   //
   // Extended output?  Add the site summary 'G' records and a final 'Q'
   // record.
   //

   if (OMODE_EXT)
   {
      for (i = 0; i < nsites; i++)
      {
         struct SITE *s = sites + i;

         VT_printf( "G %-*s %6u %6u %6.1f\n",
                    site_width, s->name,
                    s->n_sferic, s->n_matched,
                    s->n_matched ? s->tetotal/s->n_matched * 1e6 : 0);
      }

      VT_printf( "Q %d %d %d %.1f\n",
                  nmatch, n_used, n_strokes, batch_duration);
   }
}

///////////////////////////////////////////////////////////////////////////////
// Magnetic Field                                                            //
///////////////////////////////////////////////////////////////////////////////

//
// IGRF 13th generation from https://www.ngdc.noaa.gov/IAGA/vmod/igrf.html
// Coefficients https://www.ngdc.noaa.gov/IAGA/vmod/coeffs/igrf13coeffs.txt
//
// https://www.spenvis.oma.be/help/background/magfield/legendre.html#Schmidt1
//

static int igrf_loaded = FALSE;

#define MAXDEG 13
#define NYEARS (2020 - 1970 + 5)

static struct IGRF {
   float coef;
   float sv;
} gauss_g[MAXDEG+1][MAXDEG+1][NYEARS],
  gauss_h[MAXDEG+1][MAXDEG+1][NYEARS];

//
// Load the Gauss coefficients for 1970 onwards.   Hard coded for the 13th
// generation IGRF.
//

static int load_igrf_file( char const *filename)
{
   FILE *f = fopen( filename, "r");
   if (!f) return FALSE;

   int lino = 0;

   char temp[500];

   while (fgets( temp, 500, f))
   {
      lino++;
      char *p = temp;
      char *fields[50];
      int nf = 0;

      while (*p)
         if (isspace( *p)) *p++ = 0;
         else
         {
            fields[nf++] = p;
            while (*p && !isspace( *p)) p++;
         }

      struct IGRF (*c)[MAXDEG+1][MAXDEG+1][NYEARS];

      if (!strcmp( fields[0], "g")) c = &gauss_g;
      else
      if (!strcmp( fields[0], "h")) c = &gauss_h;
      else continue;

      int deg = atoi( fields[1]);
      int ord = atoi( fields[2]);

      if (deg > MAXDEG) continue;
 
      int n ;   
      for (n = 17; n <= 27; n++)   // Indexing from 1970 to 2020 coefficients
      {
         int y1 = 1970 + (n - 17) * 5;
         int y2 = y1 + 5;

         double c1 = atof( fields[n]);
         double sv = n < 27 ? (atof( fields[n+1]) - c1)/5 : atof( fields[28]);

         int y;
         for (y = y1; y < y2; y++)
         {
            (*c)[deg][ord][y - 1970].coef = c1 + (y - y1) * sv;
            (*c)[deg][ord][y - 1970].sv = sv;
         }
      }
   }

   igrf_loaded = TRUE;
   VT_report( 2, "loaded %s", filename);

   return TRUE;
}

static void load_igrf( void)
{
   if (igrf_loaded) return;

   if (load_igrf_file( "./igrf13coeffs.txt")) return;

   char *home = getenv( "HOME");
   if (!home) VT_bailout( "unable to find a igrf13coeffs.txt file");

   char *path;
   if (asprintf( &path, "%s/igrf13coeffs.txt", home) < 0 || !path) return;
   if (!load_igrf_file( path))
      VT_bailout( "unable to find a igrf13coeffs.txt file");
   free( path);
}

//
// From Numeric Recipes.   This has the Condon-Shortley phase factor and
// doesn't have the Schmidt semi-normalisation.   Both of these are fixed
// in wrapper functions.
//

static double legendre0( int l, int m, double x)
{
   double fact, pll = 0, pmm, pmmp1, somx2;

   int i, ll;

   pmm = 1;

   if (m > 0)
   {
      somx2 = sqrt( (1 - x) * (1 + x));
      fact = 1;
      for (i = 1; i <= m; i++)
      {
         pmm *= -fact * somx2;
         fact += 2;
      }
   }

   if (l == m) return pmm;

   pmmp1 = x * (2 * m + 1) * pmm;
   if (l == m + 1) return pmmp1;

   for (ll = m + 2; ll <= l; ll++)
   {
      pll = (x * (2 * ll - 1) * pmmp1 - (ll + m - 1) * pmm) / (ll - m);

      pmm = pmmp1;
      pmmp1 = pll;
   }

   return pll;
}

// Remove the Condon-Shortley phase

static inline double legendre( int n, int m, double x)
{
   double y = legendre0( n, m, x);
   return m % 2 == 1 ? -y : y;
}

static inline double factorial2( int k1, int k2)
{
   double f = 1;
   while (k2 > k1) f *= k2--;
   return f;
}

static inline double schmidt_seminorm( int n, int m, double f)
{
   return m ? f * sqrt( 2.0 / factorial2( n - m, n + m)) : f;
}

//
//  Conversions between ECEF, geodetic, and spherical geocentric.
//

static void geodetic_to_ecef( double h, double lat, double lon,
                              double *X, double *Y, double *Z)
{
   double e = sqrt( 1 - ERAD_b2/ERAD_a2);
   double N = ERAD_a / sqrt( 1 - e * e * sin( lat) * sin( lat));

   *X = (N + h) * cos( lat) * cos( lon);
   *Y = (N + h) * cos( lat) * sin( lon);
   *Z = ((1 - e*e) * N + h) * sin( lat);
}

static void ecef_to_geocentric( double X, double Y, double Z,
                                double *gclat, double *gclon, double *r)
{
   *r = sqrt( X*X + Y*Y + Z*Z);

   double g = sqrt( X*X + Y*Y);
   *gclat = atan2( Z, g);
   *gclon = atan2( Y, X);
}

static void geocentric_to_ecef( double gclat, double gclon, double r,
                                double *X, double *Y, double *Z)
{
   double colat = M_PI/2 - gclat;

   *X = r * sin( colat) * cos( gclon);
   *Y = r * sin( colat) * sin( gclon);
   *Z = r * cos( colat);
}

static void geodetic_to_geocentric( double h, double lat, double lon,
                                    double *gclat, double *gclon, double *r)
{
   double X, Y, Z;
   geodetic_to_ecef( h, lat, lon, &X, &Y, &Z);
   VT_report( 1, "X=%.6f y=%.6f z=%.6f", X, Y, Z);
   ecef_to_geocentric( X, Y, Z, gclat, gclon, r);
}

//
// Heikkinen. https://hal.science/hal-01704943v2/document
//

static void ecef_to_geodetic( double X, double Y, double Z,
                              double *lat, double *lon, double *h)
{
   double e2 = (ERAD_a2 - ERAD_b2) / ERAD_a2;
   double p = sqrt( X*X + Y*Y);
   double F = 54 * ERAD_b2 * Z * Z;
   double G = p*p + (1 - e2) * Z*Z - e2 * (ERAD_a2 - ERAD_b2);
   double c = e2 * e2 * F * p * p/(G * G * G);
   double s = pow( 1 + c + sqrt(c*c + 2*c), 1.0/3);
   double k = s + 1 + 1/s;
   double P = F / (3 * k*k * G*G);
   double Q = sqrt(1 + 2 * e2 * e2 * P);

   double ta = 0.5 * ERAD_a2 * (1 + 1/Q);
   double tb = P * (1 - e2) * Z*Z / Q / (1 + Q);
   double tc = 0.5 * P * p*p;
   double r0 = -P * e2 * p/(1 + Q) + sqrt(ta - tb - tc);

   double td = p - e2 * r0;
   double U = sqrt( td*td + Z*Z);
   double V = sqrt( td*td + (1 - e2) * Z*Z);

   double z0 = ERAD_b2 * Z / ERAD_a / V;

   *h = U * (1 - ERAD_b2/ERAD_a/V);
   *lat = atan2( Z + (ERAD_a2 - ERAD_b2)/ERAD_b2 * z0, p);
   *lon = atan2( Y, X);
}

static void geocentric_to_geodetic( double gclat, double gclon, double r,
                   double *lat, double *lon, double *h)
{
   double X, Y, Z;
   geocentric_to_ecef( gclat, gclon, r, &X, &Y, &Z);
   ecef_to_geodetic( X, Y, Z, lat, lon, h);
}

//
//  Rotate Br and Blat to allow for the small angle between the local vertical
//  and the geocentric vertical.
//

static void field_to_geodetic( double gdlat, double gdlon, double h,
                               double *Br, double *Blon, double *Blat)
{
   double e = sqrt( 1 - ERAD_b2/ERAD_a2);
   double f = (ERAD_a - ERAD_b)/ERAD_a;
   double N = ERAD_a / sqrt( 1 - e * e * sin( gdlat) * sin( gdlat));

   double theta = atan2( (N * (1-f) * (1-f) + h) * tan( gdlat), N + h);
   double angle = gdlat - theta;

   double sa = sin(angle);
   double ca = cos(angle);

   double tr = *Br;
   double tx = *Blat;
   *Blat = *Blat * ca + tr * sa;
   *Br = tr * ca - tx * sa;
}

static void igrf_field( double gclat, double gclon, double r, double year,
                        double *dv_dr, double *dv_dlon, double *dv_dlat,
                        int maxdeg)
{
   if (maxdeg == 0) maxdeg = MAXDEG;

   int iy = floor( year);
   if (iy > 2020 - 1970) iy = 2020 - 1970;
   double fy = year - iy;

   double colat = M_PI/2 - gclat;
   double x = cos( colat);

   double sum_dv_dr = 0;
   double sum_dv_dlon = 0;
   double sum_dv_dlat = 0;

   int n;
   for (n = 1; n <= maxdeg; n++) 
   {
      double anr = pow( EARTH_RAD / r, n + 1);

      int m;
      for (m = 0; m <= n; m++)
      {
         struct IGRF *g = &gauss_g[n][m][iy],
                     *h = &gauss_h[n][m][iy];

         double coef_g = g->coef + g->sv * fy,
                coef_h = h->coef + h->sv * fy;

         double Pnm = legendre( n, m, x);
         double norm_Pnm = schmidt_seminorm( n, m, Pnm);

         double C = coef_g * cos( m * gclon) + coef_h * sin( m * gclon);
         double D = - coef_g * m * sin( m * gclon) 
                    + coef_h * m * cos( m * gclon);

         sum_dv_dr += -(n+1) * anr/r * C * norm_Pnm;
         sum_dv_dlon += -anr * D * norm_Pnm;

         double y = -(n + 1) * x * Pnm +
                   (n - m + 1) * legendre( n + 1, m, x);
         y /= x*x - 1;
         y = schmidt_seminorm( n, m, y);

         sum_dv_dlat += -y * anr * C;
      }
   }

   //
   // A final scaling by EARTH_RAD, and convert the angular components from
   // radians to metres.
   //

   *dv_dr = sum_dv_dr * EARTH_RAD;
   *dv_dlon = sum_dv_dlon / sin( colat) * EARTH_RAD/r;
   *dv_dlat = sum_dv_dlat * sin( colat) * EARTH_RAD/r;
}

static void do_geomag( V3 p, timestamp T)
{
   // Year offset from 1970
   double year = timestamp_secs( T) / (86400 * 365.25);

   double h = 0, // Height above the WGS ellipsoid, km
          gdlat, gdlon;   // Geodetic coordinates, radians
   v3_latlon( p, &gdlat, &gdlon);

   double r, gclat, gclon; // Geocentric spherical coordinates
   geodetic_to_geocentric( h, gdlat, gdlon, &gclat, &gclon, &r);

   VT_report( 1, "geocentric lat %.2f lon %.2f r %.1f",
             gclat * 180/M_PI, gclon * 180/M_PI, r);

   // Magnetic field, nT
   double Br;          // Radial compoment, +ve down
   double Blon;        // Longitudinal component, east +ve
   double Blat;        // Latitudinal component, north +ve

   //
   // OPT_M == 1 : Calculate the geomagnetic field with full IGRF
   // OPT_M == 3 : Same, but only use the dipole field of the IGRF
   //

   if (OPT_M == 1 || OPT_M == 3)
   {
      int maxdeg = OPT_M == 3 ? 1 : 0;
      igrf_field( gclat, gclon, r, year, &Br, &Blon, &Blat, maxdeg);
      field_to_geodetic( gdlat, gdlon, h, &Br, &Blon, &Blat);

      if (OMODE_EXT)
      {
         VT_printf( "Blat %.1f Blon %.1f Br %.1f", Blat, Blon, Br);

         double Btot = sqrt( Br * Br + Blon * Blon + Blat * Blat);
         double Bhor = sqrt( Blon * Blon + Blat * Blat);
         VT_printf( " Bhor %.1f Btot %.1f", Bhor, Btot);

         double incl = atan2( Br, Bhor) * 180/M_PI;
         double decl = atan2( Blon, Blat) * 180/M_PI;
         VT_printf( " inc %.1f dec %.1f", incl, decl);

         double Glat = asin( Br/(2 * 31200));
         VT_printf( " Glat %.1f", Glat * 180/M_PI);
      }
      else
         VT_printf( " %.1f %.1f %.1f", Blat, Blon, Br);
   }

   //
   // OPT_M == 2 : Follow a field line to the conjugate point
   // OPT_M == 4 : Same, but only use the dipole field of the IGRF
   //

   if (OPT_M == 2 || OPT_M == 4)
   {
      double ds = 10;    // Step length along the field line, km

      int m = 0;         // Sign of Br at the standpoint,
                         // +1 if we start in the North, downward (+ve) Br,
                         // -1 if we start in the South, upward (-ve) Br;

      double L = 0; // Set to the max value of r/EARTH_RAD along the field line

      // OPT_M == 2 : Use the full IGRF
      // OPT_M == 4 : Use only the dipole component

      int maxdeg = OPT_M == 4 ? 1 : 0;

      while (1)
      {
         if (L < r / EARTH_RAD) L = r / EARTH_RAD;

         igrf_field( gclat, gclon, r, year, &Br, &Blon, &Blat, maxdeg);
         if (!m) m = Br < 0 ? -1 : 1;

         Blat *= -m;
         Blon *= -m;
         Br *= m;
      
         double a = sqrt( Br * Br + Blon * Blon + Blat * Blat) / ds;

         Blat /= a;
         Blon /= a;
         Br /= a;

         gclat += Blat/r;
         gclon += Blon/(r * cos( gclat));
         r += Br;            

         geocentric_to_geodetic( gclat, gclon, r, &gdlat, &gdlon, &h);
         if (h <= 0) break;
      } 

      VT_printf( "%.3f,%.3f", gdlat * 180/M_PI, gdlon * 180/M_PI);
      if (OMODE_EXT) VT_printf( " %.2f", L);
   }
}

///////////////////////////////////////////////////////////////////////////////
//                                                                           //
///////////////////////////////////////////////////////////////////////////////

//
// Called when -g option is given.  Output a list of great circle coordinates.
//

static void run_g_option( int argc, char *argv[])
{
   if (argc >= 2 && argc <= 4)
   {
      int k = 0;
      struct SITE *s1 = parse_latlon( argv[k++]);
      struct SITE *s2 = parse_latlon( argv[k++]);

      double angle = OPT_g == 2 ? 2 * M_PI : v3_angle( s1->v, s2->v);

      timestamp T = timestamp_NONE;
      if (OPT_s || OPT_M)  // Expecting a timestamp field now?
      {
         if (k >= argc) VT_bailout( "missing timestamp field");
         T = VT_parse_timestamp( argv[k++]);
      }

      double step = angle/100;
      if (k == argc - 1) step = atof( argv[k]) * M_PI/180;
  
      // The great circle between the two sites.
   
      V3 gc; v3_unit_normal_to( s1->v, s2->v, gc);
   
      // Adjust step size to give an integer number of points on the GC
   
      int istep = 0.5 + angle/step;
      step = angle/istep;
   
      // Generate points.
   
      int i;
      for (i = 0; i <= istep; i++)
      {
         A3 r1; a3_rot( gc, i * step, r1);
         V3 p;  v3_transform( s1->v, r1, p);

         VT_printf( "%s", v3_string( p, NULL));
         if (OPT_s)
         {
            double solaz, solel;
            astropos1( p, T, &solaz, &solel);
            VT_printf( " %.1f %.1f",
                 solaz * 180/M_PI, solel * 180/M_PI);
         }

         if (OPT_M) do_geomag( p, T);

         VT_printf( "\n");
      }
   }
   else usage();
}

static void run_p_option( int argc, char *argv[])
{
   struct SITE *s1;
   struct SITE *s2;
   struct SITE *s3;
   V3 gc, gcx;

   if (!argc)
   {
      // Read standpoint azimuth range_km from stdin
      char temp[512];
      while (fgets( temp, 512, stdin))
      {
         // Format: standpoint forepoint crosspoint [rest]

         char *stdpt = strtrim( temp);
         if (!*stdpt)  // Let blank lines through without complaint
         {
            VT_printf( "\n");
            continue;
         }

         // Skip over standpoint to reach forepoint
         char *forpt = skip_field( stdpt);
         if (!*forpt) VT_bailout( "missing forepoint");
         *forpt++ = 0;   // Terminate standpoint
         forpt = skip_space( forpt);

         // Skip over forepoint to reach crosspoint
         char *crspt = skip_field( forpt);
         if (!*crspt) VT_bailout( "missing crosspoint");
         *crspt++ = 0;   // Terminate forepoint
         crspt = skip_space( crspt);

         char *rest = skip_field( crspt);
         if (*rest) *rest++ = 0;  // Terminate crspt
         if (*rest) skip_space( rest);

         s1 = parse_latlon( stdpt);
         s2 = parse_latlon( forpt);
         s3 = parse_latlon( crspt);

         // The great circle and distance between the two sites.
         v3_unit_normal_to( s1->v, s2->v, gc);
         double pd = v3_range( s1->v, s2->v);

         // Signed angle between point s3 and the GC normal vector
         // -ve if s3 is left of GC, +ve if to the right
         double a = v3_angle( gc, s3->v) - M_PI/2;

         VT_printf( "%s %s %s %.3f", stdpt, forpt, crspt, a * EARTH_RAD);

         v3_unit_normal_to( gc, s3->v, gcx);

         V3 ip1;  v3_unit_normal_to( gc, gcx, ip1);
         V3 ip2;  v3_scale( ip1, -1, ip2);

         double d1 = v3_range( s3->v, ip1);
         double d2 = v3_range( s3->v, ip2);

         if (d1 < d2)
         {
            double pf = v3_range( s1->v, ip1) / pd;
            VT_printf( " %s %.3f", v3_string( ip1, NULL), pf);
         }
         else
         {
            double pf = v3_range( s1->v, ip2) / pd;
            VT_printf( " %s %.3f", v3_string( ip2, NULL), pf);
         }

         rest ? VT_printf( " %s\n", rest) : VT_printf( "\n");
         clear_sites();
      }
   }
   else
   if (argc == 3)
   {
      s1 = parse_latlon( argv[0]);
      s2 = parse_latlon( argv[1]);
      s3 = parse_latlon( argv[2]);

      // The great circle and distance between the two sites.
      v3_unit_normal_to( s1->v, s2->v, gc);
      double pd = v3_range( s1->v, s2->v);

      // Signed angle between point s3 and the GC normal vector
      // -ve if s3 is left of GC, +ve if to the right
      double a = v3_angle( gc, s3->v) - M_PI/2;

      v3_unit_normal_to( gc, s3->v, gcx);

      V3 ip1;  v3_unit_normal_to( gc, gcx, ip1);
      V3 ip2;  v3_scale( ip1, -1, ip2);

      double d1 = v3_range( s3->v, ip1);
      double d2 = v3_range( s3->v, ip2);

      VT_printf( "%.3f", a * EARTH_RAD);

      if (d1 < d2)
      {
         double pf = v3_range( s1->v, ip1) / pd;
         VT_printf( " %s %.3f\n", v3_string( ip1, NULL), pf);
      }
      else
      {
         double pf = v3_range( s1->v, ip2) / pd;
         VT_printf( " %s %.3f\n", v3_string( ip2, NULL), pf);
      }
   }
   else usage();
}

static void run_d_option( int argc, char *argv[])
{
   if (!argc)
   {
      // Read standpoint azimuth range_km from stdin
      char temp[512];
      while (fgets( temp, 512, stdin))
      {
         // Format: standpoint bearing range [rest]

         char *stdpt = strtrim( temp);
         if (!*stdpt)  // Let blank lines through without complaint
         {
            VT_printf( "\n");
            continue;
         }

         // Skip over standpoint to reach azimuth
         char *az = skip_field( stdpt);
         if (!*az) VT_bailout( "missing azimuth");
         *az++ = 0;   // Terminate standpoint
         az = skip_space( az);

         char *range = skip_field( az);
         if (!*range) VT_bailout( "missing range");
         *range++ = 0;    // Terminate az
         range = skip_space( range);

         char *rest = skip_field( range);
         if( rest) *rest++ = 0;    // Terminate range

         V3 vf;
         struct SITE *s = parse_latlon( stdpt);
         double b = atof( az) * M_PI/180;
         double a = atof( range)/EARTH_RAD;
         destination_point( s->v, b, a, vf);
         VT_printf( "%s %s %s %s", stdpt, az, range, v3_string( vf, NULL));
         
         rest ? VT_printf( " %s\n", rest) : VT_printf( "\n");
         clear_sites();
      }
   }
   else
   if (argc == 3)
   {
      struct SITE *s = parse_latlon( argv[0]);
      double b = atof( argv[1]) * M_PI/180;
      double a = atof( argv[2])/EARTH_RAD;

      V3 vf;
      destination_point( s->v, b, a, vf);
      VT_printf( "%s\n", v3_string( vf, NULL));
   }
   else
      usage();
}

static void run_b_option( int argc, char *argv[])
{
   if (!argc)
   {
      // Read standpoint and forepoint from standard input
      // Optional timestamp field if OPT_s also set

      char temp[512];
      while (fgets( temp, 512, stdin))
      {
         // Format: standpoint forepoint [timestamp] rest
 
         char *stdpt = strtrim( temp);
         if (!*stdpt)  // Let blank lines through without complaint
         {
            VT_printf( "\n");
            continue; 
         }

         // Skip over standpoint to reach forepoint
         char *forpt = skip_field( stdpt);
         if (!*forpt) VT_bailout( "missing forepoint");
         *forpt++ = 0;   // Terminate standpoint
         forpt = skip_space( forpt);

         // Skip over forepoint
         char *rest = skip_field( forpt);
         if (*rest) *rest++ = 0;  // Terminate forepoint
         if (*rest) skip_space( rest);

         timestamp T = timestamp_NONE;
         char *ts = NULL;
         if (OPT_s)  // Expecting a timestamp field now?
         {
            ts = rest;
            if (!*ts) VT_bailout( "missing timestamp field");
            rest = skip_field( ts);
            if (*rest) *rest++ = 0;  // Terminate timestamp

            T = VT_parse_timestamp( ts);
         }

         struct SITE *s1 = parse_latlon( stdpt);
         struct SITE *s2 = parse_latlon( forpt);

         double d = v3_range( s1->v, s2->v);
         double b = v3_bearing( s1->v, s2->v);
         b = constrain( b, 0, 2*M_PI);
         VT_printf( "%s %s", stdpt, forpt);
         if (OPT_s) VT_printf( " %s", ts);
         VT_printf( " %.3f %.1f", d, b * 180/M_PI);

         if (OPT_s)
            VT_printf( " %.2f", path_illumination1( s1->v, s2->v, T));
       
         rest ? VT_printf( " %s\n", rest) : VT_printf( "\n"); 

         clear_sites();
      }
   }
   else
   if (argc == 2)
   {
      // Standpoint and forepoint are next on the command line
      struct SITE *s1 = parse_latlon( argv[0]);  // Standpoint
      struct SITE *s2 = parse_latlon( argv[1]);  // Forepoint

      double d = v3_range( s1->v, s2->v);
      double b = v3_bearing( s1->v, s2->v);
      b = constrain( b, 0, 2*M_PI);
      VT_printf( "%.3f %.1f\n", d, b * 180/M_PI);
   }
   else
   if (argc == 3)
   {
      // Standpoint and forepoint are next on the command line, followed
      // by a timestamp
      struct SITE *s1 = parse_latlon( argv[0]);  // Standpoint
      struct SITE *s2 = parse_latlon( argv[1]);  // Forepoint
      timestamp T = VT_parse_timestamp( argv[2]);
      double d = v3_range( s1->v, s2->v);
      double b = v3_bearing( s1->v, s2->v);
      b = constrain( b, 0, 2*M_PI);
      VT_printf( "%.3f %.1f %.2f\n", d, b * 180/M_PI,
                 path_illumination1( s1->v, s2->v, T));
   }
   else usage();
}

static void run_q_option( int argc, char *argv[])
{
   if (OPT_q == 1)
   {
      int km;
      for (km = 10; km < 20000; km += 10)
      {
         double vf_day = group_velocity( M_PI, km, 1)/300e3;
         double vf_night = group_velocity( M_PI, km, 0)/300e3;

         VT_printf( "%d %.4f %.4f\n", km, vf_day, vf_night);
      }

      return;
   }

   if (OPT_q == 3 && argc == 2)
   {
      double km = atof( argv[0]);
      double illum = atof( argv[1]);
      VT_printf( "%.4f\n", group_velocity( M_PI, km, illum)/300e3);
      return;
   }

   if (OPT_q == 4)
   {
      switch( copt_model)
      {
         case MODEL_VLF2:  vfmodel_vlf2_dump();   break;
         case MODEL_ELF2:  vfmodel_elf2_dump();   break;

         default: VT_bailout( "no VF table dump for this model");
      }

      return;
   }

   if (OPT_q != 2) VT_bailout( "invalid -q argument");

   if (!argc)
   {
      // Read standpoint, forepoint, timestamp, rest from standard input

      char temp[512];
      while (fgets( temp, 512, stdin))
      {
         // Format: standpoint forepoint timestamp rest
         
         char *stdpt = strtrim( temp);
         if (!*stdpt)  // Let blank lines through without complaint
         {
            VT_printf( "\n");
            continue; 
         }
         
         // Skip over standpoint to reach forepoint
         char *forpt = skip_field( stdpt);
         if (!*forpt) VT_bailout( "missing forepoint");
         *forpt++ = 0;   // Terminate standpoint
         forpt = skip_space( forpt);
         
         // Skip over forepoint to reach timestamp
         char *ts = skip_field( forpt);
         if (!*ts) VT_bailout( "missing timestamp field");
         *ts++ = 0;   // Terminate forepoint
         char *rest = skip_space( ts);
         if (*rest) *rest++ = 0;  // Terminate timestamp

         timestamp T = VT_parse_timestamp( ts);
         struct SITE *s1 = parse_latlon( stdpt);
         struct SITE *s2 = parse_latlon( forpt);

         double d = v3_range( s1->v, s2->v);
         double t = group_delay2( s1->v, s2->v, T);
         VT_printf( "%s %s %s %.4f", stdpt, forpt, ts, d/t/300e3);
         
         rest ? VT_printf( " %s\n", rest) : VT_printf( "\n");
         clear_sites();
      }
   }
   else
   if (argc == 3)
   {
      // Standpoint and forepoint are next on the command line, followed
      // by a timestamp
      struct SITE *s1 = parse_latlon( argv[0]);  // Standpoint
      struct SITE *s2 = parse_latlon( argv[1]);  // Forepoint
      timestamp T = VT_parse_timestamp( argv[2]);
      double d = v3_range( s1->v, s2->v);
      double t = group_delay2( s1->v, s2->v, T);
      VT_printf( "%.4f\n", d/t/300e3);
   }
   else usage();
}

static void run_s_option( int argc, char *argv[])
{
   if (!argc)
   {
      // Read lat,lon and timestamp from standard input
 
      char temp[512]; 
      while (fgets( temp, 512, stdin))
      {
         char *loc = strtrim( temp);

         if (!*loc)  // Let blank lines through without complaint
         {
            VT_printf( "\n");
            continue;
         }

         // Skip over location to reach timestamp
         char *ts = skip_field( loc);
         if (!*ts) VT_bailout( "missing timestamp");
         *ts++ = 0;  // Terminate location
         ts = skip_space( ts);

         // Skip over timestamp 
         char *rest = skip_field( ts);
         if (rest) *rest++ = 0;  // Terminate timestamp

         struct SITE *s = parse_latlon( loc);
         timestamp T = VT_parse_timestamp( ts);
         double solaz, solel;
         astropos1( s->v, T, &solaz, &solel);
         VT_printf( "%s %s %.1f %.1f",
                     loc, ts, solaz * 180/M_PI, solel * 180/M_PI);
         rest ? VT_printf( " %s\n", rest) : VT_printf( "\n");
         clear_sites();
      }
   }
   else
   if (argc == 2)
   {
      struct SITE *s = parse_latlon( argv[0]);
      timestamp T = VT_parse_timestamp( argv[1]);
      double solaz, solel;
      astropos1( s->v, T, &solaz, &solel);
      VT_printf( "%.1f %.1f\n", solaz * 180/M_PI, solel * 180/M_PI);
   }
   else
      usage();
}

static void run_M_option( int argc, char *argv[])
{
   if (!argc)
   {
      // Read lat,lon and timestamp from standard input
 
      char temp[512]; 
      while (fgets( temp, 512, stdin))
      {
         char *loc = strtrim( temp);

         if (!*loc)  // Let blank lines through without complaint
         {
            VT_printf( "\n");
            continue;
         }

         // Skip over location to reach timestamp
         char *ts = skip_field( loc);
         if (!*ts) VT_bailout( "missing timestamp");
         *ts++ = 0;  // Terminate location
         ts = skip_space( ts);

         // Skip over timestamp 
         char *rest = skip_field( ts);
         if (rest) *rest++ = 0;  // Terminate timestamp

         struct SITE *s = parse_latlon( loc);
         timestamp T = VT_parse_timestamp( ts);
       
         VT_printf( "%s %s ", loc, ts); 
         do_geomag( s->v, T); 
         rest ? VT_printf( " %s\n", rest) : VT_printf( "\n");
         clear_sites();
      }
   }
   else
   if (argc == 2)
   {
      struct SITE *s = parse_latlon( argv[0]);
      timestamp T = VT_parse_timestamp( argv[1]);

      do_geomag( s->v, T);
      VT_printf( "\n");
   }
   else
      usage();
}

static void run_P_option( int argc, char *argv[])
{
   char *list[MAXSITES];
   int nlist = 0;

   char *p = strtrim( strdup( pointlist));
   while (*p)
   {
      char *t = skip_field( p);
      if (*t) *t++ = 0;

      list[nlist++] = p;
      p = t;
   }

   double az[MAXSITES];
   double dsum;

   if (argc == 1)
   {
      struct SITE *s1 = parse_latlon( argv[0]);  // Standpoint

      int i;
      for (i = 0; i < nlist; i++)
      {
         struct SITE *s = parse_latlon( list[i]);
         az[i] = v3_bearing( s->v, s1->v);
      }

      dsum = 0;
      for (i = 0; i < nlist; i++)
      {
         int j = (i + 1) % nlist;
         double d = az[j] - az[i];
         if (d < -M_PI) d += 2 * M_PI;
         if (d >= M_PI) d -= 2 * M_PI;
         dsum += d;
      }

      VT_printf( "%d\n", (int) round( dsum / (2 * M_PI)));
   }
   else
   if (argc) VT_bailout( "incorrect arguments to -P");
   else
   {
      char temp[512];
      while (fgets( temp, 512, stdin))
      {
         // Format: standpoint [rest]

         char *stdpt = strtrim( temp);
         if (!*stdpt)  // Let blank lines through without complaint
         {
            VT_printf( "\n");
            continue;
         }

         // Skip over standpoint
         char *rest = skip_field( stdpt);
         if (*rest) *rest++ = 0;  // Terminate standpoint
         if (*rest) skip_space( rest);

         struct SITE *s1 = parse_latlon( stdpt);

         int i;
         for (i = 0; i < nlist; i++)
         {
            struct SITE *s = parse_latlon( list[i]);
            az[i] = v3_bearing( s->v, s1->v);
         }

         dsum = 0;
         for (i = 0; i < nlist; i++)
         {
            int j = (i + 1) % nlist;
            double d = az[j] - az[i];
            if (d < -M_PI) d += 2 * M_PI;
            if (d >= M_PI) d -= 2 * M_PI;
            dsum += d;
         }

         VT_printf( "%s %d", stdpt, (int) round( dsum / (2 * M_PI)));
         rest ? VT_printf( " %s\n", rest) : VT_printf( "\n");

         clear_sites();
      }
   }
}

static void compute_centroid( void)
{
   V3 vsum = { 0, 0, 0};

   char *p = strtrim( strdup( pointlist));

   while (*p)
   {
      char *t = skip_field( p);
      if (*t) *t++ = 0;

      struct SITE *s = parse_latlon( p);
      v3_add( s->v, vsum, vsum);

      p = t;
   }

   char *t = v3_string( vsum, NULL);
   VT_printf( "%s\n", t);
}

///////////////////////////////////////////////////////////////////////////////
//                                                                           //
///////////////////////////////////////////////////////////////////////////////

static void parse_model( char *arg)
{
   if (!strcasecmp( optarg, "model_fixed")) copt_model = MODEL_FIXED;
   else
   if (!strcasecmp( optarg, "model_vlf0")) copt_model = MODEL_VLF0;
   else
   if (!strcasecmp( optarg, "model_vlf1")) copt_model = MODEL_VLF1;
   else
   if (!strncasecmp( optarg, "model_vlf2=", 11))
   {
      copt_model = MODEL_VLF2;
      vfmodel_loadname = strdup( optarg + 11);
   }
   else
   if (!strcasecmp( optarg, "model_vlf2")) copt_model = MODEL_VLF2;
   else
   if (!strcasecmp( optarg, "model_elf1")) copt_model = MODEL_ELF1;
   else
   if (!strncasecmp( optarg, "model_elf2=", 11))
   {
      copt_model = MODEL_ELF2;
      vfmodel_loadname = strdup( optarg + 11);
   }
   else
   if (!strcasecmp( optarg, "model_elf2")) copt_model = MODEL_ELF2;
   else
   if (!strcasecmp( optarg, "model_elf3")) copt_model = MODEL_ELF3;
   else
   if (!strcasecmp( optarg, "model_elf4")) copt_model = MODEL_ELF4;
   else
   if (isdigit( *optarg))
   {
      CVLF = 300e3 * atof( optarg);
      fixed_VF = atof( optarg);
      cvlf_given = TRUE;
   }
   else VT_bailout( "invalid -c argument");
}

int main( int argc, char *argv[])
{
   VT_init( "vtspot");

   load_spots();

   while (1)
   {
      int c = getopt( argc, argv, "vxL:c:m:n:r:f:o:U:q:a:R:S:gGsbdCpPM:?");

      if (c == 'v') VT_up_loglevel();
      else
      if (c == 'L') VT_set_logfile( "%s", optarg);
      else
      if (c == 'b') OPT_b = TRUE;
      else
      if (c == 'U') ncpu = atoi( optarg);
      else
      if (c == 'c') parse_model( optarg);
      else
      if (c == 'd') OPT_d = 1;
      else
      if (c == 's') OPT_s = 1;
      else
      if (c == 'M') OPT_M = atoi( optarg);
      else
      if (c == 'p') OPT_p = 1;
      else
      if (c == 'g') OPT_g = 1;
      else
      if (c == 'G') OPT_g = 2;
      else
      if (c == 'q') OPT_q = atoi( optarg);
      else
      if (c == 'r') max_residual = atof( optarg);
      else
      if (c == 'n') minsites = atoi( optarg);
      else
      if (c == 'f')   // Filtering options
      {
         if (!strncmp( optarg, "envelop=", 8))
            filter_envelop = atof( optarg + 8);
         else
         if (!strncmp( optarg, "nearmax=", 8))
            filter_nearmax = atof( optarg + 8);
         else
         if (!strncmp( optarg, "maxatd=", 7))
            maxatd = atof( optarg + 7);
         else
         if (!strcmp( optarg, "shortsun")) shortsun = TRUE;
         else
         if (!strncmp( optarg, "illum=", 6)) illum_type = atoi( optarg+6);
         else
            VT_bailout( "unrecognised -f option [%s]", optarg);
      }
      else
      if (c == 'o')   // Output options
      {
         if (!strcmp( optarg, "iso")) OMODE_ISO = TRUE;
         else
         if (!strcmp( optarg, "ext")) OMODE_EXT = TRUE;
         else
         if (!strcmp( optarg, "brc")) OMODE_BRC = TRUE;
         else
            VT_bailout( "unrecognised -o option [%s]", optarg);
      }
      else
      if (c == 'a')  // Sigma options
      {
         if (!strncmp( optarg, "t=", 2)) DEFAULT_AT_SIGMA = atof( optarg + 2);
         else
         if (!strncmp( optarg, "z=", 2)) DEFAULT_AZ_SIGMA = atof( optarg + 2);
         else
            VT_bailout( "unrecognised -a option [%s]", optarg);
      }
      else
      if (c == 'm') parse_matching( strdup( optarg));
      else
      if (c == 'C') OPT_C = TRUE;
      else
      if (c == 'P') OPT_P = TRUE;
      else
      if (c == 'S') pointlist = strdup( optarg);
      else
      if (c == 'R') opt_radec = strdup( optarg);
      else
      if (c == -1) break;
      else
         usage();
   }

   if (ncpu < 1) VT_bailout( "invalid number of worker threads");

   if (OPT_M) load_igrf();

   setup_vfmodel();

   if (OPT_b)
   {
      run_b_option( argc - optind, argv + optind);
      return 0;
   }

   if (OPT_q)
   {
      run_q_option( argc - optind, argv + optind);
      return 0;
   }

   if (OPT_g)
   {
      run_g_option( argc - optind, argv + optind);
      return 0;
   }

   if (OPT_p)
   {
      run_p_option( argc - optind, argv + optind);
      return 0;
   }

   if (OPT_d)
   {
      run_d_option( argc - optind, argv + optind);
      return 0;
   }

   if (OPT_s)
   {
      run_s_option( argc - optind, argv + optind);
      return 0;
   }

   if (OPT_P)
   {
      run_P_option( argc - optind, argv + optind);
      return 0;
   }

   if (OPT_M)
   {
      run_M_option( argc - optind, argv + optind);
      return 0;
   }

   if (OPT_C)
   {
      if (!pointlist) VT_bailout( "-C option needs -S to list points");
      compute_centroid();
      return 0;
   }

   if (optind < argc)
   {
      // Take a measurement set from the command line
      struct MSET m;
      init_measurement_set( &m);
      reset_measurement_set( &m);
      while (optind < argc) parse_measurement( &m, argv[optind++]);
      m.use_illum = TRUE;
      process_measurement_set( &m);
      return 0;
   }

   if (nsites)  // Some -m options given?
   {
      time_t tstart = time( NULL);   // To measure running time
      if (!minsites) minsites = nsites;
      setup_tables();
      load_matching_files();
      run_matching();

      VT_report( 1, "elapsed: %d seconds", (int)(time( NULL) - tstart));
      return 0;
   }

   //
   // Read measurement sets from stdin, one set per line, and process each.
   //

   char *inbuf = malloc( 4096), *p, *q;

   struct MSET m;
   init_measurement_set( &m);

   while (fgets( inbuf, 4096, stdin))
   {
      reset_measurement_set( &m);

      p = strtrim( inbuf);

      while (*p)
      {
         p = skip_space( p);
         if (!*p) break;
      
         q = skip_field( p);
         if (*q) *q++ = 0; 

         parse_measurement( &m, p);
         p = q;
      }

      m.use_illum = TRUE;
      process_measurement_set( &m);
   }

   return 0;
}

