//
// Created by eureka on 4/21/21.
//

#ifndef GLOBAL_H_
#define GLOBAL_H_
#include <time.h>
#include "rnx.h"

#ifdef GLOBAL_VAR
#define GLOBAL_EXTERN
#else
#define GLOBAL_EXTERN extern
#endif

#define FREQ1       1.57542E9           /* L1/E1  frequency (Hz) */
#define FREQ2       1.22760E9           /* L2     frequency (Hz) */
#define FREQ5       1.17645E9           /* L5/E5a frequency (Hz) */
#define FREQ6       1.27875E9           /* E6/LEX frequency (Hz) */
#define FREQ7       1.20714E9           /* E5b    frequency (Hz) */
#define FREQ8       1.191795E9          /* E5a+b  frequency (Hz) */
#define FREQ1_CMP   1.561098E9          /* BeiDou B1 frequency (Hz) */
#define FREQ2_CMP   1.20714E9           /* BeiDou B2 frequency (Hz) */
#define FREQ3_CMP   1.26852E9           /* BeiDou B3 frequency (Hz) */

#define PI 3.1415926535897932
#define DEG2RAD  (PI/180.0)
#define RAD2DEG  (180.0/PI)
#define RE_WGS84    6378137.0           /* earth semimajor axis (WGS84) (m) */
#define FE_WGS84    (1.0/298.257223563) /* earth flattening (WGS84) */

static  double CLIGHT = 299792458.0;

#define MU_GPS   3.9860050E14
#define MU_CMP   3.986004418E14   // earth gravitational constant
#define OMGE_GPS 7.2921151467E-5
#define OMGE_CMP 7.292115E-5
#define SIN_5 -0.0871557427476582 /* sin(-5.0 deg) */
#define COS_5  0.9961946980917456 /* cos(-5.0 deg) */

static const double gpst0[] = { 1980,1,6,0,0,0 }; //GPS
static const double gst0[] = { 1999,8,22,0,0,0 };   //GALILEO
static const double bdt0[] = { 2006,1,1,0,0,0 };    //BEIDOU

static const int leap_sec_bds2gps = 14;

GLOBAL_EXTERN char SYS[16];
GLOBAL_EXTERN char OBSTYPE[17];
GLOBAL_EXTERN CKDCFG CKF;

GLOBAL_EXTERN int neph[MAXSYS];     //number of ephemeris of each system
GLOBAL_EXTERN GPS_BRDEPH ephgps[MAXSYS][MAXEPH];  //ephemeris of each system

#define INFO_SPP 0
#define INFO_FLOAT 1
#define INFO_FIXED 2



constexpr double CPT_SAMP_RATE = 100;		//HZ
constexpr double CPT_LSB_ACCEL = 0.05 / 32768.0;
constexpr double CPT_LSB_GYRO = 0.1 / 3600.0 / 256.0;

// constexpr double PI = 3.14159265358;
constexpr double rad2deg = 180.0 / PI;
constexpr double deg2rad = PI / 180;
GLOBAL_EXTERN Ellipsoid  ELL;


#endif //RTK_GLOBAL_H
