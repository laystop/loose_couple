//
// Created by eureka on 4/21/21.
//
#ifndef RNX_H_
#define RNX_H_

#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "common.h"
#include "config.h"


// GPS Ephemeris
typedef struct {
	char cprn[4];
	int mjd;
	double sod;
	// a[0]: SV clock offset
	// a[1]: SV clock drift
	// a[2]: SV clock drift rate
	double a0, a1, a2;
	// aode: age of ephemeris upload
	// crs, crc: Ortital radius correction
	// dn: Mean motion difference
	// m0: Mean anomaly at reference epoch
	// e: Eccentricity
	// cuc, cus: Latitude argument correction
	// roota: Square root of semi-major axis
	double aode, crs, dn;
	double m0, cuc, e;
	double cus, roota;
	// toe, week: Ephemerides reference epoch in seconds with the week
	// cis, cic: Inclination correction
	// Omega0: Longtitude of ascending node at the begining of the week
	// i0: Inclination at reference epoch
	// omega: Argument of perigee
	// omegadot: Rate of node's right ascension
	double toe, cic, Omega0;
	double cis, i0, crc;
	double omega, Omega_dot;
	// idot: Rate of inclination angle
	// sesvd0:
	// resvd1:
	// accu: SV accuracy
	// hlth: SV health
	// tgd: Time group delay
	// aodc: Age of clock parameter upload
	double i_dot, resvd0, week, resvd1;
	double accu, hlth, tgd, tgd1, aodc;
	// added for b2b
	int signal_idx;
	unsigned int iodc;
	double tgd_BDS[13], isc_BDS[13];
	double delta_A, A_DOT, delta_n_dot;
} GPS_BRDEPH;


// GNSS NAVIGATION MESSAGE FILE     HEADER SECTION
typedef struct {
	double ver;
	// ionospheric correction parameters
	char ionc[MAXSYS][2][LEN_EPHION];
	double ion[MAXSYS][2][4];
	// corrections to transform the system to UTC or other time systems

	char timc[MAXSYS][2][LEN_EPHION];
	double tim[MAXSYS][2][4];
	// number of leap second since 6-Jan-980
	int leap;
} BRDHEAD;


/* Observations
 * at one single epoch
 * */
typedef struct {
	gtime_t t;
	int jd;
	double tsec;        //sod

	int flag;
	int nprn;           //num of satellites (prn)
	double clk_offset;   //optional
	char cprn[MAXSAT][LEN_OBSTYPE];    //char PRN of satellites
	char obs_type[MAXOBSTYPE][LEN_OBSTYPE];  //type of observations

	//observations of satellites, in sync with prn[]
	//obs[][0] --- L for 1st Freq
	//obs[][1] --- L for 2nd Freq
	//obs[][2] --- L for 3rd Freq
	//...
	//obs[][] --- C
	//obs[][] --- D
	//obs[][] --- S
	//TODO design 7 Freq, better?
	double obs[MAXSAT][4 * MAXFREQ];
	int LLI[MAXSAT][4 * MAXFREQ];       //Loss of Lock Indicator
	int SSI[MAXSAT][4 * MAXFREQ];       //Signal Strength Indicator
	char fob[MAXSAT][4 * MAXFREQ][LEN_OBSTYPE];     //obs code type, eg. 'C1C'
	int lstored[MAXSYS][MAXFREQ];   //bool, whether obs of sys_freq has been stored
} RNXOBS;


typedef struct {
	double ver;

	/* Satellite System:
	 * G: GPS
	 * R: GLONASS
	 * E: Galileo
	 * J: QZSS
	 * C: BDS
	 * I: IRNSS
	 * S: SBAS payload
	 * M: Mixed */
	char sys[4];
	char mark[4];
	char rectype[LEN_ANTENNA];
	char anttype[LEN_ANTENNA];
	char recnum[LEN_ANTENNA];
	char antnum[LEN_ANTENNA];
	double x, y, z, h, e, n;
	int fact1, fact2;
	int nobstype[MAXSYS];       //number of obs
	char obstype[MAXSYS][MAXOBSTYPE][LEN_OBSTYPE];
	double intv;
	int nprn;
	char cprn[MAXSAT][LEN_PRN];
	int t0[6], t1[6];
	int nsys;      //number of systems
	char tsys[8];
	char usetype[MAXSYS][MAXFREQ];      //code use type type for sys and freq; ex. PWCIXSAQLDBYMZN
} RNXHEAD;


//fill the obs line data with 0
void fillobs(char *line, int nobs, int itemlen, double ver);


int read_rnxobs(FILE *fp, RNXHEAD *HD, RNXOBS *OB);

extern void read_rnxnav(char csys, const char *flnbrd, double mjd0, double mjd1, BRDHEAD *hd,
	int *neph, GPS_BRDEPH ephm[MAXSYS][MAXEPH]);

extern void read_rnxobs_head(RNXHEAD *HD, FILE *fp);



void filleph(char *line, double ver);

#endif /* RNX_H_ */