
#ifndef MYRTK_CONFIG_H
#define MYRTK_CONFIG_H

#include <stdlib.h>

#define MIN(a, b)   ((a>=b) ? b : a)
#define MAX(a, b)   ((a>=b) ? a : b)
#define ABS(a)     ((a>=0) ? a : -(a))
#define SIGN(a, b)  ((b>=0) ? a:-(a))
#define ROUNDUP(a)    ((int)(a+SIGN(1,a)*0.5))   //round away 0

#define true        1
#define false        0
#define MAXSIT        150
#define MAXSYS        7
#define LEN_PRN        4
#define MAXFREQ       3                   /* number of carrier frequencies */
#define NEXOBS      0                   /* number of extended obs codes */
#define MAXOBSTYPE   50
#define LEN_STRING    1024
#define LEN_OBSTYPE 4
#define MAXSAT        200
#define MAXEPO    (2 * 86400)
#define MAXCOL     1024     //max rinex cols
#define LEN_ANTENNA        21
#define MAXWND 0.01
#define LEN_FREQ 4
#define MAXEPH     (MAXSAT * 24)
#define LEN_EPHION        4


typedef struct {
	int cpu_intel;
	char cobs[20];
	// the GNSS system L1 OR L2 nfreq is the count
	int nsys;
	//char system[MAXSYS];
	int nfreq[MAXSYS];

	char freq[MAXSYS][MAXFREQ][LEN_FREQ];   //eg. sys_freq_1 -> 'L1'

	double ffreq[MAXSYS][MAXFREQ];  //float freq

	// time of epoch and interval
	int mjd;
	double sod;

	// stations and satellites
	int nprn;
	char cprn[MAXSAT][LEN_PRN]; //the cprn in session.obj
} CKDCFG;



void rtk_init(const char *pfilcfg, char *pfilbrdm,
	char *pfilobs_b, char *pfilobs_r, char *pfilres);




#endif //MYRTK_CONFIG_H
