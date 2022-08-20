//
// Created by eureka on 6/6/21.
//

#ifndef MYRTK_PREPROCESS_H
#define MYRTK_PREPROCESS_H

#include <vector>
#include "common.h"

#define MAXOBSSAT  30

using namespace std;
using namespace Eigen;

typedef Vector4d phpr_t;

class Gnss_epo_sat {
	//GNSS observation data at one epoch of a satellite
public:
	phpr_t phpr;      //phase, pseudo range, m
	double doppler[MAXFREQ];
	double ssi[MAXFREQ];   //SSI
	double azel[2];      //azimuth and elev of satellite
	Satellite sat;

	double sat_clk;
	double sat_clk_dot;
	double earth_rot;
	double relative;
	double ion;
	double trp;

	Gnss_epo_sat(const RNXOBS &OB, int isat, const Station &sit,
		const GPS_BRDEPH *sel, int week, double sow_rec);
};


typedef vector<Gnss_epo_sat> Gnss_epo_sys;
typedef Gnss_epo_sys GnssDataArr[MAXSYS];

void sel_brd_eph(char *prn, double sow, GPS_BRDEPH **sel);

void get_sit_data(const RNXOBS &OB, const Station &sit, double ele_mask, GnssDataArr gnss_d);

void sel_cmn_sat(GnssDataArr gnss_base, GnssDataArr gnss_rove);

#endif //MYRTK_PREPROCESS_H
