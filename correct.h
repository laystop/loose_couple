//
// Created by eureka on 4/25/21.
//

#ifndef MYRTK_CORRECT_H
#define MYRTK_CORRECT_H

#include "common.h"


double CorrEarthRot(const Satellite &sat, const Station &sit);

double CorrRelative(const Satellite &sat);

double CorrSatClk(const GPS_BRDEPH *gps_brd, double t0);

double CorrSatClk_dot(const GPS_BRDEPH *gps_brd, double t0);

double ionmodel(gtime_t t, const double *ion, const double *blh,
	const double *azel);

double tropmodel(gtime_t time, const double *blh, const double *azel, double humi);

#endif //MYRTK_CORRECT_H