//
// Created by eureka on 4/25/21.
//
#include <math.h>
#include "global.h"
#include "common.h"
#include "correct.h"


/* Correct Earth Rotate error --------------------------------------------------
 * ---------------------------------------------------------------------------*/
double CorrEarthRot(const Satellite &sat, const Station &sit) {
	double OMEGA_E = OMGE_GPS;
	double dX = sit.pos[0] - sat.pos[0];
	double dY = sit.pos[1] - sat.pos[1];
	double d = sat.pos[1] * dX - sat.pos[0] * dY;
	double delta_rho = -OMEGA_E / CLIGHT * d;
	return delta_rho;
}


/* Correct Relative error ------------------------------------------------------
 * ---------------------------------------------------------------------------*/
double CorrRelative(const Satellite &sat) {
	double d = sat.pos.dot(sat.vel);
	double delta_rho = 2.0 / CLIGHT * d;
	return delta_rho;
}


/* Correct Satellite Clock error -----------------------------------------------
 * ---------------------------------------------------------------------------*/
double CorrSatClk(const GPS_BRDEPH *gps_brd, double t0) {
	double dt = t0 - gps_brd->toe;
	return (gps_brd->a0 + gps_brd->a1 * dt + gps_brd->a2 * dt * dt) * CLIGHT;
}


/* Correct Satellite Clock error -----------------------------------------------
 * ---------------------------------------------------------------------------*/
double CorrSatClk_dot(const GPS_BRDEPH *gps_brd, double t0) {
	double dt = t0 - gps_brd->toe;
	return (gps_brd->a1 + 2 * gps_brd->a2 * dt) * CLIGHT;
}


/* ionosphere model ------------------------------------------------------------
* compute ionospheric delay by broadcast ionosphere model (klobuchar model)
* args   : gtime_t t        I   time (gpst)
*          double *ion      I   iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3}
*          double *blh      I   receiver position {lat,lon,h} (rad,rad,m)
*          double *azel     I   azimuth/elevation angle {az,el} (rad)
* return : ionospheric delay (L1) (m)
*-----------------------------------------------------------------------------*/
double ionmodel(gtime_t t, const double *ion, const double *blh, const double *azel) {
	const double ion_default[] = { /* 2004/1/1 */
			0.1118E-07, -0.7451E-08, -0.5961E-07, 0.1192E-06,
			0.1167E+06, -0.2294E+06, -0.1311E+06, 0.1049E+07
	};
	double tt, f, psi, phi, lam, amp, per, x;
	int week;

	if (blh[2] < -1E3 || azel[1] <= 0) return 0.0;
	//    if (norm(ion, 8) <= 0.0) ion = ion_default;

		/* earth centered angle (semi-circle) */
	psi = 0.0137 / (azel[1] / PI + 0.11) - 0.022;

	/* subionospheric latitude/longitude (semi-circle) */
	phi = blh[0] / PI + psi * cos(azel[0]);
	if (phi > 0.416) phi = 0.416;
	else if (phi < -0.416) phi = -0.416;
	lam = blh[1] / PI + psi * sin(azel[0]) / cos(phi * PI);

	/* geomagnetic latitude (semi-circle) */
	phi += 0.064 * cos((lam - 1.617) * PI);

	/* local time (s) */
	double a = time2gpst(t, &week);
	tt = 43200.0 * lam + time2gpst(t, &week);
	tt -= floor(tt / 86400.0) * 86400.0; /* 0<=tt<86400 */

	/* slant factor */
	f = 1.0 + 16.0 * pow(0.53 - azel[1] / PI, 3.0);

	/* ionospheric delay */
	amp = ion[0] + phi * (ion[1] + phi * (ion[2] + phi * ion[3]));
	per = ion[4] + phi * (ion[5] + phi * (ion[6] + phi * ion[7]));
	amp = amp < 0.0 ? 0.0 : amp;
	per = per < 72000.0 ? 72000.0 : per;
	x = 2.0 * PI * (tt - 50400.0) / per;

	return CLIGHT * f * (fabs(x) < 1.57 ? 5E-9 + amp * (1.0 + x * x * (-0.5 + x * x / 24.0)) : 5E-9);
}


/* troposphere model -----------------------------------------------------------
* compute tropospheric delay by standard atmosphere and saastamoinen model
* args   : gtime_t time     I   time
*          double *blh      I   receiver position {lat,lon,h} (rad,m)
*          double el        I   elevation angle (rad)
*          double humi      I   relative humidity
* return : tropospheric delay (m)
*-----------------------------------------------------------------------------*/
double tropmodel(gtime_t time, const double *blh, const double *azel, double humi) {
	const double temp0 = 15.0; /* temparature at sea level */
	double hgt, pres, temp, e, z, trph, trpw;

	if (blh[2] < -100.0 || 1E4 < blh[2] || azel[1] <= 0) return 0.0;

	/* standard atmosphere */
	hgt = blh[2] < 0.0 ? 0.0 : blh[2];

	pres = 1013.25 * pow(1.0 - 2.2557E-5 * hgt, 5.2568);
	temp = temp0 - 6.5E-3 * hgt + 273.16;
	e = 6.108 * humi * exp((17.15 * temp - 4684.0) / (temp - 38.45));

	/* saastamoninen model */
	z = PI / 2.0 - azel[1];
	trph = 0.0022768 * pres / (1.0 - 0.00266 * cos(2.0 * blh[0]) - 0.00028 * hgt / 1E3) / cos(z);
	trpw = 0.002277 * (1255.0 / temp + 0.05) * e / cos(z);
	return trph + trpw;
}
