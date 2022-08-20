//
// Created by eureka on 6/6/21.
//
#include<vector>

#include "global.h"
#include "common.h"
#include "orbit.h"
#include "correct.h"
#include "preprocess.h"

using namespace std;
using namespace Eigen;


/* select broadcast ephemeris --------------------------------------------------
 * arg  :   char *prn               I   satellite prn
 *          double sow              I
 *          GPS_BRDEPH **sel        O   pointer to brd
 * notes:   sel = NUll if no suitable brd exist
 * ---------------------------------------------------------------------------*/
void sel_brd_eph(const char *prn, double sow, GPS_BRDEPH **sel) {
	double dtmin = 9e9;
	double dt;
	int isys = index_string(SYS, prn[0]);

	//search the right epoch of ephemeris
	for (int epo_eph = 0; epo_eph < neph[isys]; epo_eph++) {    //epoch of ephemeris
		if (strcmp(ephgps[isys][epo_eph].cprn, prn)) continue;

		dt = fabs(sow - ephgps[isys][epo_eph].toe);
		if (dt < dtmin) {       //get the closest ephemeris
			*sel = &ephgps[isys][epo_eph];
			dtmin = dt;
		}
	}
	if (dtmin > 4 * 3600)
		sel = NULL;
}


/* initialize sat and azel -----------------------------------------------------
 * ---------------------------------------------------------------------------*/
Gnss_epo_sat::Gnss_epo_sat(const RNXOBS &OB, int isat, const Station &sit,
	const GPS_BRDEPH *sel, int week, double sow_rec) {
	double sow_gps;
	if (OB.cprn[isat][0] == 'C')
		sow_gps = sow_rec - leap_sec_bds2gps;
	else
		sow_gps = sow_rec;

	sat_clk = CorrSatClk(sel, sow_gps);
	sat_clk_dot = CorrSatClk_dot(sel, sow_gps);
	double t_spread = (OB.obs[isat][0 + MAXFREQ] + sat_clk) / CLIGHT;
	brd2pos(sel, sat, sow_gps - t_spread);
	brd2vel(sel, sat, sow_gps - t_spread);

	//if sit on (0,0,0), there will be error!
	elevazel(sat.pos, sit.pos, azel);

	//correct error
	sat_clk = CorrSatClk(sel, sow_gps - t_spread);
	earth_rot = CorrEarthRot(sat, sit);
	relative = CorrRelative(sat);

	//correct trp
	double blh[3];
	xyz2blh(sit.pos, blh);
	gtime_t gtime = gpst2time(week, sow_gps);
	trp = tropmodel(gtime, blh, azel, 0.5);
}


/* Get Data of sit, using 2 Freq phase and pseudo range-------------------------
 * args  :  RNXOBS      *OB             I       RNX Observation
 *          Station     *sit            I       Referenced sit
 *          Satellite   *sats           O       array of satellites
 *          int         *nsat           O       number of satellites of each system
 *          double      *ph_pr          O       array of available obs, phase and pseudo range
 *          double      *corr           O       array of corrections of each sat
 *          double      *elev           O       array of elevation angle, in consonance with *satsele_mask
 *          double      mask            I       elevation mask angle (deg)
 * return:  int nsat, number of available satellite
 * notes :  ph_pr[0] --- L1 , 1st satellite
 *               [1] --- L2
 *               [3] --- P1
 *               [4] --- P2
 *               [5] --- L1 , 2nd satellite
 *                       ...
 *          namely, ph_pr[0/1/2/3 , isat] -> L1/L2/P1/P2 of <isat>th satellite
 *          sats and ph_pr are in system order, sync with CKF.cprn (initialized in order)
 *          if sit on (0,0,0), there will be error!
 * ---------------------------------------------------------------------------*/
void get_sit_data(const RNXOBS &OB, const Station &sit, double ele_mask, GnssDataArr gnss_d) {
	int week;
	double sow_rec;
	mjd2wksow(OB.jd, OB.tsec, &week, &sow_rec);

	int isys;
	GPS_BRDEPH *sel = NULL;
	double ph_1, ph_2, pr_1, pr_2;

	for (int isat = 0; isat < CKF.nprn; isat++) {
		if (strcmp(CKF.cprn[isat], OB.cprn[isat]) != 0)     //if no isat, OB->cprn[isat] = "000\0"
			continue;

		//get observable
		ph_1 = OB.obs[isat][0];
		ph_2 = OB.obs[isat][1];
		pr_1 = OB.obs[isat][0 + MAXFREQ];
		pr_2 = OB.obs[isat][1 + MAXFREQ];

		//if any of 4 observable lost, skip
		if ((fabs(ph_1) < 1.0e-3) || (fabs(ph_2) < 1.0e-3) ||
			(fabs(pr_1) < 1.0e-3) || (fabs(pr_2) < 1.0e-3))
			continue;


		//select brd eph
		sel = NULL;
		sel_brd_eph(OB.cprn[isat], sow_rec, &sel);
		if (sel == NULL)
			continue;

		isys = index_string(SYS, OB.cprn[isat][0]);

		//if sit on (0,0,0), there will be error!
//        gnss_d[isys].emplace_back(
//                Gnss_epo_sat(OB, isat, sit,
//                             sel, week, sow_rec)
//        );     //add item to gnss_d
		gnss_d[isys].emplace_back(
			OB, isat, sit,
			sel, week, sow_rec
		);

		if (gnss_d[isys].back().azel[1] < ele_mask * DEG2RAD) {
			gnss_d[isys].pop_back();
			continue;
		}

		double lambda_1 = CLIGHT / CKF.ffreq[isys][0];
		double lambda_2 = CLIGHT / CKF.ffreq[isys][1];
		gnss_d[isys].back().phpr[0] = ph_1 * lambda_1;
		gnss_d[isys].back().phpr[1] = ph_2 * lambda_2;
		gnss_d[isys].back().phpr[2] = pr_1;
		gnss_d[isys].back().phpr[3] = pr_2;
		gnss_d[isys].back().doppler[0] = OB.obs[isat][0 + 2 * MAXFREQ] * lambda_1;
		gnss_d[isys].back().doppler[1] = OB.obs[isat][1 + 2 * MAXFREQ] * lambda_2;
		gnss_d[isys].back().ssi[0] = OB.obs[isat][0 + 3 * MAXFREQ];
		gnss_d[isys].back().ssi[1] = OB.obs[isat][1 + 3 * MAXFREQ];
	}
}


/* select common satellite and referenced satellite-----------------------------
 * notes    : sats_cmn_b, sats_cmn_r must be in system-prn order
 * ---------------------------------------------------------------------------*/
void sel_cmn_sat(GnssDataArr gnss_base, GnssDataArr gnss_rove) {
	for (int isys = 0; isys < CKF.nsys; ++isys) {
		int ns_sys_b = gnss_base[isys].size();
		int ns_sys_r = gnss_rove[isys].size();
		int isat_b = 0, isat_r = 0;
		double el_max = 0.0;
		int iref = 0;

		while ((isat_b < ns_sys_b) && (isat_r < ns_sys_r)) {
			int dprn = prncmp(gnss_base[isys][isat_b].sat.prn, gnss_rove[isys][isat_b].sat.prn);
			if (dprn == 0) {
				//satellites are mated
				if ((gnss_base[isys][isat_b].azel[1] > el_max) &&
					(!IsGEO(gnss_base[isys][isat_b].sat.prn))) {
					iref = isat_b;
					el_max = gnss_base[isys][isat_b].azel[1];
				}

				isat_b++;
				isat_r++;
			}
			else if (dprn < 0) {
				//satellite in base won't be mated
				gnss_base[isys].erase(gnss_base[isys].begin() + isat_b);
				ns_sys_b--;
			}
			else {
				gnss_rove[isys].erase(gnss_rove[isys].begin() + isat_r);
				ns_sys_r--;
			}
		}

		if (isat_b == ns_sys_b) {
			for (int i = 0; i < ns_sys_r - ns_sys_b; ++i) {
				gnss_rove[isys].pop_back();
			}
		}
		else {
			for (int i = 0; i < ns_sys_b - ns_sys_r; ++i) {
				gnss_base[isys].pop_back();
			}
		}

		if (iref != 0) {
			swap(gnss_base[isys][0], gnss_base[isys][iref]);
			swap(gnss_rove[isys][0], gnss_rove[isys][iref]);
		}
	}
}
