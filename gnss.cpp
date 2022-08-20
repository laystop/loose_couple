//
// Created by eureka on 6/22/21.
//
#include "gnss.h"
#include <iostream>

void gnss_pv(vector<GnssRes> &gnss_res_vec) {
	RNXHEAD HD_b = { 0 };
	RNXOBS OB_b = { 0 };
	RNXHEAD HD_r = { 0 };
	RNXOBS OB_r = { 0 };
	BRDHEAD brdhd;

	char pfilobs_b[150], pfilobs_r[150], pfilbrdm[150], pfilres[150];
	const char *pfilcfg = "../config/gnss.cfg";
	rtk_init(pfilcfg, pfilbrdm, pfilobs_b, pfilobs_r, pfilres);

	read_rnxnav('M', pfilbrdm, CKF.mjd, CKF.mjd + 1, &brdhd, neph, ephgps);

	FILE *fpobs_b = fopen(pfilobs_b, "r");
	if (!pfilobs_b) {
		printf("can't open file to read!\n");
		exit(1);
	}
	FILE *fpobs_r = fopen(pfilobs_r, "r");
	if (!pfilobs_r) {
		printf("can't open file to read!\n");
		exit(1);
	}

	read_rnxobs_head(&HD_b, fpobs_b);
	read_rnxobs_head(&HD_r, fpobs_r);

	int week;
	double sow;
	double ds;

	FILE *fpres = fopen(pfilres, "w");
	int is_aligned = 1;

	Station sit_b(-2.221062109726279e+06, 4.994342239569183e+06, 3.276007078055995e+06);
	Station sit_r(-2267810.910, 5009435.734, 3220956.013);

	int num_epo = 0;
	while (!feof(fpobs_b) && (!feof(fpobs_r))) {
		if (is_aligned) {
			read_rnxobs(fpobs_b, &HD_b, &OB_b);
			read_rnxobs(fpobs_r, &HD_r, &OB_r);
		}

		ds = timdif(OB_b.jd, OB_b.tsec, OB_r.jd, OB_r.tsec);
		if (ds < -MAXWND) {
			is_aligned = false;
			read_rnxobs(fpobs_b, &HD_b, &OB_b);
		}
		else if (ds > MAXWND) {
			is_aligned = false;
			read_rnxobs(fpobs_r, &HD_r, &OB_r);
		}
		else {

			is_aligned = true;
			mjd2wksow(OB_b.jd, OB_b.tsec, &week, &sow);

			GnssDataArr gnss_base, gnss_rove;
			get_sit_data(OB_b, sit_b, 15, gnss_base);
			get_sit_data(OB_r, sit_r, 15, gnss_rove);

			GnssRes gnss_res(week, sow);
			if (num_epo < INIT_NUM_EPO) {
				gnss_res_vec[num_epo].week = week;
				gnss_res_vec[num_epo].sow = sow;
			}
			else {
				gnss_res_vec.emplace_back(week, sow);
			}

			//            spp(sit_b, gnss_base, gnss_res_vec[num_epo]);
			spp(sit_r, gnss_rove, gnss_res_vec[num_epo]);
			spv(sit_r, gnss_rove, gnss_res_vec[num_epo]);
			if (gnss_res_vec[num_epo].info == -1) {
				continue;
			}

			sel_cmn_sat(gnss_base, gnss_rove);
			rtk_pos(sit_b, sit_r, gnss_base, gnss_rove, gnss_res_vec[num_epo]);
			if (gnss_res_vec[num_epo].info == -1) {
				continue;
			}

			gnss_res_vec[num_epo].pos_b = sit_b.pos;
			gnss_res_vec[num_epo].print(fpres, 0);
			num_epo++;
		}
	}

	fclose(fpres);
	fclose(fpobs_b);
	fclose(fpobs_r);
	if (num_epo < INIT_NUM_EPO) {
		gnss_res_vec.resize(num_epo);
	}

}