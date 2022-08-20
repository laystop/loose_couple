//
// Created by eureka on 4/25/21.
//
#include <math.h>
#include <string.h>
#include <iostream>

#include "resform.h"
#include "global.h"
#include "common.h"
#include "preprocess.h"
#include "spp.h"
#include "optimum.h"

using namespace std;
using namespace Eigen;

void spp_func_model(const Station &sit, const GnssDataArr gnss_d,
	Ref<MatrixXd> H, Ref<VectorXd> z) {
	double IF_alpha = FREQ1 * FREQ1 / (FREQ1 * FREQ1 - FREQ2 * FREQ2);
	double IF_beta = FREQ2 * FREQ2 / (FREQ1 * FREQ1 - FREQ2 * FREQ2);

	int isys = 0;
	int nsat_sys = gnss_d[isys].size();
	for (int isat = 0; isat < nsat_sys; ++isat) {
		Vector3d d_pos = sit.pos - gnss_d[isys][isat].sat.pos;
		double rho_0 = d_pos.norm();
		d_pos /= rho_0;

		H.block<1, 3>(isat, 0) = d_pos.transpose();

		H(isat, 3) = 1.0;

		double obs = IF_alpha * gnss_d[isys][isat].phpr[2] - IF_beta * gnss_d[isys][isat].phpr[3];
		//        double obs = gnss_d[isys][isat].phpr[2];
		double corr = gnss_d[isys][isat].sat_clk - gnss_d[isys][isat].earth_rot
			- gnss_d[isys][isat].relative - gnss_d[isys][isat].trp;
		obs += corr;

		z[isat] = obs - rho_0 - sit.rec_clk;
	}
}


void spp(Station &sit, const GnssDataArr gnss_d, GnssRes &gnss_res) {
	int isys = 0;
	int nsat_sys = gnss_d[isys].size();
	if (nsat_sys < 4) {
		//        printf("Warning: number of satllites is less than 4  TIME: %d %f\n", week, sow);
		gnss_res.info = -1;
		return;
	}

	int m = nsat_sys;
	int n = 4;

	MatrixXd H = MatrixXd::Zero(m, n);
	VectorXd z(nsat_sys);
	VectorXd x_lsq(4);
	MatrixXd Q(4, 4);
	Vector3d dpos(999.0, 999.0, 999.0);

	for (int iter = 0; iter < 20; ++iter) {
		spp_func_model(sit, gnss_d, H, z);
		lsq(H, z, x_lsq, Q);

		dpos = x_lsq.block<3, 1>(0, 0);
		sit.pos += dpos;
		sit.rec_clk += x_lsq[3];
		if (dpos.norm() < 1.0e-5)
			break;
	}

	//    write res
	gnss_res.ns = nsat_sys;
	gnss_res.pos_r = sit.pos;
	gnss_res.clk_r = x_lsq[3];
	gnss_res.Q_pos = Q.block<3, 3>(0, 0);
	gnss_res.info = INFO_SPP;
}


void spv_func_model(const Station &sit, const GnssDataArr gnss_d,
	Ref<MatrixXd> H, Ref<VectorXd> z) {
	int isys = 0;
	int nsat_sys = gnss_d[isys].size();
	for (int isat = 0; isat < nsat_sys; ++isat) {
		Vector3d d_pos = sit.pos - gnss_d[isys][isat].sat.pos;
		double rho_0 = d_pos.norm();
		//        Vector3d d_vel = sit.vel - gnss_d[isys][isat].sat.vel;
		Vector3d d_vel = -gnss_d[isys][isat].sat.vel;
		double rho_dot = -d_pos.dot(d_vel) / rho_0; //magic -!!!!!
		d_pos /= rho_0;

		H.block<1, 3>(isat, 0) = d_pos.transpose();
		H(isat, 3) = 1.0;

		//        double obs = IF_alpha * gnss_d[isys][isat].phpr[2] - IF_beta * gnss_d[isys][isat].phpr[3];
		double obs = gnss_d[isys][isat].doppler[0];
		double corr = -gnss_d[isys][isat].sat_clk_dot;
		obs += corr;

		z[isat] = obs - rho_dot;
	}
}


/*
 * notes: use gps L1
 * */
void spv(Station &sit, const GnssDataArr gnss_d, GnssRes &gnss_res) {
	int isys = 0;
	int nsat_sys = gnss_d[isys].size();
	if (nsat_sys < 4) {
		//        printf("Warning: number of satllites is less than 4  TIME: %d %f\n", week, sow);
		gnss_res.info = -1;
		return;
	}
	int m = nsat_sys;
	int n = 4;

	MatrixXd H = MatrixXd::Zero(m, n);
	VectorXd z(nsat_sys);
	VectorXd x_lsq(4);
	MatrixXd Q(4, 4);

	spv_func_model(sit, gnss_d, H, z);

	lsq(H, z, x_lsq, Q);
	//    cout<<z<<endl;
	sit.vel = x_lsq.block<3, 1>(0, 0);
	gnss_res.vel_r = sit.vel;
}
