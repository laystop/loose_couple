//
// Created by eureka on 4/29/21.
//
#include <math.h>
#include <iostream>

#include "global.h"
#include "common.h"
#include "preprocess.h"
#include "spp.h"
#include "optimum.h"
#include "rtk.h"
#include "lambda.h"

using namespace Eigen;

void rtk_func_model(const Station &sit_b, const Station &sit_r,
	GnssDataArr gnss_base, GnssDataArr gnss_rove,
	Ref<MatrixXd> A, Ref<VectorXd> L) {
	const int nsys = CKF.nsys;
	int ind_ns = 0;       //index of sat
	for (int isys = 0; isys < nsys; ++isys) {
		double lambda_1 = CLIGHT / CKF.ffreq[isys][0];
		double lambda_2 = CLIGHT / CKF.ffreq[isys][1];

		int ns_sys = gnss_rove[isys].size() - 1;

		//j, calculate reference related item only once
		Vector3d dpos_bj = sit_b.pos - gnss_base[isys][0].sat.pos;
		Vector3d dpos_rj = sit_r.pos - gnss_rove[isys][0].sat.pos;
		double rho_bj = dpos_bj.norm();
		double rho_rj = dpos_rj.norm();
		Vector3d lmn_j = dpos_rj / rho_bj;      //sat - sit
		double sd_rho_j = rho_rj - rho_bj;      //sd, single dif: sit-sit

		phpr_t sd_obs_j = gnss_rove[isys][0].phpr - gnss_base[isys][0].phpr;

		for (int isat = 1; isat < ns_sys + 1; isat++) {
			Vector3d dpos_bk = sit_b.pos - gnss_base[isys][isat].sat.pos;
			Vector3d dpos_rk = sit_r.pos - gnss_rove[isys][isat].sat.pos;
			double rho_bk = dpos_bk.norm();
			double rho_rk = dpos_rk.norm();
			double sd_rho_k = rho_rk - rho_bk;
			phpr_t sd_obs_k = gnss_rove[isys][isat].phpr - gnss_base[isys][isat].phpr;
			Vector3d lmn_k = dpos_rk / rho_rk;
			Vector3d abc_k = lmn_k - lmn_j;

			double dd_rho = sd_rho_k - sd_rho_j;
			phpr_t dd_obs = sd_obs_k - sd_obs_j;

			//construct A and L
			int ieq = ind_ns * 4 + isat - 1;
			for (int ityp = 0; ityp < 4; ++ityp) {
				L[ieq] = dd_obs[ityp] - dd_rho;
				A.block<1, 3>(ieq, 0) = abc_k.transpose();
				ieq += ns_sys;
			}
		}
		MatrixXd I = MatrixXd::Identity(ns_sys, ns_sys);
		A.block(ind_ns * 4, 3 + 2 * ind_ns, ns_sys, ns_sys) = lambda_1 * I;
		A.block(ind_ns * 4 + ns_sys, 3 + 2 * ind_ns + ns_sys,
			ns_sys, ns_sys) = lambda_2 * I;

		ind_ns += ns_sys;
	}
}


/* RTK Random Model ------------------------------------------------------------
 * ---------------------------------------------------------------------------*/
void rtk_rand_model(const GnssDataArr gnss_base, const GnssDataArr gnss_rove,
	Ref<MatrixXd> D, double pr_rate_ph) {
	const int nsys = CKF.nsys;
	int ind_ns = 0;       //index of sat
	double fac = pr_rate_ph * pr_rate_ph;

	//    auto noise_model = [](double elev) -> double { return 0.005; };
	auto noise_model = [](double elev) -> double { return (16.0 + 9.0 / (pow(sin(elev), 2))) * 1.0e-6; };
	for (int isys = 0; isys < nsys; ++isys) {
		int ns_sys = gnss_rove[isys].size() - 1;

		double sigma2_j = noise_model(gnss_base[isys][0].azel[1])
			+ noise_model(gnss_rove[isys][0].azel[1]);
		MatrixXd one_sig = sigma2_j * MatrixXd::Ones(ns_sys, ns_sys);
		for (int ityp = 0; ityp < 4; ++ityp) {
			int ieq = 4 * ind_ns + ityp * ns_sys;
			D.block(ieq, ieq, ns_sys, ns_sys) = one_sig;
		}

		for (int isat = 1; isat < ns_sys + 1; isat++) {
			double sigma2_i = noise_model(gnss_base[isys][isat].azel[1])
				+ noise_model(gnss_rove[isys][isat].azel[1]);

			for (int ityp = 0; ityp < 4; ++ityp) {
				int ieq = 4 * ind_ns + ityp * ns_sys + isat - 1;
				D(ieq, ieq) += sigma2_i;
			}
		}

		for (int ityp = 0; ityp < 2; ++ityp) {
			int ieq = 4 * ind_ns + ityp * ns_sys;
			D.block(ieq, ieq, ns_sys, ns_sys) /= fac;
		}

		ind_ns += ns_sys;
	}
	//    cout<<D<<endl;
	//    double a=1;
}


/* update baseline solution with fixed amb -------------------------------------
 * args     :   Vector b            IO      baseline
 *              int n               I       number of amb
 *              Matrix Qaa, Qba     I
 *              Vector f            I       float
 *              Matrix z            I       integer solution of amb
 * ---------------------------------------------------------------------------*/
void ambfix_update(const Ref<const MatrixXd> Q,
	const Ref<const VectorXd> amb_float, const Ref<const VectorXd> amb_fixed,
	Vector3d &baseline, Matrix3d &Qbb) {
	int namb = amb_fixed.rows();
	MatrixXd Qaa = Q.block(3, 3, namb, namb);
	MatrixXd Qab = Q.block(3, 0, namb, 3);

	MatrixXd Qba_Qaa_inv = Qab.transpose() * Qaa.inverse();

	baseline -= Qba_Qaa_inv * (amb_float - amb_fixed);
	Qbb -= Qba_Qaa_inv * Qab;
}


/* Real Time Kinematic ----------------------------------------------------------
 * args     :   Station *sit_b                  IO
 *              Station *sit_r                  I
 *              Satellite *sats_b, *sats_r      I
 *              double *ph_pr_b, *ph_pr_r       I
 *              int *nsat_b, *nsat_r            I
 *              int *nsat_cmn                   O
 *              Matrix Q_xyz
 * -----------------------------------------------------------------------------*/
void rtk_pos(const Station &sit_base, Station &sit_rove,
	GnssDataArr gnss_base, GnssDataArr gnss_rove,
	GnssRes &gnss_res) {
	int num_sat = 0;
	for (int isys = 0; isys < CKF.nsys; ++isys)
		num_sat += gnss_rove[isys].size();
	if (num_sat < 4) {
		gnss_res.info = -1;
		return;
	}

	int namb = 2 * (num_sat - CKF.nsys);
	int m = 4 * (num_sat - CKF.nsys);
	int n = 3 + namb;

	//random model
	MatrixXd D = MatrixXd::Zero(m, m);
	rtk_rand_model(gnss_base, gnss_rove, D, 100.0);

	MatrixXd A = MatrixXd::Zero(m, n);
	VectorXd L = VectorXd::Zero(m);

	VectorXd x_lsq(n);
	MatrixXd Q = MatrixXd::Identity(n, n);
	Vector3d dpos_r(999.0, 999.0, 999.0);
	int iter = 0;
	while (dpos_r.norm() > 1e-4 && iter < 10) {
		rtk_func_model(sit_base, sit_rove, gnss_base, gnss_rove, A, L);
		lsq(A, L, x_lsq, Q, D);
		dpos_r = x_lsq.block<3, 1>(0, 0);
		sit_rove.pos += dpos_r;
		iter++;
	}

	VectorXd amb_float = x_lsq.block(3, 0, namb, 1);
	VectorXd amb_fixed(namb);
	Matrix3d Q_pos = Q.block<3, 3>(0, 0);
	MatrixXd Qaa = Q.block(3, 3, namb, namb);

	//    cout<<"A"<<endl<<A<<endl;
	//    cout<<"L"<<endl<<L<<endl;

	double ratio = amb_fix(amb_float, Qaa, amb_fixed);

	if (ratio > 3.0) {
		ambfix_update(Q, amb_float, amb_fixed, sit_rove.pos, Q_pos);
		gnss_res.amb = amb_fixed;
		gnss_res.info = INFO_FIXED;
	}
	else {
		gnss_res.amb = amb_float;
		gnss_res.info = INFO_FLOAT;
	}



	gnss_res.ns = num_sat;
	gnss_res.pos_r = sit_rove.pos;
	gnss_res.ratio = ratio;
	gnss_res.Q_pos = Q_pos;


}

