//
// Created by eureka on 4/29/21.
//

#ifndef MYRTK_RTK_H
#define MYRTK_RTK_H


#include "orbit.h"

void rtk_func_model(const Station &sit_b, const Station &sit_r,
	GnssDataArr gnss_base, GnssDataArr gnss_rove,
	Ref<MatrixXd> A, Ref<VectorXd> L);

void rtk_rand_model(const GnssDataArr gnss_base, const GnssDataArr gnss_rove,
	Ref<MatrixXd> D, double pr_rate_ph = 100.0);

void ambfix_update(const Ref<const MatrixXd> Q,
	const Ref<const VectorXd> amb_float, const Ref<const VectorXd> amb_fixed,
	Vector3d &baseline, Matrix3d &Qbb);

void rtk_pos(const Station &sit_base, Station &sit_rove,
	GnssDataArr gnss_base, GnssDataArr gnss_rove,
	GnssRes &gnss_res);

#endif //MYRTK_RTK_H
