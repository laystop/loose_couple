//
// Created by eureka on 6/6/21.
//

#ifndef MYRTK_OPTIMUM_H
#define MYRTK_OPTIMUM_H

#include <Eigen/Core>
#include <Eigen/Dense>

using namespace Eigen;

void lsq(const Ref<const MatrixXd> H, const Ref<const VectorXd> z,
	Ref<VectorXd> x, Ref<MatrixXd> Q);

void lsq(const Ref<const MatrixXd> H, const Ref<const VectorXd> z,
	Ref<VectorXd> x, Ref<MatrixXd> Q, const Ref<const MatrixXd> D);

void kf(const Ref<const MatrixXd> A, const Ref<const MatrixXd> Dw,
	const Ref<const MatrixXd> H, const Ref<const VectorXd> Z, const Ref<const MatrixXd> Dd,
	Ref<VectorXd> X, Ref<MatrixXd> DX);

#endif //MYRTK_OPTIMUM_H
