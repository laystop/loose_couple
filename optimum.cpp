//
// Created by eureka on 6/6/21.
//
#include <iostream>
#include "optimum.h"

using namespace std;


/*  Least Square --------------------------------------------------------------
 *  z = H*x + delta, Var(delta) = D
 *  --------------------------------------------------------------------------*/
void lsq(const Ref<const MatrixXd> H, const Ref<const VectorXd> z,
	Ref<VectorXd> x, Ref<MatrixXd> Q) {
	MatrixXd N = H.transpose() * H;
	VectorXd W = H.transpose() * z;

	int m = z.rows();
	int n = x.rows();
	VectorXd V = H * x - z;
	double sigma2 = V.transpose() * V;
	sigma2 /= (m - n);

	Q = N.inverse();
	x = Q * W;

	Q *= sigma2;
}


void lsq(const Ref<const MatrixXd> H, const Ref<const VectorXd> z,
	Ref<VectorXd> x, Ref<MatrixXd> Q, const Ref<const MatrixXd> D) {
	MatrixXd P = D.inverse();
	MatrixXd N = H.transpose() * P * H;
	VectorXd W = H.transpose() * P * z;

	int m = z.rows();
	int n = x.rows();
	VectorXd V = H * x - z;
	double sigma2 = V.transpose() * P * V;
	sigma2 /= (m - n);

	Q = N.inverse();
	x = Q * W;

	Q *= sigma2;
}


/*  Kalman Filter --------------------------------------------------------------
 *  X = A * X_ + w_, Var(w_) = Dw
 *  Z = H * X + delta, Var(delta) = Dd
 *  --------------------------------------------------------------------------*/
void kf(const Ref<const MatrixXd> A, const Ref<const MatrixXd> Dw,
	const Ref<const MatrixXd> H, const Ref<const VectorXd> Z, const Ref<const MatrixXd> Dd,
	Ref<VectorXd> X, Ref<MatrixXd> DX) {
	int m = Z.rows();
	int n = X.rows();
	MatrixXd I = MatrixXd::Identity(n, n);

	//forest
	X = A * X;
	DX = A * DX * A.transpose() + Dw;

	//update
	MatrixXd K = DX * H.transpose() * (H * DX * H.transpose() + Dd).inverse();
	VectorXd V = Z - H * X;
	X += K * V;
	MatrixXd IKH = I - K * H;
	DX = IKH * DX * IKH.transpose() + K * Dd * K.transpose();
}
