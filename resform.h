//
// Created by eureka on 6/17/21.
//

#ifndef MYRTK_RESFORM_H
#define MYRTK_RESFORM_H

#include "orbit.h"


class GnssRes {
	//GNSS result
public:
	int week;
	double sow;
	int ns;
	int info;
	Vector3d pos_b;
	Vector3d pos_r;
	Matrix3d Q_pos;

	Vector3d vel_r;
	Matrix3d Q_vel;

	double clk_r;

	double ratio;
	VectorXd amb;
	MatrixXd Q;     //amb and baseline

	GnssRes() {};
	GnssRes(int _week, double _sow);
	void print(FILE *fp, int typ);
};





#endif //MYRTK_RESFORM_H
