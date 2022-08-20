//
// Created by eureka on 6/17/21.
//
#include <iostream>
#include "resform.h"

using namespace std;

/* print result of GNSS
 * typ = 0  xyz of rover
 *     = 1  denu of base -> rover
 * */
void GnssRes::print(FILE *fp, int typ) {
	if (typ) {       //typ = 1, denu
		Vector3d dxyz = pos_r - pos_b;


		Matrix3d rot_mat = rot_xyz2enu(pos_b);
		Vector3d denu = rot_mat * dxyz;
		Matrix3d Q_enu = rot_mat * Q_pos * rot_mat.transpose();

		Vector3d v_enu = rot_mat * vel_r;

		fprintf(fp, "%8.3f, %10.4f, %10.4f, %10.4f, %d, %d,  %5.1f, %10.4f, %10.4f, %10.4f,"
			"%10.4f, %10.4f, %10.4f \n",
			sow, denu[0], denu[1], denu[2], info, ns, ratio,
			sqrt(Q_enu(0, 0)), sqrt(Q_enu(1, 1)), sqrt(Q_enu(2, 2)),
			v_enu[0], v_enu[1], v_enu[2]);
	}
	else {     //typ = 0, xyz
		fprintf(fp, "%8.3f, %10.4f, %10.4f, %10.4f, %d, %d,  %5.1f, %10.4f, %10.4f, %10.4f,"
			"%10.4f, %10.4f, %10.4f"
			"\n",
			sow, pos_r[0], pos_r[1], pos_r[2], info, ns, ratio,
			sqrt(Q_pos(0, 0)), sqrt(Q_pos(1, 1)), sqrt(Q_pos(2, 2)),
			vel_r[0], vel_r[1], vel_r[2]);
	}

}


GnssRes::GnssRes(int _week, double _sow) {
	week = _week;
	sow = _sow;
	ns = 0;
	info = -1;
	clk_r = 0.0;
	ratio = 0.0;
}