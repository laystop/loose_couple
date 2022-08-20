//
// Created by eureka on 4/25/21.
//

#ifndef MYRTK_ORBIT_H
#define MYRTK_ORBIT_H

#include "common.h"
#include "rnx.h"

typedef Matrix<double, 3, 2> rot_2dto3d;

double Calculate_E(double E_initial, double M, double e, double tol, int iter);

rot_2dto3d rot_plane2space(double Omega, double i);

void brd2pos(const GPS_BRDEPH *gps_brd, Satellite &sat, double t);

void brd2pos_corr_iter(const GPS_BRDEPH *gps_brd, Satellite &sat, const Station &groundRef, double t_rec);

void brd2vel(const GPS_BRDEPH *gps_brd, Satellite &sat, double t);
#endif //MYRTK_ORBIT_H