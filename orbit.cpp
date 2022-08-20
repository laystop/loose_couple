//
// Created by eureka on 4/25/21.
//
#include <math.h>
#include <string.h>
#include "global.h"
#include "orbit.h"

/* Calculate E by iterating with Newton's Method -------------------------------
 * args   : double E_initial  I
 * return : E ----------------------------------------------------------------*/
double Calculate_E(double E_initial, double M, double e, double tol, int iter) {
	double E = E_initial;
	int i;
	double err;
	for (i = 0; i < iter; i++) {
		err = (E - e * sin(E) - M) / (1 - e * cos(E));
		if (fabs(err) > tol)
			E -= err;
		else
			break;
	}
	return E;
}


/* calculate rot matrix from satellite plane to space ------------------------*/
rot_2dto3d rot_plane2space(double Omega, double i) {
	double cos_o = cos(Omega);
	double sin_o = sin(Omega);
	double cos_i = cos(i);
	double sin_i = sin(i);

	rot_2dto3d rot_mat;
	rot_mat << cos_o, -sin_o * cos_i,
		sin_o, cos_i * cos_o,
		0.0, sin_i;

	return rot_mat;
}


Matrix3d rot_mat_by_x(double phi) {
	Matrix3d rot_mat;
	double c = cos(phi);
	double s = sin(phi);
	rot_mat << c, s, 0.0,
		-s, c, 0.0,
		0.0, 0.0, 1.0;
	return rot_mat;
}


Matrix3d rot_mat_by_z(double phi) {
	Matrix3d rot_mat;
	double c = cos(phi);
	double s = sin(phi);
	rot_mat << 1.0, 0.0, 0.0,
		0.0, c, s,
		0.0, -s, c;
	return rot_mat;
}


/*卫星轨道参数改正及计算*/
void brd2pos(const GPS_BRDEPH *gps_brd, Satellite &sat, double t) {
	//0. switch parameters
	double GM, OMEGA_E;
	if (gps_brd->cprn[0] == 'G') {
		GM = MU_GPS;
		OMEGA_E = OMGE_GPS;
	}
	else {
		GM = MU_CMP;
		OMEGA_E = OMGE_CMP;
	}

	//1. 计算卫星运动的平均角速度n
	double n = sqrt(GM) / pow(gps_brd->roota, 3) + gps_brd->dn;

	//2. 计算观测瞬间卫星平近点角M
	double dt = t - gps_brd->toe;
	double M = gps_brd->m0 + n * dt;

	//3. 计算偏近点角E
	double E = Calculate_E(M, M, gps_brd->e, 1e-20, 10); //初始值为M

	//4. 计算真近点角f
	double f = atan2((sqrt(1 - gps_brd->e * gps_brd->e) * sin(E)), (cos(E) - gps_brd->e));

	//5. 升交角距u_prime
	double u_prime = gps_brd->omega + f;

	//6. 计算摄动改正项
	double cos_2u_prime = cos(2 * u_prime);
	double sin_2u_prime = sin(2 * u_prime);
	double delta_u = gps_brd->cuc * cos_2u_prime + gps_brd->cus * sin_2u_prime;
	double delta_r = gps_brd->crc * cos_2u_prime + gps_brd->crs * sin_2u_prime;
	double delta_i = gps_brd->cic * cos_2u_prime + gps_brd->cis * sin_2u_prime;

	//7. 对u_prime, r_prime, i_0进行摄动改正
	double u = u_prime + delta_u;
	double r = gps_brd->roota * gps_brd->roota * (1 - gps_brd->e * cos(E)) + delta_r;
	double i = gps_brd->i0 + delta_i + gps_brd->i_dot * dt;
	double cos_i = cos(i);

	/* 卫星轨道面坐标计算 */
	double x = r * cos(u);
	double y = r * sin(u);

	//8. 计算观测瞬间升交点经度L, in ECEF
	double L = gps_brd->Omega0 + gps_brd->Omega_dot * dt - OMEGA_E * t;

	/* 轨道面坐标 -> WGS84 */
	sat.pos[0] = x * cos(L) - y * sin(L) * cos(i);
	sat.pos[1] = x * sin(L) + y * cos(L) * cos(i);
	sat.pos[2] = y * sin(i);

	//    if(IsGEO(gps_brd->cprn)){
	//        double O = gps_brd->Omega0 + gps_brd->Omega_dot * dt - OMEGA_E * gps_brd->toe;
	//        double sinO = sin(O);
	//        double cosO = cos(O);
	//        double xg = x * cosO - y * cos_i * sinO;
	//        double yg = x * sinO + y * cos_i * cosO;
	//        double zg = y * sin(i);
	//        double sino = sin(OMEGA_E * dt);
	//        double coso = cos(OMEGA_E * dt);
	//        sat.pos[0] = xg * coso + yg * sino * COS_5 + zg * sino * SIN_5;
	//        sat.pos[1] = -xg * sino + yg * coso * COS_5 + zg * coso * SIN_5;
	//        sat.pos[2] = -yg * SIN_5 + zg * COS_5;
	//    }


	if (gps_brd->cprn[0] == 'C') {
		int ind = (int)(gps_brd->cprn[1] - '0') * 10 + (int)(gps_brd->cprn[2] - '0');

		if (ind < 6 || ind >58) {
			double O = gps_brd->Omega0 + gps_brd->Omega_dot * dt - OMEGA_E * gps_brd->toe;
			double sinO = sin(O);
			double cosO = cos(O);
			double xg = x * cosO - y * cos_i * sinO;
			double yg = x * sinO + y * cos_i * cosO;
			double zg = y * sin(i);
			double sino = sin(OMEGA_E * dt);
			double coso = cos(OMEGA_E * dt);
			sat.pos[0] = xg * coso + yg * sino * COS_5 + zg * sino * SIN_5;
			sat.pos[1] = -xg * sino + yg * coso * COS_5 + zg * coso * SIN_5;
			sat.pos[2] = -yg * SIN_5 + zg * COS_5;
		}
	}

	strcpy(sat.prn, gps_brd->cprn);
}


/*
 * t_rec: receiving time on ground, sow
 * considering of signal spreading delay(about 0.07s)
 * */
void brd2pos_corr_iter(const GPS_BRDEPH *gps_brd, Satellite &sat, const Station &groundRef, double t_rec) {
	double t_spread, t_spread_prime, g2s_dist;
	t_spread = 0.07;        //signal spreads for about 0.07s, to get better convergence

	for (int i = 0; i < 5; i++) {
		brd2pos(gps_brd, sat, t_rec - t_spread);
		g2s_dist = (groundRef.pos - sat.pos).norm();
		t_spread_prime = g2s_dist / CLIGHT;
		if (fabs(t_spread_prime - t_spread) < 1e-10)
			break;
		t_spread = t_spread_prime;
	}
}


/* Calculate velocity of satellite from Broadcast Ephemeris --------------------
 * using Three-point Centered-Difference Formula
 * F(h)_2 = df(x) = [f(x+h) - f(x-h)] / (2*h)  + K * h^2
 * using Richardson Extrapolation to get higher Order h
 * Q = [2^n * F(h/2) - F(h)] / (2^n - 1)
 * namely, five-point centered-difference formula
 * F(h)_4 = [f(x-h) - 8*f(x-h/2) + 8*f(x+h/2) - f(x+h)] / (6*h)
 * ---------------------------------------------------------------------------*/
void brd2vel(const GPS_BRDEPH *gps_brd, Satellite &sat, double t) {
	double h = 1e-5;        //V_sat ~= 3.0e3 m/s
	Satellite sat_t1 = {};
	Satellite sat_t2 = {};
	Satellite sat_t3 = {};
	Satellite sat_t4 = {};

	brd2pos(gps_brd, sat_t1, t - h);
	brd2pos(gps_brd, sat_t2, t - h / 2);
	brd2pos(gps_brd, sat_t3, t + h / 2);
	brd2pos(gps_brd, sat_t4, t + h);

	sat.vel = (sat_t1.pos - 8.0 * sat_t2.pos + 8.0 * sat_t3.pos - sat_t4.pos) / (6.0 * h);
}



