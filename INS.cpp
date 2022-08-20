
#include "INS.h"
#include "global.h"
#include <Eigen/Dense>
#include <cmath>
#include <iostream>


using namespace Eigen;
using namespace std;


void read_imu_d(FILE *fp_imu, IMU_d &imu_data, double &week, double &sow) {
	char line[150];
	if (fgets(line, 150, fp_imu)) {
		sscanf(line, "%lf %lf %lf %lf %lf %lf %lf %lf",
			&week, &sow,
			&imu_data[0], &imu_data[1], &imu_data[2],
			&imu_data[3], &imu_data[4], &imu_data[5]);
		imu_data.block<3, 1>(0, 0) *= CPT_LSB_ACCEL/** CPT_SAMP_RATE*/;
		imu_data.block<3, 1>(3, 0) *= CPT_LSB_GYRO/** CPT_SAMP_RATE*/;
	}
	else {
		printf("INS.cpp:24 read IMU end!\n");
	}


}


/*
CoarseAlignment

目的：
	静态粗对准

参数：
	att		3*1向量，输出姿态角，横滚角、俯仰角、航向角(rad)。
	obs		6*1向量，输入观测值，比力x/y/z(m/s),陀螺输出x/y/z(rad)，均为增量
	pos		3*1向量，纬经高，由RTK提供，rad/rad/m
返回值：
	无

*/
void CoarseAlignment(Vector3d &att, IMU_d obs, Vector3d  pos)
{
	double B = pos[0];
	double g = get_g(B, pos[2]);
	Vector3d gamma_n(0, 0, g);
	Vector3d omega_n(ELL.omega_e*cos(B), 0, -ELL.omega_e*sin(B));
	Vector3d vn = gamma_n.cross(omega_n);

	Vector3d gamma_b(-obs[0], -obs[1], -obs[2]);
	Vector3d omega_b(obs[3], obs[4], obs[5]);
	gamma_b *= CPT_SAMP_RATE;
	omega_b *= CPT_SAMP_RATE;
	Vector3d vb = gamma_b.cross(omega_b);

	Matrix3d cn, cb;
	cn << gamma_n, omega_n, vn;
	cb << gamma_b, omega_b, vb;

	Matrix3d cnb = cb * cn.inverse();
	Matrix3d cbn = cnb.transpose();

	////一定要记得正交化！！！
	orthonormalize(cbn);

	Vector3d att1 = cbn.eulerAngles(2, 1, 0);
	att[0] = att1[2];
	att[1] = att1[1];
	att[2] = att1[0];
	update_att(att);
}


/*
attitude_update

目的：
	姿态更新

参数：
	par			9*1,要估计的参数
	obs			6*1向量，输入观测值，比力x/y/z(m/s),陀螺输出x/y/z(rad)，均为增量
	obs_before	上一时刻的观测值
返回值：
	无

*/
void attitude_update(INS_par &par, IMU_d obs, IMU_d obs_before, Matrix3d & Cbn)
{
	//上一时刻的姿态四元数
	Matrix3d cbn_before; Quaterniond q_bn_before;
	cbn_before = AngleAxisd(par[8], Vector3d::UnitZ())
		*AngleAxisd(par[7], Vector3d::UnitY())
		*AngleAxisd(par[6], Vector3d::UnitX());
	q_bn_before = cbn_before;

	//上一时刻和这一时刻的角增量输出
	Vector3d theta_before(obs_before[3], obs_before[4], obs_before[5]),
		theta_now(obs[3], obs[4], obs[5]);

	//等效旋转矢量法更新b系
	Vector3d phik = theta_now + 1.0 / 12.0*theta_before.cross(theta_now);
	//一定要记得单位化！
	Matrix3d cbb = AngleAxisd(phik.norm(), phik.normalized()).toRotationMatrix();
	Quaterniond qbb;
	qbb = cbb;

	//等效旋转法更新n系
	Vector3d omega_ie_n, omega_en_n;
	get_omega_ie_n(omega_ie_n, par[0]);
	get_omega_en_n(omega_en_n, par[0], par[2], par[3], par[4]);
	Vector3d zeta = (omega_en_n + omega_ie_n) / CPT_SAMP_RATE;
	Matrix3d cnn = AngleAxisd(zeta.norm(), zeta.normalized()).toRotationMatrix();
	Quaterniond qnn;
	qnn = cnn;

	//计算当前时刻的姿态四元数
	Quaterniond qbn = qnn * q_bn_before*qbb;
	qbn = qbn.normalized();

	//更新姿态角
	Cbn = qbn.toRotationMatrix();
	Vector3d temp = Cbn.eulerAngles(2, 1, 0);
	update_att(temp);
	par[8] = temp[0];
	par[7] = temp[1];
	par[6] = temp[2];
}


/*
velocity_update

目的：
	速度更新

参数：
	par			9*1,要估计的参数
	obs			6*1向量，输入观测值，比力x/y/z(m/s),陀螺输出x/y/z(rad)，均为增量
	obs_before	上一时刻的观测值
返回值：
	无

*/
void velocity_update(INS_par &par, IMU_d obs, IMU_d obs_before)
{	//重力
	Vector3d g(0, 0, get_g(par[0], par[2]));

	//计算vg
	Vector3d v_before(par[3], par[4], par[5]);
	Vector3d omega_ie_n, omega_en_n;
	get_omega_ie_n(omega_ie_n, par[0]);
	get_omega_en_n(omega_en_n, par[0], par[2], par[3], par[4]);
	Vector3d temp = 2 * omega_ie_n + omega_en_n;
	Vector3d vg = (g - temp.cross(v_before)) / CPT_SAMP_RATE;

	//计算vf
	Vector3d zeta = (omega_en_n + omega_ie_n) / CPT_SAMP_RATE;

	Vector3d dv_before(obs_before[0], obs_before[1], obs_before[2]),
		dv_now(obs[0], obs[1], obs[2]);
	Vector3d theta_before(obs_before[3], obs_before[4], obs_before[5]),
		theta_now(obs[3], obs[4], obs[5]);
	Vector3d dvf_b = dv_now + 0.5*theta_now.cross(dv_now)
		+ 1.0 / 12.0*(theta_before.cross(dv_now) + dv_before.cross(theta_now));

	//姿态矩阵
	Matrix3d cbn;
	cbn = AngleAxisd(par[8], Vector3d::UnitZ())
		*AngleAxisd(par[7], Vector3d::UnitY())
		*AngleAxisd(par[6], Vector3d::UnitX());
	Matrix3d zeta_anti;
	get_anti_mat(zeta_anti, zeta);
	Vector3d dvf = (Matrix3d::Identity() - 0.5*zeta_anti)*cbn*dvf_b;

	//更新
	for (int i = 0; i < 3; ++i)
	{
		par[3 + i] = par[3 + i] + dvf[i] + vg[i];
	}
}


/*
position_update

目的：
	位置更新

参数：
	par			9*1,要估计的参数
	par_befor	上一时刻估计的参数
返回值：
	无

*/
void position_update(INS_par &par, INS_par &par_before)
{
	//更新高程
	par[2] = par_before[2] - 0.5*(par[5] + par_before[5]) / CPT_SAMP_RATE;

	//更新纬度
	double h_hat = 0.5*(par[2] + par_before[2]);
	par[0] = par_before[0] + 0.5*(par[3] + par_before[3]) / (get_RM(par_before[0]) + h_hat) / CPT_SAMP_RATE;

	//更新经度
	double phi_hat = 0.5*(par[0] + par_before[0]);
	par[1] = par_before[1] + 0.5*(par[4] + par_before[4]) / ((get_RN(phi_hat) + h_hat)*cos(phi_hat)) / CPT_SAMP_RATE;
}


///*
//ins
//
//目的：
//	实现惯导机械编排
//
//参数：
//	par			9*1,要估计的参数
//	par_befor	上一时刻估计的参数
//返回值：
//	无
//
//*/
//void ins()
//{
//
//}

