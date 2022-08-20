#include"LC.h"
#include "common.h"
#include "global.h"
#include"INS.h"
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
using namespace std;

void  buildLCTransferMat(INS_par par_before, IMU_d obs, Matrix3d Cbn, double relevantTime, TransferMat& transfermat)
{
	memset(&transfermat, 0, sizeof(TransferMat));
	//构造误差微分方程中的稀疏矩阵
	TransferMat beforeLinearized = MatrixXd::Zero(15, 15);
	//求解M,N半径 使用的是上一历元的更新后的状态向量，对应误差也是上一历元截止所累积的
	//求解n系的比力测量值
	Vector3d acc_b = obs.block(0, 0, 3, 1)*CPT_SAMP_RATE;

	//ned-> enu
	Vector3d acc_n = Cbn * acc_b;
	Matrix3d acc_n_anti;
	get_anti_mat(acc_n_anti, acc_n);


	beforeLinearized(0, 3) = 1 / (get_RM(par_before[0]) + par_before[2]);
	beforeLinearized(1, 4) = 1 / ((get_RN(par_before[0]) + par_before[2])*cos(par_before[0]));
	beforeLinearized(2, 5) = -1;
	beforeLinearized.block<3, 3>(3, 6) = -acc_n_anti;

	beforeLinearized(6, 4) = 1 / (get_RN(par_before[0]) + par_before[2]);
	beforeLinearized(7, 3) = -1 / (get_RM(par_before[0]) + par_before[2]);
	beforeLinearized(8, 4) = -tan(par_before[0]) / (get_RN(par_before[0]) + par_before[2]);

	// cbn(ned)->cbn(enu)

	beforeLinearized.block(3, 12, 3, 3) = Cbn;
	beforeLinearized.block(6, 9, 3, 3) = Cbn;

	//相关时间设为前后历元时间差
	for (int i = 9; i < 15; i++)
	{
		beforeLinearized(i, i) = -1 / relevantTime;
	}

	transfermat = MatrixXd::Identity(15, 15) + relevantTime * beforeLinearized;

	//cout << beforeLinearized << endl<<endl;
	//cout << transfermat << endl;
}

void buildDesignMat(DesignMat& designMat)
{
	memset(&designMat, 0, sizeof(DesignMat));
	int i;
	for (i = 0; i < 6; i++)
	{
		designMat(i, i) = 1;
	}
}


void buildProcessNoiseMat(ProcessNoiseMat& processNoiseMat)
{
	memset(&processNoiseMat, 0, sizeof(ProcessNoiseMat));
	for (int i = 0; i < 15; i++)
	{
		if (i < 3) {
			if (i == 0)
				processNoiseMat(i, i) = 1.00000000e-11;// rad/sqrt(s)
			else if (i == 1)
				processNoiseMat(i, i) = 1.00000000e-11;// rad/sqrt(s)
			else if (i == 2)
				processNoiseMat(i, i) = 1.00000000e-4;// m/sqrt(s)
		}
		else if (i >= 3 && i < 6) {
			processNoiseMat(i, i) = 2.16500000e-4;// m//ssqrt(s)
		}
		else if (i >= 6 && i < 9) {
			processNoiseMat(i, i) = 2.75000000e-3;// deg/sqrt(s)
		}
		else if (i >= 9 && i < 12) {
			processNoiseMat(i, i) = 6.62500000e-5;// m/s^2/sqrt(s)
		}
		else if (i >= 12 && i < 15) {
			processNoiseMat(i, i) = 7.71600000e-10;// deg/s/sqrt(s)
		}

	}
	//	cout << processNoiseMat << endl << endl;

}


void buildInitialParaNoiseMat(ParaNoiseMat& initialParaNoiseMat, Vector3d rtkblh) {

	memset(&initialParaNoiseMat, 0, sizeof(ProcessNoiseMat));
	for (int i = 0; i < 15; i++) {
		if (i < 3) {
			if (i == 0)
				initialParaNoiseMat(i, i) = 1.00000000e+2 / ELL.R;// rad
			else if (i == 1)
				initialParaNoiseMat(i, i) = 1.00000000e+2 / ELL.R / cos(rtkblh(0));// rad
			else if (i == 2)
				initialParaNoiseMat(i, i) = 1.00000000e+2;// rad
		}
		else if (i >= 3 && i < 6) {
			initialParaNoiseMat(i, i) = 1.00000000e+0;// m/s
		}
		else if (i >= 6 && i < 9) {
			initialParaNoiseMat(i, i) = 5.00000000e+0;// deg
		}
		else if (i >= 9 && i < 12) {
			initialParaNoiseMat(i, i) = 5.00000000e-2;// m/s^2
		}
		else if (i >= 12 && i < 15) {
			initialParaNoiseMat(i, i) = 5.55555000e-3;// deg/s
		}
	}
}

//LA输入的是b系
//pos都是在经纬高
//velecity都是在ned
//INS 结果转换到经纬高和ENU的速度之后再做差
void buildSDObs(Vector3d blh_GNSS, Vector3d venu_GNSS, INS_par par_INS, SDobs & SDObs, VectorXd L_b, Matrix3d Cbn, IMU_d imu_data) {
	memset(&SDObs, 0, sizeof(SDobs));

	//LA转换坐标系到n系,再转换到经纬高
	Vector3d L_n, L_blh;

	L_n = Cbn * L_b;
	L_blh(0) = L_n(1) / (get_RM(par_INS(0)) + par_INS(2));
	L_blh(1) = L_n(0) / (get_RN(par_INS(0)) + par_INS(2)) / cos(par_INS(0));
	L_blh(2) = -L_n(2);
	//求INS位置转换到经纬高，速度不变，都是ENU
	Vector3d pos_GNSS_at_INS;
	pos_GNSS_at_INS = blh_GNSS - L_blh;
	//GNSS速度转换到ENU
	Vector3d omega_ie_n, omega_en_n, vel_GNSS_at_INS;
	get_omega_ie_n(omega_ie_n, par_INS[0]);
	get_omega_en_n(omega_en_n, par_INS[0], par_INS[2], par_INS[3], par_INS[4]);


	Matrix3d anti_omega_ie_n, anti_omega_en_n, anti_l_b;
	get_anti_mat(anti_omega_ie_n, omega_ie_n);
	get_anti_mat(anti_omega_en_n, omega_en_n);
	get_anti_mat(anti_l_b, L_b);

	Vector3d omega_ib_b = imu_data.block(3, 0, 3, 1) * CPT_SAMP_RATE;
	vel_GNSS_at_INS = venu_GNSS + (anti_omega_ie_n + anti_omega_en_n) *  Cbn * L_b + Cbn * anti_l_b * omega_ib_b;
	//观测值做差
	SDObs.block(0, 0, 3, 1) = par_INS.block<3, 1>(0, 0) - pos_GNSS_at_INS;
	SDObs.block(3, 0, 3, 1) = par_INS.block(3, 0, 3, 1) - vel_GNSS_at_INS;
	//
}


//观测噪声
void buildOBserveNoiseMat(ObserveNoiseMat& observeNoiseMat)
{
	for (int i = 0; i < 6; i++)
	{
		if (i < 2)
			observeNoiseMat(i, i) = 1.000000 / ELL.R;//rad
		else if (i == 3)
		{
			observeNoiseMat(i, i) = 1.00000;//m
		}
		else if (i > 3)
			observeNoiseMat(i, i) = 0.1000000;//m/s
	}

}
