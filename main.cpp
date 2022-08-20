//
// Created by eureka on 4/30/21.
//

//
// Created by eureka on 4/27/21.
//
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <iostream>

#define GLOBAL_VAR

#include "rnx.h"
#include "global.h"
#include "preprocess.h"
#include "spp.h"
#include "rtk.h"
#include "resform.h"
#include "gnss.h"
#include "INS.h"
#include "LC.h"
#include "optimum.h"

#define MAX_GAP_GNSS_INS 5e-3

using namespace std;

int main(int argc, char *args[]) {
	//GNSS
	vector<GnssRes> gnss_res_vec(INIT_NUM_EPO);
	gnss_pv(gnss_res_vec);

	//LA
	Vector3d LA(0.1355, -0.439, 1.0355);

	//设置椭球参数
	ELL.WGS84();
	//打开文件
	FILE *fp_imu = fopen("../data/DAB12440101_25-05-2021_10-34-33.imu.txt","r");
	IMU_d imu_data;
	double week, sow;
	Vector3d att;
	Vector3d att0(0, 0, 0);//记录数据
	//初始对准
	double blh_init[3];     //
	xyz2blh(gnss_res_vec[0].pos_r, blh_init);
	Vector3d pos(blh_init[0], blh_init[1], blh_init[2]);
	IMU_d imu_data_sum;
	imu_data_sum << 0., 0., 0., 0., 0., 0.;
	//初始对准的历元数
	int count = 30000;
	for (int i = 0; i < count; ++i) {
		read_imu_d(fp_imu, imu_data, week, sow);
		imu_data_sum += imu_data;
	}
	imu_data_sum /= count;
	CoarseAlignment(att, imu_data_sum, pos);
	//设置初始对准后的参数
	INS_par par;
	par << pos[0], pos[1], pos[2], 0, 0, 0, att[0], att[1], att[2];
	//cout << par << endl<<endl;


	//构建初始方差
	ParaNoiseMat paraNoiseMat;
	Vector3d blh_init_vec(blh_init[0], blh_init[1], blh_init[2]);
	buildInitialParaNoiseMat(paraNoiseMat, blh_init_vec);
	// 过程噪声
	ProcessNoiseMat processNoiseMat;
	buildProcessNoiseMat(processNoiseMat);
	//设计矩阵
	DesignMat designMat;
	buildDesignMat(designMat);
	//观测噪声
	ObserveNoiseMat observeNoiseMat;
	buildOBserveNoiseMat(observeNoiseMat);
	//状态向量
	StatusVec statusvec = MatrixXd::Zero(15, 1);
	//传递矩阵
	TransferMat transfermat;
	//观测差值
	SDobs SDObs;
	//机械编排 KF
	FILE *lc_res = fopen("../data/lc_res.txt", "w");
	double dt_gnss_ins;
	int ind_gnss = 0;
	bool is_aligned = true;
	int epo_gnss = gnss_res_vec.size();
	read_imu_d(fp_imu, imu_data, week, sow);
	IMU_d bias;
	bias << 0, 0, 0, 0, 0, 0;

	while (!feof(fp_imu) && (ind_gnss < epo_gnss)) {
		dt_gnss_ins = gnss_res_vec[ind_gnss].sow - sow;
		//gnss, ins
		if (dt_gnss_ins < -MAX_GAP_GNSS_INS) {
			ind_gnss++;
		}
		//ins, gnss
		else {
			//机械编排
			INS_par par_before = par;
			Matrix3d Cbn;
			IMU_d imu_data_before = imu_data;

			attitude_update(par, imu_data_before, imu_data, Cbn);
			velocity_update(par, imu_data_before, imu_data);
			position_update(par, par_before);

			int is_lc = 0;
			//printf("%f\n",dt_gnss_ins);
			if (dt_gnss_ins > MAX_GAP_GNSS_INS) {
				buildLCTransferMat(par, imu_data, Cbn, 0.01, transfermat);
				statusvec = transfermat * statusvec;
				paraNoiseMat = transfermat * paraNoiseMat * transfermat.transpose() + processNoiseMat;
			}
			else {
				//KF
				buildLCTransferMat(par, imu_data, Cbn, 0.01, transfermat);
				//                cout<<transfermat<<endl<<endl;
				double blh[3];
				xyz2blh(gnss_res_vec[ind_gnss].pos_r, blh);
				Vector3d rtk_blh(blh[0], blh[1], blh[2]);

				Quaterniond qen = blh2quat(rtk_blh).inverse();
				Vector3d vel_ned = qen * gnss_res_vec[ind_gnss].vel_r;


				buildSDObs(rtk_blh, vel_ned, par, SDObs, LA, Cbn, imu_data);
				kf(transfermat, processNoiseMat, designMat, SDObs, observeNoiseMat, statusvec, paraNoiseMat);
				par -= statusvec.block<9, 1>(0, 0);
				bias = statusvec.block<6, 1>(9, 0);


				buildInitialParaNoiseMat(paraNoiseMat, par.block<3, 1>(0, 0));
				memset(&statusvec, 0, sizeof(StatusVec));
				ind_gnss++;

				is_lc = 1;
			}
			Vector3d xyz = blh2xyz(par.block<3, 1>(0, 0));
			//            fprintf(lc_res, "%7.3lf  %6.6lf  %6.6lf  %6.6lf  %6.6lf  %6.6lf  %6.6lf  %6.6lf  %6.6lf  %6.6lf %d\n",
			//                    sow, par[0] * rad2deg, par[1] * rad2deg, par[2], par[3], par[4], par[5], par[6], par[7], par[8], is_lc);
			fprintf(lc_res, "%7.3lf  %6.6lf  %6.6lf  %6.6lf  %6.6lf  %6.6lf  %6.6lf  %6.6lf  %6.6lf  %6.6lf %d\n",
				sow, xyz[0], xyz[1], xyz[2], par[3], par[4], par[5], par[6], par[7], par[8], is_lc);
			read_imu_d(fp_imu, imu_data, week, sow);

			imu_data -= bias / CPT_SAMP_RATE;
		}
	}
	fclose(fp_imu);
	fclose(lc_res);
	return 0;
}

