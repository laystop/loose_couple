#pragma once
/*
	统一先写下标再写上标


*/
#include "common.h"
#include "global.h"



void read_imu_d(FILE *fp_imu, IMU_d &imu_data, double &week, double &sow);


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
void CoarseAlignment(Vector3d &att, IMU_d obs, Vector3d  pos);



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
void attitude_update(INS_par& par, IMU_d obs, IMU_d obs_before, Matrix3d& Cbn);


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
void velocity_update(INS_par &par, IMU_d obs, IMU_d obs_before);


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
void position_update(INS_par &par, INS_par &par_before);

