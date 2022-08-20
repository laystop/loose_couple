#pragma once
/*
	ͳһ��д�±���д�ϱ�


*/
#include "common.h"
#include "global.h"



void read_imu_d(FILE *fp_imu, IMU_d &imu_data, double &week, double &sow);


/*
CoarseAlignment

Ŀ�ģ�
	��̬�ֶ�׼

������
	att		3*1�����������̬�ǣ�����ǡ������ǡ������(rad)��
	obs		6*1����������۲�ֵ������x/y/z(m/s),�������x/y/z(rad)����Ϊ����
	pos		3*1������γ���ߣ���RTK�ṩ��rad/rad/m
����ֵ��
	��

*/
void CoarseAlignment(Vector3d &att, IMU_d obs, Vector3d  pos);



/*
attitude_update

Ŀ�ģ�
	��̬����

������
	par			9*1,Ҫ���ƵĲ���
	obs			6*1����������۲�ֵ������x/y/z(m/s),�������x/y/z(rad)����Ϊ����
	obs_before	��һʱ�̵Ĺ۲�ֵ
����ֵ��
	��

*/
void attitude_update(INS_par& par, IMU_d obs, IMU_d obs_before, Matrix3d& Cbn);


/*
velocity_update

Ŀ�ģ�
	�ٶȸ���

������
	par			9*1,Ҫ���ƵĲ���
	obs			6*1����������۲�ֵ������x/y/z(m/s),�������x/y/z(rad)����Ϊ����
	obs_before	��һʱ�̵Ĺ۲�ֵ
����ֵ��
	��

*/
void velocity_update(INS_par &par, IMU_d obs, IMU_d obs_before);


/*
position_update

Ŀ�ģ�
	λ�ø���

������
	par			9*1,Ҫ���ƵĲ���
	par_befor	��һʱ�̹��ƵĲ���
����ֵ��
	��

*/
void position_update(INS_par &par, INS_par &par_before);

