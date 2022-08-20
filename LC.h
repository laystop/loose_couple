#pragma once
#include"global.h"
#include"common.h"

typedef Matrix<double, 15, 1> StatusVec;
typedef Matrix<double, 15, 15> StatusCofactorMat;
typedef Matrix<double, 15, 15> TransferMat;
typedef Matrix<double, 15, 15> TransferNoise;
typedef Matrix<double, 6, 15>DesignMat;
typedef Matrix<double, 15, 15>ProcessNoiseMat;
typedef Matrix<double, 15, 15>ParaNoiseMat;
typedef Matrix<double, 6, 1> SDobs;
typedef Matrix<double, 6, 6> ObserveNoiseMat;
typedef Matrix <double, 15, 1> StatusVec;

void  buildLCTransferMat(INS_par par_before, IMU_d obs, Matrix3d Cbn, double relevantTime, TransferMat& transfermat);
void buildDesignMat(DesignMat& designMat);
void buildProcessNoiseMat(ProcessNoiseMat& processNoiseMat);
void buildInitialParaNoiseMat(ParaNoiseMat& initialParaNoiseMat, Vector3d rtkblh);
void buildSDObs(Vector3d blh_GNSS, Vector3d venu_GNSS, INS_par par_INS, SDobs& SDObs, VectorXd L_b, Matrix3d Cbn, IMU_d imu_data);
void buildOBserveNoiseMat(ObserveNoiseMat& observeNoiseMat);


void LCtimeforcast(StatusVec lastStatusvec, StatusCofactorMat lastStatusQ, TransferNoise transfernoiseVec, StatusVec& forcastedStatusVec, StatusCofactorMat& forcastedQ);
