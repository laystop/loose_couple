//
// Created by eureka on 4/22/21.
// 基本函数, 包含三大部分
// 字符串处理
// 时间系统转换
// 坐标系统转换

#ifndef RTK_COM_H
#define RTK_COM_H

#include <Eigen/Core>
#include <Eigen/Dense>


using namespace Eigen;

typedef struct {        /* time struct */
	time_t time;        /* time (s) expressed by standard time_t */
	double sec;         /* fraction of second under 1 s */
} gtime_t;

/* Satellite status at time
 * */
typedef struct {
	char prn[4];
	gtime_t gtime;
	Vector3d pos;
	Vector3d vel;
} Satellite;


/* Station status at time
 * */
class Station {
public:
	char name[50];
	gtime_t gtime;
	double rec_clk;
	Vector3d pos;
	Vector3d vel;

	Station(double X, double Y, double Z);
};


/*--------------------------------------字符串处理-----------------------------------*/
//计算有效字符数, 去除空格换行回车
int len_trim(char *pStr);

//去除空格换行回车
char *trim(char *pStr);

// start: started with index of zero
char *substringEx(char *dest, char *src, int start, int length);

//from left to right
int index_string(char *src, char key);

int pointer_string(int row, int col, char *string_array, char *string);

char *left_justify_string(char *string);

int prncmp(char *prn1, char *prn2);

int IsGEO(const char prn[4]);
/*--------------------------------------字符串处理结束---------------------------------*/


/*--------------------------------------时间系统转换-----------------------------------*/
//2 digits year to 4 digits year
void yr2year(int *yr);

//calculate modified_julday
int modified_julday(int iyear, int imonth, int iday);

//time passed from 1 to 2
double timdif(int jd2, double sod2, int jd1, double sod1);

void mjd2wksow(int mjd, double sod, int *week, double *sow);

void mjd2date(int jd, double sod, int *iyear, int *imonth, int *iday, int *ih,
	int *imin, double *sec);

void mjd2doy(int jd, int *iyear, int *idoy);

void yeardoy2monthday(int iyear, int idoy, int *imonth, int *iday);

double time2gpst(gtime_t t, int *week);

gtime_t epoch2time(const double *ep);

gtime_t gpst2time(int week, double sec);
/*-------------------------------------时间系统转换结束----------------------------------*/


/*------------------------------------ Coord Transform ---------------------------------*/
void xyz2blh(const Vector3d &ecef, double *blh);

Vector3d xyz2enu(const Vector3d &rove, const Vector3d &base);

Matrix3d rot_xyz2enu(const Vector3d &base);

Matrix3d rot_enu2xyz(const Vector3d &ecef);

void elevazel(const Vector3d &rove, const Vector3d &base, double *azel);





class Ellipsoid
{
public:
	double GM;//地球引力常数
	double f;//扁率
	double e2;//第一偏心率平方
	double a;//长半轴
	double b;//短半轴
	double U0;//椭球正常重力位
	double omega_e;//地球自转速度
	double ga;//赤道重力
	double gb;//极点重力
	double R;//平均半径
	double beta, beta_1, beta_2, beta_3;//重力扁率
	//wgs84
	void WGS84()
	{
		GM = 3.986004418e14;
		f = 1.0 / 298.257223563;
		e2 = 0.00669437999013;
		a = 6378137.0;
		b = 6356752.3142;
		U0 = 62636860.8497;
		omega_e = 7.292115e-5;
		ga = 9.7803267715;
		gb = 9.8321863685;
		R = 2.0 / 3.0*a + 1.0 / 3.0 * b;
		beta = (gb - ga) / ga;
		beta_1 = (2 * beta*f + f * f) / 8.0;
		beta_2 = 2 * GM / pow(R, 3);
		beta_3 = 8.08e-9;
	}
};


typedef Matrix<double, 6, 1> IMU_d;//imu数据 加速度计，陀螺增量

/*
//估计结果参数  国际单位
B(0)  L(1)  H(2)
vn(3) ve(4) vd(5)
横滚(6) 俯仰(7) 航向(8)
*/
typedef Matrix<double, 9, 1> INS_par;


/*
	orthonormalize

目的：
	对矩阵进行施密特正交化

参数：
	ColVecs 需要正交化的矩阵

返回值：无；
*/
void orthonormalize(Matrix3d& ColVecs);
void orthonormalize(MatrixXd& ColVecs);


/*
get_g

目的：
	计算当地的重力

参数：
	B	当地纬度（RAD)
	H	高程(m)

返回值：
	当地重力
*/
double get_g(double B, double H);


/*
get_RM

目的：
	计算子午圈半径

参数：
	B	当地纬度（RAD)

返回值：
	子午圈半径
*/
double get_RM(double B);


/*
get_RN

目的：
	计算卯酉圈半径

参数：
	B	当地纬度（RAD)

返回值：
	卯酉圈半径
*/
double get_RN(double B);


/*
get_omega_ie_n

目的：
	计算n系地球自转速度

参数：
	omega	3*1向量，计算结果
	B		当地纬度（RAD)

返回值：
	无
*/
void get_omega_ie_n(Vector3d &omega, double B);


/*
get_omega_en_n

目的：
	计算n系角速度速度

参数：
	omega	3*1向量，计算结果
	B		纬度
	h		高程
	vn		北向速度
	ve		东向速度
返回值：
	无
*/
void get_omega_en_n(Vector3d &omega, double B, double h, double vn, double ve);


/*
get_anti_mat

目的：
	获得反对称矩阵

参数：
	out		结果
	in		输入向量
返回值：
	无
*/
void get_anti_mat(Matrix3d &out, Vector3d in);


/*
update_att

目的：
	将姿态角转到适应的范围，不然eigne库会出错

参数：
	att	姿态角
返回值：
	无
*/
void update_att(Vector3d &att);

Quaterniond blh2quat(Vector3d blh);
Vector3d blh2xyz(const Vector3d &blh);

#endif //RTK_COM_H
