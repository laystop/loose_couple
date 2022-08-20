//
// Created by eureka on 4/22/21.
//
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "global.h"
#include "common.h"


using namespace Eigen;
using namespace std;

Station::Station(double X, double Y, double Z) {
	pos[0] = X;
	pos[1] = Y;
	pos[2] = Z;

	rec_clk = 0.0;
	vel << 0., 0., 0.;
}

//计算有效字符数, 去除空格换行回车
int len_trim(char *pStr) {
	int length = strlen(pStr);
	int count = length;
	int i;
	for (i = length - 1; i >= 0; i--) {
		if (pStr[i] == '\0' || pStr[i] == '\n' || pStr[i] == '\r' || pStr[i] == ' ')
			//				空格				换行				回车			??? 和空格一样
			count--;
		else
			break;
	}
	return count;
}


// start: started with index of zero
char *substringEx(char *dest, char *src, int start, int length) {
	int i, j = 0;
	int len = strlen(src);
	if (start < 0 || start >= len) {
		return NULL;
	}

	if (start + length > len) {
		length = len - start;
	}

	for (i = start; i < start + length; i++) {
		dest[j] = src[i];
		j++;
	}

	dest[j] = '\0';
	return dest;
}

void yr2year(int *yr) {
	if (*yr > 1900)
		return;
	if (*yr <= 20)
		*yr += 2000;
	else
		*yr += 1900;
}


int modified_julday(int iyear, int imonth, int iday) {
	int iyr, result;
	int doy_of_month[12] = { 0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304,
							334 };
	if (iyear < 0 || imonth < 0 || iday < 0 || imonth > 12 || iday > 366 || (imonth != 0 && iday > 31)) {
		printf("***ERROR(modified_julday): incorrect arguments!%d %d %d\n",
			iyear, imonth, iday);
		exit(1);
	}
	iyr = iyear;
	if (imonth <= 2)
		iyr -= 1;
	result = 365 * iyear - 678941 + iyr / 4 - iyr / 100 + iyr / 400 + iday;

	if (imonth != 0)
		result = result + doy_of_month[imonth - 1];
	return result;
}


//time passed from 1 to 2, 2 - 1
double timdif(int jd2, double sod2, int jd1, double sod1) {
	return 86400.0 * (jd2 - jd1) + sod2 - sod1;
}

//from left to right
int index_string(char *src, char key) {
	int len = strlen(src);
	int i;
	for (i = 0; i < len; i++) {
		if (src[i] == key)
			break;
	}
	if (i == len)
		return -1;
	else
		return i;
}


char *trim(char *pStr) {
	int len = strlen(pStr);
	char *pIndex = pStr + len - 1;
	while (*pIndex == '\n' || *pIndex == '\r' || *pIndex == '\0' || *pIndex == ' ') {
		*pIndex = '\0';
		pIndex--;
	}
	return pStr;
}

/* find string in string_array, return the order
 * row: max order
 * col: step width
 * return -1 if not found*/
int pointer_string(int row, int col, char *string_array, char *string) {
	int i;
	char *pStr = (char *)string_array;
	trim(string);

	for (i = 0; i < row; i++) {
		if (strcmp(pStr + i * col, string) == 0)
			break;
	}
	if (i == row)
		i = -1;
	return i;
}


/* compare prn -----------------------------------------------------------------
 * ---------------------------------------------------------------------------*/
int prncmp(char *prn1, char *prn2) {
	int isys1 = index_string(SYS, prn1[0]);
	int isys2 = index_string(SYS, prn2[0]);
	int d = isys1 - isys2;
	if (d)
		return d;
	else {
		for (int i = 1; i < 3; ++i) {
			d = prn1[i] - prn2[i];
			if (d)
				return d;
		}
	}
	return 0;
}

int IsGEO(const char prn[4]) {
	if (prn[0] != 'C')
		return 0;
	else {
		int ind = (int)(prn[1] - '0') * 10 + (int)(prn[2] - '0');
		return (ind < 6 || ind > 58);
	}
}


char *left_justify_string(char *string) {
	int p = 0;
	while (*(string + p) == ' ' || *(string + p) == '\n'
		|| *(string + p) == '\r')
		p++;
	return string + p;
}

void mjd2wksow(int mjd, double sod, int *week, double *sow) {
	*week = (int)((mjd + sod / 86400.0 - 44244.0) / 7.0);
	*sow = (mjd - 44244.0 - *week * 7) * 86400.0 + sod;
}

void mjd2date(int jd, double sod, int *iyear, int *imonth, int *iday, int *ih,
	int *imin, double *sec) {
	int doy = 0;
	mjd2doy(jd, iyear, &doy);
	yeardoy2monthday(*iyear, doy, imonth, iday);

	*ih = (int)sod / 3600.0;
	*imin = (int)((sod - (*ih) * 3600.0) / 60.0);
	*sec = sod - (*ih) * 3600.0 - (*imin) * 60.0;
}


void yeardoy2monthday(int iyear, int idoy, int *imonth, int *iday) {
	int days_in_month[12] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
	int id, i;
	if ((iyear % 4 == 0 && iyear % 100 != 0) || iyear % 400 == 0)
		days_in_month[1] = 29;
	id = idoy;
	for (i = 0; i < 12; i++) {
		id = id - days_in_month[i];
		if (id > 0)
			continue;
		*iday = id + days_in_month[i];
		*imonth = i + 1;
		break;
	}
}


void mjd2doy(int jd, int *iyear, int *idoy) {
	*iyear = (jd + 678940) / 365;
	*idoy = jd - modified_julday(*iyear, 1, 1);
	while (*idoy < 0) {
		(*iyear)--;
		*idoy = jd - modified_julday(*iyear, 1, 1) + 1;
	}
}

/* time to gps time ------------------------------------------------------------
* convert gtime_t struct to week and tow in gps time
* args   : gtime_t t        I   gtime_t struct
*          int    *week     IO  week number in gps time (NULL: no output)
* return : time of week in gps time (s)
*-----------------------------------------------------------------------------*/
double time2gpst(gtime_t t, int *week) {
	gtime_t t0 = epoch2time(gpst0);
	time_t sec = t.time - t0.time;
	int w = (int)(sec / (86400 * 7));

	if (week) *week = w;
	return (double)(sec - (double)w * 86400 * 7) + t.sec;
}


/* convert calendar day/time to time -------------------------------------------
* convert calendar day/time to gtime_t struct
* args   : double *ep       I   day/time {year,month,day,hour,min,sec}
* return : gtime_t struct
* notes  : proper in 1970-2037 or 1970-2099 (64bit time_t)
*-----------------------------------------------------------------------------*/
gtime_t epoch2time(const double *ep) {
	const int doy[] = { 1, 32, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335 };
	gtime_t time = { 0 };
	int days, sec, year = (int)ep[0], mon = (int)ep[1], day = (int)ep[2];

	if (year < 1970 || 2099 < year || mon < 1 || 12 < mon) return time;

	/* leap year if year%4==0 in 1901-2099 */
	days = (year - 1970) * 365 + (year - 1969) / 4 + doy[mon - 1] + day - 2 + (year % 4 == 0 && mon >= 3 ? 1 : 0);
	sec = (int)floor(ep[5]);
	time.time = (time_t)days * 86400 + (int)ep[3] * 3600 + (int)ep[4] * 60 + sec;
	time.sec = ep[5] - sec;
	return time;
}


/* gps time to time ------------------------------------------------------------
* convert week and tow in gps time to gtime_t struct
* args   : int    week      I   week number in gps time
*          double sec       I   time of week in gps time (s)
* return : gtime_t struct
*-----------------------------------------------------------------------------*/
extern gtime_t gpst2time(int week, double sec) {
	gtime_t t = epoch2time(gpst0);

	if (sec < -1E9 || 1E9 < sec) sec = 0.0;
	t.time += 86400 * 7 * week + (int)sec;
	t.sec = sec - (int)sec;
	return t;
}


/*--------------------------------------------------- Coord Transform ---------------------------------*/
void xyz2blh(const Vector3d &ecef, double *blh) {
	double a = 6378137;
	double e2 = 0.0066943799013;

	double X = ecef(0);
	double Y = ecef(1);
	double Z = ecef(2);
	double L = atan2(Y, X);
	double B = 999.0;
	double B_prime = atan2(Z, sqrt(X * X + Y * Y));
	double W, N, H;
	while (fabs(B - B_prime) > 1e-11) {
		B = B_prime;
		W = sqrt(1 - e2 * sin(B) * sin(B));
		N = a / W;
		H = sqrt(X * X + Y * Y) / cos(B) - N;
		B_prime = atan2(Z * (N + H), sqrt(X * X + Y * Y) * (N * (1 - e2) + H));
	}
	blh[0] = B;
	blh[1] = L;
	blh[2] = H;
}


Vector3d xyz2enu(const Vector3d &rove, const Vector3d &base) {
	Matrix3d rot_mat = rot_xyz2enu(base);
	Vector3d d = rove - base;
	Vector3d enu = rot_mat * d;
	return enu;
}


Matrix3d rot_xyz2enu(const Vector3d &ecef) {
	double blh[3];
	xyz2blh(ecef, blh);

	double sin_L = sin(blh[1]);
	double sin_B = sin(blh[0]);
	double cos_L = cos(blh[1]);
	double cos_B = cos(blh[0]);

	Matrix3d rot_mat;
	rot_mat << -sin_L, cos_L, 0.0,
		sin_B * cos_L, -sin_B * sin_L, cos_B,
		cos_B * cos_L, cos_B * sin_L, sin_B;
	return rot_mat;
}


Matrix3d rot_enu2xyz(const Vector3d &ecef) {
	double blh[3];
	xyz2blh(ecef, blh);

	double sin_L = sin(blh[1]);
	double sin_B = sin(blh[0]);
	double cos_L = cos(blh[1]);
	double cos_B = cos(blh[0]);

	Matrix3d rot_mat;
	rot_mat << -sin_L, sin_B * cos_L, cos_B * cos_L,
		cos_L, -sin_B * sin_L, cos_B * sin_L,
		0.0, cos_B, sin_B;
	return rot_mat;
}


void elevazel(const Vector3d &rove, const Vector3d &base, double *azel) {
	Vector3d r = xyz2enu(rove, base);
	azel[0] = atan2(r[0], r[1]);
	azel[1] = atan(r[2] / sqrt(r[0] * r[0] + r[1] * r[1]));
}


/*
	orthonormalize

目的：
	对矩阵进行施密特正交化

参数：
	ColVecs 需要正交化的矩阵

返回值：无；
*/
void orthonormalize(Matrix3d &ColVecs) {
	ColVecs.col(0).normalize();
	double temp;
	for (int k = 0; k != ColVecs.cols() - 1; ++k) {
		for (int j = 0; j != k + 1; ++j) {
			temp = ColVecs.col(j).transpose() * ColVecs.col(k + 1);
			ColVecs.col(k + 1) -= ColVecs.col(j) * temp;
		}
		ColVecs.col(k + 1).normalize();
	}
}

void orthonormalize(MatrixXd &ColVecs) {
	ColVecs.col(0).normalize();
	double temp;
	for (int k = 0; k != ColVecs.cols() - 1; ++k) {
		for (int j = 0; j != k + 1; ++j) {
			temp = ColVecs.col(j).transpose() * ColVecs.col(k + 1);
			ColVecs.col(k + 1) -= ColVecs.col(j) * temp;
		}
		ColVecs.col(k + 1).normalize();
	}
}


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
double get_g(double B, double H) {
	double cb2 = pow(cos(B), 2), sb2 = pow(sin(B), 2);
	double m = pow(ELL.omega_e * ELL.a, 2) * ELL.b / ELL.GM;

	double g0 = ELL.a * ELL.ga * cb2 + ELL.b * ELL.gb * sb2;
	g0 /= sqrt(ELL.a * ELL.a * cb2 + ELL.b * ELL.b * sb2);
	double g = g0 * (1 - 2.0 / ELL.a * (1 + ELL.f + m - 2 * ELL.f * sb2) * H + 3 / (ELL.a * ELL.a) * H * H);
	return g;

}


/*
get_RM

目的：
	计算子午圈半径

参数：
	B	当地纬度（RAD)

返回值：
	子午圈半径
*/
double get_RM(double B) {
	return ELL.a * (1 - ELL.e2) / pow(1 - ELL.e2 * pow(sin(B), 2), 1.5);
}


/*
get_RN

目的：
	计算卯酉圈半径

参数：
	B	当地纬度（RAD)

返回值：
	卯酉圈半径
*/
double get_RN(double B) {
	return ELL.a / sqrt(1 - ELL.e2 * pow(sin(B), 2));
}


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
void get_omega_ie_n(Vector3d &omega, double B) {
	omega << ELL.omega_e * cos(B), 0, -ELL.omega_e * sin(B);
}


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
void get_omega_en_n(Vector3d &omega, double B, double h, double vn, double ve) {
	double RM = get_RM(B);
	double RN = get_RN(B);
	omega << ve / (RN + h), -vn / (RM + h), -ve * tan(B) / (RN + h);
}


/*
get_anti_mat

目的：
	获得反对称矩阵

参数：
	out		获取的反对称矩阵
	in		输入向量
返回值：
	无
*/
void get_anti_mat(Matrix3d &out, Vector3d in) {
	out << 0, -in[2], in[1],
		in[2], 0, -in[0],
		-in[1], in[0], 0;
}


/*
update_att

目的：
	将姿态角转到适应的范围，不然eigne库会出错

参数：
	att	姿态角
返回值：
	无
*/
void update_att(Vector3d &att) {
	att[0] += (att[0] > PI) ? (-2 * PI) : ((att[0] < -PI) ? (2 * PI) : 0);
	att[2] += (att[2] > PI) ? (-2 * PI) : ((att[2] < -PI) ? (2 * PI) : 0);
	att[1] += (att[1] > 0.5 * PI) ? -PI : ((att[1] < -0.5 * PI) ? PI : 0);
}

/*
blh2quat

目的：
	根据blh计算n系相对于e系的姿态（四元数表示）

参数：
	att	姿态角
返回值：
	无
*/
Quaterniond blh2quat(Vector3d blh)
{
	double B = blh(0);
	double L = blh(1);
	double CL = cos(L / 2), SL = sin(L / 2);
	double CB = cos(-PI / 4 - B / 2), SB = sin(-PI / 4 - B / 2);
	Quaterniond q(CB * CL, -SB * SL, SB * CL, CB * SL);
	q.normalized();
	return q;
}

Vector3d blh2xyz(const Vector3d &blh) {
	double a = 6378137.0;
	double e2 = 0.006694380004261;

	double sb = sin(blh[0]);
	double cb = cos(blh[0]);
	double sl = sin(blh[1]);
	double cl = cos(blh[1]);
	double h = blh[2];
	double N = a / sqrt(1 - e2 * sb*sb);
	Vector3d xyz((N + h)*cb*cl, (N + h)*cb*sl, (N*(1 - e2) + h)*sb);
	return xyz;
}