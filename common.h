//
// Created by eureka on 4/22/21.
// ��������, �������󲿷�
// �ַ�������
// ʱ��ϵͳת��
// ����ϵͳת��

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


/*--------------------------------------�ַ�������-----------------------------------*/
//������Ч�ַ���, ȥ���ո��лس�
int len_trim(char *pStr);

//ȥ���ո��лس�
char *trim(char *pStr);

// start: started with index of zero
char *substringEx(char *dest, char *src, int start, int length);

//from left to right
int index_string(char *src, char key);

int pointer_string(int row, int col, char *string_array, char *string);

char *left_justify_string(char *string);

int prncmp(char *prn1, char *prn2);

int IsGEO(const char prn[4]);
/*--------------------------------------�ַ����������---------------------------------*/


/*--------------------------------------ʱ��ϵͳת��-----------------------------------*/
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
/*-------------------------------------ʱ��ϵͳת������----------------------------------*/


/*------------------------------------ Coord Transform ---------------------------------*/
void xyz2blh(const Vector3d &ecef, double *blh);

Vector3d xyz2enu(const Vector3d &rove, const Vector3d &base);

Matrix3d rot_xyz2enu(const Vector3d &base);

Matrix3d rot_enu2xyz(const Vector3d &ecef);

void elevazel(const Vector3d &rove, const Vector3d &base, double *azel);





class Ellipsoid
{
public:
	double GM;//������������
	double f;//����
	double e2;//��һƫ����ƽ��
	double a;//������
	double b;//�̰���
	double U0;//������������λ
	double omega_e;//������ת�ٶ�
	double ga;//�������
	double gb;//��������
	double R;//ƽ���뾶
	double beta, beta_1, beta_2, beta_3;//��������
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


typedef Matrix<double, 6, 1> IMU_d;//imu���� ���ٶȼƣ���������

/*
//���ƽ������  ���ʵ�λ
B(0)  L(1)  H(2)
vn(3) ve(4) vd(5)
���(6) ����(7) ����(8)
*/
typedef Matrix<double, 9, 1> INS_par;


/*
	orthonormalize

Ŀ�ģ�
	�Ծ������ʩ����������

������
	ColVecs ��Ҫ�������ľ���

����ֵ���ޣ�
*/
void orthonormalize(Matrix3d& ColVecs);
void orthonormalize(MatrixXd& ColVecs);


/*
get_g

Ŀ�ģ�
	���㵱�ص�����

������
	B	����γ�ȣ�RAD)
	H	�߳�(m)

����ֵ��
	��������
*/
double get_g(double B, double H);


/*
get_RM

Ŀ�ģ�
	��������Ȧ�뾶

������
	B	����γ�ȣ�RAD)

����ֵ��
	����Ȧ�뾶
*/
double get_RM(double B);


/*
get_RN

Ŀ�ģ�
	����î��Ȧ�뾶

������
	B	����γ�ȣ�RAD)

����ֵ��
	î��Ȧ�뾶
*/
double get_RN(double B);


/*
get_omega_ie_n

Ŀ�ģ�
	����nϵ������ת�ٶ�

������
	omega	3*1������������
	B		����γ�ȣ�RAD)

����ֵ��
	��
*/
void get_omega_ie_n(Vector3d &omega, double B);


/*
get_omega_en_n

Ŀ�ģ�
	����nϵ���ٶ��ٶ�

������
	omega	3*1������������
	B		γ��
	h		�߳�
	vn		�����ٶ�
	ve		�����ٶ�
����ֵ��
	��
*/
void get_omega_en_n(Vector3d &omega, double B, double h, double vn, double ve);


/*
get_anti_mat

Ŀ�ģ�
	��÷��Գƾ���

������
	out		���
	in		��������
����ֵ��
	��
*/
void get_anti_mat(Matrix3d &out, Vector3d in);


/*
update_att

Ŀ�ģ�
	����̬��ת����Ӧ�ķ�Χ����Ȼeigne������

������
	att	��̬��
����ֵ��
	��
*/
void update_att(Vector3d &att);

Quaterniond blh2quat(Vector3d blh);
Vector3d blh2xyz(const Vector3d &blh);

#endif //RTK_COM_H
