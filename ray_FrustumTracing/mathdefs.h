#ifndef MATHDEFS_H
#define MATHDEFS_H

#include <cmath>
#include <iostream>
#include "define.h"
using namespace std;

const float DEG_TO_RAD = ((2.0f * PI) / 360.0f);
const float RAD_TO_DEG = (360.0f / (2.0f * PI));
const float RAD_90_DEG = (float)PI/2.0f;
const float EPS = 1e-5;

class Vector3f
{
public:
	float x;
	float y;
	float z;

	
	Vector3f(): x(0.0f), y(0.0f), z(0.0f)			{}
	Vector3f(float tmp): x(tmp), y(tmp), z(tmp)		{}
	Vector3f(float xTmp, float yTmp, float zTmp)	{x = xTmp; y = yTmp; z = zTmp;}

	void SetZero()									{x = 0.0f; y = 0.0f; z = 0.0f;}
	void Set(float tmp)								{x = tmp; y = tmp; z = tmp;}
	void Set(float xTmp, float yTmp, float zTmp)	{x = xTmp; y = yTmp; z = zTmp;}

	Vector3f&	operator+=(const Vector3f& tmp)		{x += tmp.x; y += tmp.y; z += tmp.z; return *this;}
	Vector3f&	operator*=(const Vector3f& tmp)		{x *= tmp.x; y *= tmp.y; z *= tmp.z; return *this;}
	Vector3f	operator/(int tmp)					{return Vector3f(this->x / tmp, this->y / tmp, this->z / tmp);}
	Vector3f	operator/(float tmp)				{ return Vector3f(this->x / tmp, this->y / tmp, this->z / tmp); }
	Vector3f	operator+(const Vector3f& tmp)		{ return Vector3f(this->x + tmp.x, this->y + tmp.y, this->z + tmp.z); }
	Vector3f	operator-(const Vector3f& tmp)		{ return Vector3f(this->x - tmp.x, this->y - tmp.y, this->z - tmp.z); }
	Vector3f	operator*(const Vector3f& tmp)		{ return Vector3f(this->x * tmp.x, this->y * tmp.y, this->z * tmp.z); }
	float&		operator[](int i);
};


class Vector2f
{
public:
	float x;
	float z;

	Vector2f(): x(0.0f), z(0.0f)		{}
	Vector2f(float xTmp, float zTmp)	{x = xTmp; z = zTmp;}

	void SetZero()						{x = 0.0f; z = 0.0f;}
	void Set(float xTmp, float zTmp)	{x = xTmp; z = zTmp;}
};

//////////////////////////////////////////////////////////////////////////
//Vector4f
class Vector4f
{
public:

	float x;
	float y;
	float z;
	float w;

	Vector4f(): x(0.0f), y(0.0f), z(0.0f), w(0.0f)			{}
	Vector4f(float tmp): x(tmp), y(tmp), z(tmp), w(tmp)		{}
	Vector4f(float xTmp, float yTmp, float zTmp, float wTmp = 0.0f);
	Vector4f(Vector3f& v3f);

	float&		operator[]	(int i);
	Vector4f&	operator=	(const Vector4f& tmp);
	Vector4f&	operator+=	(const Vector4f& tmp);
	Vector4f&	operator-=	(const Vector4f& tmp);
	Vector4f&	operator*=	(const Vector4f& tmp);
	Vector4f&	operator/=	(float tmp);
	Vector4f	operator/	(float tmp);
	Vector4f	operator*	(float tmp);
	Vector4f	operator+	(const Vector4f& tmp);
	Vector4f	operator-	(const Vector4f& tmp);
	Vector4f	operator*	(const Vector4f& tmp);

	void	SetZero		();
	void	Set			(float tmp);
	void	Set			(float xTmp, float yTmp, float zTmp, float wTmp = 0.0f);
	void	Set			(Vector3f tmp);
	void	Set			(Vector4f tmp);
	void	SetLenght	(float tmp);

	float	GetLenght	();

	static float	Dot3f(const Vector4f& a, const Vector4f& b);
	static Vector4f Cross3f(const Vector4f& a, const Vector4f& b);

};
Vector4f CreatPlane(Vector4f v1, Vector4f v2, Vector4f v3);
//////////////////////////////////////////////////////////////////////////
//Quaternion
class Quaternion4f
{
public:
	float x;
	float y;
	float z;
	float W;

	Quaternion4f();
	~Quaternion4f(){}
	Quaternion4f(float tmpX, float tmpY, float tmpZ, float tmpW);

	//dostajemy trzy konty Eulera i ustawiamy na ich podstawie kwaternion
	void SetBasedOnEuler(float roll, float pitch, float yaw);
};



//////////////////////////////////////////////////////////////////////////
//Matrix4x4f
class Matrix4x4f
{
public:
	Vector4f r0;
	Vector4f r1;
	Vector4f r2;
	Vector4f r3;

	Matrix4x4f();
	Matrix4x4f(float tmp);
	~Matrix4x4f();

	void LoadIdentity();
	void SetZero();

	Vector4f&	operator[](int i);
	Matrix4x4f&	operator+=(Matrix4x4f& tmp);
	Matrix4x4f&	operator/=(float tmp);
	Matrix4x4f operator*(float tmp);
	Matrix4x4f& transpose();
	Matrix4x4f operator*(Matrix4x4f mat);
	Vector4f operator*(Vector4f v4f);
	Matrix4x4f operator+(Matrix4x4f mat);

	static void Multiply(Matrix4x4f& res, Vector4f& v1, Vector4f& v2);
	
	static void CalculateMatrixFromEuler(float ax, float ay, float az, Matrix4x4f& mat);
	static void QuaternionToMatrix(const Quaternion4f& quat, Matrix4x4f& mat);
	static void InverseRotate(Matrix4x4f& mat, Vector4f& point);
	static void Rotate(Matrix4x4f& mat, Vector4f& point);
	static void Transform(Matrix4x4f& mat, Vector4f& point);
};

const double pi = 3.14159265359;

//复数定义
typedef struct COMPLEX
{
	double re;
	double im;

	COMPLEX() :re(0), im(0) {};
	COMPLEX(double re, double im) :re(re), im(im) {};
};

//复数加运算
COMPLEX Add(COMPLEX c1, COMPLEX c2);
COMPLEX operator+(COMPLEX c1, COMPLEX c2);

//复数乘运算
COMPLEX Mul(COMPLEX c1, COMPLEX c2);
COMPLEX operator*(double d, COMPLEX c);

//复数减运算
COMPLEX Sub(COMPLEX c1, COMPLEX c2);
COMPLEX operator-(COMPLEX c1, COMPLEX c2);

//快速傅里叶变换
void FFT(COMPLEX *TD, COMPLEX *FD, int power);

//逆变换
void IFFT(COMPLEX *FD, COMPLEX *TD, int power);

//定义指向性函数
double DirectBeam(Vector4f drct);
#endif




