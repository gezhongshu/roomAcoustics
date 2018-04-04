#include "mathdefs.h"

//////////////////////////////////////////////////////////////////////////
//Vector3f

float& Vector3f::operator[](int i)
{
	switch (i) {
	case 1:	return x;
	case 2:	return y;
	case 3: return z;
	}
}


//////////////////////////////////////////////////////////////////////////
//Vector4f
Vector4f::Vector4f(float xTmp, float yTmp, float zTmp, float wTmp)
{
	x = xTmp; 
	y = yTmp;
	z = zTmp;
	w = wTmp;
}

Vector4f::Vector4f(Vector3f& v3f)
{
	x = v3f.x;
	y = v3f.y;
	z = v3f.z;
	w = 0.0;
}

void Vector4f::SetZero()
{
	x = y = z = w = 0.0f;
}

void Vector4f::Set(float tmp)
{
	x = y = z = w = tmp;
}

void Vector4f::Set(float xTmp, float yTmp, float zTmp, float wTmp)
{
	x = xTmp;
	y = yTmp;
	z = zTmp;
	w = wTmp;
}

void Vector4f::Set(Vector3f tmp)
{
	x = tmp.x;
	y = tmp.y;
	z = tmp.z;
	w = 0.0;
}

void Vector4f::Set(Vector4f tmp)
{
	x = tmp.x;
	y = tmp.y;
	z = tmp.z;
	w = tmp.w;
}

float&	Vector4f::operator[](int i)
{
	switch (i) {
	case 0: return x;
	case 1:	return y;
	case 2: return z;
	case 3: return w;
	}
}

Vector4f& Vector4f::operator=(const Vector4f& tmp)
{
	x = tmp.x;
	y = tmp.y;
	z = tmp.z;
	w = tmp.w;
	return *this;
}

Vector4f& Vector4f::operator+=(const Vector4f& tmp)
{
	x += tmp.x;
	y += tmp.y;
	z += tmp.z;
	w += tmp.w;
	return *this;
}

Vector4f& Vector4f::operator-=(const Vector4f& tmp)
{
	x -= tmp.x;
	y -= tmp.y;
	z -= tmp.z;
	w -= tmp.w;
	return *this;
}

Vector4f& Vector4f::operator*=(const Vector4f& tmp)
{
	x *= tmp.x;
	y *= tmp.y;
	z *= tmp.z;
	w *= tmp.w;
	return *this;
}

Vector4f& Vector4f::operator/=(float tmp)
{
	x /= tmp;
	y /= tmp;
	z /= tmp;
	w /= tmp;
	return *this;
}

Vector4f Vector4f::operator/(float tmp)
{
	return Vector4f(this->x/tmp, this->y/tmp, this->z/tmp, this->w/tmp);
}

Vector4f Vector4f::operator*(float tmp)
{
	return Vector4f(this->x*tmp, this->y*tmp, this->z*tmp, this->w*tmp);
}

Vector4f Vector4f::operator+(const Vector4f& tmp)
{
	return Vector4f(x+tmp.x, y+tmp.y, z+tmp.z);
}

Vector4f Vector4f::operator-(const Vector4f& tmp)
{
	return Vector4f(x-tmp.x, y-tmp.y, z-tmp.z);
}

Vector4f Vector4f::operator*(const Vector4f& tmp)
{
	return Vector4f(x*tmp.x, y*tmp.y, z*tmp.z);
}


float Vector4f::Dot3f(const Vector4f& a,const Vector4f& b)
{
	return (a.x*b.x + a.y*b.y + a.z*b.z); 
}

Vector4f Vector4f::Cross3f(const Vector4f& a, const Vector4f& b)
{
	return Vector4f(a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x);
}

float Vector4f::GetLenght()
{
	return sqrt(x*x + y*y + z*z);
}

void Vector4f::SetLenght(float tmp)
{
	float lgh = GetLenght();
	if (lgh <= 1e-10)return;
	x = x/lgh * tmp;
	y = y/lgh * tmp;
	z = z/lgh * tmp;
	lgh = GetLenght();
}

//////////////////////////////////////////////////////////////////////////
//Quaternion

Quaternion4f::Quaternion4f(): x(0.0f), y(0.0f), z(0.0f), W(0.0f)
{}

Quaternion4f::Quaternion4f(float tmpX, float tmpY, float tmpZ, float tmpW): x(tmpX), y(tmpY), z(tmpZ), W(tmpW)
{}

void Quaternion4f::SetBasedOnEuler(float roll, float pitch, float yaw)
{
	roll *= 0.5f;
	pitch *= 0.5f;
	yaw *= 0.5f;

	float cr = cosf(roll);
	float cp = cosf(pitch);
	float cy = cosf(yaw);

	float sr = sinf(roll);
	float sp = sinf(pitch);
	float sy = sinf(yaw);

	float cpcy = cp * cy;
	float spsy = sp * sy;
	float spcy = sp * cy;
	float cpsy = cp * sy;

	x = (sr * cpcy - cr * spsy);
	y = (cr * spcy + sr * cpsy);
	z = (cr * cpsy - sr * spcy);
	W = (cr * cpcy + sr * spsy);
}

//////////////////////////////////////////////////////////////////////////
//Matrix4x4f
Matrix4x4f::Matrix4x4f()
{
	r0.SetZero();
	r1.SetZero();
	r2.SetZero();
	r3.SetZero();
}

Matrix4x4f::Matrix4x4f(float tmp)
{
	r0.Set(tmp);
	r1.Set(tmp);
	r2.Set(tmp);
	r3.Set(tmp);
}

Matrix4x4f::~Matrix4x4f()
{

}

void Matrix4x4f::LoadIdentity()
{
	r0[0] = 1.0f;
	r1[1] = 1.0f;
	r2[2] = 1.0f;
	r3[3] = 1.0f;
}

void Matrix4x4f::SetZero()
{
	r0.SetZero();
	r1.SetZero();
	r2.SetZero();
	r3.SetZero();
}

Vector4f& Matrix4x4f::operator [](int i)  
{
	switch (i) {
	case 0 :	return r0;
	case 1 :	return r1;
	case 2 :	return r2;
	case 3 :	return r3;
	}
}


Matrix4x4f& Matrix4x4f::operator+= (Matrix4x4f& tmp)
{
	r0 += tmp.r0;
	r1 += tmp.r1;
	r2 += tmp.r2;
	r3 += tmp.r3;
	return *this;
}

Matrix4x4f& Matrix4x4f::operator/= (float tmp)
{
	r0 /= tmp;
	r1 /= tmp;
	r2 /= tmp;
	r3 /= tmp;
	return *this;
}

Matrix4x4f Matrix4x4f::operator*(float tmp)
{
	Matrix4x4f res = Matrix4x4f();
	res[0] = r0*tmp;
	res[1] = r1*tmp;
	res[2] = r2*tmp;
	res[3] = r3*tmp;
	return res;
}

Matrix4x4f & Matrix4x4f::transpose()
{
	for (int i = 1; i < 4; i++)
		for (int j = 0; j < i; j++)
			swap((*this)[i][j], (*this)[j][i]);
	return *this;
}

Matrix4x4f Matrix4x4f::operator*(Matrix4x4f mat)
{
	mat.transpose();
	Matrix4x4f res = Matrix4x4f();
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
			res[i][j] = Vector4f::Dot3f((*this)[i], mat[j]);
	return res;
}

Vector4f Matrix4x4f::operator*(Vector4f v4f)
{
	Vector4f res = Vector4f();
	for (int i = 0; i < 3; i++)
		res[i] = Vector4f::Dot3f((*this)[i], v4f);
	return res;
}

Matrix4x4f Matrix4x4f::operator+(Matrix4x4f mat)
{
	Matrix4x4f res = Matrix4x4f();
	for (int i = 0; i < 4; i++)
		res[i] = (*this)[i] + mat[i];
	return res;
}


void Matrix4x4f::InverseRotate(Matrix4x4f& mat, Vector4f& point)
{
	float _x = point.x - mat[3][0];
	float _y = point.y - mat[3][1];
	float _z = point.z - mat[3][2];

	point.x = (mat[0][0] * _x) + (mat[0][1] * _y) + (mat[0][2] * _z);
	point.y = (mat[1][0] * _x) + (mat[1][1] * _y) + (mat[1][2] * _z);
	point.z = (mat[2][0] * _x) + (mat[2][1] * _y) + (mat[2][2] * _z);
}

void Matrix4x4f::Rotate(Matrix4x4f& mat, Vector4f& point)
{
	Vector4f oldPos(point.x, point.y, point.z);
	point.x = (mat[0][0] * oldPos.x) + (mat[1][0] * oldPos.y) + (mat[2][0] * oldPos.z);
	point.y = (mat[0][1] * oldPos.x) + (mat[1][1] * oldPos.y) + (mat[2][1] * oldPos.z);
	point.z = (mat[0][2] * oldPos.x) + (mat[1][2] * oldPos.y) + (mat[2][2] * oldPos.z);
}

void Matrix4x4f::Transform(Matrix4x4f& mat, Vector4f& point)
{
	Vector4f oldPos(point.x, point.y, point.z);
	point.x = (mat[0][0] * oldPos.x) + (mat[1][0] * oldPos.y) + (mat[2][0] * oldPos.z) + mat[3][0];
	point.y = (mat[0][1] * oldPos.x) + (mat[1][1] * oldPos.y) + (mat[2][1] * oldPos.z) + mat[3][1];
	point.z = (mat[0][2] * oldPos.x) + (mat[1][2] * oldPos.y) + (mat[2][2] * oldPos.z) + mat[3][2];
}

void Matrix4x4f::QuaternionToMatrix(const Quaternion4f& quat, Matrix4x4f& mat)
{
	float xx = quat.x * quat.x;
	float yy = quat.y * quat.y;
	float zz = quat.z * quat.z;
	float xy = quat.x * quat.y;
	float xz = quat.x * quat.z;
	float yz = quat.y * quat.z;
	float wx = quat.W * quat.x;
	float wy = quat.W * quat.y;
	float wz = quat.W * quat.z;

	mat[0][0] = 1 - 2*(yy + zz);
	mat[1][0] =		2*(xy - wz);
	mat[2][0] =		2*(xz + wy);

	mat[0][1] =		2*(xy + wz);
	mat[1][1] = 1 - 2*(xx + zz);
	mat[2][1] =		2*(yz - wx);

	mat[0][2] =		2*(xz - wy);
	mat[1][2] =		2*(yz + wx);
	mat[2][2] = 1 - 2*(xx + yy);

	mat[3][3] = 1.0f;
}

void Matrix4x4f::CalculateMatrixFromEuler(float ax, float ay, float az, Matrix4x4f& mat)
{
	Quaternion4f quat;
	quat.SetBasedOnEuler(ax, ay, az);
	QuaternionToMatrix(quat, mat);
}


void Matrix4x4f::Multiply(Matrix4x4f& res, Vector4f& v1, Vector4f& v2)
{
	for (unsigned int row = 0; row < 4; ++row)
		for (unsigned int col = 0; col < 4; ++col)
			res[row][col] = v1[row] * v2[col];
}

Vector4f CreatPlane(Vector4f v1, Vector4f v2, Vector4f v3)
{
	Vector4f plane;
	plane = Vector4f::Cross3f(v2 - v1, v3 - v1);
	plane.w = -(plane.x*v1.x + plane.y*v1.y + plane.z*v1.z);
	return plane;
}

//复数加运算
COMPLEX Add(COMPLEX c1, COMPLEX c2)
{
	COMPLEX c;
	c.re = c1.re + c2.re;
	c.im = c1.im + c2.im;
	return c;
}

COMPLEX operator+(COMPLEX c1, COMPLEX c2) 
{
	return COMPLEX(c1.re + c2.re, c1.im + c2.im);
}

//复数乘运算
COMPLEX Mul(COMPLEX c1, COMPLEX c2)
{
	COMPLEX c;
	c.re = c1.re*c2.re - c1.im*c2.im;
	c.im = c1.re*c2.im + c2.re*c1.im;
	return c;
}

COMPLEX operator*(double d, COMPLEX c)
{
	return COMPLEX(c.re*d, c.im*d);
}

//复数减运算
COMPLEX Sub(COMPLEX c1, COMPLEX c2)
{
	COMPLEX c;
	c.re = c1.re - c2.re;
	c.im = c1.im - c2.im;
	return c;
}

COMPLEX operator-(COMPLEX c1, COMPLEX c2)
{
	return Sub(c1, c2);
}

//快速傅里叶变换
void FFT(COMPLEX *TD, COMPLEX *FD, int power)
{
	int count;
	int i, j, k, bfsize, p;
	double angle;
	COMPLEX *W, *X1, *X2, *X;

	count = 1 << power;

	W = (COMPLEX *)malloc(sizeof(COMPLEX)*count);
	X1 = (COMPLEX *)malloc(sizeof(COMPLEX)*count);
	X2 = (COMPLEX *)malloc(sizeof(COMPLEX)*count);

	for (i = 0; i<count / 2; i++)
	{
		angle = -i*pi * 2 / count;
		W[i].re = cos(angle);
		W[i].im = sin(angle);
	}

	memcpy(X1, TD, sizeof(COMPLEX)*count);

	for (k = 0; k<power; k++)
	{
		for (j = 0; j<1 << k; j++)
		{
			bfsize = 1 << (power - k);
			for (i = 0; i<bfsize / 2; i++)
			{
				p = j*bfsize;
				X2[i + p] = Add(X1[i + p], X1[i + p + bfsize / 2]);
				X2[i + p + bfsize / 2] = Mul(Sub(X1[i + p], X1[i + p + bfsize / 2]), W[i*(1 << k)]);
			}
		}
		X = X1;
		X1 = X2;
		X2 = X;
	}

	for (j = 0; j<count; j++)
	{
		p = 0;
		for (i = 0; i<power; i++)
		{
			if (j&(1 << i))
				p += 1 << (power - i - 1);
		}
		FD[j] = X1[p];
	}

	free(W);
	free(X1);
	free(X2);
}

//逆变换
void IFFT(COMPLEX *FD, COMPLEX *TD, int power)
{
	int i, count;
	COMPLEX *x;

	count = 1 << power;
	x = (COMPLEX *)malloc(sizeof(COMPLEX)*count);
	memcpy(x, FD, sizeof(COMPLEX)*count);

	for (i = 0; i<count; i++)
	{
		x[i].im = -x[i].im;
	}
	FFT(x, TD, power);

	for (i = 0; i<count; i++)
	{
		TD[i].re /= count;
		TD[i].im = -TD[i].im / count;
	}
	free(x);
}

double DirectBeam(Vector4f drct) {
	return 1.;
}