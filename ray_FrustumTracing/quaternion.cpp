#include "quaternion.h"
#include <math.h>
#include <iostream>
using namespace std;
/**
Konstruktor bezparametrowy, tworzy kwaternion reprezentujący brak obrotu*/
Quaternion::Quaternion():x(0), y(0), z(0), w(1)
{

}
/**
Konstruktor parametrowy, inicjalizuje atrybuty klasy wartościami podanymi w parametrach
@param axisX - oś x
@param axisY - os Y
@param axisZ - oś Z
@param angle - kąt w radianach*/
Quaternion::Quaternion(float axisX, float axisY, float axisZ, float angle)
{
   float sinus = sinf(angle*0.5f);
   x = sinus*axisX;
   y = sinus*axisY;
   z = sinus*axisZ;
   w = cosf(angle*0.5f);
}
/**
Konstruktor kopiujący*/
Quaternion::Quaternion(const Quaternion &q)
{
   x = q.x;
   y = q.y;
   z = q.z;
   w = q.w;
}
/**
Operator przypisania
@param q - stała referencja do obiektu kwaterniona, którego wartości mają zostać przepisane
@return Quaternion - referencja do obecnego obiektu*/
Quaternion& Quaternion::operator=(const Quaternion &q)
{
   x = q.x;
   y = q.y;
   z = q.z;
   w = q.w;
   return *this;
}
/**
Operator mnożenia kwaternionów
@param q - stała referencja do obiektu przez który będzie wykonane mnożenie*/
Quaternion Quaternion::operator*(const Quaternion &q)
{
   Quaternion out;
   out.x = w*q.x + x*q.w + y*q.z - z*q.y;
   out.y = w*q.y + y*q.w + z*q.x - x*q.z;
   out.z = w*q.z + z*q.w + x*q.y - y*q.x;
   out.w = w*q.w - x*q.x - y*q.y - z*q.z;
   return out;
}
/**
Tworzy kwaternion odwrotny
@return Quaternion - kwaternion odwrotny do obecnego*/
Quaternion Quaternion::conjugate()
{
   Quaternion tmp;
   float NormRc = 1.0f / sqrtf(x*x + y*y + z*z + w*w);
   tmp.w =  w * NormRc;
   tmp.x = (-1.0f)* x * NormRc;
   tmp.y = (-1.0f)* y * NormRc;
   tmp.z = (-1.0f)* z * NormRc;
   return tmp;
}
