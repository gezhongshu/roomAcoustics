#ifndef QUATERNION_H_INCLUDED
#define QUATERNION_H_INCLUDED

#include "mathdefs.h"

/**
   Klasa reprezentująca kwaternion*/
class Quaternion {
public:
   /**
   Wspołrzędne osi (x, y,z) oraz kąt w*/
   float x,y,z,w;
public:
   /**
   Konstruktor bezparametrowy, tworzy kwaternion reprezentujący brak obrotu*/
   Quaternion();
   /**
   Konstruktor parametrowy, inicjalizuje atrybuty klasy wartościami podanymi w parametrach
   @param axisX - oś x
   @param axisY - os Y
   @param axisZ - oś Z
   @param angle - kąt w radianach*/
   Quaternion(float axisX, float axisY, float axisZ, float angle);
   /**
   Konstruktor kopiujący*/
   Quaternion(const Quaternion &q);
   /**
   Konstruktor przyjmujący jako parametr wektor 3D reprezentujący oś obortu i ustawiający kąt na 0
   @param v - stała referencja do obiektu przedstawiającego oś obrotu*/
   Quaternion(const Vector3f &v):x(v.x), y(v.y), z(v.z), w(0.0f) {}
   /**
   Operator przypisania
   @param q - stała referencja do obiektu kwaterniona, którego wartości mają zostać przepisane
   @return Quaternion - referencja do obecnego obiektu*/
   Quaternion& operator=(const Quaternion &q);
   /**
   Operator mnożenia kwaternionów
   @param q - stała referencja do obiektu przez który będzie wykonane mnożenie*/
   Quaternion operator*(const Quaternion &q);

   /**
   Tworzy kwaternion odwrotny
   @return Quaternion - kwaternion odwrotny do obecnego*/
   Quaternion conjugate();
};


#endif // QUATERNION_H_INCLUDED
