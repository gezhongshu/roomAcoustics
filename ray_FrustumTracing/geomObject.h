#ifndef GEOMOBJECT_H_INCLUDED
#define GEOMOBJECT_H_INCLUDED
#include <string>
#include "define.h"
#include <vector>
#include "mathdefs.h"
using namespace std;

class GeomObject {
public:
/**
   Liczba powierzchni obiektu (trójkątów), liczba wierzchołków jest równa liczb powierzchni przemnożonej przez 3*/
   int faceCount;
/**
   tablica z wektorami*/
	Vector3f *vertices;
/**
   tablica z wpsółrzędnymi tekstur*/
	Vector2f *texcoords;
/**
   tablica normalnych*/
	Vector3f *normals;
/**
   Przesunięcie obiektu względem początku układu współrzędnych*/
   Vector3f initPos;
/**
   Oś obrotu obiektu*/
   Vector3f initRotAxis;
/**
   Początkowy obrót obiektu*/
   float initAngle;
/**
   Ilość klatek animacji*/
	unsigned int lastFrame;
/**
   identyfikator listy wyświetlania*/
   int showListID;
/**
   Wskaźnik na obiekt rodzica*/
   GeomObject *parent;
/**
   Nazwa obiektu, unikalna w obrębie jednego mesha*/
   string name;
public:
/**
   Konstruktor bezparametrowy, zeruje wszystkie pola klasy*/
   GeomObject();
/**
   Destruktor zwalnia zajętą pamięć*/
   ~GeomObject();
/**
   Metoda ustawia pola klasy na wartości przekazane jako argumenty, dodatkowo tworzy listę wyświetlania
   @param faceC - liczba powierzchni
   @param vert - tablica zawierająca wierchołki, zostanie skopiowana
   @param texc - tablica zawierająca współrzędne tekstur, zostanie skopiowana
   @param norm - tablica zawierająca normalne dla powierzchni, zostanie skopiowana
   @param pos - inicjalna pozycja obiektu
   @param tmpName - nazwa obiektu
   @param p - wskaźnik na obiekt rodzica
   @param axis - oś obrotu
   @param angle - kąt obrotu
   @param texture - tekstura obiektu*/
   void set(int faceC,  Vector3f* vert, Vector2f* texc, Vector3f* norm, Vector3f pos, string tmpName, GeomObject *p, Vector3f axis, float angle);

/**
   Metoda zwalnia pamięć zajętą przez obiekt*/
   void free();
};

#endif // GEOMOBJECT_H_INCLUDED
