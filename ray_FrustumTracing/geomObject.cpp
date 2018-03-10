#include "geomObject.h"
#include <cstring>
#include <iostream>
using namespace std;
/**
   Konstruktor bezparametrowy, zeruje wszystkie pola klasy*/
GeomObject::GeomObject():faceCount(0), vertices(NULL), texcoords(NULL), normals(NULL), lastFrame(0), showListID(0), parent(NULL)
{

}
/**
   Destruktor zwalnia zajętą pamięć*/
GeomObject::~GeomObject()
{
   free();
}
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
   @param texture - tekstura obiektu
   @param aInfo - struktura przechowująca informacje o animacji*/
void GeomObject::set(int faceC,  Vector3f* vert, Vector2f* texc, Vector3f* norm,  Vector3f pos, string tmpName, GeomObject *p, Vector3f axis, float angle)
{
   free();
   faceCount = faceC;
   vertices = NULL;
   texcoords = NULL;
   normals = NULL;
   initPos = pos;
   initRotAxis = axis;
   initAngle = angle;
   name = tmpName;
   parent = p;
   // zarezerwowanie pamięci na tablice oraz przekopiowanie zawartości z tablic podanych jako argumenty
   if ( vert != NULL) {
      vertices = new Vector3f[faceCount*3];
//       for (int i = 0; i < faceCount * 3; i++)
//         vertices[i] = vert[i];
      memcpy(vertices, vert, sizeof(Vector3f) * 3 * faceCount);
   }

   if (texc != NULL) {
      texcoords = new Vector2f[faceCount*3];
//      for (int i =0; i < faceCount * 3; i++ )
//         texcoords[i] = texc[i];
      memcpy(texcoords, texc, sizeof(Vector2f) * 3 * faceCount);
   }
   if (norm != NULL) {
      normals = new Vector3f[faceCount];
//      for (int i = 0; i < faceCount; i++)
//         normals[i] = norm[i];
      memcpy(normals, norm, sizeof(Vector3f) * faceCount);
   }
}

void GeomObject::free()
{
   if (vertices !=NULL){
      delete [] vertices;
      vertices = NULL;
   }

   if (texcoords != NULL){
      delete [] texcoords;
      texcoords = NULL;
   }

   if (normals != NULL){
      delete [] normals;
      normals = NULL;
   }
}
