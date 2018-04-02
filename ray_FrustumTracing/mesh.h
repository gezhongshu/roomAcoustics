#ifndef MESH_H
#define MESH_H

#include <string>
#include "utils.h"
#include "geomObject.h"
#include "dataBase.h"
//#include "boundingBox.h"
using namespace std;

/**
   Klasa służąca do pobierania  informacji o obiektach 3D z pliku .ASE (a i być może do ich wyświetlania)*/
class Mesh {

private:
/**
   Tablica obiektów geometrycznych, z których składa się wyświetlany obiekt*/
   GeomObject *objects;
/**
   Liczba obiektów geometrycznych*/
   int geomCount;
public:
/**
   Konstruktor bezparametrowy, inicjalizuje wszystkie pola klasy na wartości zerowe (0 lub NULL)*/
   Mesh();
/**
   Konstruktor, jako parametry przyjmuje nazwę pliku, z którego ma wczytać informajce o obiekcie oraz flagę określającą czy normalne są zdefiniowane dla każdego wierzchołka
   @param texturePath - ścieżka do pliku .bmp zawierającego teksture
   @param folderPath - ścieżka do folderu zawierającego pliki obiektu*/
   Mesh(string id, string folderPath, vector<int>& vertId, bool alpha = false);
   Mesh(string id, string folderPath, vector<int>& vertId, vector<int>& matId, bool alpha = false);
/**
   Konstruktor (pseudo)kopiujący. Nie kopiuje nic, ustawia wszystkie pola na zero - nie wolno kopiować mesha!*/
   Mesh(const Mesh &mesh);
/**
   Destruktor zwalnia zarezerwowaną pamięć*/
   ~Mesh();
/**
   Load mesh from simple text list of vertices and faces.
   */
   void loadVert(string fileName, vector<int>& vertId);
   void loadObj(string fileName, vector<int>& vertId);
   void loadObj(string fileName, vector<int>& vertId, vector<int>& matId);
/**
   Zwalnia pamięć*/
   void free();


   void GetAllVertex(float** destTab, int &vertCount);

/**
* Identyfikator obiektu.
*/
string name;
};


#endif
