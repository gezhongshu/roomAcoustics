#include <cstring>
#include <sstream>
#include <stdlib.h>
#include <fstream>
#include <cassert>

#include "mesh.h"
#include "quaternion.h"
#include <iostream>
using namespace std;


/**
   Konstruktor bezparametrowy, inicjalizuje wszystkie pola klasy na wartości zerowe (0 lub NULL)*/
Mesh::Mesh():objects(NULL), geomCount(0)
{

}
/**
   Konstruktor, jako parametry przyjmuje nazwę pliku, z którego ma wczytać informajce o obiekcie oraz flagę określającą czy normalne są zdefiniowane dla każdego wierzchołka
   @param texturePath - ścieżka do pliku .bmp zawierającego teksture
   @param fileName - ścieżka do folderu zawierającego pliki obiektu*/
Mesh::Mesh(string id, string fileName, vector<int>& vertId, bool alpha): objects(NULL), name(id)
{
   //loadObject(fileName, alpha);
   //loadAnimation(fileName);
	//loadVert(fileName, vertId);
	loadObj(fileName, vertId);
}
Mesh::Mesh(string id, string fileName, vector<int>& vertId, vector<int>& matId, bool alpha) : objects(NULL), name(id)
{
	loadObj(fileName, vertId, matId);
}
/**
   Konstruktor (pseudo)kopiujący. Nie kopiuje nic, ustawia wszystkie pola na zero - nie wolno kopiować mesha!*/
Mesh::Mesh(const Mesh &mesh):objects(NULL), geomCount(0)
{

}
/**
   Destruktor zwalnia zarezerwowaną pamięć*/
Mesh::~Mesh()
{
   free();
}
/**
   Load mesh from simple text list of vertices and faces.
*/
void Mesh::loadVert(string fileName, vector<int>& vertId)
{
	ifstream finModel((fileName + "\\mesh.vrt").c_str(), ios_base::in);
	if (!finModel)return;//Quit if not found.
	float point[3];
	vector<Vector3f> vertices, faceVerts;
	Vector2f *texcoords = NULL;
	Vector3f *normals = NULL;
	int geomobjectsCount = 1;
	int materialNo = 0, index = 0;
	int vertNum = 0, faceNum = 0;
	bool counted = false;
	Vector3f tmpPos, tmpRot;
	float tmpRotAngle = 0;
	string name;
	GeomObject *parent = NULL;
	free();
	objects = new GeomObject[geomobjectsCount];
	geomCount = geomobjectsCount;

	char ch;
	finModel >> ch;
	assert(ch == '[');
	index = 0;
	do
	{
		finModel >> point[0] >> point[1] >> point[2] >> ch;
		vertices.push_back(Vector3f(point[0], point[1], point[2]));
		vertNum++;
	} while (ch == ',');
	assert(ch == ']');

	finModel >> ch;
	assert(ch == '[');
	do
	{
		int vIndex, ind = 0;
		do
		{
			finModel >> vIndex >> ch;
			if (vIndex >= 0 && ind < 3 && faceNum % 1700 >= 1600)
			{
				faceVerts.push_back(vertices[vIndex]);
				vertId.push_back(vIndex);
			}
			ind++;
		} while (vIndex >= 0);
		faceNum++;
	} while (ch == ',');
	assert(ch == ']');
	
	objects[geomobjectsCount - 1].set(faceVerts.size()/3, &faceVerts[0], texcoords, normals, tmpPos, name, parent, tmpRot, tmpRotAngle);

	finModel.close();
}
/**
Load mesh from simple text list of vertices and faces.
*/
void Mesh::loadObj(string fileName, vector<int>& vertId)
{
	ifstream finModel(fileName.c_str(), ios_base::in);
	if (!finModel)return;//Quit if not found.
	vector<float> point;
	vector<Vector3f> vertices, faceVerts;
	Vector2f *texcoords = NULL;
	Vector3f *normals = NULL;
	int geomobjectsCount = 1;
	int materialNo = 0, index = 0;
	int vertNum = 0, faceNum = 0;
	bool counted = false;
	Vector3f tmpPos, tmpRot;
	float tmpRotAngle = 0;
	string name;
	GeomObject *parent = NULL;
	free();
	objects = new GeomObject[geomobjectsCount];
	geomCount = geomobjectsCount;

	string line;
	index = 0;
	int skip = -1;
	while(getline(finModel,line))
	{
		if (line[0] == 'v')
		{
			point = getNumbers(line);
			vertices.push_back(Vector3f(point[0] / 1e3, point[1] / 1e3, point[2] / 1e3));
			vertNum++;
		}
		if (line[0] == 'f')
		{
			skip++;
			if (skip % 1)continue;
			int vIndex;
			point = getNumbers(line);
			for (int i = 0; i < 3; i++)
			{
				vIndex = (int)point[i];
				faceVerts.push_back(vertices[vIndex-1]);
				vertId.push_back(vIndex - 1);
			}
			faceNum++;
		}
	}

	objects[geomobjectsCount - 1].set(faceVerts.size() / 3, &faceVerts[0], texcoords, normals, tmpPos, name, parent, tmpRot, tmpRotAngle);

	finModel.close();
}

void Mesh::loadObj(string fileName, vector<int>& vertId, vector<int>& matId)
{
	ifstream finModel(fileName.c_str(), ios_base::in);
	if (!finModel)return;//Quit if not found.
	vector<float> point;
	vector<Vector3f> vertices, faceVerts;
	Vector2f *texcoords = NULL;
	Vector3f *normals = NULL;
	int geomobjectsCount = 1;
	int materialNo = 0, index = 0;
	int vertNum = 0, faceNum = 0;
	bool counted = false;
	Vector3f tmpPos, tmpRot;
	float tmpRotAngle = 0;
	string name;
	GeomObject *parent = NULL;
	free();
	objects = new GeomObject[geomobjectsCount];
	geomCount = geomobjectsCount;

	string line;
	vector<string> matNames = WallAirAbsorb::GetMatName();

	index = 0;
	int faceId = -1, matInd = -1;
	while (getline(finModel, line))
	{
		if (line.size() >= 6 && line[0] == 'u' && line.substr(0, 6) == string("usemtl"))
			matInd = strMatch(line.substr(7), WallAirAbsorb::GetMatName());
		
		if (line[0] == 'v' && line[1] == ' ')
		{
			point = getNumbers(line);
			vertices.push_back(Vector3f(point[0] / 1e3, point[1] / 1e3, point[2] / 1e3));
			vertNum++;
			//cout << "vert " << vertNum << '\t';
			//cout << point[0] << '\t' << point[1] << '\t' << point[2] << endl;
		}
		if (matInd < 0)continue;
		if (line[0] == 'f')
		{
			faceId++;
			if (faceId % 1)continue;
			int vIndex;
			point = getNumbers(line);
			for (int i = 0; i < 3; i++)
			{
				vIndex = (int)point[i];
				faceVerts.push_back(vertices[vIndex - 1]);
				vertId.push_back(vIndex - 1);
			}
			faceNum++;
			//cout << "\nface " << faceNum << '\t';
			//cout << (int)point[0] << '\t' << (int)point[1] << '\t' << (int)point[2] << '\t' << matInd << endl;
			matId.push_back(matInd);
		}
	}
	//system("pause");
	objects[geomobjectsCount - 1].set(faceVerts.size() / 3, &faceVerts[0], texcoords, normals, tmpPos, name, parent, tmpRot, tmpRotAngle);

	finModel.close();
}

/**
   Zwalnia pamięć*/
void Mesh::free()
{

   if(objects != NULL){
      for (int i = 0; i < geomCount; ++i)
         objects[i].free();
      delete [] objects;
      objects = NULL;
   }

}

void Mesh::GetAllVertex(float** destTab, int &vertCount)
{
	int offset = 0;
	int allFloatsCount = 0;
	int floatsCount = 0;
	const size_t size = geomCount;
	for (unsigned int i = 0; i < size; ++i)
		allFloatsCount += objects[i].faceCount*9;

	vertCount = allFloatsCount/3;
	(*destTab) = new float[allFloatsCount];

	for (unsigned int i = 0; i < size; ++i)
	{
		floatsCount = objects[i].faceCount*9;
		memcpy((*destTab) + offset, objects[i].vertices, sizeof(Vector3f)*objects[i].faceCount*3);
		offset += floatsCount;
	}
}