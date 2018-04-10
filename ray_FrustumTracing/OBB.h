#ifndef OBB_H
#define OBB_H

#include <cmath>
#include <vector>

#include "mathdefs.h"

struct faceInfo
{
	int id;
	int m_id;
	Vector4f center;
	float area;
	Vector4f centDA;
	Vector4f faceNorm;
	vector<Vector4f> verts;
	vector<int> vertsId;
	faceInfo(int i, Vector4f c, float a) : id(i), m_id(0), center(c), area(a), centDA(c / a) {}
	faceInfo(int i, int m_id, Vector4f c, float a) : id(i), m_id(m_id), center(c), area(a), centDA(c / a) {}
	faceInfo() :id(-1), m_id(0), center(Vector4f(0)), area(0), centDA(Vector4f(0)) {}
};

struct Edge
{
	int vertId, faceId[2];
	Edge() :vertId(-1) { faceId[0] = -1, faceId[1] = -1; };
	Edge(int v, int f, int i) :vertId(v) { faceId[i] = f; };
};

class Mtllib
{
public:
	Mtllib();
	~Mtllib();
	static float GetScatter(int m_id);
	static vector<float> GetRef(int m_id);
	static void LoadMaterials();

private:
	static vector<float> scatters;
	static vector<vector<float>> refs;
};

class OBB
{
	friend class OBBTree;
	friend class OBBTreeNode;

public:
	Vector4f&	GetCenter		()	{return OBBcenter;}
	Vector4f&	GetSides		()	{return OBBsides;}
	Vector4f*	GetPoints		()	{return points;}
	int			GetVertexCount	()	{return vertCount;}
	double		GetVolume		()	{ return OBBsides.x*OBBsides.y*OBBsides.z; }
	vector<faceInfo*> GetFaces() { return faceLink; }
	bool IsEnd() { return isEnd; }
	void AddFace(faceInfo* face) { faceLink.push_back(face); }
	void SetEnd() { isEnd = true; }

private:
	OBB();
	OBB(float* tmpVert, int tmpCount, float tmpScale = 1.0f);
	~OBB();

	bool		ComputeBox	 ();
	void		ComputeDivisionPlane(Vector4f& plane);
	void		TestDivDeg(float& divDeg, Vector4f& pl, Vector4f& plane);
	void		Move(Vector4f disp);

	void		ComputeMatrix();
	void		ComputeAABB	 (Vector4f &bMin, Vector4f &bMax);
	void		ComputeOBB	 (Vector4f& sides, Matrix4x4f& mat);
	void		ComputePoints();

	Vector4f	OBBcenter;
	Vector4f	OBBsides;
	Matrix4x4f	OBBmatrix;

	//tablica z wierzcho³kami siatki (x, y, z - po kolei)
	float*		vertices;
	//iloœæ wierzcho³ków
	int			vertCount;
	//iloœæ elementów tablicy = 3*vertCount
	int			tabSize;
	//ewentualne skalowanie w formie wektora
	Vector4f	scale4f;
	//ewentualne skalowanie
	float		scale;
	//wierzcho³ki bry³y
	Vector4f	points[8];
	vector<faceInfo*> faceLink;
	bool isEnd;
};

#endif