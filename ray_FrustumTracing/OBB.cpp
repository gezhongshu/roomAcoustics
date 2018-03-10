#include "OBB.h"

vector<float> Mtllib::scatters = vector<float>(1, 1.f);
vector<vector<float>> Mtllib::refs = vector<vector<float>>(1, vector<float>(7, 0.8));

Mtllib::Mtllib()
{
}

Mtllib::~Mtllib()
{
}

float Mtllib::GetScatter(int m_id)
{
	return scatters[m_id];
}

vector<float> Mtllib::GetRef(int m_id)
{
	return refs[m_id];
}

void Mtllib::LoadMaterials()
{
	/*Mtllib::scatters = vector<float>(1, 1.f);
	Mtllib::refs = vector<vector<float>>(1, vector<float>(7, 0.8));*/
}

OBB::OBB(float* tmpVert, int tmpCount, float tmpScale)
{
	vertices = tmpVert;		//kolejne elementy to x, y, z itd
	scale = tmpScale;		//scalowanie
	vertCount = tmpCount;	//iloœæ wierzoch³ów a nie pozycji w tablicy!!
	tabSize = vertCount*3;	//tu mamy wielkoœc tablicy
	scale4f.Set(scale);
	isEnd = false;
}

OBB::OBB()
{
	vertices = NULL;
	scale = 0.0f;
	vertCount = 0;
	tabSize = 0;
	isEnd = false;
	OBBcenter.SetZero();
	scale4f.SetZero();
};

OBB::~OBB()
{
	delete[] vertices;
	vertices = NULL;
}


void OBB::ComputeAABB(Vector4f &bMin, Vector4f &bMax)
{
	bMin.x = vertices[0];//*scale;
	bMin.y = vertices[1];//*scale;
	bMin.z = vertices[2];//*scale;

	bMax.x = vertices[0];//*scale;
	bMax.y = vertices[1];//*scale;
	bMax.z = vertices[2];//*scale;

	const size_t size = tabSize;
	for (unsigned int i = 0; i < size; i+=3)
	{
		if (vertices[i]*scale < bMin.x)		bMin.x = vertices[i];//*scale;
		if (vertices[i+1]*scale < bMin.y)	bMin.y = vertices[i+1];//*scale;
		if (vertices[i+2]*scale < bMin.z)	bMin.z = vertices[i+2];//*scale;
		
		if (vertices[i]*scale > bMax.x)		bMax.x = vertices[i];//*scale;
		if (vertices[i+1]*scale > bMax.y)	bMax.y = vertices[i+1];//*scale;
		if (vertices[i+2]*scale > bMax.z)	bMax.z = vertices[i+2];//*scale;
	}
	/*bMin.x = bMin.x*scale;
	bMin.y = bMin.y*scale;
	bMin.z = bMin.z*scale;

	bMax.x = bMax.x*scale;
	bMax.y = bMax.y*scale;
	bMax.z = bMax.z*scale;*/
}

void OBB::ComputeOBB(Vector4f& sides, Matrix4x4f& mat)
{
	Vector4f bMax(-1e9, -1e9, -1e9);
	Vector4f bMin(1e9, 1e9, 1e9);

	Vector4f testPoint;
	const size_t size = tabSize;
	for (unsigned int i = 0; i < size; i+=3)
	{
		testPoint.SetZero();
		testPoint.Set(vertices[i]/**scale*/, vertices[i+1]/**scale*/, vertices[i+2]/**scale*/);
		Matrix4x4f::InverseRotate(mat, testPoint);

		if (testPoint.x < bMin.x)	bMin.x = testPoint.x;
		if (testPoint.y < bMin.y)	bMin.y = testPoint.y;
		if (testPoint.z < bMin.z)	bMin.z = testPoint.z;

		if (testPoint.x > bMax.x)	bMax.x = testPoint.x;
		if (testPoint.y > bMax.y)	bMax.y = testPoint.y;
		if (testPoint.z > bMax.z)	bMax.z = testPoint.z;
	}

	Vector4f cnt;
	sides.x = bMax.x - bMin.x;
	sides.y = bMax.y - bMin.y;
	sides.z = bMax.z - bMin.z;

	cnt.x = sides.x*0.5f + bMin.x;
	cnt.y = sides.y*0.5f + bMin.y;
	cnt.z = sides.z*0.5f + bMin.z;

	Matrix4x4f::Rotate(mat, cnt);
	mat[3][0] += cnt.x;
	mat[3][1] += cnt.y;
	mat[3][2] += cnt.z;
}


void OBB::ComputeMatrix()
{
	//tutaj znajduje si?wynik obliczania rozmiarów obbka
	Vector4f sides;
	Matrix4x4f OBBmat;

	//minimalne wartoœæi dla AABB
	Vector4f bMin;
	//maksymalne wartoœci dla AABB
	Vector4f bMax;	

	//obliczamy AABB
	ComputeAABB(bMin, bMax);
	//œrodek
	Vector4f AABBcenter;	

	AABBcenter.x = (bMax.x - bMin.x)*0.5f + bMin.x;
	AABBcenter.y = (bMax.y - bMin.y)*0.5f + bMin.y;
	AABBcenter.z = (bMax.z - bMin.z)*0.5f + bMin.z;

	//osie dla obbków
	float ax = 0.0f;
	float ay = 0.0f;
	float az = 0.0f;

	//zasiêg odchylania po wszystkich osiach
	float range = 45.0f;
	//iloœæ kroków odchylania
	unsigned int steps = 7;
	//najlepsza objêtoœæ w trakcie wyszukiwania 
	float bestVolume = 1e9;

	Vector4f angle;

	while (range >= 1)
	{
		bool found = false;
		float stepSize = range / steps;
		for (float x = ax - range; x <= ax + range; x += stepSize)
		{
			for (float y = ay - range; y <= ay + range; y += stepSize)
			{
				for (float z = az - range; z <= az + range; z += stepSize)
				{
					Matrix4x4f testMatrix;
					Matrix4x4f::CalculateMatrixFromEuler(x*DEG_TO_RAD, y*DEG_TO_RAD, z*DEG_TO_RAD, testMatrix);
					testMatrix[3][0] = AABBcenter[0];
					testMatrix[3][1] = AABBcenter[1];
					testMatrix[3][2] = AABBcenter[2];

					Vector4f testSides;
					ComputeOBB(testSides, testMatrix);
					float volume = testSides.x * testSides.y * testSides.z;

					if (volume < bestVolume)
					{
						bestVolume = volume;
						sides.x = testSides.x;
						sides.y = testSides.y;
						sides.z = testSides.z;

						angle.x = ax;
						angle.y = ay;
						angle.z = az;
						OBBmat = testMatrix;
						found = true;
					}
				}
			}
		}
		if (found)
		{
			ax = angle.x;
			ay = angle.y;
			az = angle.z;
			range *= 0.5f;
		}
		else
		{
			break;
		}
	}
	OBBmatrix = OBBmat;
	OBBsides = sides*scale;
	OBBcenter = OBBmatrix[3]*scale;
}

bool OBB::ComputeBox()
{
	//obliczamy macierz naszej bry³y oraz jej rozmiary
	ComputeMatrix();
	//obliczamy wierzcho³ki naszej bry³y
	ComputePoints();

	return true;
}

void OBB::ComputePoints()
{
	Vector4f sideX(OBBsides.x/2.0f, 0.0f, 0.0f);
	Vector4f sideY(0.0f, OBBsides.y/2.0f, 0.0f);
	Vector4f sideZ(0.0f, 0.0f, OBBsides.z/2.0f);

	points[0] = OBBcenter + sideX + sideY + sideZ;
	points[1] = OBBcenter + sideX + sideY - sideZ;
	points[2] = OBBcenter + sideX - sideY - sideZ;
	points[3] = OBBcenter - sideX - sideY - sideZ;

	points[4] = OBBcenter + sideX - sideY + sideZ;
	points[5] = OBBcenter - sideX + sideY + sideZ;
	points[6] = OBBcenter - sideX - sideY + sideZ;
	points[7] = OBBcenter - sideX + sideY - sideZ;

	for (unsigned int i = 0; i < 8; ++i)
	{
		points[i] -= OBBcenter;
		Matrix4x4f::Rotate(OBBmatrix, points[i]);
		points[i] += OBBcenter;
	}
}


void OBB::Move(Vector4f disp)
{
	OBBcenter += disp; 
	OBBmatrix[3] += disp;
	ComputePoints();
}


void OBB::ComputeDivisionPlane(Vector4f& plane)
{
	float divideDegree = INFINITY;
	if (OBBsides[2] > EPS)
	{
		Vector4f pl = points[0] - points[1];
		pl.w = -(OBBcenter.x*pl.x + OBBcenter.y*pl.y + OBBcenter.z*pl.z);
		TestDivDeg(divideDegree, pl, plane);
	}
	if (OBBsides[0] > EPS&&OBBsides[1] > EPS)
	{
		TestDivDeg(divideDegree, CreatPlane(points[1], points[3], 
			points[1] + Vector4f::Cross3f(points[3]-points[1],points[7]-points[2])), plane);
		TestDivDeg(divideDegree, CreatPlane(points[2], points[7],
			points[2] + Vector4f::Cross3f(points[7] - points[2], points[3] - points[1])), plane);
	}
	if (OBBsides[0] > EPS&&OBBsides[2] > EPS)
	{
		TestDivDeg(divideDegree, CreatPlane(points[1], points[5],
			points[1] + Vector4f::Cross3f(points[5] - points[1], points[7] - points[0])), plane);
		TestDivDeg(divideDegree, CreatPlane(points[0], points[7],
			points[0] + Vector4f::Cross3f(points[7] - points[0], points[5] - points[1])), plane);
	}
	if (OBBsides[2] > EPS&&OBBsides[1] > EPS)
	{
		TestDivDeg(divideDegree, CreatPlane(points[1], points[4],
			points[1] + Vector4f::Cross3f(points[4] - points[1], points[0] - points[2])), plane);
		TestDivDeg(divideDegree, CreatPlane(points[2], points[0],
			points[2] + Vector4f::Cross3f(points[0] - points[2], points[4] - points[1])), plane);
	}
	if (OBBsides[0] > EPS)
	{
		Vector4f pl = points[3] - points[2];
		pl.w = -(OBBcenter.x*pl.x + OBBcenter.y*pl.y + OBBcenter.z*pl.z);
		TestDivDeg(divideDegree, pl, plane);
	}
	if (OBBsides[1] > EPS)
	{
		Vector4f pl = points[2] - points[1];
		pl.w = -(OBBcenter.x*pl.x + OBBcenter.y*pl.y + OBBcenter.z*pl.z);
		TestDivDeg(divideDegree, pl, plane);
	}
}

void OBB::TestDivDeg(float & divDeg, Vector4f & pl, Vector4f & plane)
{
	float deg = 0, count = 0;
	for (auto face : faceLink)
	{
		Vector4f pt = face->center*scale;
		if (Vector4f::Dot3f(pt, pl) + pl.w > EPS)
			deg+=1;
		else if (Vector4f::Dot3f(pt, pl) + pl.w < -EPS)
			deg-= 1;
		else
			count+= 1;
	}
	deg = abs(deg) + count;
	if (deg < divDeg)
	{
		divDeg = deg;
		plane = pl;
	}
}



/*
bool OBB::ComputeBox(float* tmpVert, int tmpCount, float tmpScale)
{
	vertices = tmpVert;		//kolejne elementy to x, y, z itd
	scale = tmpScale;		//scalowanie
	vertCount = tmpCount;	//iloœæ wierzoch³ów a nie pozycji w tablicy!!
	tabSize = vertCount*3;	//tu mamy wielkoœc tablicy
	scale4f.Set(scale);

	//obliczanie cener of OBB
	Vector4f sumVect;
	const size_t size = tabSize;
	for(unsigned int i = 0; i < size; i+=3)
	{
		sumVect.x += vertices[i]*scale;
		sumVect.y += vertices[i+1]*scale;
		sumVect.z += vertices[i+2]*scale;
	}
	center = sumVect/vertCount;
	OBBinfo[3] = center;

	//obliczamy macierz
	Vector4f tmpPoint;
	Matrix4x4f tmpMatrix;
	for(unsigned int i = 0; i < size; i+=3)
	{
		tmpPoint.SetZero();
		tmpMatrix.SetZero();
	
		tmpPoint.x = vertices[i]*scale;
		tmpPoint.y = vertices[i+1]*scale;
		tmpPoint.z = vertices[i+2]*scale;

		tmpPoint -= center;
		Matrix4x4f::Multiply(tmpMatrix, tmpPoint, tmpPoint);
		covMatrix += tmpMatrix;
	}
	covMatrix /= (float)vertCount;

	double A[3][3];
	double V[3][3];
	double Z[3];

	Matrix4x4f::MakeDoubleMatrix(A, covMatrix);
	Matrix4x4f::MakeDoubleMatrix(V, Matrix4x4f());

	eigen_decomposition(A, V, Z);
	Matrix4x4f::MakeMatrixFromDouble(OBBinfo, V);
	
	float dot1 = 0.0f;
	float dot2 = 0.0f;
	float dot3 = 0.0f;
	for(unsigned int i = 0; i < size; i+=3)
	{
		dot1 = 0.0f;
		dot2 = 0.0f;
		dot3 = 0.0f;
 		tmpPoint.SetZero();

		tmpPoint.x = vertices[i]*scale;
		tmpPoint.y = vertices[i+1]*scale;
		tmpPoint.z = vertices[i+2]*scale;
		tmpPoint -= center;
		Vector4f::Dot3f(dot1, OBBinfo[0], tmpPoint);
		Vector4f::Dot3f(dot2, OBBinfo[1], tmpPoint);
		Vector4f::Dot3f(dot3, OBBinfo[2], tmpPoint);

		dot1 = abs(dot1);
		dot2 = abs(dot2);
		dot3 = abs(dot3);

		if (dot1 > OBBinfo[0][3])
			OBBinfo[0][3] = dot1;

		if (dot2 > OBBinfo[1][3])
			OBBinfo[1][3] = dot2;

		if (dot3 > OBBinfo[2][3])
			OBBinfo[2][3] = dot3;
	}

	OBBinfo[0].SetLenght(OBBinfo[0][3]);
	OBBinfo[1].SetLenght(OBBinfo[1][3]);
	OBBinfo[2].SetLenght(OBBinfo[2][3]);

	points[0] = OBBinfo[3] + OBBinfo[0] + OBBinfo[1] + OBBinfo[2];
	points[1] = OBBinfo[3] + OBBinfo[0] + OBBinfo[1] - OBBinfo[2];
	points[2] = OBBinfo[3] + OBBinfo[0] - OBBinfo[1] - OBBinfo[2];
	points[3] = OBBinfo[3] - OBBinfo[0] - OBBinfo[1] - OBBinfo[2];

	points[4] = OBBinfo[3] + OBBinfo[0] - OBBinfo[1] + OBBinfo[2];
	points[5] = OBBinfo[3] - OBBinfo[0] + OBBinfo[1] + OBBinfo[2];
	points[6] = OBBinfo[3] - OBBinfo[0] - OBBinfo[1] + OBBinfo[2];
	points[7] = OBBinfo[3] - OBBinfo[0] + OBBinfo[1] - OBBinfo[2];

	return true;
}

*/