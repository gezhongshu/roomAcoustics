#include "obbTree.h"

//////////////////////////////////////////////////////////////////////////
//OBBTree
OBBTree::OBBTree():levels(0), tree(NULL), minVertCount(6), edges(100000){}
OBBTree::~OBBTree()
{
	if (tree)	delete tree;
	tree = NULL;
}


bool OBBTree::ComputeOBBTree(TREE_CONDITION TreeCondition, int TreeConParam, float* tmpVert, int tmpCount, vector<int>& vertId, float tmpScale /* = 1.0f */)
{
	condition = TreeCondition;
	conParam = TreeConParam;
	tree = new OBBTreeNode();
	Vector4f *vertices = new Vector4f[tmpCount];
	for (int i = 0; i < tmpCount; i++)
		vertices[i] = Vector4f(tmpVert[3 * i], tmpVert[3 * i + 1], tmpVert[3 * i + 2]);
	vector<faceInfo*> faceLink;
	for (int i = 0; i < tmpCount / 3; i++)
	{
		float area = Vector4f::Cross3f(vertices[3 * i] * tmpScale - vertices[3 * i + 1] * tmpScale,
			vertices[3 * i + 1] * tmpScale - vertices[3 * i + 2] * tmpScale).GetLenght() / 2;
		faces[i] = faceInfo(i, (vertices[3 * i] + vertices[3 * i + 1] + vertices[3 * i + 2]) / 3, area);
		faces[i].verts.push_back(vertices[3 * i]*tmpScale);
		faces[i].vertsId.push_back(vertId[3 * i]);
		faces[i].verts.push_back(vertices[3 * i + 1] * tmpScale);
		faces[i].vertsId.push_back(vertId[3 * i + 1]);
		faces[i].verts.push_back(vertices[3 * i + 2] * tmpScale);
		faces[i].vertsId.push_back(vertId[3 * i + 2]);
		faces[i].faceNorm = Vector4f::Cross3f(Vector4f(vertices[3 * i + 1] - vertices[3 * i]),
											Vector4f(vertices[3 * i + 2] - vertices[3 * i]));
		faces[i].faceNorm.SetLenght(1.0f);
		faces[i].faceNorm.w = -Vector4f::Dot3f(vertices[3 * i] * tmpScale, faces[i].faceNorm);
		faceLink.push_back(&faces[i]);
		vector<int> v = faces[i].vertsId;
		int v1 = v.back();
		for (auto v2 : v)
		{
			ManaEdge(v1, v2, i);
			v1 = v2;
		}
	}

	ManageNode(tree, 0, faceLink, tmpVert, tmpCount, tmpScale);
	return true;
}


void OBBTree::ManageNode(OBBTreeNode* node, int nodeLVL, vector<faceInfo*> &faces, float* tmpVert, int tmpCount, float tmpScale)
{
	node->ComputeBox(tmpVert, tmpCount, tmpScale);
	node->level = nodeLVL+1;
		for (auto face : faces)
			node->box->AddFace(face);
	if (node->level == conParam)
	{
		node->GetBox()->SetEnd();
		return;
	}
	//node->geomFaces = faces;
	
	if (node->level > levels)	levels = node->level;
	if (condition == OBB_DEPTH_CONDITION && conParam <= node->level)
	{
		//koniec imprezy, mamy do滄 g酬boko
	}
	else
	{
		Vector4f plane;
		float* leftVertex = NULL;
		float* rightVertex = NULL;
		vector<faceInfo*> faceLeft(0), faceRight(0);
		int leftI = 0;
		int rightI = 0;

		//policz p砤szczyzn?
		node->box->ComputeDivisionPlane(plane);
		//podziel vertexy
		node->DividePoints(plane, faces, faceLeft, &leftVertex, leftI, faceRight, &rightVertex, rightI);

		if ((!leftI) || (!rightI))
		{
			node->box->SetEnd();
			//for (auto face : faces)
				//node->box->AddFace(face);
			return;
		}
		if (leftI >= minVertCount && condition == OBB_VERTEX_CONDITION && leftI < conParam)
		{
			//koniec imprezy dla levego
			delete[] leftVertex;
		} 
		else
		{
			node->left = new OBBTreeNode;
			ManageNode(node->left, node->level, faceLeft, leftVertex, leftI, tmpScale);
		}
		
		if (rightI >= minVertCount && condition == OBB_VERTEX_CONDITION && rightI <  conParam)
		{
			//koniec imprezy dla prawego
			delete[] rightVertex;
		} 
		else
		{
			node->right = new OBBTreeNode;
			ManageNode(node->right, node->level, faceRight, rightVertex, rightI, tmpScale);
		}
		node->GetBox()->faceLink = vector<faceInfo*>();
	}
}

void OBBTree::Move(Vector4f disp)
{
	MoveNode(tree, disp);
}

void OBBTree::MoveNode(OBBTreeNode* node, Vector4f disp)
{
	node->box->Move(disp);
	if (node->left != NULL && node->right != NULL)
	{
		MoveNode(node->left, disp);
		MoveNode(node->right, disp);
	}
}

void OBBTree::ManaEdge(int v1, int v2, int faceId)
{
	if (v1 > v2)swap(v1, v2);
	/*if (edges[v1].empty())
	{
		edges[v1].push_back(Edge(v2, faceId, 0));
		return;
	}*/
	for (auto& edge : edges[v1])
	{
		if (v2 == edge.vertId)
		{
			edge.faceId[1] = faceId;
			return;
		}
	}
	edges[v1].push_back(Edge(v2, faceId, 0));
}

//////////////////////////////////////////////////////////////////////////
//OBBTreeNode

OBBTreeNode::OBBTreeNode():box(NULL), left(NULL), right(NULL), level(0){}
OBBTreeNode::~OBBTreeNode()
{
	if (left)	delete left;
	if (right)	delete right;
	left = NULL;
	right = NULL;
	if (box)	delete box;
	box = NULL;
}

bool OBBTreeNode::ComputeBox(float* tmpVert, int tmpCount, float tmpScale /* = 1.0f */)
{
	box = new OBB(tmpVert, tmpCount, tmpScale);
	box->ComputeBox();
	return true;
}

void OBBTreeNode::DividePoints(Vector4f& plane, vector<faceInfo*>& faces, vector<faceInfo*> &faceLeft, float** lV, int& lC, vector<faceInfo*> &faceRight, float** rV, int& rC)
{
 	bool* vertAffiliation = new bool[box->vertCount];
	float* tmpVert = box->vertices;
	float tmpScale = box->scale;
	Vector4f planePoint;
	planePoint = box->OBBcenter;

	Vector4f normalVect(plane.x, plane.y, plane.z);

	//wektor od punktu na p砤szczy焠ie do testowanego punktu, p鬅niej liczymy k箃 mi阣zy tym
	//wektorem a wektorem normalnym i je渓i >90 to jedna strona jak <90 to druga
	Vector4f vect;
	Vector4f testPoint;

	const size_t size = box->GetVertexCount()/3;
	for (unsigned int i = 0; i < size; i++)
	{
		testPoint.Set(faces[i]->center*tmpScale);
		
		//tworzy wektor od punktu na p砤szczy焠ie do testowanego punktu
		vect.Set(testPoint.x - planePoint.x, testPoint.y - planePoint.y, testPoint.z - planePoint.z);
		//liczymy iloczyn skalarny a potem z tego wyciagamy kat w radianach
		float dotRes = Vector4f::Dot3f(vect, normalVect);
		//obliczamy kat pomiedzy wektorami (w radianach)	
		float kat = acos(dotRes/(vect.GetLenght() * normalVect.GetLenght()));		
			
		if (kat <= RAD_90_DEG) {
			lC++;
			vertAffiliation[i] = true;
			faceLeft.push_back(faces[i]);
		}
		else {
			rC++;
			vertAffiliation[i] = false;
			faceRight.push_back(faces[i]);
		}
	}

	//tutaj mam ju?ilo滄 punkt體 przypadaj筩ych na konkretne boxy i kt髍e punkty gdzie
	(*lV) = new float[lC*9];
	(*rV) = new float[rC*9];

	int leftI = 0;
	int rightI = 0;
	for (unsigned int i = 0; i < size*9; i+=3)
	{
		if (vertAffiliation[i/9])
		{
			(*lV)[leftI] = tmpVert[i];
			(*lV)[leftI+1] = tmpVert[i+1];
			(*lV)[leftI+2] = tmpVert[i+2];
			leftI += 3;
		}
		else
		{
			(*rV)[rightI] = tmpVert[i];
			(*rV)[rightI+1] = tmpVert[i+1];
			(*rV)[rightI+2] = tmpVert[i+2];
			rightI += 3;
		}
	}
	lC = leftI / 3;
	rC = rightI / 3;
	delete[] vertAffiliation;
	vertAffiliation = NULL;
}

//////////////////////////////////////////////////////////////////////////
//OBBIntersection



bool OBBIntersection::CollisionTest(OBB* obb1, OBB* obb2)
{
	if (obb1->GetVertexCount()==0)return false;
	if (obb2->GetVertexCount()==0)return false;
	Vector4f* obb1Points = obb1->GetPoints();
	Vector4f* obb2Points = obb2->GetPoints();

	Vector4f Ax = obb1Points[2] - obb1Points[1];
	Vector4f Ay = obb1Points[1] - obb1Points[0];
	Vector4f Az = obb1Points[3] - obb1Points[2];
	Vector4f Bx = obb2Points[2] - obb2Points[1];
	Vector4f By = obb2Points[1] - obb2Points[0];
	Vector4f Bz = obb2Points[3] - obb2Points[2];
	Ax.SetLenght(1.0f);
	Ay.SetLenght(1.0f);
	Az.SetLenght(1.0f);
	Bx.SetLenght(1.0f);
	By.SetLenght(1.0f);
	Bz.SetLenght(1.0f);

	if (TestSeparatingAxis(obb1, obb2, Ax))	return false;
	if (TestSeparatingAxis(obb1, obb2, Ay))	return false;
	if (TestSeparatingAxis(obb1, obb2, Az))	return false;
	if (TestSeparatingAxis(obb1, obb2, Bx))	return false;
	if (TestSeparatingAxis(obb1, obb2, By))	return false;
	if (TestSeparatingAxis(obb1, obb2, Bz))	return false;
	if (TestSeparatingAxis(obb1, obb2, Vector4f::Cross3f(Ax, Bx)))	return false;
	if (TestSeparatingAxis(obb1, obb2, Vector4f::Cross3f(Ax, By)))	return false;
	if (TestSeparatingAxis(obb1, obb2, Vector4f::Cross3f(Ax, Bz)))	return false;
	if (TestSeparatingAxis(obb1, obb2, Vector4f::Cross3f(Ay, Bx)))	return false;
	if (TestSeparatingAxis(obb1, obb2, Vector4f::Cross3f(Ay, By)))	return false;
	if (TestSeparatingAxis(obb1, obb2, Vector4f::Cross3f(Ay, Bz)))	return false;
	if (TestSeparatingAxis(obb1, obb2, Vector4f::Cross3f(Az, Bx)))	return false;
	if (TestSeparatingAxis(obb1, obb2, Vector4f::Cross3f(Az, By)))	return false;
	if (TestSeparatingAxis(obb1, obb2, Vector4f::Cross3f(Az, Bz)))	return false;
	return true;

}


bool OBBIntersection::TestSeparatingAxis(Ray* ray, OBB * obb, Vector4f & L)
{
	if (L.GetLenght() <= 1e-10)return false;
	Vector4f obbsides = obb->GetSides();

	float Wb = obbsides[0] / 2.0f;
	float Hb = obbsides[1] / 2.0f;
	float Db = obbsides[2] / 2.0f;

	Vector4f* obbPoints = obb->GetPoints();

	Vector4f By = obbPoints[2] - obbPoints[1];
	Vector4f Bz = obbPoints[1] - obbPoints[0];
	Vector4f Bx = obbPoints[3] - obbPoints[2];
	Bx.SetLenght(1.0f);
	By.SetLenght(1.0f);
	Bz.SetLenght(1.0f);

	Vector4f rayStartPt = ray->GetStartPt();
	Vector4f obbCenter = obb->GetCenter();

	Vector4f T = obbCenter - rayStartPt;
	Vector4f drct = ray->GetDirect();

	if (abs(Vector4f::Dot3f(T, L)) <=	abs(Vector4f::Dot3f(Bx*Wb, L)) +
										abs(Vector4f::Dot3f(By*Hb, L)) +
										abs(Vector4f::Dot3f(Bz*Db, L)))
		if (abs(Vector4f::Dot3f(T, drct) > ray->GetBegin() -
			abs(Vector4f::Dot3f(Bx*Wb, drct)) -
			abs(Vector4f::Dot3f(By*Hb, drct)) -
			abs(Vector4f::Dot3f(Bz*Db, drct))))
		{
			if (obb->IsEnd())
			{
				vector<faceInfo*> &faces = obb->GetFaces();
				//bool intersect;
				for (auto face : faces)
					CalcColliPoint(ray, face);
				return !ray->IsIntersect();
				//if (intersect)return false;
				//return true;
			}
			return false;
		}
	return true;
}

bool OBBIntersection::CalcColliPoint(Ray * ray, faceInfo * face)
{
	Vector4f s = ray->GetStartPt(), d = ray->GetDirect(), n = face->faceNorm, p, a, b, c;
	vector<Vector4f> verts = face->verts;
	if (Vector4f::Dot3f(d, n) >= 0)return false;
	float t, u, v;
	bool flag = IntersectTriangle(ray->GetStartPt(), ray->GetDirect(),
		verts[0], verts[1], verts[2], &t, &u, &v);
	if (flag&&t<ray->GetEnd()&&t>ray->GetBegin())
	{
		ray->SetEnd(t);
		ray->SetFace(face);
		ray->SetIntersect();
		return true;
	}
	return false;
}

bool OBBIntersection::CollisionTest(Ray* ray, OBB * obb)
{
	if (obb->GetVertexCount() == 0)return false;
	Vector4f* obbPoints = obb->GetPoints();

	Vector4f Ax = obbPoints[2] - obbPoints[1];
	Vector4f Ay = obbPoints[1] - obbPoints[0];
	Vector4f Az = obbPoints[3] - obbPoints[2];
	Ax.SetLenght(1.0f);
	Ay.SetLenght(1.0f);
	Az.SetLenght(1.0f);
	Vector4f direct = ray->GetDirect();
	direct.SetLenght(1.0f);

	if (TestSeparatingAxis(ray, obb, Vector4f::Cross3f(Ax, direct)))	return false;
	if (TestSeparatingAxis(ray, obb, Vector4f::Cross3f(Ay, direct)))	return false;
	if (TestSeparatingAxis(ray, obb, Vector4f::Cross3f(Az, direct)))	return false;
	return true;
}

bool OBBIntersection::CollisionTest(vector<Ray>& rays, int i, OBB * obb)
{
	if (obb->GetVertexCount() == 0)return false;
	Vector4f* obbPoints = obb->GetPoints();

	Vector4f Ax = obbPoints[2] - obbPoints[1];
	Vector4f Ay = obbPoints[1] - obbPoints[0];
	Vector4f Az = obbPoints[3] - obbPoints[2];
	Ax.SetLenght(1.0f);
	Ay.SetLenght(1.0f);
	Az.SetLenght(1.0f);
	Vector4f direct = rays[i].GetDirect();
	direct.SetLenght(1.0f);

	if (TestSeparatingAxis(rays, i, obb, Vector4f::Cross3f(Ax, direct)))	return false;
	if (TestSeparatingAxis(rays, i, obb, Vector4f::Cross3f(Ay, direct)))	return false;
	if (TestSeparatingAxis(rays, i, obb, Vector4f::Cross3f(Az, direct)))	return false;
	return true;
}

bool OBBIntersection::TestSeparatingAxis(vector<Ray>& rays, int i, OBB * obb, Vector4f & L)
{
	if (L.GetLenght() <= 1e-10)return false;
	Vector4f obbsides = obb->GetSides();

	float Wb = obbsides[0] / 2.0f;
	float Hb = obbsides[1] / 2.0f;
	float Db = obbsides[2] / 2.0f;

	Vector4f* obbPoints = obb->GetPoints();

	Vector4f By = obbPoints[2] - obbPoints[1];
	Vector4f Bz = obbPoints[1] - obbPoints[0];
	Vector4f Bx = obbPoints[3] - obbPoints[2];
	Bx.SetLenght(1.0f);
	By.SetLenght(1.0f);
	Bz.SetLenght(1.0f);

	Vector4f rayStartPt = rays[i].GetStartPt();
	Vector4f obbCenter = obb->GetCenter();

	Vector4f T = obbCenter - rayStartPt;
	//Vector4f drct = rays[i].GetDirect();
	float t, r;
	t = Vector4f::Dot3f(T, L);
	r = abs(Vector4f::Dot3f(Bx*Wb, L)) +
		abs(Vector4f::Dot3f(By*Hb, L)) +
		abs(Vector4f::Dot3f(Bz*Db, L));

	if ((t + r)*(t - r) < 0 || abs(t) <= r)return false;
	for (int j = 1; j <= rays.size(); j++)
	{
		int id = (i + j) % rays.size();
		Ray r = rays[id];
		if (t*Vector4f::Dot3f((r.GetDirect()*r.GetBegin()), L) > 0)return false;
	}
	return true;
}

bool OBBIntersection::CollisionTest(Frustum * fsm, OBB * obb)
{
	if (fsm->Dividable())return false;
	//vector<Ray> rays = fsm->GetCorner();
	//for (int i = 0; i < rays.size(); i++)
	//	if (CollisionTest(&rays[i], obb))//与角相交
	//	{
	//		if (obb->IsEnd())fsm->AddObb(obb);
	//		return true;
	//	}
	for (int i = 0; i < fsm->GetNorm().size(); i++)
		if (TestSeparatingAxis(fsm, obb, fsm->GetNorm()[i], i))return false;//侧面投影分离
	/*Vector4f center = obb->GetCenter() - fsm->GetVertex();
	for (int i = 0; i < fsm->GetNorm().size(); i++)
		if (Vector4f::Dot3f(center, fsm->GetNorm()[i]))return false;*/
	if (obb->IsEnd())fsm->AddObb(obb);
	return true;
}

bool OBBIntersection::TestSeparatingAxis(Frustum * fsm, OBB * obb, Vector4f & L, int i)
{
	if (L.GetLenght() <= 1e-10)return false;
	Vector4f obbsides = obb->GetSides();

	float Wb = obbsides[0] / 2.0f;
	float Hb = obbsides[1] / 2.0f;
	float Db = obbsides[2] / 2.0f;

	Vector4f* obbPoints = obb->GetPoints();

	Vector4f By = obbPoints[2] - obbPoints[1];
	Vector4f Bz = obbPoints[1] - obbPoints[0];
	Vector4f Bx = obbPoints[3] - obbPoints[2];
	Bx.SetLenght(1.0f);
	By.SetLenght(1.0f);
	Bz.SetLenght(1.0f);

	Vector4f vert = fsm->GetVertex();
	Ray r = fsm->GetCorner().front();
	if (i == fsm->GetNorm().size() - 1)vert = r.GetStartPt() + r.GetDirect()*r.GetBegin();
	Vector4f obbCenter = obb->GetCenter();

	Vector4f T = obbCenter - vert;

	if (abs(Vector4f::Dot3f(Bx*Wb, L)) +
		abs(Vector4f::Dot3f(By*Hb, L)) +
		abs(Vector4f::Dot3f(Bz*Db, L)) + 
		Vector4f::Dot3f(T, L) >= 0)
		{
			return false;
		}
	return true;
}

bool OBBIntersection::TestSeparatingAxis(OBB* obb1, OBB* obb2, Vector4f& L)
{
	Vector4f obb1sides = obb1->GetSides();
	Vector4f obb2sides = obb2->GetSides();

	float Wa = obb1sides[0]/2.0f;
	float Ha = obb1sides[1]/2.0f;
	float Da = obb1sides[2]/2.0f;
	float Wb = obb2sides[0]/2.0f;
	float Hb = obb2sides[1]/2.0f;
	float Db = obb2sides[2]/2.0f;

	Vector4f* obb1Points = obb1->GetPoints();
	Vector4f* obb2Points = obb2->GetPoints();

	Vector4f Ay = obb1Points[2] - obb1Points[1];
	Vector4f Az = obb1Points[1] - obb1Points[0];
	Vector4f Ax = obb1Points[3] - obb1Points[2];
	Vector4f By = obb2Points[2] - obb2Points[1];
	Vector4f Bz = obb2Points[1] - obb2Points[0];
	Vector4f Bx = obb2Points[3] - obb2Points[2];
	Ax.SetLenght(1.0f);
	Ay.SetLenght(1.0f);
	Az.SetLenght(1.0f);
	Bx.SetLenght(1.0f);
	By.SetLenght(1.0f);
	Bz.SetLenght(1.0f);

	Vector4f obb1Center = obb1->GetCenter();
	Vector4f obb2Center = obb2->GetCenter();

	Vector4f T = obb2Center - obb1Center;

	if (abs(Vector4f::Dot3f(T, L)) >	abs(Vector4f::Dot3f(Ax*Wa, L)) + 
										abs(Vector4f::Dot3f(Ay*Ha, L)) +
										abs(Vector4f::Dot3f(Az*Da, L)) +
										abs(Vector4f::Dot3f(Bx*Wb, L)) +
										abs(Vector4f::Dot3f(By*Hb, L)) +
										abs(Vector4f::Dot3f(Bz*Db, L)))
		return true;
	else
		return false;
}


bool OBBIntersection::CollisionTest(OBBTree* t1, OBBTree* t2)//This function is wrong
{
	if (CollisionTest(t1->GetTree()->GetBox(), t2->GetTree()->GetBox()))
		return true;
	else 
		return false;
}

// Determine whether a ray intersect with a triangle
// Parameters
// orig: origin of the ray
// dir: direction of the ray
// v0, v1, v2: vertices of triangle
// t(out): weight of the intersection for the ray
// u(out), v(out): barycentric coordinate of intersection

bool OBBIntersection::IntersectTriangle(Vector4f& orig, Vector4f& dir,
	Vector4f& v0, Vector4f& v1, Vector4f& v2,
	float* t, float* u, float* v)
{
	// E1
	Vector4f E1 = v1 - v0;

	// E2
	Vector4f E2 = v2 - v0;

	// P
	Vector4f P = Vector4f::Cross3f(dir,E2);

	// determinant
	float det = Vector4f::Dot3f(E1,P);

	// keep det > 0, modify T accordingly
	Vector4f T;
	if (det >0)
	{
		T = orig - v0;
	}
	else
	{
		T = v0 - orig;
		det = -det;
	}

	// If determinant is near zero, ray lies in plane of triangle
	if (det < 0.0001f)
		return false;

	// Calculate u and make sure u <= 1
	*u = Vector4f::Dot3f(T,P);
	if (*u < 0.0f || *u > det)
		return false;

	// Q
	Vector4f Q = Vector4f::Cross3f(T,E1);

	// Calculate v and make sure u + v <= 1
	*v = Vector4f::Dot3f(dir,Q);
	if (*v < 0.0f || *u + *v > det)
		return false;

	// Calculate t, scale parameters, ray intersects triangle
	*t = Vector4f::Dot3f(E2,Q);

	float fInvDet = 1.0f / det;
	*t *= fInvDet;
	*u *= fInvDet;
	*v *= fInvDet;

	return true;
}