#ifndef OBB_TREE_H
#define OBB_TREE_H
#include <vector>

#include "OBB.h"
#include "mathdefs.h"
#include "frustum.h"

enum TREE_CONDITION
{
	OBB_DEPTH_CONDITION = 1,
	OBB_VERTEX_CONDITION = 2,
};


extern OBBTree;
//////////////////////////////////////////////////////////////////////////
//OBBTreeNode
class OBBTreeNode
{
	friend class OBBTree;
public:
	OBBTreeNode* GetRight()		{return right;}
	OBBTreeNode* GetLeft()		{return left;}
	int	GetLevel()				{return level;}
	OBB* GetBox()				{return box;}

private:
	OBBTreeNode();
	~OBBTreeNode();
	bool ComputeBox(float* tmpVert, int tmpCount, float tmpScale = 1.0f);
	void DividePoints(Vector4f & plane, vector<faceInfo*>& faces, vector<faceInfo*>& faceLeft, float ** lV, int & lC, vector<faceInfo*>& faceRight, float ** rV, int & rC);
	
	int level;
	OBB* box;
	vector<faceInfo> geomFaces;
	OBBTreeNode* left;
	OBBTreeNode* right;

};

//////////////////////////////////////////////////////////////////////////
//OBBTree
class OBBTree
{
public:
	OBBTree();
	~OBBTree();

	bool ComputeOBBTree(TREE_CONDITION treeCondition, int TreeConParam, float* tmpVert, 
						int tmpCount, vector<int>& vertId, float tmpScale = 1.0f);
	
	OBBTreeNode* GetTree()			{return tree;}
	void SetMinVertCount(int tmp)	{minVertCount = tmp;}
	int GetLevelsCount()			{return levels;}

	void Move(Vector4f disp);

	faceInfo faces[100000];
	vector<vector<Edge>> edges;

	//Vector3f *vertices;
private:
	void ManageNode(OBBTreeNode * node, int nodeLVL, vector<faceInfo*>& faces, float * tmpVert, int tmpCount, float tmpScale);
	void MoveNode(OBBTreeNode* node, Vector4f disp);
	void ManaEdge(int v1, int v2, int faceId);

	OBBTreeNode* tree;
	int	levels;
	int	condition;
	int	conParam;
	int minVertCount;
};


namespace OBBIntersection
{
	bool TestSeparatingAxis(OBB* obb1, OBB* obb2, Vector4f& L);

	template<typename T>
	void TestCollisionNode(T* goem, OBBTreeNode* node, int& treeDepth, bool& flag);

	bool CollisionTest(OBBTree* t1, OBBTree* t2);
	template<typename T>
	bool CollisionTest(T* goem, OBBTree* tree);
	bool CollisionTest(OBB* obb1, OBB* obb2);

	bool TestSeparatingAxis(Ray* ray, OBB* obb, Vector4f& L);
	bool CalcColliPoint(Ray* ray, faceInfo* face);
	bool CollisionTest(Ray* ray, OBB* obb);
	bool CollisionTest(vector<Ray>& rays, int i, OBB* obb);
	bool TestSeparatingAxis(vector<Ray>& rays, int i, OBB* obb, Vector4f& L);

	bool CollisionTest(Frustum* fsm, OBB* obb);
	bool TestSeparatingAxis(Frustum* fsm, OBB* obb, Vector4f& L, int i);

	bool IntersectTriangle(Vector4f& orig, Vector4f& dir,
		Vector4f& v0, Vector4f& v1, Vector4f& v2,
		float* t, float* u, float* v);
}

template<typename T>
bool OBBIntersection::CollisionTest(T* goem, OBBTree* tree)
{
	int treeDepth = tree->GetLevelsCount();
	bool collisionFlag = false;

	//test czy mamy kolizj?z bry³a najogólniejsz?
	if (!CollisionTest(goem, tree->GetTree()->GetBox()))
		return false;	//jesli nie, wyjœcie z testowania
	else if (treeDepth == 1)
		return true;	//jezeli mamy kolizj?a hierarchia sk³ada si?z jednego poziomu

	TestCollisionNode(goem, tree->GetTree(), treeDepth, collisionFlag);
	if (collisionFlag)
		return true;
	else
		return false;
}

template<typename T>
void OBBIntersection::TestCollisionNode(T* goem, OBBTreeNode* node, int& treeDepth, bool& flag)
{
	//jesli mamy tu true znaczy, ¿e wczeœniej znaleziono ju?kolizj?i nie rtzeba dalej szuka?
	//if (flag == true)
		//return;
	if (!CollisionTest(goem, node->GetBox()))
		return;
	else if (node->GetBox()->IsEnd())
	{
		//if (CollisionTest(goem, node->GetBox()))
			flag = true;
		return;
	}
	TestCollisionNode(goem, node->GetLeft(), treeDepth, flag);
	TestCollisionNode(goem, node->GetRight(), treeDepth, flag);
}
#endif