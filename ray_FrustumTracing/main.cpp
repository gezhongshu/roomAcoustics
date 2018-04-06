#include <string.h>
#include <iostream>

#include "obbTree.h"
#include "frustum.h"
#include "tracing.h"

#include "mesh.h"
#include "define.h"
#include "OBB.h"
#include "dataBase.h"

int main() 
{
	int ref = 5;
	vector<int> vertId, matId;
	WallAirAbsorb::LoadwithFileList(string("data\\matInd\\mat_scene09.txt"));
	WallAirAbsorb::Init(ref);
	//Direct::LoadCSV("data\\Genelec8020.csv");
	Mesh* obj = new Mesh("elecOrgan", "data\\model\\scene09.obj", vertId, matId, true);

	float* vertTab;
	int vertCount;

	obj->GetAllVertex(&vertTab, vertCount);

	OBBTree* tree = new OBBTree();


	cout << "\nCalculating the OBB tree of the mesh ..." << endl;
	tree->ComputeOBBTree(OBB_DEPTH_CONDITION, 20, vertTab, vertCount, vertId, matId, 1.f);
	cout << "\nOBB tree calculated." << endl;

	vector<BiNode<Ray>*> rays;
	vector<vector<FsmNode>> Fsms;

	//cout << "Loading the HRIR database ..." << endl;
	//HRIR::LoadHrir();
	//cout << "HRIR loaded." << endl;

	cout << "\nTracing rays ..." << endl;
	Tracing::ReadSourceAndTracing(rays, tree, ref, ".\\data\\sources09.txt");
	//Tracing::ReadSourceAndTracing(Fsms, tree, ref, ".\\data\\sources09.txt");
	cout << "\nTraced off." << endl;

	if (obj)		delete obj;
	if (tree)		delete tree;
	return 0;
}