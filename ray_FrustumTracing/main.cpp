#include <string.h>
#include <iostream>

#include "obbTree.h"
#include "frustum.h"
#include "tracing.h"

#include "mesh.h"
#include "define.h"
#include "OBB.h"
#include "dataBase.h"

void TraceScenes(string file)
{
	fstream fin(file, ios::in);
	int n; //Number of receivers
	string sceneFile, matFile, sourceFile;
	fin >> n;
	for (int i = 0; i < n; i++)
	{
		cout << "\n\nTracing scene: " << i + 1 << " / " << n << endl;
		fin >> sceneFile >> matFile >> sourceFile;
		vector<int> vertId, matId;
		WallAirAbsorb::LoadwithFileList(matFile);

		Mesh* obj = new Mesh("rooms", sceneFile, vertId, matId, true);

		float* vertTab;
		int vertCount;

		obj->GetAllVertex(&vertTab, vertCount);

		OBBTree* tree = new OBBTree();


		cout << "\nCalculating the OBB tree of the mesh ..." << endl;
		tree->ComputeOBBTree(OBB_DEPTH_CONDITION, 20, vertTab, vertCount, vertId, matId, 1.f);
		cout << "\nOBB tree calculated." << endl;

		vector<BiNode<Ray>*> rays;
		//vector<vector<FsmNode>> Fsms;
		vector<BiNode<FsmNode>*> fNodes;
		vector<BiNode<RayNode>*> rNodes;

		//cout << "Loading the HRIR database ..." << endl;
		//HRIR::LoadHrir();
		//cout << "HRIR loaded." << endl;
		cout << "\nTracing rays ..." << flush;
		int ref = 3;
		WallAirAbsorb::Init(ref + MAX_REF);
		Tracing::ReadSourceAndTracing(rays, tree, ref, sourceFile);

		cout << "\nTracing fNodes ..." << flush;
		ref = 0;
		WallAirAbsorb::Init(ref + MAX_REF);
		Tracing::ReadSourceAndTracing(fNodes, tree, ref, sourceFile);

		cout << "\nTracing rNodes ..." << flush;
		ref = 0;
		WallAirAbsorb::Init(ref + MAX_REF);
		Tracing::ReadSourceAndTracing(rNodes, tree, ref, sourceFile);

		if (obj)		delete obj;
		if (tree)		delete tree;
	}
}

int main() 
{
	cout << "\nTracing rays ..." << endl;
	TraceScenes("data\\scenes.txt");
	cout << "\nTraced off." << endl;

	system("pause");
	return 0;
}