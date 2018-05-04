#include <string.h>
#include <iostream>

#include "obbTree.h"
#include "frustum.h"
#include "tracing.h"

#include "mesh.h"
#include "define.h"
#include "OBB.h"
#include "dataBase.h"

extern float delta = 2;
extern bool hasScat = true;
extern bool hasDrct = true;
extern int wideBand = -2;

void TraceScenes(string file, int choose)
{
	fstream fin(file, ios::in);
	int n; //Number of scenes
	string sceneFile, matFile, sourceFile;
	fin >> n;
	for (int i = 0; i < n; i++)
	{
		cout << "\n\nTracing scene: " << i + 1 << " / " << n << endl;
		fin >> sceneFile >> matFile >> sourceFile;
		if (i != choose)continue;
		/*switch (choose)
		{
		case 2:
			delta = 1;
			break;
		case 1:
			delta = 1;
			break;
		default:
			delta = 2;
		}*/
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
		int ref = 0;
		WallAirAbsorb::Init(ref + MAX_REF);
		//Tracing::ReadSourceAndTracing(rays, tree, ref, sourceFile);

		/*cout << "\nTracing fNodes ..." << flush;
		ref = 0;
		WallAirAbsorb::Init(ref + MAX_REF);
		Tracing::ReadSourceAndTracing(fNodes, tree, ref, sourceFile);*/

		//cout << "\nTracing rNodes ..." << flush;
		/*ref = 0;
		WallAirAbsorb::Init(ref + MAX_REF);*/
		Tracing::ReadSourceAndTracing(rNodes, tree, ref, sourceFile);

		if (obj)		delete obj;
		if (tree)		delete tree;
	}
}

int main(int argc, char* argv[]) 
{
	int choose = 0;
	switch (argc)
	{
	case 5:
		wideBand = -1 - atoi(argv[4]);
	case 4:
		hasDrct = (bool)atoi(argv[3]);
	case 3:
		hasScat = (bool)atoi(argv[2]);
	case 2:
		choose = atoi(argv[1]) - 1;
	default:
		break;
	}
	cout << "~~~Configuration:\n\tHas Scatter: " << hasScat << "\n\tHas Direction: " << hasDrct 
		<< "\n\tBand Average: " << (wideBand + 2) << endl;
	cout << "\nTracing rays ..." << endl;
	TraceScenes("data\\scenes.txt", choose);
	cout << "\nTraced off." << endl;

	system("pause");
	return 0;
}