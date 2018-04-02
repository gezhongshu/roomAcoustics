#include <string.h>
#include <thread>

#include "obbTree.h"
#include "frustum.h"
#include "tracing.h"

#include "mesh.h"
#include "define.h"
#include "OBB.h"
#include "dataBase.h"

int main() 
{
	//Cube cube;
	Ray ray;
	vector<Ray> rays;
	vector<FsmNode> fNodes;
	vector<int> vertId, matId;
	WallAirAbsorb::LoadwithFileList(string("data\\matInd\\mat_scene05.txt"));
	//Direct genelec;
	//genelec.LoadCSV("data\\Genelec8020.csv");
	Mesh* obj = new Mesh("elecOrgan", "data\\model\\scene05.obj", vertId, matId, true);
	//Mesh* obj = new Mesh("elecOrgan", "data\\floor.obj", vertId, true);

	ray.Move(Vector4f(0.f, 2.f, 0.f));

	float* vertTab;
	int vertCount;
	float* vertTab2;
	int vertCount2;

	//cube.GetAllVertex(&vertTab, vertCount);
	obj->GetAllVertex(&vertTab2, vertCount2);

	OBBTree* tree2 = NULL;
	//OBBTree* tree = NULL;

	//tree = new OBBTree;
	tree2 = new OBBTree;


	cout << "Calculating the OBB tree of the mesh ..." << endl;
	//tree->ComputeOBBTree(OBB_DEPTH_CONDITION, 1, vertTab, vertCount, 1.0f);
	tree2->ComputeOBBTree(OBB_DEPTH_CONDITION, 20, vertTab2, vertCount2, vertId, matId, 1.f);
	cout << "OBB tree calculated." << endl;

	vector<vector<Ray> > rays_s;
	vector <vector<FsmNode>> fNodes_s;
	/*std::thread t = thread(Tracing::RayTracingInRoom, rays_s, tree2, 11);
	t.join();*/
	//Tracing::RayTracingInRoom(rays_s, tree2, 5);
	cout << "Loading the HRIR database ..." << endl;
	int Ind = 0;
	HRIR::LoadHrir();
	cout << "HRIR loaded." << endl;

	//ShowWindow(WND.hWnd, nShowCmd); 						// wyœwietla utworzony formularz na podstawie jego uchwytu

	int ref = 1;
	//WallAirAbsorb::Init(ref);
	cout << "Tracing rays ..." << endl;
	Tracing::ReadSourceAndTracing(rays_s, tree2, ref, ".\\data\\sources_exp.txt");
	cout << "Traced off." << endl;
	if (obj)		delete obj;
	//if (tree)		delete tree;
	if (tree2)		delete tree2;
	return 0;
}