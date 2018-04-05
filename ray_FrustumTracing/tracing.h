#ifndef TRACING_H
#define TRACING_H

#include <fstream>
#include <queue>
#include <unordered_map>
#include <set>
#include <functional>
#include <io.h>
#include <direct.h>
#include <time.h>
#include <thread>
#include <mutex>

#include "obbTree.h"
#include "mathdefs.h"
#include "dataBase.h"
#include "utils.h"

namespace Tracing
{
	void ReadPathAndColli(string filename, string destdir, vector<BiNode<Ray>*>& rays, Orient& s, int len);
	void ReadSourceAndTracing(vector<BiNode<Ray>*>& rays, OBBTree* tree, int ref, string fileIndex);
	void TracingInRoom(vector<BiNode<Ray>*>& rays, OBBTree * tree, int ref, Vector4f s, int nCircle = 150);
	void RayTracing(BiNode<Ray> * pr, OBBTree * tree, int ref);
	void RayTracingParallel(BiNode<Ray> * pr, OBBTree * tree, int ref, int* numThreads);
	void RefRay(BiNode<Ray>* pr, faceInfo& f);
	void Traversal(BiNode<Ray>* ray, Orient& rec, const vector<COMPLEX>& sDrct, vector<vector<double>>& hrir, vector<int>& refs, vector<int>& scats, double len = 0);
	void TraversalParallel(int *threadNum, BiNode<Ray>* ray, Orient& rec, const vector<COMPLEX>& sDrct, vector<vector<double>>& hrir, vector<int>& refs, vector<int>& scats, double len = 0);
	void ColliReceiver(vector<BiNode<Ray>*>& rays, Orient& s, Orient& r, vector<vector<double>>& hrir, ofstream& fout);

	void RayTracing(vector<Ray> &ray, OBBTree* tree, int ref);
	void RayTracing(vector<Ray> &ray, OBBTree* tree, float len);
	Ray RefRay(Ray &r, faceInfo &f);
	template<typename T>
	void TracingInRoom(vector<vector<Ray>>& rays, OBBTree * tree, T ref, Vector4f s, int nCircle = 500);
	void ColliReceiver(vector<vector<Ray>>& rays, Vector4f rec, Vector4f front, Vector4f up, vector<vector<double>>& hrir, ofstream& fout);

	FsmNode& ColliFace(FsmNode& fNode, OBBTree* tree);
	void JudgeVertsSides(Frustum& fsm, vector<Vector4f>& verts, vector<vector<bool>>& vertsSides);
	bool TestVertsOneSide(Vector4f plane, vector<Vector4f> verts);
	bool TestVertsTwoSides(vector<Vector4f> vertsFace, Vector4f vert, vector<Vector4f> vertsCheck, vector<vector<bool>>& vertsSides);
	bool TestShading(Vector4f vert, vector<Vector4f> vertsFace, OBBTree* tree);
	bool FaceUnionAndCull(FsmNode& fNode, OBBTree* tree);
	bool FindFacePair(vector<vector<Edge>>& edges, int v1, int v2, int fId, int& pairId);
	int FindParent(unordered_map<int, pair<int, faceInfo*>>& fLink, int fId);
	void CalcuRefPlane(FsmNode& fNode);
	Vector4f CrossPlane(Vector4f d1, Vector4f d2, Vector4f vp, Vector4f vt);
	void ReflectFrustum(queue<FsmNode>& que, vector<FsmNode>& vec);
	void FrustumTracing(queue<FsmNode>& queFNode, vector<FsmNode>& vecFNode, OBBTree* tree, int ref);
	void TracingInRoom(vector<vector<FsmNode>>& vecFsms, OBBTree* tree, int ref, Vector4f s);
	void ColliReceiver(vector<vector<FsmNode>>& fNodes, Vector4f rec, Vector4f front, Vector4f up, vector<vector<double>>& hrir, ofstream& fout);

	template<typename T>
	void ReadPathAndColli(string filename, string destdir, vector<vector<T>>& fNodes, int len);
	template<typename T>
	void ReadSourceAndTracing(vector<vector<T>>& vecFsms, OBBTree* tree, int ref, string fileIndex);
}

template <typename T>
void Tracing::TracingInRoom(vector<vector<Ray>>& rays, OBBTree * tree, T ref, Vector4f s, int nCircle)
{
	//int nCircle = 4;
	float dTheta = PI / nCircle;
	Vector4f source = s;
	for (float i = 0.5f; i < nCircle; i++)
	{
		float theta = i*dTheta;
		int m = round(2 * PI*sin(theta) / dTheta);
		for (float j = 0.5f; j < m; j++)
		{
			float phi = j*PI * 2 / m;
			Vector4f drct = Vector4f(sin(theta)*cos(phi), sin(theta)*sin(phi), cos(theta));
			double amp = DirectBeam(drct);
			Ray ray = Ray(source, drct, amp);
			OBBIntersection::CollisionTest(&ray, tree);
			rays.push_back(vector<Ray>(1, ray));
			RayTracing(rays.back(), tree, ref);
		}
	}
	fstream fout = fstream("raysInfo.txt", ios::out);
	for (auto v : rays)
		fout << v.size() << endl;
}

template <typename T>
void Tracing::ReadPathAndColli(string filename, string destdir, vector<vector<T>>& fNodes, int len)
{
	fstream fin(filename, ios::in);
	int n; //Number of receivers
	Vector4f receiver, front, up;
	vector<vector<double>> hrir;
	fin >> n;
	for (int i = 0; i < n; i++)
	{
		fin >> receiver[0] >> receiver[1] >> receiver[2] >>
			front[0] >> front[1] >> front[2] >>
			up[0] >> up[1] >> up[2];
		hrir = vector<vector<double>>(len, vector<double>(2));
		if (_access(destdir.c_str(), 6) == -1)_mkdir(destdir.c_str());
		ofstream foutbin(destdir + to_string(i) + ".binary", ios::binary);
		ColliReceiver(fNodes, receiver, front, up, hrir, foutbin);
		foutbin.close();
		/*ofstream fout(destdir + to_string(i) + ".dat", ios::binary);
		for (int k = 0; k < len; k++)
			for (int l = 0; l < 2; l++)
				fout.write((char*)&hrir[k][l], sizeof(double));
		fout.close();*/
	}
	fin.close();
}

template <typename T>
void Tracing::ReadSourceAndTracing(vector<vector<T>>& vecFsms, OBBTree * tree, int ref, string fileIndex)
{
	int numSources;
	string pathFile;
	Vector4f source;
	srand((int)time(0));
	fstream fin(fileIndex, ios::in);
	fin >> numSources;
	for (int i = 0; i < numSources; i++)
	{
		fin >> source[0] >> source[1] >> source[2] >> pathFile;
		vecFsms.clear();
		TracingInRoom(vecFsms, tree, ref, source);
		ReadPathAndColli(pathFile, ".\\data\\brir\\Source_" + to_string(i) + "\\", vecFsms, 16384);
	}
	fin.close();
}
#endif // !TRACING_H
