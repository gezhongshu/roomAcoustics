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
#include <Windows.h>

#include "obbTree.h"
#include "mathdefs.h"
#include "dataBase.h"
#include "utils.h"

namespace Tracing
{
	const int NC = 150;
	static mutex mu_thread, mu_hrir;
	static int maxThread = thread::hardware_concurrency();
	extern double angleLim;


	template<typename T>
	void ReadPathAndColli(string filename, string destdir, vector<BiNode<T>*>& rays, Orient& s, int len);
	template<typename T>
	void ReadSourceAndTracing(vector<BiNode<T>*>& rays, OBBTree* tree, int ref, string fileIndex);
	template<typename T>
	void RayTracing(BiNode<T> * pr, OBBTree * tree, int ref);
	template<typename T>
	void RayTracingParallel(BiNode<T> * pr, OBBTree * tree, int ref, int* numThreads);
	template<typename T>
	void Traversal(BiNode<T>* ray, Orient& rec, const vector<COMPLEX>& sDrct, vector<vector<double>>& hrir, vector<int>& refs, vector<int>& mirs, vector<int>& scats, int band = -2);
	template<typename T>
	void TraversalParallel(int *threadNum, BiNode<T>* ray, Orient& rec, const vector<COMPLEX>& sDrct, vector<vector<double>>& hrir, vector<int> refs, vector<int> mirs, vector<int> scats, int band = -2);
	template<typename T>
	void ColliReceiver(vector<BiNode<T>*>& rays, Orient& s, Orient& r, vector<vector<double>>& hrir, ofstream& fout);

	void TracingInRoom(vector<BiNode<Ray>*>& rays, OBBTree * tree, int ref, Vector4f s, int nCircle = NC);
	void RefRay(BiNode<Ray>* pr, bool divide = true);
	void ColliRay(Ray* ray, OBBTree* tree);
	void PassReceiver(BiNode<Ray>* ray, Orient& rec, const vector<COMPLEX>& sDrct, vector<vector<double>>& hrir, vector<int>& refs, vector<int>& mirs, vector<int>& scats, int band);
	
	void TracingInRoom(vector<BiNode<FsmNode>*>& rays, OBBTree * tree, int ref, Vector4f s);
	void RefRay(BiNode<FsmNode>* pr, bool divide = true);
	void ColliRay(FsmNode* ray, OBBTree* tree);
	void PassReceiver(BiNode<FsmNode>* ray, Orient& rec, const vector<COMPLEX>& sDrct, vector<vector<double>>& hrir, vector<int>& refs, vector<int>& mirs, vector<int>& scats, int band);

	void TracingInRoom(vector<BiNode<RayNode>*>& rays, OBBTree * tree, int ref, Vector4f s);
	void RefRay(BiNode<RayNode>* pr, bool divide = true);
	void ColliRay(RayNode* ray, OBBTree* tree);
	void PassReceiver(BiNode<RayNode>* ray, Orient& rec, const vector<COMPLEX>& sDrct, vector<vector<double>>& hrir, vector<int>& refs, vector<int>& mirs, vector<int>& scats, int band);


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
		ofstream foutbin(destdir + to_string(i) + ".dat", ios::binary);
		ColliReceiver(fNodes, receiver, front, up, hrir, foutbin);
		foutbin.close();
		ofstream fout(destdir + to_string(i) + ".binary", ios::binary);
		for (int k = 0; k < len; k++)
			for (int l = 0; l < 1; l++)
				fout.write((char*)&hrir[k][l], sizeof(double));
		fout.close();
	}
	fin.close();
}

template <typename T>
void Tracing::ReadSourceAndTracing(vector<vector<T>>& vecFsms, OBBTree * tree, int ref, string fileIndex)
{
	int numSources;
	string fname, directFile, pathFile;
	Vector4f source, front, up;
	srand((int)time(0));
	fstream fin(fileIndex, ios::in);
	fin >> numSources;
	for (int i = 0; i < numSources; i++)
	{
		fin >> fname >> source[0] >> source[2] >> source[1]
			>> front[0] >> front[2] >> front[1]
			>> up[0] >> up[2] >> up[1]
			>> directFile >> pathFile;
		vecFsms.clear();
		TracingInRoom(vecFsms, tree, ref, source);
		ReadPathAndColli(pathFile, ".\\data\\RIR\\Fsm_" + fname + "\\", vecFsms, 16384);
	}
	fin.close();
}



template <typename T>
void Tracing::ReadPathAndColli(string filename, string destdir, vector<BiNode<T>*>& rays, Orient& s, int len)
{
	fstream fin(filename, ios::in);
	int n; //Number of receivers
	Vector4f receiver, front, up;
	vector<vector<double>> hrir;
	fin >> n;
	for (int i = 0; i < n; i++)
	{
		fin >> receiver[0] >> receiver[2] >> receiver[1] >>
			front[0] >> front[2] >> front[1] >>
			up[0] >> up[2] >> up[1];
		hrir = vector<vector<double>>(len, vector<double>(1));
		cout << "\nCalculating receiver: " << i + 1 << " / " << n << endl;
		int offset = 0;
		while (offset != string::npos)
			if (_access(destdir.substr(0, offset=destdir.find_first_of('\\', offset+1)).c_str(), 6) == -1)
				_mkdir(destdir.substr(0, offset).c_str());
		ofstream fout(destdir + to_string(i) + ".binary", ios::binary);
		ColliReceiver(rays, s, Orient(receiver, front, up), hrir, fout);
		/*fout.close();
		ofstream fout(destdir + to_string(i) + ".dat", ios::binary);*/
		for (int k = 0; k < len; k++)
			for (int l = 0; l < 1; l++)
				fout.write((char*)&hrir[k][l], sizeof(double));
		fout.close();
	}
	fin.close();
	for (auto ray : rays)
		ray->relase();
}

template <typename T>
void Tracing::ReadSourceAndTracing(vector<BiNode<T>*>& rays, OBBTree * tree, int ref, string fileIndex)
{
	int numSources;
	string fname, directFile, pathFile;
	Vector4f source, front, up;
	double roomVolume = tree->GetTree()->GetBox()->GetVolume();
	Tracing::angleLim = log10(roomVolume) * pi / 2 / NC;
	/*cout << "\nangleLim = " << Tracing::angleLim << ", room volume: " << roomVolume
		<< "\nEvaluate number of rays: " << int(pow(4 / pi*NC, 2)) << endl;*/
	srand((int)time(0));
	fstream fin(fileIndex, ios::in);
	fin >> numSources;
	for (int i = 0; i < numSources; i++)
	{
		fin >> fname >> source[0] >> source[2] >> source[1]
			>> front[0] >> front[2] >> front[1]
			>> up[0] >> up[2] >> up[1]
			>> directFile >> pathFile;
		rays.clear();
		cout << "\nCalculating source: " << i + 1 << " / " << numSources << endl << endl;
		bool complete = false;
		thread task(Direct::paraLoad, directFile, &complete);
		task.detach();
		TracingInRoom(rays, tree, ref, source);
		//Direct::LoadCSV(directFile);
		while (!complete) Sleep(1);
		string tname(typeid(T).name());
		tname = tname.substr(tname.find_last_of(' ') + 1);
		string sceneName = fileIndex.substr(fileIndex.find_last_of('.') - 2, 2);
		ReadPathAndColli(pathFile, ".\\data\\RIR\\scene" + sceneName + "\\" + tname + "_" + fname + "\\", rays, Orient(source, front, up), 16384);
	}
	fin.close();
}

template <typename T>
void Tracing::RayTracing(BiNode<T>* pr, OBBTree * tree, int ref)
{
	if (pr->data.GetDist() < MAX_DIST && pr->data.IsIntersect() && ref > 0)
	{
		Tracing::RefRay(pr);
		Tracing::ColliRay(&pr->left->data, tree);
		Tracing::RayTracing(pr->left, tree, ref - 1);
		Tracing::ColliRay(&pr->right->data, tree);
		Tracing::RayTracing(pr->right, tree, ref - 1);
	}
	else if (pr->data.GetDist() < MAX_DIST && pr->data.IsIntersect() && -ref < MAX_REF)
	{
		Tracing::RefRay(pr, false);
		Tracing::ColliRay(&pr->right->data, tree);
		Tracing::RayTracing(pr->right, tree, ref - 1);
	}
}

template <typename T>
void Tracing::RayTracingParallel(BiNode<T>* pr, OBBTree * tree, int ref, int * numThreads)
{
	Tracing::RayTracing(pr, tree, ref);
	mu_thread.lock();
	(*numThreads)--;
	mu_thread.unlock();
}


template <typename T>
void Tracing::Traversal(BiNode<T>* ray, Orient & rec, const vector<COMPLEX>& sDrct, vector<vector<double>>& hrir, vector<int>& refs, vector<int>& mirs, vector<int>& scats, int band)
{
	if (!ray)
	{
		cout << "error!" << endl;
		return;
	}
	int mId = -1;
	if (ray->data.IsIntersect())
	{
		mId = ray->data.GetFace()->m_id;
		refs[mId]++;
		if (ray->left)
		{
			if (ray->isScat)scats[mId]++;
			Tracing::Traversal(ray->left, rec, sDrct, hrir, refs, mirs, scats);
			if (ray->isScat)scats[mId]--;
		}
		else
			band--;
		if (ray->right)
		{
			if (ray->isScat)mirs[mId]++;
			Tracing::Traversal(ray->right, rec, sDrct, hrir, refs, mirs, scats, band);
			if (ray->isScat)mirs[mId]--;
		}
		refs[mId]--;
	}

	PassReceiver(ray, rec, sDrct, hrir, refs, mirs, scats, band);
}

template <typename T>
void Tracing::TraversalParallel(int * numThreads, BiNode<T>* ray, Orient & rec, const vector<COMPLEX>& sDrct, vector<vector<double>>& hrir, vector<int> refs, vector<int> mirs, vector<int> scats, int band)
{
	Tracing::Traversal(ray, rec, sDrct, hrir, refs, mirs, scats, band);
	mu_thread.lock();
	(*numThreads)--;
	mu_thread.unlock();
}

template <typename T>
void Tracing::ColliReceiver(vector<BiNode<T>*>& rays, Orient& s, Orient& r, vector<vector<double>>& hrir, ofstream& fout)
{

	vector<COMPLEX> sDrct;
	vector<int> refs(WallAirAbsorb::GetMatNum(), 0), mirs(WallAirAbsorb::GetMatNum(), 0), scats(WallAirAbsorb::GetMatNum(), 0);
	cout << endl;
	int totalTask = 0, numRay = 0;
	vector<thread> tasks;
	for (auto ray : rays)
	{
		numRay++;
		Vector4f drct = ray->data.GetDirect();
		/*Vector4f StoR = r.GetPos() - s.GetPos();
		StoR.SetLenght(1.0);
		if (Vector4f::Dot3f(drct, StoR) > 0.997)
		{
 			cout << drct.x << drct.y << drct.z << endl;
		}*/
		sDrct = Direct::EvalAmp(s.LocalPolar(drct));
		//Tracing::Traversal(ray, r, sDrct, hrir, refs, mirs, scats);
		while (totalTask > maxThread) Sleep(0);
		mu_thread.lock();
		totalTask++;
		string s;
		s = "Running: " + to_string(numRay) + " / " + to_string(rays.size())
			+ ", number of threads: " + to_string(totalTask);
		for (int i = 0; i < s.size(); i++)
			printf("\b");
		//cout << s << flush;
		printf("%s", s.c_str());
		mu_thread.unlock();
		tasks.push_back(thread(TraversalParallel<T>, &totalTask, ray, r, sDrct, std::ref(hrir), refs, mirs, scats, -2));
		tasks.back().detach();
	}
	while (totalTask > 0) Sleep(1);
	cout << endl;
}

#endif // !TRACING_H
