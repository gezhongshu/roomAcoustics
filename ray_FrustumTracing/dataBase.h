#ifndef DATABASE_H
#define DATABASE_H

#include <vector>
#include <string>
#include <fstream>
#include <cmath>
#include <cassert>

#include "mathdefs.h"
#include "utils.h"

using namespace std;

const float SOUND_SPEED = 343.37f;
const float FS = 48e3f;
const float MAX_DIST = 16384 * SOUND_SPEED / FS;
const int MAX_REF = 100;

class HRIR
{
public:
	HRIR();
	~HRIR();
	static void LoadHrir();
	static void JudgeDirection(Vector4f drct, Vector4f front, Vector4f up, vector<vector<float>> & hrir_s);
	static vector<vector<float>> InterpHrir(int el, float L1, int az, float L2);

private:
	static vector<vector<vector<vector<float>>>> db;
};

class WallAirAbsorb
{
public:
	WallAirAbsorb();
	~WallAirAbsorb();
	inline static vector<string> GetMatName() { return matName; };
	inline static int GetMatNum() { return matName.size(); };
	static vector<double> Absorb(double dist, vector<int>& refs, vector<int>& mirs, vector<int>& scats, int band = -2);
	static void Init(int ref);
	static void LoadwithFileList(string fileName);
	static void LoadMat(vector<string> fileNames);
	static vector<vector<float>> ConvHrir(vector<double> filter, vector<vector<float>> hrir);
	static vector<double> FreqMult(double coef);
	static vector<double> InterpIFFT(vector<COMPLEX> fqValues, vector<double> fqs = fq);

private:
	static vector<string> matName;
	static vector<double> fq, attenAir;
	static vector<vector<double>> alpha, scatter;
	static vector<vector<vector<double>>> brefs, scatters, mirrors;
};

class Direct
{
public:
	Direct();
	~Direct();
	static void LoadCSV(string fileName);
	static void paraLoad(string fineName, bool * complete);
	static vector<COMPLEX> EvalAmp(int azim, int elev);
	static vector<COMPLEX> EvalAmp(vector<int> polar);

private:
	static string type;
	static vector<vector<vector<COMPLEX>>> directions;
};

class Orient
{
public:
	~Orient();
	Orient(Vector4f p, Vector4f f, Vector4f u);
	Orient(Vector4f p);
	vector<int> LocalPolar(Vector4f vec);
	inline Vector4f GetPos() { return position; }

private:
	Vector4f position, front, up, right;
};

#endif // !DATABASE_H
