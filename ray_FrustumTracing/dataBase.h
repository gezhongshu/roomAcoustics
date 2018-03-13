#ifndef DATABASE_H
#define DATABASE_H

#include <vector>
#include <string>
#include <fstream>
#include <cmath>
#include <cassert>

#include "mathdefs.h"

using namespace std;

const float SOUND_SPEED = 343.37f;
const float FS = 48e3f;

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
	WallAirAbsorb(string fileName);
	~WallAirAbsorb();
	static vector<double> Absorb(double dist, int ref);
	static void Init(int ref);
	static void LoadMat(vector<string> fileNames);
	static vector<vector<float>> ConvHrir(vector<double> filter, vector<vector<float>> hrir);

private:
	static vector<string> matName;
	static vector<double> fq, attenAir;
	static vector<vector<double>> alpha, bref;
};
#endif // !DATABASE_H
