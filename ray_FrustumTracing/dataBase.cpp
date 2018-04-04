#include "dataBase.h"


HRIR::HRIR()
{
}

HRIR::~HRIR()
{
}

void HRIR::LoadHrir()
{
	string file = ".\\data\\hrir\\hrir_big_d160.dat";
	fstream fin(file, ios::in);
	int ih, el, az, tmp;
	fin >> ih >> tmp >> el >> az;
	HRIR::db = vector<vector<vector<vector<float>>>>(el,
		vector < vector < vector<float>>>(az,
			vector < vector<float>>(ih,
				vector<float>(2))));
	for (int i = 0; i < el; i++)
		for (int j = 0; j < az; j++)
			for (int k = 0; k < ih; k++)
				for (int l = 0; l < tmp; l++)
					fin >> db[i][j][k][l];
}

void HRIR::JudgeDirection(Vector4f drct, Vector4f front, Vector4f up, vector<vector<float>>& hrir_s)
{
	Vector4f right = Vector4f::Cross3f(front, up);
	right.SetLenght(1.0); front.SetLenght(1.0); up.SetLenght(1.0); drct.SetLenght(1.0);
	float z = Vector4f::Dot3f(drct, up), x = Vector4f::Dot3f(drct, front), y = Vector4f::Dot3f(drct, right);
	float el = asin(z) * 180 / PI, az = atan2(y, x) * 180 / PI, L1, L2;
	int elev, azim;
	if (az < 0)az += 360;
	/*if (z < 0)el = -acos(-z) * 180 / PI;
	else el = acos(z) * 180 / PI;*/
	el = el / 10 + 4;
	az /= 5;
	if (el < 0)
	{
		elev = 0;
		L1 = 1;
	}
	else if (abs(el - round(el) < 1e-1))
	{
		elev = round(el);
		L1 = 1;
	}
	else
	{
		elev = int(el);
		L1 = elev - el + 1;
	}
	if (abs(az - round(az) < 1e-1))
	{
		azim = round(az);
		L2 = 1;
	}
	else
	{
		azim = int(az);
		L2 = azim - az + 1;
	}
	hrir_s = InterpHrir(elev, L1, azim, L2);
}

vector<vector<float>> HRIR::InterpHrir(int el, float L1, int az, float L2)
{
	vector<vector<float>> hrir;
	if (1 - L1 < 0.1)
	{
		if (1 - L2 < 0.1)
		{
			return db[el][az];
		}
		else
		{
			for (int i = 0; i < db[el][az].size(); i++)
			{
				hrir.push_back(vector<float>());
				for (int j = 0; j < 2; j++)
					hrir.back().push_back(db[el][az][i][j] * L2 
						+ db[el][az + 1][i][j] * (1 - L2));
			}
		}
	}
	else
	{
		if (1 - L2 < 0.1)
		{
			for (int i = 0; i < db[el][az].size(); i++)
			{
				hrir.push_back(vector<float>());
				for (int j = 0; j < 2; j++)
					hrir.back().push_back(db[el][az][i][j] * L1
						+ db[el + 1][az][i][j] * (1 - L1));
			}
		}
		else
		{
			for (int i = 0; i < db[el][az].size(); i++)
			{
				hrir.push_back(vector<float>());
				for (int j = 0; j < 2; j++)
					hrir.back().push_back(db[el][az][i][j] * L2 * L1
						+ db[el][az + 1][i][j] * (1 - L2) * L1
						+ db[el + 1][az][i][j] * (1 - L1) * L2
						+ db[el + 1][az + 1][i][j] * (1 - L2) * (1 - L1));
			}
		}
	}
	return hrir;
}


vector<vector<vector<vector<float>>>>
HRIR::db = vector<vector<vector<vector<float>>>>(14,
	vector < vector < vector<float>>>(80,
		vector < vector<float>>(200,
			vector<float>(2))));

WallAirAbsorb::WallAirAbsorb()
{
}

void WallAirAbsorb::LoadwithFileList(string fileName)
{
	fstream fin(fileName, ios::in);
	int matNum, last;
	string line;
	fin >> matNum;
	getline(fin, line);
	for (int i = 0; i < matNum; i++)
	{
		getline(fin, line);
		matName.push_back(line);
	}
	LoadMat(matName);
	for (int i = 0; i < matName.size(); i++)
	{
		last = matName[i].find_last_of('\\');
		matName[i] = matName[i].substr(last + 1);
	}
}

WallAirAbsorb::~WallAirAbsorb()
{
}

vector<double> WallAirAbsorb::Absorb(double dist, vector<int>& refs, vector<int>& scats, int band)
{
	// band < -1: 全频带, band == -1: 频带平均, band > -1: 单频带
	vector<double> bRef;
	if (band < -1)
		bRef = vector<double>(fq.size(), 1);
	else
		bRef = vector<double>(1, 1);
	double meanAir;
	for (auto air : attenAir)
		meanAir += pow(air, dist);
	meanAir /= attenAir.size();
	
	if (dist > 1 && band < -1)
	{
		for (int mtl = 0; mtl < refs.size(); mtl++)
			if (refs[mtl]>0)
				for (int i = 0; i < bRef.size(); i++)
					bRef[i] *= brefs[mtl][refs[mtl]][i];
		for (int mtl = 0; mtl < scats.size(); mtl++)
			if (scats[mtl]>0)
				for (int i = 0; i < bRef.size(); i++)
					bRef[i] *= srefs[mtl][scats[mtl]][i];
		for (int i = 0; i < bRef.size(); i++)
			bRef[i] *= pow(attenAir[i], dist) / dist;
	}
	else if (dist > 1 && band >= 0)
	{
		for (int mtl = 0; mtl < refs.size(); mtl++)
			if (refs[mtl]>0)
				bRef[0] *= brefs[mtl][refs[mtl]][band];
		for (int mtl = 0; mtl < scats.size(); mtl++)
			if (scats[mtl]>0)
				bRef[0] *= srefs[mtl][scats[mtl]][band];
		bRef[0] *= pow(attenAir[band], dist) / dist;
	}
	else if(dist>1)
	{
		for (int mtl = 0; mtl < refs.size(); mtl++)
			bRef[0] *= brefs[mtl][0][refs[mtl]] * srefs[mtl][0][scats[mtl]];
		bRef[0] *= meanAir / dist;
	}
	return bRef;
}

void WallAirAbsorb::Init(int ref)
{
	for (auto f : fq)
		attenAir.push_back(exp(-0.5 * 5.54e-4 * (50 / 50)*pow((f > 10000 ? 10000 : f) / 1000, 1.7)));
	int id = 0;
	for (auto alph : alpha)
	{
		vector<double> btmp;
		double meanB, meanS;
		for (auto al : alph)
			btmp.push_back(sqrt(1 - al));
		for (int i = 0; i < btmp.size();i++)
		{
			meanB += btmp[i];
			meanS += scatter[id][i];
		}
		meanB /= btmp.size();
		meanS /= btmp.size();
		brefs.push_back(vector<vector<double>>(ref + 1, vector<double>(btmp.size(), 1)));
		srefs.push_back(brefs.back());
		for (int i = 0; i < ref; i++)
		{
			for (int j = 0; j < btmp.size(); j++) // 分频带多次反射、散射系数
			{
				brefs.back()[i + 1][j] = brefs.back()[i][j] * btmp[j];
				srefs.back()[i + 1][j] = srefs.back()[i][j] * scatter[id][j];
			}
			brefs.back()[0][i + 1] = brefs.back()[0][i] * meanB; // 频带平均多次反射系数
			srefs.back()[0][i + 1] = srefs.back()[0][i] * meanS; // 频带平均多次散射系数
		}
		id++;
	}
}

void WallAirAbsorb::LoadMat(vector<string> fileNames)
{
	int matNum, last;
	string line;
	vector<float> nums;
	alpha = vector<vector<double>>();
	scatter = vector<vector<double>>();
	fq = vector<double>();
	for (int i = 0; i < fileNames.size(); i++)
	{
		fstream fin(fileNames[i], ios::in);
		getline(fin, line);
		if (i == 0)
		{
			nums = getNumbers(line);
			for (auto num : nums)
				fq.push_back(num);
		}
		getline(fin, line);
		nums = getNumbers(line);
		alpha.push_back(vector<double>());
		for (auto num : nums)
			alpha.back().push_back(num);
		getline(fin, line);
		nums = getNumbers(line);
		scatter.push_back(vector<double>());
		for (auto num : nums)
			scatter.back().push_back(num);
	}
}

vector<vector<float>> WallAirAbsorb::ConvHrir(vector<double> filter, vector<vector<float>> hrir)
{
	vector<vector<float>> hrirFiltered = vector<vector<float>>(filter.size() + hrir.size() - 1, vector<float>(2, 0));
	for (int i = 0; i < filter.size(); i++)
		for (int j = (i>=hrir.size()?i-hrir.size()+1:0); j <= i; j++)
			for (int k = 0; k < 2; k++)
				hrirFiltered[i][k] += hrir[i - j][k] * filter[j];
	for (int i = filter.size(); i < hrirFiltered.size(); i++)
		for (int j = (i >= hrir.size() ? i - hrir.size() + 1 : 0); j < filter.size(); j++)
			for (int k = 0; k < 2; k++)
				hrirFiltered[i][k] += hrir[i - j][k] * filter[j];
	double maxV = 0;
	int maxInd;
	vector<double> absHrir;
	for (int i = 0; i < hrirFiltered.size(); i++)
	{
		absHrir.push_back(abs(hrirFiltered[i][0]) + abs(hrirFiltered[i][1]));
		if (absHrir.back() > maxV)
			maxV = absHrir.back();
	}
	maxV /= 20;
	for (int i = 0; i < absHrir.size(); i++)
	{
		if (absHrir[i] > maxV)
		{
			maxInd = i;
			break;
		}
	}
	maxInd -= 100;
	if (maxInd > 0)
	{
		auto iter = hrirFiltered.begin() + maxInd;
		hrirFiltered.erase(hrirFiltered.begin(), iter);
	}
	return hrirFiltered;
}

vector<double> WallAirAbsorb::InterpIFFT(vector<COMPLEX> fqValues, vector<double> fqs)
{
	const int NRef = 512, RR = fqs.size() - 1;
	double nqst = fqs.back(), halfI, halfIS, deltaX;
	vector<double> href, fqN, window;
	fqN.push_back(0);
	fqValues.insert(fqValues.begin(), COMPLEX(0, 0));
	for (auto f : fqs)
		fqN.push_back(f / nqst);
	fqN.push_back(FS);
	fqValues.push_back(COMPLEX(0, 0));

	halfI = NRef / 2;
	halfIS = halfI - 1;
	deltaX = (fqN.back() - fqN.front()) / halfI;
	for (int i = 0; i <= NRef; i++)
		window.push_back(0.5*(1 - cos(2 * pi * i / NRef)));

	assert(fqValues.size() == fqN.size());
	int r = 0, nn = 0;
	double ndx = nn*deltaX;
	vector<COMPLEX> yy;
	yy.reserve(NRef + 1);
	while (r<RR)
	{
		while (ndx<fqN[r + 1])
		{
			yy.push_back(fqValues[r] + 1 / (fqN[r + 1] - fqN[r]) * (ndx - fqN[r])*(fqValues[r + 1] - fqValues[r]));
			nn++;
			ndx = nn*deltaX;
		}
		r++;
	}
	yy.push_back(fqValues.back());
	for (int i = halfIS; i > 0; i--)
		yy.push_back(COMPLEX(yy[i].re, -yy[i].im));
	COMPLEX *Fref, *Tref;
	Tref = (COMPLEX *)malloc(sizeof(COMPLEX)*NRef);
	Fref = (COMPLEX *)malloc(sizeof(COMPLEX)*NRef);
	assert(NRef == yy.size());
	for (int i = 0; i < NRef; i++)
	{
		Fref[i] = yy[i];
	}
	IFFT(Fref, Tref, 9);
	for (int i = halfI; i < NRef; i++)
		href.push_back(Tref[i].re);
	for (int i = 0; i <= halfI; i++)
		href.push_back(Tref[i].re);
	for (int i = 0; i < href.size(); i++)
		href[i] *= window[i];
	free(Tref);
	free(Fref);
	return vector<double>();
}

vector<string> WallAirAbsorb::matName = vector<string>(1, string("data\\matLib\\mat_scene09_concrete.csv"));
vector<vector<double>> WallAirAbsorb::alpha = { { 0.36, 0.36, 0.45, 0.51, 0.64, 0.51 } };
vector<double> WallAirAbsorb::attenAir = vector<double>();
vector<double> WallAirAbsorb::fq = {125, 250, 500, 1000, 2000, 4000};
vector<vector<vector<double>>> WallAirAbsorb::brefs = vector<vector<vector<double>>>();
vector<vector<vector<double>>> WallAirAbsorb::srefs = vector<vector<vector<double>>>();
vector<vector<double>> WallAirAbsorb::scatter = vector<vector<double>>();



Direct::Direct()
{
}

Direct::~Direct()
{
}

void Direct::LoadCSV(string fileName)
{
	string name = fileName.substr(fileName.find_last_of('\\'));
	if (type.size() > 0)
		if (type == name)
			return;
		else
			type = name;
	fstream fin(fileName, ios::in);
	COMPLEX temp;
	string line;
	getline(fin, line);
	while (getline(fin, line))
	{
		vector<string> values = splitLine(line, ',');
		string head = values.front();
		int phi, theta;
		phi = getNumbers(head.substr(1, 3)).front();
		theta = getNumbers(head.substr(5, 3)).front();
		values.erase(values.begin());
		for (auto v : values)
			if (getNumbers(v, temp)) directions[phi][theta].push_back(temp);
	}
}

vector<COMPLEX> Direct::EvalAmp(int azim, int elev)
{
	return directions[azim][90 - elev];
}

vector<COMPLEX> Direct::EvalAmp(vector<int> polar)
{
	return directions[polar[0]][polar[1]];
}

vector<vector<vector<COMPLEX>>> Direct::directions(vector<vector<vector<COMPLEX>>>(360, vector<vector<COMPLEX>>(181, vector<COMPLEX>())));
string Direct::type = string();


Orient::~Orient()
{
}

Orient::Orient(Vector4f p, Vector4f f, Vector4f u) :position(p), front(f), up(u)
{
	right = Vector4f::Cross3f(front, up);
	right.SetLenght(1.0); front.SetLenght(1.0); up.SetLenght(1.0);
}

Orient::Orient(Vector4f p)
{
	Orient(p, Vector4f(1, 0, 0), Vector4f(0, 1, 0));
}

vector<int> Orient::LocalPolar(Vector4f drct)
{
	drct.SetLenght(1.0);
	float z = Vector4f::Dot3f(drct, up), x = Vector4f::Dot3f(drct, front), y = Vector4f::Dot3f(drct, right);
	float el = asin(z) * 180 / PI, az = atan2(y, x) * 180 / PI;
	az += int(az < 0) * 360;
	return vector<int>({ int(az) ,90 - int(el) });
}
