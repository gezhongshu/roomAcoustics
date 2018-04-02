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

vector<double> WallAirAbsorb::Absorb(double dist, int ref)
{
	const int NRef = 512, RR = fq.size() - 1;
	double nqst = fq.back(), halfI, halfIS, deltaX;
	vector<double> href, fqN, window, bRef = bref[ref];
	for (auto f : fq)
		fqN.push_back(f / nqst);
	halfI = NRef / 2;
	halfIS = halfI - 1;
	deltaX = (fqN.back() - fqN.front()) / halfI;
	for (int i = 0; i <= NRef; i++)
		window.push_back(0.5*(1 - cos(2 * pi * i / NRef)));
	if (dist > 1)
		for (int i = 0; i < bRef.size(); i++)
			bRef[i] *= pow(attenAir[i], dist) / dist;
	assert(bRef.size() == fqN.size());
	int r = 0, nn = 0;
	double ndx = nn*deltaX;
	vector<double> yy;
	yy.reserve(NRef + 1);
	while (r<RR)
	{
		while (ndx<fqN[r+1])
		{
			yy.push_back(bRef[r] + (ndx - fqN[r])*(bRef[r + 1] - bRef[r]) / (fqN[r + 1] - fqN[r]));
			nn++;
			ndx = nn*deltaX;
		}
		r++;
	}
	yy.push_back(bRef.back());
	for (int i = halfIS; i > 0; i--)
		yy.push_back(yy[i]);
	COMPLEX *Fref, *Tref;
	Tref = (COMPLEX *)malloc(sizeof(COMPLEX)*NRef);
	Fref = (COMPLEX *)malloc(sizeof(COMPLEX)*NRef);
	assert(NRef == yy.size());
	for (int i = 0; i < NRef; i++)
	{
		Fref[i].re = yy[i];
		Fref[i].im = 0;
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
	return href;
}

void WallAirAbsorb::Init(int ref)
{
	vector<double> btmp;
	for (auto al : alpha[0])
		btmp.push_back(sqrt(1 - al));
	for (auto f : fq)
		attenAir.push_back(exp(-0.5 * 5.54e-4 * (50 / 50)*pow((f > 10000 ? 10000 : f)/1000, 1.7)));
	bref = vector<vector<double>>(ref + 1, vector<double>(btmp.size(), 1));
	for (int i = 0; i < ref; i++)
		for (int j = 0; j < btmp.size(); j++)
			bref[i + 1][j] = bref[i][j] * btmp[j];

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

vector<string> WallAirAbsorb::matName = vector<string>(1, string("data\\matLib\\mat_scene09_concrete.csv"));
vector<vector<double>> WallAirAbsorb::alpha = { { 0.36, 0.36, 0.36, 0.45, 0.51, 0.64, 0.51, 0.51 } };
vector<double> WallAirAbsorb::attenAir = vector<double>();
vector<double> WallAirAbsorb::fq = {0, 125, 250, 500, 1000, 2000, 4000, 24e3};
vector<vector<double>> WallAirAbsorb::bref = vector<vector<double>>();
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
