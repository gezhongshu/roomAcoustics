/* This program extract the geometry of a 3D model from a file.obj file and
* reconstruct it into three formats of data including vertexs, normals and
* faces with color information.
* Written on Oct. 17th 2016 by Ge Zhongshu*/
#include<iostream>
#include<fstream>
#include<string>
#include<vector>
#include<cmath>

using namespace std;

void split(string s, string delim, vector<string>& ret)
{
	size_t last = s.find_first_not_of(delim,0);
	size_t index = s.find_first_of(delim, last);
	while (index != string::npos)
	{
		ret.push_back(s.substr(last, index - last));
		last = s.find_first_not_of(delim, index);
		index = s.find_first_of(delim, last);
	}
	if (index - last>0)
	{
		ret.push_back(s.substr(last, index - last));
	}
}

int str2int(string str)
{
	int num = 0, len = str.size();
	for (int i = 0; i < len; i++)
		num = num * 10 + (str[i] - 48);
	return num;
}

float str2float(string str) {
	float num = 0, temp = 0, sign = 1.0;
	vector<string> parts;
	if (str[0] == '-') {
		sign = -1.0;
		str.erase(0, 1);
	}
	split(str, string("."), parts);
	str = parts[0];
	for (int i = 0; i < str.size(); i++)
		num = num * 10 + (str[i] - 48);
	str = parts[1];
	for (int i = str.size() - 1; i >= 0; i--)
		temp = (temp + (str[i] - 48)) / 10;
	num += temp;
	return sign*num;
}

void printVec(ofstream &of, vector<float> vec) {
	for (int i = 0; i < vec.size(); i++)
		of.write(reinterpret_cast<const char*>(&(vec[i])), sizeof(float));
}

int main()
{
	string line;
	vector<string> lineSplit;
	vector<vector<float>> v, vn;
	vector<int> faces;
	bool isFirstV = true;
	vector<vector<float>> colors = { { 0.5843f, 0.5843f, 0.5843f, 1.0f },
									{ 0.0588f, 0.0588f, 0.0588f, 1.0f } };
	string mtl[2] = { "Matedadrial__537", "___" };
	int colorId = 0, numFV = 0;
	ifstream fin("./Resources/pia.obj",ios::in);
	ofstream foutv("./Outputs/piano_v.binary", ios::out | ios::binary);
	ofstream foutvn("./Outputs/piano_vn.binary", ios::out | ios::binary);
	ofstream foutc("./Outputs/piano_c.binary", ios::out | ios::binary);
	ofstream foutn("./Outputs/number.dat", ios::out);
	while (getline(fin, line))
	{
		if (line.size() < 2)continue;
		lineSplit.clear();
		split(line, string(" "), lineSplit);
		string type = lineSplit[0];
		int len = lineSplit.size();
		if (type == "v") {
			/*if (isFirstV) {
				v.clear();
				vn.clear();
				isFirstV = false;
			}*/
			vector<float> vtmp;
			for (int iv = 1; iv < len; iv++)
				vtmp.push_back(str2float(lineSplit[iv]) / 2e3);
			v.push_back(vtmp);
			continue;
		}
		if (type == "vn") {
			vector<float> vtmp;
			for (int iv = 1; iv < len; iv++)
				vtmp.push_back(str2float(lineSplit[iv]));
			vn.push_back(vtmp);
			continue;
		}
		if (type == "usemtl") {
			if (lineSplit[1] == mtl[0])colorId = 0;
			if (lineSplit[1] == mtl[1])colorId = 1;
			continue;
		}
		if (type == "f") {
			vector<string> inds;
			//isFirstV = true;
			for (int i = 1; i < len; i++) {
				inds.clear();
				split(lineSplit[i], string("/"), inds);
				faces.push_back(str2int(inds.front()));
				faces.push_back(str2int(inds.back()));
			}
			for (int i = 0; i < faces.size(); i += 2) {
				printVec(foutv, v[faces[i]-1]);
				printVec(foutvn, vn[faces[i + 1]-1]);
				printVec(foutc, colors[colorId]);
				numFV++;
			}
			faces.clear();
		}
	}
	foutv << flush;
	foutvn << flush;
	foutc << flush;
	foutn << numFV << endl;
	cout << "Number of vertexs of all faces:" << numFV << endl;
	std::system("pause");
	return 0;
}