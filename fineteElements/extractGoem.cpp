/* This program extract the geometry of the human head from a rentou.obj
 * file, and reconstruct it into three formats of data including vertexs,
 * faces and lines.
 * Written on Feb. 1st 2016 by Ge Zhongshu*/
#include<iostream>
#include<fstream>
#include<string>
#include<vector>
#include<cmath>

using namespace std;

void extractVF(string line, vector<string> &vertexs, vector<string> &faces)
{
	unsigned int prei = 0, lim = line.size();
	string str;
	if (lim>1&&line.substr(0,2)=="v ")
	{
		vertexs.push_back(line);
		return;
	}
	if (lim < 2 || line.substr(0, 2) != "f ")return;
	while (prei<lim)
	{
		if (line[prei] != ' ')
		{
			prei++;
			continue;
		}
		for (int i = prei; i <= lim; i++) {
			if (i == lim || line[i] == '/')
			{
				str += line.substr(prei, i - prei);
				prei = i + 1;
				break;
			}
		}
	}
	faces.push_back(str);
}

int str2int(string str)
{
	int num = 0, len = str.size();
	for (int i = 0; i < len; i++)
	{
		num += (str[i] - 48)*pow(10, len - i - 1);
	}
	return num;
}
void face2int(string face, vector<int> &idxs)
{
	cout << "face:" << face << endl;
	unsigned int prei = 1, lim = face.size();
	for (int i = prei; i <= lim; i++) {
		if (i == lim || face[i] == ' ')
		{
			idxs.push_back(str2int(face.substr(prei, i - prei)));
			prei = i + 1;
		}
	}
}

void buildLines(vector<string> faces, vector<vector<int> > &lines)
{
	int id1, id2, tmp;
	for (int i = 0; i < faces.size(); i++)
	{
		vector<int> idxs;
		face2int(faces[i], idxs);
		for (int j = 0; j < idxs.size(); j++)
		{
			id1 = idxs[j];
			id2 = idxs[(j + 1) % idxs.size()];
			tmp = id1 + id2;
			id1 = id1 > id2 ? id2 : id1;
			id2 = tmp - id1;
			if (lines[id1].size() == 0)
			{
				lines[id1].push_back(id2);
				//cout << "add line:" << id1 << "  " << id2 << endl;
			}
			for (int k = 0; k < lines[id1].size(); k++)
			{
				if (id2 != lines[id1][k] && k < lines[id1].size() - 1)continue;
				else if (id2 == lines[id1][k]) break;
				else lines[id1].push_back(id2);
				//cout << "add line:" << id1 << "  " << id2 << endl;
			}
		}
	}
}

int main()
{
	string line, namehead="./resources/part", namend = ".txt";
	vector<string> vertexs;
	int szV, szF, szL;
	//ifstream fin("./resources/rentou.txt",ios::in);
	for (int ifile = 1; ifile < 5; ifile++)
	{
		string finname, fid;
		vector<string> faces;
		fid = char(ifile + 48);
		finname = namehead + fid + namend;
		ifstream fin(finname, ios::in);
		while (getline(fin, line))
		{
			extractVF(line, vertexs, faces);
		}
		szV = vertexs.size();
		szF = faces.size();
		szL = (szV + szF + 20);
		vector<vector<int> > lines(szL);
		buildLines(faces, lines);
		ofstream foutv("./outputs/vertexs" + fid + namend, ios::out);
		ofstream foutl("./outputs/lines" + fid + namend, ios::out);
		ofstream foutf("./outputs/faces" + fid + namend, ios::out);
		for (int i = 0; i < vertexs.size(); i++)
			foutv << vertexs[i] << '\n';
		for (int i = 0; i < lines.size(); i++)
			for (int j = 0; j < lines[i].size(); j++)
				foutl << i << '\t' << lines[i][j] << '\n';
		for (int i = 0; i < faces.size(); i++)
			foutf << faces[i] << '\n';
		foutv << flush;
		foutl << flush;
		foutf << flush;
		fin.close();
		foutv.close();
		foutl.close();
		foutf.close();
	}
	system("pause");
	return 0;
}