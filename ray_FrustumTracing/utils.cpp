#include "utils.h"


char isNumber(char c)
{
	if (isdigit(c) || c == '-' || c == '.' || c =='+')
		return true;
	else
		return false;
}

string getHeadNum(string text)
{
	int end = -1;
	while (++end < text.size() && isNumber(text[end]));
	return text.substr(0, end);
}

vector<float> getNumbers(string text)
{
	stringstream stream;
	string tmp;
	vector<float> result;

	stream << text;

	while (!stream.eof()) {
		stream >> tmp;
		if (isNumber(tmp[0])) {
			result.push_back((float)atof(getHeadNum(tmp).c_str()));
		}
	}
	return result;
}

bool getNumbers(string text, COMPLEX &c)
{
	int temp1, temp2;
	string real, imag;
	temp1 = text.find_last_of('+');
	temp2 = text.find_last_of('-');
	temp1 = temp1 > temp2 ? temp1 : temp2;
	if (temp1 == -1)return false;
	real = text.substr(0, temp1 - 1);
	for (string::size_type pos = real.find(' '); pos < string::npos; pos = real.find(' '))
		real.erase(pos, 1);
	c.re = stod(real);
	temp2 = text.find_last_of('i');
	imag = text.substr(temp1, temp2 - temp1);
	for (string::size_type pos = pos = imag.find(' '); pos < string::npos; pos = imag.find(' '))
		imag.erase(pos, 1);
	c.im = stod(imag);
	return true;
}

string getName(string text)
{
	size_t start, end;
	start = text.find_first_of("\"");
	end = text.find_last_of("\"");
	return text.substr(start + 1, end - start - 1);
}

string getBitmapName(string text)
{
	size_t start, end;
	start = text.find_last_of("\\");
	end = text.find_last_of("\"");
	return text.substr(start + 1, end - start - 1);
}

vector<string> splitLine(string line, char c)
{
	vector<string> res = vector<string>();
	size_t start = 0, end = line.find_first_of(c);
	while (start < line.size())
	{
		res.push_back(line.substr(start, end - start));
		start = end + 1;
		end = line.find_first_of(c, start);
		if (end > line.size()) end = line.size();
	}
	return res;
}

int strMatch(string str, vector<string> strs)
{
	int start, len = str.size() / 2 + 1;
	start = (str.size() - len);
	for (int i = 0; i < strs.size(); i++ )
		if (strs[i].find(str.substr(start, len)) != string::npos)
			return i;
	return -1;
}