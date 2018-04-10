#ifndef UTILS_H
#define UTILS_H

#include <string>
#include <sstream>
#include <vector>
#include "mathdefs.h"

using namespace std;



char isNumber(char c);
string getHeadNum(string text);
vector<float> getNumbers(string text);
bool getNumbers(string text, COMPLEX &c);
string getName(string text);
string getBitmapName(string text);
vector<string> splitLine(string line, char c);
int strMatch(string str, vector<string> strs);

#endif // !UTILS_H