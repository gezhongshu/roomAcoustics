#include "utils.h"


char isNumber(char c)
{
	if (isdigit(c) || c == '-')
		return true;
	else
		return false;
}


float *getNumbers(string text)
{
	stringstream stream;
	string tmp;
	float numbers[20];
	float *result = NULL;
	int count = 0;

	stream << text;

	while (!stream.eof()) {
		stream >> tmp;
		if (isNumber(tmp[0])) {
			numbers[count] = (float)atof(tmp.c_str());
			count++;
		}
	}

	result = new float[count];
	for (int i = 0; i < count; i++)
		result[i] = numbers[i];
	return result;
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