#pragma once
#include <vector>

class releaseLog
{
public:
	releaseLog(void);
	~releaseLog(void);

	static void logVectorInt(std::vector<int> &v, char* fileName="vectorInt.txt");

	static void logMatrix(double* A, int row, int col, char* fileName="matrix_log.txt");
	static void logMatrix(float* A, int row, int col, char* fileName="matrix_log.txt");

	static void urgentLog(char* format, ...);
	static void log(char* format, ...);
	static void init();
	static void destroy();
	static FILE* f;
};
