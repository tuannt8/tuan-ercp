#include "StdAfx.h"
#include "releaseLog.h"

FILE* releaseLog::f = NULL;

void releaseLog::log( char* format, ... )
{
	if(!f)
		return;

	va_list args;   //  Variable argument list
	int len;        // String length
	int i;          //  Iterator
	char * text;    // Text

	va_start(args, format);
	len = _vscprintf(format, args) + 1;
	text = new char[len];
	vsprintf_s(text, len, format, args);
	va_end(args);

	fprintf(f, text);
	fprintf(f, "\n");
}

void releaseLog::init()
{
	f = fopen("../DebugLog/log.txt", "w");
	FILE* ff = fopen("../DebugLog/urgentLog.txt", "w");
	fclose(ff);
}

void releaseLog::destroy()
{
	if (f)
	{
		fclose(f);
	}
}

void releaseLog::urgentLog( char* format, ... )
{
	FILE* ff = fopen("../DebugLog/urgentLog.txt", "a+");
	if(!ff)
		return;

	va_list args;   //  Variable argument list
	int len;        // String length
	int i;          //  Iterator
	char * text;    // Text

	va_start(args, format);
	len = _vscprintf(format, args) + 1;
	text = new char[len];
	vsprintf_s(text, len, format, args);
	va_end(args);

	fprintf(ff, text);
	fprintf(ff, "\n");

	fclose(ff);
}
void releaseLog::logMatrix( float* A, int row, int col, char* fileName/*="matrix_log.txt"*/ )
{
	CString path;
	path.Format("../DebugLog/%s", (char*)fileName);
	FILE* temp = fopen(path.GetBuffer(), ("w"));
	if (temp)
	{
		fprintf(temp, "Log matrix: %d x %d\n", row, col);
		for (int i=0; i<row; i++)
		{
			for(int j=0; j<col; j++)
			{
				fprintf(temp, "%lf ", A[i*col+j]);
			}
			fprintf(temp, "\n");
		}
		fclose(temp);
	}
}

void releaseLog::logMatrix( double* A, int row, int col, char* fileName/*="matrix_log.txt"*/ )
{
	logMatrix((float*)A, row, col, fileName);
}

void releaseLog::logVectorInt( std::vector<int> &v, char* fileName/*="vectorInt.txt"*/ )
{
	CString path;
	path.Format("../DebugLog/%s", (char*)fileName);
	FILE* temp = fopen(path.GetBuffer(), ("w"));
	if (temp)
	{
		fprintf(temp, "Log vector: %d\n", v.size());
		for (int i=0; i<v.size(); i++)
		{
			fprintf(temp, "%d  ", v[i]);
		}
		fclose(temp);
	}
}


