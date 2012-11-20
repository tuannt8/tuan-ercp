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
