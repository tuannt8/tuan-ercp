#pragma once

class releaseLog
{
public:
	releaseLog(void);
	~releaseLog(void);

	static void urgentLog(char* format, ...);
	static void log(char* format, ...);
	static void init();
	static void destroy();
	static FILE* f;
};
