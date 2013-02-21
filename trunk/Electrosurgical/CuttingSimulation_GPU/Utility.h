#pragma once
#include "stdafx.h"

namespace Utility
{
	void printw (float x, float y, float z, char* format, ...);
	void drawBox(Vec3f leftDown, Vec3f rightUp);
	void drawFace(arrayVec3f* points, arrayVec3i* faces, int mode = 0);
	void drawFace(arrayVec3f* points, arrayVec3i* faces, arrayInt* idxToDraw);

	void urgentLog(char* format, ...);
};
