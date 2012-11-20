#pragma once
#include "stdafx.h"
#include "define.h"
#include "objload.h"



class LineTool
{
public:
	LineTool(void);
	~LineTool(void);

	void init(Vec3f pt1, Vec3f pt2);
	void moveCurrentPoint(Vec3f _trans);
	void reset();

	arrayVec3f point();
	arrayVec3i face();
	arrayVec3f* frontPoint();

	void draw(int mode);

private:
	Obj::File catheter;
	arrayVec3f m_prePoint;
	arrayVec3f m_curPoint;
};
