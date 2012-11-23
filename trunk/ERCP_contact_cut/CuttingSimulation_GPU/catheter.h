#pragma once
#include "define.h"
#include "stdafx.h"
#define SEGMENT_LENGTH 70
#define CATHETER_RADIUS 10
#define STRING_RADIUS 3

#define NB_POINT 6

class catheter
{
public:
	catheter(void);
	~catheter(void);

	void draw(int mode);
	void init(Vec3f startPoint);

	arrayVec3f& catheterPoint(){return m_point;};
	arrayVec3f& stringPoint(){return m_linePoint;}

	arrayVec3f* toolPoint(){return &m_linePoint;}

	double catheterRadius(){return CATHETER_RADIUS;}
	float stringRadius(){return STRING_RADIUS;}

	void adjustStringLength(float length);
	void move(Vec3f v);
	void rotate(float angle);
private:
	void drawCylinder(Vec3f pt1, Vec3f pt2, float radius);

private:
	arrayVec3f m_point;
	arrayVec3f m_linePoint;

	Vec3d norm;
};
