#pragma once
#include "stdafx.h"

class bound2D
{
public:
	bound2D(void);
	~bound2D(void);

	void draw(int mode);

	void moveTool(Vec3d m);
	void cut();
private:
	int ptIdx1, ptIdx2;
	void intersectionResolve();
	void generatePoint();

	arrayVec3d points; // Boundary points

	arrayVec3d needlePoints;
	double radius;
	void generateNeedleDraw();
	arrayVec3d needeleForDraw;
};
