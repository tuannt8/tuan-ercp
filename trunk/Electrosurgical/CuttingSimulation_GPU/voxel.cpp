#include "StdAfx.h"
#include "voxel.h"
#include "box_tri_test.h"
#include "Utility.h"

voxel::voxel(void)
{
	leftDown = Vec3f();
	rightUp = Vec3f();
}

voxel::voxel( Vec3f _leftDown, Vec3f _rightUp )
{
	leftDown = _leftDown;
	rightUp = _rightUp;
}

voxel::~voxel(void)
{
}

float voxel::lengthX()
{
	return rightUp[0]-leftDown[0];
}

float voxel::lengthY()
{
	return rightUp[1]-leftDown[1];
}

float voxel::lengthZ()
{
	return rightUp[2]-leftDown[2];
}

bool voxel::isIntersectWithTri( arrayVec3f* point, Vec3i tri )
{
	// Need optimization - store as member

	float center[3];
	float halfSize[3];
	float triVer[3][3];

	Vec3f v_center = (rightUp+leftDown)/2.0;
	center[0]=v_center[0];
	center[1]=v_center[1];
	center[2]=v_center[2];

	halfSize[0] = lengthX()/2.0;
	halfSize[1] = lengthY()/2.0;
	halfSize[2] = lengthZ()/2.0;

	for (int i=0; i<3; i++)
	{

		for (int j=0; j<3; j++)
		{
			triVer[i][j] = point->at(tri[i])[j];
		}
	}

	int collide = box_tri::triBoxOverlap(center, halfSize, triVer);
	return collide != 0;
}

bool voxel::isCollideWithObject( arrayVec3f* points, arrayVec3i* tris )
{
	// Need optimization
	for(int i=0; i<tris->size(); i++)
	{
		if (isIntersectWithTri(points, tris->at(i)))
		{
			return true;
		}
	}
	return false;
}

void voxel::draw( int mode )
{
	Utility::drawBox(leftDown, rightUp);
}

arrayVec3f voxel::cornerPoints()
{
	Vec3f diagonal = rightUp - leftDown;

	arrayVec3f cornerP;
	for (int i=0; i<3; i++)
	{
		Vec3f tempD;
		tempD[i] = diagonal[i];

		cornerP.push_back(leftDown + tempD);
		cornerP.push_back(rightUp-tempD);
	}

	cornerP.push_back(leftDown);
	cornerP.push_back(rightUp);

	return cornerP;
}
