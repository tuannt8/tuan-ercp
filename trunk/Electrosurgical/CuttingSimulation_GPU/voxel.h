#pragma once
// Control a bounding volume
#include "stdafx.h"


class voxel
{
public:
	voxel(void);
	~voxel(void);

	voxel(Vec3f _leftDown, Vec3f _rightUp);

	void draw(int mode);

	bool isCollideWithObject( arrayVec3f* points, arrayVec3i* tris );
	bool isIntersectWithTri(arrayVec3f* point, Vec3i tri);

	arrayVec3f cornerPoints();

	float lengthX();
	float lengthY();
	float lengthZ();

public:
	// Volumetric info
	Vec3f leftDown;
	Vec3f rightUp;
};
