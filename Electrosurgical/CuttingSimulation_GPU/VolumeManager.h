#pragma once
#include "voxel.h"
#include "Graphics/surfaceobj.h"

#define VOXEL_SIZE 10
#define ELARGE_LENGTH 0.1*VOXEL_SIZE

class VolumeManager
{
public:
	VolumeManager(void);
	~VolumeManager(void);

	void init(arrayVec3f* points, arrayVec3i* tris);
	void draw(int mode);

	void collisionDetectionWithObject( SurfaceObj* obj, arrayInt& idxOfCollidedTris );
	arrayVec3f pointsInsideObj( SurfaceObj* obj );

public:
	bool isCollidWithTri(arrayVec3f*points, Vec3i tri);
	voxel element(int ix, int iy, int iz);

	void setValue(int ix, int iy, int iz, int _value);
	int getValue(int ix, int iy, int iz);


	// big volume size
	voxel m_volume;

	int *m_voxelValue;

	int m_numX, m_numY, m_numZ;
};
