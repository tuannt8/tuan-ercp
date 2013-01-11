#include "StdAfx.h"
#include "VolumeManager.h"
#include "Modules/CollisionManager.h"

VolumeManager::VolumeManager(void)
{
	m_voxelValue = NULL;
	m_numX = 0;
	m_numY = 0;
	m_numZ = 0;
}

VolumeManager::~VolumeManager(void)
{
	if (m_voxelValue)
	{
		delete m_voxelValue;
	}
}

void VolumeManager::init( arrayVec3f* points, arrayVec3i* tris )
{
	m_volume = voxel();
	// determine the whole size
	for (int i=0; i<points->size(); i++)
	{
		Vec3f curP = points->at(i);
		for (int i=0; i<3; i++)
		{
			if (m_volume.leftDown[i] > curP[i])
				m_volume.leftDown[i] = curP[i];
			if (m_volume.rightUp[i] < curP[i])
				m_volume.rightUp[i] = curP[i];
		}
	}
	// enlarge it
	Vec3f elarge(ELARGE_LENGTH, ELARGE_LENGTH, ELARGE_LENGTH);
	m_volume.leftDown -= elarge;
	m_volume.rightUp += elarge;

	// Determine number of voxels
	m_numX = ceil(m_volume.lengthX()/VOXEL_SIZE);
	m_numY = ceil(m_volume.lengthY()/VOXEL_SIZE);
	m_numZ = ceil(m_volume.lengthZ()/VOXEL_SIZE);

	m_voxelValue = new int[m_numX*m_numY*m_numZ];

	// Init voxel value
	for (int i=0; i<m_numX; i++)
	{
		for (int j=0; j<m_numY; j++)
		{
			for (int k=0; k<m_numZ; k++)
			{
				voxel curVol = element(i,j,k);
				if (curVol.isCollideWithObject(points, tris))
				{
					setValue(i,j,k,1);
				}
				else
					setValue(i,j,k,0);
			}
		}
	}
}

voxel VolumeManager::element( int ix, int iy, int iz )
{
	Vec3f _leftDown, _rightUp;

	_leftDown = m_volume.leftDown;
	_leftDown[0] += m_volume.lengthX()*ix/m_numX;
	_leftDown[1] += m_volume.lengthY()*iy/m_numY;
	_leftDown[2] += m_volume.lengthZ()*iz/m_numZ;

	_rightUp = _leftDown;
	_rightUp[0] += m_volume.lengthX()/m_numX;
	_rightUp[1] += m_volume.lengthY()/m_numY;
	_rightUp[2] += m_volume.lengthZ()/m_numZ;

	return voxel(_leftDown, _rightUp);
}

void VolumeManager::setValue( int ix, int iy, int iz, int _value )
{
	m_voxelValue[ix*m_numY*m_numZ + iy*m_numZ + iz] = _value;
}

int VolumeManager::getValue( int ix, int iy, int iz )
{
	return m_voxelValue[ix*m_numY*m_numZ + iy*m_numZ + iz]; 
}

void VolumeManager::draw( int mode )
{
	for (int i=0; i<m_numX; i++)
	{
		for (int j=0; j<m_numY; j++)
		{
			for (int k=0; k<m_numZ; k++)
			{
				if (getValue(i,j,k)==1)
				{
					voxel curVol = element(i,j,k);
					glColor3f(i,j,k);
					curVol.draw(mode);
				}
			}
		}
	}
}

void VolumeManager::collisionDetectionWithObject( SurfaceObj* obj, arrayInt& idxOfCollidedTris )
{
	arrayVec3f* points = obj->point();
	arrayVec3i* tris = obj->face();
	// Optimize with BVH tree later
	for (int i=0; i<tris->size(); i++)
	{
		if (isCollidWithTri(points, tris->at(i)))
		{
			idxOfCollidedTris.push_back(i);
		}
	}
}

bool VolumeManager::isCollidWithTri( arrayVec3f*points, Vec3i tri )
{
	for (int i=0; i<m_numX; i++)
	{
		for (int j=0; j<m_numY; j++)
		{
			for (int k=0; k<m_numZ; k++)
			{
				if (getValue(i,j,k) == 1)
				{
					voxel curV = element(i,j,k);
					if (curV.isIntersectWithTri(points, tri))
					{
						return true;
					}
				}
			}
		}
	}	
	return false;
}

arrayVec3f VolumeManager::pointsInsideObj( SurfaceObj* obj )
{
	CollisionManager colMng;
	arrayVec3f ptInside;
	for (int i=0; i<m_numX; i++)
	{
		for (int j=0; j<m_numY; j++)
		{
			for (int k=0; k<m_numZ; k++)
			{
				if (getValue(i,j,k) == 1)
				{
					voxel curV = element(i,j,k);
					arrayVec3f cornerP = curV.cornerPoints();
					for (int l=0; l<cornerP.size(); l++)
					{
						if (colMng.isPointInSurfObj(obj, cornerP[l]))
						{
							ptInside.push_back(cornerP[l]);
						}
					}
				}
			}
		}
	}	

	// process identical points
	arrayVec3f uniqePt;
	for (int i=0; i<ptInside.size(); i++)
	{
		Vec3f curP = ptInside[i];
		bool exist = false;
		for (int j=0; j<uniqePt.size(); j++)
		{
			if ((curP-uniqePt[j]).norm() < EPS)
			{
				exist = true;
				break;
			}
		}
		if (!exist)
		{
			uniqePt.push_back(curP);
		}
	}

	return uniqePt;
}
