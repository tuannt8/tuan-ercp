#pragma once
#include "stdafx.h"
#include "../../include/Graphics/surfaceobj.h"
#include "../../include/DataTypes/geometricfunc.h"
#include "../../include/Graphics/TopologyContainer.h"
#include "../../include/Graphics/topologymodifier.h"

#define ANGLE_THRES_HOLD 160*PI/180 //To smooth cut boundary

class eSurfaceCutting
{
public:
	eSurfaceCutting(void);
	~eSurfaceCutting(void);

	void cutting(SurfaceObj *surf, arrayVec3f *toolPoint);

	void stepDebug();
	void stepDebug2();
	std::vector<int> changedPointIdx();
	arrayInt boundLoop();
public://Debug
	arrayInt IdxOfCollideTri;
	arrayInt addedFaceIdx;
	arrayInt boundToTriangulate;
	arrayInt alledgeRemove;
	

static arrayInt cutFaceIdx;
private:
	SurfaceObj* m_surfObj;
	arrayVec3f* m_surfPoints;
	arrayVec3i* m_surfFaces;

	arrayVec3f *m_toolPoint;
public: // But must be private
	arrayVec3f addedPoints;
	arrayVec3i addedFaces;

	arrayInt effectedPointIdx;

	void collisionBtwCyliderAndTri( arrayVec3f* surfPoints, arrayVec3i* surfFaces, 
				arrayVec3f * toolPoint, float toolRadius, arrayInt &collideIdxs );
	void independendArea( arrayInt IdxOfCollideTri, std::vector<arrayInt>& triAreas );
	void addNeighborTri(arrayInt* allTri, arrayInt* areaTri, int triIdx);
	bool findBoundaryLoop(TopologyContainer* surfTopo, arrayInt idxOfRemoveTri,  std::vector<arrayInt>& boundLoops);
	void conterClockWiseLoop(TopologyContainer* surfTopo, arrayInt areTriIdx, arrayInt& boundPointLoop);

	void fillConcaveTri(arrayInt& boundLoop);
	int findConcaveIndex( arrayInt trimmingLoop );

	//Recursive triangulate
	void triangulate(arrayInt boundLoop);

	void triangulateArea(arrayVec3f* surPoints, arrayInt& boundaryLoop, arrayVec3f* toolPoint, float toolRadius,
		arrayVec3f& addedPoints, arrayVec3i& addedFaces);
	int findEarTriangle(arrayVec3f* surPoints, arrayInt& boundaryLoop, arrayVec3f* toolPoint, float toolRadius);

	bool indexBelongtoArray(int index, arrayInt legalArray);

	void remeshArea(arrayVec3f* surPoints, arrayInt& boundaryLoop, arrayVec3f& addedPoints, arrayVec3i& addedFaces);
	bool isLegalBound( arrayInt loop );
};
