#pragma once
#include "define.h"
#include "stdafx.h"
#include "Modules/Meshfree_GPU.h"

#define MAX 100000000
#define MIN -100000000

class cuttingTest
{
public:
	cuttingTest(void);
	~cuttingTest(void);

	void draw(int mode = 0);

	bool cutPapilla(Vec3f sPt, Vec3f cPt1, Vec3f cPt2, // Cut front info: end point of string and 2 points in centerline
					Meshfree_GPU* obj); // Object info


	bool cutSurface( Vec3f sPt, Vec3f cPt1, Vec3f cPt2, SurfaceObj* surf );
	void cutSurStep2();

	void updateConnection();
private:
	bool findBoundaryLoop( TopologyContainer* surfTopo, arrayInt idxOfRemoveTri, std::vector<arrayInt>& boundLoops, arrayInt *edgeToRemove = NULL);
	void conterClockWiseLoop( TopologyContainer* surfTopo, arrayInt areaTriIdx, arrayInt& boundPointLoop );
	void objTopoModifer(SurfaceObj* surfObj, arrayInt &generatedIdx, arrayInt _removedTriIdx, arrayVec3i _addFace, arrayVec3f* _addPoint /*= NULL*/ );
	arrayVec3i findConvexHull( arrayVec3f cPoints, int nbBoundaryPoints );
	void addPointToConvex( arrayVec3f _cPoints, int pIdx, arrayVec3i& _cFace );
	bool isDirectFace( Vec3f pt, int triIdx, arrayVec3f& cPoints, arrayVec3i& cFace );
	void addNeighbor( Meshfree_GPU* obj_GPU, arrayInt& neighbor, arrayInt &potentialNeibor, int pIdx );
	// For step debug
	arrayInt m_collidedTriIdx;
	SurfaceObj* s_surfObj;
	arrayVec3f* s_points;
	Meshfree_GPU* Obj_GPU;

	Vec3f newPt;
	arrayInt m_newFIdxs;

};
