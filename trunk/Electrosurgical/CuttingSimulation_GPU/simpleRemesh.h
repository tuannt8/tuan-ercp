#pragma once
#include "DataTypes/vectorfunc.h"
#include "DataTypes/geometricfunc.h"
#include "Graphics/TopologyContainer.h"
#include "../include/Graphics/topologymodifier.h"
#include "../include/Graphics/surfaceobj.h"
#include "../include/Modules/Meshfree_GPU.h"



#define MAX_EDGE_LENGTH 50

class simpleRemesh
{

public:
	simpleRemesh(void);
	~simpleRemesh(void);

	void remeshArea(SurfaceObj* obj, arrayInt& remeshAreaIdxs, float res);


	void smoothArea(SurfaceObj* obj, arrayInt remeshAreaIdxs);
	float maxEdgeLength(SurfaceObj* obj, arrayInt remeshAreaIdxs);
	void remesh(SurfaceObj* obj, arrayInt remeshAreaIdxs, float maxEdgeLength = MAX_EDGE_LENGTH);

	void removeEarTri( SurfaceObj* obj, arrayInt& triArea );
	void updateShapeFunc(Meshfree_GPU* obj_GPU, arrayInt pointIdx);
	void addNeighbor(Meshfree_GPU* obj_GPU, arrayInt& neighbor, arrayInt &potentialNeibor, int pIdx);//recursive
public:
	void delaunayTriangulate(arrayInt allPtIdx_org, arrayVec3i& newFace, Vec3f norm);
	bool isNeighbor(arrayInt allPtIdx, int idx1, int idx2);

	bool isCCWOrder(arrayInt polygonIdxs, Vec3f normF);
	bool isCCWOrder(Vec3i tri, Vec3f normF);
	bool isEarTri(arrayInt boundLoop, Vec3i pt, Vec3f normF);
	void convertToGloabalIdx(arrayVec3i& face, arrayInt& pairIdx);

	float varAngle(Vec3i tri);
	void roughFaceProcess( SurfaceObj* obj, arrayInt areaTri, float rough );

	arrayInt newFaceIdx;
	arrayInt newPointIdx;

public:
	void refineMesh(SurfaceObj* obj, arrayInt* refineArea);

	// Internal functions
	bool isSharpCorner(arrayVec3f* vPoints, Vec3i tri, int &pIdx);
	void objTopoModifer(SurfaceObj* surfObj, arrayInt &generatedIdx, arrayInt _removedTriIdx, arrayVec3i _addFace, arrayVec3f* _addPoint = NULL);
	bool findBoundaryLoop( TopologyContainer* surfTopo, arrayInt idxOfRemoveTri, std::vector<arrayInt>& boundLoops, arrayInt *edgeToRemove);
	void conterClockWiseLoop( TopologyContainer* surfTopo, arrayInt areaTriIdx, arrayInt& boundPointLoop );
	int findOppositeEdgeIdx( SurfaceObj* obj, int triIdx, int pIdx );
	void collapseEdge( SurfaceObj* obj, int edgeIdx, arrayInt& removeIdxs, arrayVec3i& addedFace );
private:
	arrayVec3f addedPoint0;
	arrayVec3f addedPoint;
	arrayVec3i addedFace;
	arrayVec3i addedFaceAround;
	int preNbPoints;

	std::vector<arrayInt> addedPtOnEdge;
	arrayInt removedEdge;
	// surface info
	arrayVec3f* points;


};
