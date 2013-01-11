#pragma once
#include "Graphics/surfaceobj.h"
#include "Modules/CollisionManager.h"

// ID 8
#define MIN_MOVE_DISTANCE 1
#define NEW_POINT_GAP 6

// ID 9
#define MAX_EDGE_LENGTH 30
#define ANGLE_THRES_HOLD 100*PI/180

class eLineTool
{
public:
	eLineTool(void);
	~eLineTool(void);

	void draw(int mode);
	void move(Vec3f m);
	void resetPath();

	// Idea 8
	void cut8(SurfaceObj* obj);
	void stepDebug8();
	arrayVec3f generateNewPoints8();

	arrayVec3i findConvexHull( arrayVec3f cPoints, int nbBoundaryPoints );
	void addPointToConvex(arrayVec3f cPoints, int pIdx, arrayVec3i& cFace);
	bool isDirectFace( Vec3f pt, int triIdx, arrayVec3f& cPoints, arrayVec3i& cFace );

	void addObtuseTriangle( arrayInt& bound, arrayInt &triArea );
	int findObtuseIndex( arrayInt bound, arrayInt triArea );

	// Test long cut
	void cut9(SurfaceObj* obj);
	void stepDebug9();
	void step2Debug9();
	void fillConcaveTri(arrayInt& trimmingLoop, arrayVec3i& addedFaces);
	int findConcaveIndex( arrayInt trimmingLoop );
	void triangulate( arrayInt &bound );
	bool isLegalBound( arrayInt loop );
	void remeshByFlip(arrayInt areaIdx);
	
public:
	SurfaceObj* s_surfObj;
	arrayVec3f* s_points;
	arrayVec3i* s_faces;

	arrayInt m_collidedTriIdx;
	arrayInt m_bound;
	arrayVec3i m_addedFaces;
	arrayVec3f m_addedPoints;
private:
	arrayVec3f m_prePoints;
	arrayVec3f m_curPoints;

	arrayVec3f m_allPoints;
	arrayVec3i m_face;
	void updateFaceInfo();
	

	void objTopoModifer(SurfaceObj* surfObj, arrayInt &generatedIdx, arrayInt _removedTriIdx, 
						arrayVec3i _addFace, arrayVec3f* _addPoint = NULL);

	void conterClockWiseLoop( TopologyContainer* surfTopo, arrayInt areaTriIdx, arrayInt& boundPointLoop );
	bool findBoundaryLoop(TopologyContainer* surfTopo, arrayInt idxOfRemoveTri, 
						std::vector<arrayInt>& boundLoops, arrayInt *edgeToRemove = NULL);
	float distanceToLineSeg( Vec3f pt1, Vec3f pt2 );
	float varianceAngle( arrayVec3f* points, arrayInt bound );
	float varianceAngle( arrayVec3f* points, int idx1, int idx2, int idx3 );

	bool isIntersectWithEdge( Vec3f pt1, Vec3f pt2 );
	bool isIntersectWithTri( Vec3f tri[] );

};
