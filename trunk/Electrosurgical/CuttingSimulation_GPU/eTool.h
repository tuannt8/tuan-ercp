#pragma once
#include "stdafx.h"
#include "Modules/Meshfree_GPU.h"
#include "Graphics/surfaceobj.h"
#include "Modules/CollisionManager.h"
#include "VolumeManager.h"

#define MIN_INT -100000
#define MAX_INT 100000


#define ANGLE_THRES_HOLD 150*PI/180
#define FACE_DISTANCE 0.0

#define COS_ANGLE(A,B) A*B/(A.norm()*B.norm())

// idea 4
#define MAX_ANGLE cos(20*PI/180)
#define MAX_DISTANCE_RATIO 2.0
#define MAX_DISTANCE_TETRA 15

// idea 5
#define RADIO_TETRAHEDRAL 20

// idea 6
#define MIN_DISTANCE_TO_FACE 5

class eTool
{
public:
	eTool(void);
	~eTool(void);

	void draw(int mode = 0);
	void drawCutInfo(SurfaceObj* obj);
	void drawCutInfoDetail(SurfaceObj* obj, int mode);
	void move(Vec3f m);

	void cut2(SurfaceObj* obj); //Triangle Incremental generation
	void testAdd(); //Debug: One by one adding triangles

	void cut3(SurfaceObj* obj); //Convex hull

	void cut4(SurfaceObj* obj); //Tetra hedral incremental
	void testAdd4();	// generate new tetrahedral
	void testAdd5(Meshfree_GPU* _meshFreeObj); // Idea 5

	void cut6(SurfaceObj* obj);	// Voxel + convex hull
	void debug6(SurfaceObj* obj);
	bool stepDebug6();
	void testAdd6();

	void cut8(SurfaceObj* obj);

	//debug
	arrayInt bound;
	arrayInt newFaceIdx; //for drawing
private:
	void objTopoModifer(arrayInt _removedTriIdx, arrayVec3i _addFace, arrayVec3f* _addPoint = NULL);
	// Boundary loop
	arrayInt alledgeRemove;
	bool indexBelongtoArray( int index, arrayInt legalArray);
	void conterClockWiseLoop( TopologyContainer* surfTopo, arrayInt areaTriIdx, arrayInt& boundPointLoop );
	bool findBoundaryLoop(TopologyContainer* surfTopo, arrayInt idxOfRemoveTri, std::vector<arrayInt>& boundLoops);

	// Generate mesh
	void generate1Tri(arrayInt &bound);
	BOOL isLegalTri(Vec3i _tri);
	Vec3f generatePoint( arrayInt & bound, int* ptIdx);
	Vec3f directNorm(Vec3f ePt1, Vec3f ePt2, Vec3f pt);
	float angleCW( Vec3f  norm, Vec3f v1, Vec3f v2 );

	// idea 3: convex hull
	arrayInt pointInsideObject( arrayVec3f* toolPoints, arrayVec3i* toolTris, SurfaceObj* obj);

	arrayVec3i findConvexHull( arrayVec3f cPoints, int nbBoundaryPoints );
	void addPointToConvex(arrayVec3f cPoints, int pIdx, arrayVec3i& cFace);

	// Idea 4: tetrahedral incremental
	bool isTriIntersectWithTool(Vec3f _tri[]);
	void generateTetrahedral( int triIdx );
	// Idea 5
	void generateTetrahedralUsingEFGNode(int triIdx, Meshfree_GPU* _meshFreeObj);
	bool isDirectFace( Vec3f pt, int triIdx, arrayVec3f& cPoints, arrayVec3i& cFace );

	arrayInt mappedEfgNode;
	
	// Idea 6: Voxel
	void convexByFlip( arrayInt &_bound, arrayInt &_areTriIdx, SurfaceObj* _obj );
	void convexByRelocate(arrayInt _bound, SurfaceObj* _obj);
	VolumeManager volMng;

	arrayVec3f cPoints;
	arrayVec3i cFace;
	int nbBoundaryPoints;

	int currentPIdx;

	// For all
	arrayInt generatedIdx;
private:
	SurfaceObj knife;
	arrayVec3f points;
	arrayVec3i tris;
	arrayVec3f normPoints;

	// temporal surface info
	SurfaceObj* surfObj;
	arrayVec3f* objPoints;
	arrayVec3i* objTris;
	TopologyContainer* topo;
	int preNbPoint;

	// for cutting
	arrayInt idxOfCollidedTris;
	std::vector<arrayInt> boundLoops;
	Vec3f normBoundLoop;

	arrayVec3f addedPoints; // We may should add point to topo each time we generate new point
							// It's easier to work with point list
	arrayVec3i addedFaces;
};
