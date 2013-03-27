#pragma once
#include "define.h"
#include "stdafx.h"
#include "Modules/Meshfree_GPU.h"

#define C_Gravity 10

// centerline parameters
#define C_KS 100000				// Spring constant
#define C_Kr 10000				// Restore spring constant
#define C_Ba 1000				// Air damping
#define C_MASS 100
#define C_HOLE_RADIUS 20

// Collision parameters
#define C_normFactor	10000	// Normal coefficient
#define C_friction		0.1		// Friction coefficient
#define C_tighten		10000	// Friction by tightening

// Cutting
#define PROPORTION		0.1

struct collisionInfo
{
	int type;			//0 -> wire point; 1 -> centerline point
	int pWireIdx;		// Index of point in insert wire
	int segmentIdx;		// Index of segment of centerline
	Vec3d wirePoint;	// Position of point in inserting wire
	Vec3d pointOnSegment; // Point on segment (Projected from wire point)

	collisionInfo(int wireIdx, int segIdx, Vec3d wirePt, Vec3f ptOnSeg)
	{
		pWireIdx = wireIdx;
		segmentIdx =segIdx;
		wirePoint = wirePt;
		pointOnSegment = ptOnSeg;
	}
};

struct pointConstraint
{
	int nodeIdx;		// Index of simulation node in meshless model
	int segmentIdx;		// Index of segment in center line
	float lineCoord;	// Coordinate to determine projected point inside the segment
						// p = lineCoord*pt1 + (1-lineCoord)*pt2
	Vec3f directionVector;	//Vector indicate simulation node

	pointConstraint(int nIdx, int sIdx, float coord, Vec3f v)
	{
		nodeIdx = nIdx;
		segmentIdx = sIdx;
		lineCoord = coord;
		directionVector = v;
	}
};

class centerLine
{
public:
	centerLine(void);
	~centerLine(void);

	void init();
	void deform(float dt);

	void draw(int mode);
	void drawCollison();
	void drawNodeContraint();
	void drawCylinder( Vec3f a, Vec3f b, float radius, float radius2 );

	// with meshless model
	void constraintModel(EFG_CUDA_RUNTIME* object);
	void updateMeshlessContraint();
	void updateTipPosition( Vec3f newPos );

	// With catheter
	void interactWithWire(arrayVec3f wirePoints, arrayVec3f wireVelocity, int insideIdx, 
		float radius, arrayVec3f* forceToWire);

	// Private function
	void addRestoreForce();
	void addInternalSpringForce();

	float computeNormalForce( float distance, float radius, float holeRadius );
	float computeFriction( float normalForce, float wireRadius, float holeRadius );

	// Temporal variable - interact with catheter
	void detectInsertionIdx(arrayVec3f wirePoints);
	bool isPointInCylinder( Vec3f pt, Vec3f c1, Vec3f c2, float radius );
	
	float force;
	arrayVec3f m_interactionForce;
	std::vector<collisionInfo> collisionPtArray;
	int t_curInsertIdx;

	// Temporal variable: interact with meshless model
	std::vector<pointConstraint> constraintPoints;
	EFG_CUDA_RUNTIME* efgObj;

	//Variable
	int m_NbPoints;
	arrayVec3f m_Force;
	arrayVec3f m_points;
	arrayVec3f m_points0;
	arrayVec3f m_Velocity;
	std::vector<float> m_radius;
};
