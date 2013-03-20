#pragma once
#include "define.h"
#include "stdafx.h"

#define C_Gravity 10

// centerline parameters
#define C_KS 100000				// Spring constant
#define C_Kr 1000				// Restore spring constant
#define C_Ba 10				// Air damping
#define C_MASS 100

// Collision parameters
#define C_normFactor 1000	// Normal coefficient
#define C_friction		0.1	// Friction coefficient
#define C_tighten		100 // Friction by tightening

struct collisionInfo
{

	int pWireIdx;
	int segmentIdx;
	Vec3d wirePoint;
	Vec3d pointOnSegment;

	collisionInfo(int wireIdx, int segIdx, Vec3d wirePt, Vec3f ptOnSeg)
	{
		pWireIdx = wireIdx;
		segmentIdx =segIdx;
		wirePoint = wirePt;
		pointOnSegment = ptOnSeg;
	}
};

class centerLine
{
public:
	centerLine(void);
	~centerLine(void);

	void init();
	void draw(int mode);
	void drawCollison();
	void deform(float dt);

	void detectInsertionIdx(arrayVec3f wirePoints);
	void interactWithWire(arrayVec3f wirePoints, arrayVec3f wireVelocity, int insideIdx, float radius);

	// Private function
	void addRestoreForce();
	void addInternalSpringForce();

	float computeNormalForce( float distance, float radius, float holeRadius );
	float computeFriction( float normalForce, float wireRadius, float holeRadius );
	// Temporal variable
	float force;
	arrayVec3f m_interactionForce;
	std::vector<collisionInfo> collisionPtArray;
	int t_curInsertIdx;

	//Variable
	int m_NbPoints;
	arrayVec3f m_Force;
	arrayVec3f m_points;
	arrayVec3f m_points0;
	arrayVec3f m_Velocity;
	std::vector<float> m_radius;
};
