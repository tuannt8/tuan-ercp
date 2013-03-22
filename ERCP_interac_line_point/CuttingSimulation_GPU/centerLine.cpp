#include "StdAfx.h"
#include "centerLine.h"
#include "GL/GLAux.h"
#include "Utility.h"

centerLine::centerLine(void)
{
	force = 0;
	t_curInsertIdx = -1;
}

centerLine::~centerLine(void)
{

}

void centerLine::init()
{
	// Mass point
	float length = 100.0;
	arrayVec3f points;
// 	points.push_back(Vec3f(0,0,0));
// 	points.push_back(Vec3f(length,0,0));
// 	points.push_back(Vec3f(2*length,length,0));
// 	points.push_back(Vec3f(2*length,2*length,0));
// 	points.push_back(Vec3f(2*length,3*length,0));
// 	points.push_back(Vec3f(2*length,4*length,0));

	points.push_back(Vec3f(30,-30,7));
// 	points.push_back(Vec3f(28,0,13));
// 	points.push_back(Vec3f(22,18,13));
	points.push_back(Vec3f(-1,44,14));
	points.push_back(Vec3f(-30,50,14));
	points.push_back(Vec3f(-30,100,14));

	// Info
	m_NbPoints = points.size();
	m_Velocity.resize(m_NbPoints);
	m_Force.resize(m_NbPoints);
	
	m_points = points;
	m_points0 = points;
}

void centerLine::draw( int mode )
{
	if (mode == 1) // draw line only
	{
		glColor3f(1,0,0);
		glBegin(GL_LINES);
		for (int i=0; i<m_NbPoints-1; i++)
		{
			glVertex3f(m_points[i][0], m_points[i][1], m_points[i][2]);
			glVertex3f(m_points[i+1][0], m_points[i+1][1], m_points[i+1][2]);
		}
		glEnd();
	}

	if (mode == 1)
	{
		glColor3f(0,1,0);
		glPointSize(4.0);
		glBegin(GL_POINTS);
		for (int i=0; i<m_NbPoints; i++)
		{
			glVertex3f(m_points[i][0], m_points[i][1], m_points[i][2]);
		}
		glEnd();
	}
}

void centerLine::deform( float dt )
{
	float mass = C_MASS;
	// Add gravity
	Vec3d garavity = Vec3d(0,-C_Gravity,0);
	for (int i=0; i<m_NbPoints; i++)
	{
		m_Force[i] += garavity*mass;
	}

	// Internal force
	addInternalSpringForce();

	// Restore force
	addRestoreForce();

	// Damping force
	for (int i=0; i<m_NbPoints; i++)
	{
		m_Force[i] -= m_Velocity[i]*C_Ba;
	}

	// External force
	m_Force[0] += Vec3d(force,0,0);

	// Explicit integration
	for (int i=0; i<m_NbPoints; i++)
	{
		Vec3d dv = m_Force[i]/mass*dt;
		m_Velocity[i] += dv;
		m_points[i] += m_Velocity[i]*dt;
	}

	// Reset force
	for(int i=0;i<m_NbPoints;i++)
		m_Force[i].clear();
}

void centerLine::addRestoreForce()
{
	float kr = C_Kr;	// Restore elastic constant

	for (int i=0; i<m_NbPoints; i++)
	{
		Vec3d offsetV = m_points[i] - m_points0[i];
		Vec3d restoreForce = offsetV*(-kr) ;

		m_Force[i] = m_Force[i] + restoreForce;
	}
}

void centerLine::addInternalSpringForce()
{
	float ks = C_KS;

	for (int i=0; i<m_NbPoints-1; i++)
	{
		Vec3d p12 = m_points[i+1]-m_points[i];
		float lenghtChange = p12.norm() - (m_points0[i+1]-m_points0[i]).norm();
		p12.normalize();
		Vec3d springF = p12*ks*lenghtChange;


		m_Force[i] += springF;
		m_Force[i+1] -= springF;
	}
}

void centerLine::interactWithWire( arrayVec3f wirePoints, arrayVec3f wireVelocity, int _insideIdx, 
								  float radius , arrayVec3f* forceToWire)
{
	// Point is indexed from 0: tip of the wire
	// insideIdx mark a point in wire points. This point is last point the lie inside hole
	
	collisionPtArray.clear();
	detectInsertionIdx(wirePoints);
	int insideIdx = t_curInsertIdx;
	if (insideIdx<0)
	{
		return;
	}

	// Corresponding collision pair
	for (int i=0; i<= insideIdx; i++)
	{
		// Optimize later
		Vec3d pt = wirePoints[i];
		int idx=-1; float nearest = 9999;
		for (int i=0; i<m_points.size()-1; i++)
		{
			float distance = (pt - (m_points[i]+m_points[i+1])/2).norm();
			if (nearest > distance)
			{
				idx = i;
				nearest = distance;
			}
		}
		ASSERT(idx!=-1);

		// Collision info
		GeometricFunc func;
		Vec3f pointOnSegment;
		func.distanceBtwPointAndLine(pt, m_points[idx], m_points[idx+1], &pointOnSegment);
		collisionInfo newCollid(i, idx, pt, pointOnSegment);
		collisionPtArray.push_back(newCollid);
	
		// Force info
		float holeRadius = C_HOLE_RADIUS;
		float normalForce = computeNormalForce((pt-pointOnSegment).norm(), radius, holeRadius);
		float fricton = computeFriction(normalForce, radius, holeRadius);

		Vec3d contactForce = Vec3d();
		Vec3d normDirec = pointOnSegment - pt;
		if (normDirec.norm()>0.001)
		{
			normDirec.normalize();
			contactForce += normDirec*normalForce;
		}

		Vec3d segDirect = m_points[idx+1] - m_points[idx];
		segDirect = segDirect*(segDirect*wireVelocity[i]);
		if (segDirect.norm()>0.001)
		{
			segDirect.normalize();
			contactForce += segDirect*fricton;
		}

		// Store force to wire points
		if (forceToWire)
		{
			forceToWire->at(i) += contactForce;
		}

		// Distribute force to center points
		float segLenght = (m_points[idx+1]-m_points[idx]).norm();
		m_Force[idx] -= contactForce*((m_points[idx+1]-pointOnSegment).norm()/segLenght);
		m_Force[idx+1] -= contactForce*((m_points[idx]-pointOnSegment).norm()/segLenght);
	}
}

float centerLine::computeNormalForce( float distance, float wireRadius, float holeRadius )
{
	float k= C_normFactor;

	if (holeRadius < wireRadius)
	{
		return distance*k;
	}
	else
	{
		return distance>(holeRadius-wireRadius)? k*(distance-(holeRadius-wireRadius)):0;
	}
}

float centerLine::computeFriction( float normalForce, float wireRadius, float holeRadius )
{
	float k = C_friction; // Friction factor
	float kTighten = C_tighten;
	if (holeRadius > wireRadius)
	{
		return k*normalForce;
	}
	else
	{
		return k*normalForce + kTighten*(1-holeRadius/wireRadius);
	}
}

void centerLine::detectInsertionIdx( arrayVec3f wirePoints )
{
	Vec3d dr = m_points[1]-m_points[0];

	if (t_curInsertIdx == -1)
	{
		//check next point
		int nextIdx = t_curInsertIdx+1;
		if (nextIdx >= wirePoints.size())
			return;

		Vec3f ptd2 = wirePoints[nextIdx] - m_points[0];
		t_curInsertIdx = ptd2*dr >0? nextIdx:t_curInsertIdx;
		return;
	}

	// check current idx
	Vec3f ptd = wirePoints[t_curInsertIdx] - m_points[0];

	if (dr*ptd<0)//out
	{
		//check previous point
		if (t_curInsertIdx == 0)
		{
			t_curInsertIdx = -1;
			return;
		}
		int preIdx = t_curInsertIdx-1;
		Vec3f ptd1 = wirePoints[preIdx] - m_points[0];
		t_curInsertIdx = ptd1*dr >0? preIdx:t_curInsertIdx;
	}
	else
	{
		//check next point
		int nextIdx = t_curInsertIdx+1;
		if (nextIdx >= wirePoints.size())
			return;

		Vec3f ptd2 = wirePoints[nextIdx] - m_points[0];
		t_curInsertIdx = ptd2*dr >0? nextIdx:t_curInsertIdx;
	}
}

void centerLine::drawCollison()
{
	glColor3f(0,0,1);
	glBegin(GL_LINES);
	for (int i=0; i<collisionPtArray.size(); i++)
	{
		Vec3d pt1 = collisionPtArray[i].wirePoint;
		glVertex3f(pt1[0], pt1[1], pt1[2]);
		Vec3d pt2 = collisionPtArray[i].pointOnSegment;
		glVertex3f(pt2[0], pt2[1], pt2[2]);
	}
	glEnd();
}

void centerLine::constraintModel( EFG_CUDA_RUNTIME* object )
{
	efgObj = object;
	arrayVec3f* nodePos = efgObj->nodePos0Vec();

	// Currently we use only 1 segment for 
	// Optimize later
	arrayInt fixIdxs;
	for (int i=0; i<efgObj->nbNode(); i++)
	{
		if (isPointInCylinder((*nodePos)[i], m_points[0], m_points[1], 2*C_HOLE_RADIUS))
		{
			GeometricFunc func;
			Vec3f pointOnSeg;
			func.distanceBtwPointAndLine((*nodePos)[i], m_points[0], m_points[1], &pointOnSeg);

			float coord = (pointOnSeg-m_points[1]).norm() / (m_points[1]-m_points[0]).norm();
			Vec3f v = (*nodePos)[i] - pointOnSeg;
			pointConstraint newConstraint(i, 0, coord, v);
			constraintPoints.push_back(newConstraint);
			fixIdxs.push_back(i);
		}
	}

//	efgObj->addConstraintIdx(fixIdxs);
	// This is 2nd BC condition (Constraint position, not 1BC condition)???
}

void centerLine::updateMeshlessContraint()
{
	arrayVec3f* nodePos0 = efgObj->nodePos0Vec();
	arrayVec3f* nodePos = efgObj->nodePosVec();

	float* nodeDis = efgObj->nodeDis();

	for (int i=0; i<constraintPoints.size(); i++)
	{
		pointConstraint curC = constraintPoints[i];
		Vec3f newPos = (m_points[0]*curC.lineCoord + m_points[1]*(1-curC.lineCoord)) + curC.directionVector;

		Vec3f displacement = newPos - (*nodePos0)[curC.nodeIdx];

		nodeDis[curC.nodeIdx*3] = displacement[0];
		nodeDis[curC.nodeIdx*3+1] = displacement[1];
		nodeDis[curC.nodeIdx*3+2] = displacement[2];
	}
}

bool centerLine::isPointInCylinder( Vec3f pt, Vec3f c1, Vec3f c2, float radius )
{
	Vec3f c12 = c2-c1;
	float length = c12.norm();
	c12.normalize();
	Vec3f p1t = pt-c1;

	if ((p1t.cross(c12)).norm() < radius 
		&& p1t*c12 < length
		&& p1t*c12 > 0)
	{
		return true;
	}

	return false;
}

void centerLine::drawNodeContraint()
{
	if (!efgObj)
		return;

	GLUquadricObj *qobj = 0;
	qobj = gluNewQuadric();

	arrayVec3f* NodePosVec = efgObj->nodePosVec();

	for(int i=0;i<constraintPoints.size();i++)
	{
		glColor3f(0,0,1);
		glPushMatrix();
		glTranslatef((GLfloat)(*NodePosVec)[constraintPoints[i].nodeIdx][0],
			(GLfloat)(*NodePosVec)[constraintPoints[i].nodeIdx][1],
			(GLfloat)(*NodePosVec)[constraintPoints[i].nodeIdx][2]);
		gluSphere(qobj,1.2,20,20);
		glPopMatrix();
	}
}

