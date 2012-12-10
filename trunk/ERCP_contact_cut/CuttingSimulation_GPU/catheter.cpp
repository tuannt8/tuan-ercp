#include "StdAfx.h"
#include "catheter.h"
#include "GL/GLAux.h"
#include "Matrix.h"
#include "mat.h"

catheter::catheter(void)
{
}

catheter::~catheter(void)
{
}

void catheter::init( Vec3f startPoint )
{
	Vec3f direct(-1,0,0);

	norm = Vec3f(0,-1,0);
	m_point.resize(NB_POINT);
	for (int i=NB_POINT-1; i>=0; i--)
	{
		m_point[i] = startPoint+direct*(SEGMENT_LENGTH*(NB_POINT-1-i));
	}

	m_linePoint.push_back(m_point[1]);
	m_linePoint.push_back(m_point[4]);
}

void catheter::draw( int mode )
{
	if (mode == 1)
	{
		glBegin(GL_LINES);
		glColor3f(1,0,0);
		for(int i=0; i<m_point.size()-1; i++)
		{
			glVertex3f(m_point[i][0], m_point[i][1], m_point[i][2]);
			glVertex3f(m_point[i+1][0], m_point[i+1][1], m_point[i+1][2]);
		}
		glColor3f(0,0,1);
		glVertex3f(m_linePoint[0][0], m_linePoint[0][1], m_linePoint[0][2]);
		glVertex3f(m_linePoint[1][0], m_linePoint[1][1], m_linePoint[1][2]);
		glEnd();

		for(int i=0; i<m_point.size()-1; i++)
		{
			glPushMatrix();
			glTranslatef(m_point[i][0], m_point[i][1], m_point[i][2]);
			GLUquadricObj *quadObj = gluNewQuadric();
			gluSphere(quadObj, 1, 5,5);
			glPopMatrix();
		}
	}
	if (mode == 2)
	{
		glColor3f(0.0,0.7,0.3);
		for (int i=0; i<m_point.size()-1; i++)
		{
			drawCylinder(m_point[i], m_point[i+1], CATHETER_RADIUS);
		}
		glColor3f(0.3,0.3,0.3);
		drawCylinder(m_linePoint[0], m_linePoint[1], STRING_RADIUS);
	}
	if (mode == 3)
	{
		glBegin(GL_LINES);
		glColor3f(1,0,0);
		for(int i=0; i<m_point.size()-1; i++)
		{
			glVertex3f(m_point[i][0], m_point[i][1], m_point[i][2]);
			glVertex3f(m_point[i+1][0], m_point[i+1][1], m_point[i+1][2]);
		}
		glColor3f(0,0,1);
		glVertex3f(m_linePoint[0][0], m_linePoint[0][1], m_linePoint[0][2]);
		glVertex3f(m_linePoint[1][0], m_linePoint[1][1], m_linePoint[1][2]);
		glEnd();

		glColor3f(0.3,0.3,0.3);
		drawCylinder(m_linePoint[0], m_linePoint[1], STRING_RADIUS);
	}
}

Matrix setUpRotationMatrix(float angle, float u, float v, float w)
{
	float L = (u*u + v * v + w * w);
	angle = angle * PI / 180.0; //converting to radian value
	float u2 = u * u;
	float v2 = v * v;
	float w2 = w * w;

	Matrix rotationMatrix;

	rotationMatrix(0,0) = (u2 + (v2 + w2) * cos(angle)) / L;
	rotationMatrix(0,1) = (u * v * (1 - cos(angle)) - w * sqrt(L) * sin(angle)) / L;
	rotationMatrix(0,2) = (u * w * (1 - cos(angle)) + v * sqrt(L) * sin(angle)) / L;
	rotationMatrix(0,3) = 0.0; 

	rotationMatrix(1,0) = (u * v * (1 - cos(angle)) + w * sqrt(L) * sin(angle)) / L;
	rotationMatrix(1,1) = (v2 + (u2 + w2) * cos(angle)) / L;
	rotationMatrix(1,2) = (v * w * (1 - cos(angle)) - u * sqrt(L) * sin(angle)) / L;
	rotationMatrix(1,3) = 0.0; 
	
	rotationMatrix(2,0) = (u * w * (1 - cos(angle)) - v * sqrt(L) * sin(angle)) / L;
	rotationMatrix(2,1) = (v * w * (1 - cos(angle)) + u * sqrt(L) * sin(angle)) / L;
	rotationMatrix(2,2) = (w2 + (u2 + v2) * cos(angle)) / L;
	rotationMatrix(2,3) = 0.0; 
	
	rotationMatrix(3,0) = 0.0;
	rotationMatrix(3,1) = 0.0;
	rotationMatrix(3,2) = 0.0;
	rotationMatrix(3,3) = 1.0;

	return rotationMatrix;
} 


void catheter::adjustStringLength( float length )
{
	float L = (m_linePoint[0]-m_linePoint[1]).norm() + length;
	
	float cosa = (L-SEGMENT_LENGTH)/(2*SEGMENT_LENGTH);

	if (cosa>1||cosa<0)
	{
		return;
	}

	float alpha = acos(cosa);
	Mat3x3f rotMat;
	rotMat.setRotationMatrix(norm,alpha);

	//point 3
	Vec3f d54 = m_point[4]-m_point[5];
	d54.normalize();

	Vec3f d43 = rotMat*d54;
	m_point[3] = m_point[4]+d43*SEGMENT_LENGTH;

	//point 2
	Vec3f d32 = rotMat*d43;
	m_point[2] = m_point[3]+d32*SEGMENT_LENGTH;
	//point 1
	Vec3f d21 = rotMat*d32;
	m_point[1] = m_point[2]+d21*SEGMENT_LENGTH;
	//point 0
	m_point[0] = m_point[1] + d21*SEGMENT_LENGTH;

	// update line
	m_linePoint[0] = m_point[1];
	m_linePoint[1] = m_point[4];
}

void catheter::drawCylinder( Vec3f a, Vec3f b, float radius )
{
	// This is the default direction for the cylinders to face in OpenGL
	Vec3f z = Vec3f(0,0,1);         
	// Get diff between two points you want cylinder along
	Vec3f p = (a - b);                               
	// Get CROSS product (the axis of rotation)
	Vec3f t = z.cross(p); 

	// Get angle. LENGTH is magnitude of the vector
	double angle = 180 / PI * acos (z*p/ p.norm());

	glPushMatrix();
	glTranslated(b[0],b[1],b[2]);
	glRotated(angle,t[0],t[1],t[2]);

	GLUquadricObj *quadratic = gluNewQuadric();
	gluCylinder(quadratic, radius, radius, p.norm(), 10, 5);
	glPopMatrix();
}

void catheter::move( Vec3f v )
{
	for (int i=0; i<m_point.size(); i++)
	{
		m_point[i] += v;
	}

	// update line
	m_linePoint[0] = m_point[1];
	m_linePoint[1] = m_point[4];
}

void catheter::rotate( float angle )
{
	Mat3x3f rotMat;
	rotMat.setRotationMatrix(norm,angle);

	for (int i=m_point.size()-2; i>=0; i--)
	{
		Vec3f curV = m_point[i]-m_point[m_point.size()-1];
		Vec3f newV = rotMat*curV;
		m_point[i] = m_point[m_point.size()-1]+newV;
	}

	// update line
	m_linePoint[0] = m_point[1];
	m_linePoint[1] = m_point[4];
}

