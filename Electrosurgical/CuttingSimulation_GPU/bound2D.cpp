#include "StdAfx.h"
#include "bound2D.h"
#include "GL/GLAux.h"

bound2D::bound2D(void)
{
	double dist = 50;
	int nbPoint = 10;

	Vec3d direct = Vec3d(0,1,0);
	Vec3d initPos = Vec3d(0,0,0) + direct*(dist*nbPoint/2);
	for (int i=0; i<nbPoint; i++)
	{
		points.push_back(initPos - direct*(i*dist));
	}

	needlePoints.push_back(Vec3d(10,0,0));
	needlePoints.push_back(Vec3d(300,0,0));
	radius = 10;
	generateNeedleDraw();
}

bound2D::~bound2D(void)
{
}

void bound2D::draw( int mode )
{
	// Draw object boundary
	glColor3f(0,0,1);
	glBegin(GL_LINES);
	for (int i=0; i<points.size()-1; i++)
	{
		glVertex3f(points[i][0], points[i][1], points[i][2]);
		glVertex3f(points[i+1][0], points[i+1][1], points[i+1][2]);
	}
	glEnd();

	glColor3f(1,0,0);
	glPointSize(2.0);
	glBegin(GL_POINTS);
	for (int i=0; i<points.size(); i++)
	{
		glVertex3f(points[i][0], points[i][1], points[i][2]);
	}
	glEnd();

	// Draw needle
	glColor3f(0,1,0);
	glBegin(GL_LINES);
	for (int i=0; i<needeleForDraw.size(); i++)
	{
		glVertex3f(needeleForDraw[i][0], needeleForDraw[i][1], needeleForDraw[i][2]);
		int nextIdx = (i+1)%(needeleForDraw.size());
		glVertex3f(needeleForDraw[nextIdx][0], needeleForDraw[nextIdx][1], needeleForDraw[nextIdx][2]);
	}
	glEnd();
}

void bound2D::moveTool( Vec3d m )
{
	for (int i=0; i<needlePoints.size(); i++)
	{
		needlePoints[i] += m;
	}

	generateNeedleDraw();
}

void bound2D::generateNeedleDraw()
{
	needeleForDraw.push_back(needlePoints[0]-Vec3d(0,radius,0));
	needeleForDraw.push_back(needlePoints[0]+Vec3d(0,radius,0));
	needeleForDraw.push_back(needlePoints[1]+Vec3d(0,radius,0));
	needeleForDraw.push_back(needlePoints[1]-Vec3d(0,radius,0));
}

void bound2D::cut()
{
	
}

void bound2D::intersectionResolve()
{
	
}

void bound2D::generatePoint()
{
	
}
