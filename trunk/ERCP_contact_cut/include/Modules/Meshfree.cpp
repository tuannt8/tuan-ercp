#include "stdafx.h"
#include "Meshfree.h"

Meshfree::Meshfree(void)
{
	SurfObj=NULL;
	EFGObj=NULL;
}

Meshfree::~Meshfree(void)
{
	if(SurfObj)
		delete SurfObj;
	if(EFGObj)
		delete EFGObj;
}

void Meshfree::loadSurfObj(char* filename)
{
	SurfObj=new SurfaceObj;
	SurfObj->readObjData(filename);
}

void Meshfree::generateNode(int res)
{
	NodeGenerator.setSurfObj(SurfObj);
	NodeGenerator.generateUniformNode(res);

	// 1. Init topology information
	TopologyCtr.init(SurfObj, NodeGenerator.nodePos(), NodeGenerator.boxSize());
}

void Meshfree::drawSurfObj(Vec3f color, int mode)
{
	if(mode==0)
		SurfObj->drawObject(color);
	if(mode==1)
		SurfObj->drawWireFrame(color);
}

void Meshfree::drawMeshfreeNode(Vec3f color, float radius, int mode)
{
	GLUquadricObj *qobj = 0;
	qobj = gluNewQuadric();

	std::vector<Vec3f>* nodePos=TopologyCtr.nodePos();
	if(mode==0)
	{
		for(int i=0;i<nodePos->size();i++)
		{
			glPushMatrix();
			glTranslatef((*nodePos)[i][0], (*nodePos)[i][1], (*nodePos)[i][2]);
			gluSphere(qobj, radius, 20, 20);
			glPopMatrix();
		}
	}
	if(mode==1)
	{
		glPointSize(radius);
		glBegin(GL_POINTS);
		for(int i=0;i<nodePos->size();i++)
			glVertex3f((*nodePos)[i][0], (*nodePos)[i][1], (*nodePos)[i][2]);
		glEnd();
	}
}

void Meshfree::drawNodeBox(Vec3f color, float radius)
{
	TopologyCtr.drawNodeBox(color, radius);
}

void Meshfree::printNbNeighbor()
{
	TopologyCtr.printNbNeighbor();
}