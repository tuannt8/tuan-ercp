#include "stdafx.h"
#include "CuttingTool.h"

CuttingTool::CuttingTool(void)
{
	BVHAABB=NULL;
	Container=NULL;
}

CuttingTool::~CuttingTool(void)
{
	if(BVHAABB)
		delete BVHAABB;
	if(Container)
		delete Container;
}

void CuttingTool::setCutFront(std::vector<Vec3f>* cutFront)
{
	CutFront.resize(cutFront->size());
	CutEnd.resize(cutFront->size());
	CutPoint.resize(cutFront->size()*2);

	for(int i=0;i<cutFront->size();i++)
	{
		CutFront[i]=(*cutFront)[i];
		Point.push_back(CutFront[i]);
	}
	generateFace();
}
void CuttingTool::trans(Vec3f _trans)
{
	for(int i=0;i<CutFront.size();i++)
	{
		CutEnd[i]=CutFront[i];
		CutFront[i]+=_trans;

		CutPoint[i]=CutFront[i];
		CutPoint[i+CutFront.size()]=CutEnd[i];
	}
	computeCutFaceNormal();

	int preNbPoint=Point.size()-CutFront.size();
	for(int i=0;i<CutFront.size();i++)
	{
		Point.push_back(CutFront[i]);
	}
	for(int i=0;i<CutFace.size();i++)
	{
		Vec3i face;
		face[0]=CutFace[i][0]+preNbPoint;
		face[1]=CutFace[i][1]+preNbPoint;
		face[2]=CutFace[i][2]+preNbPoint;
		Face.push_back(face);
		Normal.push_back(CutFaceNormal[i]);
	}
}
void CuttingTool::transTotal(Vec3f _trans)
{
	for(int i=0;i<CutFront.size();i++)
	{
		CutEnd[i]=CutFront[i];
		CutFront[i]+=_trans;

		CutPoint[i]=CutFront[i];
		CutPoint[i+CutFront.size()]=CutEnd[i];
	}
	computeCutFaceNormal();

	int preNbPoint=PointTotal.size()-CutFront.size();
	for(int i=0;i<CutFront.size();i++)
	{
		PointTotal.push_back(CutFront[i]);
	}
	for(int i=0;i<CutFace.size();i++)
	{
		Vec3i face;
		face[0]=CutFace[i][0]+preNbPoint;
		face[1]=CutFace[i][1]+preNbPoint;
		face[2]=CutFace[i][2]+preNbPoint;
		FaceTotal.push_back(face);
		NormalTotal.push_back(CutFaceNormal[i]);
	}
}

void CuttingTool::drawCutSurf(Vec3f color, int mode)
{
	glColor3f(color[0],color[1],color[2]);
	if(mode==0)
	{
		glBegin(GL_TRIANGLES);
		for(int i=0;i<CutFace.size();i++)
		{
			glNormal3f(CutFaceNormal[i][0],CutFaceNormal[i][1],CutFaceNormal[i][2]);
			glVertex3f(CutPoint[CutFace[i][0]][0],CutPoint[CutFace[i][0]][1],CutPoint[CutFace[i][0]][2]);
			glVertex3f(CutPoint[CutFace[i][1]][0],CutPoint[CutFace[i][1]][1],CutPoint[CutFace[i][1]][2]);
			glVertex3f(CutPoint[CutFace[i][2]][0],CutPoint[CutFace[i][2]][1],CutPoint[CutFace[i][2]][2]);

			glNormal3f(-CutFaceNormal[i][0],-CutFaceNormal[i][1],-CutFaceNormal[i][2]);
			glVertex3f(CutPoint[CutFace[i][0]][0],CutPoint[CutFace[i][0]][1],CutPoint[CutFace[i][0]][2]);
			glVertex3f(CutPoint[CutFace[i][2]][0],CutPoint[CutFace[i][2]][1],CutPoint[CutFace[i][2]][2]);
			glVertex3f(CutPoint[CutFace[i][1]][0],CutPoint[CutFace[i][1]][1],CutPoint[CutFace[i][1]][2]);
		}
		glEnd();
	}

	else
	{
		glBegin(GL_LINES);
		for(int i=0;i<CutFace.size();i++)
		{
			glVertex3f(CutPoint[CutFace[i][0]][0],CutPoint[CutFace[i][0]][1],CutPoint[CutFace[i][0]][2]);
			glVertex3f(CutPoint[CutFace[i][1]][0],CutPoint[CutFace[i][1]][1],CutPoint[CutFace[i][1]][2]);
			glVertex3f(CutPoint[CutFace[i][1]][0],CutPoint[CutFace[i][1]][1],CutPoint[CutFace[i][1]][2]);
			glVertex3f(CutPoint[CutFace[i][2]][0],CutPoint[CutFace[i][2]][1],CutPoint[CutFace[i][2]][2]);
			glVertex3f(CutPoint[CutFace[i][2]][0],CutPoint[CutFace[i][2]][1],CutPoint[CutFace[i][2]][2]);
			glVertex3f(CutPoint[CutFace[i][0]][0],CutPoint[CutFace[i][0]][1],CutPoint[CutFace[i][0]][2]);
		}
		glEnd();
	}
}

void CuttingTool::drawSurf(Vec3f color, int mode)
{
	glColor3f(color[0],color[1],color[2]);
	if(mode==0)
	{
		glBegin(GL_TRIANGLES);
		for(int i=0;i<Face.size();i++)
		{
			glNormal3f(Normal[i][0],Normal[i][1],Normal[i][2]);
			glVertex3f(Point[Face[i][0]][0],Point[Face[i][0]][1],Point[Face[i][0]][2]);
			glVertex3f(Point[Face[i][1]][0],Point[Face[i][1]][1],Point[Face[i][1]][2]);
			glVertex3f(Point[Face[i][2]][0],Point[Face[i][2]][1],Point[Face[i][2]][2]);

			glNormal3f(-Normal[i][0],-Normal[i][1],-Normal[i][2]);
			glVertex3f(Point[Face[i][0]][0],Point[Face[i][0]][1],Point[Face[i][0]][2]);
			glVertex3f(Point[Face[i][2]][0],Point[Face[i][2]][1],Point[Face[i][2]][2]);
			glVertex3f(Point[Face[i][1]][0],Point[Face[i][1]][1],Point[Face[i][1]][2]);
		}
		glEnd();
	}
	else
	{
		glBegin(GL_LINES);
		for(int i=0;i<Face.size();i++)
		{
			glVertex3f(Point[Face[i][0]][0],Point[Face[i][0]][1],Point[Face[i][0]][2]);
			glVertex3f(Point[Face[i][1]][0],Point[Face[i][1]][1],Point[Face[i][1]][2]);
			glVertex3f(Point[Face[i][1]][0],Point[Face[i][1]][1],Point[Face[i][1]][2]);
			glVertex3f(Point[Face[i][2]][0],Point[Face[i][2]][1],Point[Face[i][2]][2]);
			glVertex3f(Point[Face[i][2]][0],Point[Face[i][2]][1],Point[Face[i][2]][2]);
			glVertex3f(Point[Face[i][0]][0],Point[Face[i][0]][1],Point[Face[i][0]][2]);
		}
		glEnd();
	}
}

void CuttingTool::drawSurfTotal(Vec3f color, int mode)
{
	glColor3f(color[0],color[1],color[2]);
	if(mode==0)
	{
		glBegin(GL_TRIANGLES);
		for(int i=0;i<FaceTotal.size();i++)
		{
			glNormal3f(NormalTotal[i][0],NormalTotal[i][1],NormalTotal[i][2]);
			glVertex3f(PointTotal[FaceTotal[i][0]][0],PointTotal[FaceTotal[i][0]][1],PointTotal[FaceTotal[i][0]][2]);
			glVertex3f(PointTotal[FaceTotal[i][1]][0],PointTotal[FaceTotal[i][1]][1],PointTotal[FaceTotal[i][1]][2]);
			glVertex3f(PointTotal[FaceTotal[i][2]][0],PointTotal[FaceTotal[i][2]][1],PointTotal[FaceTotal[i][2]][2]);

			glNormal3f(-NormalTotal[i][0],-NormalTotal[i][1],-NormalTotal[i][2]);
			glVertex3f(PointTotal[FaceTotal[i][0]][0],PointTotal[FaceTotal[i][0]][1],PointTotal[FaceTotal[i][0]][2]);
			glVertex3f(PointTotal[FaceTotal[i][2]][0],PointTotal[FaceTotal[i][2]][1],PointTotal[FaceTotal[i][2]][2]);
			glVertex3f(PointTotal[FaceTotal[i][1]][0],PointTotal[FaceTotal[i][1]][1],PointTotal[FaceTotal[i][1]][2]);
		}
		glEnd();
	}
	else
	{
		glBegin(GL_LINES);
		for(int i=0;i<FaceTotal.size();i++)
		{
			glVertex3f(PointTotal[FaceTotal[i][0]][0],PointTotal[FaceTotal[i][0]][1],PointTotal[FaceTotal[i][0]][2]);
			glVertex3f(PointTotal[FaceTotal[i][1]][0],PointTotal[FaceTotal[i][1]][1],PointTotal[FaceTotal[i][1]][2]);
			glVertex3f(PointTotal[FaceTotal[i][1]][0],PointTotal[FaceTotal[i][1]][1],PointTotal[FaceTotal[i][1]][2]);
			glVertex3f(PointTotal[FaceTotal[i][2]][0],PointTotal[FaceTotal[i][2]][1],PointTotal[FaceTotal[i][2]][2]);
			glVertex3f(PointTotal[FaceTotal[i][2]][0],PointTotal[FaceTotal[i][2]][1],PointTotal[FaceTotal[i][2]][2]);
			glVertex3f(PointTotal[FaceTotal[i][0]][0],PointTotal[FaceTotal[i][0]][1],PointTotal[FaceTotal[i][0]][2]);
		}
		glEnd();
	}
}

void CuttingTool::drawCutFront(float radius)
{
	GLUquadricObj *qobj = 0;
	qobj = gluNewQuadric();

	for(int i=0;i<CutFront.size();i++)
	{
		glPushMatrix();
		glTranslatef(CutFront[i][0],CutFront[i][1],CutFront[i][2]);
		gluSphere(qobj, radius, 20, 20);
		glPopMatrix();
	}

	glBegin(GL_LINES);
	for(int i=0;i<CutFront.size()-1;i++)
	{
		glVertex3f(CutFront[i][0],CutFront[i][1],CutFront[i][2]);
		glVertex3f(CutFront[i+1][0],CutFront[i+1][1],CutFront[i+1][2]);
	}
	glEnd();
}

void CuttingTool::generateFace()
{
	// 1. generate face
	for(int i=0;i<CutFront.size()-1;i++)
	{
		Vec3f face=Vec3f(i, i+1, i+CutFront.size());
		CutFace.push_back(face);
		face=Vec3f(i+1, i+1+CutFront.size(), i+CutFront.size());
		CutFace.push_back(face);
	}
	CutFaceNormal.resize(CutFace.size());
}

void CuttingTool::computeCutFaceNormal()
{
	GeometricFunc func;
	for(int i=0;i<CutFace.size();i++)
	{
		CutFaceNormal[i]=func.computeNormal(CutPoint[CutFace[i][0]],CutPoint[CutFace[i][1]],CutPoint[CutFace[i][2]]);
	}
}

void CuttingTool::initEx1()
{
	std::vector<Vec3f> point;
	int nbPoint=20;
	point.resize(nbPoint);
	point[0]=Vec3f(150, 330, -200);

	for(int i=1;i<nbPoint;i++)
		point[i]=point[i-1]+Vec3f(0,0,25);
	setCutFront(&point);

	// generate tool path
	int nbToolPath=20;
	//int nbToolPath=7;
	ToolPath.resize(nbToolPath);
	for(int i=0;i<nbToolPath;i++)
	{
		ToolPath[i]=Vec3f(0,-15,0);
	}

	for(int i=0;i<point.size();i++)
		PointTotal.push_back(point[i]);

	for(int i=0;i<nbToolPath;i++)
		updateToolPosTotal();
	CutPoint.clear();
	CutFace.clear();
	CutFront.clear();
	CutEnd.clear();
	setCutFront(&point);

	//Container
	Container=new TopologyContainer;
	Container->init(&PointTotal, &PointTotal, &FaceTotal, &EdgeTotal);

	//BVH construction
	BVHAABB=new AABBTreeTri(&PointTotal, &FaceTotal, &EdgeTotal);
	BVHAABB->constructAABBTree();
}

void CuttingTool::initEx2()
{
	std::vector<Vec3f> point;
	int nbPoint=20;
	point.resize(nbPoint);
	point[0]=Vec3f(150, 330, -200);

	for(int i=1;i<nbPoint;i++)
		point[i]=point[i-1]+Vec3f(0,0,25);
	setCutFront(&point);

	// generate tool path
	int nbToolPath=25;
	//int nbToolPath=7;
	ToolPath.resize(nbToolPath);
	for(int i=0;i<nbToolPath;i++)
	{
		float x=-sin(i*0.2)*7.5;
		ToolPath[i]=Vec3f(x,-15,0);
	}

	for(int i=0;i<point.size();i++)
		PointTotal.push_back(point[i]);

	for(int i=0;i<nbToolPath;i++)
		updateToolPosTotal();
	CutPoint.clear();
	CutFace.clear();
	CutFront.clear();
	CutEnd.clear();
	setCutFront(&point);

	//Container
	Container=new TopologyContainer;
	Container->init(&PointTotal, &PointTotal, &FaceTotal, &EdgeTotal);

	//BVH construction
	BVHAABB=new AABBTreeTri(&PointTotal, &FaceTotal, &EdgeTotal);
	BVHAABB->constructAABBTree();
}

void CuttingTool::initEx3()
{
	std::vector<Vec3f> point;
	int nbPoint=20;
	point.resize(nbPoint);
	point[0]=Vec3f(190, 330, -200);

	float x=0;
	float y=330;
	float z=-200;

	for(int i=0;i<nbPoint;i++)
	{
		x=-z*z/500.0+250;
		point[i]=Vec3f(x,y,z);
		z+=20;
	}
	setCutFront(&point);

	// generate tool path
	//int nbToolPath=15;
	int nbToolPath=7;
	ToolPath.resize(nbToolPath);
	for(int i=0;i<nbToolPath;i++)
		ToolPath[i]=Vec3f(-0.1,-20,0);

	for(int i=0;i<point.size();i++)
		PointTotal.push_back(point[i]);

	for(int i=0;i<nbToolPath;i++)
		updateToolPosTotal();
	CutPoint.clear();
	CutFace.clear();
	CutFront.clear();
	CutEnd.clear();
	setCutFront(&point);
}

void CuttingTool::initEx4()
{
	std::vector<Vec3f> point;
	int nbPoint=20;
	point.resize(nbPoint);
	point[0]=Vec3f(190, 330, -200);

	float x=190;
	float y=330;
	float z=-200;

	for(int i=0;i<nbPoint;i++)
	{
		point[i]=Vec3f(x,y,z);
		y-=25;
	}
	setCutFront(&point);

	// generate tool path
	int nbToolPath=20;
	//int nbToolPath=7;
	ToolPath.resize(nbToolPath);
	for(int i=0;i<nbToolPath;i++)
		ToolPath[i]=Vec3f(0,0,25);

	for(int i=0;i<point.size();i++)
		PointTotal.push_back(point[i]);

	for(int i=0;i<nbToolPath;i++)
		updateToolPosTotal();
	CutPoint.clear();
	CutFace.clear();
	CutFront.clear();
	CutEnd.clear();
	setCutFront(&point);
}

void CuttingTool::initEx5()
{
	std::vector<Vec3f> point;
	int nbPoint=20;
	point.resize(nbPoint);
	point[0]=Vec3f(190, 330, -200);

	float x=190;
	float y=330;
	float z=-200;

	for(int i=0;i<nbPoint;i++)
	{
		point[i]=Vec3f(x,y,z);
		y-=25;
	}
	setCutFront(&point);

	// generate tool path
	int nbToolPath=20;
	//int nbToolPath=7;
	ToolPath.resize(nbToolPath);
	for(int i=0;i<nbToolPath;i++)
	{
		float x=13*sin(i*0.5);
		ToolPath[i]=Vec3f(x,0,20);
	}

	for(int i=0;i<point.size();i++)
		PointTotal.push_back(point[i]);

	for(int i=0;i<nbToolPath;i++)
		updateToolPosTotal();
	CutPoint.clear();
	CutFace.clear();
	CutFront.clear();
	CutEnd.clear();
	setCutFront(&point);
}

void CuttingTool::updateToolPos()
{
	static int count=0;
	if(count<ToolPath.size())
	{
		trans(ToolPath[count]);
		count++;
	}
}

void CuttingTool::updateToolPosTotal()
{
	static int count=0;
	if(count<ToolPath.size())
	{
		transTotal(ToolPath[count]);
		count++;
	}
}

std::vector<Vec3f>* CuttingTool::point()
{
	return &Point;
}
std::vector<Vec3i>* CuttingTool::face()
{
	return &Face;
}
std::vector<Vec3f>* CuttingTool::normal()
{
	return &Normal;
}

std::vector<Vec3f>* CuttingTool::cutPoint()
{
	return &CutPoint;
}
std::vector<Vec3i>* CuttingTool::cutFace()
{
	return &CutFace;
}
std::vector<Vec3f>* CuttingTool::cutFaceNormal()
{
	return &CutFaceNormal;
}

std::vector<Vec3f>* CuttingTool::pointTotal()
{
	return &PointTotal;
}
std::vector<Vec3i>* CuttingTool::faceTotal()
{
	return &FaceTotal;
}
std::vector<Vec3f>* CuttingTool::normalTotal()
{
	return &NormalTotal;
}

void CuttingTool::getCutFrontIdxLocal(std::vector<int>& idx)
{
	idx.clear();
	for(int i=0;i<CutFront.size();i++)
	{
		idx.push_back(i);
	}
}

void CuttingTool::getCutFrontIdx(std::vector<int>& idx)
{
	idx.clear();
	for(int i=0;i<CutFront.size();i++)
	{
		int a=Point.size()-CutFront.size()-1+i;
		idx.push_back(a);
	}
}