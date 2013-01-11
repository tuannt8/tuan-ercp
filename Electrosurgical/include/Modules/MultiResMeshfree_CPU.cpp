#include "stdafx.h"
#include "MultiResMeshfree_CPU.h"

MultiResMeshfree_CPU::MultiResMeshfree_CPU(void)
{
	SurfObj=NULL;
	LowResEFGObj=NULL;
	HighResEFGObj=NULL;
}

MultiResMeshfree_CPU::~MultiResMeshfree_CPU(void)
{
	if(SurfObj)
		delete SurfObj;
	if(LowResEFGObj)
		delete LowResEFGObj;
	if(HighResEFGObj)
		delete HighResEFGObj;
}

void MultiResMeshfree_CPU::loadSurfObj(char* filename)
{
	SurfObj=new SurfaceObj;
	SurfObj->readObjData(filename);
}

void MultiResMeshfree_CPU::generateEFGObj(int lowRes, int highRes)
{
	// 1. Low resolution node
	MeshfreeNodeGenerator nodeGeneratorLow;
	nodeGeneratorLow.setSurfObj(SurfObj);
	nodeGeneratorLow.generateUniformNode(lowRes);

	SupportRadiusLow=nodeGeneratorLow.boxSize().norm()*1.1;
	LowResEFGObj=new EFG_CPU;
	float density=DENSITY;
	LowResEFGObj->init(nodeGeneratorLow.nodePos(), nodeGeneratorLow.nodeVolume(), SupportRadiusLow, density);

	// 2. High resolution node
	MeshfreeNodeGenerator nodeGeneratorHigh;
	nodeGeneratorHigh.setSurfObj(SurfObj);
	nodeGeneratorHigh.generateUniformNode(highRes);
	SupportRadiusHigh=nodeGeneratorHigh.boxSize().norm()*1.1;
	SupportRadiusSurf=SupportRadiusHigh*1.2;

	int nbNodeInside=nodeGeneratorHigh.findNodeNearSurface(30);
	HighResEFGObj=new EFG_CPU;
	HighResEFGObj->init(nodeGeneratorHigh.nodePos(), nodeGeneratorHigh.nodeVolume(), SupportRadiusHigh, nbNodeInside);

	// 3. Multi-res
	initMultiResNeighbor();
	initMultirResShapeFunc();
}

void MultiResMeshfree_CPU::drawSurfObj(Vec3f color, int mode)
{
	if(mode==0)
		SurfObj->drawObject(color);
	else
		SurfObj->drawWireFrame(color);
}

void MultiResMeshfree_CPU::drawLowResEFGObj(Vec3f color, float radius, int mode)
{
	LowResEFGObj->draw(color, radius, mode);
}

void MultiResMeshfree_CPU::drawHighResEFGObj(Vec3f color, float radius, int mode)
{
	HighResEFGObj->draw(color, radius, mode);
	HighResEFGObj->drawNodeInside(Vec3f(0,1,0), radius*1.2);
}

void MultiResMeshfree_CPU::updatePositionExplicit(float dt, int itter)
{
	// 1. update low resolution node
	LowResEFGObj->updatePositionExplicit(dt, itter);

	// 2. update passive node
	std::vector<Vec3f>* highResNodePos=HighResEFGObj->nodePos();
	std::vector<Vec3f>* highResNodePos0=HighResEFGObj->nodePos0();
	std::vector<Vec3f>* highResNodeDis=HighResEFGObj->nodeDis();

	std::vector<Vec3f>* lowResNodePos=LowResEFGObj->nodePos();
	std::vector<Vec3f>* lowResNodeDis=LowResEFGObj->nodeDis();
	int nbNodeInside=HighResEFGObj->nbNodeInside();

	for(int i=0;i<nbNodeInside;i++)
	{
		Vec3f dis;
		for(int j=0;j<NeighborNodeIdx[i].size();j++)
		{
			int idx=NeighborNodeIdx[i][j];
			dis+=(*lowResNodeDis)[idx]*ShapeFuncValue[i][j];
		}
		(*highResNodePos)[i]=(*highResNodePos0)[i]+dis;
		(*highResNodeDis)[i]=dis;
	}

	HighResEFGObj->updatePositionExplicit(dt, itter);
	updateSurfPosition();
}

void MultiResMeshfree_CPU::boxConstraint(Vec3f leftDown, Vec3f rightUp)
{
	LowResEFGObj->boxConstraint(leftDown, rightUp);
	HighResEFGObj->boxConstraint(leftDown, rightUp);
}

void MultiResMeshfree_CPU::initMultiResNeighbor()
{
	std::vector<Vec3f>* lowResNodePos=LowResEFGObj->nodePos();
	std::vector<Vec3f>* highResNodePos=HighResEFGObj->nodePos();

	NeighborNodeIdx.resize(highResNodePos->size());
	for(int i=0;i<highResNodePos->size();i++)
	{
		for(int j=0;j<lowResNodePos->size();j++)
		{
			if(((*highResNodePos)[i]-(*lowResNodePos)[j]).norm()<SupportRadiusLow)
				NeighborNodeIdx[i].push_back(j);
		}
	}
}
void MultiResMeshfree_CPU::initMultirResShapeFunc()
{
	MLSShapeFunc shapeFunc;
	std::vector<Mat4x4f> momentMatrix;
	std::vector<Mat4x4f> momentMatrixInv;

	//1. Init MLS shape function
	shapeFunc.init(LowResEFGObj->nodePos(), SupportRadiusLow, WEIGHT_FUNC_TYPE);

	//2. Compute moment matrix at each node
	momentMatrix.resize(HighResEFGObj->nbNode());
	for(int i=0;i<HighResEFGObj->nbNode();i++)
		shapeFunc.computeMomentMatrix(momentMatrix[i], NeighborNodeIdx[i], (*HighResEFGObj->nodePos())[i]);

	//3. Compute moment matrix inverse at each node
	momentMatrixInv.resize(HighResEFGObj->nbNode());
	for(int i=0;i<HighResEFGObj->nbNode();i++)
		invertMatrix(momentMatrixInv[i], momentMatrix[i]);

	//4. Compute shape function at node
	ShapeFuncValue.resize(HighResEFGObj->nbNode());
	for(int i=0;i<HighResEFGObj->nbNode();i++)
	{
		int nbNeighbor=NeighborNodeIdx[i].size();
		ShapeFuncValue[i].resize(nbNeighbor);
		for(unsigned int j=0;j<nbNeighbor;j++)
		{
			int idx=NeighborNodeIdx[i][j];
			ShapeFuncValue[i][j]=shapeFunc.computeShapeFuncValue(idx, (*HighResEFGObj->nodePos())[i], momentMatrixInv[i]);
		}
	}
}

void MultiResMeshfree_CPU::connectSurfAndEFG()
{
	float support=SupportRadiusSurf;
	initSurfaceNeighborNode(support);
	initShapeFuncValueAtSurfPoint(support);

	testInitNeighbor(support);
	testInitShapeFuncValue(support);
}

void MultiResMeshfree_CPU::initSurfaceNeighborNode(float supportRadius)
{
	int nbSurfPoint=SurfObj->point()->size();
	int nbNode=HighResEFGObj->nbNode();
	std::vector<Vec3f>* surfPoint=SurfObj->point();
	std::vector<Vec3f>* node=HighResEFGObj->nodePos0();

	//memory allocation
	NeighborNodeOfSurfVertice.resize(nbSurfPoint);

	//search and save neighbor information
	for(int i=0;i<nbSurfPoint;i++)
	{
		for(int j=0;j<nbNode;j++)
		{
			if(((*surfPoint)[i]-(*node)[j]).norm()<supportRadius)
				NeighborNodeOfSurfVertice[i].push_back(j);
		}
	}
}

void MultiResMeshfree_CPU::initShapeFuncValueAtSurfPoint(float support)
{
	MLSShapeFunc* shapeFunc=new MLSShapeFunc;
	shapeFunc->init(HighResEFGObj->nodePos(), support, WEIGHT_FUNC_TYPE);
	std::vector<Vec3f>* point=SurfObj->point();

	//1. Memory allocation
	ShapeFuncValueAtSurfPoint.resize(SurfObj->point()->size());
	for(int i=0;i<point->size();i++)
		ShapeFuncValueAtSurfPoint[i].resize(NeighborNodeOfSurfVertice[i].size());

	//2. Shape function value computation

	//2-1. Compute moment matrix at surface point
	Mat4x4f* moment=new Mat4x4f[point->size()];
	Mat4x4f* momentInv=new Mat4x4f[point->size()];

	for(int i=0;i<point->size();i++)
		shapeFunc->computeMomentMatrix(moment[i], NeighborNodeOfSurfVertice[i], (*point)[i]);

	for(int i=0;i<point->size();i++)
		invertMatrix(momentInv[i], moment[i]);

	//2-2 Compute shape function value
	for(int i=0;i<point->size();i++)
	{
		Vec3d pos=(*point)[i];
		for(int j=0;j<NeighborNodeOfSurfVertice[i].size();j++)
			ShapeFuncValueAtSurfPoint[i][j]=shapeFunc->computeShapeFuncValue(NeighborNodeOfSurfVertice[i][j], pos, momentInv[i]);
	}

	delete [] moment;
	delete [] momentInv;
	delete shapeFunc;
}

void MultiResMeshfree_CPU::updateSurfPosition()
{
	int nbSurfPoint=SurfObj->point()->size();
	std::vector<Vec3f>* surfDis=SurfObj->dis();
	std::vector<Vec3f>* nodeDis=HighResEFGObj->nodeDis();

	for(int i=0;i<nbSurfPoint;i++)
	{
		(*surfDis)[i].clear();
		for(int j=0;j<NeighborNodeOfSurfVertice[i].size();j++)
		{
			(*surfDis)[i]+=(*nodeDis)[NeighborNodeOfSurfVertice[i][j]]*ShapeFuncValueAtSurfPoint[i][j];
		}
	}
	SurfObj->updatePoint();

	// Test point update
	for(int i=0;i<1000;i++)
	{
		TestDis[i].clear();
		for(int j=0;j<TestNeighbor[i].size();j++)
		{
			TestDis[i]+=(*nodeDis)[TestNeighbor[i][j]]*TestShapeFuncValue[i][j];
		}
		TestPoint[i]=TestPoint0[i]+TestDis[i];
	}
}

void MultiResMeshfree_CPU::testInitNeighbor(float support)
{
	int nbTestPoint=1000;
	for(int i=0;i<nbTestPoint;i++)
	{
		TestPoint[i]=Vec3f(i-500,100,0);
		TestPoint0[i]=Vec3f(i-500,100,0);
	}
	int nbNode=HighResEFGObj->nbNode();
	std::vector<Vec3f>* node=HighResEFGObj->nodePos0();

	//search and save neighbor information
	for(int i=0;i<nbTestPoint;i++)
	{
		for(int j=0;j<nbNode;j++)
		{
			if((TestPoint[i]-(*node)[j]).norm()<support)
				TestNeighbor[i].push_back(j);
		}
	}
}


void MultiResMeshfree_CPU::testInitShapeFuncValue(float support)
{
	int nbTestPoint=1000;
	MLSShapeFunc* shapeFunc=new MLSShapeFunc;
	shapeFunc->init(HighResEFGObj->nodePos(), support, WEIGHT_FUNC_TYPE);

	//1. Memory allocation
	for(int i=0;i<nbTestPoint;i++)
		TestShapeFuncValue[i].resize(TestNeighbor[i].size());

	//2. Shape function value computation

	//2-1. Compute moment matrix at surface point
	Mat4x4f* moment=new Mat4x4f[nbTestPoint];
	Mat4x4f* momentInv=new Mat4x4f[nbTestPoint];

	for(int i=0;i<nbTestPoint;i++)
		shapeFunc->computeMomentMatrix(moment[i], TestNeighbor[i], TestPoint[i]);

	for(int i=0;i<nbTestPoint;i++)
		invertMatrix(momentInv[i], moment[i]);

	//2-2 Compute shape function value
	for(int i=0;i<nbTestPoint;i++)
	{
		Vec3d pos=TestPoint[i];
		for(int j=0;j<TestNeighbor[i].size();j++)
			TestShapeFuncValue[i][j]=shapeFunc->computeShapeFuncValue(TestNeighbor[i][j], pos, momentInv[i]);
	}

	delete [] moment;
	delete [] momentInv;
	delete shapeFunc;
}
void MultiResMeshfree_CPU::writeTest(char* filename)
{
	FILE* f=fopen(filename,"w");
	for(int i=0;i<1000;i++)
	{
		fprintf(f,"%f %f %f\n",TestDis[i][0],TestDis[i][1],TestDis[i][2]);
	}
	fclose(f);
}