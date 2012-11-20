#include "stdafx.h"
#include "Meshfree_CPU.h"

Meshfree_CPU::Meshfree_CPU(void)
{
	SurfObj=NULL;
	EFGObj=NULL;
	F=fopen("Meshfree_CPU.txt","w");
	EFGTime=0;
	SurfTime=0;
	Count=0;
}

Meshfree_CPU::~Meshfree_CPU(void)
{
	fprintf(F,"EFGAve: %f\n",EFGTime/(float)Count);
	fprintf(F,"SurfAve: %f\n",SurfTime/(float)Count);

	if(SurfObj)
		delete SurfObj;
	if(EFGObj)
		delete EFGObj;
	fclose(F);
}

void Meshfree_CPU::loadSurfObj(char* filename)
{
	SurfObj=new SurfaceObj;
	SurfObj->readObjData(filename);
	SurfObj->constructAABBTree();
}

void Meshfree_CPU::generateEFGObj(int res)
{
	MeshfreeNodeGenerator nodeGenerator;
	nodeGenerator.setSurfObj(SurfObj);
	nodeGenerator.generateUniformNode(res);

	SupportRadius=nodeGenerator.boxSize().norm()*1.1;
	SupportRadiusSurf=SupportRadius*1.5;
	EFGObj=new EFG_CPU;
	EFGObj->init(nodeGenerator.nodePos(), nodeGenerator.nodeVolume(), SupportRadius);

	fprintf(F,"NbNode: %d\n", EFGObj->nbNode());
	fprintf(F,"NbNeighMin: %d\n", EFGObj->nbNeighborNodeMin());
}

void Meshfree_CPU::drawSurfObj(Vec3f color, int mode)
{
	if(mode==0)
		SurfObj->drawObject(color);
	else
		SurfObj->drawWireFrame(color);
}

void Meshfree_CPU::drawEFGObj(Vec3f color, float radius, int mode)
{
	EFGObj->draw(color, radius, mode);
}

void Meshfree_CPU::updatePositionExplicit(float dt, int itter)
{
	TimeTick.SetStart();
	EFGObj->updatePositionExplicit(dt, itter);
	TimeTick.SetEnd();
	fprintf(F,"EFGUpdate: %f\n", TimeTick.GetTick());
	EFGTime+=TimeTick.GetTick();

	TimeTick.SetStart();
	updateSurfPosition();
	TimeTick.SetEnd();
	fprintf(F,"SurfUpdate: %f\n", TimeTick.GetTick());
	SurfTime+=TimeTick.GetTick();
	Count++;
}

void Meshfree_CPU::boxConstraint(Vec3f leftDown, Vec3f rightUp)
{
	EFGObj->boxConstraint(leftDown, rightUp);
}

void Meshfree_CPU::connectSurfAndEFG()
{
	float support=SupportRadiusSurf;
	initSurfaceNeighborNode(support);
	initShapeFuncValueAtSurfPoint(support);

	testInitNeighbor(support);
	testInitShapeFuncValue(support);
}

void Meshfree_CPU::initSurfaceNeighborNode(float supportRadius)
{
	int nbSurfPoint=SurfObj->point()->size();
	int nbNode=EFGObj->nbNode();
	std::vector<Vec3f>* surfPoint=SurfObj->point();
	std::vector<Vec3f>* node=EFGObj->nodePos0();

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

void Meshfree_CPU::initShapeFuncValueAtSurfPoint(float support)
{
	MLSShapeFunc* shapeFunc=new MLSShapeFunc;
	shapeFunc->init(EFGObj->nodePos(), support, WEIGHT_FUNC_TYPE);
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

void Meshfree_CPU::updateSurfPosition()
{
	int nbSurfPoint=SurfObj->point()->size();
	std::vector<Vec3f>* surfDis=SurfObj->dis();
	std::vector<Vec3f>* nodeDis=EFGObj->nodeDis();

	for(int i=0;i<nbSurfPoint;i++)
	{
		(*surfDis)[i].clear();
		for(int j=0;j<NeighborNodeOfSurfVertice[i].size();j++)
		{
			(*surfDis)[i]+=(*nodeDis)[NeighborNodeOfSurfVertice[i][j]]*ShapeFuncValueAtSurfPoint[i][j];
		}
	}
	SurfObj->updatePoint();
}

int Meshfree_CPU::findNearestNode(Vec3f point)
{
	int idx=-1;
	std::vector<Vec3f>* nodePos=EFGObj->nodePos();
	int min=1000000;
	for(int i=0;i<nodePos->size();i++)
	{
		if((point-(*nodePos)[i]).norm()<min)
		{
			min=(point-(*nodePos)[i]).norm();
			idx=i;
		}
	}
	return idx;
}

void Meshfree_CPU::testInitNeighbor(float support)
{
	int nbTestPoint=1000;
	for(int i=0;i<nbTestPoint;i++)
	{
		TestPoint[i]=Vec3f(i-500,100,0);
		TestPoint0[i]=Vec3f(i-500,100,0);
	}
	int nbNode=EFGObj->nbNode();
	std::vector<Vec3f>* node=EFGObj->nodePos0();

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


void Meshfree_CPU::testInitShapeFuncValue(float support)
{
	int nbTestPoint=1000;
	MLSShapeFunc* shapeFunc=new MLSShapeFunc;
	shapeFunc->init(EFGObj->nodePos(), support, WEIGHT_FUNC_TYPE);

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
void Meshfree_CPU::writeTest(char* filename)
{
	FILE* f=fopen(filename,"w");
	for(int i=0;i<1000;i++)
	{
		fprintf(f,"%f %f %f\n",TestDis[i][0],TestDis[i][1],TestDis[i][2]);
	}
	fclose(f);
}

void Meshfree_CPU::updateShapeFunction(std::vector<int>& pointIdx)
{
	MLSShapeFunc func;
	std::vector<Vec3f>* point=SurfObj->point();
	std::vector<Vec3f>* node=EFGObj->nodePos0();
	int nbPoint=point->size();
	int nbNode=node->size();

	//1. Init MLS shape function
	func.init(node, SupportRadiusSurf, WEIGHT_FUNC_TYPE);

	std::vector<Mat4x4f> momentMatrixAtPoint;
	std::vector<Mat4x4f> momentMatrixAtPointInv;

	//2. Compute moment matrix at each node
	momentMatrixAtPoint.resize(nbPoint);
	momentMatrixAtPointInv.resize(nbPoint);

	for(int i=0;i<pointIdx.size();i++)
	{
		func.computeMomentMatrix(momentMatrixAtPoint[pointIdx[i]], NeighborNodeOfSurfVertice[pointIdx[i]], (*point)[pointIdx[i]]);
		invertMatrix(momentMatrixAtPointInv[pointIdx[i]], momentMatrixAtPoint[pointIdx[i]]);
	}

	//4. Compute shape function at node
	ShapeFuncValueAtSurfPoint.resize(nbPoint);
	for(int i=0;i<pointIdx.size();i++)
	{
		int nbNeighbor=NeighborNodeOfSurfVertice[pointIdx[i]].size();
		ShapeFuncValueAtSurfPoint[pointIdx[i]].resize(nbNeighbor);
		for(unsigned int j=0;j<nbNeighbor;j++)
		{
			ShapeFuncValueAtSurfPoint[pointIdx[i]][j]=func.computeShapeFuncValue(NeighborNodeOfSurfVertice[pointIdx[i]][j], (*point)[pointIdx[i]], momentMatrixAtPointInv[pointIdx[i]]);
		}
	}
}