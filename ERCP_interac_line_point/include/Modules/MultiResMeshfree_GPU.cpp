#include "stdafx.h"
#include "MultiResMeshfree_GPU.h"

MultiResMeshfree_GPU::MultiResMeshfree_GPU(void)
{
	SurfObj=NULL;
	LowResEFGObj=NULL;
	HighResEFGObj=NULL;
}

MultiResMeshfree_GPU::~MultiResMeshfree_GPU(void)
{
	if(SurfObj)
		delete SurfObj;
	if(LowResEFGObj)
		delete LowResEFGObj;
	if(HighResEFGObj)
		delete HighResEFGObj;
}

void MultiResMeshfree_GPU::loadSurfObj(char* filename)
{
	SurfObj=new SurfaceObj;
	SurfObj->readObjData(filename);
}

void MultiResMeshfree_GPU::generateEFGObj(int lowRes, int highRes)
{
	// 1. Low resolution node
	MeshfreeNodeGenerator nodeGeneratorLow;
	nodeGeneratorLow.setSurfObj(SurfObj);
	nodeGeneratorLow.generateUniformNode(lowRes);

	SupportRadiusLow=nodeGeneratorLow.boxSize().norm()*1.1;
	LowResEFGObj=new EFG_CUDA_RUNTIME;
	LowResEFGObj->init(nodeGeneratorLow.nodePos(), nodeGeneratorLow.nodeVolume(), SupportRadiusLow);

	// 2. High resolution node
	MeshfreeNodeGenerator nodeGeneratorHigh;
	nodeGeneratorHigh.setSurfObj(SurfObj);
	nodeGeneratorHigh.generateUniformNode(highRes);
	SupportRadiusHigh=nodeGeneratorHigh.boxSize().norm()*1.1;
	SupportRadiusSurf=SupportRadiusHigh*1.2;

	int nbNodeInside=nodeGeneratorHigh.findNodeNearSurface(30);
	HighResEFGObj=new EFG_CUDA_RUNTIME;
	HighResEFGObj->init(nodeGeneratorHigh.nodePos(), nodeGeneratorHigh.nodeVolume(), SupportRadiusHigh, nbNodeInside);

	// 3. Multi-res
	//initMultiResNeighbor();
	//initMultirResShapeFunc();
}

void MultiResMeshfree_GPU::drawSurfObj(Vec3f color, int mode)
{
	if(mode==0)
		SurfObj->drawObject(color);
	else
		SurfObj->drawWireFrame(color);
}

void MultiResMeshfree_GPU::drawLowResEFGObj(Vec3f color, float radius, int mode)
{
	LowResEFGObj->draw(color, radius, mode);
}

void MultiResMeshfree_GPU::drawHighResEFGObj(Vec3f color, float radius, int mode)
{
	HighResEFGObj->draw(color, radius, mode);
	HighResEFGObj->drawNodeInside(Vec3f(0,1,0), radius*1.2);
}

void MultiResMeshfree_GPU::updatePositionExplicit(float dt, int itter)
{
	// 1. update low resolution node
	LowResEFGObj->updatePositionExplicit_CUDA(dt, itter);

	// 2. update passive node
	/*std::vector<Vec3f>* highResNodePos=HighResEFGObj->nodePos();
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
	}*/

	//HighResEFGObj->updatePositionExplicit_CUDA(dt, itter);
	//updateSurfPosition();
}

void MultiResMeshfree_GPU::boxConstraint(Vec3f leftDown, Vec3f rightUp)
{
	if(LowResEFGObj)
		LowResEFGObj->boxConstraint(leftDown, rightUp);
//	if(HighResEFGObj)
//		HighResEFGObj->boxConstraint(leftDown, rightUp);
}

void MultiResMeshfree_GPU::initFixedConstraintGPU()
{
	if(LowResEFGObj)
		LowResEFGObj->initFixedConstraintGPU();
//	if(HighResEFGObj)
//		HighResEFGObj->initFixedConstraintGPU();
}

/*void MultiResMeshfree_GPU::initMultiResNeighbor()
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
void MultiResMeshfree_GPU::initMultirResShapeFunc()
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

void MultiResMeshfree_GPU::connectSurfAndEFG()
{
	float support=SupportRadiusSurf;
	initSurfaceNeighborNode(support);
	initShapeFuncValueAtSurfPoint(support);
}

void MultiResMeshfree_GPU::initSurfaceNeighborNode(float supportRadius)
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

void MultiResMeshfree_GPU::initShapeFuncValueAtSurfPoint(float support)
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

void MultiResMeshfree_GPU::updateSurfPosition()
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
}*/