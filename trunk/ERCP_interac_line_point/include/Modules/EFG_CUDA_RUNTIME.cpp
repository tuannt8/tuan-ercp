#include "stdafx.h"
#include "EFG_CUDA_RUNTIME.h"
#include "CollisionManager.h"
#include "Utility.h"

extern "C" void d_initGPU(int nbNode, int nbNodeAdded, float* nodeVolume, float* nodePos0, float* nodePos, float* nodeVel, int nbNodeInside);
extern "C" void d_initStressPoint(int nbStress, float* stressVolume, float* stressPos0, float* stressPos);
extern "C" void d_initMaterialStiffness(float* materialStiffness, float* nodeMass);
extern "C" void d_initNeighborInformation(int* neighborNodeIdx, int* nbNeighborNode, int nbNeighborMax);
extern "C" void d_initNeighborInformationStress(int* neighborStressPointIdx, int* nbNeighborStressPoint, int* stressNeighborNodeIdx, int* nbStressNeighborNode);
extern "C" void d_initShapeFuncValue(float* shapeFuncDerivAtNode, float* shapeFuncDerivAtNodeInv);
extern "C" void d_initShapeFuncValueStressPoint(float* shapeFuncDerivAtStressPoint, float* shapeFuncDerivAtStressPointInv);
extern "C" void d_initFixedConstraint(int nbFixedConstraint, int* fixedIdx);
extern "C" void d_explicitIntegration(float dt, int nbFixedConstraint);
extern "C" void d_explicitIntegrationConst(float dt);
extern "C" void d_computeStrain();
extern "C" void d_computeStress();
extern "C" void d_computeForce(float damping);

extern "C" void d_setForce(float* nodeForce);
extern "C" void d_getForce(float* nodeForce);

extern "C" void d_getMass(float* nodeMass);
extern "C" void d_getStressPointStrain(float* stressPointStrain);
extern "C" void d_getStressPointStress(float* stressPointStress);
extern "C" void d_getNodeStrain(float* nodeStrain);
extern "C" void d_getNodeStress(float* nodeStress);
extern "C" void d_getPosition(float* pos);
extern "C" void d_setPosition(float* pos);
extern "C" void d_getDisplacement(float* nodeDis);
extern "C" void d_setDisplacement(float* nodeDis);

extern "C" void d_updateNbNeighborNodeFull(int* nbNeighborNode, int size);
extern "C" void d_updateNbNeighborNodePartial(int* updatedValue, int* updatedIdx, int nbUpdated);
extern "C" void d_updateNeighborNodeIdxFull(int* neighborNodeIdx, int size);
extern "C" void d_updateNeighborNodeIdxPartial(int* updatedValue, int* updatedIdx, int nbUpdated);

extern "C" void d_updateShapeFuncDrvFull(float* value, int size);
extern "C" void d_updateShapeFuncDrvPartial(float* updatedValue, int* updatedIdx, int nbUpdated);
extern "C" void d_updateShapeFuncDrvInvFull(float* value, int size);
extern "C" void d_updateShapeFuncDrvInvPartial(float* updatedValue, int* updatedIdx, int nbUpdated);

EFG_CUDA_RUNTIME::EFG_CUDA_RUNTIME(void)
{
	F=fopen("EFG_CUDA.txt","w");
	NbNodeInside=0;

	X=Vec3f(2,0,0);
	Y=Vec3f(0,2,0);
	Z=Vec3f(0,0,2);

	NodePos0Vec=NULL;
	NodePosVec=NULL;
	NodePosXVec=NULL;
	NodePosYVec=NULL;
	NodePosZVec=NULL;
	NodePos0=NULL;
	NodePos=NULL;
	NodeVel=NULL;
	NodeForce=NULL;
	NodeDis=NULL;
	NodePreDis=NULL;
	NodeVolume=NULL;
	NodeMass=NULL;
	NodeStress=NULL;
	NodeStrain=NULL;
	Edge=NULL;

	MaterialStiffness=NULL;
	NeighborNodeIdx=NULL;
	NbNeighborNode=NULL;

	MomentMatrixAtNode=NULL;
	MomentMatrixInvAtNode=NULL;

	MomentMatrixDrvXAtNode=NULL;
	MomentMatrixDrvYAtNode=NULL;
	MomentMatrixDrvZAtNode=NULL;

	ShapeFuncAtNode=NULL;
	ShapeFuncDerivXAtNode=NULL;
	ShapeFuncDerivYAtNode=NULL;
	ShapeFuncDerivZAtNode=NULL;

	ShapeFuncDerivXAtNodeInv=NULL;
	ShapeFuncDerivYAtNodeInv=NULL;
	ShapeFuncDerivZAtNodeInv=NULL;

	ShapeFuncDerivAtNode=NULL;
	ShapeFuncDerivAtNodeInv=NULL;

	StressPos=NULL;
	StressPos0=NULL;
	StressPointDis=NULL;
	StressVolume=NULL;
	StressPointStress=NULL;
	StressPointStrain=NULL;
	NbStressPoint=-1;

	NeighborStressPointIdx=NULL;
	StressNeighborNodeIdx=NULL;
	NbNeighborStressPoint=NULL;
	NbStressNeighborNode=NULL;

	MomentMatrixAtStressPoint=NULL;
	MomentMatrixInvAtStressPoint=NULL;
	MomentMatrixDrvXAtStressPoint=NULL;
	MomentMatrixDrvYAtStressPoint=NULL;
	MomentMatrixDrvZAtStressPoint=NULL;

	ShapeFuncAtStressPoint=NULL;
	ShapeFuncDerivAtStressPoint=NULL;
	ShapeFuncDerivXAtStressPoint=NULL;
	ShapeFuncDerivYAtStressPoint=NULL;
	ShapeFuncDerivZAtStressPoint=NULL;
	ShapeFuncDerivAtStressPointInv=NULL;
	ShapeFuncDerivXAtStressPointInv=NULL;
	ShapeFuncDerivYAtStressPointInv=NULL;
	ShapeFuncDerivZAtStressPointInv=NULL;

	NbNodeInBox=2;

	NbUpdated=0;
	Count=0;
}

EFG_CUDA_RUNTIME::~EFG_CUDA_RUNTIME(void)
{
	fprintf(F,"NbUpdatedAve: %f\n",(float)NbUpdated/(float)Count);
	fclose(F);
	if(NodePos0Vec)
		delete NodePos0Vec;
	if(NodePosVec)
		delete NodePosVec;
	if(NodePosXVec)
		delete NodePosXVec;
	if(NodePosYVec)
		delete NodePosYVec;
	if(NodePosZVec)
		delete NodePosZVec;
	if(Edge)
		delete Edge;

	if(NodePos0)
		delete [] NodePos0;
	if(NodePos)
		delete [] NodePos;
	if(NodeVel)
		delete [] NodeVel;
	if(NodeForce)
		delete [] NodeForce;
	if(NodeDis)
		delete [] NodeDis;
	if(NodePreDis)
		delete [] NodePreDis;
	if(NodeVolume)
		delete [] NodeVolume;
	if(NodeMass)
		delete [] NodeMass;
	if(NodeStress)
		delete [] NodeStress;
	if(NodeStrain)
		delete [] NodeStrain;
	if(MaterialStiffness)
		delete [] MaterialStiffness;
	if(NeighborNodeIdx)
		delete [] NeighborNodeIdx;
	if(NbNeighborNode)
		delete [] NbNeighborNode;

	if(MomentMatrixAtNode)
		delete [] MomentMatrixAtNode;
	if(MomentMatrixInvAtNode)
		delete [] MomentMatrixInvAtNode;
	if(MomentMatrixDrvXAtNode)
		delete [] MomentMatrixDrvXAtNode;
	if(MomentMatrixDrvYAtNode)
		delete [] MomentMatrixDrvYAtNode;
	if(MomentMatrixDrvZAtNode)
		delete [] MomentMatrixDrvZAtNode;

	if(ShapeFuncAtNode)
		delete [] ShapeFuncAtNode;
	if(ShapeFuncDerivXAtNode)
		delete [] ShapeFuncDerivXAtNode;
	if(ShapeFuncDerivYAtNode)
		delete [] ShapeFuncDerivYAtNode;
	if(ShapeFuncDerivZAtNode)
		delete [] ShapeFuncDerivZAtNode;
	if(ShapeFuncDerivXAtNodeInv)
		delete [] ShapeFuncDerivXAtNodeInv;
	if(ShapeFuncDerivYAtNodeInv)
		delete [] ShapeFuncDerivYAtNodeInv;
	if(ShapeFuncDerivZAtNodeInv)
		delete [] ShapeFuncDerivZAtNodeInv;

	if(ShapeFuncDerivAtNode)
		delete [] ShapeFuncDerivAtNode;
	if(ShapeFuncDerivAtNodeInv)
		delete [] ShapeFuncDerivAtNodeInv;

	if(StressPos)
		delete [] StressPos;
	if(StressPos0)
		delete [] StressPos0;
	if(StressPointDis)
		delete [] StressPointDis;
	if(StressVolume)
		delete [] StressVolume;
	if(StressPointStress)
		delete [] StressPointStress;
	if(StressPointStrain)
		delete [] StressPointStrain;

	if(NeighborStressPointIdx)
		delete [] NeighborStressPointIdx;
	if(StressNeighborNodeIdx)
		delete [] StressNeighborNodeIdx;
	if(NbNeighborStressPoint)
		delete [] NbNeighborStressPoint;
	if(NbStressNeighborNode)
		delete [] NbStressNeighborNode;

	if(MomentMatrixAtStressPoint)
		delete [] MomentMatrixAtStressPoint;
	if(MomentMatrixInvAtStressPoint)
		delete [] MomentMatrixInvAtStressPoint;
	if(MomentMatrixDrvXAtStressPoint)
		delete [] MomentMatrixDrvXAtStressPoint;
	if(MomentMatrixDrvYAtStressPoint)
		delete [] MomentMatrixDrvYAtStressPoint;
	if(MomentMatrixDrvZAtStressPoint)
		delete [] MomentMatrixDrvZAtStressPoint;

	if(ShapeFuncAtStressPoint)
		delete [] ShapeFuncAtStressPoint;
	if(ShapeFuncDerivAtStressPoint)
		delete [] ShapeFuncDerivAtStressPoint;
	if(ShapeFuncDerivXAtStressPoint)
		delete [] ShapeFuncDerivXAtStressPoint;
	if(ShapeFuncDerivYAtStressPoint)
		delete [] ShapeFuncDerivYAtStressPoint;
	if(ShapeFuncDerivZAtStressPoint)
		delete [] ShapeFuncDerivZAtStressPoint;
	if(ShapeFuncDerivAtStressPointInv)
		delete [] ShapeFuncDerivAtStressPointInv;
	if(ShapeFuncDerivXAtStressPointInv)
		delete [] ShapeFuncDerivXAtStressPointInv;
	if(ShapeFuncDerivYAtStressPointInv)
		delete [] ShapeFuncDerivYAtStressPointInv;
	if(ShapeFuncDerivZAtStressPointInv)
		delete [] ShapeFuncDerivZAtStressPointInv;
}

void EFG_CUDA_RUNTIME::init(std::vector<Vec3f>* nodePos, float nodeVolume, float supportRadius, int nbNodeInside, SurfaceObj* surf)
{
	NbNode=nodePos->size();
	SupportRadius=supportRadius;
	NbNodeAddedMax=NbNode;
	NbNodeAdded=0;
	NbNodeInside=nbNodeInside;
	int size=NbNode+NbNodeAddedMax;

	//Memory allocation
	NodePos0Vec=new std::vector<Vec3f>;
	NodePosVec=new std::vector<Vec3f>;
	NodePosXVec=new std::vector<Vec3f>;
	NodePosYVec=new std::vector<Vec3f>;
	NodePosZVec=new std::vector<Vec3f>;
	NodePos0=new float[size*DIM];
	NodePos=new float[size*DIM];
	NodeVel=new float[size*DIM];
	NodeForce=new float[size*DIM];
	NodeDis=new float[size*DIM];
	NodePreDis=new float[size*DIM];
	NodeVolume=new float[size];
	NodeMass=new float[size];
	NodeStress=new float[size*SDIM];
	NodeStrain=new float[size*SDIM];

	//Initialization
	for(int i=0;i<size;i++)
		NodeVolume[i]=0;
	for(int i=0;i<size*DIM;i++)
	{
		NodePos0[i]=0;
		NodePos[i]=0;
		NodeVel[i]=0;
		NodeForce[i]=0;
		NodeDis[i]=0;
	}
	for(int i=0;i<size*SDIM;i++)
	{
		NodeStress[i]=0;
		NodeStrain[i]=0;
	}

	//Data copy
	for(int i=0;i<NbNode;i++)
	{
		NodePos[i*DIM]=NodePos0[i*DIM]=(*nodePos)[i][0];
		NodePos[i*DIM+1]=NodePos0[i*DIM+1]=(*nodePos)[i][1];
		NodePos[i*DIM+2]=NodePos0[i*DIM+2]=(*nodePos)[i][2];

		NodePos0Vec->push_back((*nodePos)[i]);
		NodePosVec->push_back((*nodePos)[i]);
		NodeVolume[i]=nodeVolume;
	}
	NodePosXVec->resize(NbNode);
	NodePosYVec->resize(NbNode);
	NodePosZVec->resize(NbNode);

	for(int i=NbNode;i<size;i++)
	{
		NodeVolume[i]=NodeVolume[NbNode-1];
	}

	// 1. construct material stiffness matrix
	constructMaterialStiffness();

	// 2. neighbor information 
	initNeighborInformation(surf);

	// 3. compute node mass and volume
	computeNodeMass();

	// 4. init shape function value
	initShapeFuncValue();

	// 5. GPU
	d_initGPU(NbNode, NbNodeAddedMax, NodeVolume, NodePos0, NodePos, NodeVel, NbNodeInside);
/*	d_initMaterialStiffness(MaterialStiffness, NodeMass);
	d_initNeighborInformation(NeighborNodeIdx, NbNeighborNode, NB_NEIGHBOR_MAX);

	size=(NbNode+NbNodeAddedMax)*NB_NEIGHBOR_MAX;
	float* shapeFuncDeriveAtNode=new float[size*3];
	float* shapeFuncDeriveAtNodeInv=new float[size*3];
	for(int i=0;i<size;i++)
	{
		shapeFuncDeriveAtNode[i]=ShapeFuncDerivXAtNode[i];
		shapeFuncDeriveAtNode[i+size]=ShapeFuncDerivYAtNode[i];
		shapeFuncDeriveAtNode[i+size*2]=ShapeFuncDerivZAtNode[i];

		shapeFuncDeriveAtNodeInv[i]=ShapeFuncDerivXAtNodeInv[i];
		shapeFuncDeriveAtNodeInv[i+size]=ShapeFuncDerivYAtNodeInv[i];
		shapeFuncDeriveAtNodeInv[i+size*2]=ShapeFuncDerivZAtNodeInv[i];
	}
	d_initShapeFuncValue(shapeFuncDeriveAtNode, shapeFuncDeriveAtNodeInv);

	delete [] shapeFuncDeriveAtNode;
	delete [] shapeFuncDeriveAtNodeInv;*/
}

void EFG_CUDA_RUNTIME::init(std::vector<Vec3f>* nodePos, float nodeVolume, float supportRadius, SurfaceObj* surf)
{
	NbNode=nodePos->size();
	SupportRadius=supportRadius;
	NbNodeAddedMax=NbNode;
	NbNodeAdded=0;
	int size=NbNode+NbNodeAddedMax;

	//Memory allocation
	NodePos0Vec=new std::vector<Vec3f>;
	NodePosVec=new std::vector<Vec3f>;
	NodePosXVec=new std::vector<Vec3f>;
	NodePosYVec=new std::vector<Vec3f>;
	NodePosZVec=new std::vector<Vec3f>;
	NodePos0=new float[size*DIM];
	NodePos=new float[size*DIM];
	NodeVel=new float[size*DIM];
	NodeForce=new float[size*DIM];
	NodePreDis=new float[size*DIM];
	NodeDis=new float[size*DIM];
	NodeVolume=new float[size];
	NodeMass=new float[size];
	NodeStress=new float[size*SDIM];
	NodeStrain=new float[size*SDIM];

	//Initialization
	for(int i=0;i<size;i++)
		NodeVolume[i]=0;
	for(int i=0;i<size*DIM;i++)
	{
		NodePos0[i]=0;
		NodePos[i]=0;
		NodeVel[i]=0;
		NodeForce[i]=0;
		NodeDis[i]=0;
	}
	for(int i=0;i<size*SDIM;i++)
	{
		NodeStress[i]=0;
		NodeStrain[i]=0;
	}

	//Data copy
	for(int i=0;i<NbNode;i++)
	{
		NodePos[i*DIM]=NodePos0[i*DIM]=(*nodePos)[i][0];
		NodePos[i*DIM+1]=NodePos0[i*DIM+1]=(*nodePos)[i][1];
		NodePos[i*DIM+2]=NodePos0[i*DIM+2]=(*nodePos)[i][2];
		
		NodePos0Vec->push_back((*nodePos)[i]);
		NodePosVec->push_back((*nodePos)[i]);
		NodeVolume[i]=nodeVolume;
	}
	NodePosXVec->resize(NbNode);
	NodePosYVec->resize(NbNode);
	NodePosZVec->resize(NbNode);

	for(int i=NbNode;i<size;i++)
	{
		NodeVolume[i]=NodeVolume[NbNode-1];
	}

	// 1. construct material stiffness matrix
	constructMaterialStiffness();

	// 2. neighbor information 
	initNeighborInformation(surf);

	// 3. compute node mass and volume
	computeNodeMass();

	// 4. init shape function value
	initShapeFuncValue();

	// 5. init bvh
	BVHAABB.init(NodePosVec, Edge);
	BVHAABB.constructAABBTree();

	BVHAABBPoint.init(NodePosVec, NbNodeInBox);
	BVHAABBPoint.constructAABBTree();

	// 6. GPU
	d_initGPU(NbNode, NbNodeAddedMax, NodeVolume, NodePos0, NodePos, NodeVel, NbNodeInside);
	d_initMaterialStiffness(MaterialStiffness, NodeMass);
	d_initNeighborInformation(NeighborNodeIdx, NbNeighborNode, NB_NEIGHBOR_MAX);

	size=(NbNode+NbNodeAddedMax)*NB_NEIGHBOR_MAX;
	ShapeFuncDerivAtNode=new float[size*3];
	ShapeFuncDerivAtNodeInv=new float[size*3];
	for(int i=0;i<size;i++)
	{
		ShapeFuncDerivAtNode[i]=ShapeFuncDerivXAtNode[i];
		ShapeFuncDerivAtNode[i+size]=ShapeFuncDerivYAtNode[i];
		ShapeFuncDerivAtNode[i+size*2]=ShapeFuncDerivZAtNode[i];

		ShapeFuncDerivAtNodeInv[i]=ShapeFuncDerivXAtNodeInv[i];
		ShapeFuncDerivAtNodeInv[i+size]=ShapeFuncDerivYAtNodeInv[i];
		ShapeFuncDerivAtNodeInv[i+size*2]=ShapeFuncDerivZAtNodeInv[i];
	}
	d_initShapeFuncValue(ShapeFuncDerivAtNode, ShapeFuncDerivAtNodeInv);
}

void EFG_CUDA_RUNTIME::init(std::vector<Vec3f>* nodePos, std::vector<Vec3f>* stressPos, float nodeVolume, float supportRadius, SurfaceObj* surf)
{
	NbNode=nodePos->size();
	NbStressPoint=stressPos->size();
	SupportRadius=supportRadius;
	NbNodeAddedMax=NbNode;
	NbNodeAdded=0;
	int size=NbNode+NbNodeAddedMax;
	int sizeStress=NbStressPoint+NbNodeAddedMax;

	//Memory allocation
	NodePos0Vec=new std::vector<Vec3f>;
	NodePosVec=new std::vector<Vec3f>;
	NodePosXVec=new std::vector<Vec3f>;
	NodePosYVec=new std::vector<Vec3f>;
	NodePosZVec=new std::vector<Vec3f>;
	NodePos0=new float[size*DIM];
	NodePos=new float[size*DIM];
	NodeVel=new float[size*DIM];
	NodeForce=new float[size*DIM];
	NodeDis=new float[size*DIM];
	NodePreDis=new float[size*DIM];
	NodeVolume=new float[size];
	NodeMass=new float[size];
	NodeStress=new float[size*SDIM];
	NodeStrain=new float[size*SDIM];

	StressPos=new float[sizeStress*DIM];
	StressPos0=new float[sizeStress*DIM];
	StressPointDis=new float[sizeStress*DIM];
	StressVolume=new float[sizeStress];
	StressPointStress=new float[sizeStress*SDIM];
	StressPointStrain=new float[sizeStress*SDIM];

	//Initialization
	for(int i=0;i<size;i++)
		NodeVolume[i]=0;
	for(int i=0;i<size*DIM;i++)
	{
		NodePos0[i]=0;
		NodePos[i]=0;
		NodeVel[i]=0;
		NodeForce[i]=0;
		NodeDis[i]=0;
	}
	for(int i=0;i<size*SDIM;i++)
	{
		NodeStress[i]=0;
		NodeStrain[i]=0;
	}

	for(int i=0;i<sizeStress;i++)
		StressVolume[i]=0;
	for(int i=0;i<sizeStress*DIM;i++)
	{
		StressPos0[i]=0;
		StressPos[i]=0;
		StressPointDis[i]=0;
	}
	for(int i=0;i<sizeStress*SDIM;i++)
	{
		StressPointStress[i]=0;
		StressPointStrain[i]=0;
	}

	//Data copy
	for(int i=0;i<NbNode;i++)
	{
		NodePos[i*DIM]=NodePos0[i*DIM]=(*nodePos)[i][0];
		NodePos[i*DIM+1]=NodePos0[i*DIM+1]=(*nodePos)[i][1];
		NodePos[i*DIM+2]=NodePos0[i*DIM+2]=(*nodePos)[i][2];

		NodePos0Vec->push_back((*nodePos)[i]);
		NodePosVec->push_back((*nodePos)[i]);
		NodeVolume[i]=nodeVolume;
	}
	NodePosXVec->resize(NbNode);
	NodePosYVec->resize(NbNode);
	NodePosZVec->resize(NbNode);

	for(int i=0;i<NbStressPoint;i++)
	{
		StressPos[i*DIM]=StressPos0[i*DIM]=(*stressPos)[i][0];
		StressPos[i*DIM+1]=StressPos0[i*DIM]=(*stressPos)[i][1];
		StressPos[i*DIM+2]=StressPos0[i*DIM]=(*stressPos)[i][2];
		StressVolume[i]=nodeVolume;
	}

	// 1. construct material stiffness matrix
	constructMaterialStiffness();

	// 2. neighbor information 
	initNeighborInformation(surf);

	// 3. compute node mass and volume
	computeNodeMass();

	// 4. init shape function value
	initShapeFuncValue();

	// 5. init bvh
	BVHAABB.init(NodePosVec, Edge);
	BVHAABB.constructAABBTree();

	BVHAABBPoint.init(NodePosVec, NbNodeInBox);
	BVHAABBPoint.constructAABBTree();

	// 6. GPU
	d_initGPU(NbNode, NbNodeAddedMax, NodeVolume, NodePos0, NodePos, NodeVel, NbNodeInside);
	d_initMaterialStiffness(MaterialStiffness, NodeMass);
	d_initNeighborInformation(NeighborNodeIdx, NbNeighborNode, NB_NEIGHBOR_MAX);

	d_initStressPoint(NbStressPoint, StressVolume, StressPos0, StressPos);
	d_initNeighborInformationStress(NeighborStressPointIdx, NbNeighborStressPoint, StressNeighborNodeIdx, NbStressNeighborNode);

	size=(NbNode+NbNodeAddedMax)*NB_NEIGHBOR_MAX;
	ShapeFuncDerivAtNode=new float[size*3];
	ShapeFuncDerivAtNodeInv=new float[size*3];
	for(int i=0;i<size;i++)
	{
		ShapeFuncDerivAtNode[i]=ShapeFuncDerivXAtNode[i];
		ShapeFuncDerivAtNode[i+size]=ShapeFuncDerivYAtNode[i];
		ShapeFuncDerivAtNode[i+size*2]=ShapeFuncDerivZAtNode[i];

		ShapeFuncDerivAtNodeInv[i]=ShapeFuncDerivXAtNodeInv[i];
		ShapeFuncDerivAtNodeInv[i+size]=ShapeFuncDerivYAtNodeInv[i];
		ShapeFuncDerivAtNodeInv[i+size*2]=ShapeFuncDerivZAtNodeInv[i];
	}
	d_initShapeFuncValue(ShapeFuncDerivAtNode, ShapeFuncDerivAtNodeInv);

	size=(NbNode+NbNodeAddedMax)*NB_NEIGHBOR_MAX;
	ShapeFuncDerivAtStressPoint=new float[size*3];
	for(int i=0;i<size;i++)
	{
		ShapeFuncDerivAtStressPoint[i]=ShapeFuncDerivXAtStressPoint[i];
		ShapeFuncDerivAtStressPoint[i+size]=ShapeFuncDerivYAtStressPoint[i];
		ShapeFuncDerivAtStressPoint[i+size*2]=ShapeFuncDerivZAtStressPoint[i];
	}

	size=(NbStressPoint+NbNodeAddedMax)*NB_NEIGHBOR_MAX;
	ShapeFuncDerivAtStressPointInv=new float[size*3];
	for(int i=0;i<size;i++)
	{
		ShapeFuncDerivAtStressPointInv[i]=ShapeFuncDerivXAtStressPointInv[i];
		ShapeFuncDerivAtStressPointInv[i+size]=ShapeFuncDerivYAtStressPointInv[i];
		ShapeFuncDerivAtStressPointInv[i+size*2]=ShapeFuncDerivZAtStressPointInv[i];
	}
	d_initShapeFuncValueStressPoint(ShapeFuncDerivAtStressPoint, ShapeFuncDerivAtStressPointInv);
}

void EFG_CUDA_RUNTIME::init(char* filename)
{
	FILE* f=fopen(filename,"r");
	fscanf(f,"%d %f", &NbNode, &SupportRadius);

	NbNodeAddedMax=NbNode;
	NbNodeAdded=0;
	int size=NbNode+NbNodeAddedMax;

	//Memory allocation
	NodePos0Vec=new std::vector<Vec3f>;
	NodePosVec=new std::vector<Vec3f>;
	NodePosXVec=new std::vector<Vec3f>;
	NodePosYVec=new std::vector<Vec3f>;
	NodePosZVec=new std::vector<Vec3f>;
	NodePos0=new float[size*DIM];
	NodePos=new float[size*DIM];
	NodeVel=new float[size*DIM];
	NodeForce=new float[size*DIM];
	NodeDis=new float[size*DIM];
	NodePreDis=new float[size*DIM];
	NodeVolume=new float[size];
	NodeMass=new float[size];
	NodeStress=new float[size*SDIM];
	NodeStrain=new float[size*SDIM];

	//Initialization
	for(int i=0;i<size;i++)
		NodeVolume[i]=0;
	for(int i=0;i<size*DIM;i++)
	{
		NodePos0[i]=0;
		NodePos[i]=0;
		NodeVel[i]=0;
		NodeForce[i]=0;
		NodeDis[i]=0;
	}
	for(int i=0;i<size*SDIM;i++)
	{
		NodeStress[i]=0;
		NodeStrain[i]=0;
	}

	//Data copy
	for(int i=0;i<NbNode;i++)
	{
		Vec3f nodePosVec;
		fscanf(f,"%f %f %f %f", &NodePos0[i*DIM], &NodePos0[i*DIM+1], &NodePos0[i*DIM+2], &NodeVolume[i]);
		NodePos[i*DIM]=nodePosVec[0]=NodePos0[i*DIM];
		NodePos[i*DIM+1]=nodePosVec[1]=NodePos0[i*DIM+1];
		NodePos[i*DIM+2]=nodePosVec[2]=NodePos0[i*DIM+2];
		NodePos0Vec->push_back(nodePosVec);
		NodePosVec->push_back(nodePosVec);
	}
	NodePosXVec->resize(NbNode);
	NodePosYVec->resize(NbNode);
	NodePosZVec->resize(NbNode);

	for(int i=NbNode;i<size;i++)
	{
		NodeVolume[i]=NodeVolume[NbNode-1];
	}

	// 1. construct material stiffness matrix
	constructMaterialStiffness();

	// 2. neighbor information 
	initNeighborInformation(NULL);

	// 3. compute node mass and volume
	computeNodeMass();

	// 4. init shape function value
	initShapeFuncValue();

	// 5. init bvh
	BVHAABB.init(NodePosVec, Edge);
	BVHAABB.constructAABBTree();

	BVHAABBPoint.init(NodePosVec, NbNodeInBox);
	BVHAABBPoint.constructAABBTree();

	// 6. GPU
	d_initGPU(NbNode, NbNodeAddedMax, NodeVolume, NodePos0, NodePos, NodeVel, NbNodeInside);
	d_initMaterialStiffness(MaterialStiffness, NodeMass);
	d_initNeighborInformation(NeighborNodeIdx, NbNeighborNode, NB_NEIGHBOR_MAX);

	size=(NbNode+NbNodeAddedMax)*NB_NEIGHBOR_MAX;
	ShapeFuncDerivAtNode=new float[size*3];
	ShapeFuncDerivAtNodeInv=new float[size*3];
	for(int i=0;i<size;i++)
	{
		ShapeFuncDerivAtNode[i]=ShapeFuncDerivXAtNode[i];
		ShapeFuncDerivAtNode[i+size]=ShapeFuncDerivYAtNode[i];
		ShapeFuncDerivAtNode[i+size*2]=ShapeFuncDerivZAtNode[i];

		ShapeFuncDerivAtNodeInv[i]=ShapeFuncDerivXAtNodeInv[i];
		ShapeFuncDerivAtNodeInv[i+size]=ShapeFuncDerivYAtNodeInv[i];
		ShapeFuncDerivAtNodeInv[i+size*2]=ShapeFuncDerivZAtNodeInv[i];
	}
	d_initShapeFuncValue(ShapeFuncDerivAtNode, ShapeFuncDerivAtNodeInv);
}

void EFG_CUDA_RUNTIME::constructMaterialStiffness()
{
	MaterialStiffness=new float[SDIM*SDIM];

	//construct material stiffness matrix
	MaterialStiffness[0*SDIM+0] = MaterialStiffness[1*SDIM+1] = MaterialStiffness[2*SDIM+2] = 1;
	MaterialStiffness[0*SDIM+1] = MaterialStiffness[0*SDIM+2] = MaterialStiffness[1*SDIM+0] = MaterialStiffness[1*SDIM+2] = MaterialStiffness[2*SDIM+0] = MaterialStiffness[2*SDIM+1] = NU/(1-NU);
	MaterialStiffness[0*SDIM+3] = MaterialStiffness[0*SDIM+4] = MaterialStiffness[0*SDIM+5] = 0;
	MaterialStiffness[1*SDIM+3] = MaterialStiffness[1*SDIM+4] = MaterialStiffness[1*SDIM+5] = 0;
	MaterialStiffness[2*SDIM+3] = MaterialStiffness[2*SDIM+4] = MaterialStiffness[2*SDIM+5] = 0;
	MaterialStiffness[3*SDIM+0] = MaterialStiffness[3*SDIM+1] = MaterialStiffness[3*SDIM+2] = MaterialStiffness[3*SDIM+4] = MaterialStiffness[3*SDIM+5] = 0;
	MaterialStiffness[4*SDIM+0] = MaterialStiffness[4*SDIM+1] = MaterialStiffness[4*SDIM+2] = MaterialStiffness[4*SDIM+3] = MaterialStiffness[4*SDIM+5] = 0;
	MaterialStiffness[5*SDIM+0] = MaterialStiffness[5*SDIM+1] = MaterialStiffness[5*SDIM+2] = MaterialStiffness[5*SDIM+3] = MaterialStiffness[5*SDIM+4] = 0;
	MaterialStiffness[3*SDIM+3] = MaterialStiffness[4*SDIM+4] = MaterialStiffness[5*SDIM+5] = (1-2*NU)/(2*(1-NU));

	for(int i=0;i<SDIM*SDIM;i++)
		MaterialStiffness[i] *= (E*(1-NU))/((1+NU)*(1-2*NU));
}

void EFG_CUDA_RUNTIME::initNeighborInformation(SurfaceObj* surf)
{
	int size=NbNode+NbNodeAddedMax;
	std::vector<int>* neighborNodeIdx=new std::vector<int>[NbNode];
	Edge=new std::vector<Vec2i>;
	EdgesAroundNode.resize(NbNode);

	CollisionManager collision;
	for(int i=0;i<NbNode;i++)
	{
		for(int j=0;j<NbNode;j++)
		{
			if(norm(&NodePos[i*DIM], &NodePos[j*DIM], DIM)<SupportRadius)
			{
				Vec3f pt1,pt2;
				pt1[0]=NodePos[i*DIM];pt1[1]=NodePos[i*DIM+1];pt1[2]=NodePos[i*DIM+2];
				pt2[0]=NodePos[j*DIM];pt2[1]=NodePos[j*DIM+1];pt2[2]=NodePos[j*DIM+2];
				if(!collision.collisionBtwSurfAndLineSeg(surf, pt1, pt2) || !surf)
				{
					Vec2i edge(i,j);
					Edge->push_back(edge);
					EdgesAroundNode[i].push_back(Edge->size()-1);
					neighborNodeIdx[i].push_back(j);
				}
			}
		}
	}

	NbNeighborNode=new int[size];
	for(int i=0;i<size;i++)
	{
		if(i<NbNode)
			NbNeighborNode[i]=neighborNodeIdx[i].size();
		else
			NbNeighborNode[i]=0;
	}

	// Initialization
	NeighborNodeIdx=new int[size*NB_NEIGHBOR_MAX];
	for(int i=0;i<size*NB_NEIGHBOR_MAX;i++)
		NeighborNodeIdx[i]=-1;

	for(int i=0;i<NbNode;i++)
	{
		for(int j=0;j<NbNeighborNode[i];j++)
		{
			NeighborNodeIdx[NB_NEIGHBOR_MAX*i+j]=neighborNodeIdx[i][j];
		}
	}

	// Stress point neighbor node information
	if(NbStressPoint>0)
	{
		int sizeStress=NbStressPoint+NbNodeAddedMax;
		std::vector<std::vector<int>> neighborStressPointIdx;
		std::vector<std::vector<int>> stressNeighborNodeIdx;
		neighborStressPointIdx.resize(NbNode);
		stressNeighborNodeIdx.resize(NbStressPoint);
		
		for(int i=0;i<NbNode;i++)
		{
			for(int j=0;j<NbStressPoint;j++)
			{
				Vec3f stressPos(StressPos[j*DIM],StressPos[j*DIM+1],StressPos[j*DIM+2]);
				if(((*NodePosVec)[i]-stressPos).norm()<SupportRadius)
				{
					neighborStressPointIdx[i].push_back(j);
				}
			}
		}
		for(int i=0;i<NbStressPoint;i++)
		{
			Vec3f stressPos(StressPos[i*DIM],StressPos[i*DIM+1],StressPos[i*DIM+2]);
			for(int j=0;j<NbNode;j++)
			{
				if(((*NodePosVec)[j]-stressPos).norm()<SupportRadius)
				{
					stressNeighborNodeIdx[i].push_back(j);
				}
			}
		}

		NbNeighborStressPoint=new int[size];
		for(int i=0;i<size;i++)
		{
			if(i<NbNode)
				NbNeighborStressPoint[i]=neighborStressPointIdx[i].size();
			else
				NbNeighborStressPoint[i]=0;
		}
		NbStressNeighborNode=new int[sizeStress];
		for(int i=0;i<sizeStress;i++)
		{
			if(i<NbStressPoint)
				NbStressNeighborNode[i]=stressNeighborNodeIdx[i].size();
			else
				NbStressNeighborNode[i]=0;
		}

		NeighborStressPointIdx=new int[size*NB_NEIGHBOR_MAX];
		for(int i=0;i<size*NB_NEIGHBOR_MAX;i++)
			NeighborStressPointIdx[i]=-1;

		for(int i=0;i<NbNode;i++)
		{
			for(int j=0;j<NbNeighborStressPoint[i];j++)
			{
				NeighborStressPointIdx[NB_NEIGHBOR_MAX*i+j]=neighborStressPointIdx[i][j];
			}
		}

		StressNeighborNodeIdx=new int[sizeStress*NB_NEIGHBOR_MAX];
		for(int i=0;i<sizeStress*NB_NEIGHBOR_MAX;i++)
			StressNeighborNodeIdx[i]=-1;

		for(int i=0;i<NbStressPoint;i++)
		{
			for(int j=0;j<NbStressNeighborNode[i];j++)
			{
				StressNeighborNodeIdx[NB_NEIGHBOR_MAX*i+j]=stressNeighborNodeIdx[i][j];
			}
		}
	}
	delete [] neighborNodeIdx;

	//Print neighbor infor
	/*for(int i=0;i<NbNode;i++)
	{
		for(int j=0;j<NbNeighborStressPoint[i];j++)
		{
			int idx=NB_NEIGHBOR_MAX*i+j;
			fprintf(F,"%d ",NeighborStressPointIdx[idx]);
		}
		fprintf(F,"\n");
	}*/
	/*for(int i=0;i<NbStressPoint;i++)
	{
		for(int j=0;j<NbStressNeighborNode[i];j++)
		{
			int idx=NB_NEIGHBOR_MAX*i+j;
			fprintf(F,"%d ",StressNeighborNodeIdx[idx]);
		}
		fprintf(F,"\n");
	}*/
}

float EFG_CUDA_RUNTIME::norm(float* a, float* b, int dim)
{
	float res=0;
	for(int i=0;i<dim;i++)
		res+=(a[i]-b[i])*(a[i]-b[i]);
	return sqrt(res);
}

void EFG_CUDA_RUNTIME::computeNodeMass()
{
	int size=NbNode+NbNodeAddedMax;
	double mass;
	mass=DENSITY*NodeVolume[0];
	//compute mass matrix
	for(int i=0;i<size;i++){
		NodeMass[i]=DENSITY*NodeVolume[i];
	}
}

void EFG_CUDA_RUNTIME::initShapeFuncValue()
{
	int size=NbNode+NbNodeAddedMax;

	//1. Init MLS shape function
	ShapeFunc.init(NodePos0Vec, SupportRadius, WEIGHT_FUNC_TYPE);

	//2. Compute moment matrix at each node
	MomentMatrixAtNode=new Mat4x4f[size];
	for(int i=0;i<NbNode;i++)
		ShapeFunc.computeMomentMatrixAtNode(MomentMatrixAtNode[i], &NeighborNodeIdx[NB_NEIGHBOR_MAX*i], NbNeighborNode[i], i);

	//3. Compute moment matrix inverse at each node
	MomentMatrixInvAtNode=new Mat4x4f[size];
	for(int i=0;i<NbNode;i++)
		invertMatrix(MomentMatrixInvAtNode[i], MomentMatrixAtNode[i]);

	//4. Compute shape function at node
	ShapeFuncAtNode=new float[NB_NEIGHBOR_MAX*size];
	for(int i=0;i<NbNode;i++)
	{
		for(unsigned int j=0;j<NbNeighborNode[i];j++)
		{
			int idx=NB_NEIGHBOR_MAX*i+j;
			ShapeFuncAtNode[idx]=ShapeFunc.computeShapeFuncValue(i, (*NodePos0Vec)[NeighborNodeIdx[idx]], MomentMatrixInvAtNode[NeighborNodeIdx[idx]]);
		}
	}

	//5. Compute shape function drvX at node
	MomentMatrixDrvXAtNode=new Mat4x4f[size];
	ShapeFunc.computeMomentMatrixDrvXAtNode(MomentMatrixDrvXAtNode, NeighborNodeIdx, NbNeighborNode, NB_NEIGHBOR_MAX);
	ShapeFuncDerivXAtNode=new float[NB_NEIGHBOR_MAX*size];
	ShapeFuncDerivXAtNodeInv=new float[NB_NEIGHBOR_MAX*size];
	for(int i=0;i<NbNode;i++)
	{
		for(unsigned int j=0;j<NbNeighborNode[i];j++)
		{
			int idx=NB_NEIGHBOR_MAX*i+j;
			ShapeFuncDerivXAtNode[idx]=ShapeFunc.computeShapeFuncDrvX(i, (*NodePos0Vec)[NeighborNodeIdx[idx]], MomentMatrixInvAtNode[NeighborNodeIdx[idx]], MomentMatrixDrvXAtNode[NeighborNodeIdx[idx]]);
		}
	}

	for(int i=0;i<NbNode;i++)
	{
		for(unsigned int j=0;j<NbNeighborNode[i];j++)
		{
			int idx=NB_NEIGHBOR_MAX*i+j;
			ShapeFuncDerivXAtNodeInv[idx]=ShapeFunc.computeShapeFuncDrvX(NeighborNodeIdx[idx], (*NodePos0Vec)[i], MomentMatrixInvAtNode[i], MomentMatrixDrvXAtNode[i]);
		}
	}

	//6. Compute shape function drvY at node
	MomentMatrixDrvYAtNode=new Mat4x4f[size];
	ShapeFunc.computeMomentMatrixDrvYAtNode(MomentMatrixDrvYAtNode, NeighborNodeIdx, NbNeighborNode, NB_NEIGHBOR_MAX);
	ShapeFuncDerivYAtNode=new float[NB_NEIGHBOR_MAX*size];
	ShapeFuncDerivYAtNodeInv=new float[NB_NEIGHBOR_MAX*size];
	for(int i=0;i<NbNode;i++)
	{
		for(unsigned int j=0;j<NbNeighborNode[i];j++)
		{
			int idx=NB_NEIGHBOR_MAX*i+j;
			ShapeFuncDerivYAtNode[idx]=ShapeFunc.computeShapeFuncDrvY(i, (*NodePos0Vec)[NeighborNodeIdx[idx]], MomentMatrixInvAtNode[NeighborNodeIdx[idx]], MomentMatrixDrvYAtNode[NeighborNodeIdx[idx]]);
		}
	}
	for(int i=0;i<NbNode;i++)
	{
		for(unsigned int j=0;j<NbNeighborNode[i];j++)
		{
			int idx=NB_NEIGHBOR_MAX*i+j;
			ShapeFuncDerivYAtNodeInv[idx]=ShapeFunc.computeShapeFuncDrvY(NeighborNodeIdx[idx], (*NodePos0Vec)[i], MomentMatrixInvAtNode[i], MomentMatrixDrvYAtNode[i]);
		}
	}

	//7. Compute shape function drvZ at node
	MomentMatrixDrvZAtNode=new Mat4x4f[size];
	ShapeFunc.computeMomentMatrixDrvZAtNode(MomentMatrixDrvZAtNode, NeighborNodeIdx, NbNeighborNode, NB_NEIGHBOR_MAX);
	ShapeFuncDerivZAtNode=new float[NB_NEIGHBOR_MAX*size];
	ShapeFuncDerivZAtNodeInv=new float[NB_NEIGHBOR_MAX*size];
	for(int i=0;i<NbNode;i++)
	{
		for(unsigned int j=0;j<NbNeighborNode[i];j++)
		{
			int idx=NB_NEIGHBOR_MAX*i+j;
			ShapeFuncDerivZAtNode[idx]=ShapeFunc.computeShapeFuncDrvZ(i, (*NodePos0Vec)[NeighborNodeIdx[idx]], MomentMatrixInvAtNode[NeighborNodeIdx[idx]], MomentMatrixDrvZAtNode[NeighborNodeIdx[idx]]);
		}
	}
	for(int i=0;i<NbNode;i++)
	{
		for(unsigned int j=0;j<NbNeighborNode[i];j++)
		{
			int idx=NB_NEIGHBOR_MAX*i+j;
			ShapeFuncDerivZAtNodeInv[idx]=ShapeFunc.computeShapeFuncDrvZ(NeighborNodeIdx[idx], (*NodePos0Vec)[i], MomentMatrixInvAtNode[i], MomentMatrixDrvZAtNode[i]);
		}
	}

	if(NbStressPoint>0)
	{
		int sizeStress=NbStressPoint+NbNodeAddedMax;

		// 1. Compute moment matrix at stress point
		MomentMatrixAtStressPoint=new Mat4x4f[sizeStress];
		for(int i=0;i<NbStressPoint;i++)
		{
			Vec3f pos(StressPos[i*DIM],StressPos[i*DIM+1],StressPos[i*DIM+2]);
			ShapeFunc.computeMomentMatrix(MomentMatrixAtStressPoint[i], &StressNeighborNodeIdx[NB_NEIGHBOR_MAX*i], NbStressNeighborNode[i], pos);
		}

		//2. Compute moment matrix inverse at stress point
		MomentMatrixInvAtStressPoint=new Mat4x4f[sizeStress];
		for(int i=0;i<NbStressPoint;i++)
			invertMatrix(MomentMatrixInvAtStressPoint[i], MomentMatrixAtStressPoint[i]);

		//3. Compute shape function at node
		ShapeFuncAtStressPoint=new float[NB_NEIGHBOR_MAX*sizeStress];

		for(int i=0;i<NbStressPoint;i++)
		{
			Vec3f stressPos=Vec3f(StressPos[i*DIM],StressPos[i*DIM+1],StressPos[i*DIM+2]);
			for(unsigned int j=0;j<NbStressNeighborNode[i];j++)
			{
				int idx=NB_NEIGHBOR_MAX*i+j;
				ShapeFuncAtStressPoint[idx]=ShapeFunc.computeShapeFuncValue(StressNeighborNodeIdx[idx], stressPos, MomentMatrixInvAtStressPoint[i]);
			}
		}

		//4. Compute shape function drvX at stress point
		MomentMatrixDrvXAtStressPoint=new Mat4x4f[sizeStress];
		ShapeFuncDerivXAtStressPoint=new float[NB_NEIGHBOR_MAX*size];
		ShapeFuncDerivXAtStressPointInv=new float[NB_NEIGHBOR_MAX*sizeStress];

		for(int i=0;i<NbStressPoint;i++)
		{
			Vec3f stressPoint(StressPos[i*DIM],StressPos[i*DIM+1],StressPos[i*DIM+2]);
			ShapeFunc.computeMomentMatrixDrvX(stressPoint, MomentMatrixDrvXAtStressPoint[i], &StressNeighborNodeIdx[i*NB_NEIGHBOR_MAX], NbStressNeighborNode[i]);
		}

		for(int i=0;i<NbNode;i++)
		{
			for(unsigned int j=0;j<NbNeighborStressPoint[i];j++)
			{
				int idx=NB_NEIGHBOR_MAX*i+j;
				Vec3f stressPos(StressPos[NeighborStressPointIdx[idx]*DIM],StressPos[NeighborStressPointIdx[idx]*DIM+1],StressPos[NeighborStressPointIdx[idx]*DIM+2]);
				ShapeFuncDerivXAtStressPoint[idx]=ShapeFunc.computeShapeFuncDrvX(i, stressPos, MomentMatrixInvAtStressPoint[NeighborStressPointIdx[idx]], MomentMatrixDrvXAtStressPoint[NeighborStressPointIdx[idx]]);
			}
		}
		for(int i=0;i<NbStressPoint;i++)
		{
			Vec3f stressPos(StressPos[i*DIM],StressPos[i*DIM+1],StressPos[i*DIM+2]);
			for(unsigned int j=0;j<NbStressNeighborNode[i];j++)
			{
				int idx=NB_NEIGHBOR_MAX*i+j;
				int neiIdx=StressNeighborNodeIdx[idx];
				ShapeFuncDerivXAtStressPointInv[idx]=ShapeFunc.computeShapeFuncDrvX(neiIdx, stressPos, MomentMatrixInvAtStressPoint[i], MomentMatrixDrvXAtStressPoint[i]);
			}
		}

		//5. Compute shape function drvY at stress point
		MomentMatrixDrvYAtStressPoint=new Mat4x4f[sizeStress];
		ShapeFuncDerivYAtStressPoint=new float[NB_NEIGHBOR_MAX*size];
		ShapeFuncDerivYAtStressPointInv=new float[NB_NEIGHBOR_MAX*sizeStress];

		for(int i=0;i<NbStressPoint;i++)
		{
			Vec3f stressPoint(StressPos[i*DIM],StressPos[i*DIM+1],StressPos[i*DIM+2]);
			ShapeFunc.computeMomentMatrixDrvY(stressPoint, MomentMatrixDrvYAtStressPoint[i], &StressNeighborNodeIdx[i*NB_NEIGHBOR_MAX], NbStressNeighborNode[i]);
		}

		for(int i=0;i<NbNode;i++)
		{
			for(unsigned int j=0;j<NbNeighborStressPoint[i];j++)
			{
				int idx=NB_NEIGHBOR_MAX*i+j;
				Vec3f stressPos(StressPos[NeighborStressPointIdx[idx]*DIM],StressPos[NeighborStressPointIdx[idx]*DIM+1],StressPos[NeighborStressPointIdx[idx]*DIM+2]);
				ShapeFuncDerivYAtStressPoint[idx]=ShapeFunc.computeShapeFuncDrvY(i, stressPos, MomentMatrixInvAtStressPoint[NeighborStressPointIdx[idx]], MomentMatrixDrvYAtStressPoint[NeighborStressPointIdx[idx]]);
			}
		}
		for(int i=0;i<NbStressPoint;i++)
		{
			Vec3f stressPos(StressPos[i*DIM],StressPos[i*DIM+1],StressPos[i*DIM+2]);
			for(unsigned int j=0;j<NbStressNeighborNode[i];j++)
			{
				int idx=NB_NEIGHBOR_MAX*i+j;
				int neiIdx=StressNeighborNodeIdx[idx];
				ShapeFuncDerivYAtStressPointInv[idx]=ShapeFunc.computeShapeFuncDrvY(neiIdx, stressPos, MomentMatrixInvAtStressPoint[i], MomentMatrixDrvYAtStressPoint[i]);
			}
		}

		//6. Compute shape function drvZ at stress point
		MomentMatrixDrvZAtStressPoint=new Mat4x4f[sizeStress];
		ShapeFuncDerivZAtStressPoint=new float[NB_NEIGHBOR_MAX*size];
		ShapeFuncDerivZAtStressPointInv=new float[NB_NEIGHBOR_MAX*sizeStress];

		for(int i=0;i<NbStressPoint;i++)
		{
			Vec3f stressPoint(StressPos[i*DIM],StressPos[i*DIM+1],StressPos[i*DIM+2]);
			ShapeFunc.computeMomentMatrixDrvZ(stressPoint, MomentMatrixDrvZAtStressPoint[i], &StressNeighborNodeIdx[i*NB_NEIGHBOR_MAX], NbStressNeighborNode[i]);
		}

		for(int i=0;i<NbNode;i++)
		{
			for(unsigned int j=0;j<NbNeighborStressPoint[i];j++)
			{
				int idx=NB_NEIGHBOR_MAX*i+j;
				Vec3f stressPos(StressPos[NeighborStressPointIdx[idx]*DIM],StressPos[NeighborStressPointIdx[idx]*DIM+1],StressPos[NeighborStressPointIdx[idx]*DIM+2]);
				ShapeFuncDerivZAtStressPoint[idx]=ShapeFunc.computeShapeFuncDrvZ(i, stressPos, MomentMatrixInvAtStressPoint[NeighborStressPointIdx[idx]], MomentMatrixDrvZAtStressPoint[NeighborStressPointIdx[idx]]);
			}
		}
		for(int i=0;i<NbStressPoint;i++)
		{
			Vec3f stressPos(StressPos[i*DIM],StressPos[i*DIM+1],StressPos[i*DIM+2]);
			for(unsigned int j=0;j<NbStressNeighborNode[i];j++)
			{
				int idx=NB_NEIGHBOR_MAX*i+j;
				int neiIdx=StressNeighborNodeIdx[idx];
				ShapeFuncDerivZAtStressPointInv[idx]=ShapeFunc.computeShapeFuncDrvZ(neiIdx, stressPos, MomentMatrixInvAtStressPoint[i], MomentMatrixDrvZAtStressPoint[i]);
			}
		}
	}
}

void EFG_CUDA_RUNTIME::boxConstraint(Vec3f LeftDown, Vec3f RightUp)
{
	for(int i=0;i<NbNode;i++)
	{
		if((NodePos0[i*DIM+0]>LeftDown[0])&&(NodePos0[i*DIM+0]<RightUp[0]))
		{
			if((NodePos0[i*DIM+1]>LeftDown[1])&&(NodePos0[i*DIM+1]<RightUp[1]))
			{
				if((NodePos0[i*DIM+2]>LeftDown[2])&&(NodePos0[i*DIM+2]<RightUp[2]))
					FixedNodeIdx.push_back(i);
			}
		}
	}
}

void EFG_CUDA_RUNTIME::initFixedConstraintGPU()
{
	int nbFixedConstraint=FixedNodeIdx.size();
	int* temp=new int[nbFixedConstraint];
	for(int i=0;i<nbFixedConstraint;i++)
		temp[i]=FixedNodeIdx[i];
	d_initFixedConstraint(nbFixedConstraint, temp);
	delete [] temp;
}

void EFG_CUDA_RUNTIME::draw(Vec3f color, int radius, int mode)
{
	if(mode==0)
	{
		GLUquadricObj *qobj = 0;
		qobj = gluNewQuadric();

		for(int i=0;i<FixedNodeIdx.size();i++)
		{
			glColor3f(0,1,0);
			glPushMatrix();
			glTranslatef((GLfloat)(*NodePosVec)[FixedNodeIdx[i]][0],(GLfloat)(*NodePosVec)[FixedNodeIdx[i]][1],(GLfloat)(*NodePosVec)[FixedNodeIdx[i]][2]);
			gluSphere(qobj,radius*1.2,20,20);
			glPopMatrix();
		}

		for(int i=0;i<NodePosVec->size();i++)
		{
			glColor3f(color[0],color[1],color[2]);
			glPushMatrix();
			glTranslatef((GLfloat)(*NodePosVec)[i][0],(GLfloat)(*NodePosVec)[i][1],(GLfloat)(*NodePosVec)[i][2]);
			gluSphere(qobj,radius,20,20);
			glPopMatrix();
		}

	}
	if(mode==1)
	{
		glPointSize(radius);
		glBegin(GL_POINTS);
		for(int i=0;i<NbNode;i++)
		{
			glColor3f(color[0],color[1],color[2]);
			glVertex3f((GLfloat)(*NodePosVec)[i][0],(GLfloat)(*NodePosVec)[i][1],(GLfloat)(*NodePosVec)[i][2]);
			//glVertex3f((GLfloat)NodePos[i*DIM+0],(GLfloat)NodePos[i*DIM+1],(GLfloat)NodePos[i*DIM+2]);
		}
		glEnd();
	}
}

void EFG_CUDA_RUNTIME::drawStressPoint(Vec3f color, int radius, int mode)
{
	if(mode==0)
	{
		GLUquadricObj *qobj = 0;
		qobj = gluNewQuadric();

		for(int i=0;i<NbStressPoint;i++)
		{
			glColor3f(color[0],color[1],color[2]);
			glPushMatrix();
			glTranslatef((GLfloat)StressPos[i*DIM],(GLfloat)StressPos[i*DIM+1],(GLfloat)StressPos[i*DIM+2]);
			gluSphere(qobj,radius,5,5);
			glPopMatrix();
		}
	}
	if(mode==1)
	{
		glPointSize(radius);
		glBegin(GL_POINTS);
		for(int i=0;i<NbNode;i++)
		{
			glColor3f(color[0],color[1],color[2]);
			glVertex3f((GLfloat)StressPos[i*DIM],(GLfloat)StressPos[i*DIM+1],(GLfloat)StressPos[i*DIM+2]);
		}
		glEnd();
	}
}

void EFG_CUDA_RUNTIME::drawEdge()
{
	for(int i=0;i<Edge->size();i++)
		drawEdge(i);
}

void EFG_CUDA_RUNTIME::drawBVH()
{
	BVHAABB.drawBoundingBox();
}

void EFG_CUDA_RUNTIME::drawBVHPoint()
{
	BVHAABBPoint.drawBoundingBox();
}

void EFG_CUDA_RUNTIME::drawEdge(int idx)
{
	glBegin(GL_LINES);
	glVertex3f((*NodePosVec)[(*Edge)[idx][0]][0],(*NodePosVec)[(*Edge)[idx][0]][1],(*NodePosVec)[(*Edge)[idx][0]][2]);
	glVertex3f((*NodePosVec)[(*Edge)[idx][1]][0],(*NodePosVec)[(*Edge)[idx][1]][1],(*NodePosVec)[(*Edge)[idx][1]][2]);
	glEnd();
}

void EFG_CUDA_RUNTIME::drawNodeInside(Vec3f color, int radius)
{
	glColor3f(color[0],color[1],color[2]);
	GLUquadricObj *qobj = 0;
	qobj = gluNewQuadric();
	for(int i=0;i<NbNodeInside;i++)
	{
		glPushMatrix();
		glTranslatef((GLfloat)(*NodePosVec)[i][0],(GLfloat)(*NodePosVec)[i][1],(GLfloat)(*NodePosVec)[i][2]);
		gluSphere(qobj,radius,20,20);
		glPopMatrix();
	}
}

void EFG_CUDA_RUNTIME::updatePositionExplicitFree(float dt)
{

	for(int i=0;i<NbNode*DIM;i++)
	{
		NodeForce[i]=0;
	}

	//add internal force
	addInternalForce();

	//add gravity
	for(int i=0;i<NbNode;i++)
	{
		NodeForce[i*DIM+1]-=GRAVITY*NodeMass[i];
	}

	//add damping force
	for(int i=0;i<NbNode*DIM;i++)
	{
		NodeForce[i]-=NodeVel[i]*DAMPING;
	}

	//Explicit integration
	
	for(int i=0;i<NbNode;i++)
	{
		for(int j=0;j<DIM;j++)
		{
			float dv=NodeForce[i*DIM+j]/NodeMass[i]*dt;
			//float dv=NodeForce[i*DIM+j]*dt;
			NodeVel[i*DIM+j]+=dv;
			NodeDis[i*DIM+j]+=(NodeVel[i*DIM+j]*dt);
			NodePos[i*DIM+j]=NodePos0[i*DIM+j]+NodeDis[i*DIM+j];
		}
	}

	//Fixed Constraint
	for(unsigned int i=0;i<FixedNodeIdx.size();i++)
	{
		for(int j=0;j<DIM;j++)
		{
			NodePos[FixedNodeIdx[i]*DIM+j]=NodePos0[FixedNodeIdx[i]*DIM+j];
			NodeDis[FixedNodeIdx[i]*DIM+j]=0;
		}
	}

	for(int i=0;i<NbNode;i++)
	{
		for(int j=0;j<3;j++)
			(*NodePosVec)[i][j]=NodePos[i*3+j];
	}

	for(int i=0;i<NbNode*DIM;i++)
	{
		NodeForce[i]=0;
	}
}

void EFG_CUDA_RUNTIME::updatePositionExplicitConst(float dt)
{
	//Explicit integration
	for(int i=0;i<NbNode;i++)
	{
		for(int j=0;j<DIM;j++)
		{
			float dv=NodeForce[i*DIM+j]/NodeMass[i]*dt;
			//float dv=NodeForce[i*DIM+j]*dt;
			NodeDis[i*DIM+j]+=(dv*dt);
			NodePos[i*DIM+j]=NodePos0[i*DIM+j]+NodeDis[i*DIM+j];
		}
	}

	//Fixed Constraint
	for(unsigned int i=0;i<FixedNodeIdx.size();i++)
	{
		for(int j=0;j<DIM;j++)
		{
			NodePos[FixedNodeIdx[i]*DIM+j]=NodePos0[FixedNodeIdx[i]*DIM+j];
			NodeDis[FixedNodeIdx[i]*DIM+j]=0;
		}
	}

}

void EFG_CUDA_RUNTIME::updatePositionExplicitConst(float dt, int n)
{
	//Explicit integration
	for(int i=0;i<NbNode;i++)
	{
		for(int j=0;j<DIM;j++)
		{
			float du=dt*dt*n*(n+1)/(2*NodeMass[i]);
			float dv=NodeForce[i*DIM+j]/NodeMass[i]*dt*n;
			NodeVel[i*DIM+j]+=dv;
			NodeDis[i*DIM+j]+=du;
			NodePos[i*DIM+j]=NodePos0[i*DIM+j]+NodeDis[i*DIM+j];
		}
	}

	//Fixed Constraint
	for(unsigned int i=0;i<FixedNodeIdx.size();i++)
	{
		for(int j=0;j<DIM;j++)
		{
			NodePos[FixedNodeIdx[i]*DIM+j]=NodePos0[FixedNodeIdx[i]*DIM+j];
			NodeDis[FixedNodeIdx[i]*DIM+j]=0;
		}
	}

}

void EFG_CUDA_RUNTIME::updatePositionExplicit(float dt, int itter)
{
	for(int k=0;k<itter;k++)
	{
		

		//add internal force
		addInternalForce();

		//add gravity
		for(int i=0;i<NbNode;i++)
		{
			NodeForce[i*DIM+1]-=GRAVITY*NodeMass[i];
		}

		//add damping force
		for(int i=0;i<NbNode*DIM;i++)
		{
			NodeForce[i]-=NodeVel[i]*DAMPING;
		}

		//add compression force !!!
		// This force help the cut open when the constrained points is release
		for (int i=0; i<NbNode; i++)
		{
			for (int j=0; j<3; j++)
			{
				NodeForce[i*3+j] += m_compressForce[i][j];
			}

		}

		//Explicit integration
		for(int i=0;i<NbNode;i++)
		{
			for(int j=0;j<DIM;j++)
			{
				float dv=NodeForce[i*DIM+j]/NodeMass[i]*dt;
				NodeVel[i*DIM+j]+=dv;
				NodeDis[i*DIM+j]+=(NodeVel[i*DIM+j]*dt);
				NodePos[i*DIM+j]=NodePos0[i*DIM+j]+NodeDis[i*DIM+j];
			}
		}

		//Fixed Constraint
		for(unsigned int i=0;i<FixedNodeIdx.size();i++)
		{
			for(int j=0;j<DIM;j++)
			{
				NodePos[FixedNodeIdx[i]*DIM+j]=NodePos0[FixedNodeIdx[i]*DIM+j];
				NodeDis[FixedNodeIdx[i]*DIM+j]=0;
			}
		}

		for(int i=0;i<NbNode*DIM;i++)
		{
			NodeForce[i]=0;
		}

	}

	for(int i=0;i<NbNode;i++)
	{
		for(int j=0;j<3;j++)
			(*NodePosVec)[i][j]=NodePos[i*3+j];
	}
//	BVHAABB.updateAABBTreeBottomUp();
}


void EFG_CUDA_RUNTIME::updatePositionExplicit_CUDA(float dt, int itter)
{
	CUresult error;
	int size=NbNode*DIM*sizeof(float);

	for(int k=0;k<itter;k++)
	{
		//add internal force
		d_addInternalForce();

		//Explicit integration
		int nbFixedConstraint=FixedNodeIdx.size();
		d_explicitIntegration(dt, nbFixedConstraint);
	}
	cuCtxSynchronize();
	d_getPosition(NodePos);
	d_getDisplacement(NodeDis);

	for(int i=0;i<NbNode;i++)
	{
		for(int j=0;j<3;j++)
			(*NodePosVec)[i][j]=NodePos[i*3+j];
	}
	//BVHAABB.updateAABBTreeBottomUp();
}

void EFG_CUDA_RUNTIME::updatePositionExplicitFree_CUDA(float dt)
{
	CUresult error;
	int size=NbNode*DIM*sizeof(float);

	//add internal force
	d_addInternalForce();

	//Explicit integration
	int nbFixedConstraint=FixedNodeIdx.size();
	d_explicitIntegration(dt, nbFixedConstraint);

	cuCtxSynchronize();
	d_getPosition(NodePos);
	d_getDisplacement(NodeDis);

	for(int i=0;i<NbNode;i++)
	{
		for(int j=0;j<3;j++)
			(*NodePosVec)[i][j]=NodePos[i*3+j];
	}
//	BVHAABB.updateAABBTreeBottomUp(); // Cost too much time here
}

void EFG_CUDA_RUNTIME::updatePositionExplicitConst_CUDA(float dt)
{
	d_setForce(NodeForce);
	d_explicitIntegrationConst(dt);
}

void EFG_CUDA_RUNTIME::addInternalForce()
{
	computeStrain();
	computeStress();
	computeForce();
}

void EFG_CUDA_RUNTIME::d_addInternalForce()
{
	d_computeStrain();
	d_computeStress();
	d_computeForce(DAMPING);
}

void EFG_CUDA_RUNTIME::computeStrain()
{
	for(int i=0;i<NbNode*SDIM;i++)
		NodeStrain[i]=0;

	for(int i=0;i<NbNode;i++)
	{
		for(int j=0;j<NbNeighborNode[i];j++)
		{
			int idx=NB_NEIGHBOR_MAX*i+j;
			float dis[3];

			//dis[0]=NodeDis[NeighborNodeIdx[idx]*DIM]-NodeDis[i*DIM];
			//dis[1]=NodeDis[NeighborNodeIdx[idx]*DIM+1]-NodeDis[i*DIM+1];
			//dis[2]=NodeDis[NeighborNodeIdx[idx]*DIM+2]-NodeDis[i*DIM+2];
			dis[0]=NodeDis[NeighborNodeIdx[idx]*DIM];
			dis[1]=NodeDis[NeighborNodeIdx[idx]*DIM+1];
			dis[2]=NodeDis[NeighborNodeIdx[idx]*DIM+2];

			NodeStrain[i*SDIM]+=ShapeFuncDerivXAtNodeInv[idx]*dis[0];
			NodeStrain[i*SDIM+1]+=ShapeFuncDerivYAtNodeInv[idx]*dis[1];
			NodeStrain[i*SDIM+2]+=ShapeFuncDerivZAtNodeInv[idx]*dis[2];

			NodeStrain[i*SDIM+3]+=(ShapeFuncDerivYAtNodeInv[idx]*dis[0]+ShapeFuncDerivXAtNodeInv[idx]*dis[1]);
			NodeStrain[i*SDIM+4]+=(ShapeFuncDerivZAtNodeInv[idx]*dis[1]+ShapeFuncDerivYAtNodeInv[idx]*dis[2]);
			NodeStrain[i*SDIM+5]+=(ShapeFuncDerivZAtNodeInv[idx]*dis[0]+ShapeFuncDerivXAtNodeInv[idx]*dis[2]);
		}
	}

	if(NbStressPoint>0)
	{
		for(int i=0;i<NbStressPoint*SDIM;i++)
			StressPointStrain[i]=0;

		for(int i=0;i<NbStressPoint;i++)
		{
			for(int j=0;j<NbStressNeighborNode[i];j++)
			{
				int idx=NB_NEIGHBOR_MAX*i+j;
				float dis[3];
				dis[0]=NodeDis[StressNeighborNodeIdx[idx]*DIM];
				dis[1]=NodeDis[StressNeighborNodeIdx[idx]*DIM+1];
				dis[2]=NodeDis[StressNeighborNodeIdx[idx]*DIM+2];

				StressPointStrain[i*SDIM]+=ShapeFuncDerivXAtStressPointInv[idx]*dis[0];
				StressPointStrain[i*SDIM+1]+=ShapeFuncDerivYAtStressPointInv[idx]*dis[1];
				StressPointStrain[i*SDIM+2]+=ShapeFuncDerivZAtStressPointInv[idx]*dis[2];

				StressPointStrain[i*SDIM+3]+=(ShapeFuncDerivYAtStressPointInv[idx]*dis[0]+ShapeFuncDerivXAtStressPointInv[idx]*dis[1]);
				StressPointStrain[i*SDIM+4]+=(ShapeFuncDerivZAtStressPointInv[idx]*dis[1]+ShapeFuncDerivYAtStressPointInv[idx]*dis[2]);
				StressPointStrain[i*SDIM+5]+=(ShapeFuncDerivZAtStressPointInv[idx]*dis[0]+ShapeFuncDerivXAtStressPointInv[idx]*dis[2]);
			}
		}
	}
}

void EFG_CUDA_RUNTIME::computeStress()
{
	for(int i=0;i<NbNode;i++)
	{
		for(int j=0;j<6;j++)
		{
			NodeStress[i*SDIM+j]=0.0;
			for(int k=0;k<6;k++)
			{
				NodeStress[i*SDIM+j]+=MaterialStiffness[j*SDIM+k]*NodeStrain[i*SDIM+k];
			}
		}
	}
	
	if(NbStressPoint>0)
	{
		for(int i=0;i<NbStressPoint;i++)
		{
			for(int j=0;j<6;j++)
			{
				StressPointStress[i*SDIM+j]=0.0;
				for(int k=0;k<6;k++)
				{
					StressPointStress[i*SDIM+j]+=MaterialStiffness[j*SDIM+k]*StressPointStrain[i*SDIM+k];
				}
			}
		}
	}
}

void EFG_CUDA_RUNTIME::computeForce()
{
	for(int i=0;i<NbNode;i++)
	{
		for(int j=0;j<NbNeighborNode[i];j++)
		{
			int idx=NB_NEIGHBOR_MAX*i+j;
			const float shapeFuncDrvX=ShapeFuncDerivXAtNode[idx];
			const float shapeFuncDrvY=ShapeFuncDerivYAtNode[idx];
			const float shapeFuncDrvZ=ShapeFuncDerivZAtNode[idx];
			float* stress=&NodeStress[NeighborNodeIdx[idx]*SDIM];

			NodeForce[i*DIM]-=(shapeFuncDrvX*stress[0]+shapeFuncDrvY*stress[3]+shapeFuncDrvZ*stress[5])*NodeVolume[i];
			NodeForce[i*DIM+1]-=(shapeFuncDrvY*stress[1]+shapeFuncDrvX*stress[3]+shapeFuncDrvZ*stress[4])*NodeVolume[i];
			NodeForce[i*DIM+2]-=(shapeFuncDrvZ*stress[2]+shapeFuncDrvY*stress[4]+shapeFuncDrvX*stress[5])*NodeVolume[i];
		}

		if(NbStressPoint>0)
		{
			for(int j=0;j<NbNeighborStressPoint[i];j++)
			{
				int idx=NB_NEIGHBOR_MAX*i+j;
				int neiIdx=NeighborStressPointIdx[idx];
				const float shapeFuncDrvX=ShapeFuncDerivXAtStressPoint[idx];
				const float shapeFuncDrvY=ShapeFuncDerivYAtStressPoint[idx];
				const float shapeFuncDrvZ=ShapeFuncDerivZAtStressPoint[idx];
				float* stress=&StressPointStress[neiIdx*SDIM];

				float volume=StressVolume[neiIdx];
				NodeForce[i*DIM]-=(shapeFuncDrvX*stress[0]+shapeFuncDrvY*stress[3]+shapeFuncDrvZ*stress[5])*volume;
				NodeForce[i*DIM+1]-=(shapeFuncDrvY*stress[1]+shapeFuncDrvX*stress[3]+shapeFuncDrvZ*stress[4])*volume;
				NodeForce[i*DIM+2]-=(shapeFuncDrvZ*stress[2]+shapeFuncDrvY*stress[4]+shapeFuncDrvX*stress[5])*volume;
			}
		}
	}
}

void EFG_CUDA_RUNTIME::computeNodeRotation()
{
	updatePositionAtNodeCoord();

	if(NodeRot.empty())
		NodeRot.resize(NbNode);

	std::vector<Mat3x3f> rot; rot.resize(NbNode);
	for(int i=0;i<NbNode;i++)
	{
		Vec3f x=(*NodePosXVec)[i]-(*NodePosVec)[i]; x.normalize();
		Vec3f y=(*NodePosYVec)[i]-(*NodePosVec)[i]; y.normalize();
		Vec3f z=x.cross(y);
		y=z.cross(x);
		for(int j=0;j<3;j++)
		{
			rot[i](j,0)=x[j];
			rot[i](j,1)=y[j];
			rot[i](j,2)=z[j];
		}
	}
	for(int i=0;i<NbNode;i++)
	{
		for(int j=0;j<NbNeighborNode[i];j++)
		{
			int idx=NeighborNodeIdx[NB_NEIGHBOR_MAX*i+j];
			NodeRot[i]+=rot[idx];
		}
		NodeRot[i].normalize();
	}
}

void EFG_CUDA_RUNTIME::updatePositionAtNodeCoord()
{
}

void EFG_CUDA_RUNTIME::drawNodeOrientation()
{
	computeNodeRotation();
	if(!NodeRot.empty())
	{
		for(int i=0;i<NbNode;i++)
			drawCoord((*NodePosVec)[i], NodeRot[i]);
	}
}

void EFG_CUDA_RUNTIME::drawCoord(Vec3f pos, Mat3x3f rot)
{
	float length=10;

	Vec3f x(length,0,0);
	Vec3f y(0,length,0);
	Vec3f z(0,0,length);

	x=rot*x;
	y=rot*y;
	z=rot*z;

	glBegin(GL_LINES);
	glColor3f(1,0,0);
	glVertex3f(pos[0],pos[1],pos[2]);
	glVertex3f(pos[0]+x[0],pos[1]+x[1],pos[2]+x[2]);

	glColor3f(0,1,0);
	glVertex3f(pos[0],pos[1],pos[2]);
	glVertex3f(pos[0]+y[0],pos[1]+y[1],pos[2]+y[2]);

	glColor3f(0,0,1);
	glVertex3f(pos[0],pos[1],pos[2]);
	glVertex3f(pos[0]+z[0],pos[1]+z[1],pos[2]+z[2]);
	glEnd();
}

int EFG_CUDA_RUNTIME::nbNeighborNodeMax()
{
	int max=-100000000;
	for(int i=0;i<NbNode;i++)
	{
		int n=NbNeighborNode[i];
		if(max<n)
			max=n;
	}
	return max;
}

int EFG_CUDA_RUNTIME::nbNeighborNodeMin()
{
	int min=100000000;
	for(int i=0;i<NbNode;i++)
	{
		int n=NbNeighborNode[i];
		if(min>n)
			min=n;
	}
	return min;
}

float EFG_CUDA_RUNTIME::nbNeighborNodeAve()
{
	float ave=0;
	for(int i=0;i<NbNode;i++)
	{
		ave+=NbNeighborNode[i];
	}
	ave/=(float)NbNode;
	return ave;
}

void EFG_CUDA_RUNTIME::removeEdges(std::vector<int>& idx)
{
	VectorFunc vectorFunc;
	vectorFunc.arrangeVector(idx);

	removeEdgesInPhysicalModel(idx);
	removeEdgesInCollisionModel(idx);
}

void EFG_CUDA_RUNTIME::removeEdgesInPhysicalModel(std::vector<int>& idx)
{
	std::vector<int> updatedNodeIdx;
	std::vector<int> updatedNeighborIdx;
	std::vector<int> updatedNeighborValue;
	VectorFunc vectorFunc;

	// 1. update neighbor information
	for(int i=idx.size()-1;i>=0;i--)
	{
		Vec2i edge=(*Edge)[idx[i]];
		updatedNodeIdx.push_back(edge[0]);
		updatedNodeIdx.push_back(edge[1]);
		updateNeighborInfo(idx[i], updatedNeighborIdx, updatedNeighborValue);
	}
	vectorFunc.arrangeVector(updatedNodeIdx);
	if(updatedNodeIdx.size()>0)
	{
		fprintf(F,"NbUpdated: %d\n", updatedNodeIdx.size());
		NbUpdated+=updatedNodeIdx.size();
		Count++;
	}

	int* updatedIdx=new int[updatedNeighborIdx.size()];
	int* updatedValue=new int[updatedNeighborValue.size()];
	for(int i=0;i<updatedNeighborIdx.size();i++)
	{
		updatedIdx[i]=updatedNeighborIdx[i];
		updatedValue[i]=updatedNeighborValue[i];
	}

	// 1-1 update gpu memory
	d_updateNbNeighborNodeFull(NbNeighborNode, (NbNode+NbNodeAddedMax)*sizeof(int));
	d_updateNeighborNodeIdxFull(NeighborNodeIdx, (NbNode+NbNodeAddedMax)*NB_NEIGHBOR_MAX*sizeof(int));
	//d_updateNeighborNodeIdxPartial(updatedValue, updatedIdx, updatedNeighborIdx.size());
	cuCtxSynchronize();
	delete [] updatedIdx;
	delete [] updatedValue;


	// 2. update shape functions
	std::vector<int> updatedShapeFuncIdx;
	std::vector<Vec3f> updatedShapeFuncVal;
	std::vector<Vec3f> updatedShapeFuncValInv;
	updateShapeFuncValue(updatedNodeIdx, updatedShapeFuncIdx, updatedShapeFuncVal, updatedShapeFuncValInv);

	int* shapeFuncIdx=new int[updatedShapeFuncIdx.size()*3];
	float* shapeFuncVal=new float[updatedShapeFuncIdx.size()*3];
	float* shapeFuncValInv=new float[updatedShapeFuncIdx.size()*3];
	for(int i=0;i<updatedShapeFuncIdx.size();i++)
	{
		shapeFuncIdx[i*3]=updatedShapeFuncIdx[i];
		shapeFuncIdx[i*3+1]=updatedShapeFuncIdx[i]+(NbNode+NbNodeAddedMax)*NB_NEIGHBOR_MAX;
		shapeFuncIdx[i*3+2]=updatedShapeFuncIdx[i]+(NbNode+NbNodeAddedMax)*NB_NEIGHBOR_MAX*2;
		shapeFuncVal[i*3]=updatedShapeFuncVal[i][0];
		shapeFuncVal[i*3+1]=updatedShapeFuncVal[i][1];
		shapeFuncVal[i*3+2]=updatedShapeFuncVal[i][2];

		shapeFuncValInv[i*3]=updatedShapeFuncValInv[i][0];
		shapeFuncValInv[i*3+1]=updatedShapeFuncValInv[i][1];
		shapeFuncValInv[i*3+2]=updatedShapeFuncValInv[i][2];
	}

	// 2-1 update gpu memory
	d_updateShapeFuncDrvPartial(shapeFuncVal, shapeFuncIdx, updatedShapeFuncIdx.size()*3);
	d_updateShapeFuncDrvInvPartial(shapeFuncValInv, shapeFuncIdx, updatedShapeFuncIdx.size()*3);
	cuCtxSynchronize();
	delete [] shapeFuncIdx;
	delete [] shapeFuncVal;
	delete [] shapeFuncValInv;

}
void EFG_CUDA_RUNTIME::removeEdgesInCollisionModel(std::vector<int>& idx)
{
	for(int i=idx.size()-1;i>=0;i--)
	{
		int edgeIdxEnd=Edge->size()-1;

		// 1. Update BVH
		AABBNode* node=BVHAABB.findLeafNode(idx[i]);ASSERT(node);
		if(node)
			BVHAABB.removeNode(node);
		node=BVHAABB.findLeafNode(edgeIdxEnd);ASSERT(node);
		if(node)
			node->IndexInLeafNode=idx[i];

		// 2. Remove edge
		(*Edge)[idx[i]]=(*Edge)[edgeIdxEnd];
		Edge->pop_back();
	}
}

void EFG_CUDA_RUNTIME::updateNeighborInfo(int idx, std::vector<int>& updatedIdx, std::vector<int>& updatedValue)
{
	//1. update neighbor information
	int idx1=(*Edge)[idx][0];
	int idx2=(*Edge)[idx][1];

	for(int i=0;i<NbNeighborNode[idx1];i++)
	{
		int _idx=NB_NEIGHBOR_MAX*idx1+i;
		if(NeighborNodeIdx[_idx]==idx2)
		{
			NeighborNodeIdx[_idx]=NeighborNodeIdx[NB_NEIGHBOR_MAX*idx1+NbNeighborNode[idx1]-1];
			updatedIdx.push_back(_idx);
			updatedValue.push_back(NeighborNodeIdx[_idx]);
			NbNeighborNode[idx1]--;
			break;
		}
	}

	for(int i=0;i<NbNeighborNode[idx2];i++)
	{
		int _idx=NB_NEIGHBOR_MAX*idx2+i;
		if(NeighborNodeIdx[_idx]==idx1)
		{
			NeighborNodeIdx[_idx]=NeighborNodeIdx[NB_NEIGHBOR_MAX*idx2+NbNeighborNode[idx2]-1];
			updatedIdx.push_back(_idx);
			updatedValue.push_back(NeighborNodeIdx[_idx]);
			NbNeighborNode[idx2]--;
			break;
		}
	}
}

void EFG_CUDA_RUNTIME::updateShapeFuncValue(std::vector<int>& updatedNodeIdx, std::vector<int>& updatedShapeFuncIdx, std::vector<Vec3f>& updatedVal, std::vector<Vec3f>& updatedValInv)
{
	//1. update neighbor information
	for(int i=0;i<updatedNodeIdx.size();i++)
		ShapeFunc.computeMomentMatrixAtNode(MomentMatrixAtNode[updatedNodeIdx[i]], &NeighborNodeIdx[NB_NEIGHBOR_MAX*updatedNodeIdx[i]], NbNeighborNode[updatedNodeIdx[i]], updatedNodeIdx[i]);

	//2. Compute moment matrix inverse at each node
	for(int i=0;i<updatedNodeIdx.size();i++)
		invertMatrix(MomentMatrixInvAtNode[updatedNodeIdx[i]], MomentMatrixAtNode[updatedNodeIdx[i]]);

	//3. Compute moment matrix derivative at each node
	for(int i=0;i<updatedNodeIdx.size();i++)
	{
		ShapeFunc.computeMomentMatrixDrvXAtNode(updatedNodeIdx[i], MomentMatrixDrvXAtNode[updatedNodeIdx[i]], &NeighborNodeIdx[NB_NEIGHBOR_MAX*updatedNodeIdx[i]], NbNeighborNode[updatedNodeIdx[i]]);
		ShapeFunc.computeMomentMatrixDrvYAtNode(updatedNodeIdx[i], MomentMatrixDrvYAtNode[updatedNodeIdx[i]], &NeighborNodeIdx[NB_NEIGHBOR_MAX*updatedNodeIdx[i]], NbNeighborNode[updatedNodeIdx[i]]);
		ShapeFunc.computeMomentMatrixDrvZAtNode(updatedNodeIdx[i], MomentMatrixDrvZAtNode[updatedNodeIdx[i]], &NeighborNodeIdx[NB_NEIGHBOR_MAX*updatedNodeIdx[i]], NbNeighborNode[updatedNodeIdx[i]]);
	}

	int shapeFuncSize=(NbNode+NbNodeAddedMax)*NB_NEIGHBOR_MAX;
	//4. Compute shape function value and derivative 
	for(int i=0;i<updatedNodeIdx.size();i++)
	{
		int idx=updatedNodeIdx[i];
		for(unsigned int j=0;j<NbNeighborNode[idx];j++)
		{
			int _idx=NB_NEIGHBOR_MAX*idx+j;
			int neiIdx=NeighborNodeIdx[_idx];
			ShapeFuncAtNode[_idx]=ShapeFunc.computeShapeFuncValue(idx, (*NodePos0Vec)[neiIdx], MomentMatrixInvAtNode[neiIdx]);
			int idx1=-1;
			for(int k=0;k<NbNeighborNode[neiIdx];k++)
			{
				if(NeighborNodeIdx[NB_NEIGHBOR_MAX*neiIdx+k]==idx)
				{
					idx1=k;
					break;
				}
			}
			idx1=NB_NEIGHBOR_MAX*neiIdx+idx1;
			ShapeFuncAtNode[idx1]=ShapeFunc.computeShapeFuncValue(neiIdx, (*NodePos0Vec)[idx], MomentMatrixInvAtNode[idx]);

			Vec3f val;
			updatedShapeFuncIdx.push_back(_idx);
			updatedShapeFuncIdx.push_back(idx1);
			ShapeFuncDerivXAtNode[_idx]=ShapeFunc.computeShapeFuncDrvX(idx, (*NodePos0Vec)[neiIdx], MomentMatrixInvAtNode[neiIdx], MomentMatrixDrvXAtNode[neiIdx]);
			ShapeFuncDerivYAtNode[_idx]=ShapeFunc.computeShapeFuncDrvY(idx, (*NodePos0Vec)[neiIdx], MomentMatrixInvAtNode[neiIdx], MomentMatrixDrvYAtNode[neiIdx]);
			ShapeFuncDerivZAtNode[_idx]=ShapeFunc.computeShapeFuncDrvZ(idx, (*NodePos0Vec)[neiIdx], MomentMatrixInvAtNode[neiIdx], MomentMatrixDrvZAtNode[neiIdx]);
			ShapeFuncDerivXAtNode[idx1]=ShapeFunc.computeShapeFuncDrvX(neiIdx, (*NodePos0Vec)[idx], MomentMatrixInvAtNode[idx], MomentMatrixDrvXAtNode[idx]);
			ShapeFuncDerivYAtNode[idx1]=ShapeFunc.computeShapeFuncDrvY(neiIdx, (*NodePos0Vec)[idx], MomentMatrixInvAtNode[idx], MomentMatrixDrvYAtNode[idx]);
			ShapeFuncDerivZAtNode[idx1]=ShapeFunc.computeShapeFuncDrvZ(neiIdx, (*NodePos0Vec)[idx], MomentMatrixInvAtNode[idx], MomentMatrixDrvZAtNode[idx]);

			ShapeFuncDerivAtNode[_idx]=ShapeFuncDerivXAtNode[_idx];
			ShapeFuncDerivAtNode[idx1]=ShapeFuncDerivXAtNode[idx1];
			ShapeFuncDerivAtNode[_idx+shapeFuncSize]=ShapeFuncDerivYAtNode[_idx];
			ShapeFuncDerivAtNode[idx1+shapeFuncSize]=ShapeFuncDerivYAtNode[idx1];
			ShapeFuncDerivAtNode[_idx+shapeFuncSize*2]=ShapeFuncDerivZAtNode[_idx];
			ShapeFuncDerivAtNode[idx1+shapeFuncSize*2]=ShapeFuncDerivZAtNode[idx1];

			ShapeFuncDerivXAtNodeInv[_idx]=ShapeFuncDerivXAtNode[idx1];
			ShapeFuncDerivYAtNodeInv[_idx]=ShapeFuncDerivYAtNode[idx1];
			ShapeFuncDerivZAtNodeInv[_idx]=ShapeFuncDerivZAtNode[idx1];
			ShapeFuncDerivXAtNodeInv[idx1]=ShapeFuncDerivXAtNode[_idx];
			ShapeFuncDerivYAtNodeInv[idx1]=ShapeFuncDerivYAtNode[_idx];
			ShapeFuncDerivZAtNodeInv[idx1]=ShapeFuncDerivZAtNode[_idx];

			ShapeFuncDerivAtNodeInv[_idx]=ShapeFuncDerivXAtNodeInv[_idx];
			ShapeFuncDerivAtNodeInv[idx1]=ShapeFuncDerivXAtNodeInv[idx1];
			ShapeFuncDerivAtNodeInv[_idx+shapeFuncSize]=ShapeFuncDerivYAtNodeInv[_idx];
			ShapeFuncDerivAtNodeInv[idx1+shapeFuncSize]=ShapeFuncDerivYAtNodeInv[idx1];
			ShapeFuncDerivAtNodeInv[_idx+shapeFuncSize*2]=ShapeFuncDerivZAtNodeInv[_idx];
			ShapeFuncDerivAtNodeInv[idx1+shapeFuncSize*2]=ShapeFuncDerivZAtNodeInv[idx1];

			val=Vec3f(ShapeFuncDerivXAtNode[_idx], ShapeFuncDerivYAtNode[_idx], ShapeFuncDerivZAtNode[_idx]);
			updatedVal.push_back(val);
			val=Vec3f(ShapeFuncDerivXAtNode[idx1], ShapeFuncDerivYAtNode[idx1], ShapeFuncDerivZAtNode[idx1]);
			updatedVal.push_back(val);
			val=Vec3f(ShapeFuncDerivXAtNodeInv[_idx], ShapeFuncDerivYAtNodeInv[_idx], ShapeFuncDerivZAtNodeInv[_idx]);
			updatedValInv.push_back(val);
			val=Vec3f(ShapeFuncDerivXAtNodeInv[idx1], ShapeFuncDerivYAtNodeInv[idx1], ShapeFuncDerivZAtNodeInv[idx1]);
			updatedValInv.push_back(val);
		}
	}
}

void EFG_CUDA_RUNTIME::setForce()
{
	d_setForce(NodeForce);
}

double* EFG_CUDA_RUNTIME::returnInternalForce(float dt)
{

		
	for(int i=0;i<NbNode*DIM;i++)	
	{
		NodeForce[i]=0;
	}

	//add internal force
	addInternalForce();

	//add gravity
	for(int i=0;i<NbNode;i++)
	{
		NodeForce[i*DIM+1]-=GRAVITY*NodeMass[i];
	}

	//add damping force
	for(int i=0;i<NbNode*DIM;i++)
	{
		NodeForce[i]-=NodeVel[i]*DAMPING;
	}

	return (double*)NodeForce;

}

double* EFG_CUDA_RUNTIME::returnVertorFormExplicitCompliancematrix(double dt,int itter)
{
	double* C;
	C=new double[DIM*NbNode];
	//Fixed Constraint
	for(int i=0;i<DIM*NbNode;i++)
	{
		C[i]=dt*dt*itter*(itter+1)/2;
	}

	for(unsigned int i=0;i<FixedNodeIdx.size();i++)
	{
		for(int j=0;j<DIM;j++)
		{
			C[FixedNodeIdx[i]*DIM+j]=0;	
		}
	}


	return C;

	delete [] C;

}

void EFG_CUDA_RUNTIME::returnVertorFormExplicitCompliancematrix(double dt,int itter,double* &C)
{
	//Fixed Constraint
	for(int i=0;i<DIM*NbNode;i++)
	{
		C[i]=dt*dt*itter*(itter+1)/2;
	}

	for(unsigned int i=0;i<FixedNodeIdx.size();i++)
	{
		for(int j=0;j<DIM;j++)
		{
			C[FixedNodeIdx[i]*DIM+j]=0;	
		}
	}
}

float** EFG_CUDA_RUNTIME::returnPreDis_CUDA(float dt, int itter)
{
	for(int i=0;i<NbNode*DIM;i++)
	{
		NodePreDis[i]=NodePos[i]-NodePos0[i];
	}

	synchronizeHostAndDevide(SYNC_HOST_TO_DEVICE);

	for(int k=0;k<itter;k++)
	{
		//add internal force
		d_addInternalForce();

// 		d_getDisplacement(NodeDis);
// 		releaseLog::logMatrix(NodeDis, NbNode*DIM, 1, "Node dis.txt");

		//Explicit integration
		int nbFixedConstraint=FixedNodeIdx.size();
		d_explicitIntegration(dt, nbFixedConstraint);

// 		d_getDisplacement(NodeDis);
// 		releaseLog::logMatrix(NodeDis, NbNode*DIM, 1, "Node dis 1.txt");
		//
	}
		d_getDisplacement(NodeDis);
		releaseLog::logMatrix(NodeDis, NbNode*DIM, 1, "Node dis 1.txt");
	cuCtxSynchronize();
	synchronizeHostAndDevide(SYNC_DEVICE_TO_HOST);

	for(int i=0;i<NbNode*DIM;i++)
	{
		NodePreDis[i]=NodeDis[i]-NodePreDis[i];
	}



	return &NodePreDis;
}

float** EFG_CUDA_RUNTIME::returnPreDis(float dt, int itter)
{
	for(int i=0;i<NbNode*DIM;i++)
	{
		NodePreDis[i]=NodePos[i]-NodePos0[i];
	}

	for(int k=0;k<itter;k++)
	{
		//add internal force
		addInternalForce();

		//add gravity
		for(int i=0;i<NbNode;i++)
		{
			NodeForce[i*DIM+1]-=GRAVITY*NodeMass[i];
		}

		//add damping force
		for(int i=0;i<NbNode*DIM;i++)
		{
			NodeForce[i]-=NodeVel[i]*DAMPING;
		}

		//Explicit integration
		for(int i=0;i<NbNode;i++)
		{
			for(int j=0;j<DIM;j++)
			{
				float dv=NodeForce[i*DIM+j]/NodeMass[i]*dt;
				NodeVel[i*DIM+j]+=dv;
				NodeDis[i*DIM+j]+=(NodeVel[i*DIM+j]*dt);
			}
		}


		//Fixed Constraint
		for(unsigned int i=0;i<FixedNodeIdx.size();i++)
		{
			for(int j=0;j<DIM;j++)
			{
				NodeDis[FixedNodeIdx[i]*DIM+j]=0;
			}
		}

		for(int i=0;i<NbNode*DIM;i++)
		{
			NodeForce[i]=0;
		}

	}
	
	for(int i=0;i<NbNode*DIM;i++)
	{
		NodePreDis[i]=NodeDis[i]-NodePreDis[i];
	}

	return &NodePreDis;
}

float** EFG_CUDA_RUNTIME::returnDis(float dt, int itter)
{
	for(int k=0;k<itter;k++)
	{


		//add internal force
		addInternalForce();

		//add gravity
		for(int i=0;i<NbNode;i++)
		{
			NodeForce[i*DIM+1]-=GRAVITY*NodeMass[i];
		}

		//add damping force
		for(int i=0;i<NbNode*DIM;i++)
		{
			NodeForce[i]-=NodeVel[i]*DAMPING;
		}

		//Explicit integration
		for(int i=0;i<NbNode;i++)
		{
			for(int j=0;j<DIM;j++)
			{
				float dv=NodeForce[i*DIM+j]/NodeMass[i]*dt;
				NodeVel[i*DIM+j]+=dv;
				NodeDis[i*DIM+j]+=(NodeVel[i*DIM+j]*dt);
			}
		}

		//Fixed Constraint
		for(unsigned int i=0;i<FixedNodeIdx.size();i++)
		{
			for(int j=0;j<DIM;j++)
			{
			//	NodePos[FixedNodeIdx[i]*DIM+j]=NodePos0[FixedNodeIdx[i]*DIM+j];
				NodeDis[FixedNodeIdx[i]*DIM+j]=0;
			}
		}

		for(int i=0;i<NbNode*DIM;i++)
		{
			NodeForce[i]=0;
		}

	}
	return &NodeDis;
}

void EFG_CUDA_RUNTIME::updatePosition(double* Dis)
{
	for(int i=0;i<NbNode;i++)
	{
		for(int j=0;j<DIM;j++)
		{
			float temp=Dis[i*DIM+j];
			NodeDis[i*DIM+j]+=Dis[i*DIM+j];
			NodePos[i*DIM+j]=NodePos0[i*DIM+j]+NodeDis[i*DIM+j];
		}
	}
	//Fixed Constraint
	for(unsigned int i=0;i<FixedNodeIdx.size();i++)
	{
		for(int j=0;j<DIM;j++)
		{
			NodePos[FixedNodeIdx[i]*DIM+j]=NodePos0[FixedNodeIdx[i]*DIM+j];
			NodeDis[FixedNodeIdx[i]*DIM+j]=0;
		}
	}

	for(int i=0;i<NbNode;i++)
	{
		for(int j=0;j<3;j++){
			(*NodePosVec)[i][j]=NodePos[i*3+j];
		}
	}
	BVHAABB.updateAABBTreeBottomUp();
}

void EFG_CUDA_RUNTIME::updatePosition(double* nodeforce, double C, double dt, int itter)
{
	for(int i=0;i<NbNode;i++)
	{
		for(int j=0;j<DIM;j++)
		{
			float dv=nodeforce[i*DIM+j]/NodeMass[i]*dt*itter;
			float du=nodeforce[i*DIM+j]/NodeMass[i]*itter*(itter+1)/2*dt*dt;
			NodeVel[i*DIM+j]+=dv;
			NodeDis[i*DIM+j]+=du;
			NodePos[i*DIM+j]=NodePos0[i*DIM+j]+NodeDis[i*DIM+j];
		}
	}

	//Fixed Constraint
	for(unsigned int i=0;i<FixedNodeIdx.size();i++)
	{
		for(int j=0;j<DIM;j++)
		{
			NodePos[FixedNodeIdx[i]*DIM+j]=NodePos0[FixedNodeIdx[i]*DIM+j];
			NodeVel[FixedNodeIdx[i]*DIM+j]=0;
			NodeDis[FixedNodeIdx[i]*DIM+j]=0;
		}
	}

	for(int i=0;i<NbNode;i++)
	{
		for(int j=0;j<3;j++){
			(*NodePosVec)[i][j]=NodePos[i*3+j];
		}
	}

//	BVHAABB.updateAABBTreeBottomUp();
}



void EFG_CUDA_RUNTIME::updatePosition()
{
	for(int i=0;i<NbNode;i++)
	{
		for(int j=0;j<DIM;j++)
		{
			NodePos[i*DIM+j]=NodePos0[i*DIM+j]+NodeDis[i*DIM+j];
		}
	}

	//Fixed Constraint
	for(unsigned int i=0;i<FixedNodeIdx.size();i++)
	{
		for(int j=0;j<DIM;j++)
		{
			NodePos[FixedNodeIdx[i]*DIM+j]=NodePos0[FixedNodeIdx[i]*DIM+j];
			NodeDis[FixedNodeIdx[i]*DIM+j]=0;
		}
	}

	for(int i=0;i<NbNode;i++)
	{
		for(int j=0;j<3;j++){
			(*NodePosVec)[i][j]=NodePos[i*3+j];
		}
	}

	BVHAABB.updateAABBTreeBottomUp();
}

void EFG_CUDA_RUNTIME::addDisplacement(int index, double Value)
{		
	NodeDis[index]+=Value;		
}

void EFG_CUDA_RUNTIME::synchronizeHostAndDevide( int mode )
{
	if (mode == SYNC_DEVICE_TO_HOST)
	{
		d_getPosition(NodePos);
		d_setDisplacement(NodeDis);
	}
	else if (mode == SYNC_HOST_TO_DEVICE)
	{
		d_setPosition(NodePos);
		d_setDisplacement(NodeDis);
	}
}

float** EFG_CUDA_RUNTIME::returnPreDisNoDeform()
{
	for(int i=0;i<NbNode*DIM;i++)
	{
		NodePreDis[i]=NodePos[i]-NodePos0[i];
	}

	return &NodePreDis;
}

void EFG_CUDA_RUNTIME::addConstraintIdx( arrayInt fixIdxs )
{
	FixedNodeIdx.insert(FixedNodeIdx.end(), fixIdxs.begin(), fixIdxs.end());
}
