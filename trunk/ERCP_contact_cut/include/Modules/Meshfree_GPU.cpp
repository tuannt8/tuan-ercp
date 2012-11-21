#include "stdafx.h"
#include "Meshfree_GPU.h"

Meshfree_GPU::Meshfree_GPU(void)
{
	SurfObj=NULL;
	EFGObj=NULL;
	MappingMatrix=NULL;
	F=fopen("MeshfreeGPU.txt","w");
}

Meshfree_GPU::~Meshfree_GPU(void)
{
	fprintf(F,"EFGAve: %f\n", EFGTime/Count);
	fprintf(F,"SurfAve: %f\n", SurfTime/Count);
	fclose(F);
	if(SurfObj)
		delete SurfObj;
	if(EFGObj)
		delete EFGObj;
	if(MappingMatrix)
	{
		for (int i=0;i<SurfObj->nbPoint();i++)
		{
			delete [] MappingMatrix[i];
		}
	}
}

void Meshfree_GPU::loadSurfObj(char* filename)
{
	SurfObj=new SurfaceObj;
	SurfObj->readObjData(filename);
	SurfObj->constructAABBTree();
}

void Meshfree_GPU::generateEFGObj(int res, bool stress)
{
	MeshfreeNodeGenerator nodeGenerator;
	nodeGenerator.setSurfObj(SurfObj);

	if(stress)
	{
		nodeGenerator.generateUniformNodeWithStressPoint(res);
		SupportRadius=nodeGenerator.boxSize().norm()*1.2;
		SupportRadiusSurf=SupportRadius*1.2;
		EFGObj=new EFG_CUDA_RUNTIME;
		EFGObj->init(nodeGenerator.nodePos(), nodeGenerator.stressPoint(), nodeGenerator.nodeVolume(), SupportRadius);
	}
	else
	{
		nodeGenerator.generateUniformNode(res);
		SupportRadius=nodeGenerator.boxSize().norm()*1.2;
		SupportRadiusSurf=SupportRadius*1.2;
		EFGObj=new EFG_CUDA_RUNTIME;
		EFGObj->init(nodeGenerator.nodePos(), nodeGenerator.nodeVolume(), SupportRadius);
	}
}

void Meshfree_GPU::initFixedConstraintGPU()
{
	EFGObj->initFixedConstraintGPU();
}

void Meshfree_GPU::drawSurfObj(Vec3f color, int mode)
{
	if(mode==0)
		SurfObj->drawObject(color);
	else
		SurfObj->drawWireFrame(color);
}

void Meshfree_GPU::drawEFGObj(Vec3f color, float radius, int mode)
{
	EFGObj->draw(color, radius, mode);
}

void Meshfree_GPU::updatePositionExplicit(float dt, int itter)
{
	EFGObj->updatePositionExplicit(dt, itter);

	updateSurfPosition();
}

void Meshfree_GPU::updatePositionExplicitFree_CPU(float dt)
{
	EFGObj->updatePositionExplicitFree(dt);
}

void Meshfree_GPU::updatePositionExplicitConst_CPU(float dt)
{
	EFGObj->updatePositionExplicitConst(dt);
}

void Meshfree_GPU::updatePositionExplicitConst_CPU(float dt,int n)
{
	EFGObj->updatePositionExplicitConst(dt,n);
}

void Meshfree_GPU::updatePositionExplicitFree(float dt)
{
	EFGObj->updatePositionExplicitFree_CUDA(dt);
	updateSurfPosition();
}

void Meshfree_GPU::updatePositionExplicitConst(float dt)
{
	EFGObj->updatePositionExplicitConst_CUDA(dt);
}

void Meshfree_GPU::boxConstraint(Vec3f leftDown, Vec3f rightUp)
{
	EFGObj->boxConstraint(leftDown, rightUp);
}

void Meshfree_GPU::connectSurfAndEFG()
{
	float support=SupportRadiusSurf;
	initSurfaceNeighborNode(support);
	initShapeFuncValueAtSurfPoint(support);
	BVHAABB.init(SurfObj->point(), EFGObj->nodePosVec(), &Edge);
	BVHAABB.constructAABBTree();
}

void Meshfree_GPU::initSurfaceNeighborNode(float supportRadius)
{
	int nbSurfPoint=SurfObj->point()->size();
	int nbNode=EFGObj->nbNode();
	std::vector<Vec3f>* surfPoint=SurfObj->point();
	std::vector<Vec3f>* node=EFGObj->nodePos0Vec();

	//memory allocation
	NeighborNodeOfSurfVertice.resize(nbSurfPoint);

	//search and save neighbor information
	for(int i=0;i<nbSurfPoint;i++)
	{
		for(int j=0;j<nbNode;j++)
		{
			if(((*surfPoint)[i]-(*node)[j]).norm()<supportRadius)
			{
				NeighborNodeOfSurfVertice[i].push_back(j);
				Vec2i edge(i,j);
				Edge.push_back(edge);
			}
		}
	}
}

void Meshfree_GPU::initShapeFuncValueAtSurfPoint(float support)
{
	MLSShapeFunc* shapeFunc=new MLSShapeFunc;
	shapeFunc->init(EFGObj->nodePosVec(), support, WEIGHT_FUNC_TYPE);
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

void Meshfree_GPU::updateSurfPosition()
{
	int nbSurfPoint=SurfObj->point()->size();
	std::vector<Vec3f>* surfDis=SurfObj->dis();
	float* nodeDis=EFGObj->nodeDis();

	for(int i=0;i<nbSurfPoint;i++)
	{
		(*surfDis)[i].clear();
		for(int j=0;j<NeighborNodeOfSurfVertice[i].size();j++)
		{
			Vec3f dis; 
			dis[0]=nodeDis[NeighborNodeOfSurfVertice[i][j]*3];
			dis[1]=nodeDis[NeighborNodeOfSurfVertice[i][j]*3+1];
			dis[2]=nodeDis[NeighborNodeOfSurfVertice[i][j]*3+2];
			(*surfDis)[i]+=dis*ShapeFuncValueAtSurfPoint[i][j];
		}
	}
	SurfObj->updatePoint();
	SurfObj->updateBVH();
}


void Meshfree_GPU::makeMappingMatrix()
{
	int nbSurfPoint=SurfObj->point()->size();
	int nbPoint=EFGObj->nbNode();
	float* nodeDis=EFGObj->nodeDis();

	MappingMatrix=new double*[nbSurfPoint];
	for(int i=0;i<nbSurfPoint;i++){
		MappingMatrix[i]=new double[3*nbPoint];
	}

	for(int i=0;i<nbSurfPoint;i++)
	{
		for(int j=0;j<3*nbPoint;j++)
		{
			MappingMatrix[i][j]=0;
		}
	}

	for(int i=0;i<nbSurfPoint;i++)
	{
		for(int j=0;j<NeighborNodeOfSurfVertice[i].size();j++)
		{
			MappingMatrix[i][NeighborNodeOfSurfVertice[i][j]*3]=ShapeFuncValueAtSurfPoint[i][j];
			MappingMatrix[i][NeighborNodeOfSurfVertice[i][j]*3+1]=ShapeFuncValueAtSurfPoint[i][j];
			MappingMatrix[i][NeighborNodeOfSurfVertice[i][j]*3+2]=ShapeFuncValueAtSurfPoint[i][j];
		}
	}
}



//double* Meshfree_GPU::returnMappingMatrix(int triIdx,Vec3d Pos,Vec3d normal)
//{
//	double* m;
//	int NbPoint=EFGObj->nbNode();
//
//	m=new double[3*NbPoint];
//	for(int i=0;i<3*NbPoint;i++)
//	{
//		m[i]=0;
//	}
//	std::vector<Vec3f>* point=SurfObj->point();
//	std::vector<Vec3i>* face=SurfObj->face();
//
//	double A;
//	double Ai;
//	double N;
//	int ii[2];
//	int Index;
//
//	A=(((*point)[(*face)[triIdx][1]]-(*point)[(*face)[triIdx][0]]).cross((*point)[(*face)[triIdx][2]]-(*point)[(*face)[triIdx][0]])).norm();
//
//	for(int i=0;i<3;i++)
//	{
//		ii[0]=(i+1)%3;
//		ii[1]=(i+2)%3;
//		Ai=(((*point)[(*face)[triIdx][ii[1]]]-(*point)[(*face)[triIdx][ii[0]]]).cross(Pos-(*point)[(*face)[triIdx][ii[0]]])).norm();
//		N=Ai/A;
//		Index=(*face)[triIdx][i];
//
//		for(int k=0;k<3*NbPoint;k++){	
//			m[k]+=MappingMatrix[Index][k];//*normal[k%3]*N;
//		}
//
//	}
//
//	return m;
//
//	delete [] m;
//}

double* Meshfree_GPU::returnMappingMatrix(int triIdx,Vec3d Pos,Vec3d normal)
{
	double* m;
	int NbPoint=EFGObj->nbNode();

	m=new double[3*NbPoint];
	for(int i=0;i<3*NbPoint;i++)
	{
		m[i]=0;
	}
	std::vector<Vec3f>* point=SurfObj->point();
	std::vector<Vec3i>* face=SurfObj->face();

	double A;
	double Ai;
	double N;
	int ii[2];
	int Index;
	int NodeIndex;

	A=(((*point)[(*face)[triIdx][1]]-(*point)[(*face)[triIdx][0]]).cross((*point)[(*face)[triIdx][2]]-(*point)[(*face)[triIdx][0]])).norm();

	for(int i=0;i<3;i++)
	{
		ii[0]=(i+1)%3;
		ii[1]=(i+2)%3;
		Ai=(((*point)[(*face)[triIdx][ii[1]]]-(*point)[(*face)[triIdx][ii[0]]]).cross(Pos-(*point)[(*face)[triIdx][ii[0]]])).norm();
		N=Ai/A;
		Index=(*face)[triIdx][i];
			
		for(int j=0;j<NeighborNodeOfSurfVertice[Index].size();j++)
		{	
			NodeIndex=NeighborNodeOfSurfVertice[Index][j];
			if(NodeIndex>NbPoint)
			{
				break;

			}
			m[NodeIndex*3]+=ShapeFuncValueAtSurfPoint[Index][j]*normal[0]*N;
			m[NodeIndex*3+1]+=ShapeFuncValueAtSurfPoint[Index][j]*normal[1]*N;
			m[NodeIndex*3+2]+=ShapeFuncValueAtSurfPoint[Index][j]*normal[2]*N;
		}

	}

	return m;

	delete [] m;
}

void Meshfree_GPU::returnMappingMatrix(int triIdx,Vec3d Pos,Vec3d normal,double* &m)
{
	int NbPoint=EFGObj->nbNode();

	std::vector<Vec3f>* point=SurfObj->point();
	std::vector<Vec3i>* face=SurfObj->face();

	double A;
	double Ai;
	double N;
	int ii[2];
	int Index;
	int NodeIndex;

	A=(((*point)[(*face)[triIdx][1]]-(*point)[(*face)[triIdx][0]]).cross((*point)[(*face)[triIdx][2]]-(*point)[(*face)[triIdx][0]])).norm();

	for(int i=0;i<3*NbPoint;i++)
	{
		m[i]=0;
	}

	for(int i=0;i<3;i++)
	{
		ii[0]=(i+1)%3;
		ii[1]=(i+2)%3;
		Ai=(((*point)[(*face)[triIdx][ii[1]]]-(*point)[(*face)[triIdx][ii[0]]]).cross(Pos-(*point)[(*face)[triIdx][ii[0]]])).norm();
		N=Ai/A;
		Index=(*face)[triIdx][i];

		for(int j=0;j<NeighborNodeOfSurfVertice[Index].size();j++)
		{	
			NodeIndex=NeighborNodeOfSurfVertice[Index][j];
			if(NodeIndex>NbPoint)
			{
				break;

			}
			m[NodeIndex*3]+=ShapeFuncValueAtSurfPoint[Index][j]*normal[0]*N;
			m[NodeIndex*3+1]+=ShapeFuncValueAtSurfPoint[Index][j]*normal[1]*N;
			m[NodeIndex*3+2]+=ShapeFuncValueAtSurfPoint[Index][j]*normal[2]*N;
		}

	}
}



void Meshfree_GPU::returnMappingMatrix(int triIdx,Vec3d Pos,Vec3d normal,std::vector<Vec2f> &Map)
{
	int NbPoint=EFGObj->nbNode();

	std::vector<Vec3f>* point=SurfObj->point();
	std::vector<Vec3i>* face=SurfObj->face();

	double A;
	double Ai;
	double N;
	int ii[2];
	int Index;
	int NodeIndex;

	Vec2f temp;

	A=(((*point)[(*face)[triIdx][1]]-(*point)[(*face)[triIdx][0]]).cross((*point)[(*face)[triIdx][2]]-(*point)[(*face)[triIdx][0]])).norm();



	for(int i=0;i<3;i++)
	{
		ii[0]=(i+1)%3;
		ii[1]=(i+2)%3;
		Ai=(((*point)[(*face)[triIdx][ii[1]]]-(*point)[(*face)[triIdx][ii[0]]]).cross(Pos-(*point)[(*face)[triIdx][ii[0]]])).norm();
		N=Ai/A;
		Index=(*face)[triIdx][i];
		if(abs(N)>0.01)
		{
			for(int j=0;j<NeighborNodeOfSurfVertice[Index].size();j++)
			{	
				NodeIndex=NeighborNodeOfSurfVertice[Index][j];

				for(int k=0;k<3;k++)
				{
					temp[0]=NodeIndex*3+k;
					temp[1]=ShapeFuncValueAtSurfPoint[Index][j]*normal[k]*N;
					if(abs(temp[1])>0.0000001)
					{
						Map.push_back(temp);
					}
				}
			}
		}

	}
}

std::vector<Vec2d> Meshfree_GPU::returnVectorMappingMatrix(int triIdx,Vec3d Pos,Vec3d normal)
{
	int NbPoint=EFGObj->nbNode();

	
	std::vector<Vec3f>* point=SurfObj->point();
	std::vector<Vec3i>* face=SurfObj->face();

	std::vector<Vec2d> mapping;
	Vec2d temp;

	double A;
	double Ai;
	double N;
	int ii[2];
	int Index;
	int NodeIndex;

	A=(((*point)[(*face)[triIdx][1]]-(*point)[(*face)[triIdx][0]]).cross((*point)[(*face)[triIdx][2]]-(*point)[(*face)[triIdx][0]])).norm();

	for(unsigned int i=0;i<3;i++)
	{
		ii[0]=(i+1)%3;
		ii[1]=(i+2)%3;
		Ai=(((*point)[(*face)[triIdx][ii[1]]]-(*point)[(*face)[triIdx][ii[0]]]).cross(Pos-(*point)[(*face)[triIdx][ii[0]]])).norm();
		N=Ai/A;
		Index=(*face)[triIdx][i];

		for(unsigned int j=0;j<NeighborNodeOfSurfVertice[Index].size();j++)
		{	
			NodeIndex=NeighborNodeOfSurfVertice[Index][j];
			if(NodeIndex>NbPoint)
			{
				break;
			}
			for(int k=0;k<3;k++)
			{
				temp[0]=NodeIndex*3+k;
				temp[1]=ShapeFuncValueAtSurfPoint[Index][j]*normal[k]*N;
				if(abs(temp[1])>0.00001){
					mapping.push_back(temp);
				}
			}			
		}
	}

	return mapping;


}

void Meshfree_GPU::returnVectorMappingMatrix(int triIdx,Vec3d Pos,Vec3d normal,std::vector<Vec2d> &mapping)
{
	int NbPoint=EFGObj->nbNode();


	std::vector<Vec3f>* point=SurfObj->point();
	std::vector<Vec3i>* face=SurfObj->face();

	Vec2d temp;

	double A;
	double Ai;
	double N;
	int ii[2];
	int Index;
	int NodeIndex;



	A=(((*point)[(*face)[triIdx][1]]-(*point)[(*face)[triIdx][0]]).cross((*point)[(*face)[triIdx][2]]-(*point)[(*face)[triIdx][0]])).norm();

	for(unsigned int i=0;i<3;i++)
	{
		ii[0]=(i+1)%3;
		ii[1]=(i+2)%3;
		Ai=(((*point)[(*face)[triIdx][ii[1]]]-(*point)[(*face)[triIdx][ii[0]]]).cross(Pos-(*point)[(*face)[triIdx][ii[0]]])).norm();
		N=Ai/A;
		Index=(*face)[triIdx][i];

		for(unsigned int j=0;j<NeighborNodeOfSurfVertice[Index].size();j++)
		{	
			NodeIndex=NeighborNodeOfSurfVertice[Index][j];
			if(NodeIndex>NbPoint)
			{
				break;
			}
			for(int k=0;k<3;k++)
			{
				temp[0]=NodeIndex*3+k;
				temp[1]=ShapeFuncValueAtSurfPoint[Index][j]*normal[k]*N;
				if(abs(temp[1])>0.00001){
					mapping.push_back(temp);
				}

			}			
		}
	}

}

void Meshfree_GPU::updateShapeFunction(std::vector<int>& pointIdx)
{
	MLSShapeFunc func;
	std::vector<Vec3f>* point=SurfObj->point();
	std::vector<Vec3f>* node=EFGObj->nodePos0Vec();
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

void Meshfree_GPU::removeEdges(std::vector<int>& idx)
{
	VectorFunc func;
	for(int i=idx.size()-1;i>=0;i--)
	{
		// update neighbor information
		Vec2i edge=Edge[idx[i]];
		for(int k=0;k<NeighborNodeOfSurfVertice[edge[0]].size();k++)
		{
			if(NeighborNodeOfSurfVertice[edge[0]][k]==edge[1])
			{
				func.removeElement(NeighborNodeOfSurfVertice[edge[0]], k);
				break;
			}
		}
	}

	for(int i=idx.size()-1;i>=0;i--)
	{
		int edgeIdxEnd=Edge.size()-1;

		// 1. Update BVH
		AABBNode* node=BVHAABB.findLeafNode(idx[i]);
		if(node)
			BVHAABB.removeNode(node);
		node=BVHAABB.findLeafNode(edgeIdxEnd);
		if(node)
			node->IndexInLeafNode=idx[i];

		// 2. Remove edge
		Edge[idx[i]]=Edge[edgeIdxEnd];
		Edge.pop_back();
	}
}

void Meshfree_GPU::addForceToSurfaceTri(int triIdx,Vec3d Pos,Vec3f force)
{
	int NbPoint=EFGObj->nbNode();

	std::vector<Vec3f>* point=SurfObj->point();
	std::vector<Vec3i>* face=SurfObj->face();

	double A;
	double Ai;
	double N;
	int ii[2];
	int Index;
	int NodeIndex;

	A=(((*point)[(*face)[triIdx][1]]-(*point)[(*face)[triIdx][0]]).cross((*point)[(*face)[triIdx][2]]-(*point)[(*face)[triIdx][0]])).norm();

	for(int i=0;i<3;i++)
	{
		ii[0]=(i+1)%3;
		ii[1]=(i+2)%3;
		Ai=(((*point)[(*face)[triIdx][ii[1]]]-(*point)[(*face)[triIdx][ii[0]]]).cross(Pos-(*point)[(*face)[triIdx][ii[0]]])).norm();
		N=Ai/A;
		Index=(*face)[triIdx][i];

		addForceToSurfacePoint(Index,force*N);
	}


}

void Meshfree_GPU::addForceToSurfacePoint(int Index,Vec3f force)
{
	int NodeIndex;
	double shapeValue;
		
	for(int j=0;j<NeighborNodeOfSurfVertice[Index].size();j++)
	{			
		NodeIndex=NeighborNodeOfSurfVertice[Index][j];	
		shapeValue=ShapeFuncValueAtSurfPoint[Index][j];
		efgObj()->nodeForce()[NodeIndex*3]+=shapeValue*force[0];
		efgObj()->nodeForce()[NodeIndex*3+1]+=shapeValue*force[1];
		efgObj()->nodeForce()[NodeIndex*3+2]+=shapeValue*force[2];
	}

}

