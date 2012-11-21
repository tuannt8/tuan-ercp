#include "stdafx.h"
#include "MeshfreeCuttingManager.h"
#include "eSurfaceCutting.h"

MeshfreeCuttingManager::MeshfreeCuttingManager(void)
{
	AddedPoint=NULL;
	AddedPointNormal=NULL;
	F=fopen("MeshfreeCuttingManager.txt","w");
}

MeshfreeCuttingManager::~MeshfreeCuttingManager(void)
{
	fclose(F);
}

void MeshfreeCuttingManager::cutting(Meshfree_CPU* obj, std::vector<Vec3f>* cutPoint, std::vector<Vec3i>* cutFace, std::vector<Vec3f>* cutFaceNormal,  std::vector<int>* pointsOnCutFront)
{
	Obj_CPU=obj;
	CutPoint=cutPoint;
	CutSurf=cutFace;

	EFGCuttingManager efgCuttingManager;
	SurfaceCuttingManager surfCuttingManager;
	efgCuttingManager.cutting(obj->efgObj(), cutPoint, cutFace);
	surfCuttingManager.cutting(obj->surfObj(), cutPoint, cutFace, cutFaceNormal, pointsOnCutFront);

	AddedPoint=surfCuttingManager.addedPoint();
	AddedPointNormal=surfCuttingManager.addedPointNormal();
	PreNbPoint=surfCuttingManager.preNbPoint();
	VertexOnCutFront=surfCuttingManager.vertexIdxOnCutFront();

	updateConnection_CPU();
}

void MeshfreeCuttingManager::cuttingProg(Meshfree_GPU* obj, CuttingTool* tool, int toolPathIdx)
{
	if(toolPathIdx==0)
	{
		Obj_GPU=obj;
		Tool=tool;
		CutPoint=tool->pointTotal();
		CutSurf=tool->faceTotal();
		CutFaceNormal=tool->normalTotal();
		std::vector<int> pointsOnCutFront;
		tool->getCutFrontIdx(pointsOnCutFront);

		SurfCutting.cutting(obj->surfObj(), tool, &pointsOnCutFront);

		AddedPoint=SurfCutting.addedPoint();
		AddedPointNormal=SurfCutting.addedPointNormal();
		PreNbPoint=SurfCutting.preNbPoint();
		VertexOnCutFront=SurfCutting.vertexIdxOnCutFront();
		return;
	}
	TimeTick.SetStart();
	EFGCuttingManager efgCuttingManager;
	efgCuttingManager.cutting(obj->efgObj(), tool->cutPoint(), tool->cutFace());
	TimeTick.SetEnd();
	fprintf(F,"EFGCutting: %f\n",TimeTick.GetTick());

	TimeTick.SetStart();
	updateConnection_GPUProg(toolPathIdx);
	TimeTick.SetEnd();
	fprintf(F,"UpdateConnection: %f\n",TimeTick.GetTick());
}

void MeshfreeCuttingManager::cutting(Meshfree_GPU* obj, CuttingTool* tool)
{
	Obj_GPU=obj;
	Tool=tool;
	CutPoint=tool->pointTotal();
	CutSurf=tool->faceTotal();
	CutFaceNormal=tool->normalTotal();
	std::vector<int> pointsOnCutFront;
	tool->getCutFrontIdx(pointsOnCutFront);

	TimeTick.SetStart();
	EFGCuttingManager efgCuttingManager;
	SurfaceCuttingManager surfCuttingManager;
	efgCuttingManager.cutting(obj->efgObj(), CutPoint, CutSurf);
	TimeTick.SetEnd();
	fprintf(F,"EFGCutting: %f\n",TimeTick.GetTick());

	TimeTick.SetStart();
	surfCuttingManager.cutting(obj->surfObj(), tool, &pointsOnCutFront);
	TimeTick.SetEnd();
	fprintf(F,"SurfCutting: %f\n",TimeTick.GetTick());

	AddedPoint=surfCuttingManager.addedPoint();
	AddedPointNormal=surfCuttingManager.addedPointNormal();
	PreNbPoint=surfCuttingManager.preNbPoint();
	VertexOnCutFront=surfCuttingManager.vertexIdxOnCutFront();

	TimeTick.SetStart();
	updateConnection_GPU();
	TimeTick.SetEnd();
	fprintf(F,"UpdateConnection: %f\n",TimeTick.GetTick());
}

void MeshfreeCuttingManager::updateConnection_GPUProg(int toolPathIdx)
{
	VectorFunc func;
	GeometricFunc geoFunc;

	// Surface object data
	SurfaceObj* surfObj=Obj_GPU->surfObj();
	std::vector<Vec3f>* surfNode=surfObj->point();
	std::vector<Vec3i>* surfFace=surfObj->face();
	float supportRadius=Obj_GPU->supportRadiusSurf();

	// Meshfree object data
	std::vector<Vec3f>* efgNode=Obj_GPU->efgObj()->nodePosVec();

	// Meshfree object data
	std::vector<std::vector<int>>* neighborNodeIdx=Obj_GPU->neighborNodeOfSurfVertice();
	std::vector<int> updatedPointIdx;
	neighborNodeIdx->resize(surfNode->size());

	// 4. Collision detection
	CollisionManager collision;
	std::vector<int> colEdgeIdx;

	// 4-1. Collision detection between cut surface and edge of the EFG
	std::vector<Vec2i>* edge=Obj_GPU->edge();
	collision.collisionBtwTriAndEdgesWithBVHDiff(Tool->cutPoint(), Tool->cutFace(), surfNode, efgNode, edge, Obj_GPU->getBVH(), colEdgeIdx);

	for(int i=0;i<colEdgeIdx.size();i++)
		updatedPointIdx.push_back((*edge)[colEdgeIdx[i]][0]);

	// 4-2. Remove intersected edges
	func.arrangeVector(colEdgeIdx);
	Obj_GPU->removeEdges(colEdgeIdx);

	int nb=AddedPoint->size()/Tool->nbToolPath();
	int begin=nb*(toolPathIdx-1);
	int end=begin+nb;
	if(toolPathIdx==Tool->nbToolPath())
		end=AddedPoint->size();

	// Added point들의 neighbor information을 update한다
	for(int i=begin;i<end;i++)
	{
		int pointIdx=i+PreNbPoint;
		updatedPointIdx.push_back(pointIdx);
		Vec3f n=(*AddedPointNormal)[i];

		Vec3f l1=(*surfNode)[pointIdx]+n;

		std::vector<int> colNodeIdx;
		collision.PQBtwBoxAndPoint(Obj_GPU->efgObj()->getBVHPoint()->root(), efgNode, l1, colNodeIdx, supportRadius);
		VectorFunc func;
		func.arrangeVector(colNodeIdx);

		for(int j=0;j<colNodeIdx.size();j++)
		{
			Vec3f l2=(*efgNode)[colNodeIdx[j]];
			Vec3f inter;
			if(!collision.collisionBtwSurfAndLineSeg(CutPoint, CutSurf, Tool->getBVH()->root(), l1, l2))
				(*neighborNodeIdx)[pointIdx].push_back(colNodeIdx[j]);
		}
	}

	// Update shape function at surface point
	Obj_GPU->updateShapeFunction(updatedPointIdx);
	surfObj->pointNormal()->resize(surfNode->size());
	surfObj->faceNormal()->resize(surfFace->size());
	surfObj->updateNormal();
	surfObj->dis()->resize(surfNode->size());
}

void MeshfreeCuttingManager::updateConnection_GPUProg()
{
}

void  MeshfreeCuttingManager::updateConnection_GPU()
{
	VectorFunc func;
	GeometricFunc geoFunc;

	// Surface object data
	SurfaceObj* surfObj=Obj_GPU->surfObj();
	std::vector<Vec3f>* surfNode=surfObj->point();
	std::vector<Vec3i>* surfFace=surfObj->face();
	float supportRadius=Obj_GPU->supportRadiusSurf();

	// Meshfree object data
	std::vector<Vec3f>* efgNode=Obj_GPU->efgObj()->nodePosVec();

	// Meshfree object data
	std::vector<std::vector<int>>* neighborNodeIdx=Obj_GPU->neighborNodeOfSurfVertice();
	std::vector<int> updatedPointIdx;
	neighborNodeIdx->resize(surfNode->size());

	// 4. Collision detection
	CollisionManager collision;
	std::vector<int> colEdgeIdx;

	// 4-1. Collision detection between cut surface and edge of the EFG
	std::vector<Vec2i>* edge=Obj_GPU->edge();
	collision.collisionBtwTriAndEdgesWithBVHDiff(CutPoint, CutSurf, surfNode, efgNode, edge, Obj_GPU->getBVH(), colEdgeIdx);

	for(int i=0;i<colEdgeIdx.size();i++)
		updatedPointIdx.push_back((*edge)[colEdgeIdx[i]][0]);

	// 4-2. Remove intersected edges
	func.arrangeVector(colEdgeIdx);
	Obj_GPU->removeEdges(colEdgeIdx);

	// Added point들의 neighbor information을 update한다
	for(int i=0;i<AddedPoint->size();i++)
	{
		int pointIdx=i+PreNbPoint;
		updatedPointIdx.push_back(pointIdx);
		Vec3f n=(*AddedPointNormal)[i];

		Vec3f l1=(*surfNode)[pointIdx]+n;

		std::vector<int> colNodeIdx;
		collision.PQBtwBoxAndPoint(Obj_GPU->efgObj()->getBVHPoint()->root(), efgNode, l1, colNodeIdx, supportRadius);
		VectorFunc func;
		func.arrangeVector(colNodeIdx);
		
		for(int j=0;j<colNodeIdx.size();j++)
		{
			Vec3f l2=(*efgNode)[colNodeIdx[j]];
			Vec3f inter;
			if(!collision.collisionBtwSurfAndLineSeg(CutPoint, CutSurf, Tool->getBVH()->root(), l1, l2))
				(*neighborNodeIdx)[pointIdx].push_back(colNodeIdx[j]);

			/*bool flag1=true;
			for(int k=0;k<CutSurf->size();k++)
			{
				Vec3f tri[3];
				tri[0]=(*CutPoint)[(*CutSurf)[k][0]];
				tri[1]=(*CutPoint)[(*CutSurf)[k][1]];
				tri[2]=(*CutPoint)[(*CutSurf)[k][2]];
				if(geoFunc.collisionBtwLinesegAndTri(l1, l2, tri, inter))
				{
					flag1=false;
					break;
				}
			}
			if(flag1)
			{
				(*neighborNodeIdx)[pointIdx].push_back(colNodeIdx[j]);
			}*/
		}
	}
	
	// Update shape function at surface point
	Obj_GPU->updateShapeFunction(updatedPointIdx);
	surfObj->pointNormal()->resize(surfNode->size());
	surfObj->faceNormal()->resize(surfFace->size());
	surfObj->updateNormal();
	surfObj->dis()->resize(surfNode->size());
}

void  MeshfreeCuttingManager::updateConnection_CPU()
{
	VectorFunc func;
	GeometricFunc geoFunc;

	// Surface object data
	SurfaceObj* surfObj=Obj_CPU->surfObj();
	std::vector<Vec3f>* surfNode=surfObj->point();
	std::vector<Vec3i>* surfFace=surfObj->face();
	float supportRadius=Obj_CPU->supportRadiusSurf();

	// Meshfree object data
	std::vector<Vec3f>* efgNode=Obj_CPU->efgObj()->nodePos();

	// Meshfree object data
	std::vector<std::vector<int>>* neighborNodeIdx=Obj_CPU->neighborNodeOfSurfVertice();
	std::vector<int> updatedPointIdx;
	neighborNodeIdx->resize(surfNode->size());

	// Cutting에 의해 영향을 받는 point들을 찾는다.
	for(int i=0;i<PreNbPoint;i++)
	{
		for(int j=0;j<AddedPoint->size();j+=2)
		{
			if(((*AddedPoint)[j]-(*surfNode)[i]).norm()<supportRadius)
			{
				updatedPointIdx.push_back(i);
				break;
			}
		}
	}
	func.arrangeVector(updatedPointIdx);

	// Cutting에 의해 영향을 받는 point들의 neighbor information을 update한다
	for(int i=0;i<updatedPointIdx.size();i++)
	{
		int pointIdx=updatedPointIdx[i];
		for(int j=0;j<(*neighborNodeIdx)[pointIdx].size();j++)
		{
			int nodeIdx=(*neighborNodeIdx)[pointIdx][j];
			Vec3f l1=(*surfNode)[pointIdx];
			Vec3f l2=(*efgNode)[nodeIdx];
			Vec3f inter;

			for(int k=0;k<CutSurf->size();k++)
			{
				Vec3f tri[3];
				tri[0]=(*CutPoint)[(*CutSurf)[k][0]];
				tri[1]=(*CutPoint)[(*CutSurf)[k][1]];
				tri[2]=(*CutPoint)[(*CutSurf)[k][2]];
				if(geoFunc.collisionBtwLinesegAndTri(l1, l2, tri, inter))
				{
					func.removeElement((*neighborNodeIdx)[pointIdx], j);
					j--;
					break;
				}
			}
		}
	}

	// Added point들의 neighbor information을 update한다
	for(int i=0;i<AddedPoint->size();i++)
	{
		int pointIdx=i+PreNbPoint;
		updatedPointIdx.push_back(pointIdx);
		Vec3f n=(*AddedPointNormal)[i];

		bool flag=false;
		for(int j=0;j<VertexOnCutFront->size();j++)
		{
			if(pointIdx==(*VertexOnCutFront)[j])
			{
				flag=true;
				break;
			}
			if(pointIdx==((*VertexOnCutFront)[j]+1))
			{
				flag=true;
				break;
			}
		}
		if(flag)
		{
			for(int j=0;j<efgNode->size();j++)
			{
				Vec3f v=(*efgNode)[j]-(*surfNode)[pointIdx];
				if(v.norm()<supportRadius)
					(*neighborNodeIdx)[pointIdx].push_back(j);
			}
		}
		else
		{
			Vec3f l1=(*surfNode)[pointIdx]+n;
			for(int j=0;j<efgNode->size();j++)
			{
				Vec3f l2=(*efgNode)[j];
				Vec3f inter;

				if((l1-l2).norm()<supportRadius)
				{
					bool flag1=true;
					for(int k=0;k<CutSurf->size();k++)
					{
						Vec3f tri[3];
						tri[0]=(*CutPoint)[(*CutSurf)[k][0]];
						tri[1]=(*CutPoint)[(*CutSurf)[k][1]];
						tri[2]=(*CutPoint)[(*CutSurf)[k][2]];
						if(geoFunc.collisionBtwLinesegAndTri(l1, l2, tri, inter))
						{
							flag1=false;
							break;
						}
					}
					if(flag1)
						(*neighborNodeIdx)[pointIdx].push_back(j);
				}
			}
		}
	}

	// Update shape function at surface point
	Obj_CPU->updateShapeFunction(updatedPointIdx);
	surfObj->pointNormal()->resize(surfNode->size());
	surfObj->faceNormal()->resize(surfFace->size());
	surfObj->updateNormal();
	surfObj->dis()->resize(surfNode->size());
}

void MeshfreeCuttingManager::cylinderCutting( Meshfree_GPU* obj, std::vector<Vec3f>* toolPoint_in, float radius )
{
	Obj_GPU=obj;
	toolPoint = toolPoint_in;
	toolRadius = radius;


	EFGCuttingManager efgCuttingManager;
	eSurfaceCutting surCutting;

	surCutting.cutting(obj->surfObj(), toolPoint_in);
	surCutting.stepDebug();

	//UPdate internal nodes
	efgCuttingManager.cylinderCut(obj->efgObj(), toolPoint, toolRadius);
	//Update surface nodes
 	updateConnectionCylinder();
}

void MeshfreeCuttingManager::updateConnectionCylinder()
{
	VectorFunc func;
	GeometricFunc geoFunc;

	// Surface object data
	SurfaceObj* surfObj=Obj_GPU->surfObj();
	std::vector<Vec3f>* surfNode=surfObj->point();
	std::vector<Vec3i>* surfFace=surfObj->face();
	float supportRadius=Obj_GPU->supportRadiusSurf();

	// Meshfree object data
	std::vector<Vec3f>* efgNode=Obj_GPU->efgObj()->nodePosVec();

	// Meshfree object data
	std::vector<std::vector<int>>* neighborNodeIdx=Obj_GPU->neighborNodeOfSurfVertice();
	std::vector<int> updatedPointIdx;
	neighborNodeIdx->resize(surfNode->size());

	// 4. Collision detection
	CollisionManager collision;
	std::vector<int> colEdgeIdx;

	// 4-1. Collision detection between cut surface and edge of the EFG
	std::vector<Vec2i>* edge=Obj_GPU->edge();
	collision.collisionBtwCylinderAndEfgSurfaceEdge(toolPoint, toolRadius, surfNode, efgNode, edge, Obj_GPU->getBVH(), colEdgeIdx);
	func.arrangeVector(colEdgeIdx);

	for(int i=0;i<colEdgeIdx.size();i++)
		updatedPointIdx.push_back((*edge)[colEdgeIdx[i]][0]);

	// 4-2. Remove intersected edges
	Obj_GPU->removeEdges(colEdgeIdx);

	// Update neighbor of surface (For added point only)
	// 	for (int i=0; i<pointIdxNeedUpdate.size(); i++)
	// 	{
	// 		int pointIdx=pointIdxNeedUpdate[i];
	// 		updatedPointIdx.push_back(pointIdx);
	// 
	// 		Vec3f l1 = (*surfNode)[pointIdx];
	// 
	// 		std::vector<int> colNodeIdx;
	// 		collision.PQBtwBoxAndPoint(Obj_GPU->efgObj()->getBVHPoint()->root(), efgNode, l1, colNodeIdx, supportRadius);
	// 		VectorFunc func;
	// 		func.arrangeVector(colNodeIdx);
	// 
	// 		for(int j=0;j<colNodeIdx.size();j++)
	// 		{
	// 			Vec3f l2=(*efgNode)[colNodeIdx[j]];
	// 			bool flag1=true;
	// 			flag1 = (GeometricFunc::distanceBtwLineAndLine(toolPoint->at(0), toolPoint->at(1), l1, l2) < toolRadius);
	// 
	// 			if(flag1)
	// 			{
	// 				(*neighborNodeIdx)[pointIdx].push_back(colNodeIdx[j]);
	// 			}
	// 		}
	// 	}

	func.arrangeVector(updatedPointIdx);

	// Update shape function at surface point
	Obj_GPU->updateShapeFunction(updatedPointIdx);
	surfObj->pointNormal()->resize(surfNode->size());
	surfObj->faceNormal()->resize(surfFace->size());
	surfObj->updateNormal();
	surfObj->dis()->resize(surfNode->size());
}
