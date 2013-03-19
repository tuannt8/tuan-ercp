#include "stdafx.h"
#include "EFGCuttingManager.h"

EFGCuttingManager::EFGCuttingManager(void)
{

}

EFGCuttingManager::~EFGCuttingManager(void)
{
}

void EFGCuttingManager::cutting(EFG_CPU* obj, std::vector<Vec3f>* cutPoint, std::vector<Vec3i>* cutSurf)
{
	VectorFunc func;
	GeometricFunc geoFunc;
	CollisionManager collision;

	std::vector<Vec3f>* efgNode=obj->nodePos();
	std::vector<Vec2i>* efgEdge=obj->edge();
	AABBTreeEdge* edfBVH=obj->getBVH();

	std::vector<int> colEdgeIdx;
	std::vector<Vec3f> intersectionPoint;

	// 1-1. Collision detection between cut surface and edge of the EFG
	collision.collisionBtwTriAndEdgesWithBVH(cutPoint, cutSurf, efgNode, efgEdge, edfBVH, colEdgeIdx, intersectionPoint);

	// 1-2. Remove intersected edges
	obj->removeEdges(colEdgeIdx);
}

void EFGCuttingManager::cutting(EFG_CUDA_RUNTIME* obj, std::vector<Vec3f>* cutPoint, std::vector<Vec3i>* cutSurf)
{
	VectorFunc func;
	GeometricFunc geoFunc;
	CollisionManager collision;

	std::vector<Vec3f>* efgNode=obj->nodePosVec();
	std::vector<Vec2i>* efgEdge=obj->edge();
	AABBTreeEdge* edfBVH=obj->getBVH();

	std::vector<int> colEdgeIdx;
	std::vector<Vec3f> intersectionPoint;

	// 1-1. Collision detection between cut surface and edge of the EFG
	collision.collisionBtwTriAndEdgesWithBVH(cutPoint, cutSurf, efgNode, efgEdge, edfBVH, colEdgeIdx, intersectionPoint);

	// 1-2. Remove intersected edges
	obj->removeEdges(colEdgeIdx);
}

void EFGCuttingManager::cylinderCut( EFG_CUDA_RUNTIME* obj, std::vector<Vec3f>* toolPoint, float radius  )
{
	VectorFunc func;
	GeometricFunc geoFunc;
	CollisionManager collision;

	std::vector<Vec3f>* efgNode=obj->nodePosVec();
	std::vector<Vec2i>* efgEdge=obj->edge();
	AABBTreeEdge* efgBVH=obj->getBVH();

	std::vector<int> colEdgeIdx;

//	arrayInt test;
//	collision.collisionBtwCylinderAndEdge(toolPoint, radius, efgNode, efgEdge, test);
	collision.collisionBtwCylinderAndEdgeWithBVH(toolPoint, radius, efgNode, efgEdge, efgBVH, colEdgeIdx);

	// 1-2. Remove intersected edges
	if (colEdgeIdx.size()>0)
	{
		obj->removeEdges(colEdgeIdx);

// 		releaseLog::logVectorInt(test, "realInt.txt");
// 		releaseLog::logVectorInt(colEdgeIdx, "WithBVH.txt");
	}

}
