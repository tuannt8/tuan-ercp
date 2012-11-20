#include "stdafx.h"
#include "CuttingManager.h"

CuttingManager::CuttingManager(void)
{
}

CuttingManager::~CuttingManager(void)
{
}

void CuttingManager::cutting(EFG_CUDA_RUNTIME* obj, CuttingTool* tool)
{
	// 1. Find the intersection point between edges of connected node and cut surface
	CollisionManager collision;
	VectorFunc func;

	std::vector<int> collisionEdgeIdx;
	std::vector<Vec3f> intersectionPoints;
	FILE* f=fopen("cutting1.txt","w");
	g_Time.SetStart();
	collision.collisionBtwMeshfreeEdgeAndRect(obj, tool, collisionEdgeIdx, intersectionPoints);
	//collision.collisionBtwMeshfreeEdgeAndRectBVH(obj,tool,collisionEdgeIdx,intersectionPoints);
	g_Time.SetEnd();
	fprintf(f,"collision=%f\n",g_Time.GetTick());
	obj->removeEdges(collisionEdgeIdx);
}

void CuttingManager::cuttingWithCell(EFG_CUDA_RUNTIME* obj, CuttingTool* tool)
{
	// 1. Find the intersection point between edges of connected node and cut surface
	CollisionManager collision;
	VectorFunc func;

	std::vector<MeshfreeCell*>* cell=obj->cell();
	std::vector<int> cellIdx;
	std::vector<std::vector<int>> edgeIdx;
	std::vector<int> cutEdgeIdx;
	std::vector<std::vector<Vec3f>> intersection;
	std::vector<Vec3f> intersectionPoint;

	FILE* f=fopen("cutting.txt","w");
	g_Time.SetStart();
	collision.collisionBtwMeshfreeEdgeAndRectBVHCell(obj, tool, cellIdx, edgeIdx, intersection);
	//collision.collisionBtwMeshfreeEdgeAndRect(obj, tool, cutEdgeIdx, intersectionPoint);
	g_Time.SetEnd();
	fprintf(f,"collision=%f\n",g_Time.GetTick());

	// 2. Remove edges in cell
	for(int i=0;i<cellIdx.size();i++)
	{
		(*cell)[cellIdx[i]]->removeEdges(edgeIdx[i]);
		for(int j=0;j<edgeIdx[i].size();j++)
			cutEdgeIdx.push_back(edgeIdx[i][j]);
	}
	func.arrangeVector(cutEdgeIdx);
	obj->removeEdges(cutEdgeIdx);
	fclose(f);
}