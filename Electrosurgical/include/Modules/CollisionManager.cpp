#include "stdafx.h"
#include "CollisionManager.h"

CollisionManager::CollisionManager(void)
{
}

CollisionManager::~CollisionManager(void)
{
}

void CollisionManager::collisionBtwRectAndEdges(Rect rect, std::vector<Vec3f>& pos, std::vector<Vec2i>& edge, std::vector<int>& colEdgeIdx, std::vector<Vec3f>& intersectionPoint)
{
	GeometricFunc func;
	for(int i=0;i<edge.size();i++)
	{
		Line line; line.l1=pos[edge[i][0]]; line.l2=pos[edge[i][1]];
		Vec3f intersection;
		if(func.collisionBtwLinesegAndRect(line, rect, intersection))
		{
			colEdgeIdx.push_back(i);
			intersectionPoint.push_back(intersection);
		}
	}
}

void CollisionManager::collisionBtwRectAndEdges(Rect rect, std::vector<Vec3f>* pos, std::vector<Vec2i>* edge, std::vector<int>& colEdgeIdx, std::vector<Vec3f>& intersectionPoint)
{
	GeometricFunc func;
	for(int i=0;i<edge->size();i++)
	{
		Line line; line.l1=(*pos)[(*edge)[i][0]]; line.l2=(*pos)[(*edge)[i][1]];
		Vec3f intersection;
		if(func.collisionBtwLinesegAndRect(line, rect, intersection))
		{
			colEdgeIdx.push_back(i);
			intersectionPoint.push_back(intersection);
		}
	}
}

void CollisionManager::collisionBtwTriAndEdges(std::vector<Vec3f>* posT, std::vector<Vec3i>* tri, std::vector<Vec3f>* pos, std::vector<Vec2i>* edge, std::vector<int>& colEdgeIdx, std::vector<Vec3f>& intersectionPoint)
{
	GeometricFunc func;
	for(int i=0;i<edge->size();i++)
	{
		for(int j=0;j<tri->size();j++)
		{
			Vec3f intersection;
			Vec3f _tri[3]; 
			_tri[0]=(*posT)[(*tri)[j][0]]; 
			_tri[1]=(*posT)[(*tri)[j][1]]; 
			_tri[2]=(*posT)[(*tri)[j][2]];
			if(func.collisionBtwLinesegAndTri((*pos)[(*edge)[i][0]], (*pos)[(*edge)[i][1]], _tri, intersection))
			{
				colEdgeIdx.push_back(i);
				intersectionPoint.push_back(intersection);
			}
		}
	}
}

void CollisionManager::collisionBtwTriAndEdgesWithBVH(std::vector<Vec3f>* posT, std::vector<Vec3i>* tri, std::vector<Vec3f>* pos, std::vector<Vec2i>* edge, AABBTree* BVH, std::vector<int>& colEdgeIdx, std::vector<Vec3f>& intersectionPoint)
{
	// 각 triangle에 대해서 collision 검사
	for(int i=0;i<tri->size();i++)
	{
		// 1-1. Bounding box of the triangle
		Box box;
		box.leftDown=Vec3f(MAX, MAX, MAX);
		box.rightUp=Vec3f(MIN, MIN, MIN);
		Vec3f _tri[3]; 
		_tri[0]=(*posT)[(*tri)[i][0]]; 
		_tri[1]=(*posT)[(*tri)[i][1]]; 
		_tri[2]=(*posT)[(*tri)[i][2]];
		for(int j=0;j<3;j++)
		{
			for(int k=0;k<3;k++)
			{
				if(box.leftDown[k]>_tri[j][k])
					box.leftDown[k]=_tri[j][k];
				if(box.rightUp[k]<_tri[j][k])
					box.rightUp[k]=_tri[j][k];
			}
		}
		box.center=(box.leftDown+box.rightUp)/2.0;

		//2. Culling using AABB
		std::vector<int> pCollisionEdgeIdx;
		collisionBtwAABBAndBox(BVH->root(), box, pCollisionEdgeIdx);

		//3. Intersection test
		GeometricFunc func;
		for(int j=0;j<pCollisionEdgeIdx.size();j++)
		{
			Vec3f intersection;
			int idx=pCollisionEdgeIdx[j];
			if(func.collisionBtwLinesegAndTri((*pos)[(*edge)[idx][0]], (*pos)[(*edge)[idx][1]], _tri, intersection))
			{
				colEdgeIdx.push_back(idx);
				intersectionPoint.push_back(intersection);
			}
		}
	}
}

void CollisionManager::collisionBtwTrisAndTrisWithBVH(std::vector<Vec3f>* pos1, std::vector<Vec3i>* tri1, std::vector<Vec3f>* pos2, std::vector<Vec3i>* tri2, AABBTree* BVH, std::vector<int>& colTriIdx1, std::vector<int>& colTriIdx2)
{
	// 각 triangle에 대해서 collision 검사
	for(int i=0;i<tri1->size();i++)
	{
		// 1-1. Bounding box of the triangle
		Box box;
		box.leftDown=Vec3f(MAX, MAX, MAX);
		box.rightUp=Vec3f(MIN, MIN, MIN);
		Vec3f _tri1[3]; 
		_tri1[0]=(*pos1)[(*tri1)[i][0]]; 
		_tri1[1]=(*pos1)[(*tri1)[i][1]]; 
		_tri1[2]=(*pos1)[(*tri1)[i][2]];
		for(int j=0;j<3;j++)
		{
			for(int k=0;k<3;k++)
			{
				if(box.leftDown[k]>_tri1[j][k])
					box.leftDown[k]=_tri1[j][k];
				if(box.rightUp[k]<_tri1[j][k])
					box.rightUp[k]=_tri1[j][k];
			}
		}
		box.center=(box.leftDown+box.rightUp)/2.0;

		//2. Culling using AABB
		std::vector<int> pCollisionTriIdx;
		collisionBtwAABBAndBox(BVH->root(), box, pCollisionTriIdx);

		//3. Intersection test
		GeometricFunc func;
		bool flag=false;
		for(int j=0;j<pCollisionTriIdx.size();j++)
		{
			Vec3f _tri2[3];
			_tri2[0]=(*pos2)[(*tri2)[pCollisionTriIdx[j]][0]];
			_tri2[1]=(*pos2)[(*tri2)[pCollisionTriIdx[j]][1]];
			_tri2[2]=(*pos2)[(*tri2)[pCollisionTriIdx[j]][2]];
			if(func.collisionBtwTri(_tri1, _tri2))
			{
				colTriIdx2.push_back(pCollisionTriIdx[j]);
				flag=true;
			}
		}
		if(flag)
			colTriIdx1.push_back(i);
	}
}

void CollisionManager::collisionBtwAABBAndBox(AABBNode* root, Box& box, std::vector<int>& collisionIdx)
{
	Box _box;
	_box.leftDown=root->LeftDown;
	_box.rightUp=root->RightUp;
	_box.center=(_box.leftDown+_box.rightUp)/2.0;

	GeometricFunc func;
	if(func.collisionBtwBox(_box, box))
	{
		if(root->End)
		{
			collisionIdx.push_back(root->IndexInLeafNode);
			return;
		}
		else
		{
			collisionBtwAABBAndBox(root->Left, box, collisionIdx);
			collisionBtwAABBAndBox(root->Right, box, collisionIdx);
		}
	}
	return;
}

bool CollisionManager::collisionBtwSurfAndLineSeg(std::vector<Vec3f>* point, std::vector<Vec3i>* face, AABBNode* root, Vec3f l1, Vec3f l2)
{
	GeometricFunc func;
	
	// 1. Culling using AABB
	std::vector<int> pCollisionTriIdx;
	collisionBtwAABBAndLineSeg(root, l1, l2, pCollisionTriIdx);

	// 2. Compute intersection 
	for(int i=0;i<pCollisionTriIdx.size();i++)
	{
		Vec3f tri[3];
		tri[0]=(*point)[(*face)[pCollisionTriIdx[i]][0]];
		tri[1]=(*point)[(*face)[pCollisionTriIdx[i]][1]];
		tri[2]=(*point)[(*face)[pCollisionTriIdx[i]][2]];
		Vec3f _intersection;

		if(func.collisionBtwLinesegAndTri(l1, l2, tri, _intersection))
			return true;
	}
	return false;
}

bool CollisionManager::collisionBtwSurfAndLineSeg(SurfaceObj* obj, Vec3f l1, Vec3f l2)
{
	GeometricFunc func;
	AABBNode* root=obj->getBVH()->root();
	std::vector<Vec3f>* point=obj->point();
	std::vector<Vec3i>* face=obj->face();

	// 1. Culling using AABB
	std::vector<int> pCollisionTriIdx;
	collisionBtwAABBAndLineSeg(root, l1, l2, pCollisionTriIdx);

	// 2. Compute intersection 
	for(int i=0;i<pCollisionTriIdx.size();i++)
	{
		Vec3f tri[3];
		tri[0]=(*point)[(*face)[pCollisionTriIdx[i]][0]];
		tri[1]=(*point)[(*face)[pCollisionTriIdx[i]][1]];
		tri[2]=(*point)[(*face)[pCollisionTriIdx[i]][2]];
		Vec3f _intersection;

		if(func.collisionBtwLinesegAndTri(l1, l2, tri, _intersection))
			return true;
	}
	return false;
}

bool CollisionManager::collisionBtwSurfAndAxisLine(SurfaceObj* obj, Vec3f pos, Vec3i direc, std::vector<Vec3f>& intersection)
{
	GeometricFunc func;
	AABBNode* root=obj->getBVH()->root();
	std::vector<Vec3f>* point=obj->point();
	std::vector<Vec3i>* face=obj->face();

	// 1. Culling using AABB
	std::vector<int> pCollisionTriIdx;
	collisionBtwAABBAndAxisLine(root, pos, direc, pCollisionTriIdx);

	// 2. Compute intersection 
	for(int i=0;i<pCollisionTriIdx.size();i++)
	{
		Tri tri;
		tri.p[0]=(*point)[(*face)[pCollisionTriIdx[i]][0]];
		tri.p[1]=(*point)[(*face)[pCollisionTriIdx[i]][1]];
		tri.p[2]=(*point)[(*face)[pCollisionTriIdx[i]][2]];
		tri.normal=func.computeNormal(tri.p);
		Vec3f _intersection;

		if(func.collisionBtwTriDirectionalLine(pos, direc, tri, _intersection))
		{
			intersection.push_back(_intersection);
		}
	}
	return true;
}

bool CollisionManager::isPointInSurfObj(SurfaceObj* obj, Vec3f pos)
{
	Vec3i direc(0,0,1);
	std::vector<Vec3f> intersection;
	collisionBtwSurfAndAxisLine(obj, pos, direc, intersection);
	if((intersection.size()%2)==0)
		return false;
	else
		return true;
}

bool CollisionManager::collisionBtwAABBAndLineSeg(AABBNode* root, Vec3f l1, Vec3f l2, std::vector<int>& pCollisionTriIdx)
{
	GeometricFunc func;

	Box box;
	box.leftDown=root->LeftDown;
	box.rightUp=root->RightUp;
	box.center=(box.leftDown+box.rightUp)/2.0;

	if(func.collisionBtwBoxAndLineSeg(box, l1, l2))
	{
		if(root->End)
		{
			pCollisionTriIdx.push_back(root->IndexInLeafNode);
			return true;
		}
		else
		{
			collisionBtwAABBAndLineSeg(root->Left, l1, l2, pCollisionTriIdx);
			collisionBtwAABBAndLineSeg(root->Right, l1, l2, pCollisionTriIdx);
		}
	}
	return true;
}

bool CollisionManager::collisionBtwAABBAndAxisLine(AABBNode* root, Vec3f pos, Vec3i direc, std::vector<int>& pCollisionTriIdx)
{
	GeometricFunc func;

	Box box;
	box.leftDown=root->LeftDown;
	box.rightUp=root->RightUp;
	box.center=(box.leftDown+box.rightUp)/2.0;

	if(func.collisionBtwBoxAndAxisLine(box, pos, direc))
	{
		if(root->End)
		{
			pCollisionTriIdx.push_back(root->IndexInLeafNode);
			return true;
		}
		else
		{
			collisionBtwAABBAndAxisLine(root->Left, pos, direc, pCollisionTriIdx);
			collisionBtwAABBAndAxisLine(root->Right, pos, direc, pCollisionTriIdx);
		}
	}
	return true;
}

void CollisionManager::PQBtwBoxAndPoint(AABBNode* root, std::vector<Vec3f>* nodePos, Vec3f point, std::vector<int>& pointIdx, float dis)
{
	// 1. traverse bvh
	std::vector<int> pNodeIdx;
	traverseBVHPQ(root, point, pNodeIdx, dis);

	for(int i=0;i<pNodeIdx.size();i++)
	{
		if(((*nodePos)[pNodeIdx[i]]-point).norm()<dis)
			pointIdx.push_back(pNodeIdx[i]);
	}
}

void CollisionManager::PQBtwSurfaceObj(SurfaceObj* obj, SurfaceObj* environment, std::vector<int>& pointIdx, std::vector<Vec3f>& collisionNormal, std::vector<float>& delta, float dis)
{
	AABBNode* root1=obj->getBVH()->root();
	AABBNode* root2=environment->getBVH()->root();
	std::vector<Vec3f>* point1=obj->point();
	std::vector<Vec3f>* point2=environment->point();
	std::vector<Vec3i>* face1=obj->face();
	std::vector<Vec3i>* face2=environment->face();
	std::vector<Vec3f>* normal2=environment->faceNormal();
	VectorFunc vecFunc;
	GeometricFunc geoFunc;

	// 1. traverse bvh
	std::vector<Vec2i> pTriPair;
	traverseBVHPQ(root1, root2, pTriPair, dis);

	// 2. collision 될 가능성이 있는 point들
	std::vector<int> pCollisionPointIdx;
	std::vector<int> pCollisionTriIdx;
	for(int i=0;i<pTriPair.size();i++)
	{
		for(int j=0;j<3;j++)
			pCollisionPointIdx.push_back((*face1)[pTriPair[i][0]][j]);
		pCollisionTriIdx.push_back(pTriPair[i][1]);
	}
	vecFunc.arrangeVector(pCollisionPointIdx);
	vecFunc.arrangeVector(pCollisionTriIdx);

	// 2. exact distance computation
	for(int i=0;i<pCollisionPointIdx.size();i++)
	{
		float minDis=10000000;
		Vec3f p=(*point1)[pCollisionPointIdx[i]];
		Vec3f colPoint;
		Vec3f colNormal;
		for(int j=0;j<pCollisionTriIdx.size();j++)
		{
			Vec3f tri[3]; 
			Vec3f _colPoint;
			tri[0]=(*point2)[(*face2)[pCollisionTriIdx[j]][0]];
			tri[1]=(*point2)[(*face2)[pCollisionTriIdx[j]][1]];
			tri[2]=(*point2)[(*face2)[pCollisionTriIdx[j]][2]];
			float _dis=geoFunc.disBtwPointAndTri(p, tri, _colPoint);
			if(minDis>_dis)
			{
				minDis=_dis;
				colPoint=_colPoint;
				colNormal=(*normal2)[pCollisionTriIdx[j]];
			}
		}
		if(minDis<dis)
		{
			pointIdx.push_back(pCollisionPointIdx[i]);
			collisionNormal.push_back(colNormal);
			delta.push_back(minDis-dis);
		}
	}
}

void CollisionManager::traverseBVHPQ(AABBNode* root, Vec3f point, std::vector<int>& pNodeIdx, float dis)
{
	// 1. distance computation between bounding boxes 
	GeometricFunc func;
	Box box; box.leftDown=root->LeftDown; box.rightUp=root->RightUp; box.center=(box.leftDown+box.rightUp)/2.0;

	if(func.disBtwPointAndBox(box, point)<dis)
	{
		if(root->End)
		{
			std::vector<int>* idx=root->indicesInLeafNode();
			for(int i=0;i<idx->size();i++)
				pNodeIdx.push_back((*idx)[i]);
			return;
		}
		else
		{
			traverseBVHPQ(root->Left, point, pNodeIdx, dis);
			traverseBVHPQ(root->Right, point, pNodeIdx, dis);
		}
	}
}

void CollisionManager::traverseBVHPQ(AABBNode* root1, AABBNode* root2, std::vector<Vec2i>& pTriPair, float dis)
{
	// 1. distance computation between bounding boxes 
	GeometricFunc func;
	Box box1; box1.leftDown=root1->LeftDown; box1.rightUp=root1->RightUp; box1.center=(box1.leftDown+box1.rightUp)/2.0;
	Box box2; box2.leftDown=root2->LeftDown; box2.rightUp=root2->RightUp; box2.center=(box2.leftDown+box2.rightUp)/2.0;

	if(func.disBtwBox(box1, box2)<dis)
	{
		if(root1->End)
		{
			if(root2->End)
			{
				pTriPair.push_back(Vec2i(root1->IndexInLeafNode, root2->IndexInLeafNode));
				return;
			}
			else
			{
				traverseBVHPQ(root1, root2->Left, pTriPair, dis);
				traverseBVHPQ(root1, root2->Right, pTriPair, dis);
			}
		}
		else
		{
			if(root2->End)
			{
				traverseBVHPQ(root1->Left, root2, pTriPair, dis);
				traverseBVHPQ(root1->Right, root2, pTriPair, dis);
			}
			else
			{
				traverseBVHPQ(root1->Left, root2->Left, pTriPair, dis);
				traverseBVHPQ(root1->Left, root2->Right, pTriPair, dis);
				traverseBVHPQ(root1->Right, root2->Left, pTriPair, dis);
				traverseBVHPQ(root1->Right, root2->Right, pTriPair, dis);
			}
		}
	}
}

void CollisionManager::collisionBtwTriAndEdgesWithBVHDiff(std::vector<Vec3f>* posT, std::vector<Vec3i>* tri, std::vector<Vec3f>* point, std::vector<Vec3f>* nodePos, std::vector<Vec2i>* edge, AABBTreeEdgeDiff* BVH, std::vector<int>& colEdgeIdx)
{
	// 각 triangle에 대해서 collision 검사
	for(int i=0;i<tri->size();i++)
	{
		// 1-1. Bounding box of the triangle
		Box box;
		box.leftDown=Vec3f(MAX, MAX, MAX);
		box.rightUp=Vec3f(MIN, MIN, MIN);
		Vec3f _tri[3]; 
		_tri[0]=(*posT)[(*tri)[i][0]]; 
		_tri[1]=(*posT)[(*tri)[i][1]]; 
		_tri[2]=(*posT)[(*tri)[i][2]];
		for(int j=0;j<3;j++)
		{
			for(int k=0;k<3;k++)
			{
				if(box.leftDown[k]>_tri[j][k])
					box.leftDown[k]=_tri[j][k];
				if(box.rightUp[k]<_tri[j][k])
					box.rightUp[k]=_tri[j][k];
			}
		}
		box.center=(box.leftDown+box.rightUp)/2.0;

		//2. Culling using AABB
		std::vector<int> pCollisionEdgeIdx;
		collisionBtwAABBAndBox(BVH->root(), box, pCollisionEdgeIdx);

		//3. Intersection test
		GeometricFunc func;
		for(int j=0;j<pCollisionEdgeIdx.size();j++)
		{
			Vec3f intersection;
			int idx=pCollisionEdgeIdx[j];
			if(func.collisionBtwLinesegAndTri((*point)[(*edge)[idx][0]], (*nodePos)[(*edge)[idx][1]], _tri, intersection))
				colEdgeIdx.push_back(idx);
		}
	}
}