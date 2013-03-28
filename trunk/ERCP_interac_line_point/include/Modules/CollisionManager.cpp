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

void CollisionManager::collisionBtwTriAndEdgesWithBVH(std::vector<Vec3f>* posT, std::vector<Vec3i>* tri, std::vector<Vec3f>* pos, std::vector<Vec2i>* edge, AABBTreeEdge* BVH, std::vector<int>& colEdgeIdx, std::vector<Vec3f>& intersectionPoint)
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

bool CollisionManager::collisionBtwSurfAndLineSeg_return_Index(SurfaceObj* obj, Vec3f l1, Vec3f l2, double radius)
{
	GeometricFunc func;
	AABBNode* root=obj->getBVH()->root();
	std::vector<Vec3f>* point=obj->point();
	std::vector<Vec3i>* face=obj->face();

	// 1. Culling using AABB
	std::vector<int> pCollisionTriIdx;
	collisionBtwAABBAndLineSeg(root, l1,l2, pCollisionTriIdx);
	Vec3d collidedCylPoint;
	Vec3d collidedTriPoint;
	Vec3d P1,P2,P3;
	int TriIndex;
	double length;


	distance.clear();
	Distancefield dis;

	for(int i=0;i<pCollisionTriIdx.size();i++)
	{
		TriIndex=pCollisionTriIdx[i];
		P1=(*point)[(*face)[TriIndex][0]];
		P2=(*point)[(*face)[TriIndex][1]];
		P3=(*point)[(*face)[TriIndex][2]];

		measureMinimumDistanceBetLineandTriangle(l1, l2,P1,P2,P3, collidedTriPoint, collidedCylPoint);
		length=(collidedCylPoint-collidedTriPoint).norm();
		if(length<radius)
		{

			Vec3d CollidedDirec=collidedTriPoint-collidedCylPoint;
			CollidedDirec.normalize();
			dis.triIdx=TriIndex;
			dis.measuredPointInCylinder=collidedCylPoint;
			dis.collidedCylPoint=collidedCylPoint+CollidedDirec*(radius);
			dis.collidedTriPoint=collidedTriPoint;
			dis.PenetratedDirec=-CollidedDirec;
			dis.Penetration=length-radius;

			distance.push_back(dis);


		}
	}

	if(distance.size())
	{
		return true;
	}else
	{
		return false;
	}

}

bool CollisionManager::collisionBtwSurfAndLineSeg_return_Index_V2(SurfaceObj* obj, Vec3f l1, Vec3f l2, double radius, double margin)
{
	GeometricFunc func;
	AABBNode* root=obj->getBVH()->root();
	std::vector<Vec3f>* point=obj->point();
	std::vector<Vec3i>* face=obj->face();

	// 1. Culling using AABB
	
	std::vector<int> pCollisionTriIdx;
	Box box;
	for(int i=0;i<3;i++)
	{
		if(l1[i]>=l2[i])
		{
			box.leftDown[i]=l2[i]-radius*margin;
			box.rightUp=l1[i]+radius*margin;
			box.center[i]=(l1[i]+l2[i])/2.0;
		}else
		{
			box.leftDown[i]=l1[i]-radius*margin;
			box.rightUp=l2[i]+radius*margin;
			box.center[i]=(l1[i]+l2[i])/2.0;
		}
	}
	collisionBtwAABBAndBox(root, box, pCollisionTriIdx);
	Vec3d collidedCylPoint;
	Vec3d collidedTriPoint;
	Vec3d P1,P2,P3;
	int TriIndex;
	double length;


	distance.clear();
	Distancefield dis;

	for(int i=0;i<pCollisionTriIdx.size();i++)
	{
		TriIndex=pCollisionTriIdx[i];
		P1=(*point)[(*face)[TriIndex][0]];
		P2=(*point)[(*face)[TriIndex][1]];
		P3=(*point)[(*face)[TriIndex][2]];

		measureMinimumDistanceBetLineandTriangle(l1, l2,P1,P2,P3, collidedTriPoint, collidedCylPoint);
		length=(collidedCylPoint-collidedTriPoint).norm();
		if(length<radius*margin)
		{

			Vec3d CollidedDirec=collidedTriPoint-collidedCylPoint;
			CollidedDirec.normalize();
			dis.triIdx=TriIndex;
			dis.measuredPointInCylinder=collidedCylPoint;
			dis.collidedCylPoint=collidedCylPoint+CollidedDirec*(radius);
			dis.collidedTriPoint=collidedTriPoint;
			dis.PenetratedDirec=-CollidedDirec;
			dis.Penetration=length-radius;

			distance.push_back(dis);


		}
	}

	if(distance.size())
	{
		return true;
	}else
	{
		return false;
	}
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

void CollisionManager::measureMinimumDistanceBetPointandTriangle(Vec3d p1, Vec3d tri1, Vec3d tri2, Vec3d tri3, Vec3d &collidedTri, Vec3d &collidedCyl)
{
	double Length;
	Vec3d tripoint[3];

	Vec3d collidedTriPoint;
	Vec3d collidedTriPlane;
	Vec3d measuredCylinder;
	tripoint[0]=tri1;
	tripoint[1]=tri2;
	tripoint[2]=tri3;

	double L;

	//Point and Line


	Length=1000000000000;


	//end point of line element and triangle

	collidedTriPlane=findClosestPointInTri(p1,tri1,tri2,tri3);
	if((p1-collidedTriPlane).norm()<Length)
	{
		Length=(p1-collidedTriPlane).norm();
		measuredCylinder=p1;
		collidedTriPoint=collidedTriPlane;
	}	

	//return

	collidedTri=collidedTriPoint;
	collidedCyl=measuredCylinder;

}


Vec3d CollisionManager::findClosestPointInTri(Vec3d point, Vec3d tri1, Vec3d tri2, Vec3d tri3)
{
	double Length=1000000000000000;

	Vec3d Normal=(tri2-tri1).cross(tri3-tri1);
	Normal.normalize();
	Vec3d Center=tri1;
	Vec3d ProjectedPoint=findProjectedPointToPlane(point,Normal,Center);
	Vec3d tri[3];
	int index=0;
	tri[0]=tri1;
	tri[1]=tri2;
	tri[2]=tri3;
	//점-면

	double flag1=((tri2-tri1).cross(ProjectedPoint-tri1))*((ProjectedPoint-tri1).cross(tri3-tri1));
	double flag2=((tri3-tri2).cross(ProjectedPoint-tri2))*((ProjectedPoint-tri2).cross(tri1-tri2));

	if(flag1>=0 && flag2>=0)
	{
		return ProjectedPoint;
	}
	else
	{
		for(int i=0;i<3;i++)
		{
			if((tri[i]-point).norm()<Length)
			{
				Length=(tri[i]-point).norm();
				index=i;
			}
		}

		return tri[index];

	}
}


Vec3d CollisionManager::findProjectedPointToPlane(Vec3d Point, Vec3d Normal, Vec3d Center)
{
	Normal.normalize();
	double d=(Point-Center)*Normal;

	Point=Point-Normal*d;

	return Point;
}



void CollisionManager::measureMinimumDistanceBetLineandTriangle(Vec3d p1, Vec3d p2,Vec3d tri1, Vec3d tri2, Vec3d tri3, Vec3d &collidedTri, Vec3d &collidedCyl)
{
	Vec3d direc=(p2-p1);
	direc.normalize();
	double Length;
	Vec3d tripoint[3];

	Vec3d collidedTriVertex;
	Vec3d collidedTriEdge;
	Vec3d collidedTriPlane;
	Vec3d collidedTriPoint;
	Vec3d collidedCylinder;
	Vec3d measuredCylinder;
	tripoint[0]=tri1;
	tripoint[1]=tri2;
	tripoint[2]=tri3;

	double L;

	//Point and Line


	Length=1000000000000;

	for(int i=0;i<3;i++)
	{
		collidedCylinder=p1+direc*((tripoint[i]-p1)*direc);
		if((collidedCylinder-p1)*(collidedCylinder-p2)<0)
		{
			L=(collidedCylinder-tripoint[i]).norm();
			if(L<Length){
				measuredCylinder=collidedCylinder;
				collidedTriPoint=tripoint[i];
				Length=L;
			}
		}
	}


	//Line and Line
	int ii[2];

	for(int i=0;i<3;i++)
	{

		ii[0]=i%3;
		ii[1]=(i+1)%3;
		collidedTriEdge=findPointBetweenTwoline1(tripoint[ii[0]],tripoint[ii[1]],p1,p2);
		collidedCylinder=p1+direc*((collidedTriEdge-p1)*direc);
		if((collidedTriEdge-tripoint[ii[0]])*(collidedTriEdge-tripoint[ii[1]])<0)
		{
			if((collidedCylinder-p1)*(collidedCylinder-p2)<0)
			{
				L=(collidedCylinder-collidedTriEdge).norm();
				if(L<Length){
					Length=L;
					measuredCylinder=collidedCylinder;
					collidedTriPoint=collidedTriEdge;
				}
			}
		}
	}

	//end point of line element and triangle

	collidedTriPlane=findClosestPointInTri(p1,tri1,tri2,tri3);
	if((p1-collidedTriPlane).norm()<Length)
	{
		Length=(p1-collidedTriPlane).norm();
		measuredCylinder=p1;
		collidedTriPoint=collidedTriPlane;
	}	

	collidedTriPlane=findClosestPointInTri(p2,tri1,tri2,tri3);

	if((p2-collidedTriPlane).norm()<Length)
	{
		Length=(p2-collidedTriPlane).norm();
		measuredCylinder=p2;
		collidedTriPoint=collidedTriPlane;
	}

	//return

	collidedTri=collidedTriPoint;
	collidedCyl=measuredCylinder;

}

Vec3d CollisionManager::findPointBetweenTwoline1(Vec3d p1, Vec3d p2, Vec3d p3, Vec3d p4)
{
	Vec3d direc=p4-p3; direc.normalize();
	Vec3d r0=p3;
	double ta,tb,da,db,t,s;
	Vec3d Qa,Qb,Pa,Pb,Pa2,P,P2;


	Pa=p1;	Pb=p2;		
	ta=projectOnLine(r0, direc, Pa);		
	tb=projectOnLine(r0, direc, Pb);
	Qa=r0+direc*ta;		Qb=r0+direc*tb;
	da=(Pa-Qa).norm();	db=(Pb-Qb).norm();
	Pa2=Pa+Qb-Qa;
	s=(Pa2-Pb).norm();
	t=(da*da-db*db+s*s)/(2*s*s);

	P=Pa+(Pb-Pa)*t;

	return P;	
}

double CollisionManager::projectOnLine(Vec3d r0, Vec3d direc, Vec3d P)
{
	return (P*direc-r0*direc);
}

void CollisionManager::drawCollisionInfo(Vec3d color)
{

	glColor3f(color[0],color[1],color[2]);
	Vec3d P1,P2;
	glLineWidth((GLfloat)3);
	glBegin(GL_LINES);
	for(int i=0;i<distance.size();i++)
	{
		P1=distance[i].collidedCylPoint;
		P2=distance[i].collidedTriPoint;
		glVertex3f((GLfloat)P1[0],(GLfloat)P1[1],(GLfloat)P1[2]);
		glVertex3f((GLfloat)P2[0],(GLfloat)P2[1],(GLfloat)P2[2]);
	}
	glEnd();

}

void CollisionManager::collisionBtwCylinderAndEdgeWithBVH( std::vector<Vec3f>* toolPoint, float radius, std::vector<Vec3f>* efgNode, std::vector<Vec2i>* efgEdge, AABBTreeEdge* efgBVH, std::vector<int>& colEdgeIdx )
{
	std::vector<int> pEdgeIdx;
	traverseEdgeBVHByLine(efgBVH->root(), toolPoint, radius, pEdgeIdx);

	GeometricFunc func;
	for (int i=0; i<pEdgeIdx.size(); i++)
	{
		Vec2i curE = efgEdge->at(pEdgeIdx[i]);
		float dist = func.distanceBtwLineAndLine((*toolPoint)[0], (*toolPoint)[1], 
			efgNode->at(curE[0]), efgNode->at(curE[1]));
		if (dist<radius)
		{
			colEdgeIdx.push_back(pEdgeIdx[i]);
		}
	}
	VectorFunc vecFunc;
	vecFunc.arrangeVector(colEdgeIdx);
}

void CollisionManager::traverseEdgeBVHByLine(AABBNode* root, std::vector<Vec3f>* toolPoint, float radius, std::vector<int>& colEdgeIdx )
{
	// 1. distance computation between bounding boxes 
	GeometricFunc func;
	Box box; box.leftDown=root->LeftDown; box.rightUp=root->RightUp; box.center=(box.leftDown+box.rightUp)/2.0;
	Vec3f center = box.center;

	float dis = radius + (root->RightUp - root->LeftDown).norm()/2;

	if(func.distanceBtwPointAndLine(center, (*toolPoint)[0], (*toolPoint)[1])<dis)
	{
		if(root->End)
		{
			colEdgeIdx.push_back(root->IndexInLeafNode);
			return;
		}
		else
		{
			traverseEdgeBVHByLine(root->Left, toolPoint, radius, colEdgeIdx);
			traverseEdgeBVHByLine(root->Right, toolPoint, radius, colEdgeIdx);
		}
	}
}

void CollisionManager::collisionBtwCylinderAndEdge( std::vector<Vec3f>* toolPoint, float radius, 
												   std::vector<Vec3f>* efgNode, std::vector<Vec2i>* efgEdge, std::vector<int>& colEdgeIdx )
{
	GeometricFunc func;
	for (int i=0; i<efgEdge->size(); i++)
	{
		Vec2i curE = efgEdge->at(i);
		float dist = func.distanceBtwLineAndLine((*toolPoint)[0], (*toolPoint)[1], 
			efgNode->at(curE[0]), efgNode->at(curE[1]));
		if (dist<radius)
		{
			colEdgeIdx.push_back(i);
		}
	}
}

void CollisionManager::collisionBtwCylinderAndEfgSurfaceEdge( std::vector<Vec3f>* toolPoint, float toolRadius, 
															 std::vector<Vec3f>* surfNode, std::vector<Vec3f>* efgNode, std::vector<Vec2i>* edge, 
															 AABBTreeEdgeDiff* BVH, std::vector<int>& colEdgeIdx )
{
 	GeometricFunc func;

// 	arrayInt test;
// 	for (int i=0; i<edge->size(); i++)
// 	{
// 		Vec2i curE = edge->at(i);
// 		float dist = func.distanceBtwLineAndLine(toolPoint->at(0), toolPoint->at(1),
// 			surfNode->at(curE[0]), efgNode->at(curE[1]));
// 
// 		if (dist < toolRadius)
// 		{
// 			test.push_back(i);
// 		}
// 	}

	std::vector<int> pCollided;
	traverseEdgeBVHByLine(BVH->root(), toolPoint, toolRadius, pCollided);

	for (int i=0; i<pCollided.size(); i++)
	{
		Vec2i curE = edge->at(pCollided[i]);
		float dist = func.distanceBtwLineAndLine(toolPoint->at(0), toolPoint->at(1),
							surfNode->at(curE[0]), efgNode->at(curE[1]));

		if (dist < toolRadius)
		{
			colEdgeIdx.push_back(pCollided[i]);
		}
	}

	VectorFunc vecFunc;
	vecFunc.arrangeVector(colEdgeIdx);
}

bool CollisionManager::collisionBtwSurfAndLineSeg_part( SurfaceObj* obj, Vec3f l1, Vec3f l2, double radius, double margin )
{
	GeometricFunc func;
	AABBNode* root=obj->getBVH()->root();
	std::vector<Vec3f>* point=obj->point();
	std::vector<Vec3i>* face=obj->face();

	// 1. Culling using AABB

	std::vector<int> pCollisionTriIdx;
	Box box;
	for(int i=0;i<3;i++)
	{
		if(l1[i]>=l2[i])
		{
			box.leftDown[i]=l2[i]-radius*margin;
			box.rightUp=l1[i]+radius*margin;
			box.center[i]=(l1[i]+l2[i])/2.0;
		}else
		{
			box.leftDown[i]=l1[i]-radius*margin;
			box.rightUp=l2[i]+radius*margin;
			box.center[i]=(l1[i]+l2[i])/2.0;
		}
	}
	collisionBtwAABBAndBox(root, box, pCollisionTriIdx);
	Vec3d collidedCylPoint;
	Vec3d collidedTriPoint;
	Vec3d P1,P2,P3;
	int TriIndex;
	double length;

	Distancefield dis;
	bool bCollisde = false;

	for(int i=0;i<pCollisionTriIdx.size();i++)
	{
		TriIndex=pCollisionTriIdx[i];
		P1=(*point)[(*face)[TriIndex][0]];
		P2=(*point)[(*face)[TriIndex][1]];
		P3=(*point)[(*face)[TriIndex][2]];

		measureMinimumDistanceBetLineandTriangle(l1, l2,P1,P2,P3, collidedTriPoint, collidedCylPoint);
		length=(collidedCylPoint-collidedTriPoint).norm();
		if(length<radius*margin)
		{

			Vec3d CollidedDirec=collidedTriPoint-collidedCylPoint;
			CollidedDirec.normalize();
			dis.triIdx=TriIndex;
			dis.measuredPointInCylinder=collidedCylPoint;
			dis.collidedCylPoint=collidedCylPoint+CollidedDirec*(radius);
			dis.collidedTriPoint=collidedTriPoint;
			dis.PenetratedDirec=-CollidedDirec;
			dis.Penetration=length-radius;

			distance.push_back(dis);
			
			bCollisde = true;

		}
	}

	return bCollisde;
}
