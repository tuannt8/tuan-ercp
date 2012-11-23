#ifndef COLLISION_MANAGER
#define COLLISION_MANAGER


#include "DataTypes/Define.h"
#include "DataTypes/Vec.h"
#include "DataTypes/Mat.h"
#include "DataTypes/GeometricFunc.h"
#include "DataTypes/VectorFunc.h"
#include "Modules/AABBEdge.h"
#include "Modules/AABBEdgeDiff.h"
#include "Graphics/SurfaceObj.h"
#include <afxwin.h>
#include <GL/glu.h>
#include <GL/gl.h>
#include <vector>
#include <map>

//#include "Modules/Meshfree_CPU.h"

//#include "Modules/Meshfree_GPU.h"


class CollisionManager
{
public:
	CollisionManager(void);
	~CollisionManager(void);

//functions
public:
	void collisionBtwRectAndEdges(Rect rect, std::vector<Vec3f>& pos, std::vector<Vec2i>& edge, std::vector<int>& colEdgeIdx, std::vector<Vec3f>& intersectionPoint);
	void collisionBtwRectAndEdges(Rect rect, std::vector<Vec3f>* pos, std::vector<Vec2i>* edge, std::vector<int>& colEdgeIdx, std::vector<Vec3f>& intersectionPoint);
	void collisionBtwTriAndEdges(std::vector<Vec3f>* posT, std::vector<Vec3i>* tri, std::vector<Vec3f>* pos, std::vector<Vec2i>* edge, std::vector<int>& colEdgeIdx, std::vector<Vec3f>& intersectionPoint);
	void collisionBtwTriAndEdgesWithBVH(std::vector<Vec3f>* posT, std::vector<Vec3i>* tri, std::vector<Vec3f>* pos, std::vector<Vec2i>* edge, AABBTree* BVH, std::vector<int>& colEdgeIdx, std::vector<Vec3f>& intersectionPoint);
	void collisionBtwTrisAndTrisWithBVH(std::vector<Vec3f>* pos1, std::vector<Vec3i>* tri1, std::vector<Vec3f>* pos2, std::vector<Vec3i>* tri2, AABBTree* BVH, std::vector<int>& colTriIdx1, std::vector<int>& colTriIdx2);
	bool collisionBtwSurfAndAxisLine(SurfaceObj* obj, Vec3f pos, Vec3i direc, std::vector<Vec3f>& intersection);
	bool collisionBtwSurfAndLineSeg(SurfaceObj* obj, Vec3f l1, Vec3f l2);//
	bool collisionBtwSurfAndLineSeg(std::vector<Vec3f>* point, std::vector<Vec3i>* face, AABBNode* root, Vec3f l1, Vec3f l2);
	
	bool collisionBtwAABBAndAxisLine(AABBNode* root, Vec3f pos, Vec3i direc, std::vector<int>& pCollisionTriIdx);
	bool collisionBtwAABBAndLineSeg(AABBNode* root, Vec3f l1, Vec3f l2, std::vector<int>& pCollisionTriIdx);
	bool isPointInSurfObj(SurfaceObj* obj, Vec3f pos);

	void PQBtwSurfaceObj(SurfaceObj* obj, SurfaceObj* environment, std::vector<int>& pointIdx, std::vector<Vec3f>& collisionPoint, std::vector<float>& delta, float dis);
	void PQBtwBoxAndPoint(AABBNode* root, std::vector<Vec3f>* nodePos, Vec3f point, std::vector<int>& pointIdx, float dis);
	void collisionBtwTriAndEdgesWithBVHDiff(std::vector<Vec3f>* posT, std::vector<Vec3i>* tri, std::vector<Vec3f>* point, std::vector<Vec3f>* nodePos, std::vector<Vec2i>* edge, AABBTreeEdgeDiff* BVH, std::vector<int>& colEdgeIdx);


	void collisionBtwCylinderAndEdge( std::vector<Vec3f>* toolPoint, float radius, std::vector<Vec3f>* efgNode, 
		std::vector<Vec2i>* efgEdge, std::vector<int>& colEdgeIdx );
	void collisionBtwCylinderAndEdgeWithBVH( std::vector<Vec3f>* toolPoint, float radius, std::vector<Vec3f>* efgNode, 
		std::vector<Vec2i>* efgEdge, AABBTreeEdge* efgBVH, std::vector<int>& colEdgeIdx );

	void collisionBtwCylinderAndEfgSurfaceEdge( std::vector<Vec3f>* toolPoint, float toolRadius, 
		std::vector<Vec3f>* surfNode, std::vector<Vec3f>* efgNode, std::vector<Vec2i>* edge, 
		AABBTreeEdgeDiff* BVH, std::vector<int>& colEdgeIdx );

	bool collisionBtwSurfAndLineSeg_return_Index(SurfaceObj* obj, Vec3f l1, Vec3f l2, double radius);//return Collided Tri Idx
	bool collisionBtwSurfAndLineSeg_return_Index_V2(SurfaceObj* obj, Vec3f l1, Vec3f l2, double radius, double margin);//return Collided Tri Idx

	//Multi part
	void clearCollisionInfo(){distance.clear();}
	bool collisionBtwSurfAndLineSeg_part(SurfaceObj* obj, Vec3f l1, Vec3f l2, double radius, double margin);//return Collided Tri Idx

	typedef struct  
	{
		int triIdx;
		Vec3d measuredPointInCylinder;
		Vec3d collidedCylPoint;
		Vec3d collidedTriPoint;
		Vec3d PenetratedDirec;
		double Penetration;
	} Distancefield;

	std::vector<Distancefield> distance;
	std::vector<Distancefield> getDistance(){return distance;};
	std::vector<Distancefield>* getDistanceAddress(){return &distance;};

	void drawCollisionInfo(Vec3d color);
	//void ComputeforcefromCompliance(Meshfree_GPU* surf);





private:
	void collisionBtwAABBAndBox(AABBNode* root, Box& box, std::vector<int>& collisionIdx);
	void traverseBVHPQ(AABBNode* root1, AABBNode* root2, std::vector<Vec2i>& pTriPair, float dis);
	void traverseBVHPQ(AABBNode* root, Vec3f point, std::vector<int>& pNodeIdx, float dis);
	void measureMinimumDistanceBetPointandTriangle(Vec3d p1, Vec3d tri1, Vec3d tri2, Vec3d tri3, Vec3d &collidedTri, Vec3d &collidedCyl);
	Vec3d findClosestPointInTri(Vec3d point, Vec3d tri1, Vec3d tri2, Vec3d tri3);
	Vec3d findProjectedPointToPlane(Vec3d Point, Vec3d Normal, Vec3d Center);
	void measureMinimumDistanceBetLineandTriangle(Vec3d p1, Vec3d p2,Vec3d tri1, Vec3d tri2, Vec3d tri3, Vec3d &collidedTri, Vec3d &collidedCyl);
	Vec3d findPointBetweenTwoline1(Vec3d p1, Vec3d p2, Vec3d p3, Vec3d p4);
	double projectOnLine(Vec3d r0, Vec3d direc, Vec3d P);
	
	void traverseEdgeBVHByLine(AABBNode* root, std::vector<Vec3f>* toolPoint, float radius, std::vector<int>& colEdgeIdx );

};
#endif