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
	bool collisionBtwSurfAndLineSeg(SurfaceObj* obj, Vec3f l1, Vec3f l2);
	bool collisionBtwSurfAndLineSeg(std::vector<Vec3f>* point, std::vector<Vec3i>* face, AABBNode* root, Vec3f l1, Vec3f l2);
	bool collisionBtwAABBAndAxisLine(AABBNode* root, Vec3f pos, Vec3i direc, std::vector<int>& pCollisionTriIdx);
	bool collisionBtwAABBAndLineSeg(AABBNode* root, Vec3f l1, Vec3f l2, std::vector<int>& pCollisionTriIdx);
	bool isPointInSurfObj(SurfaceObj* obj, Vec3f pos);

	void PQBtwSurfaceObj(SurfaceObj* obj, SurfaceObj* environment, std::vector<int>& pointIdx, std::vector<Vec3f>& collisionPoint, std::vector<float>& delta, float dis);
	void PQBtwBoxAndPoint(AABBNode* root, std::vector<Vec3f>* nodePos, Vec3f point, std::vector<int>& pointIdx, float dis);
	void collisionBtwTriAndEdgesWithBVHDiff(std::vector<Vec3f>* posT, std::vector<Vec3i>* tri, std::vector<Vec3f>* point, std::vector<Vec3f>* nodePos, std::vector<Vec2i>* edge, AABBTreeEdgeDiff* BVH, std::vector<int>& colEdgeIdx);
private:
	void collisionBtwAABBAndBox(AABBNode* root, Box& box, std::vector<int>& collisionIdx);
	void traverseBVHPQ(AABBNode* root1, AABBNode* root2, std::vector<Vec2i>& pTriPair, float dis);
	void traverseBVHPQ(AABBNode* root, Vec3f point, std::vector<int>& pNodeIdx, float dis);
};
#endif