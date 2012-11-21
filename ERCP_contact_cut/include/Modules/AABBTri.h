#ifndef AABBTRI_H
#define AABBTRI_H

#include "DataTypes/Define.h"
#include "DataTypes/GeometricFunc.h"
#include "DataTypes/Vec.h"
#include "DataTypes/Mat.h"
#include "Modules/AABB.h"
#include <afxwin.h>
#include <GL/glu.h>
#include <GL/gl.h>
#include <vector>

class AABBTreeTri : public AABBTree
{
public:
	AABBTreeTri(void);
	AABBTreeTri(std::vector<Vec3f>* point, std::vector<Vec3i>* face, std::vector<Vec2i>* edge);
	virtual ~AABBTreeTri(void);

//functions
public:
	void init(std::vector<Vec3f>* point, std::vector<Vec3i>* face, std::vector<Vec2i>* edge);

	//AABB tree construction
	virtual void constructAABBTree();
	virtual void constructAABBTree(std::vector<int>& faceIdx);
	virtual AABBNode* findLeafNode(int triIdx);

	void findLeafNode(AABBNode* _root, Vec3f* tri, std::vector<AABBNode*>* nodes);

	//AABB tree update bottom up
	virtual void updateAABBTreeBottomUp();
	
private:
	void updateBoundingBoxBottomUp(AABBNode* root);
	void updateAABBLeafNode(AABBNode* root);

	void generateBoundingBox(AABBNode* root);
	void generateBoundingBox(std::vector<int>& faceIdx, AABBNode* root);
	
//variables
private:
	
	//Triangle information
	std::vector<Vec3f>* Point;
	std::vector<Vec3i>* Face;
	std::vector<Vec2i>* Edge;
};
#endif