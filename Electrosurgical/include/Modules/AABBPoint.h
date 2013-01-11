#ifndef AABBPOINT_H
#define AABBPOINT_H

#include "DataTypes/Define.h"
#include "DataTypes/GeometricFunc.h"
#include "DataTypes/Vec.h"
#include "DataTypes/Mat.h"
#include "Modules/AABB.h"
#include <afxwin.h>
#include <GL/glu.h>
#include <GL/gl.h>
#include <vector>

class AABBTreePoint : public AABBTree
{
public:
	AABBTreePoint(void);
	AABBTreePoint(std::vector<Vec3f>* node, int nbNodeInBox);
	virtual ~AABBTreePoint(void);

//functions
public:
	void init(std::vector<Vec3f>* nodePos, int nbNodeInBox);

	//AABB tree construction
	virtual void constructAABBTree();
	virtual void constructAABBTree(std::vector<int>& nodeIdx);

	//AABB tree update bottom up
	virtual void updateAABBTreeBottomUp();

private:
	void updateBoundingBoxBottomUp(AABBNode* root);
	void updateAABBLeafNode(AABBNode* root);
	void generateBoundingBox(AABBNode* root);
	void generateBoundingBox(std::vector<int>& edgeIdx, AABBNode* root);
	
//variables
private:
	std::vector<Vec3f>* Node;
	int NbNodeInBox;
};
#endif