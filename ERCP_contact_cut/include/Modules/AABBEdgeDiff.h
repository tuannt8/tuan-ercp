#ifndef AABBEDGE_DIFF_H
#define AABBEDGE_DIFF_H

#include "DataTypes/Define.h"
#include "DataTypes/GeometricFunc.h"
#include "DataTypes/Vec.h"
#include "DataTypes/Mat.h"
#include "Modules/AABB.h"
#include <afxwin.h>
#include <GL/glu.h>
#include <GL/gl.h>
#include <vector>

class AABBTreeEdgeDiff : public AABBTree
{
public:
	AABBTreeEdgeDiff(void);
	AABBTreeEdgeDiff(std::vector<Vec3f>* point, std::vector<Vec3f>* node, std::vector<Vec2i>* edge);
	virtual ~AABBTreeEdgeDiff(void);

//functions
public:
	void init(std::vector<Vec3f>* point, std::vector<Vec3f>* node, std::vector<Vec2i>* edge);

	//AABB tree construction
	virtual void constructAABBTree();
	virtual void constructAABBTree(std::vector<int>& edgeIdx);

	//AABB tree update bottom up
	virtual void updateAABBTreeBottomUp();
	virtual AABBNode* findLeafNode(int idx);

private:
	void updateBoundingBoxBottomUp(AABBNode* root);
	void updateAABBLeafNode(AABBNode* root);
	void generateBoundingBox(AABBNode* root);
	void generateBoundingBox(std::vector<int>& edgeIdx, AABBNode* root);
	void treeValancing(int nbTotal, std::vector<int>& edgeIdx1, std::vector<int>& edgeIdxInBoundary1, std::vector<int>& edgeIdx2, std::vector<int>& edgeIdxInBoundary2);
	void findLeafNode(AABBNode* root, int edgeIdx, std::vector<AABBNode*>& node);
	
//variables
private:
	std::vector<Vec3f>* Point;
	std::vector<Vec3f>* Node;
	std::vector<Vec2i>* Edge;
};
#endif