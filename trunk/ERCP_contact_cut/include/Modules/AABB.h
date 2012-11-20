#ifndef AABB_H
#define AABB_H

#include "DataTypes/Define.h"
#include "DataTypes/Vec.h"
#include "DataTypes/Mat.h"
#include <afxwin.h>
#include <GL/glu.h>
#include <GL/gl.h>
#include <vector>
#include <map>

class AABBNode
{
public:
	AABBNode(void);
	virtual ~AABBNode(void);

//functions
public:
	//set values
	void setBoundingBox(Vec3f leftDown, Vec3f rightUp);
	
	//drawings
	void drawBoundingBox();

	//copy data
	void copy(AABBNode* node);

	std::vector<int>* indicesInLeafNode(){return &IndicesInLeafNode;};
	

//variables
public:

	//Bounding box
	Vec3f LeftDown;
	Vec3f RightUp;

	//Ascendant
	AABBNode* Parent;

	//descendant
	AABBNode* Left;
	AABBNode* Right;

	//indicate leaf node
	bool End;

	//indicate update
	bool Updated;

	//indicate depth of the tree
	int Depth;

	//indicate depth component index
	int IndexInLeafNode;
	std::vector<int> IndicesInLeafNode;
};

class AABBTree
{
public:
	AABBTree(void);
	virtual ~AABBTree(void);

//functions
public:
	
	//AABB tree construction
	virtual void constructAABBTree();
	virtual void constructAABBTree(std::vector<int>& idx);

	//AABB tree update bottom up
	virtual void updateAABBTreeBottomUp();

	//AABB tree search
	virtual AABBNode* findLeafNode(int idx);

	virtual bool findCellIncludingEdge(int edgeIdx, std::vector<int>& cellIdx);

	//AABB tree update for topology change
	void removeNode(AABBNode* node);
	void addNode(AABBNode* node);
	
	//update depth
	void updateDepth(AABBNode* root);

	//copy tree
	void copy(AABBTree* tree);
	void copy(AABBNode* root, AABBTree* tree, AABBNode* rootCopy);

	//Drawings
	void drawBoundingBox();
	void drawBoundingBox(AABBNode* root);
	void drawBoundingBoxTo(int depth);
	void drawBoundingBoxTo(int depth, AABBNode* root);
	void drawBoundingBoxAt(int depth);
	void drawBoundingBoxAt(int depth, AABBNode* root);
	void drawBoundingBoxLeafNode();
	void drawBoundingBoxLeafNode(AABBNode* root);
	
	//get data
	AABBNode* root();

	//get all leaf node index
	void getIndexInLeafNode(std::vector<int>& index);
	void getIndexInLeafNode(AABBNode* root, std::vector<int>& index);

	//void changeIdxInLeafNode(int from, int to);
	
//variables
protected:
	
	//AABB
	AABBNode* Root;

	//LeafNodes
	//std::vector<int> LeafNodeIdx;
	//std::vector<AABBNode*> LeafNodePtr;
	//std::map<int,AABBNode*> LeafNode;
};
#endif