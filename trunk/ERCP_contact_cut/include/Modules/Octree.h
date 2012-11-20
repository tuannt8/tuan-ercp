#ifndef OCTREE_H
#define OCTREE_H

#include "DataTypes/Define.h"
#include "DataTypes/Vec.h"
#include "DataTypes/Mat.h"
#include <afxwin.h>
#include <GL/glu.h>
#include <GL/gl.h>
#include <vector>

#define OCTREE_UNIFORM 0
#define OCTREE_MULTILEVEL 1
#define OCTREE_NORMAL 2

class OctreeNode
{
public:
	OctreeNode(void);
	~OctreeNode(void);

//functions
public:
	void setBoundingBox(Vec3f& leftDown, Vec3f& rightUp);
	void drawBoundingBox();

//variables
public:
	bool			End;
	OctreeNode*		Descendant[8];
	Vec3f			LeftDown;
	Vec3f			RightUp;
	int				NodeIdx;
	int				Depth;
};

class Octree
{
public:
	Octree(void);
	//Octree(Vec3f* node, int nbNode);
	Octree(std::vector<Vec3f>* node, int nbNode);
	Octree(std::vector<Vec3f>* node, int nbNode, int mode);
	~Octree(void);

//functions
public:
	void constructOctree(int depth);
	int genOctree(OctreeNode* root, Vec3f& leftDown, Vec3f& rightUp, std::vector<int>& nodeIdx, int maxDepth);

	void drawBoundingBox(int depth);
	void drawBoundingBox(OctreeNode* root, int depth);
	void drawBoundingBox();
	void drawBoundingBox(OctreeNode* root);

	OctreeNode* root(){return Root;};

//variables
private:
	
	//Meshfree node information
	std::vector<Vec3f>* Node;
	int NbNode;
	int Mode;

	//AABB
	OctreeNode* Root;
};

#endif