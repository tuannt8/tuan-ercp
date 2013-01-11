#ifndef MESHFREE_TOPOLOGY_CTR
#define MESHFREE_TOPOLOGY_CTR

#include "DataTypes/Define.h"
#include "DataTypes/Vec.h"
#include "DataTypes/Mat.h"
#include "Graphics/SurfaceObj.h"
#include "Modules/CollisionManager.h"
#include <afxwin.h>
#include <GL/glu.h>
#include <GL/gl.h>
#include <vector>

class MeshfreeNode
{
public:
	MeshfreeNode(MeshfreeNode* parent, Vec3f nodePos, Vec3f boxSize);
	~MeshfreeNode();

public:
	void findNeighborNode(std::vector<std::vector<int>>& neighbor, std::vector<int>& dest, std::vector<Vec3f>* nodePos, float support);
	void generateChildNode(SurfaceObj* obj);
	void activateChildNode(SurfaceObj* obj);

	void drawNode(Vec3f color, float radius);
	void drawActiveChildNode(Vec3f color, float radius);
	void drawBox();


	// get data
	MeshfreeNode* parent(){return Parent;};
	MeshfreeNode* child(int idx){return Child[idx];};
	int nodeIdx(){return NodeIdx;};
	Vec3f nodePos(){return NodePos;};
	Vec3f leftDown(){return LeftDown;};
	Vec3f rightUp(){return RightUp;};
	float volume(){return Volume;};

	// set data
	void setNodeIdx(int idx);
	void activate();
	void deAtivate();

	bool hasChild();
	bool isActive();

	// get active child node
	bool getActiveChildNode(std::vector<MeshfreeNode*>& node);
	void rootNode(std::vector<MeshfreeNode*>& root);

private:
	MeshfreeNode* Parent;
	MeshfreeNode* Child[8];

	int NodeIdx;
	Vec3f NodePos;
	Vec3f LeftDown;
	Vec3f RightUp;
	float Volume;
	bool Active;
};

class MeshfreeTopologyCtr
{
public:
	MeshfreeTopologyCtr();
	~MeshfreeTopologyCtr();

// functions
public:
	void init(SurfaceObj* obj, std::vector<Vec3f>* nodePos, Vec3f boxSize);
	void initNeighbor();
	void legalizeNeighbor(std::vector<int>& illegalNodeIdx);
	void glPrint(const char* text);
	void printNbNeighbor();
	void drawNodeBox(Vec3f color, float radius);

	//get data
	SurfaceObj* surfObj(){return SurfObj;};
	std::vector<Vec3f>* nodePos(){return &NodePos;};
	std::vector<std::vector<int>>* neighborNodeIdx(){return &NeighborNodeIdx;};
	float supportRadius(){return SupportRadius;};

private:
	SurfaceObj* SurfObj;
	std::vector<Vec3f> NodePos;
	std::vector<MeshfreeNode*> NodeCtr;
	std::vector<std::vector<int>> NeighborNodeIdx;
	float SupportRadius;
	

	int count;
};

#endif