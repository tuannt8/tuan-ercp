#ifndef MESHFREE_NODE_GENERATOR
#define MESHFREE_NODE_GENERATOR



#include "Graphics/SurfaceObj.h"
#include "DataTypes/Define.h"
#include "DataTypes/Vec.h"
#include "DataTypes/Mat.h"
#include "Modules/CollisionManager.h"
#include <afxwin.h>
#include <GL/glu.h>
#include <GL/gl.h>
#include <vector>

class MeshfreeNodeGenerator
{
public:
	MeshfreeNodeGenerator(void);
	~MeshfreeNodeGenerator(void);

//functions
public:
	void setSurfObj(SurfaceObj* obj);
	void generateBoundingBox(std::vector<Vec3f>* point);

	// 1. Uniform node placement
	void generateUniformNode(int res);
	void generateNodeInBoundingBox(Vec3i res);
	void removeNodeOutsideObject();
	void computeNodeVolume();

	// 2. Stress point integration
	void generateUniformNodeWithStressPoint(int res);
	void generateNodeInBoundingBoxWithStressPoint(Vec3i res);
	void removeNodeOutsideObjectWithStressPoint();
	void computeNodeVolumeWithStressPoint();

	// 3. Multi-resolution approach
	int findNodeNearSurface(float radius);
	
	// get data
	std::vector<Vec3f>* nodePos(){return &NodePos;};
	std::vector<Vec3f>* stressPoint(){return &StressPoint;};
	Vec3f boxSize(){return BoxSize;};
	float nodeVolume(){return NodeVolume;};
	
//variables
private:
	SurfaceObj* Obj;
	Vec3f LeftDownPoint;
	Vec3f RightUpPoint;
	Vec3f BoxSize;
	std::vector<Vec3f> NodePos;
	std::vector<Vec3f> StressPoint;
	float NodeVolume;
};

#endif