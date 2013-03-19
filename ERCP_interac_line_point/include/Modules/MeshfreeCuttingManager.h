#ifndef MESHFREE_CUTTING_MANAGER
#define MESHFREE_CUTTING_MANAGER

#include "DataTypes/Define.h"
#include "DataTypes/Vec.h"
#include "DataTypes/Mat.h"
#include "Modules/CollisionManager.h"
#include "Modules/Meshfree_CPU.h"
#include "Modules/Meshfree_GPU.h"
#include "Modules/EFGCuttingManager.h"
#include "Modules/SurfaceCuttingManager.h"
#include "Modules/CuttingTool.h"
#include "Modules/TimeTick.h"
#include <afxwin.h>
#include <GL/glu.h>
#include <GL/gl.h>
#include <vector>

class MeshfreeCuttingManager
{

public:
	MeshfreeCuttingManager(void);
	~MeshfreeCuttingManager(void);

//functions
public:
	void cutting(Meshfree_CPU* obj, std::vector<Vec3f>* cutPoint, std::vector<Vec3i>* cutFace, std::vector<Vec3f>* cutFaceNormal,  std::vector<int>* pointsOnCutFront);
	void cutting(Meshfree_GPU* obj, CuttingTool* tool);
	void cuttingProg(Meshfree_GPU* obj, CuttingTool* tool, int toolPathIdx);
	void updateConnection_CPU();
	void updateConnection_GPU();

	void updateConnection_GPUProg(int toolPathIdx);
	void updateConnection_GPUProg();

	std::vector<Vec3f>* addedPoint(){return AddedPoint;};

	//Cylinder cut
	bool cylinderCutting( Meshfree_GPU* obj, std::vector<Vec3f>* toolPoint_in, float radius );
	void updateConnectionCylinder();
public:
	FILE* F;
	CTimeTick TimeTick;
	CuttingTool* Tool;

	Meshfree_CPU* Obj_CPU;
	Meshfree_GPU* Obj_GPU;
	std::vector<Vec3f>* CutPoint;
	std::vector<Vec3i>* CutSurf;
	std::vector<Vec3f>* CutFaceNormal;

	int PreNbPoint;
	std::vector<Vec3f>* AddedPoint;
	std::vector<Vec3f>* AddedPointNormal;
	std::vector<int>* VertexOnCutFront;

	AABBTreeTri BVHCutSurf;
	SurfaceCuttingManager SurfCutting;

	// Cylinder	
	std::vector<Vec3f>* toolPoint;
	float toolRadius;
};

#endif