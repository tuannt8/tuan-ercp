#ifndef MESHFREE_GPU
#define MESHFREE_GPU

#include "DataTypes/Define.h"
#include "DataTypes/Vec.h"
#include "DataTypes/Mat.h"
#include "Graphics/SurfaceObj.h"
#include "Modules/EFG_CUDA_RUNTIME.h"
#include "Modules/MeshfreeNodeGenerator.h"
#include "Modules/TimeTick.h"
#include "Modules/AABBEdgeDiff.h"
#include <afxwin.h>
#include <GL/glu.h>
#include <GL/gl.h>
#include <vector>

class Meshfree_GPU
{
public:
	Meshfree_GPU(void);
	~Meshfree_GPU(void);

//functions
public:
	void loadSurfObj(char* filename);
	void generateEFGObj(int res, bool stress);
	void connectSurfAndEFG();
	void initFixedConstraintGPU();

	// drawing
	void drawSurfObj(Vec3f color, int mode);
	void drawEFGObj(Vec3f color, float radius, int mode);

	// deformation
	void updatePositionExplicit(float dt, int itter);
	void updatePositionExplicitFree(float dt);
	void updatePositionExplicitConst(float dt);
	void updateSurfPosition();
	void boxConstraint(Vec3f leftDown, Vec3f rightUp);
	void updateShapeFunction(std::vector<int>& pointIdx);

	void removeEdges(std::vector<int>& idx);

	// get data
	SurfaceObj* surfObj(){return SurfObj;};
	EFG_CUDA_RUNTIME* efgObj(){return EFGObj;};
	float supportRadiusSurf(){return SupportRadiusSurf;};
	std::vector<std::vector<int>>* neighborNodeOfSurfVertice(){return &NeighborNodeOfSurfVertice;};
	std::vector<std::vector<float>>* shapeFuncValueAtSurfPoint(){return &ShapeFuncValueAtSurfPoint;};
	AABBTreeEdgeDiff* getBVH(){return &BVHAABB;};
	std::vector<Vec2i>* edge(){return &Edge;};

private:
	void initSurfaceNeighborNode(float supportRadius);
	void initShapeFuncValueAtSurfPoint(float supportRadius);
	
//variables
private:
	FILE* F;
	CTimeTick TimeTick;

	// 1. Surface object
	SurfaceObj* SurfObj;

	// 2. EFG object
	EFG_CUDA_RUNTIME* EFGObj;

	// 3. Support radius
	float SupportRadius;
	float SupportRadiusSurf;

	// 4. Neighbor nodes of each surface vertex
	std::vector<std::vector<int>> NeighborNodeOfSurfVertice;

	// 5. Shape function value at surface point
	std::vector<std::vector<float>> ShapeFuncValueAtSurfPoint;

	// 7. AABB tree
	AABBTreeEdgeDiff BVHAABB;

	// 8. Edges
	std::vector<Vec2i> Edge;
	std::vector<std::vector<int>> EdgeAroundPoint;

	float EFGTime;
	float SurfTime;
	float Count;
};

#endif