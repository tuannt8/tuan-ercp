#ifndef MULTI_RES_MESHFREE_GPU
#define MULTI_RES_MESHFREE_GPU

#include "DataTypes/Define.h"
#include "DataTypes/Vec.h"
#include "DataTypes/Mat.h"
#include "Graphics/SurfaceObj.h"
#include "Modules/EFG_CUDA_RUNTIME.h"
#include "Modules/MeshfreeNodeGenerator.h"
#include <afxwin.h>
#include <GL/glu.h>
#include <GL/gl.h>
#include <vector>

class MultiResMeshfree_GPU
{
public:
	MultiResMeshfree_GPU(void);
	~MultiResMeshfree_GPU(void);

//functions
public:
	void loadSurfObj(char* filename);
	void generateEFGObj(int lowRes, int highRes);
	void connectSurfAndEFG();

	// drawing
	void drawSurfObj(Vec3f color, int mode);
	void drawLowResEFGObj(Vec3f color, float radius, int mode);
	void drawHighResEFGObj(Vec3f color, float radius, int mode);

	// deformation
	void updatePositionExplicit(float dt, int itter);
	void updateSurfPosition();
	void boxConstraint(Vec3f leftDown, Vec3f rightUp);
	void initFixedConstraintGPU();

	// get data
	SurfaceObj* surfObj(){return SurfObj;};
	EFG_CUDA_RUNTIME* lowResEFGObj(){return LowResEFGObj;};
	EFG_CUDA_RUNTIME* highResEFGObj(){return HighResEFGObj;};

private:
	void initSurfaceNeighborNode(float supportRadius);
	void initShapeFuncValueAtSurfPoint(float supportRadius);

	void initMultirResShapeFunc();
	void initMultiResNeighbor();
	
//variables
private:

	// 1. Surface object
	SurfaceObj* SurfObj;

	// 2. EFG object
	EFG_CUDA_RUNTIME* LowResEFGObj;
	EFG_CUDA_RUNTIME* HighResEFGObj;

	// 3. Support radius
	float SupportRadiusLow;
	float SupportRadiusHigh;
	float SupportRadiusSurf;

	// 4. Neighbor nodes of each surface vertex
	std::vector<std::vector<int>> NeighborNodeOfSurfVertice;

	// 5. Shape function value at surface point
	std::vector<std::vector<float>> ShapeFuncValueAtSurfPoint;

	// 6. Multiresolution approach
	std::vector<std::vector<int>> NeighborNodeIdx;
	std::vector<std::vector<float>> ShapeFuncValue;
};

#endif