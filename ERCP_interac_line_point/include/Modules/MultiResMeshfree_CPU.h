#ifndef MULTI_RES_MESHFREE_CPU
#define MULTI_RES_MESHFREE_CPU

#include "DataTypes/Define.h"
#include "DataTypes/Vec.h"
#include "DataTypes/Mat.h"
#include "Graphics/SurfaceObj.h"
#include "Modules/EFG_CPU.h"
#include "Modules/MeshfreeNodeGenerator.h"
#include <afxwin.h>
#include <GL/glu.h>
#include <GL/gl.h>
#include <vector>

class MultiResMeshfree_CPU
{
public:
	MultiResMeshfree_CPU(void);
	~MultiResMeshfree_CPU(void);

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

	// get data
	SurfaceObj* surfObj(){return SurfObj;};
	EFG_CPU* lowResEFGObj(){return LowResEFGObj;};
	EFG_CPU* highResEFGObj(){return HighResEFGObj;};

	////////////////////////////////////////////
	void testInitNeighbor(float support);
	void testInitShapeFuncValue(float support);
	void writeTest(char* filename);

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
	EFG_CPU* LowResEFGObj;
	EFG_CPU* HighResEFGObj;

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


	Vec3f TestPoint[1000];
	Vec3f TestPoint0[1000];
	Vec3f TestDis[1000];
	std::vector<int> TestNeighbor[1000];
	std::vector<float> TestShapeFuncValue[1000];
};

#endif