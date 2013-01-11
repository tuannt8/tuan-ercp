#ifndef MESHFREE_CPU
#define MESHFREE_CPU

#include "DataTypes/Define.h"
#include "DataTypes/Vec.h"
#include "DataTypes/Mat.h"
#include "Graphics/SurfaceObj.h"
#include "Modules/EFG_CPU.h"
#include "Modules/MeshfreeNodeGenerator.h"
#include "Modules/TimeTick.h"
#include <afxwin.h>
#include <GL/glu.h>
#include <GL/gl.h>
#include <vector>

class Meshfree_CPU
{
public:
	Meshfree_CPU(void);
	~Meshfree_CPU(void);

//functions
public:
	void loadSurfObj(char* filename);
	void generateEFGObj(int res);
	void connectSurfAndEFG();

	// drawing
	void drawSurfObj(Vec3f color, int mode);
	void drawEFGObj(Vec3f color, float radius, int mode);

	// deformation
	void updatePositionExplicit(float dt, int itter);
	void updateSurfPosition();
	void boxConstraint(Vec3f leftDown, Vec3f rightUp);
	void updateShapeFunction(std::vector<int>& pointIdx);

	// get data
	SurfaceObj* surfObj(){return SurfObj;};
	EFG_CPU* efgObj(){return EFGObj;};
	float supportRadius(){return SupportRadius;};
	float supportRadiusSurf(){return SupportRadiusSurf;};
	std::vector<std::vector<int>>* neighborNodeOfSurfVertice(){return &NeighborNodeOfSurfVertice;};
	std::vector<std::vector<float>>* shapeFuncValueAtSurfPoint(){return &ShapeFuncValueAtSurfPoint;};

	int findNearestNode(Vec3f point);

	////////////////////////////////////////////
	void testInitNeighbor(float support);
	void testInitShapeFuncValue(float support);
	void writeTest(char* filename);

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
	EFG_CPU* EFGObj;

	// 3. Support radius
	float SupportRadius;
	float SupportRadiusSurf;

	// 4. Neighbor nodes of each surface vertex
	std::vector<std::vector<int>> NeighborNodeOfSurfVertice;

	// 5. Shape function value at surface point
	std::vector<std::vector<float>> ShapeFuncValueAtSurfPoint;

	float EFGTime;
	float SurfTime;
	float Count;

	Vec3f TestPoint[1000];
	Vec3f TestPoint0[1000];
	Vec3f TestDis[1000];
	std::vector<int> TestNeighbor[1000];
	std::vector<float> TestShapeFuncValue[1000];
};

#endif