#ifndef MESHFREE
#define MESHFREE

#include "DataTypes/Define.h"
#include "DataTypes/Vec.h"
#include "DataTypes/Mat.h"
#include "Graphics/SurfaceObj.h"
#include "Modules/EFG_CPU.h"
#include "Modules/MeshfreeNodeGenerator.h"
#include "Modules/MeshfreeTopologyCtr.h"
#include "Modules/CollisionManager.h"
#include <afxwin.h>
#include <GL/glu.h>
#include <GL/gl.h>
#include <vector>

class Meshfree
{
public:
	Meshfree(void);
	~Meshfree(void);

//functions
public:
	void loadSurfObj(char* filename);
	void generateNode(int res);
	void drawSurfObj(Vec3f color, int mode);
	void drawMeshfreeNode(Vec3f color, float radius, int mode);
	void drawNodeBox(Vec3f color, float radius);
	void printNbNeighbor();
	
//variables
private:

	// 1. Surface object
	SurfaceObj* SurfObj;

	// 2. EFG object
	EFG_CPU* EFGObj;

	// 3. Meshfree node generator
	MeshfreeNodeGenerator NodeGenerator;

	// 4. Meshfree topology controller
	MeshfreeTopologyCtr TopologyCtr;

	// 4. Support radius
	float SupportRadius;
};

#endif