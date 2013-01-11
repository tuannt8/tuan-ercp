#ifndef MESHFREE_CONTACT_MANAGER
#define MESHFREE_CONTACT_MANAGER

#include "DataTypes/Define.h"
#include "DataTypes/Vec.h"
#include "DataTypes/Mat.h"
#include "DataTypes/matrix.h"
#include "Modules/CollisionManager.h"
#include "Modules/Meshfree_CPU.h"
#include "Modules/Meshfree_GPU.h"
#include "Modules/TimeTick.h"
#include "DataTypes/VectorFunc.h"
#include "DataTypes/SparseMatrix.h"
#include <afxwin.h>
#include <GL/glu.h>
#include <GL/gl.h>
#include <vector>
#include <map>

class MeshfreeContactManager
{

public:
	MeshfreeContactManager(void);
	~MeshfreeContactManager(void);

//functions
public:
	FILE* F;
	void contactResolutionWithSurfObj(Meshfree_GPU* obj, SurfaceObj* environment, float dt);
	void contactResolutionWithSurfObj(Meshfree_CPU* obj, SurfaceObj* environment, float dt);
	void contactResolutionWithPlane(Meshfree_CPU* obj, float dt);
	void constructMappingMatrix(Meshfree_CPU* obj, std::vector<int>& colNodeIdx, std::vector<Vec3f>& colNormal, std::vector<int>& relatedNodeIdx, Matrix& HT, Matrix& HCHT, float dt);
	void constructMappingMatrixSparse(Meshfree_CPU* obj, std::vector<int>& colNodeIdx, std::vector<Vec3f>& colNormal, std::vector<int>& relatedNodeIdx, Matrix& HT, Matrix& HCHT, float dt);
	void constructMappingMatrixSparse(Meshfree_GPU* obj, std::vector<int>& colNodeIdx, std::vector<Vec3f>& colNormal, std::vector<int>& relatedNodeIdx, Matrix& HT, Matrix& HCHT, float dt);

private:
	void GaussSeidelMethod(Matrix& hcht, std::vector<float>& deltaFree, Matrix& force, int Iteration);

private:
	Meshfree_CPU* Obj;
};

#endif