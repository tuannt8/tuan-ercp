#include "StdAfx.h"


#include "./DataTypes/Define.h"
#include "./DataTypes/Vec.h"
#include "./DataTypes/Mat.h"
#include "./DataTypes/Quat.h"
#include "./Modules/TimeTick.h"
#include "DataTypes/VectorFunc.h"
#include "DataTypes/SparseMatrix.h"


#include "Modules/Meshfree_CPU.h"
#include "Modules/CuttingTool.h"
#include "Modules/Meshfree_GPU.h"
#include "Modules/CollisionManager.h"

#include <GL/glu.h>
#include <GL/gl.h>
#include <vector>

#include "Cmat.h"

#include "armadillo"

using namespace arma;


class CollisionResponse
{
public:
	CollisionResponse(void);
	~CollisionResponse(void);


	void ComputeforcefromComplianceV1(Meshfree_GPU* surf,CollisionManager* collision);
	void ComputeforcefromComplianceV2(float dt, int itter, Meshfree_GPU* surf,CollisionManager* collision);
	void ComputeforcefromComplianceV3(float dt, int itter, Meshfree_GPU* surf,CollisionManager* collision);
	void ComputeforcefromComplianceV4(float dt, int itter, Meshfree_GPU* surf,CollisionManager* collision);
	void ComputeforcefromComplianceV5(float dt, int itter, Meshfree_GPU* surf,CollisionManager* collision);
	void ComputeforcefromComplianceV6(float dt, int itter, Meshfree_GPU* surf,CollisionManager* collision);
	void ComputeforcefromComplianceV7(float dt, int itter, Meshfree_GPU* surf,CollisionManager* collision);
	void ComputeforcefromComplianceV8(float dt, int itter, Meshfree_GPU* surf,CollisionManager* collision);
	void ComputeforcefromComplianceV9(float dt, int itter, Meshfree_GPU* surf,CollisionManager* collision);
	void ComputeforcefromComplianceV10(float dt, int itter, Meshfree_GPU* surf,CollisionManager* collision);
	void ComputeforcefromComplianceV11(float dt, int itter, Meshfree_GPU* surf,CollisionManager* collision);
	void PenaltyMethod(Meshfree_GPU* surf,CollisionManager* collision,double K);
	void ComputeforcefromComplianceV12(float dt, int itter, Meshfree_GPU* surf,CollisionManager* collision);

	
private:
	typedef struct  
	{
		Vec3d measuredPointInCylinder;
		Vec3d collidedCylPoint;
		Vec3d collidedTriPoint;
		Vec3d PenetratedDirec;
		double Penetration;
	} Distancefield;


	double* c;

	Cmat	GaussSeidelMethod1(Cmat mcm,Cmat penetration,int Iteration);
	mat		GaussSeidelMethod1(mat mcm,mat penetration,int Iteration);
	void	GaussSeidelMethod(Matrix& hcht, std::vector<float>& deltaFree, Matrix& force, int Iteration);


};
