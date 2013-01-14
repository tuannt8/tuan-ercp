#ifndef ELEMENT_FREE_GALLERKIN_CUDA_RUNTIME
#define ELEMENT_FREE_GALLERKIN_CUDA_RUNTIME

#include "DataTypes/Define.h"
#include "DataTypes/VectorFunc.h"
#include "DataTypes/GeometricFunc.h"
#include "DataTypes/Vec.h"
#include "DataTypes/Mat.h"
#include "Modules/MLSShapeFunc.h"
#include "Modules/TimeTick.h"
#include "Modules/AABBEdge.h"
#include "Modules/AABBPoint.h"
#include "Modules/TimeTick.h"
#include <afxwin.h>
#include <GL/glu.h>
#include <GL/gl.h>
#include <vector>
#include <queue>
#include <cuda.h>
#include <cutil_inline.h>

#define WEIGHT_FUNC_TYPE 1
#define DIM 3
#define SDIM 6

#define _MAX 100000000
#define _MIN -100000000

#define NB_NEIGHBOR_MAX 70
#define COROTATION true

class EFG_CUDA_RUNTIME
{
public:
	EFG_CUDA_RUNTIME(void);
	~EFG_CUDA_RUNTIME(void);

public:
	// Initialization 1
	void init(std::vector<Vec3f>* nodePos, float nodeVolume, float supportRadius);
	void init(std::vector<Vec3f>* nodePos, float nodeVolume, float supportRadius, int nbNodeInside);

	// Initialization 2
	void init(char* filename);

	// Initialization 3 (stress point)
	void init(std::vector<Vec3f>* nodePos, std::vector<Vec3f>* stressPos, float nodeVolume, float supportRadius);

	// material stiffness matrix
	void constructMaterialStiffness();

	// init neighbor information
	void initNeighborInformation();

	// compute node volume
	void computeNodeMass();

	// init shape function value
	void initShapeFuncValue();

	// constraint
	void boxConstraint(Vec3f LeftDown, Vec3f RightUp);
	void initFixedConstraintGPU();

	// drawing
	void draw(Vec3f color, int radius, int mode);
	void drawStressPoint(Vec3f color, int radius, int mode);
	void drawEdge();
	void drawBVH();
	void drawBVHPoint();
	void drawEdge(int idx);
	void drawNodeOrientation();
	void drawCoord(Vec3f pos, Mat3x3f rot);
	void drawNodeInside(Vec3f color, int radius);

	// explicit integration
	void updatePositionExplicit(float dt, int itter);
	void updatePositionExplicitFree(float dt);
	void updatePositionExplicitConst(float dt);
	void updatePositionExplicit_CUDA(float dt, int itter);
	void updatePositionExplicitFree_CUDA(float dt);
	void updatePositionExplicitConst_CUDA(float dt);

	// add internal force
	void addInternalForce();
	void d_addInternalForce();;
	void computeStrain();
	void computeStress();
	void computeForce();

	// node rotation
	void computeNodeRotation();
	void updatePositionAtNodeCoord();

	// get data
	int nbNeighborNodeMax();
	int nbNeighborNodeMin();
	float nbNeighborNodeAve();

	int nbNode(){return NbNode;};
	std::vector<Vec3f>* nodePosVec(){return NodePosVec;};
	std::vector<Vec3f>* nodePos0Vec(){return NodePos0Vec;};
	std::vector<Vec2i>* edge(){return Edge;};
	float* nodeDis(){return NodeDis;};
	float* nodeForce(){return NodeForce;};
	float* nodeMass(){return NodeMass;};
	float* nodePos0(){return NodePos0;};
	float supportRadius(){return SupportRadius;};
	MLSShapeFunc* shapeFunc(){return &ShapeFunc;};
	AABBTreeEdge* getBVH(){return &BVHAABB;};
	AABBTreePoint* getBVHPoint(){return &BVHAABBPoint;};
	void setForce();

	//Topology control
	void removeEdges(std::vector<int>& idx);

private:
	float norm(float* a, float* b, int dim);

	//Topology control
	void removeEdgesInPhysicalModel(std::vector<int>& idx);
	void removeEdgesInCollisionModel(std::vector<int>& idx);
	void updateNeighborInfo(int idx, std::vector<int>& updatedIdx, std::vector<int>& updatedValue);
	void updateShapeFuncValue(std::vector<int>& updatedNodeIdx, std::vector<int>& updatedShapeFuncIdx, std::vector<Vec3f>& updatedVal, std::vector<Vec3f>& updatedValInv);

private:
	FILE* F;
	CTimeTick TimeTick;

	//Node information
	int NbNode;
	int NbStressPoint;
	int NbNodeAdded;
	int NbNodeAddedMax;
	int NbNodeInside;

	float* NodePos0;
	float* NodePos;
	float* NodeDis;
	float* NodeVel;
	float* NodeForce;
	float* NodeVolume;
	float* NodeMass;
	float* NodeStrain;
	float* NodeStress;

	float* StressPos;
	float* StressPos0;
	float* StressPointDis;
	float* StressVolume;
	float* StressPointStress;
	float* StressPointStrain;

	std::vector<Vec3f>* NodePos0Vec;
	std::vector<Vec3f>* NodePosVec;
	std::vector<Vec3f>* NodePosXVec;
	std::vector<Vec3f>* NodePosYVec;
	std::vector<Vec3f>* NodePosZVec;
	std::vector<Vec2i>* Edge;
	std::vector<std::vector<int>> EdgesAroundNode;

	//Support radius of the shape function
	float SupportRadius;
	float* MaterialStiffness; 

	//Neighbor information
	int* NeighborNodeIdx;
	int* NbNeighborNode;

	//Stress point neighbor information
	int* NeighborStressPointIdx;
	int* StressNeighborNodeIdx;
	int* NbNeighborStressPoint;
	int* NbStressNeighborNode;

	//Moving least square shape function
	MLSShapeFunc ShapeFunc;

	//Moment matrix and derivative
	Mat4x4f* MomentMatrixAtNode;
	Mat4x4f* MomentMatrixInvAtNode;

	Mat4x4f* MomentMatrixAtStressPoint;
	Mat4x4f* MomentMatrixInvAtStressPoint;

	Mat4x4f* MomentMatrixDrvXAtNode;
	Mat4x4f* MomentMatrixDrvYAtNode;
	Mat4x4f* MomentMatrixDrvZAtNode;

	Mat4x4f* MomentMatrixDrvXAtStressPoint;
	Mat4x4f* MomentMatrixDrvYAtStressPoint;
	Mat4x4f* MomentMatrixDrvZAtStressPoint;

	//Shape function and derivative
	float* ShapeFuncAtNode;
	float* ShapeFuncAtStressPoint;
	
	float* ShapeFuncDerivAtNode;
	float* ShapeFuncDerivXAtNode;
	float* ShapeFuncDerivYAtNode;
	float* ShapeFuncDerivZAtNode;

	float* ShapeFuncDerivAtNodeInv;
	float* ShapeFuncDerivXAtNodeInv;
	float* ShapeFuncDerivYAtNodeInv;
	float* ShapeFuncDerivZAtNodeInv;

	float* ShapeFuncDerivAtStressPoint;
	float* ShapeFuncDerivXAtStressPoint;
	float* ShapeFuncDerivYAtStressPoint;
	float* ShapeFuncDerivZAtStressPoint;
	float* ShapeFuncDerivAtStressPointInv;
	float* ShapeFuncDerivXAtStressPointInv;
	float* ShapeFuncDerivYAtStressPointInv;
	float* ShapeFuncDerivZAtStressPointInv;

	//Constraint
	std::vector<int> FixedNodeIdx;

	// Nodal rotation
	std::vector<Mat3x3f> NodeRot;

	Vec3f X;
	Vec3f Y;
	Vec3f Z;

	// Bounding volume hierarchy
	AABBTreeEdge BVHAABB;
	AABBTreePoint BVHAABBPoint;
	int NbNodeInBox;

	int NbUpdated;
	int Count;
};
#endif