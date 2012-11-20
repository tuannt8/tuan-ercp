#ifndef ELEMENT_FREE_GALLERKIN_CPU
#define ELEMENT_FREE_GALLERKIN_CPU

#include "DataTypes/Define.h"
#include "DataTypes/VectorFunc.h"
#include "DataTypes/GeometricFunc.h"
#include "DataTypes/Vec.h"
#include "DataTypes/Mat.h"
#include "DataTypes/matrix.h"
#include "Modules/MLSShapeFunc.h"
#include "Modules/TimeTick.h"
#include "Modules/AABBEdge.h"
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
#define COROTATION false

// CPU Uniform support
class EFG_CPU
{
public:
	EFG_CPU(void);
	~EFG_CPU(void);

//functions
public:
	FILE* F;
	CTimeTick TimeTick; 

	// Initialization 1
	void init(std::vector<Vec3f>* nodePos, float nodeVolume, float supportRadius);
	void init(std::vector<Vec3f>* nodePos, float nodeVolume, float supportRadius, int nbNodeInside);
	void init(std::vector<Vec3f>* nodePos, float nodeVolume, float supportRadius, float density);

	// Initialization 2
	void init(char* filename);
	
	void readNodeVolume(char* filename);
	void writeEFGFile(char* filename);
	
	void addFixedConstraint(int nodeIdx);
	void boxConstraint(Vec3f LeftDown, Vec3f RightUp);

	//add internal force
	void addInternalForce();
	void computeStrain();
	void computeStress();
	void computeForce();

	//update position
	void updatePositionExplicit(float dt, int itter);

	// drawing
	void draw(Vec3f color, int radius, int mode);
	void drawNodeInside(Vec3f color, int radius);
	void drawEdge();
	void drawBVH();
	void drawEdge(int idx);
	void drawNodeOrientation();
	void drawCoord(Vec3f pos, Mat3x3f rot);
	void drawNodeCoord();

	// get data
	std::vector<Vec3f>* nodePos0();
	std::vector<Vec3f>* nodePos();
	std::vector<Vec3f>* nodeVel();
	std::vector<Vec3f>* nodeForce();
	std::vector<Vec3f>* nodeDis();
	std::vector<float>* nodeSupportRadius();
	std::vector<Vec2i>* edge();
	float supportRadius();
	std::vector<float>* nodeMass();
	int nbNode();
	int nbNodeInside();
	int nbNeighborNodeMax();
	int nbNeighborNodeMin();
	float nbNeighborNodeAve();
	MLSShapeFunc* shapeFunc();
	AABBTreeEdge* getBVH();

	// global stiffness matrix
	void assembleGlobalStiffness();
	void writeGlobalStiffnessMatrix(char* filename);
	void rot(Mat3x3f mat);
	void setDensity(float density){Density=density;};

	//Topology control
	void removeEdges(std::vector<int>& idx);

private:
	void constructMaterialStiffness();
	void initNeighborInformation();
	void initShapeFuncValue();
	void reInitShapeFuncValue();
	void computeNodeVolume();
	void computeNodeRotation();

	//Topology control
	void removeEdgesInPhysicalModel(std::vector<int>& idx);
	void removeEdgesInCollisionModel(std::vector<int>& idx);
	void updateShapeFuncValue(std::vector<int>& updatedNodeIdx);

	// Node의 rotation matrix를 계산하기 위한 함수
	void updatePositionAtNodeCoord();

private:
	int NbNode;
	int NbNodeInside;

	// Nodal values
	std::vector<Vec3f> NodePos0;
	std::vector<Vec3f> NodePos;
	std::vector<Vec3f> NodePosX;
	std::vector<Vec3f> NodePosY;
	std::vector<Vec3f> NodePosZ;
	std::vector<Vec3f> NodeVel;
	std::vector<Vec3f> NodeForce;
	std::vector<Vec3f> NodeDis;
	std::vector<Vec6f> NodeStress;
	std::vector<Vec6f> NodeStrain;
	std::vector<Mat4x4f> MomentMatrixAtNode;
	std::vector<Mat4x4f> MomentMatrixAtNodeX;
	std::vector<Mat4x4f> MomentMatrixAtNodeY;
	std::vector<Mat4x4f> MomentMatrixAtNodeZ;
	std::vector<Mat4x4f> MomentMatrixInvAtNode;
	std::vector<Mat4x4f> MomentMatrixInvAtNodeX;
	std::vector<Mat4x4f> MomentMatrixInvAtNodeY;
	std::vector<Mat4x4f> MomentMatrixInvAtNodeZ;
	std::vector<Mat4x4f> MomentMatrixDrvXAtNode;
	std::vector<Mat4x4f> MomentMatrixDrvYAtNode;
	std::vector<Mat4x4f> MomentMatrixDrvZAtNode;

	// Neighbor node information
	std::vector<std::vector<int>> NeighborNodeIdx; // Nodal integration 할때 사용
	std::vector<std::vector<float>> ShapeFuncAtNode;
	std::vector<std::vector<float>> ShapeFuncAtNodeX;
	std::vector<std::vector<float>> ShapeFuncAtNodeY;
	std::vector<std::vector<float>> ShapeFuncAtNodeZ;

	std::vector<std::vector<float>> ShapeFuncDerivXAtNode;
	std::vector<std::vector<float>> ShapeFuncDerivYAtNode;
	std::vector<std::vector<float>> ShapeFuncDerivZAtNode;
	std::vector<std::vector<float>> ShapeFuncDerivXAtNodeInv;
	std::vector<std::vector<float>> ShapeFuncDerivYAtNodeInv;
	std::vector<std::vector<float>> ShapeFuncDerivZAtNodeInv;

	// Weight function value
	std::vector<std::vector<float>> WeightFunc;

	// Vectors to neighbhor node
	std::vector<std::vector<Vec3f>> VecToNeighbor;

	// Nodal rotation
	std::vector<Mat3x3f> NodeRot;

	// Node의 coordinate를 나타내는 주변 노드
	std::vector<Vec3i> NodeCoordIdx;
	std::vector<Vec3i> NodeCoordSign;

	// Edges
	std::vector<Vec2i> Edge;
	std::vector<std::vector<int>> EdgeAroundNode;

	// Constraint
	std::vector<int> FixedNodeIdx;
	int ConstrainedNodeIdx;
	Vec3f ConstrainedDis;

	// Moving least square shape functions
	MLSShapeFunc ShapeFunc;

	// Physical property
	std::vector<float> NodeMass;
	Mat<SDIM, SDIM, float> MaterialStiffness;

	float SupportRadius;
	std::vector<float> NodeVolume;

	// AABB Tree
	Matrix* GlobalStiffness;
	Vec3f X;
	Vec3f Y;
	Vec3f Z;
	float Density;

	// Bounding volume hierarchy
	AABBTreeEdge BVHAABB;

};
#endif