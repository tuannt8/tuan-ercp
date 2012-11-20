#ifndef MLS_SHAP_FUNC
#define MLS_SHAP_FUNC

#include "DataTypes/Define.h"
#include "DataTypes/Vec.h"
#include "DataTypes/Mat.h"
#include <afxwin.h>
#include <GL/glu.h>
#include <GL/gl.h>
#include <vector>

// EFG_CPU, uniform support radius에서 사용
class MLSShapeFunc
{
public:
	MLSShapeFunc(void);
	~MLSShapeFunc(void);

//functions
public:
	// initialization
	void init(std::vector<Vec3f>* nodePos, float supportRadius, int weightFuncType);
	void setSupport(float support);

	// moment matrix computation
	void computeMomentMatrix(Mat4x4f& moment, std::vector<int>& neighborNodeIdx, Vec3f pos);
	void computeMomentMatrix(Mat4x4f& moment, int* neighborNodeIdx, int nbNeighborNode, Vec3f pos);
	void computeMomentMatrixAtNode(Mat4x4f& moment, std::vector<int>& neighborNodeIdx, int nodeIdx);
	void computeMomentMatrixAtNode(Mat4x4f& moment, int* neighborNodeIdx, int nbNeighborNode, int nodeIdx);

	// moment matrix derivative (x)
	void computeMomentMatrixDrvX(Vec3f pos, Mat4x4f& momentDrvX, std::vector<int>& neighborNodeIdx);
	void computeMomentMatrixDrvX(Vec3f pos, Mat4x4f& momentDrvX, int* neighborNodeIdx, int nbNeighborNode);
	void computeMomentMatrixDrvXAtNode(int nodeIdx, Mat4x4f& momentDrvX, int* neighborNodeIdx, int nbNeighbor);
	void computeMomentMatrixDrvXAtNode(int nodeIdx, Mat4x4f& momentDrvX, std::vector<int>& neighborNodeIdx);
	void computeMomentMatrixDrvXAtNode(Mat4x4f* momentDrvX, int* neighborNodeIdx, int* nbNeighborNode, int nbNeighborMax);
	
	// moment matrix derivative (y)
	void computeMomentMatrixDrvY(Vec3f pos, Mat4x4f& momentDrvY, std::vector<int>& neighborNodeIdx);
	void computeMomentMatrixDrvY(Vec3f pos, Mat4x4f& momentDrvY, int* neighborNodeIdx, int nbNeighborNode);
	void computeMomentMatrixDrvYAtNode(int nodeIdx, Mat4x4f& momentDrvY, int* neighborNodeIdx, int nbNeighbor);
	void computeMomentMatrixDrvYAtNode(int nodeIdx, Mat4x4f& momentDrvY, std::vector<int>& neighborNodeIdx);
	void computeMomentMatrixDrvYAtNode(Mat4x4f* momentDrvY, int* neighborNodeIdx, int* nbNeighborNode, int nbNeighborMax);

	// moment matrix derivative (z)
	void computeMomentMatrixDrvZ(Vec3f pos, Mat4x4f& momentDrvZ, std::vector<int>& neighborNodeIdx);
	void computeMomentMatrixDrvZ(Vec3f pos, Mat4x4f& momentDrvZ, int* neighborNodeIdx, int nbNeighborNode);
	void computeMomentMatrixDrvZAtNode(int nodeIdx, Mat4x4f& momentDrvZ, int* neighborNodeIdx, int nbNeighbor);
	void computeMomentMatrixDrvZAtNode(int nodeIdx, Mat4x4f& momentDrvZ, std::vector<int>& neighborNodeIdx);
	void computeMomentMatrixDrvZAtNode(Mat4x4f* momentDrvZ, int* neighborNodeIdx, int* nbNeighborNode, int nbNeighborMax);
	
	// shape function computation
	float computeShapeFuncValue(int nodeIdx, Vec3f pos, Mat4x4f& momentInv);
	float computeShapeFuncDrvX(int nodeIdx, Vec3f pos, Mat4x4f& momentInv, Mat4x4f& momentDrv);
	float computeShapeFuncDrvY(int nodeIdx, Vec3f pos, Mat4x4f& momentInv, Mat4x4f& momentDrv);
	float computeShapeFuncDrvZ(int nodeIdx, Vec3f pos, Mat4x4f& momentInv, Mat4x4f& momentDrv);
	
	// weight function computation
	float weightFunc(float r, int funcType);

private:
	// moment matrix computation
	void constructMomentMatrix(Mat4x4f& moment, int nodeIdx, std::vector<int>& neighborNodeIdx);
	void constructMomentMatrix(Mat4x4f& moment, Vec3f pos, std::vector<int>& neighborNodeIdx);
	void constructMomentMatrix(Mat4x4f& moment, Vec3f pos, int* neighborNodeIdx, int nbNeighborNode);

	// weight function derivative computation
	float weightFuncDrvX(float r, int funcType, float x, float support);
	float weightFuncDrvY(float r, int funcType, float y, float support);
	float weightFuncDrvZ(float r, int funcType, float z, float support);

	// vector로 matrix 만들기
	void constructMatrixFrmVector(Mat4x4f& mat, Vec4d vec);
	void constructMatrixFrmVector(Mat4x4f& mat, Vec4d p, Vec4d pt);

private:
	std::vector<Vec3f>* NodePos;
	float SupportRadius;
	int WeightFuncType;
};
#endif