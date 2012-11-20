#include "stdafx.h"
#include "MLSShapeFunc.h"

MLSShapeFunc::MLSShapeFunc(void)
{
	NodePos=NULL;
}

MLSShapeFunc::~MLSShapeFunc(void)
{
}

void MLSShapeFunc::init(std::vector<Vec3f>* nodePos, float supportRadius, int weightFuncType)
{
	NodePos=nodePos;
	SupportRadius=supportRadius;
	WeightFuncType=weightFuncType;
}

void MLSShapeFunc::setSupport(float support)
{
	SupportRadius=support;
}

// Moment matrix computation
void MLSShapeFunc::computeMomentMatrix(Mat4x4f& moment, std::vector<int>& neighborNodeIdx, Vec3f pos)
{
	constructMomentMatrix(moment, pos, neighborNodeIdx);
}

void MLSShapeFunc::computeMomentMatrix(Mat4x4f& moment, int* neighborNodeIdx, int nbNeighborNode, Vec3f pos)
{
	constructMomentMatrix(moment, pos, neighborNodeIdx, nbNeighborNode);
}

void MLSShapeFunc::computeMomentMatrixAtNode(Mat4x4f& moment, std::vector<int>& neighborNodeIdx, int nodeIdx)
{
	constructMomentMatrix(moment, nodeIdx, neighborNodeIdx);
}

void MLSShapeFunc::computeMomentMatrixAtNode(Mat4x4f& moment, int* neighborNodeIdx, int nbNeighborNode, int nodeIdx)
{
	constructMomentMatrix(moment, (*NodePos)[nodeIdx], neighborNodeIdx, nbNeighborNode);
}

void MLSShapeFunc::computeMomentMatrixDrvX(Vec3f pos, Mat4x4f& momentDrvX, std::vector<int>& neighborNodeIdx)
{
	momentDrvX.clear();
	for(unsigned int i=0;i<neighborNodeIdx.size();i++)
	{
		Vec4d p(1, (*NodePos)[neighborNodeIdx[i]][0], (*NodePos)[neighborNodeIdx[i]][1], (*NodePos)[neighborNodeIdx[i]][2]); 
		Mat4x4f pxpt;
		constructMatrixFrmVector(pxpt, p);
		Vec3f r=(pos-(*NodePos)[neighborNodeIdx[i]])/SupportRadius;
		float radius=r.norm();
		momentDrvX=momentDrvX+pxpt*weightFuncDrvX(radius, WeightFuncType, r[0], SupportRadius);
	}
}

void MLSShapeFunc::computeMomentMatrixDrvX(Vec3f pos, Mat4x4f& momentDrvX, int* neighborNodeIdx, int nbNeighborNode)
{
	momentDrvX.clear();
	for(unsigned int i=0;i<nbNeighborNode;i++)
	{
		Vec4d p(1, (*NodePos)[neighborNodeIdx[i]][0], (*NodePos)[neighborNodeIdx[i]][1], (*NodePos)[neighborNodeIdx[i]][2]); 
		Mat4x4f pxpt;
		constructMatrixFrmVector(pxpt, p);
		Vec3f r=(pos-(*NodePos)[neighborNodeIdx[i]])/SupportRadius;
		float radius=r.norm();
		momentDrvX=momentDrvX+pxpt*weightFuncDrvX(radius, WeightFuncType, r[0], SupportRadius);
	}
}

void MLSShapeFunc::computeMomentMatrixDrvXAtNode(int nodeIdx, Mat4x4f& momentDrvX, int* neighborNodeIdx, int nbNeighbor)
{
	momentDrvX.clear();
	for(unsigned int i=0;i<nbNeighbor;i++)
	{
		Vec4d p(1, (*NodePos)[neighborNodeIdx[i]][0], (*NodePos)[neighborNodeIdx[i]][1], (*NodePos)[neighborNodeIdx[i]][2]); 
		Mat4x4f pxpt;
		constructMatrixFrmVector(pxpt, p);
		Vec3f r=((*NodePos)[nodeIdx]-(*NodePos)[neighborNodeIdx[i]])/SupportRadius;
		float radius=r.norm();
		momentDrvX=momentDrvX+pxpt*weightFuncDrvX(radius, WeightFuncType, r[0], SupportRadius);
	}
}

void MLSShapeFunc::computeMomentMatrixDrvXAtNode(int nodeIdx, Mat4x4f& momentDrvX, std::vector<int>& neighborNodeIdx)
{
	momentDrvX.clear();
	for(unsigned int i=0;i<neighborNodeIdx.size();i++)
	{
		Vec4d p(1, (*NodePos)[neighborNodeIdx[i]][0], (*NodePos)[neighborNodeIdx[i]][1], (*NodePos)[neighborNodeIdx[i]][2]); 
		Mat4x4f pxpt;
		constructMatrixFrmVector(pxpt, p);
		Vec3f r=((*NodePos)[nodeIdx]-(*NodePos)[neighborNodeIdx[i]])/SupportRadius;
		float radius=r.norm();
		momentDrvX=momentDrvX+pxpt*weightFuncDrvX(radius, WeightFuncType, r[0], SupportRadius);
	}
}

void MLSShapeFunc::computeMomentMatrixDrvXAtNode(Mat4x4f* momentDrvX, int* neighborNodeIdx, int* nbNeighborNode, int nbNeighborMax)
{
	for(int i=0;i<NodePos->size();i++)
	{
		for(unsigned int j=0;j<nbNeighborNode[i];j++)
		{
			int idx=nbNeighborMax*i+j;
			Vec4d p(1, (*NodePos)[neighborNodeIdx[idx]][0], (*NodePos)[neighborNodeIdx[idx]][1], (*NodePos)[neighborNodeIdx[idx]][2]); 
			Mat4x4f pxpt;
			constructMatrixFrmVector(pxpt, p);
			Vec3f r=((*NodePos)[i]-(*NodePos)[neighborNodeIdx[idx]])/SupportRadius;
			float radius=r.norm();
			momentDrvX[i]=momentDrvX[i]+pxpt*weightFuncDrvX(radius, WeightFuncType, r[0], SupportRadius);
		}
	}
}

void MLSShapeFunc::computeMomentMatrixDrvY(Vec3f pos, Mat4x4f& momentDrvY, int* neighborNodeIdx, int nbNeighborNode)
{
	momentDrvY.clear();
	for(unsigned int i=0;i<nbNeighborNode;i++)
	{
		Vec4d p(1, (*NodePos)[neighborNodeIdx[i]][0], (*NodePos)[neighborNodeIdx[i]][1], (*NodePos)[neighborNodeIdx[i]][2]); 
		Mat4x4f pxpt;
		constructMatrixFrmVector(pxpt, p);
		Vec3f r=(pos-(*NodePos)[neighborNodeIdx[i]])/SupportRadius;
		float radius=r.norm();
		momentDrvY=momentDrvY+pxpt*weightFuncDrvY(radius, WeightFuncType, r[1], SupportRadius);
	}
}

void MLSShapeFunc::computeMomentMatrixDrvY(Vec3f pos, Mat4x4f& momentDrvY, std::vector<int>& neighborNodeIdx)
{
	momentDrvY.clear();
	for(unsigned int i=0;i<neighborNodeIdx.size();i++)
	{
		Vec4d p(1, (*NodePos)[neighborNodeIdx[i]][0], (*NodePos)[neighborNodeIdx[i]][1], (*NodePos)[neighborNodeIdx[i]][2]); 
		Mat4x4f pxpt;
		constructMatrixFrmVector(pxpt, p);
		Vec3f r=(pos-(*NodePos)[neighborNodeIdx[i]])/SupportRadius;
		float radius=r.norm();
		momentDrvY=momentDrvY+pxpt*weightFuncDrvY(radius, WeightFuncType, r[1], SupportRadius);
	}
}

void MLSShapeFunc::computeMomentMatrixDrvYAtNode(int nodeIdx, Mat4x4f& momentDrvY, int* neighborNodeIdx, int nbNeighbor)
{
	momentDrvY.clear();
	for(unsigned int i=0;i<nbNeighbor;i++)
	{
		Vec4d p(1, (*NodePos)[neighborNodeIdx[i]][0], (*NodePos)[neighborNodeIdx[i]][1], (*NodePos)[neighborNodeIdx[i]][2]); 
		Mat4x4f pxpt;
		constructMatrixFrmVector(pxpt, p);
		Vec3f r=((*NodePos)[nodeIdx]-(*NodePos)[neighborNodeIdx[i]])/SupportRadius;
		float radius=r.norm();
		momentDrvY=momentDrvY+pxpt*weightFuncDrvY(radius, WeightFuncType, r[1], SupportRadius);
	}
}

void MLSShapeFunc::computeMomentMatrixDrvYAtNode(int nodeIdx, Mat4x4f& momentDrvY, std::vector<int>& neighborNodeIdx)
{
	momentDrvY.clear();
	for(unsigned int i=0;i<neighborNodeIdx.size();i++)
	{
		Vec4d p(1, (*NodePos)[neighborNodeIdx[i]][0], (*NodePos)[neighborNodeIdx[i]][1], (*NodePos)[neighborNodeIdx[i]][2]); 
		Mat4x4f pxpt;
		constructMatrixFrmVector(pxpt, p);
		Vec3f r=((*NodePos)[nodeIdx]-(*NodePos)[neighborNodeIdx[i]])/SupportRadius;
		float radius=r.norm();
		momentDrvY=momentDrvY+pxpt*weightFuncDrvY(radius, WeightFuncType, r[1], SupportRadius);
	}
}

void MLSShapeFunc::computeMomentMatrixDrvYAtNode(Mat4x4f* momentDrvY, int* neighborNodeIdx, int* nbNeighborNode, int nbNeighborMax)
{
	for(int i=0;i<NodePos->size();i++)
	{
		for(unsigned int j=0;j<nbNeighborNode[i];j++)
		{
			int idx=nbNeighborMax*i+j;
			Vec4d p(1, (*NodePos)[neighborNodeIdx[idx]][0], (*NodePos)[neighborNodeIdx[idx]][1], (*NodePos)[neighborNodeIdx[idx]][2]); 
			Mat4x4f pxpt;
			constructMatrixFrmVector(pxpt, p);
			Vec3f r=((*NodePos)[i]-(*NodePos)[neighborNodeIdx[idx]])/SupportRadius;
			float radius=r.norm();
			momentDrvY[i]=momentDrvY[i]+pxpt*weightFuncDrvY(radius, WeightFuncType, r[1], SupportRadius);
		}
	}
}

void MLSShapeFunc::computeMomentMatrixDrvZ(Vec3f pos, Mat4x4f& momentDrvZ, int* neighborNodeIdx, int nbNeighbor)
{
	momentDrvZ.clear();
	for(unsigned int i=0;i<nbNeighbor;i++)
	{
		Vec4d p(1, (*NodePos)[neighborNodeIdx[i]][0], (*NodePos)[neighborNodeIdx[i]][1], (*NodePos)[neighborNodeIdx[i]][2]); 
		Mat4x4f pxpt;
		constructMatrixFrmVector(pxpt, p);
		Vec3f r=(pos-(*NodePos)[neighborNodeIdx[i]])/SupportRadius;
		float radius=r.norm();
		momentDrvZ=momentDrvZ+pxpt*weightFuncDrvZ(radius, WeightFuncType, r[2], SupportRadius);
	}
}

void MLSShapeFunc::computeMomentMatrixDrvZ(Vec3f pos, Mat4x4f& momentDrvZ, std::vector<int>& neighborNodeIdx)
{
	momentDrvZ.clear();
	for(unsigned int i=0;i<neighborNodeIdx.size();i++)
	{
		Vec4d p(1, (*NodePos)[neighborNodeIdx[i]][0], (*NodePos)[neighborNodeIdx[i]][1], (*NodePos)[neighborNodeIdx[i]][2]); 
		Mat4x4f pxpt;
		constructMatrixFrmVector(pxpt, p);
		Vec3f r=(pos-(*NodePos)[neighborNodeIdx[i]])/SupportRadius;
		float radius=r.norm();
		momentDrvZ=momentDrvZ+pxpt*weightFuncDrvZ(radius, WeightFuncType, r[2], SupportRadius);
	}
}

void MLSShapeFunc::computeMomentMatrixDrvZAtNode(int nodeIdx, Mat4x4f& momentDrvZ, int* neighborNodeIdx, int nbNeighbor)
{
	momentDrvZ.clear();
	for(unsigned int i=0;i<nbNeighbor;i++)
	{
		Vec4d p(1, (*NodePos)[neighborNodeIdx[i]][0], (*NodePos)[neighborNodeIdx[i]][1], (*NodePos)[neighborNodeIdx[i]][2]); 
		Mat4x4f pxpt;
		constructMatrixFrmVector(pxpt, p);
		Vec3f r=((*NodePos)[nodeIdx]-(*NodePos)[neighborNodeIdx[i]])/SupportRadius;
		float radius=r.norm();
		momentDrvZ=momentDrvZ+pxpt*weightFuncDrvZ(radius, WeightFuncType, r[2], SupportRadius);
	}
}

void MLSShapeFunc::computeMomentMatrixDrvZAtNode(int nodeIdx, Mat4x4f& momentDrvZ, std::vector<int>& neighborNodeIdx)
{
	momentDrvZ.clear();
	for(unsigned int i=0;i<neighborNodeIdx.size();i++)
	{
		Vec4d p(1, (*NodePos)[neighborNodeIdx[i]][0], (*NodePos)[neighborNodeIdx[i]][1], (*NodePos)[neighborNodeIdx[i]][2]); 
		Mat4x4f pxpt;
		constructMatrixFrmVector(pxpt, p);
		Vec3f r=((*NodePos)[nodeIdx]-(*NodePos)[neighborNodeIdx[i]])/SupportRadius;
		float radius=r.norm();
		momentDrvZ=momentDrvZ+pxpt*weightFuncDrvZ(radius, WeightFuncType, r[2], SupportRadius);
	}
}

void MLSShapeFunc::computeMomentMatrixDrvZAtNode(Mat4x4f* momentDrvZ, int* neighborNodeIdx, int* nbNeighborNode, int nbNeighborMax)
{
	for(int i=0;i<NodePos->size();i++)
	{
		for(unsigned int j=0;j<nbNeighborNode[i];j++)
		{
			int idx=nbNeighborMax*i+j;
			Vec4d p(1, (*NodePos)[neighborNodeIdx[idx]][0], (*NodePos)[neighborNodeIdx[idx]][1], (*NodePos)[neighborNodeIdx[idx]][2]); 
			Mat4x4f pxpt;
			constructMatrixFrmVector(pxpt, p);
			Vec3f r=((*NodePos)[i]-(*NodePos)[neighborNodeIdx[idx]])/SupportRadius;
			float radius=r.norm();
			momentDrvZ[i]=momentDrvZ[i]+pxpt*weightFuncDrvZ(radius, WeightFuncType, r[2], SupportRadius);
		}
	}
}

void MLSShapeFunc::constructMomentMatrix(Mat4x4f& moment, Vec3f pos, std::vector<int>& neighborNodeIdx)
{
	moment.clear();
	for(unsigned int i=0;i<neighborNodeIdx.size();i++)
	{
		Vec4d p(1, (*NodePos)[neighborNodeIdx[i]][0], (*NodePos)[neighborNodeIdx[i]][1], (*NodePos)[neighborNodeIdx[i]][2]); 
		Mat4x4f pxpt;
		constructMatrixFrmVector(pxpt, p);
		float r=(pos-(*NodePos)[neighborNodeIdx[i]]).norm()/SupportRadius;
		moment=moment+pxpt*weightFunc(r, WeightFuncType);
	}
}

void MLSShapeFunc::constructMomentMatrix(Mat4x4f& moment, int nodeIdx, std::vector<int>& neighborNodeIdx)
{
	moment.clear();
	Vec3f pos=(*NodePos)[nodeIdx];
	for(unsigned int i=0;i<neighborNodeIdx.size();i++)
	{
		Vec4d p(1, (*NodePos)[neighborNodeIdx[i]][0], (*NodePos)[neighborNodeIdx[i]][1], (*NodePos)[neighborNodeIdx[i]][2]); 
		Mat4x4f pxpt;
		constructMatrixFrmVector(pxpt, p);
		float r=(pos-(*NodePos)[neighborNodeIdx[i]]).norm()/SupportRadius;
		moment=moment+pxpt*weightFunc(r, WeightFuncType);
	}
}

void MLSShapeFunc::constructMomentMatrix(Mat4x4f& moment, Vec3f pos, int* neighborNodeIdx, int nbNeighborNode)
{
	moment.clear();

	for(unsigned int i=0;i<nbNeighborNode;i++)
	{
		Vec4d p(1, (*NodePos)[neighborNodeIdx[i]][0], (*NodePos)[neighborNodeIdx[i]][1], (*NodePos)[neighborNodeIdx[i]][2]); 
		Mat4x4f pxpt;
		constructMatrixFrmVector(pxpt, p);
		float r=(pos-(*NodePos)[neighborNodeIdx[i]]).norm()/SupportRadius;
		moment=moment+pxpt*weightFunc(r, WeightFuncType);
	}
}


float MLSShapeFunc::computeShapeFuncValue(int nodeIdx, Vec3f pos, Mat4x4f& momentInv)
{
	/* pai=pt(x)[M(x)]-1w(x-xi)p(xi) */
	float shapeFuncValue=0;

	// 1. P vector
	Vec4f pt(1, pos[0], pos[1], pos[2]);
	Vec4f p(1, (*NodePos)[nodeIdx][0], (*NodePos)[nodeIdx][1], (*NodePos)[nodeIdx][2]);

	// 2. Shape function value computation
	float r=(pos-(*NodePos)[nodeIdx]).norm()/SupportRadius;
	shapeFuncValue=pt*(momentInv*p)*weightFunc(r, WeightFuncType);
	return shapeFuncValue;
}

float MLSShapeFunc::computeShapeFuncDrvX(int nodeIdx, Vec3f pos, Mat4x4f& momentInv, Mat4x4f& momentDrv)
{
	float shapeFuncDrv=0;

	/* 1. pt,x*M-1*w(x-xi)*p(xi) */
	Vec4d dpdx(0, 1, 0, 0);
	Vec4d pt(1, pos[0], pos[1], pos[2]);
	Vec4d p(1, (*NodePos)[nodeIdx][0], (*NodePos)[nodeIdx][1], (*NodePos)[nodeIdx][2]);

	//1-1. Computation
	Vec3f r=(pos-(*NodePos)[nodeIdx])/SupportRadius;
	float radius=r.norm();
	shapeFuncDrv=dpdx*(momentInv*p)*weightFunc(radius, WeightFuncType);

	/* 2. pt*M-1,x*w(x-xi)*p(xi) */
	Mat4x4f momentInvDrv=momentInv*momentDrv*momentInv*-1;
	shapeFuncDrv+=pt*(momentInvDrv*p)*weightFunc(radius, WeightFuncType);

	/* 3. pt*M-1*w(x-xi),x*p(xi) */
	shapeFuncDrv+=pt*(momentInv*p)*weightFuncDrvX(radius, WeightFuncType, r[0], SupportRadius);
	return shapeFuncDrv;
}

float MLSShapeFunc::computeShapeFuncDrvY(int nodeIdx, Vec3f pos, Mat4x4f& momentInv, Mat4x4f& momentDrv)
{
	float shapeFuncDrv=0;

	/* 1. pt,x*M-1*w(x-xi)*p(xi) */
	Vec4d dpdx(0, 0, 1, 0);
	Vec4d pt(1, pos[0], pos[1], pos[2]);
	Vec4d p(1, (*NodePos)[nodeIdx][0], (*NodePos)[nodeIdx][1], (*NodePos)[nodeIdx][2]);

	//1-1. Computation
	Vec3f r=(pos-(*NodePos)[nodeIdx])/SupportRadius;
	float radius=r.norm();
	shapeFuncDrv=dpdx*(momentInv*p)*weightFunc(radius, WeightFuncType);

	/* 2. pt*M-1,x*w(x-xi)*p(xi) */
	Mat4x4f momentInvDrv=momentInv*momentDrv*momentInv*-1;
	shapeFuncDrv+=pt*(momentInvDrv*p)*weightFunc(radius, WeightFuncType);

	/* 3. pt*M-1*w(x-xi),x*p(xi) */
	shapeFuncDrv+=pt*(momentInv*p)*weightFuncDrvY(radius, WeightFuncType, r[1], SupportRadius);

	return shapeFuncDrv;
}

float MLSShapeFunc::computeShapeFuncDrvZ(int nodeIdx, Vec3f pos, Mat4x4f& momentInv, Mat4x4f& momentDrv)
{
	float shapeFuncDrv=0;

	/* 1. pt,x*M-1*w(x-xi)*p(xi) */
	Vec4d dpdx(0, 0, 0, 1);
	Vec4d pt(1, pos[0], pos[1], pos[2]);
	Vec4d p(1, (*NodePos)[nodeIdx][0], (*NodePos)[nodeIdx][1], (*NodePos)[nodeIdx][2]);

	//1-1. Computation
	Vec3f r=(pos-(*NodePos)[nodeIdx])/SupportRadius;
	float radius=r.norm();
	shapeFuncDrv=dpdx*(momentInv*p)*weightFunc(radius, WeightFuncType);

	/* 2. pt*M-1,x*w(x-xi)*p(xi) */
	Mat4x4f momentInvDrv=momentInv*momentDrv*momentInv*-1;
	shapeFuncDrv+=pt*(momentInvDrv*p)*weightFunc(radius, WeightFuncType);

	/* 3. pt*M-1*w(x-xi),x*p(xi) */
	shapeFuncDrv+=pt*(momentInv*p)*weightFuncDrvZ(radius, WeightFuncType, r[2], SupportRadius);

	return shapeFuncDrv;
}

void MLSShapeFunc::constructMatrixFrmVector(Mat4x4f& mat, Vec4d vec)
{
	for(int i=0;i<4;i++)
	{
		for(int j=0;j<4;j++)
			mat(i,j)=vec(i)*vec(j);
	}
}

void MLSShapeFunc::constructMatrixFrmVector(Mat4x4f& mat, Vec4d p, Vec4d pt)
{
	for(int i=0;i<4;i++)
	{
		for(int j=0;j<4;j++)
			mat(i,j)=p(i)*pt(j);
	}
}

//dw/dx=dw/dr*dr/dx=dw/dr*x/r
float MLSShapeFunc::weightFuncDrvX(float r, int funcType, float x, float support)
{
	float value=0;

	switch(funcType)
	{
	case SPLINE3:
		{
			if(r<=0.5)
				value=-8.0*r+12.0*r*r;
			else if(r<=1.0)
				value=-4.0+8*r-4*r*r;
			else
				value=0;
			break;
		}

	case SPLINE4:
		{
			if(r<=1.0)
				value=-12*r+24*r*r-12*r*r*r;
			else
				value=0;
			break;
		}
	}

	if(r<EPS)
		return 0;
	else
		return value*x/r/support;
}

//dw/dy=dw/dr*dr/dx=dw/dr*y/r
float MLSShapeFunc::weightFuncDrvY(float r, int funcType, float y, float support)
{
	float value=0;

	switch(funcType)
	{
	case SPLINE3:
		{
			if(r<=0.5)
				value=-8.0*r+12.0*r*r;
			else if(r<=1.0)
				value=-4.0+8*r-4*r*r;
			else
				value=0;
			break;
		}

	case SPLINE4:
		{
			if(r<=1.0)
				value=-12*r+24*r*r-12*r*r*r;
			else
				value=0;
			break;
		}
	}

	if(r<EPS)
		return 0;
	else
		return value*y/r/support;
}

//dw/dz=dw/dr*dr/dx=dw/dr*z/r
float MLSShapeFunc::weightFuncDrvZ(float r, int funcType, float z, float support)
{
	float value=0;

	switch(funcType)
	{
	case SPLINE3:
		{
			if(r<=0.5)
				value=-8.0*r+12.0*r*r;
			else if(r<=1.0)
				value=-4.0+8*r-4*r*r;
			else
				value=0;
			break;
		}

	case SPLINE4:
		{
			if(r<=1.0)
				value=-12*r+24*r*r-12*r*r*r;
			else
				value=0;
			break;
		}
	}
	if(r<EPS)
		return 0;
	else
		return value*z/r/support;
}

float MLSShapeFunc::weightFunc(float r, int funcType)
{
	float value=0;

	switch(funcType)
	{
	case SPLINE3:
		{
			if(r<=0.5)
				value=2.0/3.0-4.0*r*r+4.0*r*r*r;
			else if(r<=1.0)
				value=4.0/3.0-4.0*r+4.0*r*r-4.0/3.0*r*r*r;
			else
				value=0;
			break;
		}

	case SPLINE4:
		{
			if(r<=1.0)
				value=1.0-6.0*r*r+8.0*r*r*r-3.0*r*r*r*r;
			else
				value=0;
			break;
		}

	case EXPONENTIAL:
		{
			if(r<=1.0)
				value=exp(-(r/0.4)*(r/0.4));
			else
				value=0;
			break;
		}
	}
	return value;
}