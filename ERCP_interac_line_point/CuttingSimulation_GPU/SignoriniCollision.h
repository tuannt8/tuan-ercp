#ifndef SIGNORINI_COLLISION_H
#define SIGNORINI_COLLISION_H

#include "Modules/Meshfree_GPU.h"
#include "DataTypes/newmat/newmat.h"
#include "DataTypes/Vec.h"
#include "DataTypes/Mat.h"
#include "DataTypes/Define.h"
#include "DataTypes/sparsematrix.h"
#include "MyFFD.h"
#include <afxwin.h>
#include <GL/glu.h>
#include <GL/gl.h>
#include <vector>
#include <math.h>
#include <stdio.h>
#include <float.h>
#include "stdafx.h"


#include <omp.h>
#include <time.h>

#include <stdlib.h>

#include <ostream>



//-----------------------------------------------------------------------------------------------------

using namespace NEWMAT;
using namespace arma;

class SignoriniCollision
{
public:

	typedef struct 
	{  	
		int	  triIdx;
		Vec3d p1;
		Vec3d p2;
		Vec3d p3;
	} Triangle;

	typedef struct
	{
		int cylinderIdx;
		Vec3d projectedPointOnCenterline;
		Vec3d penetration;
		Vec3d OnSurface;
		Vec3d collidedPos;
		Vec3d collidedPosInCylinder;
		Vec3i tri;
		int tristart;
		int triend;
		int triIdx;
		int nodeIdx;
		int CollidedSurfacePointIdx;
		int Case;
		int NbCollision;
	} CollisionInfo;

	typedef struct  
	{
		int cylinderIdx;
		int triIdx;
		int trivertexIdx;
		Vec3d measuredPointInCylinder;
		Vec3d collidedCylPoint;
		Vec3d collidedTriPoint;
		double Distance;
		Vec3d PenetratedDirec;
		double Penetration;
	} Distancefield;

	typedef struct
	{
		int cylinderIdx;
		Vec3d projectedPointOnCenterline;
		Vec3d penetration;
		Vec3d OnSurface;
		Vec3d collidedPos;
		Vec3d collidedPosInCylinder;
		Vec3i tri;
		int tristart;
		int triend;
		int triIdx;
		int nodeIdx;
		int Case;
		int NbCollision;
	} MyCollisionInfo;

	typedef struct
	{
		int nodeIdx;
		int nodeIdx1;
		int nodeIdx2;
		double penetration;
		Vec3d testforce;
		double test;
		double test2; 
		double test3;
	} SimpleCollision;

	typedef struct  
	{
		int ControlIdx;
		double weighting;
		Vec3d penetration;
	} SecondCollision;

public:
	SignoriniCollision(void);
	~SignoriniCollision(void);

	//functions
public:	

	void testInverse();
	void testInverse(char* filename,double** A,int Nb);
	void testInverse(double** A,int Nb);
	void drawVec(Vec3d p1, Vec3d p2);
	void drawCollisionInfo();



	std::vector<Distancefield> getDistance(){return distance;};
	std::vector<Distancefield>* getDistanceAddress(){return &distance;};

	//------------------------------------------------------------------------------------------

	bool CollisionDetectionUsingDistanceTest(MyFFD* surf,MyFFD* cont);
	bool CollisionDetectionUsingDistanceEndoscope(MyFFD* surf,MyFFD* cont);
	bool CollisionDetectionUsingDistanceEndoscopeV2(MyFFD* surf,MyFFD* cont);
	bool LatticeBasedCollisionDetectionUsingDistanceEndoscope(MyFFD* surf,MyFFD* cont);
	bool SimpleLatticeBasedCollisionDetectionUsingDistanceEndoscope(MyFFD* surf,MyFFD* cont);

	bool PotentialCollisionDetection(MyFFD* surf,MyFFD* cont);
	bool PotentialCollisionDetection(MyFFD* surf,MyFFD* cont,int Margin, double boundary);
	bool PotentialCollisionDetectionV2(MyFFD* surf,MyFFD* cont,int Margin, double boundary);
	bool PotentialCollisionDetectionV3(MyFFD* surf,MyFFD* cont,int Margin, double boundary);


	bool CollisionDetection(MyFFD* surf,MyFFD* cont);

private:

	double projectOnLine(Vec3d r0, Vec3d direc, Vec3d P);
	double projectOnPlane(Vec3d Normal,Vec3d Center,Vec3d Point);


	/* Cylinder triangle collision detection (New version) */
	

	Vec3d	computeFaceNormal(Triangle& tri);
	double	computeDistanceBetweenTwoline(Vec3d p1, Vec3d p2, Vec3d p3, Vec3d p4);

	Vec3d	findPointBetweenTwoline1(Vec3d p1, Vec3d p2, Vec3d p3, Vec3d p4);
	Vec3d	findPointBetweenTwoline2(Vec3d p1, Vec3d p2, Vec3d p3, Vec3d p4);
	Vec3d   findProjectedPointToPlane(Vec3d Point, Vec3d Normal, Vec3d Center);
	Vec3d   findClosestPointInTri(Vec3d point, Vec3d tri1, Vec3d tri2, Vec3d tri3);

public:
	Vec3d P[2];
	int idx[2];

	//=======================================
	int size;
	Vec3i* ii;
	Vec3i* jj;
	int* newii;
	int* newjj;
	double** newB;
	double** InverseB;
	double** newBB;
	Vec3d* dP;
	int* DuplicatedPoint;
	double** Mapping;
	int NbcollisionInfo;


	std::vector<Distancefield> distance;


	//==================================================

	void PenaltyMethod(MyFFD* surf, MyFFD* cont, double K);


	void ComputeforcefromComplianceV19(MyFFD* surf,MyFFD* cont);
	void ComputeforcefromComplianceV20(MyFFD* surf,MyFFD* cont);
	void ComputeforcefromComplianceV21(MyFFD* surf,MyFFD* cont);

	void InteractionSimulation(MyFFD* surf,MyFFD* endo, MyFFD* cath);
	void InteractionSimulationV2(MyFFD* surf,MyFFD* endo, MyFFD* cath);
	void InteractionSimulationV3(MyFFD* surf,MyFFD* endo, MyFFD* cath);
	void InteractionSimulationV4(MyFFD* surf,MyFFD* endo, MyFFD* cath);
	void InteractionSimulationV5(MyFFD* surf,MyFFD* endo, MyFFD* cath);
	void InteractionSimulationV6(MyFFD* surf,MyFFD* endo);
	//
	void interactionSimulation(MyFFD* _catheter, Meshfree_GPU* object, float dt, int itter);
	bool CollisionDetectionBtwCatheterAndMeshles( MyFFD* _catheter, Meshfree_GPU* object, double marginRatio );
	bool CollisionDetectionBtw1CatheterAndMeshles( MyFFD* _catheter, Meshfree_GPU* object, double marginRatio );

	int PotentialCollideSegment( MyFFD* _catheter, Meshfree_GPU* object );

	void ComputeforceImplicit(MyFFD* surf,MyFFD* cont);

	void ComputeforcefromComplianceForExplicit(MyFFD* surf,MyFFD* cont);

	void ComputeforceUsingPenalty(MyFFD* surf,MyFFD* cont,double K);


	void SetVariableforLCP(int Nb1,int Nb2,int _NbCollide);
	void SetVariableforLCP(MyFFD* surf,MyFFD* cont);

	//==========================Lattice based Collsiion
	void SetLatticebasedCollision(MyFFD* surf);
	std::vector<int>* getTriIndex(){return TriIndex;};
	std::vector<int> getTriInHyper(){return TriInHyper;};
	std::vector<int> getInspectedTri(){return InspectedTri;};

	//Sparse matrix
	SparseMatrix convertMatrixtoSparse(mat A);
	mat convertSparsetoMatrix(SparseMatrix SparseA);

private:


	//================================================================================

	
	void measureMinimumDistanceBetLineandTriangle(Vec3d p1, Vec3d p2,Vec3d tri1, Vec3d tri2, Vec3d tri3, Vec3d &collidedTri, Vec3d &collidedCyl);
	void measureMinimumDistanceBetPointandTriangle(Vec3d p1, Vec3d tri1, Vec3d tri2, Vec3d tri3, Vec3d &collidedTri, Vec3d &collidedCyl);

	mat GaussSeidelMethod1(mat mcm,mat penetration,int Iteration);
	mat GaussSeidelMethod2(mat mcm,mat penetration,double precision);


	







	int NbCollide;
	double* MC1;
	double* MC2;
	double** MCM;
	double* delta;
	double* Force;
	double** M1;
	double** M2;


	Vec<12,double>* ReducedM1;
	Vec2d* ReducedM2;

	double*** Caddress1;
	double*** Caddress2;
	double* Maddress1;
	double* Maddress2;



	//Lattice based collision
	std::vector<int>* TriIndex;
	std::vector<int> TriInHyper;
	std::vector<int> InspectedTri;


	FILE* fp;
	FILE* Delta;


	CTimeTick timeTick;
	//	void calculateInverse(double** A,int Nb);
	/*int NB;
	double*** Compliance;*/
	
};

#endif