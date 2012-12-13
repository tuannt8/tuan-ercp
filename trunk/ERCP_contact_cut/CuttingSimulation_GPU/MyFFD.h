#ifndef MYFFD_H
#define MYFFD_H

#include "DataTypes/Vec.h"
#include "DataTypes/Mat.h"
#include "DataTypes/Define.h"
#include "DataTypes/reducedmatrix.h"
#include <afxwin.h>
#include <GL/glu.h>
#include <GL/gl.h>
#include "Modules/SpringForce.h"
#include <math.h>
#include "DataTypes/Newmat/Newmat.h"
#include <iostream>

#include <omp.h>


#define ABS(x) (x < 0 ? -(x) : (x))

#define PENALTY_CONSTANT 300000



using namespace NEWMAT;

using namespace arma;
using namespace std;

class MyFFD
{
public:
	MyFFD(void);
	~MyFFD(void);

	//functions
public:	
	/* initialization */
	void init(int _NbPoint, Vec3d* _ControlPoint, Vec3d* _surfCP);

	/* ControlPoint load */
	void loadControlPoint(char* filename);
	void writeControlPoint(char* filename);

	//Load
	void loadParameter(char* filename);
	void loadParameterVar(char* filename);
	void loadSurfaceVar(char* filename);
	void loadStomachModel(char* filename);
	void loadTextureCoordinate(char* filename);
	void loadConstraints(char* filename);

	//Write
	void writeStomachModel(char* filename);

	//Textures
	mat texCoord;
	mat FaceofTex;

	void updateSurfacePoint();
	void updateSurfacePointusingMappingMatrix();
	


	//Make
	void makeSquareFFD(Vec3d Center, int a, int b, int c, double width1, double width2, double height);
	void makePlaneFFD(Vec3d Center, int a, int b, double width1, double width2);
	void makeLineFFD(Vec3d Center, Vec3d direc, int nb);
	void makeEndoscope(Vec3d Center, Vec3d direc,double elementLength,double radius, int nb);
	void makeCenterlinebasedSurface(int NbV,int NbW,int NbCen,double radius, Vec3d startpoint,Vec3d endpoint);
	void makeCenterlinebasedSurface(int NbV,int NbW,int NbCen,double radius, Vec3d startpoint,double length);
	void makeCenterlinebasedSurface(char* filename,int NbV,int NbW,double radius);
	void makeUppergastrointestinalSurface(char* filename,int NbV,int NbW,double radius);
	void makeUppergastrointestinalSurfaceV2(char* filename,int NbV,int NbW,double radius);
	void makeUppergastrointestinalSurfaceV3(char* filename,int NbV,int NbW,double radius);


	//Mapping
	mat returnMappingMatforOneSurface(int triIdx,Vec3d Pos);
	mat returnMappingMatforOneLattice(int idx1, int idx2, Vec3d pos);

	mat returnMappingMatforOneSurface(int triIdx,Vec3d Pos,Vec3d normal);
	std::vector<int> returnMappingMatIndexforOneSurface(int triIdx,Vec3d Pos);
	mat returnMappingMatforOneLattice(int idx1, Vec3d pos, Vec3d normal);
	mat returnSparseMappingMatforSurface(int triIdx,Vec3d Pos,Vec3d normal);
	mat returnSparseMappingMatforEndoscope(int idx1, Vec3d pos, Vec3d normal);

	
	Vec<12,double>  returnReducedMappingMatrixforSurface(int index){return Mapping[index];};
	Vec2d returnReducedMappingMatrixforLattice(int idx1, Vec3d pos);

	double returnRatioOfArea(int triIdx,Vec3d Pos, int index);

	//Load
	void loadCenterlineBasedCP(char* filename);

	/* Drawing */
	void drawControlPoint(double radius, double lineWidth);
	void drawControlPoint0(double radius, double lineWidth);
	void drawWire(double lineWidth);
	void drawConstraintPoint(double radius);
	void drawPoint(Vec3d Point, double radius);
	void drawControlPoint(int Idx, double radius);
	void drawPlane(double lineWidth,int a,int b);
	void drawCenterPoint(int Idx, double radius);
	void drawSurface();
	void drawSurfaceTri();
	void drawSmoothSurface();
	void drawTexturedSurface();
	void drawSurfacePoint(int Idx, double radius);
	void drawTri(int idx);
	void drawPenetrationAtSurface(int Idx,Vec3d penetration,double lineWidth);
	void drawPenetrationAtControl(int Idx,Vec3d penetration,double lineWidth);
	void drawLine(Vec3d P1,Vec3d P2,double lineWidth);
	void drawCylinder(double radius);
	void drawCylinder( Vec3f a, Vec3f b, float radius,  float radius2=0 );
	void drawCylinder( Vec3f b, Vec3f a, float radius , double colorPercent, Vec3f color, float radius2=0);
	void drawControlGroup(int centerIdx, int circualrIdx,double radius,double lineWidth);
	void drawHyperPatchLine(int centerIdx, int circularIdx);
	void drawHyperPatchLine();
	void drawHyperPatch(int centerIdx, int circularIdx);
	void drawHyperPatch();
	void drawBendingAxis();
	void drawForce();
	void drawEndoscope();
	void drawFaceNormalVector(double LineWidth);
	void drawCatheter(int mode);

	void GLPrint(const char* text);
	void printNodeNumber();


	//Around Point
	void computeAroundPoint();

	/* transformation */
	void translate(Vec3d trans);
	void translate(double x, double y, double z);
	void rotate(Vec3d axis, double angle);
	void rotate(double* rot);
	void rotate(Mat3x3d rot);
	void scale(float x, float y, float z);
	void scale(float scale);

	/* get data */

	Vec3d* controlPoint(){return ControlPoint;};


	Vec3d paraToSurfacePoint(int centerIdx, int circualrIdx, double u, double v, double w);
	Vec3d computeNormal(Vec3d pos1, Vec3d pos2, Vec3d pos3);
	void computeControlPointNormal();

	/* move control point */
	void resetControlPoints();

	//Mass spring
	void setMassSpringDamper(double mass, double ks, double kd,double ed,double _dt);
	void updatePosition(double dt);
	void updatePosition(double dt,bool gravity);
	void updatePositionWithForce(double dt,Vec3d Gravity);
	void updatePositionImplicit(double dt,Vec3d Gravity);
	void updatePositionFastImplicit(double dt,Vec3d Gravity);
	void updatePositionWithBendingAndForce(double dt,Vec3d Gravity);
	void updatePositionUsingStiffness(double dt,bool gravity);
	void updatePositionUsingStiffness(double dt,Vec3d Gravity);
	void updatePositionExplicit(double dt,Vec3d Gravity);
	
	void updateUpperImplicit(double dt,Vec3d Gravity);
	void updateUpperExplicit(double dt,Vec3d Gravity);

	void makePlaneConstraints(int a, int b);

	//Constraint
	void addFixedConstraint(int idx);
	void addElasticConstraint(int idx, double K);
	void applyElasticConstraintForce();
	

	//FEM
	void	makeStiffnessWithoutPenalty();
	void    makeStiffness();
	void    makeBendingStiffness();
	void	makeLocalBendingstiffness1(Vec3d p1, Vec3d p2, Vec3d p3, double K);
	void    makeLocalBendingstiffness2(Vec3d p1, Vec3d p2, Vec3d p3, double K);
	void    makeBendingDamping();
	void	makeLocalBendingdamping1(Vec3d p1, Vec3d p2, Vec3d p3, double C);
	void    makeLocalBendingdamping2(Vec3d p1, Vec3d p2, Vec3d p3, double C);
	void	makeMassmatrix();
	void	makeMassmatrix2();
	void    makeDamping();
	void	makeCompliancematrix();
	void	makeCompliancematrix2();
	double** returnStiffness(){return Kglobal;};
	double** returnMassmatrix(){return Mglobal;};
	double** returnCompliancematrix(){makeCompliancematrix();return Compliance;};
	double** returnCompliancematrix2(){makeCompliancematrix2();return Compliance;};
	double*** returnComplianceaddress(){makeCompliancematrix();return &Compliance;};
	double*** returnComplianceaddress2(){makeCompliancematrix2();return &Compliance;};

	mat  returnImplicitCompliancematrix();
	mat  returnExplicitCompliancematrix();
	mat  returnImplicitCompliancematrixForUpper();
	mat  returnExplicitCompliancematrixForUpper();

	mat returnExplicitDynamicStiffnessMatrixForUpper();
	mat returnImplicitDynamicStiffnessMatrixForUpper();
	mat returnVertorFormExplicitCompliancematrixForUpper();

	mat returnExplicitDynamicStiffnessMatrixForFFD();

	mat* returnaddressInverseCompliance();
	mat returnInverseCompliance();

	void applyMassAndConstraint(mat &mapping);
	

	//return
	Vec3d*	 returnControlPoint(){return ControlPoint;};
	int		 returnNbPoint(){return NbPoint;};
	Vec3d*	 returnSurfacePoint(){return SurfacePoint;};
	Vec3d*   returnCenterPoint(){return CenterPoint;};
	Vec3d*	 returnSurfacePointPara(){return SurfacePointPara;};
	Vec2i*   returnSurfacePointParaIdx(){return SurfacePointParaIdx;};
	Vec3i*   returnFace(){return Face;};
	Vec3d*   returnForce(){return Force;};
	int*	 returnFaceIndex(){return FaceIndex;};
	int		 returnNbFace(){return NbFace;};
	int		 returnNbSurface(){return NbSurfacePoint;};
	int		 returnNbCenter(){return NbCenterPoint;};
	int		 returnNbCircular(){return NbCircularPoint;};

	int closestCircularPointIdx(int AxisIdx,Vec3d Point);
	int closestCenterPointIdx(Vec3d Point);
	int closestControlCenterPointIdx(Vec3d Point);
	void	 moveControlPoint(int Idx,Vec3d trans);



	void	setFEM(double _M, double _K,double _C,double _Ed, double _beta, double _theta, double _dt);
	void	updateFEM();
	void	updateExplicitFEM(double deltat);
	void	updateExplicitFEM2(double deltat);
	void	updateExplicitFEM3(double deltat);
	void	newmark(double beta, double theta, double dt);

	void addForcetoPoint(int idx,Vec3d force);
	void addForcetoAllPoint(Vec3d force);
	void addForcetoAllPoint(mat force);
	void addForcetoSurfacePoint(int idx,Vec3d force);
	void addForcetoSurfacePoint(int triIdx, int trivertexIdx, Vec3d force);
	void addForcetoSurfacePoint(int triIdx, Vec3d Pos, Vec3d force);
	void addForcetoSurfaceTri(int triIdx, Vec3d Pos, Vec3d force);
	void addForcetoControlPoint(mat force);
	void addForcetoControlPoint(int triIdx, Vec3d Pos, Vec3d force);
	void addForceBetweenPoint(int idx1,int idx2,Vec3d pos,Vec3d force);
	void addTorqueToPoint(Vec3d direc,double Torque,int Idx);
	void addBendingForce();
	void addBendingForce2();
	void setInitialVelocity(int Idx,Vec3d Vel);


	//Offline
	void setOffline(int nbtimestep);
	void updateOffline(int timestep);
	void updateOfflinefordraw(int timestep);

	//Mapping
	void makeMappingMatrix();
	Vec<12,double>* returnMapping(){return Mapping;};
	mat returnSurfaceMappingIndex(){return MappingIndex;};


	//Math
	bool GSolve(double** a,int n,double* &x);
	bool GSolveParallel(double** a,int n,double* &x);
	bool TridiagonalSolver(double** a,int n, int m, double* &x);

	//Endoscope
	void setMassSpringEndoscope(double mass, double ks, double kd, double kb, double cb, double ed, double _dt);
	void makeEndoscopeStiffness(bool Constraint);
	void makeEndoscopeDamping();
	void makeEndoscopeCompliance(bool Constraint);
	void makeEndoscopeCompliance2();
	double*** returnEndoscopeComplianceaddress(bool flag){makeEndoscopeCompliance(flag);return &Compliance;};
	double*** returnEndoscopeComplianceaddress(){makeEndoscopeCompliance2();return &Compliance;};
	void InsertEndoscope(double dv);
	void ControlEndsocope(double dx,double dy);
	void ControlEndsocopeByDisplacement(double Angle1,double Angle2);
	void HoldEndoscope();
	void ReleaseEndoscope();
	void updateEndoscopeImplicit(double dt,Vec3d Gravity);
	double returnRadiusOfEndoscope(){return RadiusOfEndoscope;};
	Vec3d returnInsertedDirec(){return InsertedDirec;};
	Vec3d returnInsertedPoint(){return InsertedPoint;};
	void updateEndoscopeFastImplicit(double dt,Vec3d Gravity);

	mat returnImplicitCompliancematrixForEndoscope();
	mat returnExplicitCompliancematrixForEndoscope();
	mat returnVectorFormExplicitCompliancematrixForEndoscope();

	void updateEndoscopeExplicit(double dt,Vec3d Gravity);
	void updateCatheterExplicit(Vec3d gravity, mat* contactForce = NULL);

	mat returnEndoscopeTipDisplacement(){return EndoscopeTipDisplacement;EndoscopeTipDisplacement.fill(0.0);};

	mat returnImplicitDynamicStiffnessMatrixForEndoscope();

	void rotateEndoscopeCam(double angle);

	//Catheter
	void makeCatheter(double elementLength,double radius, int nb);
	void updateCatheterInsertionPosition(Vec3d Pos, Vec3d Direc);
	void rotateCatheter(Vec3d basis, double angle);
	void updateCatheter(Vec3d Pos, Vec3d Direc, Vec3d basis, double angle);

	void moveCatheter(Vec3d distance);
	void addForce(double force);

	//ETC
	int findCloestCenterPointIndex(Vec3d Pos);
	int findCloestCircularPointIndex(Vec3d Pos);

	//
	mat returnInternalImplicitForceOfFFD();
	mat returnInternalExplicitForceOfFFD();
	mat returnInternalImplicitForceOfEndoscope();
	mat returnInternalExplicitForceOfEndoscope();
	mat returnInternalImplicitForceOfUpper();
	mat returnInternalExplicitForceOfUpper();

	mat returnInternalImplicitDisplacementOfFFD(Vec3d Gravity);
	mat returnInternalImplicitDisplacementOfEndoscope(Vec3d Gravity);
	mat returnInternalExplicitDisplacementOfFFD(Vec3d Gravity);
	mat returnInternalExplicitDisplacementOfEndoscope(Vec3d Gravity);
	mat returnInternalExplicitDisplacementOfUpper(Vec3d Gravity);
	void addDisplacementToControlPoint(mat Dis);

	mat returnPreDisplacement();


	//anatomical constraints

	

	//variables
private:
	void computeNormalVector();
	void computeRotationMatrix(Vec3d axis, double angle, Mat3x3d& rot);

	//FEM
	void makeLocalstiffness(Vec3d p1, Vec3d p2, double K);
	void makeLocalmass(Vec3d p1, Vec3d p2, double M);
	void makeLocalmass2(Vec3d p1, Vec3d p2, double M);
	
	
	
	void matrixClear(int n, double** a);
	void matrixClear(int n, double* a);

	//===Library »ç¿ë========
	void makeStiffnessMatrixWithoutPenalty();
	void makeStiffnessMatrix();
	void makeStiffnessMatrixForUpper(bool Constraint);
	void makeStiffnessMatrixWithoutPenaltyForUpper();
	void makeDampingMatrixForUpper();
	void makeBendingStiffnessMatrix();
	void makeDampingMatrix();
	void makeBendingDampingMatrix();
	void makeDiagonalMassmatrix();
	void makeLocalStiffnessMatrix(Vec3d p1, Vec3d p2, double K);
	void makeLocalBendingStiffnessMatrix1(Vec3d p1, Vec3d p2, Vec3d p3, double K);
	void makeLocalBendingStiffnessMatrix2(Vec3d p1, Vec3d p2, Vec3d p3, double K);
	void makeLocalBendingDampingMatrix1(Vec3d p1, Vec3d p2, Vec3d p3, double C);
	void makeLocalBendingDampingMatrix2(Vec3d p1, Vec3d p2, Vec3d p3, double K);
	void makeEndoscopeStiffnessMatrix(bool Constraint);
	void makeEndoscopeDampingMatrix();
	




	//Variable
	

	FILE* checkDis;


	//parameter
	int NbCenterPoint;
	Vec3d* CenterPointPara;
	Vec2i* CenterPointParaIdx;

	int NbCircularPoint;
	int* NbCircularPointVar;
	Vec3d* SurfacePointPara;
	Vec2i* SurfacePointParaIdx;

	//Surface
	int NbSurfacePoint;

	Vec3d* SurfacePoint;
	Vec3d* CenterPoint;
	
	int NbFace;
	Vec3i* Face;
	int* FaceIndex;
	

	
	/* control point */
	
	Vec3d*  ControlPoint;
	Vec3d*	PreControlPoint;
	Vec3d*	ControlPoint0;
	Vec3d** ControlPointOffline;
	

	Vec3d*	Velocity;
	Vec2i*	Con;
	Vec3d*	Force;
//	Vec3d*  Displacement;
	double* x;
	

	int		NbPoint,NbCon,Nbtimestep;
	double  Mass,Ed,Ks,Kd;
	SpringForce m_Spring;

	//mapping

	Vec<12,double>* Mapping;
	mat MappingMatrix;
	mat MappingIndex;

	//FEM
	double** Mglobal;
	double** Kglobal;
	double** Cglobal;
	double** Compliance;
	double** local;
	double** local2;
	
	mat MassMatrix;
	mat DiagonalMass;
	mat DampingMatrix;
	mat StiffnessMatrix;
	mat Local;
	mat Displacement;
	mat Connectivity;

	Rmat ReducedMass;
	Rmat ReducedDamping;
	Rmat ReducedStiffness;

	mat PrePos;


	
	double* F;
	double* PreF;

	double M,K,Kb,C,Cb,Ed2;

	//Coefficient
	double beta;
	double theta;
	double dt;

	//Centerline based
	int AxisCPNum;
	int SurfCPNum;
	


	// Fixed constraint
	std::vector<int> FixedConstraint;
	std::vector<Vec2d> ElasticConstraint;
	
	double** MappingMatrixforLattice;

	// Endoscope

	Vec3d InsertedPoint;
	Vec3d InsertedDirec;
	double RadiusOfEndoscope;
	mat EndoscopeTipDisplacement;

	//Around Point
	std::vector<int>* AroundPoint;


	//Check

	FILE* fp;

public:
	//Catheter --------------------------- Tuan added
	double L0;
	Vec3d normF;

private:

};

#endif