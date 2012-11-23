#ifndef SURFACE_OBJ
#define SURFACE_OBJ

#include "./DataTypes/Define.h"
#include "./DataTypes/Vec.h"
#include "./DataTypes/Mat.h"
#include "./DataTypes/Quat.h"
#include "TopologyContainer.h"
#include "TopologyModifier.h"
#include "./Modules/TimeTick.h"
#include "./Modules/AABB.h"
#include "./Modules/AABBTri.h"
#include <afxwin.h>
#include <GL/glu.h>
#include <GL/gl.h>
#include <vector>

class SurfaceObj
{
public:
	SurfaceObj(void);
	~SurfaceObj(void);
	
//functions
public:
	void readObjData(char* filename);
	void init(Vec3f* point, int nbPoint, Vec3i* face, int nbFace);
	void writeObjData(char* filename);
	
	//Drawings
	void drawObject(Vec3f color);
	void drawWireFrame(Vec3f color);
	void drawPointsAroundPoint(int idx);
	void drawEdgesAroundPoint(int idx);
	void drawFacesAroundPoint(int idx);
	void drawFacesAroundEdge(int idx);
	void drawEdgesInFace(int idx);
	void drawPoints();
	void drawPointIdx();
	void drawFace(int idx);
	void drawBVH(int depth);
	void drawBVH();

	void updatePoint();
	void updateNormal();
	void updateBVH();

	//Increase the resolution
	void increaseResolution();

	//Bounding volume hierarchy
	void constructAABBTree();

	//get values
	std::vector<Vec3f>*	point0();
	std::vector<Vec3f>*	point();
	std::vector<Vec3f>*	dis();
	std::vector<Vec3i>*	face();
	std::vector<Vec2i>*	edge();
	std::vector<Vec3f>*	faceNormal();
	std::vector<Vec3f>* pointNormal();
	Vec3f   midPoint();
	AABBTree* getBVH();
	int nbPoint();
	int nbFace();

	TopologyContainer* container();
	TopologyModifier* modifier();

	void computeCenterPoint();
	void computeRotationMatrix(Vec3f axis, float angle, Mat3x3f& rot);
	void moveToCenter();
	void translateI(float x, float y, float z);
	void translateI(Vec3f trans);
	void translate(float x, float y, float z);
	void translateC(Vec3f trans);
	void translate(Vec3f trans);
	void scaleI(float scale);
	void scaleI(float x, float y, float z);
	void scaleC(float x, float y, float z);
	void scaleC(float scale);
	void scale(float scale);
	void scale(float x, float y, float z);
	void rotate(Mat3x3f& rot);
	void rotateC(Mat3x3f rot);
	void rotateC(Quat q);
	void rotate(Quat q);
	void rotate(float* rot);
	void rotateC(Vec3f axis, float angle);
	void rotate(Vec3f axis, float angle);

private:
	void computeFaceNormal();
	Vec3f computeFaceNormal(Vec3f p1, Vec3f p2, Vec3f p3);
	void computePointNormal();

//variables
public:
	FILE* FTime;
	std::vector<Vec3f>	Point0;
	std::vector<Vec3f>	Point;
	std::vector<Vec3f>	Dis;
	std::vector<Vec3i>	Face;
	std::vector<Vec2i>	Edge;
	std::vector<Vec3f>	FaceNormal;
	std::vector<Vec3f>  PointNormal;
	Vec3f   MidPoint;

	TopologyContainer* Container;
	TopologyModifier* Modifier;
	AABBTreeTri* BVHAABB; 
};

#endif