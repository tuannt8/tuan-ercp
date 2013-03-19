#ifndef CUTTING_TOOL
#define CUTTING_TOOL

#include "DataTypes/Define.h"
#include "DataTypes/Vec.h"
#include "DataTypes/Mat.h"
#include "Modules/TimeTick.h"
#include "DataTypes/geometricfunc.h"
#include "Graphics/TopologyContainer.h"
#include "Modules/AABBTri.h"
#include <afxwin.h>
#include <GL/glu.h>
#include <GL/gl.h>
#include <vector>

class CuttingTool
{
public:
	CuttingTool(void);
	~CuttingTool(void);

//functions
public:
	void setCutFront(std::vector<Vec3f>* cutFront);
	void trans(Vec3f _trans);
	void transTotal(Vec3f _trans);
	void drawCutSurf(Vec3f color, int mode);
	void drawSurf(Vec3f color, int mode);
	void drawSurfTotal(Vec3f color, int mode);
	void drawCutFront(float radius);
	void updateToolPos();
	void updateToolPosTotal();

	// Examples
	void initEx1();
	void initEx2();
	void initEx3();
	void initEx4();
	void initEx5();

	// Get data
	std::vector<Vec3f>* point();
	std::vector<Vec3i>* face();
	std::vector<Vec3f>* normal();

	std::vector<Vec3f>* cutPoint();
	std::vector<Vec3i>* cutFace();
	std::vector<Vec3f>* cutFaceNormal();

	std::vector<Vec3f>* pointTotal();
	std::vector<Vec3i>* faceTotal();
	std::vector<Vec3f>* normalTotal();

	void getCutFrontIdx(std::vector<int>& idx);
	void getCutFrontIdxLocal(std::vector<int>& idx);

	int nbCutFront(){return CutFront.size();};
	int nbToolPath(){return ToolPath.size();};

	TopologyContainer* container(){return Container;};
	AABBTree* getBVH(){return BVHAABB;}; 

private:
	void generateFace();
	void computeCutFaceNormal();

//variables
public:
	std::vector<Vec3f> CutFront;
	std::vector<Vec3f> CutEnd;
	std::vector<Vec3f> CutPoint;
	std::vector<Vec3i> CutFace;
	std::vector<Vec3f> CutFaceNormal;

	std::vector<Vec3f> Point;
	std::vector<Vec3i> Face;
	std::vector<Vec3f> Normal;

	std::vector<Vec3f> PointTotal;
	std::vector<Vec3i> FaceTotal;
	std::vector<Vec3f> NormalTotal;
	std::vector<Vec2i> EdgeTotal;
	std::vector<Vec3f> ToolPath;

	TopologyContainer* Container;
	AABBTree* BVHAABB; 
};

#endif