#ifndef SURFACE_CUTTING_MANAGER
#define SURFACE_CUTTING_MANAGER

#include "DataTypes/Define.h"
#include "DataTypes/Vec.h"
#include "DataTypes/Mat.h"
#include "Modules/CollisionManager.h"
#include "Graphics/SurfaceObj.h"
#include "Modules/TimeTick.h"
#include "Modules/ReTriangulation.h"
#include "Modules/CuttingTool.h"
#include <afxwin.h>
#include <GL/glu.h>
#include <GL/gl.h>
#include <vector>

class SurfaceCuttingManager
{

public:
	typedef struct
	{
		int faceIdx;
		Vec3i face;
		std::vector<int> pointsOnEdgeNormalDirec;
		std::vector<int> pointsOnEdgeCNormalDirec;
		std::vector<int> pointsOnFaceNormalDirec;
		std::vector<int> pointsOnFaceCNormalDirec;
		std::vector<Vec3f> pointsOnEdgeNormal;
		std::vector<Vec3f> pointsOnFaceNormal;
		std::vector<int> cutEdgeIdx;

		// progress cutting
		int vtxIdxOnCutFrontNormal;
		int vtxIdxOnCutFrontCNormal;
	} CutFace;

public:
	SurfaceCuttingManager(void);
	~SurfaceCuttingManager(void);

//functions
public:
	void cutting(SurfaceObj* obj, std::vector<Vec3f>* cutPoint, std::vector<Vec3i>* cutFace, std::vector<Vec3f>* cutFaceNormal,  std::vector<int>* pointsOnCutFront);
	void cutting(SurfaceObj* obj, CuttingTool* tool, std::vector<int>* pointsOnCutFront);
	void cuttingProg(SurfaceObj* obj, CuttingTool* tool, int toolPathIdx);

	// Get data
	std::vector<Vec3f>* addedPoint(){return &AddedPoint;};
	std::vector<Vec3f>* addedPointNormal(){return &AddedPointNormal;};
	std::vector<int>* vertexIdxOnCutFront(){return &VertexIdxOnCutFront;};
	int preNbPoint(){return PreNbPoint;};

private:
	void computeCutEdgeNormal();
	void computeCutPointNormal();
	void intersectionBtwSurfEdgeAndCutSurf(std::vector<int>& colTriIdxInSurfObj, std::vector<int>& colTriIdxInCutSurf, std::vector<CutFace>& cutFaceInSurfObj, std::vector<CutFace>& cutFaceInCutFace, std::vector<int>& colEdgeIdx);
	void intersectionBtwCutEdgeAndSurfObj(std::vector<int>& colTriIdxInSurfObj, std::vector<int>& colTriIdxInCutSurf, std::vector<CutFace>& cutFaceInSurfObj, std::vector<CutFace>& cutFaceInCutFace);
	void addCutFacesInsideSurfObj(std::vector<CutFace>& cutFaceInCutFace, std::vector<int>* pointsOnCutFront);
	void retriangulateSurfCutFace(std::vector<CutFace>& cutFaceInSurfObj, std::vector<int>* pointsOnCutFront);
	void retriangulatecutSurfCutFace(std::vector<CutFace>& cutFaceInCutFace);

	void addCutFacesInsideSurfObjProg(std::vector<CutFace>& cutFaceInCutFace, std::vector<int>* pointsOnCutFront);
	void intersectionBtwPartialCutFaceAndCutSurf(std::vector<int>& colTriIdxInSurfObj, std::vector<int>& colTriIdxInCutSurf, std::vector<CutFace>& cutFaceInSurfObj, std::vector<CutFace>& cutFaceInCutFace, std::vector<int>& colEdgeIdx);
	void intersectionBtwSurfEdgeAndCutSurfProg(std::vector<int>& colTriIdxInSurfObj, std::vector<int>& colTriIdxInCutSurf, std::vector<CutFace>& cutFaceInSurfObj, std::vector<CutFace>& cutFaceInCutFace, std::vector<int>& colEdgeIdx);
	void intersectionBtwCutEdgeAndSurfObjProg(std::vector<int>& colTriIdxInSurfObj, std::vector<int>& colTriIdxInCutSurf, std::vector<CutFace>& cutFaceInSurfObj, std::vector<CutFace>& cutFaceInCutFace);
	void retriangulateSurfCutFaceProg(std::vector<CutFace>& cutFaceInSurfObj, std::vector<int>* pointsOnCutFront, std::vector<int>& colEdgeIdx);

private:
	FILE* F;
	CTimeTick TimeTick;
	SurfaceObj* Obj;
	CuttingTool* Tool;
	int ToolPathIdx;
	
	// Cut surface를 위한 variable
	TopologyContainer* CutSurfContainer;
	std::vector<Vec3f>* CutPoint;
	std::vector<Vec3i>* CutSurf;
	std::vector<Vec2i>* CutEdge;
	std::vector<Vec3f>* CutSurfNormal;
	std::vector<Vec3f> CutEdgeNormal;
	std::vector<Vec3f> CutPointNormal;

	// Surface object의 topology information
	std::vector<Vec3f> AddedPoint;
	std::vector<Vec3i> AddedFace;
	std::vector<Vec3f> AddedPointNormal;
	int PreNbPoint;
	
	// Progressive cutting을 위한 variable
	std::vector<int> VertexIdxOnCutFront;
	std::vector<int> VertexIdxOnCutFrontSurf;
	std::vector<Vec3f> NormalOnCutFront;
	std::vector<Vec3f> NormalOnCutFrontSurf;
	std::vector<int> FaceIdxIncludingVertexOnCutFront;
	bool FIRST_INTERSECTION;

	std::vector<Vec3f>* CutPointTotal;
	std::vector<Vec3i>* CutSurfTotal;
	std::vector<Vec3f>* CutSurfNormalTotal;

	std::vector<int> CutFront;
	std::vector<int> CutFrontGlobal;

	std::vector<int> CutEnd;
	std::vector<int> CutEndGlobal;

	// Surface object의 data
	TopologyContainer* SurfContainer;
	AABBTree* SurfBVH;

	std::vector<Vec3f>* SurfNode;
	std::vector<Vec3i>* SurfFace;
	std::vector<Vec3f>* SurfNormal;
	std::vector<Vec2i>* SurfEdge;

	int NbUpdated;
	int Count;
};

#endif