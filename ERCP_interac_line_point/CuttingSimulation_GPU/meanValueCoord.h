#pragma once
#include "stdafx.h"

#include "../../include/DataTypes/vectorfunc.h"
#include "../../include/DataTypes/geometricfunc.h"
#include "../../include/Graphics/TopologyContainer.h"
#include "../../include/Modules/SurfaceCuttingManager.h"
#include "define.h"
#include "Utility.h"
#include "DataStruct.h"

class meanValueCoord
{
public:
	meanValueCoord(void);
	~meanValueCoord(void);

	// for testing and debugging
// 	void init(TopologyContainer* container, arrayVec3f* points, arrayVec3i* face, arrayInt areaFaceIdxs,
// 				std::vector<pointInfo>& cutPoint);
	void draw(int mode); 

	arrayVec2f holePoints;

public:
	void init(arrayVec3f points, arrayVec3i face);
	Vec2f convertPointOnEdgeto2D(Vec3f point, int pIdx1, int pIdx2);
	Vec3f convert2to3(Vec2f point);
private:
	arrayInt findBoundary();
	void findNeighborWeight();

	void assignPlanarBoundary( arrayInt orderedPointIdxs );
	bool isEdge(int pIdx1, int pIdx2, arrayInt* edgeIdx=NULL);

	void assignPlanarInternalPoints( arrayInt orderedPointIdxs );

public:
	// 3D info
	TopologyContainer* m_container;
	arrayVec3f m_point;
	arrayVec3i m_face;
	arrayVec2i m_edge;
	std::vector<std::vector<float>> m_weightNeighbor;
	// 2D info
	arrayVec2f m_planarPoint;

	int m_nbPointOnBound;

};
