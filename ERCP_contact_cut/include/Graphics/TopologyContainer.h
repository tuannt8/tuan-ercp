#ifndef TOPOLOGY_CONTAINER
#define TOPOLOGY_CONTAINER

#include "DataTypes/Define.h"
#include "DataTypes/Vec.h"
#include "DataTypes/Mat.h"
#include <afxwin.h>
#include <GL/glu.h>
#include <GL/gl.h>
#include <vector>

class TopologyContainer
{
public:
	TopologyContainer(void);
	~TopologyContainer(void);

//functions
public:
	void init(std::vector<Vec3f>* point0, std::vector<Vec3f>* point, std::vector<Vec3i>* face, std::vector<Vec2i>* edge);
	std::vector<std::vector<int>>* pointsAroundPoint();
	std::vector<std::vector<int>>* edgesAroundPoint();
	std::vector<std::vector<int>>* facesAroundPoint();
	std::vector<std::vector<int>>* facesAroundEdge();
	std::vector<int> facesAroundEdge(int idx);
	std::vector<std::vector<int>>* edgesInFace();

	std::vector<Vec3f>* point0();
	std::vector<Vec3f>* point();
	std::vector<Vec2i>* edge();
	std::vector<Vec3i>* face();

//variables
private:
	std::vector<Vec3f>* Point0;
	std::vector<Vec3f>* Point;
	std::vector<Vec3i>* Face;
	std::vector<Vec2i>* Edge;

	std::vector<std::vector<int>>* PointsAroundPoint;
	std::vector<std::vector<int>>* EdgesAroundPoint;
	std::vector<std::vector<int>>* FacesAroundPoint;
	std::vector<std::vector<int>>* FacesAroundEdge;
	std::vector<std::vector<int>>* EdgesInFace;
};

#endif