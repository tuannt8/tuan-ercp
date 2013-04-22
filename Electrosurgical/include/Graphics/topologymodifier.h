#ifndef TOPOLOGY_MODIFIER
#define TOPOLOGY_MODIFIER

#include "DataTypes/Define.h"
#include "DataTypes/Vec.h"
#include "DataTypes/Mat.h"
#include "TopologyContainer.h"
#include <afxwin.h>
#include <GL/glu.h>
#include <GL/gl.h>
#include <vector>

class TopologyModifier
{
public:
	TopologyModifier(void);
	~TopologyModifier(void);

//functions
public:
	void addPoints(TopologyContainer* container, std::vector<Vec3f>* points);
	void addPoints(TopologyContainer* container, std::vector<Vec3f>& points);

	void addPoints(TopologyContainer* container, std::vector<Vec3f>* point0, std::vector<Vec3f>* points);
	
	void addEdges(TopologyContainer* container, std::vector<Vec2i>* edges);
	void addEdges(TopologyContainer* container, std::vector<Vec2i>& edges);
	
	void addFaces(TopologyContainer* container, std::vector<Vec3i>* faces, int preNbPoint);
	void addFaces(TopologyContainer* container, std::vector<Vec3i>& faces, int preNbPoint);
	
	void removePoints(TopologyContainer* container, std::vector<int>* idxs);
	void removePoints(TopologyContainer* container, std::vector<int>& idxs);
	
	void removeEdges(TopologyContainer* container, std::vector<int>* idxs);
	void removeEdges(TopologyContainer* container, std::vector<int>& idxs);
	
	void removeFaces(TopologyContainer* container, std::vector<int>* idxs);
	void removeFaces(TopologyContainer* container, std::vector<int>& idxs);

public: //Should be private
	void addPoint(TopologyContainer* container, Vec3f _point0, Vec3f _point);
	void addPoint(TopologyContainer* container, Vec3f point);
	void addEdge(TopologyContainer* container, Vec2i edge);
	void addFace(TopologyContainer* container, Vec3i face, int preNbPoint);
	void removePoint(TopologyContainer* container, int idx);
	void removeEdge(TopologyContainer* container, int idx);
	void removeFace(TopologyContainer* container, int idx);

	void removeVectorComp(std::vector<int>& vec, int idx);

//variables
private:
};

#endif