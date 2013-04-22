#include "stdafx.h"
#include "TopologyModifier.h"

TopologyModifier::TopologyModifier(void)
{
}

TopologyModifier::~TopologyModifier(void)
{
}

void TopologyModifier::addPoints(TopologyContainer* container, std::vector<Vec3f>* points)
{
	for(int i=0;i<points->size();i++)
		addPoint(container, (*points)[i]);
}
void TopologyModifier::addPoints(TopologyContainer* container, std::vector<Vec3f>& points)
{
	for(int i=0;i<points.size();i++)
		addPoint(container, points[i]);
}

void TopologyModifier::addPoints( TopologyContainer* container, std::vector<Vec3f>* point0, std::vector<Vec3f>* points )
{
	for(int i=0;i<points->size();i++)
		addPoint(container, point0->at(i), points->at(i));
}

void TopologyModifier::addPoint( TopologyContainer* container, Vec3f _point0, Vec3f _point )
{
	std::vector<Vec3f>* point0=container->point0();
	std::vector<Vec3f>* point=container->point();
	point0->push_back(_point0);
	point->push_back(_point);

	//update topology information
	std::vector<std::vector<int>>* pointsAroundPoint=container->pointsAroundPoint();
	std::vector<std::vector<int>>* edgesAroundPoint=container->edgesAroundPoint();
	std::vector<std::vector<int>>* facesAroundPoint=container->facesAroundPoint();

	pointsAroundPoint->resize(point->size());
	edgesAroundPoint->resize(point->size());
	facesAroundPoint->resize(point->size());
}

void TopologyModifier::addPoint(TopologyContainer* container, Vec3f _point)
{
	std::vector<Vec3f>* point0=container->point0();
	std::vector<Vec3f>* point=container->point();
	point0->push_back(_point);
	point->push_back(_point);

	//update topology information
	std::vector<std::vector<int>>* pointsAroundPoint=container->pointsAroundPoint();
	std::vector<std::vector<int>>* edgesAroundPoint=container->edgesAroundPoint();
	std::vector<std::vector<int>>* facesAroundPoint=container->facesAroundPoint();

	pointsAroundPoint->resize(point->size());
	edgesAroundPoint->resize(point->size());
	facesAroundPoint->resize(point->size());
}
void TopologyModifier::addEdges(TopologyContainer* container, std::vector<Vec2i>* edges)
{
	for(int i=0;i<edges->size();i++)
		addEdge(container, (*edges)[i]);
}
void TopologyModifier::addEdges(TopologyContainer* container, std::vector<Vec2i>& edges)
{
	std::vector<std::vector<int>>* pointAroundPoint=container->pointsAroundPoint();
	for(int i=0;i<edges.size();i++)
	{
		Vec2i edge=edges[i];
		addEdge(container, edges[i]);
	}
}
void TopologyModifier::addEdge(TopologyContainer* container, Vec2i _edge)
{
	std::vector<Vec2i>* edge=container->edge();
	edge->push_back(_edge);

	//update topology information
	std::vector<std::vector<int>>* pointsAroundPoint=container->pointsAroundPoint();
	(*pointsAroundPoint)[_edge[0]].push_back(_edge[1]);
	(*pointsAroundPoint)[_edge[1]].push_back(_edge[0]);

	std::vector<std::vector<int>>* edgesAroundPoint=container->edgesAroundPoint();
	(*edgesAroundPoint)[_edge[0]].push_back(edge->size()-1);
	(*edgesAroundPoint)[_edge[1]].push_back(edge->size()-1);

	std::vector<std::vector<int>>* facesAroundEdge=container->facesAroundEdge();
	facesAroundEdge->resize(edge->size());
}
void TopologyModifier::addFaces(TopologyContainer* container, std::vector<Vec3i>* faces, int preNbPoint)
{
	for(int i=0;i<faces->size();i++)
		addFace(container, (*faces)[i], preNbPoint);
}
void TopologyModifier::addFaces(TopologyContainer* container, std::vector<Vec3i>& faces, int preNbPoint)
{
	for(int i=0;i<faces.size();i++)
		addFace(container, faces[i], preNbPoint);
}
void TopologyModifier::addFace(TopologyContainer* container, Vec3i _face, int preNbPoint)
{
	std::vector<Vec3i>* face=container->face();
	std::vector<Vec2i>* edge=container->edge();
	face->push_back(_face);

	//update topology information

	//faces around point
	std::vector<std::vector<int>>* facesAroundPoint=container->facesAroundPoint();
	for(int i=0;i<3;i++)
		(*facesAroundPoint)[_face[i]].push_back(face->size()-1);

	//edges in face
	std::vector<std::vector<int>>* edgesInFace=container->edgesInFace();
	std::vector<std::vector<int>>* edgesAroundPoint=container->edgesAroundPoint();
	edgesInFace->resize(face->size());

	for(int i=0;i<3;i++)
	{
		Vec2i edge1;
		if(_face[i]<_face[(i+1)%3])
			edge1=Vec2i(_face[i], _face[(i+1)%3]);
		else
			edge1=Vec2i(_face[(i+1)%3], _face[i]);
		
		bool flag=false;
		int edgeIdx=-1;
		for(int j=0;j<(*edgesAroundPoint)[edge1[0]].size();j++)
		{
			edgeIdx=(*edgesAroundPoint)[edge1[0]][j];
			Vec2i edge2=(*edge)[edgeIdx];
			if(edge1==edge2)
			{
				flag=true;
				break;
			}
		}
		if(flag)
			(*edgesInFace)[face->size()-1].push_back(edgeIdx);
		else
		{
			addEdge(container, edge1);
			(*edgesInFace)[face->size()-1].push_back(edge->size()-1);
		}

		/*if(edge1[0]>=preNbPoint)
		{
			bool flag=false;
			int edgeIdx=-1;
			for(int j=0;j<(*edgesAroundPoint)[edge1[0]].size();j++)
			{
				edgeIdx=(*edgesAroundPoint)[edge1[0]][j];
				Vec2i edge2=(*edge)[edgeIdx];
				if(edge1==edge2)
				{
					flag=true;
					break;
				}
			}
			if(flag)
				(*edgesInFace)[face->size()-1].push_back(edgeIdx);
			else
			{
				addEdge(container, edge1);
				(*edgesInFace)[face->size()-1].push_back(edge->size()-1);
			}
		}
		else if(edge1[1]>=preNbPoint)
		{
			// edge가 이미 추가 되었는지 검사
			bool flag=false;
			int edgeIdx=-1;
			for(int j=0;j<(*edgesAroundPoint)[edge1[0]].size();j++)
			{
				edgeIdx=(*edgesAroundPoint)[edge1[0]][j];
				Vec2i edge2=(*edge)[edgeIdx];
				if(edge1==edge2)
				{
					flag=true;
					break;
				}
			}
			if(flag)
				(*edgesInFace)[face->size()-1].push_back(edgeIdx);
			else
			{
				addEdge(container, edge1);
				(*edgesInFace)[face->size()-1].push_back(edge->size()-1);
			}
		}
		else
		{
			// edge index를 찾는다
			int edgeIdx=-1;
			for(int j=0;j<(*edgesAroundPoint)[edge1[0]].size();j++)
			{
				edgeIdx=(*edgesAroundPoint)[edge1[0]][j];
				Vec2i edge2=(*edge)[edgeIdx];
				if(edge1==edge2)
					break;
			}
			(*edgesInFace)[face->size()-1].push_back(edgeIdx);
		}*/
	}

	//faces around edge
	std::vector<std::vector<int>>* facesAroundEdge=container->facesAroundEdge();
	for(int i=0;i<3;i++)
	{
		int edgeIdx=(*edgesInFace)[face->size()-1][i];
		(*facesAroundEdge)[edgeIdx].push_back(face->size()-1);
	}
}

void TopologyModifier::removeEdges(TopologyContainer* container, std::vector<int>* idxs)
{
	//for(int i=0;i<idxs->size();i++)
	for(int i=idxs->size()-1;i>=0;i--)
		removeEdge(container, (*idxs)[i]);
}
void TopologyModifier::removeEdges(TopologyContainer* container, std::vector<int>& idxs)
{
	//for(int i=0;i<idxs.size();i++)
	for(int i=idxs.size()-1;i>=0;i--)
		removeEdge(container, idxs[i]);
}
void TopologyModifier::removeEdge(TopologyContainer* container, int idx)
{
	std::vector<Vec2i>* edges=container->edge();
	int nbEdge=edges->size();

	//update topology information
	
	//1. EdgesAroundPoint
	std::vector<std::vector<int>>* edgesAroundPoint=container->edgesAroundPoint();
	for(int i=0;i<2;i++)
	{
		int pointIdx=(*edges)[idx][i];
		for(int j=0;j<(*edgesAroundPoint)[pointIdx].size();j++)
		{
			if((*edgesAroundPoint)[pointIdx][j]==idx)
			{
				removeVectorComp((*edgesAroundPoint)[pointIdx],j);
				break;
			}
		}

		pointIdx=(*edges)[nbEdge-1][i];
		for(int j=0;j<(*edgesAroundPoint)[pointIdx].size();j++)
		{
			if((*edgesAroundPoint)[pointIdx][j]==(nbEdge-1))
			{
				(*edgesAroundPoint)[pointIdx][j]=idx;
				break;
			}
		}
	}

	//2. PointsAroundPoint
	std::vector<std::vector<int>>* pointsAroundPoint=container->pointsAroundPoint();
	for(int i=0;i<2;i++)
	{
		int pointIdx=(*edges)[idx][i];
		for(int j=0;j<(*pointsAroundPoint)[pointIdx].size();j++)
		{
			if((*pointsAroundPoint)[pointIdx][j]==(*edges)[idx][(i+1)%2])
			{
				removeVectorComp((*pointsAroundPoint)[(*edges)[idx][i]],j);
				break;
			}

		}
	}

	//3. EdgesInFace
	std::vector<std::vector<int>>* edgesInFace=container->edgesInFace();
	std::vector<std::vector<int>>* facesAroundEdge=container->facesAroundEdge();

	for(int i=0;i<(*facesAroundEdge)[nbEdge-1].size();i++)
	{
		int faceIdx=(*facesAroundEdge)[nbEdge-1][i];
		for(int j=0;j<3;j++)
		{
			if((*edgesInFace)[faceIdx][j]==(nbEdge-1))
			{
				(*edgesInFace)[faceIdx][j]=idx;
				break;
			}
		}
	}

	//4. FacesAroundEdge
	(*facesAroundEdge)[idx]=(*facesAroundEdge)[nbEdge-1];
	facesAroundEdge->pop_back();

	//5. Edge
	(*edges)[idx]=(*edges)[nbEdge-1];
	edges->pop_back();
}
void TopologyModifier::removeFaces(TopologyContainer* container, std::vector<int>* idxs)
{
	for(int i=0;i<idxs->size();i++)
		removeFace(container, (*idxs)[i]);
}
void TopologyModifier::removeFaces(TopologyContainer* container, std::vector<int>& idxs)
{
	//for(int i=0;i<idxs.size();i++)
	for(int i=idxs.size()-1;i>=0;i--)
		removeFace(container, idxs[i]);
}
void TopologyModifier::removeFace(TopologyContainer* container, int idx)
{
	std::vector<Vec3i>* face=container->face();
	int nbFace=face->size();

	//Topology information update
	
	//1. FacesAroundPoint
	std::vector<std::vector<int>>* facesAroundPoint=container->facesAroundPoint();
	for(int i=0;i<3;i++)
	{
		int pointIdx=(*face)[idx][i];
		for(int j=0;j<(*facesAroundPoint)[pointIdx].size();j++)
		{
			if((*facesAroundPoint)[pointIdx][j]==idx)
			{
				removeVectorComp((*facesAroundPoint)[pointIdx],j);
				break;
			}
		}

		pointIdx=(*face)[nbFace-1][i];
		for(int j=0;j<(*facesAroundPoint)[pointIdx].size();j++)
		{
			if((*facesAroundPoint)[pointIdx][j]==(nbFace-1))
			{
				(*facesAroundPoint)[pointIdx][j]=idx;
				break;
			}
		}
	}

	//2. FacesAroundEdge
	std::vector<std::vector<int>>* facesAroundEdge=container->facesAroundEdge();
	std::vector<std::vector<int>>* edgesInFace=container->edgesInFace();
	for(int i=0;i<(*edgesInFace)[idx].size();i++)
	{
		int edgeIdx=(*edgesInFace)[idx][i];
		for(int j=0;j<(*facesAroundEdge)[edgeIdx].size();j++)
		{
			if((*facesAroundEdge)[edgeIdx][j]==idx)
			{
				removeVectorComp((*facesAroundEdge)[edgeIdx], j);
				break;
			}
		}

		/*edgeIdx=(*edgesInFace)[nbFace-1][i];
		for(int j=0;j<(*facesAroundEdge)[edgeIdx].size();j++)
		{
			if((*facesAroundEdge)[edgeIdx][j]==(nbFace-1))
			{
				(*facesAroundEdge)[edgeIdx][j]=idx;
				break;
			}
		}*/
	}

	for(int i=0;i<(*edgesInFace)[nbFace-1].size();i++)
	{
		int edgeIdx=(*edgesInFace)[nbFace-1][i];
		for(int j=0;j<(*facesAroundEdge)[edgeIdx].size();j++)
		{
			if((*facesAroundEdge)[edgeIdx][j]==(nbFace-1))
				(*facesAroundEdge)[edgeIdx][j]=idx;
		}
	}

	//3. EdgesInFace
	(*edgesInFace)[idx]=(*edgesInFace)[nbFace-1];
	edgesInFace->pop_back();

	//4. Face
	(*face)[idx]=(*face)[nbFace-1];
	face->pop_back();
}

void TopologyModifier::removeVectorComp(std::vector<int>& vec, int idx)
{
	int nbComp=vec.size();
	vec[idx]=vec[nbComp-1];
	vec.pop_back();
}