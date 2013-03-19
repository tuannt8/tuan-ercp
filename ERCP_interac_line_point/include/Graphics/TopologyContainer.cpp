#include "stdafx.h"
#include "TopologyContainer.h"

TopologyContainer::TopologyContainer(void)
{
	PointsAroundPoint=NULL;
	EdgesAroundPoint=NULL;
	FacesAroundPoint=NULL;

	FacesAroundEdge=NULL;
	EdgesInFace=NULL;
}

TopologyContainer::~TopologyContainer(void)
{
	if(PointsAroundPoint)
		delete PointsAroundPoint;
	if(EdgesAroundPoint)
		delete EdgesAroundPoint;
	if(FacesAroundPoint)
		delete FacesAroundPoint;

	if(FacesAroundEdge)
		delete FacesAroundEdge;
	if(EdgesInFace)
		delete EdgesInFace;
}

void TopologyContainer::init(std::vector<Vec3f>* point0, std::vector<Vec3f>* point, std::vector<Vec3i>* face, std::vector<Vec2i>* edge)
{
	Point0=point0;
	Point=point;
	Face=face;
	Edge=edge;

	PointsAroundPoint=new std::vector<std::vector<int>>; PointsAroundPoint->resize(Point->size());
	EdgesAroundPoint=new std::vector<std::vector<int>>; EdgesAroundPoint->resize(Point->size());
	FacesAroundPoint=new std::vector<std::vector<int>>; FacesAroundPoint->resize(Point->size());

	//Face around point
	for(int i=0;i<Face->size();i++)
	{
		for(int j=0;j<3;j++)
			(*FacesAroundPoint)[(*Face)[i][j]].push_back(i);
	}

	//Point around point
	for(int i=0;i<Point->size();i++)
	{
		for(int j=0;j<(*FacesAroundPoint)[i].size();j++)
		{
			for(int k=0;k<3;k++)
			{
				if(i!=(*Face)[(*FacesAroundPoint)[i][j]][k])
				{
					if((*PointsAroundPoint)[i].size()==0)
						(*PointsAroundPoint)[i].push_back((*Face)[(*FacesAroundPoint)[i][j]][k]);
					else
					{
						bool flag=true;
						for(int l=0;l<(*PointsAroundPoint)[i].size();l++)
						{
							if((*Face)[(*FacesAroundPoint)[i][j]][k]==(*PointsAroundPoint)[i][l])
							{
								flag=false;
								break;
							}
						}
						if(flag)
							(*PointsAroundPoint)[i].push_back((*Face)[(*FacesAroundPoint)[i][j]][k]);
					}
				}
			}
		}
	}

	//Edge information
	for(int i=0;i<Point->size();i++)
	{
		for(int j=0;j<(*PointsAroundPoint)[i].size();j++)
		{
			if((*PointsAroundPoint)[i][j]>i)
			{
				Vec2i _edge(i,(*PointsAroundPoint)[i][j]);
				Edge->push_back(_edge);
			}
		}
	}

	//Edge around point
	for(int i=0;i<Edge->size();i++)
	{
		for(int j=0;j<2;j++)
			(*EdgesAroundPoint)[(*Edge)[i][j]].push_back(i);
	}

	//Edges in face
	EdgesInFace=new std::vector<std::vector<int>>; EdgesInFace->resize(Face->size());
	for(int i=0;i<Face->size();i++)
	{
		for(int j=0;j<2;j++)
		{
			for(int k=0;k<(*EdgesAroundPoint)[(*Face)[i][j]].size();k++)
			{
				Vec2i edge=(*Edge)[(*EdgesAroundPoint)[(*Face)[i][j]][k]];

				int count=0;
				for(int l=j;l<3;l++)
				{
					if((*Face)[i][l]==edge[0])
						count++;
					if((*Face)[i][l]==edge[1])
						count++;
				}
				if(count==2)
					(*EdgesInFace)[i].push_back((*EdgesAroundPoint)[(*Face)[i][j]][k]);
			}
		}
	}

	//faces around edge
	FacesAroundEdge=new std::vector<std::vector<int>>; FacesAroundEdge->resize(Edge->size());
	for(int i=0;i<Face->size();i++)
	{
		std::vector<int> edgesInFace=(*EdgesInFace)[i];
		for(int j=0;j<3;j++)
		{
			int edgeIdx=(*EdgesInFace)[i][j];
			(*FacesAroundEdge)[edgeIdx].push_back(i);
		}
	}
}

std::vector<std::vector<int>>* TopologyContainer::pointsAroundPoint()
{
	return PointsAroundPoint;
}
std::vector<std::vector<int>>* TopologyContainer::edgesAroundPoint()
{
	return EdgesAroundPoint;
}
std::vector<std::vector<int>>* TopologyContainer::facesAroundPoint()
{
	return FacesAroundPoint;
}
std::vector<std::vector<int>>* TopologyContainer::facesAroundEdge()
{
	return FacesAroundEdge;
}
std::vector<int> TopologyContainer::facesAroundEdge( int idx )
{
	return (*FacesAroundEdge)[idx];
}

std::vector<std::vector<int>>* TopologyContainer::edgesInFace()
{
	return EdgesInFace;
}

std::vector<Vec3f>* TopologyContainer::point0()
{
	return Point0;
}
std::vector<Vec3f>* TopologyContainer::point()
{
	return Point;
}
std::vector<Vec2i>* TopologyContainer::edge()
{
	return Edge;
}
std::vector<Vec3i>* TopologyContainer::face()
{
	return Face;
}