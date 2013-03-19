#include "stdafx.h"
#include "ReTriangulation.h"

ReTriangulation::ReTriangulation(void)
{
}

ReTriangulation::~ReTriangulation(void)
{
}

void ReTriangulation::delaunayTriangulation(std::vector<Vec3f>* point, Vec3i face, Vec2i edge, std::vector<int>& idx, int pointOnEdge, std::vector<int>& pointOnFace, std::vector<Vec3i>& triangles)
{
	// 1. 순서대로 point 정렬
	std::vector<int> pointIdx;
	std::vector<float> dis;
	pointIdx.push_back(pointOnEdge);
	dis.push_back(0.0);
	for(int i=0;i<pointOnFace.size();i++)
	{
		float _dis=((*point)[pointOnEdge]-(*point)[pointOnFace[i]]).norm();
		dis.push_back(_dis);
		pointIdx.push_back(pointOnFace[i]);
		for(int j=0;j<dis.size();j++)
		{
			if(dis[j]>_dis)
			{
				for(int k=dis.size()-1;k>j;k--)
				{
					dis[k]=dis[k-1];
					pointIdx[k]=pointIdx[k-1];
				}
				dis[j]=_dis;
				pointIdx[j]=pointOnFace[i];
				break;
			}
		}
	}

	// 2. Intersection 된 edge에 포함되지 않는 point
	int _pointIdx;
	for(int i=0;i<3;i++)
	{
		if(!((face[i]==edge[0])||(face[i]==edge[1])))
			_pointIdx=face[i];
	}

	// 2. Triangulation
	for(int i=0;i<pointIdx.size()-1;i++)
	{
		triangles.push_back(Vec3i(edge[0], pointIdx[i], pointIdx[i+1]));
		triangles.push_back(Vec3i(edge[1], pointIdx[i], pointIdx[i+1]));
	}
	triangles.push_back(Vec3i(edge[0], pointIdx[pointIdx.size()-1], _pointIdx));
	triangles.push_back(Vec3i(edge[1], pointIdx[pointIdx.size()-1], _pointIdx));

	for(int i=0;i<idx.size();i++)
	{
		for(int j=0;j<triangles.size();j++)
		{
			for(int k=0;k<3;k++)
			{
				if(triangles[j][k]==idx[i])
				{
					triangles[j]=triangles[triangles.size()-1];
					triangles.pop_back();
					j--;
					break;
				}
			}
		}
	}
}

void ReTriangulation::delaunayTriangulation(std::vector<Vec3f>* point, Vec3i face, std::vector<int>& idx, std::vector<int>& pointOnEdge, std::vector<int>& pointOnFace, std::vector<Vec3i>& triangles, int preNbPoint)
{
	GeometricFunc func;
	std::vector<std::vector<int>> trianglesAroundEdge;
	std::vector<std::vector<int>> edgesInTriangle;
	std::vector<Vec2i> edges;
	Vec3f normal=func.computeNormal(point, face);

	triangles.push_back(face);

	for(int i=0;i<pointOnEdge.size();i++)
	{
		for(int j=0;j<triangles.size();j++)
		{
			int edgeIdx=-1;
			for(int k=0;k<3;k++)
			{
				Vec3f l1=(*point)[triangles[j][k]];
				Vec3f l2=(*point)[triangles[j][(k+1)%3]];
				Vec3f p=(*point)[pointOnEdge[i]];
				if(func.isPointInLine(l1, l2, p))
				{
					edgeIdx=(k+2)%3;
					break;
				}
			}
			if(edgeIdx>-1)
			{
				//add triangles and edges
				triangles.push_back(Vec3i(triangles[j][(edgeIdx+1)%3], pointOnEdge[i], triangles[j][edgeIdx]));
				triangles.push_back(Vec3i(pointOnEdge[i], triangles[j][(edgeIdx+2)%3], triangles[j][edgeIdx]));

				//remove triangle
				triangles[j]=triangles[triangles.size()-1];
				triangles.pop_back();
				break;
			}
		}
	}

	for(int i=0;i<pointOnFace.size();i++)
	{
		int count=0;
		for(int j=0;j<triangles.size();j++)
		{
			int edgeIdx=-1;
			for(int k=0;k<3;k++)
			{
				Vec3f l1=(*point)[triangles[j][k]];
				Vec3f l2=(*point)[triangles[j][(k+1)%3]];
				Vec3f p=(*point)[pointOnFace[i]];
				if(func.isPointInLine(l1, l2, p))
				{
					edgeIdx=(k+2)%3;
					break;
				}
			}
			if(edgeIdx>-1)
			{
				//add triangles and edges
				triangles.push_back(Vec3i(triangles[j][(edgeIdx+1)%3], pointOnFace[i], triangles[j][edgeIdx]));
				triangles.push_back(Vec3i(pointOnFace[i], triangles[j][(edgeIdx+2)%3], triangles[j][edgeIdx]));

				//remove triangle
				triangles[j]=triangles[triangles.size()-1];
				triangles.pop_back();
				count++;
				if(count>1)
					break;
			}
			else if(func.isPointInTri(point, triangles[j], pointOnFace[i]))
			{
				//add triangles
				triangles.push_back(Vec3i(pointOnFace[i], triangles[j][0], triangles[j][1]));
				triangles.push_back(Vec3i(pointOnFace[i], triangles[j][1], triangles[j][2]));
				triangles.push_back(Vec3i(pointOnFace[i], triangles[j][2], triangles[j][0]));

				//remove triangle
				triangles[j]=triangles[triangles.size()-1];
				triangles.pop_back();

				int triIdx=triangles.size()-2;
				legalizeEdge(point, pointOnFace[i], Vec2i(triangles[triIdx][1], triangles[triIdx][2]), triIdx, triangles);
				triIdx=triangles.size()-1;
				legalizeEdge(point, pointOnFace[i], Vec2i(triangles[triIdx][1], triangles[triIdx][2]), triIdx, triangles);
				triIdx=j;
				legalizeEdge(point, pointOnFace[i], Vec2i(triangles[triIdx][1], triangles[triIdx][2]), triIdx, triangles);
				break;
			}
		}
	}

	for(int i=0;i<idx.size();i++)
	{
		for(int j=0;j<triangles.size();j++)
		{
			for(int k=0;k<3;k++)
			{
				if(triangles[j][k]==idx[i])
				{
					triangles[j]=triangles[triangles.size()-1];
					triangles.pop_back();
					j--;
					break;
				}
			}
		}
	}
}

// 삼각형의 일부분만 포함하여 retriangulation 하는 함수. face[i]=-1인 점은 포함시키지 않는다.
void ReTriangulation::delaunayTriangulation(std::vector<Vec3f>* point, Vec3i face, Vec3f tri[], std::vector<int>& pointOnEdge, std::vector<int>& pointOnFace, std::vector<Vec3i>& triangles, int preNbPoint)
{
	GeometricFunc func;
	std::vector<std::vector<int>> trianglesAroundEdge;
	std::vector<std::vector<int>> edgesInTriangle;
	std::vector<Vec2i> edges;
	std::vector<int> idx;

	for(int i=0;i<3;i++)
	{
		if(face[i]==-1)
		{
			point->push_back(tri[i]);
			idx.push_back(point->size()-1);
			face[i]=point->size()-1;
		}
	}
	//Vec3f normal=-func.computeNormal(point, face);

	triangles.push_back(face);

	for(int i=0;i<pointOnEdge.size();i++)
	{
		for(int j=0;j<triangles.size();j++)
		{
			int edgeIdx=-1;
			for(int k=0;k<3;k++)
			{
				Vec3f direc1=(*point)[triangles[j][(k+1)%3]]-(*point)[triangles[j][k]];
				direc1.normalize();
				Vec3f direc2=(*point)[pointOnEdge[i]]-(*point)[triangles[j][k]]; 
				direc2.normalize();
				Vec3f direc3=(*point)[pointOnEdge[i]]-(*point)[triangles[j][(k+1)%3]];
				direc3.normalize();
				if((1-direc1*direc2)<EPS)
				{
					if(direc2*direc3<0)
					{
						edgeIdx=(k+2)%3;
						break;
					}
				}
			}
			if(edgeIdx>-1)
			{
				//add triangles and edges
				triangles.push_back(Vec3i(triangles[j][(edgeIdx+1)%3], pointOnEdge[i], triangles[j][edgeIdx]));
				triangles.push_back(Vec3i(pointOnEdge[i], triangles[j][(edgeIdx+2)%3], triangles[j][edgeIdx]));

				//remove triangle
				triangles[j]=triangles[triangles.size()-1];
				triangles.pop_back();
				break;
			}
		}
	}

	for(int i=0;i<pointOnFace.size();i++)
	{
		int count=0;
		for(int j=0;j<triangles.size();j++)
		{
			int edgeIdx=-1;
			for(int k=0;k<3;k++)
			{
				Vec3f l1=(*point)[triangles[j][k]];
				Vec3f l2=(*point)[triangles[j][(k+1)%3]];
				Vec3f p=(*point)[pointOnFace[i]];
				if(func.isPointInLine(l1, l2, p))
				{
					edgeIdx=(k+2)%3;
					break;
				}
			}

			if(edgeIdx>-1)
			{
				//add triangles and edges
				triangles.push_back(Vec3i(triangles[j][(edgeIdx+1)%3], pointOnFace[i], triangles[j][edgeIdx]));
				triangles.push_back(Vec3i(pointOnFace[i], triangles[j][(edgeIdx+2)%3], triangles[j][edgeIdx]));

				//remove triangle
				triangles[j]=triangles[triangles.size()-1];
				triangles.pop_back();
				count++;
				if(count>1)
					break;
			}
			else if(func.isPointInTri(point, triangles[j], pointOnFace[i]))
			{
				//add triangles
				triangles.push_back(Vec3i(pointOnFace[i], triangles[j][0], triangles[j][1]));
				triangles.push_back(Vec3i(pointOnFace[i], triangles[j][1], triangles[j][2]));
				triangles.push_back(Vec3i(pointOnFace[i], triangles[j][2], triangles[j][0]));

				//remove triangle
				triangles[j]=triangles[triangles.size()-1];
				triangles.pop_back();

				int triIdx=triangles.size()-2;
				legalizeEdge(point, pointOnFace[i], Vec2i(triangles[triIdx][1], triangles[triIdx][2]), triIdx, triangles);
				triIdx=triangles.size()-1;
				legalizeEdge(point, pointOnFace[i], Vec2i(triangles[triIdx][1], triangles[triIdx][2]), triIdx, triangles);
				triIdx=j;
				legalizeEdge(point, pointOnFace[i], Vec2i(triangles[triIdx][1], triangles[triIdx][2]), triIdx, triangles);
				break;
			}
		}
	}

	for(int i=0;i<idx.size();i++)
	{
		point->pop_back();
		for(int j=0;j<triangles.size();j++)
		{
			for(int k=0;k<3;k++)
			{
				if(triangles[j][k]==idx[i])
				{
					triangles[j]=triangles[triangles.size()-1];
					triangles.pop_back();
					j--;
					break;
				}
			}
		}
	}

//	for(int i=0;i<triangles.size();i++)
//		legalizeTriangle(point, triangles[i], normal);
}

void ReTriangulation::findIllegalEdge(std::vector<Vec3f>* point, std::vector<int>& pointOnEdge, std::vector<int>& pointOnFace, std::vector<Vec2i>& illegalEdge)
{
	Vec3f v1=(*point)[pointOnEdge[1]]-(*point)[pointOnEdge[0]]; v1.normalize();
	std::vector<float> dis;
	std::vector<int> pointOrder;
	for(int i=0;i<pointOnFace.size();i++)
	{
		float a=v1*((*point)[pointOnFace[i]]-(*point)[pointOnEdge[0]]);
		dis.push_back(a);
	}

	if(pointOnFace.size()==1)
	{
		illegalEdge.push_back(Vec2i(pointOnEdge[0], pointOnEdge[1]));
		return;
	}
	if(pointOnFace.size()==2)
	{
		pointOrder.resize(4);
		pointOrder[0]=pointOnEdge[0];
		if(dis[0]>dis[1])
		{
			pointOrder[1]=pointOnFace[1];
			pointOrder[2]=pointOnFace[0];
		}
		else
		{
			pointOrder[1]=pointOnFace[0];
			pointOrder[2]=pointOnFace[1];
		}
		pointOrder[3]=pointOnEdge[1];

		illegalEdge.push_back(Vec2i(pointOrder[0],pointOrder[3]));
		illegalEdge.push_back(Vec2i(pointOrder[0],pointOrder[2]));
		illegalEdge.push_back(Vec2i(pointOrder[1],pointOrder[3]));
		return;
	}
	if(pointOnFace.size()==3)
	{
		pointOrder.resize(5);
		pointOrder[0]=pointOnEdge[0];
		if(dis[0]<dis[1])
		{
			if(dis[0]<dis[2])
			{
				if(dis[1]<dis[2])
				{
					pointOrder[1]=pointOnFace[0];
					pointOrder[2]=pointOnFace[1];
					pointOrder[3]=pointOnFace[2];
				}
				else
				{
					pointOrder[1]=pointOnFace[0];
					pointOrder[2]=pointOnFace[2];
					pointOrder[3]=pointOnFace[1];
				}	
			}
			else
			{
				pointOrder[1]=pointOnFace[2];
				pointOrder[2]=pointOnFace[0];
				pointOrder[3]=pointOnFace[1];
			}
		}
		else
		{
			if(dis[0]<dis[2])
			{
				pointOrder[1]=pointOnFace[1];
				pointOrder[2]=pointOnFace[0];
				pointOrder[3]=pointOnFace[2];
			}
			else
			{
				if(dis[1]<dis[2])
				{
					pointOrder[1]=pointOnFace[1];
					pointOrder[2]=pointOnFace[2];
					pointOrder[3]=pointOnFace[0];
				}
				else
				{
					pointOrder[1]=pointOnFace[2];
					pointOrder[2]=pointOnFace[1];
					pointOrder[3]=pointOnFace[0];
				}	
			}
		}
		pointOrder[4]=pointOnEdge[1];

		illegalEdge.push_back(Vec2i(pointOrder[0],pointOrder[4]));
		illegalEdge.push_back(Vec2i(pointOrder[0],pointOrder[3]));
		illegalEdge.push_back(Vec2i(pointOrder[1],pointOrder[4]));
		illegalEdge.push_back(Vec2i(pointOrder[0],pointOrder[2]));
		illegalEdge.push_back(Vec2i(pointOrder[1],pointOrder[3]));
		illegalEdge.push_back(Vec2i(pointOrder[2],pointOrder[4]));
	}
}

void ReTriangulation::legalizeTriangle(std::vector<Vec3f>* point, Vec3i& face, Vec3f& normal)
{
	GeometricFunc func;
	Vec3f n=func.computeNormal(point, face);
	if(n*normal<0)
	{
		int temp=face[1];
		face[1]=face[2];
		face[2]=temp;
	}
}

void ReTriangulation::legalizeTriangle(Vec3f point[], Vec3i& face, Vec3f& normal)
{
	GeometricFunc func;
	Vec3f n=func.computeNormal(point);
	if(n*normal<0)
	{
		int temp=face[1];
		face[1]=face[2];
		face[2]=temp;
	}
}

void ReTriangulation::legalizeEdge(std::vector<Vec3f>* point, std::vector<Vec2i>& illegalEdge, std::vector<Vec3i>& triangles)
{
	GeometricFunc func;
	int i=0;
	while(1)
	{
		if(illegalEdge.empty())
			break;

		std::vector<int> triIdx;
		std::vector<int> pointIdx;

		if(illegalEdge.size()<=i)
			i=0;
		Vec2i edge=illegalEdge[i];
		
		// illegal한 edge를 포함하는 triangle들을 찾는다
		for(int j=0;j<triangles.size();j++)
		{
			if(func.isTriangleContainEdge(illegalEdge[i], triangles[j]))
			{
				triIdx.push_back(j);
				for(int k=0;k<3;k++)
				{
					if((triangles[j][k]!=edge[0])&&(triangles[j][k]!=edge[1]))
					{
						pointIdx.push_back(triangles[j][k]);
						break;
					}
				}
			}
		}

		if(triIdx.size()==2)
		{
			// 새로 생기는 edge가 illegal edge인지 검사한다.
			Vec2i _edge(pointIdx[0], pointIdx[1]);
			bool flag=true;
			for(int j=0;j<illegalEdge.size();j++)
			{
				if(func.isEdgeSame(illegalEdge[j], _edge))
				{
					flag=false;
					break;
				}
			}
			if(flag)
			{
				triangles[triIdx[0]]=Vec3i(pointIdx[0], pointIdx[1], edge[0]);
				triangles[triIdx[1]]=Vec3i(pointIdx[0], pointIdx[1], edge[1]);
				illegalEdge[i]=illegalEdge[illegalEdge.size()-1];
				illegalEdge.pop_back();
				i--;
			}
		}
		else
		{
			illegalEdge[i]=illegalEdge[illegalEdge.size()-1];
			illegalEdge.pop_back();
			i--;
		}
		i++;
	}
}

int ReTriangulation::legalizeEdge(std::vector<Vec3f>& point, int pointIdx, Vec2i edge, int triIdx, std::vector<Vec3i>& triangles)
{
	GeometricFunc func;
	int _triIdx;
	int _pointIdx;
	bool flag=true;

	//1. find the triangle that contains the edge
	for(int i=0;i<triangles.size();i++)
	{
		if(func.isTriangleContainEdge(edge, triangles[i]))
		{
			if(i!=triIdx)
			{
				_triIdx=i;
				for(int j=0;j<3;j++)
				{
					if((triangles[i][j]!=edge[0])&&(triangles[i][j]!=edge[1]))
					{
						_pointIdx=triangles[i][j];
						break;
					}
				}
				flag=false;
				break;
			}
		}
	}
	if(flag)
	{
		return 0;
	}
	else
	{
		Vec3f faceNormal=func.computeNormal(point, triangles[_triIdx]);
		Vec3f circumCenter=func.computeCircumcenter(point, triangles[_triIdx], faceNormal);
		float radius=(point[edge[0]]-circumCenter).norm();

		if((point[pointIdx]-circumCenter).norm()<radius)
		{
			Vec3i face1=Vec3i(pointIdx, _pointIdx, edge[0]);
			Vec3i face2=Vec3i(pointIdx, _pointIdx, edge[1]);
			Vec3f normal1=func.computeNormal(point, face1);
			if(normal1*faceNormal<0)
			{
				int temp=face1[2];
				face1[2]=face1[1];
				face1[1]=temp;
			}
			else
			{
				int temp=face2[2];
				face2[2]=face2[1];
				face2[1]=temp;
			}

			//replace triangles
			triangles[triIdx]=face1;
			triangles[_triIdx]=face2;

			legalizeEdge(point, pointIdx, Vec2i(_pointIdx, edge[0]), triIdx, triangles);
			legalizeEdge(point, pointIdx, Vec2i(_pointIdx, edge[1]), _triIdx, triangles);
		}
	}
	return 0;
}

int ReTriangulation::legalizeEdge(std::vector<Vec3f>* point, int pointIdx, Vec2i edge, int triIdx, std::vector<Vec3i>& triangles)
{
	GeometricFunc func;
	int _triIdx;
	int _pointIdx;
	bool flag=true;

	//1. find the triangle that contains the edge
	for(int i=0;i<triangles.size();i++)
	{
		if(func.isTriangleContainEdge(edge, triangles[i]))
		{
			if(i!=triIdx)
			{
				_triIdx=i;
				for(int j=0;j<3;j++)
				{
					if((triangles[i][j]!=edge[0])&&(triangles[i][j]!=edge[1]))
					{
						_pointIdx=triangles[i][j];
						break;
					}
				}
				flag=false;
				break;
			}
		}
	}
	if(flag)
	{
		return 0;
	}
	else
	{
		Vec3f faceNormal=func.computeNormal(point, triangles[_triIdx]);
		Vec3f circumCenter=func.computeCircumcenter(point, triangles[_triIdx], faceNormal);
		float radius=((*point)[edge[0]]-circumCenter).norm();

		if(((*point)[pointIdx]-circumCenter).norm()<radius)
		{
			Vec3i face1=Vec3i(pointIdx, _pointIdx, edge[0]);
			Vec3i face2=Vec3i(pointIdx, _pointIdx, edge[1]);
			Vec3f normal1=func.computeNormal(point, face1);
			if(normal1*faceNormal<0)
			{
				int temp=face1[2];
				face1[2]=face1[1];
				face1[1]=temp;
			}
			else
			{
				int temp=face2[2];
				face2[2]=face2[1];
				face2[1]=temp;
			}

			//replace triangles
			triangles[triIdx]=face1;
			triangles[_triIdx]=face2;

			legalizeEdge(point, pointIdx, Vec2i(_pointIdx, edge[0]), triIdx, triangles);
			legalizeEdge(point, pointIdx, Vec2i(_pointIdx, edge[1]), _triIdx, triangles);
		}
	}
	return 0;
}