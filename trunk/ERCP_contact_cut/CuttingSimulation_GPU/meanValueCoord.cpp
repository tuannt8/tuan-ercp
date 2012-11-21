#include "StdAfx.h"
#include "meanValueCoord.h"
#include "releaseLog.h"


meanValueCoord::meanValueCoord(void)
{
	m_container = NULL;
}

meanValueCoord::~meanValueCoord(void)
{
	if (m_container)
	{
		delete m_container;
	}
}

// void meanValueCoord::init( TopologyContainer* container, arrayVec3f* points, arrayVec3i* face, 
// 						  arrayInt areaFaceIdxs, std::vector<pointInfo>& cutPoint )
// {
// 	arrayVec2i* edges = container->edge();
// 
// 	arrayInt areaPointIdxs; // Reference with point idx
// 	for (int i=0; i<areaFaceIdxs.size(); i++)
// 	{
// 		Vec3i curF = face->at(areaFaceIdxs[i]);
// 		areaPointIdxs.push_back(curF[0]);
// 		areaPointIdxs.push_back(curF[1]);
// 		areaPointIdxs.push_back(curF[2]);
// 	}
// 	VectorFunc func;
// 	func.arrangeVector(areaPointIdxs);
// 
// 	arrayVec3f areaPoint;
// 	for (int i=0; i<areaPointIdxs.size(); i++)
// 	{
// 		areaPoint.push_back(points->at(areaPointIdxs[i]));
// 	}
// 
// 	arrayVec3i areaFace;
// 	for (int i=0; i<areaFaceIdxs.size(); i++)
// 	{
// 		Vec3i oldF = face->at(areaFaceIdxs[i]);
// 		Vec3i newF;
// 		for (int j=0; j<3; j++)
// 		{
// 			newF[j] = func.indexOfElement(&areaPointIdxs, oldF[j]);
// 		}
// 		areaFace.push_back(newF);
// 	}
// 	init(areaPoint, areaFace);
// 
// 	// Sort the Cut front
// 	std::vector<arrayInt>* edgeOnFace = container->edgesInFace();
// 	std::vector<arrayInt> addedPointNeightbor;
// 	for (int i = 0; i < cutPoint.size(); i++)
// 	{
// 		arrayInt neighborIdx;
// 		int eIdxi = cutPoint[i].index;
// 		for (int j=0; j<cutPoint.size(); j++)
// 		{
// 			if(i==j)
// 				continue;
// 			int eIdxj = cutPoint[j].index;
// 			bool isNeighbor = false;
// 			for (int k=0; k<areaFaceIdxs.size(); k++)
// 			{
// 				arrayInt curF = edgeOnFace->at(areaFaceIdxs[k]);
// 				if ((eIdxi == curF[0] || eIdxi == curF[1] || eIdxi == curF[2])
// 					&&(eIdxj == curF[0] || eIdxj == curF[1] || eIdxj == curF[2]))
// 				{
// 					isNeighbor = true;
// 					break;
// 				}
// 			}
// 			if (isNeighbor)
// 			{
// 				neighborIdx.push_back(j);
// 			}
// 		}
// 		addedPointNeightbor.push_back(neighborIdx);
// 	}
// 
// 	arrayInt sortedPoints;
// 	for (int i=0; i<addedPointNeightbor.size(); i++)
// 	{
// 		if (addedPointNeightbor[i].size() == 1)
// 		{
// 			sortedPoints.push_back(i);
// 			break;
// 		}
// 	}
// 	while (sortedPoints.size()<cutPoint.size())
// 	{
// 		int curIdx = sortedPoints.back();
// 		int preIdx = sortedPoints.size()>1? sortedPoints[sortedPoints.size()-2]:-1;
// 		arrayInt curNeighbor = addedPointNeightbor[curIdx];
// 		for (int i=0; i<curNeighbor.size(); i++)
// 		{
// 			if (curNeighbor[i]!=preIdx)
// 			{
// 				sortedPoints.push_back(curNeighbor[i]);
// 				break;
// 			}
// 		}
// 	}
// 
// //	arrayVec2f holePoints;
// 	for (int i=0; i<sortedPoints.size(); i++)
// 	{
// 		Vec2i eIdx = (*edges)[cutPoint[sortedPoints[i]].index];
// 		int pIdx1 = func.indexOfElement(&areaPointIdxs, eIdx[0]);
// 		int pIdx2 = func.indexOfElement(&areaPointIdxs, eIdx[1]);
// 		Vec2f point2D = convertPointOnEdgeto2D(cutPoint[sortedPoints[i]].pos, pIdx1, pIdx2);
// 		holePoints.push_back(point2D);
// 	}
// }

void meanValueCoord::init( arrayVec3f points, arrayVec3i face )
{
	if (points.size() ==0)
	{
		return;
	}

	m_face = face;
	m_point = points;
	m_planarPoint.resize(m_point.size());

	VectorFunc func;
	m_edge.clear();
	m_container = new TopologyContainer();
	m_container->init(&m_point, &m_point, &m_face, &m_edge);

	// Neighbor weight
	findNeighborWeight();

	// Find boundary
	arrayInt boundaryPoint = findBoundary();
	m_nbPointOnBound = boundaryPoint.size();

	// Order points
	arrayInt orderedPointIdxs=boundaryPoint;
	for (int i=0; i<m_point.size(); i++)
	{
		if(!func.isElementInVector(boundaryPoint, i))
		{
			orderedPointIdxs.insert(orderedPointIdxs.begin(), 1, i);
		}
	}

	assignPlanarBoundary(orderedPointIdxs);
	assignPlanarInternalPoints(orderedPointIdxs);
}

arrayInt meanValueCoord::findBoundary()
{
	arrayInt edgeBoundary;
	arrayInt allBoundPoints;
	std::vector<arrayInt>* faceAroundEdge = m_container->facesAroundEdge();
	arrayVec2i* edge = m_container->edge();

	for (int i=0; i<faceAroundEdge->size(); i++)
	{
		if (faceAroundEdge->at(i).size()==1)
		{
			edgeBoundary.push_back(i);
			allBoundPoints.push_back((*edge)[i][0]);
			allBoundPoints.push_back((*edge)[i][1]);
		}
	}
	VectorFunc func;
	func.arrangeVector(allBoundPoints);

	arrayInt boundaryPoint;
	boundaryPoint.push_back(allBoundPoints[0]);
	allBoundPoints.erase(allBoundPoints.begin());
	while(allBoundPoints.size()>0)
	{
		int curP = boundaryPoint[boundaryPoint.size()-1];
		int preP = boundaryPoint.size()>1? boundaryPoint[boundaryPoint.size()-2]:-1;
		bool found = false;
		for (int i=0; i<allBoundPoints.size(); i++)
		{
			int checkP = allBoundPoints[i];
			if (checkP==curP || checkP==preP)
			{
				continue;
			}

			if (isEdge(curP, checkP, &edgeBoundary))
			{
				boundaryPoint.push_back(checkP);
				allBoundPoints.erase(allBoundPoints.begin()+i);
				found = true;
				break;
			}
		}
		ASSERT(found);
	}

	return boundaryPoint;
}

void meanValueCoord::draw( int mode )
{
	float RADIUS = 200;
	if (mode==1)// 3D face
	{
		glColor3f(1,0,0);
		glBegin(GL_TRIANGLES);
		for (int i=0; i<m_face.size(); i++)
		{
			for(int j=0; j<3; j++)
			{
				Vec3f curP = m_point[m_face[i][j]];
				glVertex3f(curP.x(), curP.y(), curP.z());
			}
		}
		glEnd();

		glColor3f(0,1,0);
		glBegin(GL_LINES);
		for (int i=0; i<m_edge.size(); i++)
		{
			for (int j=0; j<2; j++)
			{
				Vec3f curP = m_point[m_edge[i][j]];
				glVertex3f(curP.x(), curP.y(), curP.z());
			}
		}
		glEnd();

		glColor3f(0,0,1);
		for (int i=0; i<m_point.size(); i++)
		{
			Vec3f curP = m_point[i];
			glPushMatrix();
			glTranslatef(curP.x(), curP.y(), curP.z());
			Utility::printw(0,0,0,("%d"),i);
			glPopMatrix();
		}
	}

	if (mode==2) // 2D face
	{
		glColor3f(1,0,0);
// 		glBegin(GL_LINES);
// 		for (int i=0; i<m_edge.size(); i++)
// 		{
// 			for (int j=0; j<2; j++)
// 			{
// 				Vec2f curP = m_planarPoint[m_edge[i][j]];
// 				glVertex3f(RADIUS*curP[0], RADIUS*curP[1], 0.0);
// 			}
// 		}
// 		glEnd();

		glColor3f(0,1,0);
		glBegin(GL_TRIANGLES);
		for (int i=0; i<m_face.size(); i++)
		{
			for(int j=0; j<3; j++)
			{
				Vec2f curP = m_planarPoint[m_face[i][j]];
				glVertex3f(RADIUS*curP[0], RADIUS*curP[1], 0.0);
			}
		}
		glEnd();

		glColor3f(0,0,1);
		for (int i=0; i<m_planarPoint.size(); i++)
		{
			Vec2f curP = m_planarPoint[i];
			glPushMatrix();
			glTranslatef(RADIUS*curP[0], RADIUS*curP[1], 0.0);
			Utility::printw(0,0,0,("%d"),i);
			glPopMatrix();
		}

		glColor3f(0,0,1);
		glBegin(GL_LINES);
		for (int i=0; i<holePoints.size()-1; i++)
		{
			Vec2f curP = holePoints[i];
			glVertex3f(RADIUS*curP[0], RADIUS*curP[1], 0.0);	
			curP = holePoints[i+1];
			glVertex3f(RADIUS*curP[0], RADIUS*curP[1], 0.0);	
		}
		glEnd();

		glColor3f(1,0,0);
		for (int i=0; i<holePoints.size(); i++)
		{
			Vec2f curP = holePoints[i];
			glPushMatrix();
			glTranslatef(RADIUS*curP[0], RADIUS*curP[1], 0.0);
			Utility::printw(0,0,0,("%d"),i);
			glPopMatrix();
		}
	}
} 

void meanValueCoord::findNeighborWeight()
{
	std::vector<arrayInt>* pointAroundPoint = m_container->pointsAroundPoint();
	VectorFunc func;
	for (int i=0; i<m_point.size(); i++)
	{
		Vec3f centerP = m_point[i];
		arrayInt aroundPointIdx = (*pointAroundPoint)[i];
		std::vector<float> weight;
		float totalWeight=0.0;
		for (int j =0; j<m_point.size(); j++)
		{
			float w=0.0;
			if (func.isElementInVector(aroundPointIdx, j))
			{
				int curP = j;
				for (int k=0; k<aroundPointIdx.size(); k++)
				{
					int nIdx = aroundPointIdx[k];
					if (curP != nIdx)
					{
						float alp = GeometricFunc::angleBtwVector(m_point[curP]-centerP, m_point[nIdx]-centerP);
						w+=tan(alp/2.0);
					}
				}
				w/=(m_point[curP]-centerP).norm();
			}
			weight.push_back(w);
			totalWeight+=w;
		}
		for (int j=0; j<weight.size(); j++)
		{
			weight[j]/=totalWeight;
		}

		m_weightNeighbor.push_back(weight);
	}
}

bool meanValueCoord::isEdge( int curP, int checkP , arrayInt* edgeIdx)
{
	VectorFunc func;
	bool isEdge = false;
	for (int i=0; i<m_edge.size(); i++)
	{
		if (!(edgeIdx && func.isElementInVector(edgeIdx, i)))
		{
			continue;
		}
		if ((m_edge[i][0]==curP && m_edge[i][1]==checkP)
			||(m_edge[i][1]==curP && m_edge[i][0]==checkP))
		{
			isEdge = true;
			break;
		}
	}
	return isEdge;
}

void meanValueCoord::assignPlanarBoundary( arrayInt orderedPointIdxs )
{
	int totalPoint=m_point.size();
	std::vector<float> edgeLength;
	float totalLenght=0.0;
	for (int i=totalPoint-m_nbPointOnBound; i<totalPoint; i++)
	{
		Vec3f pti = m_point[orderedPointIdxs[i]];
		Vec3f pti1 = m_point[orderedPointIdxs[(i+1)%m_nbPointOnBound]];
		edgeLength.push_back((pti1-pti).norm());
		totalLenght+=edgeLength.back();
	}

	int startIdx = totalPoint-m_nbPointOnBound;
	m_planarPoint[orderedPointIdxs[startIdx]] = Vec2f(1.0, 0.0);
	float curAngle=0;
	for (int i=startIdx+1; i<totalPoint; i++)
	{
		curAngle += edgeLength[i-startIdx-1]*2*PI/totalLenght;
		Vec2f newP(0.5+0.5*cos(curAngle), 0.5+0.5*sin(curAngle));
		m_planarPoint[orderedPointIdxs[i]] = newP;
	}
}

void meanValueCoord::assignPlanarInternalPoints( arrayInt orderedPointIdxs )
{
	int totalPoint=m_point.size();
	int NbInternalPoinr = totalPoint-m_nbPointOnBound;
	
	matrix A(NbInternalPoinr, NbInternalPoinr);
	matrix bu(NbInternalPoinr,1);
	matrix bv(NbInternalPoinr,1);

	std::vector<arrayInt>* pointAroundPoint = m_container->pointsAroundPoint();
	for (int i=0; i<NbInternalPoinr; i++)
	{
		std::vector<float> neighborW = m_weightNeighbor[orderedPointIdxs[i]];
		A(i,i)+=1;
		for (int j=0; j<NbInternalPoinr; j++)
		{
			A(i,j) += -neighborW[orderedPointIdxs[j]];
		}
		for (int j=NbInternalPoinr; j<totalPoint; j++)
		{
			bu(i,0) += neighborW[orderedPointIdxs[j]]*m_planarPoint[orderedPointIdxs[j]][0];
			bv(i,0) += neighborW[orderedPointIdxs[j]]*m_planarPoint[orderedPointIdxs[j]][1];
		}
	}
	
	matrix invertA = A.inverse();

	matrix u = invertA*bu;
	matrix v = invertA*bv;

	// Assign to list
	for (int i=0; i<NbInternalPoinr; i++)
	{
		m_planarPoint[orderedPointIdxs[i]] = Vec2f(u(i,0), v(i,0));
	}
}

Vec2f meanValueCoord::convertPointOnEdgeto2D( Vec3f point, int pIdx1, int pIdx2 )
{
	float l = (point-m_point[pIdx1]).norm()/(m_point[pIdx2]-m_point[pIdx1]).norm();
	return m_planarPoint[pIdx1] + (m_planarPoint[pIdx2]-m_planarPoint[pIdx1])*l;
}

Vec3f meanValueCoord::convert2to3( Vec2f point )
{
	// Find which triangle the point lie in

	// Interpolate 3d position by are coordinate
	return Vec3f();
}
