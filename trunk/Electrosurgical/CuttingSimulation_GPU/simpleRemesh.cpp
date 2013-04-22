#include "StdAfx.h"
#include "simpleRemesh.h"

#define MAX 100000000
#define MIN -100000000

simpleRemesh::simpleRemesh(void)
{
}

simpleRemesh::~simpleRemesh(void)
{
}

void simpleRemesh::remeshArea( SurfaceObj* obj, arrayInt& remeshAreaIdxs, float res)
{
	arrayInt curTriIdx = remeshAreaIdxs;
	float maxLength= maxEdgeLength(obj, curTriIdx);
	while(maxLength > res)
	{
		remesh(obj, curTriIdx, maxLength/2.0);
		curTriIdx = newFaceIdx;
		maxLength = maxEdgeLength(obj, curTriIdx);
	}
}


float simpleRemesh::maxEdgeLength( SurfaceObj* obj, arrayInt remeshAreaIdxs )
{
	TopologyContainer* container = obj->container();
	arrayVec3i* face = container->face();
	points = container->point();

	// Find appropriate length
	float maxLength=0.0;
	for (int i=0; i<remeshAreaIdxs.size(); i++)
	{
		Vec3i curFace = face->at(remeshAreaIdxs[i]);
		for (int j=0; j<3; j++)
		{
			Vec3f pt1 = points->at(curFace[j]);
			Vec3f pt2 = points->at(curFace[(j+1)%3]);

			if ((pt1-pt2).norm() > maxLength)
			{
				maxLength = (pt1-pt2).norm();
			}
		}
	}

	return maxLength;
}

void simpleRemesh::smoothArea( SurfaceObj* obj, arrayInt remeshAreaIdxs )
{
	float maxLength= maxEdgeLength(obj, remeshAreaIdxs);
	remesh(obj, remeshAreaIdxs, maxLength/2.0);
}

void simpleRemesh::remesh(SurfaceObj* obj,  arrayInt remeshAreaIdxs,  float maxEdgeLength )
{
	if (remeshAreaIdxs.size()==0)
	{
		return;
	}
	bool bRoughFace = true;
	float roughNess = 1.0;

	TopologyContainer* container = obj->container();

	arrayVec3i* face = container->face();
	arrayVec2i* edge = container->edge();
	arrayVec3f* point0 = container->point0();
	points = container->point();
	int preNbPoint = points->size();
	std::vector<arrayInt> *edgeOnFace = container->edgesInFace();
	std::vector<arrayInt> *faceAroundEdge = container->facesAroundEdge();

	removedEdge.clear();
	addedFaceAround.clear();
	addedFace.clear();
	addedPoint.clear();
	addedPoint0.clear();
	addedPtOnEdge.clear();
	
	// 1. Add point to long edge
	addedPtOnEdge.resize(edge->size());
	for (int i=0; i<remeshAreaIdxs.size(); i++)
	{
		arrayInt eIdxs = edgeOnFace->at(remeshAreaIdxs[i]);
		for (int j=0; j<eIdxs.size(); j++)
		{
			if (addedPtOnEdge[eIdxs[j]].size() > 0)
			{
				continue;
			}

			Vec2i curE = (*edge)[eIdxs[j]];
			Vec3f pt1 = points->at(curE[0]);
			Vec3f pt2 = points->at(curE[1]);
			Vec3f pt10 = point0->at(curE[0]);
			Vec3f pt20 = point0->at(curE[1]);

			if ((pt2-pt1).norm() > 1.5*maxEdgeLength) // We should add point to this edge
			{
				int nbNewPt = ceil((pt2-pt1).norm() / maxEdgeLength);
				arrayInt newPtIdx;
				for (int k=0; k<nbNewPt; k++)
				{
					Vec3f newP = pt1+(pt2-pt1)*(k+1.0)/(nbNewPt+1.0);
					Vec3f newP0 = pt10+(pt20-pt10)*(k+1.0)/(nbNewPt+1.0);
					addedPoint.push_back(newP);
					addedPoint0.push_back(newP0);
					newPtIdx.push_back(addedPoint.size()+preNbPoint-1);
				}
				addedPtOnEdge[eIdxs[j]] = newPtIdx;
			}
		}
	}

	TopologyModifier modifier;
	modifier.addPoints(container, &addedPoint0, &addedPoint);

	// 2. Triangulate
	arrayInt removedFace;
	arrayInt markedFace(face->size(), 0);

	VectorFunc func;
	for (int i=0; i<addedPtOnEdge.size(); i++)
	{
		if (addedPtOnEdge[i].size()>0)
		{
			arrayInt faceAround = (*faceAroundEdge)[i];
			for (int j=0; j<faceAround.size(); j++)
			{
				if (markedFace[faceAround[j]] == 1)
				{
					continue;
				}
				markedFace[faceAround[j]] = 1;
				removedFace.push_back(faceAround[j]);	

				arrayInt allPtIdx;

				Vec3i curFace = face->at(faceAround[j]);
				arrayInt eIdxs = edgeOnFace->at(faceAround[j]);
				Vec3f norm = GeometricFunc::computeNormal(points, curFace);

				// Add corner point
				allPtIdx.push_back(curFace[0]);
				allPtIdx.push_back(curFace[1]); 
				allPtIdx.push_back(curFace[2]); 
				// points on 3 edge
				
				for (int j=0; j<eIdxs.size(); j++)
				{
					Vec2i curE = edge->at(eIdxs[j]);
					arrayInt newPIdxs = addedPtOnEdge[eIdxs[j]];
					for (int k=0; k<newPIdxs.size(); k++)
					{
						allPtIdx.push_back(newPIdxs[k]);
					}
				}

				arrayVec3i newFaces;
				delaunayTriangulate(allPtIdx, newFaces, norm);

				int idx = func.indexOfElement(&remeshAreaIdxs, faceAround[j]);
				if (idx==-1)
				{
					addedFaceAround.insert(addedFaceAround.end(), newFaces.begin(), newFaces.end());
				}
				else
					addedFace.insert(addedFace.end(), newFaces.begin(), newFaces.end());
			}
		}
	}

	// Make rough face
// 	if (bRoughFace)
// 	{
// 		for (int i=0; i< addedPtOnEdge.size(); i++)
// 		{
// 			if (addedPtOnEdge[i].size()>0)
// 			{
// 				Vec3f eNorm0, eNorm;
// 				Vec3f eTangen0, eTangen;
// 				{
// 					arrayInt faceAround = faceAroundEdge->at(i);
// 					for (int k=0; k<faceAround.size(); k++)
// 					{
// 						eNorm0 += GeometricFunc::computeNormal(point0, face->at(faceAround[k]));
// 						eNorm += GeometricFunc::computeNormal(points, face->at(faceAround[k]));
// 					}
// 					eNorm.normalize();
// 					eNorm0.normalize();
// 
// 					eTangen = eNorm.cross(points->at(edge->at(i)[0]) - points->at(edge->at(i)[1]));
// 					eTangen0 = eNorm0.cross(point0->at(edge->at(i)[0]) - point0->at(edge->at(i)[1]));
// 					eTangen.normalize();eTangen0.normalize();
// 				}
// 				arrayInt addedPIdx = addedPtOnEdge[i];
// 				for (int j=0; j<addedPIdx.size(); j++)
// 				{
// 					Vec3f& pt = points->at(addedPIdx[j]);
// 					Vec3f& pt0 = point0->at(addedPIdx[j]);
// 
// 					float ad = (float)rand()*roughNess/RAND_MAX-roughNess/2;
// 					pt+= eNorm*ad;
// 					pt0+= eNorm0*ad;
// 
// 					float ad1 = (float)rand()*roughNess/RAND_MAX-roughNess/2;
// 					pt+= eTangen*ad1;
// 					pt0+= eTangen0*ad1;
// 				}
// 			}
// 		}
// 	}


	func.arrangeVector(removedFace); 


	// 0. removed edge
	for (int i=0; i<removedFace.size(); i++)
	{
		arrayInt es = edgeOnFace->at(removedFace[i]);
		for (int j=0; j<es.size(); j++)
		{
			arrayInt faceArounf = faceAroundEdge->at(es[j]);
			int otherFace = faceArounf[0]==removedFace[i]? faceArounf[1]:faceArounf[0];
			//Check if this edge wil be removed
			for (int k=0; k<removedFace.size(); k++)
			{
				if (otherFace==removedFace[k])
				{
					removedEdge.push_back(es[j]);
					break;
				}
			}
		}
	}
	func.arrangeVector(removedEdge);

	// Update
	newFaceIdx = remeshAreaIdxs;
	for (int i=removedFace.size()-1; i>=0; i--)
	{
		int fIdx = removedFace[i];
		int swapIdx = face->size()-1;
		modifier.removeFace(container, removedFace[i]);

		int rIdx = func.indexOfElement(&newFaceIdx, fIdx);
		if(rIdx>=0)
			newFaceIdx.erase(newFaceIdx.begin()+rIdx);

		int sIdx = func.indexOfElement(&newFaceIdx, swapIdx);
		if (sIdx>=0)
			newFaceIdx[sIdx] = fIdx;
	}
	//modifier.removeFaces(container, removedFace);
	modifier.removeEdges(container, removedEdge);
	int nbFace = face->size();
	modifier.addFaces(container, addedFace, preNbPoint);
	int nbFace2 = face->size();
	modifier.addFaces(container, addedFaceAround, preNbPoint);


	for (int i=nbFace; i<nbFace2; i++)
	{
		newFaceIdx.push_back(i);
	}

	// Update shape function
	newPointIdx.clear();
	for (int i=preNbPoint; i<points->size(); i++)
	{
		newPointIdx.push_back(i);
	}
	func.arrangeVector(newPointIdx);

	obj->updateNormal();
}


void simpleRemesh::updateShapeFunc( Meshfree_GPU* obj_GPU, arrayInt pointIdx )
{
	VectorFunc func;
	GeometricFunc geoFunc;
	CollisionManager collision;

	// Surface object data
	SurfaceObj* surfObj=obj_GPU->surfObj();
	std::vector<Vec3f>* surfNode=surfObj->point();
	std::vector<Vec3i>* surfFace=surfObj->face();
	float supportRadius=obj_GPU->supportRadiusSurf();

	// Meshfree object data
	std::vector<Vec3f>* efgNode=obj_GPU->efgObj()->nodePosVec();

	// Meshfree object data
	std::vector<std::vector<int>>* neighborNodeIdx=obj_GPU->neighborNodeOfSurfVertice();
	std::vector<int> updatedPointIdx;
	neighborNodeIdx->resize(surfNode->size());

	for (int i=0; i<pointIdx.size(); i++)
	{
		int pIdx=pointIdx[i];

		Vec3f l1 = (*surfNode)[pIdx];

		std::vector<int> colNodeIdx;
		collision.PQBtwBoxAndPoint(obj_GPU->efgObj()->getBVHPoint()->root(), efgNode, l1, colNodeIdx, supportRadius);
		VectorFunc func;
		func.arrangeVector(colNodeIdx);

		arrayInt neighbor;
		// Find start point to recursive
		Vec3f normV = surfObj->pointNormal()->at(pIdx);
		for (int i=0; i<colNodeIdx.size(); i++)
		{
			if ((efgNode->at(colNodeIdx[i])-l1)*normV < 0)
			{
				neighbor.push_back(colNodeIdx[i]);
				break;
			}
		}
		
		if (neighbor.size()>0)
		{
			addNeighbor(obj_GPU, neighbor, colNodeIdx, neighbor[0]);
		}
		else
		{
			throw std::exception("Error when update neighbor of new node");
		}


		for(int j=0;j<neighbor.size();j++)
		{
		//	releaseLog::log("Update shapeFunc node: %d <- %d", pointIdx[i], neighbor[j]);
			(*neighborNodeIdx)[pIdx].push_back(neighbor[j]);
		}
	}



	// Update shape function at surface point
	obj_GPU->updateShapeFunction(pointIdx);
	surfObj->pointNormal()->resize(surfNode->size());
	surfObj->faceNormal()->resize(surfFace->size());
	surfObj->updateNormal();
	surfObj->dis()->resize(surfNode->size());
}


void simpleRemesh::addNeighbor( Meshfree_GPU* obj_GPU, arrayInt& neighbor, arrayInt &potentialNeibor, int pIdx )
{
	arrayVec2i* efgEdge = obj_GPU->efgObj()->edge();
	arrayInt newNeighbor;
	for (int i=0; i<efgEdge->size(); i++)
	{
		Vec2i curE = efgEdge->at(i);
		if (curE[0]!= pIdx && curE[1]!=pIdx)
		{
			continue;
		}
		int curIdx = curE[0]==pIdx? curE[1]:curE[0];

		bool bInPotentialList = false;
		for(int i=0; i<potentialNeibor.size(); i++)
		{
			if (curIdx == potentialNeibor[i])
			{
				bInPotentialList = true;
			}
		}
		if (!bInPotentialList)
		{
			continue;
		}
		bool existed = false;
		for (int j=0; j<neighbor.size(); j++)
		{
			if (curIdx == neighbor[j])
			{
				existed = true;
				break;
			}
		}
		if (!existed)
		{
			newNeighbor.push_back(curIdx);
			neighbor.push_back(curIdx);
		}
	}

	for (int i =0; i < newNeighbor.size(); i++)
	{
		addNeighbor(obj_GPU, neighbor, potentialNeibor, newNeighbor[i]);
	}
}

void simpleRemesh::delaunayTriangulate(arrayInt allPtIdx_org, arrayVec3i& newFace, Vec3f norm)
{
	arrayInt allPtIdx = allPtIdx_org;
	//1. Find boundary loop
	VectorFunc func;
	GeometricFunc geoFunc;
	arrayInt boundLoop;
	boundLoop.push_back(allPtIdx[0]);
	allPtIdx.erase(allPtIdx.begin());
	while(allPtIdx.size()>0)
	{
		int curIdx = boundLoop.back();
		int preIdx = boundLoop.size()>2? boundLoop[boundLoop.size()-2]:-1;

		bool found = false;
		for (int i=0; i<allPtIdx.size(); i++)
		{
			if (allPtIdx[i]==curIdx || allPtIdx[i]==preIdx)
			{
				continue;
			}
			if (isNeighbor(allPtIdx_org, allPtIdx[i], curIdx))
			{
				boundLoop.push_back(allPtIdx[i]);
				allPtIdx.erase(allPtIdx.begin()+i);
				found = true;
				break;
			}
		}
		CHECK(found, "simpleRemesh::delaunayTriangulate2");
	}
	// invert if order is incorrect
	if (!isCCWOrder(boundLoop, norm))
	{
		std::reverse(boundLoop.begin(), boundLoop.end());
	}

	//2. Triangulate by using ear-triangle
	while(boundLoop.size()>3)
	{
		//Find ear
		int nbPoint = boundLoop.size();
		bool found = false;
		int minIdx = -1;
		float minVar = 9999;
		for (int i=0; i<nbPoint; i++)
		{
			Vec3i newF(boundLoop[(i-1+nbPoint)%nbPoint], boundLoop[i], boundLoop[(i+1)%nbPoint]);
			if (isEarTri(boundLoop, newF, norm))
			{
				if (minVar > varAngle(newF))
				{
					minIdx = i;
					minVar = varAngle(newF);
				}
			}
		}
		
		if (minIdx != -1)
		{
			Vec3i newF(boundLoop[(minIdx-1+nbPoint)%nbPoint], boundLoop[minIdx], boundLoop[(minIdx+1)%nbPoint]);
			newFace.push_back(newF);
			boundLoop.erase(boundLoop.begin()+minIdx);
		}
		else
			CHECK(0, "simpleRemesh::delaunayTriangulate3");
	}
	newFace.push_back(Vec3i(boundLoop[0], boundLoop[1], boundLoop[2]));
}

void simpleRemesh::convertToGloabalIdx( arrayVec3i& face, arrayInt& pairIdx )
{
	for (int i=0 ;i<face.size(); i++)
	{
		Vec3i curF = face[i];
		curF[0] = pairIdx[curF[0]];
		curF[1] = pairIdx[curF[1]];
		curF[2] = pairIdx[curF[2]];
		face[i] = curF;
	}
}

bool simpleRemesh::isNeighbor(arrayInt allPtIdx, int idx1, int idx2 )
{
	GeometricFunc func;
	for (int i =0; i<3; i++)
	{
		Vec3f l1 = points->at(allPtIdx[i]);
		Vec3f l2 = points->at(allPtIdx[(i+1)%3]);

		if (func.isPointInLine(l1, l2, points->at(idx1)) || allPtIdx[i]==idx1 || allPtIdx[(i+1)%3]==idx1 )
		{
			if (func.isPointInLine(l1, l2, points->at(idx2)) || allPtIdx[i]==idx2 || allPtIdx[(i+1)%3]==idx2)
			{
				for(int j=0; j<allPtIdx.size(); j++)
				{
					if (allPtIdx[j]==idx1 || allPtIdx[j]==idx2)
					{
						continue;
					}
					if (func.isPointInLine(points->at(idx1), points->at(idx2), points->at(allPtIdx[j])))
					{
						return false;
					}
				}
				return true;
			}
		}
	}
	return false;
}

bool simpleRemesh::isCCWOrder(arrayInt polygonIdxs, Vec3f normF)
{
	Vec3f norm;
	for (int i=0; i<polygonIdxs.size(); i++)
	{
		Vec3f pt1 = (*points)[polygonIdxs[i]];
		Vec3f pt2 = (*points)[polygonIdxs[(i+1)%polygonIdxs.size()]];
		norm+= pt1.cross(pt2);
	}
	return norm*normF > 0;
}

bool simpleRemesh::isCCWOrder( Vec3i tri, Vec3f normF )
{
	Vec3f norm;
	for (int i=0; i<3; i++)
	{
		Vec3f pt1 = (*points)[tri[i]];
		Vec3f pt2 = (*points)[tri[(i+1)%3]];
		norm+= pt1.cross(pt2);
	}
	return norm*normF > 0;
}

bool simpleRemesh::isEarTri(arrayInt boundLoop, Vec3i pt, Vec3f normF )
{
	Vec3f norm;
	for (int i=0; i<3; i++)
	{
		Vec3f pt1 = (*points)[pt[i]];
		Vec3f pt2 = (*points)[pt[(i+1)%3]];
		norm+= pt1.cross(pt2);
	}
	if(norm*normF > 0)
	{
		GeometricFunc func;
		Vec3f l1 = (*points)[pt[0]];
		Vec3f l2 = (*points)[pt[2]];
		// No point lie between p02
		for (int i=0; i<boundLoop.size(); i++)
		{
			if (func.isPointInLine(l1, l2, points->at(boundLoop[i])))
			{
				return false;
			}
		}
		return true;
	}
	return false;
}

float simpleRemesh::varAngle( Vec3i tri)
{
	float var = 0.0;
	float mid = PI/3;
	for (int i=0; i<3; i++)
	{
		Vec3f pt1 = points->at(tri[(i-1+3)%3]) - points->at(tri[i]);
		Vec3f pt2 = points->at(tri[(i+1)%3]) - points->at(tri[i]);
		pt1.normalize();
		pt2.normalize();
		float alp = acos((pt1*pt2));
		var += (alp - mid)*(alp - mid);
	}
	return var;
}

void simpleRemesh::removeEarTri( SurfaceObj* obj, arrayInt& triArea )
{
	TopologyContainer* surfTopo = obj->container();
	std::vector<arrayInt>* faceAroundEdge = surfTopo->facesAroundEdge();
	std::vector<arrayInt>* edgeOnFace = surfTopo->edgesInFace();
	arrayVec2i* edges = surfTopo->edge();
	arrayVec3f* points = surfTopo->point();
	VectorFunc func;

	// 1. edge mesh
	arrayInt edgeOnBuondary;
	arrayInt edgeRemove;

	arrayInt allEdge;
	for (int i=0; i<triArea.size(); i++)
	{
		arrayInt edgeIn = edgeOnFace->at(triArea[i]);
		for (int j=0; j<edgeIn.size(); j++)
		{
			allEdge.push_back(edgeIn[j]);
		}
	}

	while(allEdge.size()>0)
	{
		int curE = allEdge[0];
		allEdge.erase(allEdge.begin());
		bool found = false;
		for (int i=0; i<allEdge.size(); i++)
		{
			if (curE == allEdge[i])
			{
				found = true;
				edgeRemove.push_back(curE);
				allEdge.erase(allEdge.begin()+i);
				break;
			}
		}
		if (!found)
		{
			edgeOnBuondary.push_back(curE);
		}
	}

	while(1)
	{
		bool hasRemove = false;
		for (int i=0; i<triArea.size(); i++)
		{
			int fIdx = triArea[i];
			arrayInt edgeIn = edgeOnFace->at(fIdx);
			int count = 0;
			int edgeb[3];
			for (int j=0; j<edgeIn.size(); j++)
			{
				bool found = false;
				for (int k=0; k<edgeOnBuondary.size(); k++)
				{
					if (edgeIn[j]==edgeOnBuondary[k])
					{
						edgeb[count] = edgeIn[j];
						count++;
						found = true;
						break;
					}
				}
				if (!found)
				{
					edgeb[2] = edgeIn[j];
				}
			}
			if (count == 2) // ear, check if we can remove it
			{
				int eIdx1 = edgeb[0];
				int eIdx2 = edgeb[1];

				Vec2i e1 = edges->at(eIdx1);
				Vec2i e2 = edges->at(eIdx2);

				int pIdx = (e1[0]==e2[0] || e1[0]==e2[1])? e1[0]:e1[1];
				int pIdx1 = (e1[0]==e2[0] || e1[0]==e2[1])? e1[1]:e1[0];
				int pIdx2 = (e2[0]==e1[0] || e2[0]==e1[1])? e2[1]:e2[0];

				Vec3f l1 = points->at(pIdx1)-points->at(pIdx);l1.normalize();
				Vec3f l2 = points->at(pIdx2)-points->at(pIdx);l2.normalize();

				double alp = l1*l2;
				if (alp>cos(140*PI/180))//that it
				{
					hasRemove = true;
					triArea.erase(triArea.begin()+i);
					edgeOnBuondary.erase(edgeOnBuondary.begin()+func.indexOfElement(&edgeOnBuondary, eIdx1));
					edgeOnBuondary.erase(edgeOnBuondary.begin()+func.indexOfElement(&edgeOnBuondary, eIdx2));
					edgeOnBuondary.push_back(edgeb[2]);
					edgeRemove.erase(edgeRemove.begin() + func.indexOfElement(&edgeRemove, edgeb[2]));
					break;
				}
			}
			else if (count==3)
			{
				for (int j=0; j<3; j++)
					edgeOnBuondary.erase(edgeOnBuondary.begin()+func.indexOfElement(&edgeOnBuondary, edgeb[j]));
				triArea.erase(triArea.begin()+i);
				break;
			}
		}
		if (!hasRemove)
		{
			break;
		}
	}
}

void simpleRemesh::roughFaceProcess( SurfaceObj* obj, arrayInt areaTri, float rough )
{
	
}


void simpleRemesh::refineMesh( SurfaceObj* obj, arrayInt* refineArea )
{
	// Using basic algorithm: Edge collapse and edge flip
	TopologyContainer* container = obj->container();

	arrayVec3i* face = container->face();
	arrayVec2i* edge = container->edge();
	arrayVec3f* point0 = container->point0();
	points = container->point();
	int preNbPoint = points->size();
	std::vector<arrayInt> *edgeOnFace = container->edgesInFace();
	std::vector<arrayInt> *faceAroundEdge = container->facesAroundEdge();

	removedEdge.clear();
	addedFaceAround.clear();
	addedFace.clear();
	addedPoint.clear();
	addedPoint0.clear();
	addedPtOnEdge.clear();

	// Find least error edges
	arrayInt curTriIdxs = *refineArea;
	for (int i=0; i<curTriIdxs.size(); i++)
	{
		int pIdx;
		if (isSharpCorner(points, face->at(curTriIdxs[i]), pIdx))
		{
			arrayInt removeIdxs;
			arrayVec3i addedFace;
			int edgeIdx = findOppositeEdgeIdx(obj, curTriIdxs[i], pIdx);
			ASSERT(edgeIdx!=-1);
			// Collapse to boundary point if possible
			// Do not collapse if edge on boundary??
			collapseEdge(obj, edgeIdx, removeIdxs,addedFace);

			objTopoModifer(obj, curTriIdxs, removeIdxs, addedFace);

			break;
		}
	}
	*refineArea = curTriIdxs;
}
#define EDGE_RATIO_THRES 0.5
bool simpleRemesh::isSharpCorner( arrayVec3f* vPoints, Vec3i tri, int &pIdx )
{
	// Now the weight function is simply the edge length
	// Later add affect to mesh weight if necessary
	float length[3];
	for (int i=0; i<3; i++)
	{
		length[i] = (vPoints->at(tri[(i+1)%3]) - vPoints->at(tri[(i+2)%3])).norm();
	}

	for (int i=0; i<3; i++)
	{
		if (length[i] < EDGE_RATIO_THRES*length[(i+1)%3]
		&& length[i] < EDGE_RATIO_THRES*length[(i+2)%3])
		{
			pIdx = tri[i];
			return true;
		}
	}

	return false;
}




void simpleRemesh::objTopoModifer(SurfaceObj* surfObj, arrayInt &generatedIdx, arrayInt _removedTriIdx, arrayVec3i _addFace, arrayVec3f* _addPoint /*= NULL*/ )
{
	AABBTreeTri* BVH = (AABBTreeTri*)surfObj->getBVH();
	TopologyModifier modifier;
	TopologyContainer* topo = surfObj->container();
	arrayVec3f* objPoints = surfObj->point();
	arrayVec3i* objTris = surfObj->face();
	int _preNbPoint = objPoints->size();
	VectorFunc vecFunc;

	// Edge will be removed
	VectorFunc func;
	arrayInt alledgeRemove;
	std::vector<arrayInt> boundLoops;
	findBoundaryLoop(topo, _removedTriIdx, boundLoops, &alledgeRemove);
	func.arrangeVector(alledgeRemove);

	if (_addPoint)
		modifier.addPoints(topo, *_addPoint);

	// Remove face
	for (int i=_removedTriIdx.size()-1;i>=0;i--)
	{
		int rmIdx = _removedTriIdx[i];
		int changeIdx = objTris->size()-1;


		// Update BVH
		AABBNode* node = BVH->findLeafNode(changeIdx);
		if (node)
			node->IndexInLeafNode = rmIdx;
		node = BVH->findLeafNode(rmIdx);
		BVH->removeNode(node);

		modifier.removeFace(topo, _removedTriIdx[i]);

		// update affected tri list
		if (vecFunc.isElementInVector(generatedIdx, rmIdx))
		{
			vecFunc.removeElementValue(generatedIdx, rmIdx);
		}
		int idx = vecFunc.indexOfElement(generatedIdx, changeIdx);
		if (idx>=0)
		{
			generatedIdx[idx] = rmIdx;
		}

	}

	// Remove edge -- Depend on specific type, we should change this pecie of code
	modifier.removeEdges(topo, alledgeRemove);

	// add faces
	int nbFace = objTris->size();
	for (int i=0; i<_addFace.size(); i++)
	{
		modifier.addFace(topo, _addFace[i], _preNbPoint);
		AABBNode* newNode = new AABBNode;

		newNode->IndexInLeafNode = nbFace+i;
		Vec3f leftDown=Vec3f(MAX,MAX,MAX);
		Vec3f rightUp=Vec3f(MIN,MIN,MIN);
		Vec3i curTri = _addFace[i];
		for (int j=0; j<3; j++)
		{
			for (int k=0; k<3; k++)
			{
				if((*objPoints)[curTri[j]][k]<leftDown[k])
					leftDown[k]=(*objPoints)[curTri[j]][k];
				if((*objPoints)[curTri[j]][k]>rightUp[k])
					rightUp[k]=(*objPoints)[curTri[j]][k];
			}
		}
		newNode->setBoundingBox(leftDown, rightUp);
		BVH->addNode(newNode);

		// Update affected triangle list
		generatedIdx.push_back(objTris->size()-1);
	}

	surfObj->updateNormal();
	surfObj->updateBVH(); // This can be optimized
}

bool simpleRemesh::findBoundaryLoop( TopologyContainer* surfTopo, arrayInt idxOfRemoveTri, std::vector<arrayInt>& boundLoops, arrayInt *edgeToRemove)
{
	std::vector<arrayInt>* faceAroundEdge = surfTopo->facesAroundEdge();
	std::vector<arrayInt>* edgeOnFace = surfTopo->edgesInFace();
	arrayVec2i* edges = surfTopo->edge();
	VectorFunc func;

	// 1. edge mesh
	arrayInt edgeOnBuondary;
	arrayInt edgeRemove;


	arrayInt allEdge;
	for (int i=0; i<idxOfRemoveTri.size(); i++)
	{
		arrayInt edgeIn = edgeOnFace->at(idxOfRemoveTri[i]);
		for (int j=0; j<edgeIn.size(); j++)
		{
			allEdge.push_back(edgeIn[j]);
		}
	}

	while(allEdge.size()>0)
	{
		int curE = allEdge[0];
		allEdge.erase(allEdge.begin());
		bool found = false;
		for (int i=0; i<allEdge.size(); i++)
		{
			if (curE == allEdge[i])
			{
				found = true;
				edgeRemove.push_back(curE);
				allEdge.erase(allEdge.begin()+i);
				break;
			}
		}
		if (!found)
		{
			edgeOnBuondary.push_back(curE);
		}
	}

	if (edgeToRemove)
		edgeToRemove->insert(edgeToRemove->end(), edgeRemove.begin(), edgeRemove.end());

	if (edgeOnBuondary.size()==0)
	{
		return true;//??
		return false;
	}

	// 2. Point mesh
	arrayInt pointsIdx;
	for (int i=0; i<edgeOnBuondary.size();i++)
	{
		pointsIdx.push_back((*edges)[edgeOnBuondary[i]][0]);
		pointsIdx.push_back((*edges)[edgeOnBuondary[i]][1]);
	}
	func.arrangeVector(pointsIdx);

	// 3. Find boundary loop
	while(pointsIdx.size()>0) // For entire split -> there will be 2 bound loop, but sometime, it is hole
	{
		arrayInt boundLoop;
		boundLoop.push_back(pointsIdx[0]);
		pointsIdx.erase(pointsIdx.begin());
		while(pointsIdx.size()>0)
		{
			int curP = boundLoop[boundLoop.size()-1];
			bool found = false;
			for (int i = 0; i<pointsIdx.size(); i++)
			{
				int checkIdx = pointsIdx[i];
				int index =-1;
				if (GeometricFunc::isEdge(curP, checkIdx, edges, index))
				{
					// index belong to edgeBound and not belong to remove edge
					if (func.isElementInVector(edgeOnBuondary, index) &&
						!func.isElementInVector(edgeRemove, index))
					{
						boundLoop.push_back(checkIdx);
						pointsIdx.erase(pointsIdx.begin()+i);
						found = true;
						break;					
					}
				}
			}
			int eIdx;
			if (boundLoop.size()>3 && GeometricFunc::isEdge(boundLoop[0], boundLoop.back(), edges, eIdx)
				&& func.isElementInVector(edgeOnBuondary, eIdx) &&
				!func.isElementInVector(edgeRemove, eIdx))
			{
				break;
			}
			if (!found)
			{
				return false;
			}
		}
		CHECK(boundLoop.size()>=3, "eSurfaceCutting::findBoundaryLoop3");
		conterClockWiseLoop(surfTopo, idxOfRemoveTri, boundLoop);
		boundLoops.push_back(boundLoop);
	}

	return true;
}

void simpleRemesh::conterClockWiseLoop( TopologyContainer* surfTopo, arrayInt areaTriIdx, arrayInt& boundPointLoop )
{
	int idx1 = boundPointLoop[0];
	int idx2 = boundPointLoop[1];

	bool correctOrder = true;

	int edgeIdx = -1;
	std::vector<arrayInt>* edgeAroundPoint = surfTopo->edgesAroundPoint();
	std::vector<Vec2i>* edges = surfTopo->edge();
	arrayVec3i* faces = surfTopo->face();
	arrayVec3f* points = surfTopo->point();
	arrayInt edgesAroundPt1 = (*edgeAroundPoint)[idx1];

	// edge 0-1
	for (int i  =0; i<edgesAroundPt1.size(); i++)
	{
		Vec2i curE = (*edges)[edgesAroundPt1[i]];
		if ( (curE[0]==idx1 && curE[1]==idx2)
			|| (curE[0]==idx2 && curE[1]==idx1) )
		{
			edgeIdx = edgesAroundPt1[i];
			break;
		}
	}

	// check if 0-1 is same order with the triangle inside
	arrayInt faceAroundE = surfTopo->facesAroundEdge(edgeIdx);
	for (int i =0; i<faceAroundE.size(); i++)
	{
		int triIdx = faceAroundE[i];
		for (int j = 0; j<areaTriIdx.size(); j++)
		{
			Vec3i triPoint = (*faces)[triIdx];
			if (triIdx == areaTriIdx[j])
			{
				// Check correct order
				Vec3f normTri = GeometricFunc::computeNormal(points, triPoint);

				Vec3f midPoint = ((*points)[triPoint[0]] + (*points)[triPoint[1]] + (*points)[triPoint[2]])/3;
				Vec3f normCheck = ((*points)[idx1] - midPoint).cross((*points)[idx2] - midPoint);
				correctOrder = normTri*normCheck > 0;
				break;
			}
		}
	}

	if (!correctOrder)
	{
		arrayInt temp = boundPointLoop;
		for (int i =0; i<boundPointLoop.size(); i++)
		{
			boundPointLoop[i] = temp[temp.size()-1 - i];
		}
	}
}

int simpleRemesh::findOppositeEdgeIdx( SurfaceObj* obj, int triIdx, int pIdx )
{
	TopologyContainer* container = obj->container();
	std::vector<arrayInt>* edgeOnface = container->edgesInFace();
	arrayVec2i* edges = container->edge();

	arrayInt edgesOn = edgeOnface->at(triIdx);
	for (int i=0; i<3; i++)
	{
		Vec2i curE = edges->at(edgesOn[i]);
		if (curE[0]!=pIdx && curE[1]!=pIdx)
		{
			return edgesOn[i];
		}
	}

	return -1;
}

void simpleRemesh::collapseEdge( SurfaceObj* obj, int edgeIdx, arrayInt& removeIdxs, arrayVec3i& addedFace )
{
	TopologyContainer *container = obj->container();
	std::vector<arrayInt> *triAroundPoint = container->facesAroundPoint();
	arrayVec2i* edges = container->edge();
	arrayVec3i* faces = container->face();

	Vec2i cEdge = edges->at(edgeIdx);
	arrayInt allFaceAround;
	arrayInt aroundPt1 = triAroundPoint->at(cEdge[0]);
	arrayInt aroundPt2 = triAroundPoint->at(cEdge[1]);
	allFaceAround.insert(allFaceAround.end(), aroundPt1.begin(), aroundPt1.end());
	allFaceAround.insert(allFaceAround.end(), aroundPt2.begin(), aroundPt2.end());

	VectorFunc func;
	func.arrangeVector(allFaceAround);

	int kPointIdx = cEdge[0];	// Keep point 0, remove point 1 in edge
								// TODO: We may also relocate the vertex
	for (int i=0; i<allFaceAround.size(); i++)
	{
		Vec3i curF = faces->at(allFaceAround[i]);
		int sameCount = 0;
		int idxOfPt;
		for (int j=0; j<3; j++)
		{
			if (curF[j]==cEdge[0] || curF[j]==cEdge[1])
			{
				sameCount++;
				idxOfPt = j;
			}
		}
		ASSERT(sameCount>0);
		if (sameCount==1)//Modify this triangle
		{
			Vec3i newF = curF;
			newF[idxOfPt] = kPointIdx;
			addedFace.push_back(newF);
		}
		else if (sameCount==2)//remove this triangle, do nothing here
		{
			
		}
	}

	removeIdxs.insert(removeIdxs.end(), allFaceAround.begin(), allFaceAround.end());
	
}
