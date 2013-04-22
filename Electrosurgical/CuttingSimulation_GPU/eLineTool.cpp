#include "StdAfx.h"
#include "eLineTool.h"

#include "Utility.h"

eLineTool::eLineTool(void)
{
	m_curPoints.push_back(Vec3f(350,600,0));
	m_curPoints.push_back(Vec3f(350,200,0));

	m_prePoints = m_curPoints;
	updateFaceInfo();
}

eLineTool::~eLineTool(void)
{
}

void eLineTool::draw( int mode )
{
	glColor3f(0.3, 0.3 ,0.5);

	if (mode==0)
		Utility::drawFace(&m_allPoints, &m_face);
	if (mode ==1)
	{
		glColor3f(0, 1 ,0);
		Utility::drawFace(s_points, s_faces, &m_collidedTriIdx);

		glColor3f(1, 1 ,0);
		Utility::drawFace(s_points, s_faces, &m_newFIdxs);
	}
}

void eLineTool::move( Vec3f m )
{
	for (int i=0; i<m_curPoints.size(); i++)
	{
		m_curPoints[i] += m;
	}
	updateFaceInfo();
}

void eLineTool::cut8( SurfaceObj* obj )
{
	// Get share data
	s_surfObj = obj;
	s_points = obj->point();
	s_faces = obj->face();
	AABBTree* BVH = obj->getBVH();

	// Intersection
	VectorFunc vecFunc;
	m_collidedTriIdx.clear();
	CollisionManager colMng;
	arrayInt toolTriIdx;
	colMng.collisionBtwTrisAndTrisWithBVH(&m_allPoints, &m_face, s_points, s_faces, BVH, toolTriIdx, m_collidedTriIdx);
	vecFunc.arrangeVector(m_collidedTriIdx);

}


void eLineTool::stepDebug8()
{
	// Bounding loop
	std::vector<arrayInt> boundLoops;
	findBoundaryLoop(s_surfObj->container(), m_collidedTriIdx, boundLoops);
	ASSERT(boundLoops.size()==1);
	if (boundLoops.size()==0)
		return;
 
	arrayInt bound = boundLoops[0];

	int preNbPoint = s_points->size();
	arrayVec3f addedPoints;
	arrayVec3i addedFaces;

	// Smooth boundary
	addObtuseTriangle(bound, m_collidedTriIdx);
	VectorFunc func;
	func.arrangeVector(m_collidedTriIdx);
	fillConcaveTri(bound, addedFaces);

	// generate new points
	arrayVec3f newPs = generateNewPoints8();

	// We separate the problem
	arrayVec3f cPoints;
	int nbBoundaryPoints;
	nbBoundaryPoints = bound.size();
	for (int i=0; i<nbBoundaryPoints; i++)
		cPoints.push_back(s_points->at(bound[i]));

	for (int i=0; i<newPs.size(); i++)
		cPoints.push_back(newPs[i]);

	arrayVec3i convexFace = findConvexHull(cPoints, nbBoundaryPoints);

	// Convert back to Problem

	for (int i=0; i<convexFace.size(); i++)
	{
		Vec3i orgF = convexFace[i];

		Vec3i newF;
		for (int j=0; j<3; j++)
		{
			int idx = orgF[j];
			if (idx<nbBoundaryPoints) // It is surface point. Corresponding to boundary loop
			{
				newF[j] = bound[idx];
			}
			else //new point
			{
				Vec3f newP = cPoints[idx];
				// Check if it belong to addedPoints
				int indexInNewAdded=-1;
				for (int k=0; k<addedPoints.size(); k++)
				{
					if ((addedPoints[k]-newP).norm() < EPS)//same point
					{	
						indexInNewAdded = preNbPoint + k;
						break;
					}
				}
				if (indexInNewAdded==-1)//not found
				{
					addedPoints.push_back(newP);
					indexInNewAdded = preNbPoint + addedPoints.size()-1;
				}

				newF[j] = indexInNewAdded;
			}
		}

		addedFaces.push_back(newF);
	}

	arrayInt generatedPointIdx = m_collidedTriIdx;
	objTopoModifer(s_surfObj, generatedPointIdx, m_collidedTriIdx, addedFaces, &addedPoints);

	m_collidedTriIdx = generatedPointIdx;

	// Reset tool
	m_prePoints = m_curPoints;
	updateFaceInfo();
}


void eLineTool::updateFaceInfo()
{
	m_allPoints.clear();
	m_allPoints.push_back(m_prePoints[0]);
	m_allPoints.push_back(m_prePoints[1]);
	m_allPoints.push_back(m_curPoints[0]);
	m_allPoints.push_back(m_curPoints[1]);

	m_face.clear();
	m_face.push_back(Vec3i(0,1,2));
	m_face.push_back(Vec3i(1,2,3));
}

arrayVec3f eLineTool::generateNewPoints8()
{
	arrayVec3f newPs;

	CollisionManager colMng;
	int idxInside, idxOutSide;
	if (colMng.isPointInSurfObj(s_surfObj, m_curPoints[0]))
		idxInside = 0;
	else
		idxInside = 1;

	idxOutSide = idxInside==0? 1:0;

	// Determine direction to make a new point
	Vec3f newPDirect;
	if((m_curPoints[idxInside]-m_prePoints[idxInside]).norm() < MIN_MOVE_DISTANCE)
	{
		newPDirect = m_curPoints[idxInside] - m_curPoints[idxOutSide]; 
		newPDirect.normalize();
	}
	else
	{
		Vec3f dr1 = m_curPoints[idxInside] - m_curPoints[idxOutSide]; 
		Vec3f dr2 = m_curPoints[idxInside] - m_prePoints[idxInside];

		dr1.normalize(); dr2.normalize();
		newPDirect = dr1+dr2;
		newPDirect.normalize();
	}

	Vec3f newP = m_curPoints[idxInside] + newPDirect*NEW_POINT_GAP;
	newPs.push_back(newP);
	return newPs;
}

void eLineTool::objTopoModifer(SurfaceObj* surfObj, arrayInt &generatedIdx, arrayInt _removedTriIdx, arrayVec3i _addFace, arrayVec3f* _addPoint /*= NULL*/ )
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

		// Update global list tri
		{
			if (vecFunc.isElementInVector(m_newFIdxs, rmIdx))
			{
				vecFunc.removeElementValue(m_newFIdxs, rmIdx);
			}
			int idx = vecFunc.indexOfElement(m_newFIdxs, changeIdx);
			if (idx>=0)
			{
				m_newFIdxs[idx] = rmIdx;
			}
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

		// Global cut face
		m_newFIdxs.push_back(objTris->size()-1);
	}

 	surfObj->updateNormal();
 	surfObj->updateBVH(); // This can be optimized
}

bool eLineTool::findBoundaryLoopOptimize( TopologyContainer* surfTopo, arrayInt idxOfRemoveTri, std::vector<arrayInt>& boundLoops, arrayInt *edgeToRemove)
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

	std::sort(allEdge.begin(), allEdge.end());

	for (int i=0; i<allEdge.size(); i++)
	{
		if (i==allEdge.size()-1)
		{
			edgeOnBuondary.push_back(allEdge[i]);
			continue;
		}
		if (allEdge[i]==allEdge[i+1])
		{
			edgeRemove.push_back(allEdge[i]);
			i++;
		}
		else
			edgeOnBuondary.push_back(allEdge[i]);
	}


	// 	while(allEdge.size()>0)
	// 	{
	// 		int curE = allEdge[0];
	// 		allEdge.erase(allEdge.begin());
	// 		bool found = false;
	// 		for (int i=0; i<allEdge.size(); i++)
	// 		{
	// 			if (curE == allEdge[i])
	// 			{
	// 				found = true;
	// 				edgeRemove.push_back(curE);
	// 				allEdge.erase(allEdge.begin()+i);
	// 				break;
	// 			}
	// 		}
	// 		if (!found)
	// 		{
	// 			edgeOnBuondary.push_back(curE);
	// 		}
	// 	}

	if (edgeToRemove)
		edgeToRemove->insert(edgeToRemove->end(), edgeRemove.begin(), edgeRemove.end());

	if (edgeOnBuondary.size()==0)
	{
		return true;//??
		return false;
	}



	// 2. Point mesh
	// 	arrayInt pointsIdx;
	// 	for (int i=0; i<edgeOnBuondary.size();i++)
	// 	{
	// 		pointsIdx.push_back((*edges)[edgeOnBuondary[i]][0]);
	// 		pointsIdx.push_back((*edges)[edgeOnBuondary[i]][1]);
	// 	}
	// 	func.arrangeVector(pointsIdx);

	// 3. Find boundary loop

	std::sort(edgeOnBuondary.begin(), edgeOnBuondary.end());
	Vec2i firstE = edges->at(edgeOnBuondary[0]);
	arrayInt boundLoop;
	boundLoop.push_back(firstE[0]);
	boundLoop.push_back(firstE[1]);
	edgeOnBuondary.erase(edgeOnBuondary.begin());
	int curPIdx = firstE[1];

	while(1)
	{
		if (edgeOnBuondary.size() == 1)
		{
			break;
		}

		// Find next 
		bool found =false;
		for (int i=0; i<edgeOnBuondary.size(); i++)
		{
			Vec2i curE = edges->at(edgeOnBuondary[i]);
			if (curPIdx==curE[0])
			{
				boundLoop.push_back(curE[1]);
				curPIdx = curE[1];
				edgeOnBuondary.erase(edgeOnBuondary.begin()+i);
				found = true;
				break;
			}
			else if (curPIdx == curE[1])
			{
				boundLoop.push_back(curE[0]);
				curPIdx = curE[0];
				edgeOnBuondary.erase(edgeOnBuondary.begin()+i);
				found=true;
				break;
			}
		}

		ASSERT(found);//??

	}


	conterClockWiseLoop(surfTopo, idxOfRemoveTri, boundLoop);
	boundLoops.push_back(boundLoop);

	return true;
}
bool eLineTool::findBoundaryLoop( TopologyContainer* surfTopo, arrayInt idxOfRemoveTri, std::vector<arrayInt>& boundLoops, arrayInt *edgeToRemove)
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

	std::sort(allEdge.begin(), allEdge.end());

	for (int i=0; i<allEdge.size(); i++)
	{
		if (i==allEdge.size()-1)
		{
			edgeOnBuondary.push_back(allEdge[i]);
			continue;
		}
		if (allEdge[i]==allEdge[i+1])
		{
			edgeRemove.push_back(allEdge[i]);
			i++;
		}
		else
			edgeOnBuondary.push_back(allEdge[i]);
	}


// 	while(allEdge.size()>0)
// 	{
// 		int curE = allEdge[0];
// 		allEdge.erase(allEdge.begin());
// 		bool found = false;
// 		for (int i=0; i<allEdge.size(); i++)
// 		{
// 			if (curE == allEdge[i])
// 			{
// 				found = true;
// 				edgeRemove.push_back(curE);
// 				allEdge.erase(allEdge.begin()+i);
// 				break;
// 			}
// 		}
// 		if (!found)
// 		{
// 			edgeOnBuondary.push_back(curE);
// 		}
// 	}

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

void eLineTool::conterClockWiseLoop( TopologyContainer* surfTopo, arrayInt areaTriIdx, arrayInt& boundPointLoop )
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

arrayVec3i eLineTool::findConvexHull( arrayVec3f cPoints, int nbBoundaryPoints )
{
	ASSERT(cPoints.size()>nbBoundaryPoints);
	arrayVec3i cFace;
	// init
	for (int i=0; i<nbBoundaryPoints; i++)
	{
		cFace.push_back(Vec3i(nbBoundaryPoints, i, (i+1)%nbBoundaryPoints));
	}

	if (cPoints.size()>nbBoundaryPoints+1)
	{
		for (int i=nbBoundaryPoints+1; i<cPoints.size(); i++)
		{
			addPointToConvex(cPoints, i, cFace);
		}
	}

	return cFace;
}

void eLineTool::addPointToConvex( arrayVec3f _cPoints, int pIdx, arrayVec3i& _cFace )
{
	VectorFunc vecFunc;
	GeometricFunc func;
	arrayInt facetIdx;
	Vec3f pt = _cPoints[pIdx];
	// Faces that opposite to the point
	for (int i=0; i<_cFace.size(); i++)
	{
		if (isDirectFace(pt, i, _cPoints, _cFace))
		{
			facetIdx.push_back(i);
		}
	}

	// Find boundary edge
	arrayVec2i boundEdge;
	for (int i=0; i<facetIdx.size(); i++)
	{
		Vec3i curF = _cFace[facetIdx[i]];
		for (int j=0; j<3; j++)
		{
			Vec2i curE(curF[j], curF[(j+1)%3]);
			bool same = false;
			for (int k=0; k<boundEdge.size(); k++)
			{
				if (func.isEdgeSame(curE, boundEdge[k]))
				{
					boundEdge.erase(boundEdge.begin()+k);
					same = true;
					break;
				}
			}
			if (!same)
			{
				boundEdge.push_back(curE);
			}
		}
	}

	vecFunc.arrangeVector(facetIdx);
	for (int i=facetIdx.size()-1; i>=0; i--)
	{
		_cFace.erase(_cFace.begin() + facetIdx[i]);
	}

	// Add new triangles
	for (int i=0; i<boundEdge.size(); i++)
	{
		_cFace.push_back(Vec3i(pIdx, boundEdge[i][0], boundEdge[i][1]));
	}
}

bool eLineTool::isDirectFace( Vec3f pt, int triIdx, arrayVec3f& cPoints, arrayVec3i& cFace )
{
	// Need optimization later

	// First, it must be on normal direction
	GeometricFunc func;
	Vec3f faceNorm = func.computeNormal(cPoints, cFace[triIdx]);
	if (faceNorm*(pt-cPoints[cFace[triIdx][0]]) > 0) // >0 because we compute on opposite side
	{
		return false;
	}

	// We check by testing intersect of 3 segments, that connect pt and 3 corner points, and surface
	for (int i=0; i<3; i++)
	{
		Vec3f cornerP = cPoints[cFace[triIdx][i]];
		// Adjust
		cornerP = cornerP - (cornerP-pt)*0.05;
		for (int j=0; j<cFace.size(); j++)
		{
			Vec3f _tri[3];
			Vec3f intersect;
			_tri[0] = cPoints[cFace[j][0]];
			_tri[1] = cPoints[cFace[j][1]];
			_tri[2] = cPoints[cFace[j][2]];

			if(func.collisionBtwLinesegAndTri(pt, cornerP, _tri, intersect))
			{
				return false;
			}
		}
	}

	return true;
}

void eLineTool::resetPath()
{
	m_prePoints = m_curPoints;
	updateFaceInfo();
}

void eLineTool::cut9( SurfaceObj* obj )
{
	// Get share data
	s_surfObj = obj;
	s_points = obj->point();
	s_faces = obj->face();
	AABBTree* BVH = obj->getBVH();

	// Intersection
	VectorFunc func;
	m_collidedTriIdx.clear();
	CollisionManager colMng;
	arrayInt toolTriIdx;


	colMng.collisionBtwTrisAndTrisWithBVH(&m_allPoints, &m_face, s_points, s_faces, BVH, toolTriIdx, m_collidedTriIdx);
	func.arrangeVector(m_collidedTriIdx);
}


void eLineTool::stepDebug9()
{
	CTimeTick time;
	time.SetStart();

	// Bounding loop
	std::vector<arrayInt> boundLoops;
	findBoundaryLoopOptimize(s_surfObj->container(), m_collidedTriIdx, boundLoops);
	ASSERT(boundLoops.size()==1);
	m_bound = boundLoops[0];
	
	// triangulate
	m_addedFaces.clear();
	m_addedPoints.clear();

	time.SetEnd();
	Utility::urgentLog("findBoundaryLoop: %lf", time.GetTick());
	fillConcaveTri(m_bound, m_addedFaces);

	time.SetStart();

	if (m_bound.size()>=3)
		triangulate(m_bound);
	time.SetEnd();
	Utility::urgentLog("Meshing: %lf", time.GetTick());

	arrayInt generated = m_collidedTriIdx;
	objTopoModifer(s_surfObj, generated, m_collidedTriIdx, m_addedFaces);
	m_collidedTriIdx = generated;

	// remesh the area
	step2Debug9();
}


void eLineTool::triangulate( arrayInt &boundLoop )
{
	// bound loop is a triangle
	if (boundLoop.size()==3)
	{
		m_addedFaces.push_back(Vec3i(boundLoop[0], boundLoop[1], boundLoop[2]));
		return;
	}

	// Find best edge to separate boundary
	Vec3f midPoint;
	for (int i=0; i<boundLoop.size(); i++)
	{
		midPoint += s_points->at(boundLoop[i]);
	}
	midPoint/=boundLoop.size();

	GeometricFunc func;
	float smallestDis = 9999;
	float largestDistance = 0;
	arrayInt boundLoop1, boundLoop2;
	int idx1, idx2;
	bool shouldOut = false;
	for (int i=0; i<boundLoop.size(); i++)
	{
		if (shouldOut)
		{
			break;
		}
		for (int j=0; j<boundLoop.size(); j++)
		{
			if (i==j || abs(i-j)==1 || abs(i-j)==boundLoop.size()-1)
			{
				continue;
			}

			// Check if new tri intersect with tool
			if (isIntersectWithEdge(s_points->at(boundLoop[i]), s_points->at(boundLoop[j])))
			{
				continue;
			}

			//Check legal area also
			arrayInt bound1, bound2;
			int curIdx = j;
			while(1)
			{
				bound1.push_back(boundLoop[curIdx]);
				if (curIdx==i)
					break;
				curIdx++;
				if (curIdx==boundLoop.size())
					curIdx=0;
			}

			curIdx = i;
			while(1)
			{
				bound2.push_back(boundLoop[curIdx]);
				if (curIdx==j)
					break;
				curIdx++;
				if (curIdx==boundLoop.size())
					curIdx=0;
			}
// 			if (!(isLegalBound(bound1) && isLegalBound(bound2)))
// 			{
// 				continue;
// 			}
	
			// Edge near mid point the most
// 			Vec3f curMidP = (s_points->at(boundLoop[i]) + s_points->at(boundLoop[j]))/2;
// 			float disToMidPoint = (curMidP-midPoint).norm();
// 			if (smallestDis > disToMidPoint)
// 			{
// 				smallestDis = disToMidPoint;
// 				boundLoop1=bound1;
// 				boundLoop2=bound2;
// 				idx1 = boundLoop[j];
// 				idx2 = boundLoop[i];
// 			}

			// Edge far cuttool the most
// 			float dis = distanceToLineSeg(s_points->at(boundLoop[i]), s_points->at(boundLoop[j]));
// 			if (largestDistance < dis)
			{
	//			largestDistance = dis;
				boundLoop1=bound1;
				boundLoop2=bound2;
				idx1 = boundLoop[j];
				idx2 = boundLoop[i];
			}

			shouldOut = true;
			break;

			// Good angle
// 			float var1 = varianceAngle(s_points, bound1);
// 			float var2 = varianceAngle(s_points, bound2);
// 			if (smallestDis > var1*var1+var2*var2)
// 			{
// 				smallestDis = var1*var1+var2*var2;
// 				boundLoop1=bound1;
// 				boundLoop2=bound2;
// 				idx1 = boundLoop[j];
// 				idx2 = boundLoop[i];
// 			}
		}	
	}

	ASSERT(boundLoop1.size()>0);

	// Add points to new edge if need
// 	Vec3f pt1 = s_points->at(idx1);
// 	Vec3f pt2 = s_points->at(idx2);
// 	if ((pt1-pt2).norm() > 1.5*MAX_EDGE_LENGTH)
// 	{
// 		int preNbPoints = s_points->size();
// 		int nbSegment = ceil((pt1-pt2).norm()/MAX_EDGE_LENGTH);
// 		float segLength = (pt1-pt2).norm()/nbSegment;
// 		
// 		arrayVec3f newPs;
// 		Vec3f direct = pt2-pt1;direct.normalize();
// 		for (int i=0; i<nbSegment-1; i++)
// 		{
// 			newPs.push_back(pt1+direct*(i+1)*segLength);
// 		}
// 		TopologyModifier modifier;
// 		modifier.addPoints(s_surfObj->container(), newPs);
// 
// 		for (int i=0; i<nbSegment-1; i++)
// 		{
// 			boundLoop1.push_back(preNbPoints+(nbSegment-i-1) -1);
// 			boundLoop2.push_back(i+preNbPoints);
// 		}
// 	}

	triangulate(boundLoop1);
	triangulate(boundLoop2);
}

bool eLineTool::isIntersectWithEdge( Vec3f pt1, Vec3f pt2 )
{
	GeometricFunc func;
	for (int i=0; i<m_face.size(); i++)
	{
		Vec3f _tri[3];
		_tri[0]=m_allPoints[m_face[i][0]];
		_tri[1]=m_allPoints[m_face[i][1]];
		_tri[2]=m_allPoints[m_face[i][2]];

		Vec3f intersect;
		if(func.collisionBtwLinesegAndTri(pt1, pt2, _tri, intersect))
			return true;
	}

	return false;
}

bool eLineTool::isLegalBound( arrayInt loop )
{
	GeometricFunc func;
	int nbPoint = loop.size();

	//Check all point not lie in a line
	Vec3f pt1 = s_points->at(loop[0]);
	Vec3f pt2 = s_points->at(loop[1]);
	bool allSame = true;
	for (int i=2; i<nbPoint; i++)
	{
		Vec3f pt = s_points->at(loop[i]);
		float test = ((pt-pt1).cross(pt2-pt1)).norm()/(pt-pt1).norm()/(pt2-pt1).norm();
		if (test > EPS)
		{
			allSame = false;
			break;
		}
	}
	if (allSame)
	{
		return false;
	}

	//Check area does not cover the tool line
	float angle =0;
	for (int i=0; i<nbPoint; i++)
	{
		Vec3f toolP = m_curPoints[0];
		Vec3f toolDirect = m_curPoints[1]-m_curPoints[0];

		Vec3f pt1 = s_points->at(loop[i]);
		Vec3f pt2 = s_points->at(loop[(i+1)%nbPoint]);
		float s1 = ((pt1-toolP)*toolDirect)/(toolDirect*toolDirect);
		float s2 = ((pt2-toolP)*toolDirect)/(toolDirect*toolDirect);

		Vec3f ltp1 = pt1-(toolP + toolDirect*s1); // Project to the orthogonal plane
		Vec3f ltp2 = pt2-(toolP + toolDirect*s2);

		ltp1.normalize();ltp2.normalize();toolDirect.normalize();
		float curA = atan2((ltp1.cross(ltp2))*toolDirect, ltp1*ltp2);
		angle += atan2((ltp1.cross(ltp2))*toolDirect, ltp1*ltp2);
	}
	return abs(angle) < 0.001;
}

float eLineTool::distanceToLineSeg( Vec3f pt1, Vec3f pt2 )
{
	GeometricFunc func;
	float distace = 9999;
	for(int i=0; i<m_face.size(); i++)
	{
		Vec3f cp1, cp2;
		func.measureMinimumDistanceBetLineandTriangle(pt1, pt2, 
					m_allPoints[m_face[i][0]], m_allPoints[m_face[i][1]], m_allPoints[m_face[i][2]], cp1, cp2);
		float dis = (cp1-cp2).norm();
		if (distace > dis)
		{
			distace = dis;
		}
	}

	return distace;
}

float eLineTool::varianceAngle( arrayVec3f* points, arrayInt bound )
{
	int nbbound = bound.size();
	float var = 0;
	for (int i=0; i<nbbound; i++)
	{
		Vec3f  v1 = points->at(bound[(i-1+nbbound)%nbbound]) - points->at(bound[i]);
		Vec3f v2 = points->at(bound[(i+1)%nbbound]) - points->at(bound[i]);

		float angle = acos((v1*v2)/(v1.norm()*v2.norm()));
		var += abs(angle-PI/3);
	}
	return var/nbbound;
}

float eLineTool::varianceAngle( arrayVec3f* points, int idx1, int idx2, int idx3 )
{
	arrayInt idxs;
	idxs.push_back(idx1);
	idxs.push_back(idx2);
	idxs.push_back(idx3);
	return varianceAngle(points, idxs);
}

void eLineTool::remeshByFlip(arrayInt areaIdx )
{
	GeometricFunc func;
	arrayInt edgeInside;
	std::vector<arrayInt> boundLoops;
	arrayVec2i* edges = s_surfObj->edge();
	findBoundaryLoop(s_surfObj->container(), areaIdx, boundLoops, &edgeInside);

	float minVar = 9999;
	float maxDis = 0;
	int idxInArray = -1;
	for (int i=0; i<edgeInside.size(); i++)
	{
		// Process point idx
		arrayInt faceAround = s_surfObj->container()->facesAroundEdge(edgeInside[i]);
		int peIdx1, peIdx2, poIdx1, poIdx2;
		peIdx1 = edges->at(edgeInside[i])[0];
		peIdx2 = edges->at(edgeInside[i])[1];
		poIdx1 = -1; poIdx2 = -1;
		for (int j=0; j<faceAround.size(); j++)
		{
			Vec3i tri = s_faces->at(faceAround[j]);
			for (int k=0; k<3; k++)
			{
				if ( !(tri[k]==peIdx1 || tri[k]==peIdx2))
				{
					if (poIdx1==-1)
						poIdx1 = tri[k];
					else if(poIdx2 == -1)
						poIdx2 = tri[k];
				}
			}
		}

		// Check if this edge can flip
		// New edge not intersect tool
		if (isIntersectWithEdge(s_points->at(poIdx1), s_points->at(poIdx2)))
		{
			continue;
		}
		// Angle should flip
		float varO1 = varianceAngle(s_points, peIdx1, poIdx1, poIdx2);
		float varO2 = varianceAngle(s_points, peIdx2, poIdx1, poIdx2);
		float varE1 = varianceAngle(s_points, poIdx1, peIdx1, peIdx2);
		float varE2 = varianceAngle(s_points, poIdx2, peIdx1, peIdx2);
		
		if ((varE1+varE2)>(varO1+varO2)*1.1)
		{
			if (maxDis < varE1+varE2-varO1-varO2)
			{
				maxDis = varE1+varE2-varO1-varO2;
				idxInArray = i;
			}
		}
	}

	if (idxInArray != -1)
	{
		// Process point idx
		arrayInt faceAround = s_surfObj->container()->facesAroundEdge(edgeInside[idxInArray]);
		int peIdx1, peIdx2, poIdx1, poIdx2;
		peIdx1 = edges->at(edgeInside[idxInArray])[0];
		peIdx2 = edges->at(edgeInside[idxInArray])[1];
		poIdx1 = -1; poIdx2 = -1;
		for (int j=0; j<faceAround.size(); j++)
		{
			Vec3i tri = s_faces->at(faceAround[j]);
			for (int k=0; k<3; k++)
			{
				if ( !(tri[k]==peIdx1 || tri[k]==peIdx2))
				{
					if (poIdx1==-1)
						poIdx1 = tri[k];
					else if(poIdx2 == -1)
						poIdx2 = tri[k];
				}
			}
		}

		arrayVec3i newF;

		Vec2i shareE(peIdx1, peIdx2);
		Vec3i tri = s_faces->at(faceAround[0]);

		for (int i=0; i<3; i++)
		{
			Vec2i curE(tri[i], tri[(i+1)%3]);
			if(!func.isEdgeSame(curE, shareE))
			{
				newF.push_back(Vec3i(curE[0], curE[1], poIdx2));
			}
		}

		arrayInt gg;
		objTopoModifer(s_surfObj, gg, faceAround, newF);

		VectorFunc vecFunc;
		vecFunc.removeElementValue(areaIdx, faceAround[0]);
		vecFunc.removeElementValue(areaIdx, faceAround[1]);
		areaIdx.push_back(s_faces->size()-1);
		areaIdx.push_back(s_faces->size()-2);
	}
}

void eLineTool::step2Debug9()
{
	remeshByFlip(m_collidedTriIdx);
}

void eLineTool::fillConcaveTri( arrayInt& trimmingLoop, arrayVec3i& addedFaces )
{
	while(1)
	{
		int concaveIdx = findConcaveIndex(trimmingLoop);
		if (concaveIdx==-1)
		{
			break;
		}
		else
		{
			//fill
			Vec3i newTri;
			newTri[0]= trimmingLoop[(concaveIdx-1 + trimmingLoop.size())%trimmingLoop.size()];
			newTri[1]= trimmingLoop[concaveIdx];
			newTri[2]= trimmingLoop[(concaveIdx+1)%trimmingLoop.size()];
			addedFaces.push_back(newTri);

			if (trimmingLoop.size() ==3)
			{
				trimmingLoop.clear();
				break;
			}
			trimmingLoop.erase(trimmingLoop.begin()+concaveIdx);
		}
	}
}

int eLineTool::findConcaveIndex( arrayInt trimmingLoop )
{
	arrayVec3f* pointNorm = s_surfObj->pointNormal();


	GeometricFunc func;
	int nbNode = trimmingLoop.size();
	int sIdx;
	float smallest = 9999;
	for (unsigned i=0; i<nbNode; i++)
	{
		Vec3i ptIdx;
		Vec3f pt[3];
		ptIdx[0]=trimmingLoop[(i-1+nbNode)%nbNode];
		ptIdx[1]=trimmingLoop[i];
		ptIdx[2]=trimmingLoop[(i+1)%nbNode];

		pt[0] = s_points->at(ptIdx[0]);
		pt[1] = s_points->at(ptIdx[1]);
		pt[2] = s_points->at(ptIdx[2]);

		if (isIntersectWithTri(pt))
		{
			continue;
		}

		//Check counter clockwise also
		Vec3f norm;
		for (int j=0; j<3; j++)
		{
			norm = pointNorm->at(ptIdx[j]);
		}
		Vec3f curNorm = GeometricFunc::computeNormal(s_points, ptIdx);
		if (norm*curNorm < 0)
		{
			continue;
		}

		Vec3f ptV1 = pt[0]-pt[1];
		Vec3f ptV2 = pt[2]-pt[1];

		float angle = acos(ptV1*ptV2/(ptV1.norm()*ptV2.norm()));

		if (angle < smallest)
		{
			smallest = angle;
			sIdx = i;
		}
	}

	if (smallest<ANGLE_THRES_HOLD)
	{
		return sIdx;
	}
	return -1;
}

bool eLineTool::isIntersectWithTri( Vec3f tri[]  )
{
	GeometricFunc func;
	for (int i=0; i<m_face.size(); i++)
	{
		Vec3f curTri[3];
		curTri[0]=m_allPoints[m_face[i][0]];
		curTri[1]=m_allPoints[m_face[i][1]];
		curTri[2]=m_allPoints[m_face[i][2]];

		if (func.collisionBtwTri(curTri, tri))
			return true;
	}

	return false;
}

void eLineTool::addObtuseTriangle( arrayInt& bound, arrayInt &triArea  )
{
	std::vector<arrayInt>* faceAroundP = s_surfObj->container()->facesAroundPoint();

	while(1)
	{
		int idx = findObtuseIndex(bound, triArea);
		if (idx != -1)
		{
			Vec3i ptIdx;
			int nbNode = bound.size();
			ptIdx[0]=bound[(idx-1+nbNode)%nbNode];
			ptIdx[1]=bound[idx];
			ptIdx[2]=bound[(idx+1)%nbNode];

			// tris to remove
			VectorFunc vecFunc;
			arrayInt faceAround = faceAroundP->at(bound[idx]);
			arrayInt removeIdxs;
			for (int i=0; i<faceAround.size(); i++)
			{
				if (!vecFunc.isElementInVector(triArea, faceAround[i]))
				{
					removeIdxs.push_back(faceAround[i]);
				}
			}
			ASSERT(removeIdxs.size()==2);

			// find the other points
			int idx4;
			Vec3i curTri = s_faces->at(removeIdxs[0]);
			for (int i=0; i<3; i++)
			{
				if (curTri[i]!=ptIdx[0] && curTri[i]!=ptIdx[1] && curTri[i]!=ptIdx[2])
				{
					idx4 = curTri[i];
				}
			}

			// process change
			arrayVec3i addIdxs;
			addIdxs.push_back(Vec3i(ptIdx[1], ptIdx[0], ptIdx[2]));
			addIdxs.push_back(Vec3i(ptIdx[2], ptIdx[0], idx4));

			// update topo
			arrayInt gege = triArea;
			objTopoModifer(s_surfObj, gege, removeIdxs, addIdxs);

			triArea = gege;
			vecFunc.removeElementValue(triArea, s_faces->size()-1);

			bound.erase(bound.begin()+idx);

		}
		else
			break;
	}

}

int eLineTool::findObtuseIndex( arrayInt trimmingLoop, arrayInt triArea )
{
	arrayVec3f* pointNorm = s_surfObj->pointNormal();
	std::vector<arrayInt>* faceAroundP = s_surfObj->container()->facesAroundPoint();

	GeometricFunc func;
	int nbNode = trimmingLoop.size();
	int sIdx;
	float smallest = 9999;
	for (unsigned i=0; i<nbNode; i++)
	{
		Vec3i ptIdx;
		Vec3f pt[3];
		ptIdx[0]=trimmingLoop[(i-1+nbNode)%nbNode];
		ptIdx[1]=trimmingLoop[i];
		ptIdx[2]=trimmingLoop[(i+1)%nbNode];

		pt[0] = s_points->at(ptIdx[0]);
		pt[1] = s_points->at(ptIdx[1]);
		pt[2] = s_points->at(ptIdx[2]);

		if (isIntersectWithTri(pt))
		{
			continue;
		}

		//Check counter clockwise also
		Vec3f norm;
		for (int j=0; j<3; j++)
		{
			norm = pointNorm->at(ptIdx[j]);
		}
		Vec3f curNorm = GeometricFunc::computeNormal(s_points, ptIdx);
		if (norm*curNorm > 0)
		{
			continue;
		}

		// Currently, we only fill 2 triangles
		VectorFunc vecFunc;
		arrayInt faceAround = faceAroundP->at(trimmingLoop[i]);
		arrayInt faceOutIdx;
		for (int j=0; j<faceAround.size(); j++)
		{
			if (!vecFunc.isElementInVector(triArea, faceAround[j]))
			{
				faceOutIdx.push_back(faceAround[j]);
			}
		}
		if (faceOutIdx.size() != 2) // We may need more conditions
		{
			continue;
		}


		Vec3f ptV1 = pt[0]-pt[1];
		Vec3f ptV2 = pt[2]-pt[1];

		float angle = acos(ptV1*ptV2/(ptV1.norm()*ptV2.norm()));

		if (angle < smallest)
		{
			smallest = angle;
			sIdx = i;
		}
	}

	if (smallest<ANGLE_THRES_HOLD)
	{
		return sIdx;
	}
	return -1;
}

void eLineTool::smoothBoundary()
{
	arrayVec3f* points = s_points;
	arrayVec3f* point0 = s_surfObj->point0();
	arrayVec3i* face = s_faces;

	VectorFunc func;
	std::vector<arrayInt> allLoops;
	findBoundaryLoop(s_surfObj->container(), m_newFIdxs, allLoops);
	arrayInt loops = allLoops[0];

	int nbPoint = loops.size();
	arrayVec3f newPs;
	arrayVec3f newPs0;
	for (int i=0; i<loops.size(); i++)
	{
		Vec3f pt = (points->at(loops[(i-1+nbPoint)%nbPoint])+points->at(loops[i])+
			points->at(loops[(i+1)%nbPoint]))/3;
		Vec3f pt0 = (point0->at(loops[(i-1+nbPoint)%nbPoint])+point0->at(loops[i])+
			point0->at(loops[(i+1)%nbPoint]))/3;
		newPs.push_back(pt);
		newPs0.push_back(pt0);
	}
	for (int i=0; i<loops.size(); i++)
	{
		points->at(loops[i])=newPs[i];
		point0->at(loops[i])=newPs0[i];
	}
}

// Find material coordinate of a new point
// Base on shape function relative to efg nodes
// Solution: Newton-Raphson method
Vec3f eLineTool::invertMappingFunction( Meshfree_GPU *obj, Vec3f curP )
{
	EFG_CUDA_RUNTIME *efgObj = obj->efgObj();
	arrayVec3f* efgNodes = efgObj->nodePosVec();
	arrayVec3f* efgNode0 = efgObj->nodePos0Vec();
	float* displace = efgObj->nodeDis();
	float SupportRadius = efgObj->supportRadius();
	// Optimization later 
	// Determine neighbor base on current position

	arrayInt neighborIdx;
	for (int i=0; i<efgNodes->size(); i++)
	{
		if(((*efgNodes)[i] - curP).norm() <SupportRadius)
		{	
			neighborIdx.push_back(i);
		}	
	}

	// Compute initial solution
	Vec3f potentialPoint=Vec3f(0,0,0);
	for (int i=0; i<neighborIdx.size(); i++)
	{
		potentialPoint += efgNode0->at(neighborIdx[i]);
	}
	potentialPoint = potentialPoint/neighborIdx.size();

	// Loop to find approximate solution
	// Input is potential Points
	while(1)
	{
		std::vector<float> shapeFuncValue;
		shapeFuncValue.resize(neighborIdx.size());

		std::vector<float> shapeFuncDrvAtX, shapeFuncDrvAtY, shapeFuncDrvAtZ;
		shapeFuncDrvAtX.resize(neighborIdx.size());
		shapeFuncDrvAtY.resize(neighborIdx.size());
		shapeFuncDrvAtZ.resize(neighborIdx.size());

		// Find shape function and its derivative
		MLSShapeFunc shapeFunc;
		shapeFunc.init(efgNode0, SupportRadius, WEIGHT_FUNC_TYPE);

		// Moment matrix - Compute shape function
		Mat4x4f momentMatrixAtNode, invertMomentMatrix;
		shapeFunc.computeMomentMatrix(momentMatrixAtNode, neighborIdx, potentialPoint);
		invertMatrix(invertMomentMatrix, momentMatrixAtNode);

		for (int i =0; i<neighborIdx.size(); i++)
		{
			shapeFuncValue[i] = shapeFunc.computeShapeFuncValue(neighborIdx[i], potentialPoint, invertMomentMatrix);
		}

		// Shape function derivative at X
		Mat4x4f momentDrvAtX;
		shapeFunc.computeMomentMatrixDrvX(potentialPoint, momentDrvAtX, neighborIdx);
		for(unsigned int i=0; i<neighborIdx.size(); i++)
		{
			shapeFuncDrvAtX[i] = shapeFunc.computeShapeFuncDrvX(neighborIdx[i], potentialPoint, invertMomentMatrix, momentDrvAtX);
		}

		// Shape function derivative at Y
		Mat4x4f momentDrvAtY;
		shapeFunc.computeMomentMatrixDrvY(potentialPoint, momentDrvAtY, neighborIdx);
		for(unsigned int i=0; i<neighborIdx.size(); i++)
		{
			shapeFuncDrvAtY[i] = shapeFunc.computeShapeFuncDrvX(neighborIdx[i], potentialPoint, invertMomentMatrix, momentDrvAtY);
		}

		// Shape function derivative at Z
		Mat4x4f momentDrvAtZ;
		shapeFunc.computeMomentMatrixDrvY(potentialPoint, momentDrvAtZ, neighborIdx);
		for(unsigned int i=0; i<neighborIdx.size(); i++)
		{
			shapeFuncDrvAtZ[i] = shapeFunc.computeShapeFuncDrvZ(neighborIdx[i], potentialPoint, invertMomentMatrix, momentDrvAtZ);
		}

		// Now compute error
		Vec3f a,b;
		for (int i=0; i<neighborIdx.size(); i++)
		{
			int nodeIdx = neighborIdx[i];
			a[0] += shapeFuncDrvAtX[i]*displace[nodeIdx*DIM];
			a[1] += shapeFuncDrvAtY[i]*displace[nodeIdx*DIM+1];
			a[2] += shapeFuncDrvAtZ[i]*displace[nodeIdx*DIM+2];

			b[0] += shapeFuncValue[i]*displace[nodeIdx*DIM];
			b[1] += shapeFuncValue[i]*displace[nodeIdx*DIM+1];
			b[2] += shapeFuncValue[i]*displace[nodeIdx*DIM+2];
		}
		a[0] += 1; a[1] += 1; a[2] += 1;
		b[0] = curP[0] - (potentialPoint[0] + b[0]);
		b[1] = curP[1] - (potentialPoint[1] + b[1]);
		b[2] = curP[2] - (potentialPoint[2] + b[2]);

		Vec3f dx;
		for (int i=0; i<3; i++)
		{
			ASSERT(abs(a[0])>EPS);
			dx[i] = b[i]/a[i];
		}

		potentialPoint = potentialPoint + dx;

		if (dx.norm() < 0.01)
		{
			break;
		}
	}

	return potentialPoint;
}

Vec3f eLineTool::invertMappingFunction2( Meshfree_GPU *obj, Vec3f curP )
{
// 	// We do this later if there is a performance problem
// 
// 	SurfaceObj* surfObj = obj->surfObj();
// 	arrayVec3f* surPoints = obj->surfObj()->point();
// 	AABBTree* BVH = surfObj.getBVH();
// 	// Take first point
// 	int point1Idx = 0;
// 	Vec3f point1 = surPoints->at(point1Idx);
// 
// 	// 
// 	CollisionManager colMng;
// 	arrayInt triIdxs;
// 	colMng.collisionBtwAABBAndAxisLine(BVH, point1, curP-point1, triIdxs);
	
	return Vec3f();
}
