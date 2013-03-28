#include "StdAfx.h"
#include "cuttingTest.h"

cuttingTest::cuttingTest(void)
{
}

cuttingTest::~cuttingTest(void)
{
}

bool cuttingTest::cutPapilla( Vec3f sPt, Vec3f cPt1, Vec3f cPt2,  Meshfree_GPU* obj )
{
	// Reset temporal variable
	m_collidedTriIdx.clear();
	newPt = cPt2;

	s_surfObj = obj->surfObj();
	s_points = s_surfObj->point();
	Obj_GPU = obj;

	// cut front
	arrayVec3f cutP;
	arrayVec2i cutE;
	arrayVec3i cutTri;

	cutP.push_back(sPt);
	cutP.push_back(cPt1);
	cutP.push_back(cPt2);
	cutE.push_back(Vec2i(0,1));
	cutE.push_back(Vec2i(1,2));
	cutE.push_back(Vec2i(2,0));
	cutTri.push_back(Vec3i(0,1,2));

	bool bIntersected = cutSurface(cutP, cutTri, obj->surfObj());

	if (bIntersected)
	{
		cutSurStep2();
		cutEFGObj(cutP, cutTri);
		updateConnection();
		return true;
	}

	return false; // No intersection
}

void cuttingTest::draw( int mode )
{
	arrayVec3f* surPoint = s_surfObj->point();
	arrayVec3i* surTri = s_surfObj->face();

	glBegin(GL_TRIANGLES);
	for (int i=0; i<m_collidedTriIdx.size(); i++)
	{
		Vec3i curT = surTri->at(m_collidedTriIdx[i]);
		for (int j=0; j<3; j++)
		{
			Vec3f curPt = surPoint->at(curT[j]);
			glVertex3f(curPt[0], curPt[1], curPt[2]);
		}
	}
	glEnd();

}

bool cuttingTest::cutSurface( arrayVec3f cutP, arrayVec3i cutTri, SurfaceObj* surf )
{
	arrayVec3f* surPoint = surf->point();
	arrayVec3i* surTri = surf->face();
	AABBTree* surBVH = surf->getBVH();
	VectorFunc vecF;

	// Detect collision
	arrayInt temp;
	CollisionManager _collision;
	_collision.collisionBtwTrisAndTrisWithBVH( &cutP, &cutTri, surPoint, surTri, surBVH, temp, m_collidedTriIdx);
	vecF.arrangeVector(m_collidedTriIdx);

	if (m_collidedTriIdx.size() == 0) // No intersection
		return false;

	return true;
}


void cuttingTest::cutSurStep2()
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

// 	// Smooth boundary
// 	addObtuseTriangle(bound, m_collidedTriIdx);
// 	VectorFunc func;
// 	func.arrangeVector(m_collidedTriIdx);
// 	fillConcaveTri(bound, addedFaces);

	// generate new points
	arrayVec3f newPs;
	newPs.push_back(newPt);

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
	// We need added points 0 also
	objTopoModifer(s_surfObj, generatedPointIdx, m_collidedTriIdx, addedFaces, &addedPoints);

	m_collidedTriIdx = generatedPointIdx;
}

arrayVec3i cuttingTest::findConvexHull( arrayVec3f cPoints, int nbBoundaryPoints )
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

void cuttingTest::addPointToConvex( arrayVec3f _cPoints, int pIdx, arrayVec3i& _cFace )
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

bool cuttingTest::isDirectFace( Vec3f pt, int triIdx, arrayVec3f& cPoints, arrayVec3i& cFace )
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
void cuttingTest::objTopoModifer(SurfaceObj* surfObj, arrayInt &generatedIdx, arrayInt _removedTriIdx, arrayVec3i _addFace, arrayVec3f* _addPoint /*= NULL*/ )
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
		modifier.addPoints(topo, _addPoint, _addPoint);

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

bool cuttingTest::findBoundaryLoop( TopologyContainer* surfTopo, arrayInt idxOfRemoveTri, std::vector<arrayInt>& boundLoops, arrayInt *edgeToRemove)
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

void cuttingTest::conterClockWiseLoop( TopologyContainer* surfTopo, arrayInt areaTriIdx, arrayInt& boundPointLoop )
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

void cuttingTest::updateConnection()
{
	VectorFunc func;
	GeometricFunc geoFunc;
	CollisionManager collision;

	// Surface object data
	SurfaceObj* surfObj=Obj_GPU->surfObj();
	std::vector<Vec3f>* surfNode=surfObj->point();
	std::vector<Vec3i>* surfFace=surfObj->face();

	// Meshfree object data
	std::vector<std::vector<int>>* neighborNodeIdx=Obj_GPU->neighborNodeOfSurfVertice();
	std::vector<int> updatedPointIdx;
	neighborNodeIdx->resize(surfNode->size());

	updatedPointIdx.push_back(surfNode->size()-1);

	func.arrangeVector(updatedPointIdx);

	// Update neighbor information
	arrayInt pointIdx;
	pointIdx.push_back(surfNode->size()-1);
	float supportRadius=Obj_GPU->supportRadiusSurf();
	std::vector<Vec3f>* efgNode=Obj_GPU->efgObj()->nodePosVec();
	neighborNodeIdx->resize(surfNode->size());

	for (int i=0; i<pointIdx.size(); i++)
	{
		int pIdx=pointIdx[i];

		Vec3f l1 = (*surfNode)[pIdx];

		std::vector<int> colNodeIdx;
		collision.PQBtwBoxAndPoint(Obj_GPU->efgObj()->getBVHPoint()->root(), efgNode, l1, colNodeIdx, supportRadius);
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
			addNeighbor(Obj_GPU, neighbor, colNodeIdx, neighbor[0]);
		}
		else
		{
			throw std::exception("Error when update neighbor of new node");
		}

		for(int j=0;j<neighbor.size();j++)
		{
			(*neighborNodeIdx)[pIdx].push_back(neighbor[j]);
		}
	}


	// Update shape function at surface point
	Obj_GPU->updateShapeFunction(updatedPointIdx);
	surfObj->pointNormal()->resize(surfNode->size());
	surfObj->faceNormal()->resize(surfFace->size());
	surfObj->updateNormal();
	surfObj->dis()->resize(surfNode->size());
}

void cuttingTest::addNeighbor( Meshfree_GPU* obj_GPU, arrayInt& neighbor, arrayInt &potentialNeibor, int pIdx )
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

void cuttingTest::cutEFGObj(arrayVec3f cutP, arrayVec3i cutTri)
{
	EFG_CUDA_RUNTIME *obj = Obj_GPU->efgObj();
	VectorFunc func;
	GeometricFunc geoFunc;
	CollisionManager collision;

	std::vector<Vec3f>* efgNode=obj->nodePosVec();
	std::vector<Vec2i>* efgEdge=obj->edge();
	AABBTreeEdge* edfBVH=obj->getBVH();

	std::vector<int> colEdgeIdx;
	std::vector<Vec3f> intersectionPoint;

	// 1-1. Collision detection between cut surface and edge of the EFG
	collision.collisionBtwTriAndEdgesWithBVH(&cutP, &cutTri, efgNode, efgEdge, edfBVH, colEdgeIdx, intersectionPoint);
	func.arrangeVector(colEdgeIdx);

	// 1-a. Add pushing force to affected nodes
	arrayVec3f *force = Obj_GPU->compressForce();
	for (int i=0; i<colEdgeIdx.size(); i++)
	{
		Vec2i curE = efgEdge->at(colEdgeIdx[i]);
		Vec3f direct = efgNode->at(curE[1])-efgNode->at(curE[0]);
		direct.normalize();
		force->at(curE[0]) -= direct*COMPRESS_FORCE; // Should it proportion with edge length?
		force->at(curE[1]) += direct*COMPRESS_FORCE;
	}

	// 1-2. Remove intersected edges
	obj->removeEdges(colEdgeIdx);
}
