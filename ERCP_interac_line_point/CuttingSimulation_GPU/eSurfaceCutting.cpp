#include "StdAfx.h"
#include "eSurfaceCutting.h"
//#include "MeshRefineMent.h"
#include "../../include/DataTypes/vectorfunc.h"


arrayInt eSurfaceCutting::cutFaceIdx;


eSurfaceCutting::eSurfaceCutting(void)
{
}

eSurfaceCutting::~eSurfaceCutting(void)
{
}

bool eSurfaceCutting::cutting( SurfaceObj *surf, arrayVec3f *toolPoint)
{
	IdxOfCollideTri.clear();
	addedFaceIdx.clear();
	boundToTriangulate.clear();
	addedPoints.clear();
	addedFaces.clear();
	effectedPointIdx.clear();

	VectorFunc func;

	// 0. Prepare data
	m_surfObj = surf;
	m_surfPoints = surf->point();
	m_surfFaces = surf->face();
	m_toolPoint = toolPoint;
	AABBTree* surfBVH = surf->getBVH();

	float toolRadius = TOOL_RADIUS;
	TopologyContainer *surfContainer = m_surfObj->container();
	TopologyModifier modifier;
	int PreNbPoint = m_surfPoints->size();
	
	arrayVec3f addedPoints;
	arrayVec3i addedFaces;
	// 1.Find potential intersection triangle
	IdxOfCollideTri.clear();
	//Optimize later
	collisionBtwCyliderAndTriWithBVH(m_surfPoints, m_surfFaces, surfBVH, m_toolPoint, toolRadius, IdxOfCollideTri);
	func.arrangeVector(IdxOfCollideTri);
	//
	for (int i=0; i<IdxOfCollideTri.size(); i++)
	{
		Vec3i curF = m_surfFaces->at(IdxOfCollideTri[i]);
		effectedPointIdx.push_back(curF[0]);
		effectedPointIdx.push_back(curF[1]);
		effectedPointIdx.push_back(curF[2]);
	}

	func.arrangeVector(effectedPointIdx);

	return IdxOfCollideTri.size()>0;
}

void eSurfaceCutting::stepDebug()
{
	if (IdxOfCollideTri.size()==0)
	{
		return;
	}

	addedFaceIdx.clear();
	addedFaces.clear();
	addedPoints.clear();
	alledgeRemove.clear();
	AABBTreeTri* surfBVH = (AABBTreeTri*)m_surfObj->getBVH();
	
	float toolRadius = TOOL_RADIUS;
	TopologyContainer *surfContainer = m_surfObj->container();
	TopologyModifier modifier;
	int PreNbPoint = m_surfPoints->size();
	VectorFunc func;


	std::vector<arrayInt> triAreas;
	independendArea(IdxOfCollideTri, triAreas);

	for (int i=0; i<triAreas.size(); i++)
	{
		arrayInt triArea = triAreas[i];
		std::vector<arrayInt> boundaryLoops;

		if(!findBoundaryLoop(surfContainer, triArea, boundaryLoops))
		{
			releaseLog::urgentLog("Exeption on finding bound loop");
			return;
		}
		
		for (int j=0; j<boundaryLoops.size(); j++)
		{
			arrayInt boundaryLoop = boundaryLoops[j];
			fillConcaveTri(boundaryLoop);
			boundToTriangulate = boundaryLoop;

			if (boundaryLoop.size() > 0)
			{
				triangulate(boundaryLoop);
			}
		}
	}
	
	func.arrangeVector(alledgeRemove);
	// 2. Modify topology
	for (int i=IdxOfCollideTri.size()-1;i>=0;i--)
	{
		int rmIdx = IdxOfCollideTri[i];
		int changeIdx = m_surfFaces->size()-1;


		// Update BVH
		AABBNode* node = surfBVH->findLeafNode(changeIdx);
		if (node)
			node->IndexInLeafNode = rmIdx;
		node = surfBVH->findLeafNode(rmIdx);
		surfBVH->removeNode(node);

		modifier.removeFace(surfContainer, IdxOfCollideTri[i]);

		// Cut surface index
		int a1 = func.indexOfElement(&cutFaceIdx, rmIdx);
		if (a1!=-1)
			cutFaceIdx.erase(cutFaceIdx.begin()+a1);
		a1 = func.indexOfElement(&cutFaceIdx, changeIdx);
		if (a1!=-1)
			cutFaceIdx[a1]=rmIdx;	
	}

	modifier.removeEdges(surfContainer, alledgeRemove);
	int nbFace = m_surfFaces->size();

	for (int i=0; i<addedFaces.size(); i++)
	{
		modifier.addFace(surfContainer, addedFaces[i], PreNbPoint);
		AABBNode* newNode = new AABBNode;
		
		newNode->IndexInLeafNode = nbFace+i;
		Vec3f leftDown=Vec3f(MAX,MAX,MAX);
		Vec3f rightUp=Vec3f(MIN,MIN,MIN);
		Vec3i curTri = addedFaces[i];
		for (int j=0; j<3; j++)
		{
			for (int k=0; k<3; k++)
			{
				if((*m_surfPoints)[curTri[j]][k]<leftDown[k])
					leftDown[k]=(*m_surfPoints)[curTri[j]][k];
				if((*m_surfPoints)[curTri[j]][k]>rightUp[k])
					rightUp[k]=(*m_surfPoints)[curTri[j]][k];
			}
		}
		newNode->setBoundingBox(leftDown, rightUp);

		surfBVH->addNode(newNode);
	}
	int nbFace2 = m_surfFaces->size();

	for (int i =nbFace; i<nbFace2; i++)
	{
		addedFaceIdx.push_back(i);
	}

	// Update obj
	m_surfObj->updateNormal();

	// processing cut faces' index

	for (int i =nbFace; i<nbFace2; i++)
	{
		cutFaceIdx.push_back(i);
	}
//	func.arrangeVector(cutFaceIdx);
}

void eSurfaceCutting::stepDebug2()
{

}

void eSurfaceCutting::collisionBtwCyliderAndTri(  arrayVec3f* surfPoints, arrayVec3i* surfFaces, 
								arrayVec3f * toolPoint, float toolRadius, arrayInt &collideIdxs   )
{
	// a little bit hard code
	Vec3f l1 = toolPoint->at(0);
	Vec3f l2 = toolPoint->at(1);
	for (int i = 0; i<surfFaces->size(); i++)
	{
		float test = GeometricFunc::distanceBtwTriAndLine(surfPoints, (*surfFaces)[i], l1, l2);
		if (GeometricFunc::distanceBtwTriAndLine(surfPoints, (*surfFaces)[i], l1, l2) < toolRadius)
		{
			collideIdxs.push_back(i);
		}
	}
}


bool eSurfaceCutting::findBoundaryLoop(TopologyContainer* surfTopo, arrayInt idxOfRemoveTri, std::vector<arrayInt>& boundLoops)
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

	alledgeRemove.insert(alledgeRemove.end(), edgeRemove.begin(), edgeRemove.end());

	if (edgeOnBuondary.size()==0)
	{
		releaseLog::urgentLog("eSurfaceCutting::findBoundaryLoop");
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
					if (indexBelongtoArray(index, edgeOnBuondary) &&
						!indexBelongtoArray(index, edgeRemove))
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
				&& indexBelongtoArray(eIdx, edgeOnBuondary) 
				&& !indexBelongtoArray(eIdx, edgeRemove))
			{
				break;
			}
			if (!found)
			{
				return false;
			}
		}
		CHECK(boundLoop.size()>=3, "eSurfaceCutting::findBoundaryLoop3");
		conterClockWiseLoop(surfTopo, IdxOfCollideTri, boundLoop);
		boundLoops.push_back(boundLoop);
	}

	return true;
}

bool eSurfaceCutting::indexBelongtoArray( int index, arrayInt legalArray)
{
	bool legal=false;
	for(int i = 0; i<legalArray.size(); i++)
	{
		if (legalArray[i] == index)
		{
			legal = true;
			break;
		}
	}

	return legal;
}


void eSurfaceCutting::triangulateArea( arrayVec3f* surPoints, arrayInt& boundaryLoop, 
	arrayVec3f* toolPoint, float toolRadius, arrayVec3f& addedPoints, arrayVec3i& addedFaces )
{
	Vec3f l1 = (*toolPoint)[0];
	Vec3f l2 = (*toolPoint)[1];

	while(boundaryLoop.size() > 3)
	{
		int nbBound = boundaryLoop.size();
		int idx = findEarTriangle(surPoints, boundaryLoop, toolPoint, toolRadius);
		
		Vec3i newTri(boundaryLoop[(idx-1+nbBound)%nbBound], boundaryLoop[idx], boundaryLoop[(idx+1)%nbBound]);
		Vec3f normV, pOnTri, pOnLine;
		GeometricFunc::distanceBtwTriAndLine(surPoints, newTri, l1, l2, &pOnTri, &pOnLine);
		// normV = pOnLine-pOnTri;
		//counterClockWiseTri(surPoints, newTri, normV);
		
		addedFaces.push_back(newTri);
		boundaryLoop.erase(boundaryLoop.begin()+idx);
	}
	// Last triangle
	Vec3i newTri(boundaryLoop[0], boundaryLoop[1], boundaryLoop[2]);
	Vec3f normV, pOnTri, pOnLine;
	GeometricFunc::distanceBtwTriAndLine(surPoints, newTri, l1, l2, &pOnTri, &pOnLine);
// 	normV = pOnLine-pOnTri;
// 	counterClockWiseTri(surPoints, newTri, normV);

	addedFaces.push_back(newTri);
}

int eSurfaceCutting::findEarTriangle( arrayVec3f* surPoints, arrayInt& boundaryLoop, arrayVec3f* toolPoint, float toolRadius )
{
	Vec3f l1 = (*toolPoint)[0];
	Vec3f l2 = (*toolPoint)[1];
	int nbBoundPoints = boundaryLoop.size();
	for (int i = 0; i < boundaryLoop.size(); i++)
	{
		Vec3i candidateTri;
		candidateTri[0] = boundaryLoop[(i-1+nbBoundPoints)%nbBoundPoints];
		candidateTri[1] = boundaryLoop[i];
		candidateTri[2] = boundaryLoop[(i+1)%nbBoundPoints];

		Vec3f triPoint, linePoint;
		if (GeometricFunc::distanceBtwTriAndLine(surPoints, candidateTri, l1, l2,&triPoint,&linePoint) > toolRadius)
		{
			Vec3f triNorm = GeometricFunc::computeNormal(surPoints, candidateTri);
			if (triNorm*(linePoint-triPoint) > 0)
			{
				return i;
			}
		}
	}
	CHECK(0, "eSurfaceCutting::findEarTriangle"); //Can not find appropriate triangle
}


void eSurfaceCutting::conterClockWiseLoop( TopologyContainer* surfTopo, arrayInt areaTriIdx, arrayInt& boundPointLoop )
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

void eSurfaceCutting::remeshArea( arrayVec3f* surPoints, arrayInt& boundaryLoop, 
								 arrayVec3f& addedPoints, arrayVec3i& addedFaces )
{
	
}

void eSurfaceCutting::fillConcaveTri( arrayInt& trimmingLoop )
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

int eSurfaceCutting::findConcaveIndex( arrayInt trimmingLoop )
{
	arrayVec3f* pointNorm = m_surfObj->pointNormal();


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

		pt[0] = m_surfPoints->at(ptIdx[0]);
		pt[1] = m_surfPoints->at(ptIdx[1]);
		pt[2] = m_surfPoints->at(ptIdx[2]);

		float rr = func.distanceBtwTriAndLine(m_surfPoints, ptIdx, (*m_toolPoint)[0], (*m_toolPoint)[1]);
		if (rr<TOOL_RADIUS)
		{
			continue;
		}
		//Check counter clockwise also
		Vec3f norm;
		for (int j=0; j<3; j++)
		{
			norm = pointNorm->at(ptIdx[j]);
		}
		Vec3f curNorm = GeometricFunc::computeNormal(m_surfPoints, ptIdx);
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

void eSurfaceCutting::independendArea( arrayInt IdxOfCollideTri, std::vector<arrayInt>& triAreas )
{
	TopologyContainer* container = m_surfObj->container();
	std::vector<arrayInt> *faceAroundEdge = container->facesAroundEdge();

	arrayInt allTri = IdxOfCollideTri;
	while(allTri.size() > 0)
	{
		arrayInt newArea;
		newArea.push_back(allTri[0]);
		allTri.erase(allTri.begin());
		addNeighborTri(&allTri, &newArea, 0);

		triAreas.push_back(newArea);
	}
}

void eSurfaceCutting::addNeighborTri( arrayInt* allTri, arrayInt* areaTri, int triIdx)
{
	int curTri = areaTri->at(triIdx);
	TopologyContainer *container = m_surfObj->container();
	std::vector<arrayInt> *edgeInFace = container->edgesInFace();
	std::vector<arrayInt> *faceAroundEdge = container->facesAroundEdge();

	arrayInt curEs = edgeInFace->at(curTri);
	for (int i =0; i<curEs.size(); i++)
	{
		CHECK(faceAroundEdge->size()>curEs[i], "eSurfaceCutting::addNeighborTri 1");
		arrayInt neighborTri = (*faceAroundEdge)[curEs[i]];
		CHECK(neighborTri.size()>0, "eSurfaceCutting::addNeighborTri 2");
		int nTriIdx = neighborTri[0]==curTri? neighborTri[1]:neighborTri[0];

		//check if this neighbor lie inside cut area
		int idxIncutArea = -1;
		for (int j=0; j<allTri->size(); j++)
		{
			if (nTriIdx == allTri->at(j))
			{
				idxIncutArea = j;
				break;
			}
		}
		if (idxIncutArea!=-1)
		{
			areaTri->push_back(nTriIdx);
			allTri->erase(allTri->begin() + idxIncutArea);
			addNeighborTri(allTri, areaTri, areaTri->size()-1);
		}
	}
}

void eSurfaceCutting::triangulate( arrayInt boundLoop )
{
	if (boundLoop.size()==3)
	{
		addedFaces.push_back(Vec3i(boundLoop[0], boundLoop[1], boundLoop[2]));
		return;
	}
	// Divide and conquer
	Vec3f midPoint;
	for (int i=0; i<boundLoop.size(); i++)
	{
		midPoint += m_surfPoints->at(boundLoop[i]);
	}
	midPoint/=boundLoop.size();

	GeometricFunc func;
	float smallestDis = 9999;
	arrayInt boundLoop1, boundLoop2;
	for (int i=0; i<boundLoop.size(); i++)
	{
		for (int j=0; j<boundLoop.size(); j++)
		{
			if (i==j || abs(i-j)==1 || abs(i-j)==boundLoop.size()-1)
			{
				continue;
			}

			float dis = func.distanceBtwLineAndLine((*m_toolPoint)[0], (*m_toolPoint)[1],
										m_surfPoints->at(boundLoop[i]), m_surfPoints->at(boundLoop[j]));

			if (dis < TOOL_RADIUS)
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
			if (!(isLegalBound(bound1) && isLegalBound(bound2)))
			{
				continue;
			}

			Vec3f curMidP = (m_surfPoints->at(boundLoop[i]) + m_surfPoints->at(boundLoop[j]))/2;
			float disToMidPoint = (curMidP-midPoint).norm();
			if (smallestDis > disToMidPoint)
			{
				smallestDis = disToMidPoint;
				boundLoop1=bound1;
				boundLoop2=bound2;
			}
		}
	}

	CHECK(boundLoop1.size()>0, "eSurfaceCutting::triangulate");//We also should handle this

	triangulate(boundLoop1);
	triangulate(boundLoop2);
}

bool eSurfaceCutting::isLegalBound( arrayInt loop )//all point lie on a line
{
	GeometricFunc func;
	int nbPoint = loop.size();
	//Check all point not lie in a line
	Vec3f pt1 = m_surfPoints->at(loop[0]);
	Vec3f pt2 = m_surfPoints->at(loop[1]);
	bool allSame = true;
	for (int i=2; i<nbPoint; i++)
	{
		Vec3f pt = m_surfPoints->at(loop[i]);
		if (abs((pt-pt1)*(pt2-pt1)) > EPS)
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
		Vec3f toolP = m_toolPoint->at(0);
		Vec3f toolDirect = (*m_toolPoint)[1]-(*m_toolPoint)[0];

		Vec3f pt1 = m_surfPoints->at(loop[i]);
		Vec3f pt2 = m_surfPoints->at(loop[(i+1)%nbPoint]);
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

std::vector<int> eSurfaceCutting::changedPointIdx()
{
	return effectedPointIdx;
}

arrayInt eSurfaceCutting::boundLoop()
{
	return boundToTriangulate;
}


void eSurfaceCutting::collisionBtwCyliderAndTriWithBVH( arrayVec3f* surfPoints, arrayVec3i* surfFaces, AABBTree* surfBVH, arrayVec3f * toolPoint, float toolRadius, arrayInt &collideIdxs )
{
	arrayInt pTriIdx;
	traverseBVHPQ_line(surfBVH->root(), pTriIdx, toolPoint, toolRadius);

	Vec3f l1 = toolPoint->at(0);
	Vec3f l2 = toolPoint->at(1);
	for (int i = 0; i<pTriIdx.size(); i++)
	{
		if (GeometricFunc::distanceBtwTriAndLine(surfPoints, (*surfFaces)[pTriIdx[i]], l1, l2) < toolRadius)
		{
			collideIdxs.push_back(pTriIdx[i]);
		}
	}
}

void eSurfaceCutting::traverseBVHPQ_line( AABBNode* root, std::vector<int>& collidedTris, std::vector<Vec3f>* toolPoints, float radius )
{
	// 1. distance computation between bounding boxes 
	GeometricFunc func;
	Box box; box.leftDown=root->LeftDown; box.rightUp=root->RightUp; box.center=(box.leftDown+box.rightUp)/2.0;
	Vec3f center = box.center;

	float dis = radius + (root->RightUp - root->LeftDown).norm()/2;

	if(func.distanceBtwPointAndLine(center, (*toolPoints)[0], (*toolPoints)[1])<dis)
	{
		if(root->End)
		{
			collidedTris.push_back(root->IndexInLeafNode);
			return;
		}
		else
		{
			traverseBVHPQ_line(root->Left, collidedTris, toolPoints, radius);
			traverseBVHPQ_line(root->Right, collidedTris, toolPoints, radius);
		}
	}
}