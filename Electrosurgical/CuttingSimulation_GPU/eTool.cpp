#include "StdAfx.h"
#include "eTool.h"
#include "Utility.h"

#include "GL/GL.h"

eTool::eTool(void)
{
	
	knife.readObjData("../data/needle.txt");
	knife.translate(Vec3f(0,350,0));

	arrayVec3f* knifeP = knife.point();
	arrayVec3i* knifeF = knife.face();
	arrayVec3f* knifePNorm = knife.pointNormal();

	points = *knifeP;
	tris = *knifeF;
	normPoints = *knifePNorm;

// 	points.push_back(Vec3f(0,0,1));
// 	points.push_back(Vec3f(20,0,0));
// 	points.push_back(Vec3f(0,100,0));
// 	tris.push_back(Vec3i(0,1,2));
// 
// 	Vec3f norm(-1,-1,0);norm.normalize();
// 	normPoints.push_back(norm);
// 	norm = Vec3f(1,-1,0);norm.normalize();
// 	normPoints.push_back(norm);
// 	norm = Vec3f(-1,1,0);norm.normalize();
// 	normPoints.push_back(norm);
}

eTool::~eTool(void)
{
}

void eTool::draw(int mode)
{
	if (mode == 0)
	{
		glColor3f(0,1,0);
		glBegin(GL_TRIANGLES);
		for (int i=0; i<tris.size(); i++)
		{
			Vec3i _tri = tris[i];
			for (int j=0; j<3; j++)
			{
				glVertex3f(points[_tri[j]][0], points[_tri[j]][1], points[_tri[j]][2]);
			}
		}
		glEnd();

		glColor3f(0,0,1);
		glBegin(GL_LINES);
		for (int i=0; i<tris.size(); i++)
		{
			Vec3i _tri = tris[i];
			for (int j=0; j<3; j++)
			{
				glVertex3f(points[_tri[j]][0], points[_tri[j]][1], points[_tri[j]][2]);
				glVertex3f(points[_tri[(j+1)%3]][0], points[_tri[(j+1)%3]][1], points[_tri[(j+1)%3]][2]);
			}
		}
		glEnd();
	}


	if (mode == 1)
		volMng.draw(0);
}

void eTool::cut2( SurfaceObj* obj )
{
	surfObj = obj;
	AABBTree* BVH = obj->getBVH();
	objPoints = obj->point();
	objTris = obj->face();
	topo = obj->container();
	arrayVec3f* objPointNorm = obj->pointNormal();
	VectorFunc func;
	preNbPoint = objPoints->size();
	
	// Find intersection triangles
	idxOfCollidedTris.clear();

	arrayInt toolCollidedIdx;
	CollisionManager colMng;
	colMng.collisionBtwTrisAndTrisWithBVH(&points, &tris, objPoints, objTris, BVH, toolCollidedIdx, idxOfCollidedTris);

	// Boundary loop
	alledgeRemove.clear();
	boundLoops.clear();
	findBoundaryLoop(topo, idxOfCollidedTris, boundLoops);
	func.arrangeVector(alledgeRemove);

	// generate mesh
	addedPoints.clear()	;
	addedFaces.clear();
	bound = boundLoops[0];
	normBoundLoop = Vec3f();
	for (int i=0; i<bound.size(); i++)
	{
		normBoundLoop += objPointNorm->at(bound[i]);
	}
	normBoundLoop.normalize();

	generate1Tri(bound);

	// Modify surface
	TopologyModifier modifier;
	modifier.removeEdges(topo, alledgeRemove);
	modifier.removeFaces(topo, idxOfCollidedTris);

//	modifier.addPoints(topo, addedPoints);
	modifier.addFaces(topo, addedFaces, preNbPoint);

	obj->updateNormal();
}


bool eTool::findBoundaryLoop(TopologyContainer* surfTopo, arrayInt idxOfRemoveTri, std::vector<arrayInt>& boundLoops)
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
		conterClockWiseLoop(surfTopo, idxOfCollidedTris, boundLoop);
		boundLoops.push_back(boundLoop);
	}

	return true;
}

void eTool::conterClockWiseLoop( TopologyContainer* surfTopo, arrayInt areaTriIdx, arrayInt& boundPointLoop )
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
bool eTool::indexBelongtoArray( int index, arrayInt legalArray)
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

void eTool::generate1Tri( arrayInt &bound )
{
	// choose potential edge
	int nbPoint = bound.size();
	int ptIdx[2];
	ptIdx[0] = 0; // Now hard code. this is index in boundary array: bound
	ptIdx[1] = 1;
 
	// Check if it can join with neighbor
	for (int i=0; i<2; i++)
	{
		int p = ptIdx[i];

		Vec3i po_tri1;
		po_tri1[0]=bound[(p-1+nbPoint) % nbPoint];
		po_tri1[1] = bound[p];
		po_tri1[2] = bound[(p+1) % nbPoint];
		if (isLegalTri(po_tri1))
		{
			addedFaces.push_back(po_tri1);
			bound.erase(bound.begin()+p);
//			generate1Tri(bound);
			return;
		}
	}

	// Cannot join with neighbor, so we have to add points
	Vec3f newP = generatePoint(bound, ptIdx);
	
	TopologyModifier modifier;
	modifier.addPoint(topo, newP);
	addedFaces.push_back(Vec3i(bound[ptIdx[0]], bound[ptIdx[1]], objPoints->size()-1));


	bound.insert(bound.begin()+ptIdx[1], objPoints->size()-1);
//	generate1Tri(bound);
}

BOOL eTool::isLegalTri( Vec3i _tri ) // index 1 is potential corner
{
	Vec3f pt[3];
	pt[0] = objPoints->at(_tri[0]);
	pt[1] = objPoints->at(_tri[1]);
	pt[2] = objPoints->at(_tri[2]);
	
	// angle < THRES_HOLD
	Vec3f ptV1 = pt[0]-pt[1];
	Vec3f ptV2 = pt[2]-pt[1];

	float angle = acos(ptV1*ptV2/(ptV1.norm()*ptV2.norm())); // We should compare cos() directly. Not acos
	if (angle > ANGLE_THRES_HOLD)
	{
		return FALSE;
	}

	// does not intersect with cut tool
	GeometricFunc geoFunc;
	for (int i=0; i<tris.size(); i++)
	{
		Vec3f toolTri[3];
		toolTri[0] = points[tris[i][0]];
		toolTri[1] = points[tris[i][1]];
		toolTri[2] = points[tris[i][2]];

		if(geoFunc.collisionBtwTri(pt, toolTri))
			return FALSE;
	}

	return TRUE;
}

Vec3f eTool::generatePoint( arrayInt & bound, int* ptIdx )
{
	Vec3f pt1 = objPoints->at(bound[ptIdx[0]]);
	Vec3f pt2 = objPoints->at(bound[ptIdx[1]]);
	// Find potential around points
	arrayInt nearByIdxs;
	// now we hard code
// 	nearByIdxs.push_back(0);
// 	nearByIdxs.push_back(1);
	nearByIdxs.push_back(2);
	
	// Find the most outer direction
	Vec3f normDirect = directNorm(pt1, pt2, pt1+normBoundLoop); // should be change depend on the edge is old or new
	float largest = 0;
	int tIdx = -1;
	
	for (int i=0; i<nearByIdxs.size(); i++)
	{
		Vec3f curDirect = directNorm(pt1, pt2, points[nearByIdxs[i]]);
		
		float angle = angleCW(pt2-pt1, normDirect, curDirect);
		if (largest < angle)
		{
			largest = angle;
			tIdx = nearByIdxs[i];
		}
	}
	ASSERT(tIdx!= -1);

	Vec3f normP = normPoints[tIdx];
	Vec3f ddirect = directNorm(pt1, pt2, points[tIdx] + normP*FACE_DISTANCE); // Use normal vector or adjust angle???
	float l = (pt2-pt1).norm()*sqrt(3.0)/2;
	Vec3f newP = (pt1+pt2)/2 + ddirect*l;

	return newP;
}

// Find co-planar vector that is perpendicular to {ePt1, ePt2}
Vec3f eTool::directNorm( Vec3f ePt1, Vec3f ePt2, Vec3f pt )
{
	Vec3f norm = (ePt2-ePt1).cross(pt-ePt1);
	Vec3f direct = norm.cross(ePt2-ePt1);
	direct.normalize();
	return direct;
}

float eTool::angleCW( Vec3f norm, Vec3f v1, Vec3f v2 ) // v1 and v2 should be normalized
{
	Vec3f _norm = norm;
	_norm.normalize();

	float _cos = v1*v2;
	float _sin = (v1.cross(v2))*_norm;

	float angle = atan2(_sin, _cos);
	if (angle < 0)
	{
		angle = angle + 2*PI;
	}
	return angle;
}

void eTool::testAdd()
{
	addedFaces.clear();
	generate1Tri(bound);

	TopologyModifier modifier;
	modifier.addFaces(topo, addedFaces, preNbPoint);

	surfObj->updateNormal();
}

void eTool::move( Vec3f m )
{
	for (int i=0; i<points.size(); i++)
	{
		points[i] += m;
	}
}

void eTool::cut3( SurfaceObj* obj )
{
	surfObj = obj;
	AABBTreeTri* BVH = (AABBTreeTri*)obj->getBVH();
	objPoints = obj->point();
	objTris = obj->face();
	topo = obj->container();
	arrayVec3f* objPointNorm = obj->pointNormal();
	VectorFunc func;
	preNbPoint = objPoints->size();

	// Find intersection triangles
	idxOfCollidedTris.clear();

	arrayInt toolCollidedIdx;
	CollisionManager colMng;
	colMng.collisionBtwTrisAndTrisWithBVH(&points, &tris, objPoints, objTris, BVH, toolCollidedIdx, idxOfCollidedTris);
	func.arrangeVector(idxOfCollidedTris);

	// Boundary loop
	alledgeRemove.clear();
	boundLoops.clear();
	findBoundaryLoop(topo, idxOfCollidedTris, boundLoops);
	func.arrangeVector(alledgeRemove);

	// Find all point inside 
	arrayInt pIdxInside = pointInsideObject(&points, &tris, obj);
	
	// Triangulate convex hull
	bound = boundLoops[0];
	// We separate the problem
	arrayVec3f cPoints;
	int nbBoundaryPoints;
	nbBoundaryPoints = bound.size();
	for (int i=0; i<nbBoundaryPoints; i++)
	{
		cPoints.push_back(objPoints->at(bound[i]));
	}
	for (int i=0; i<pIdxInside.size(); i++)
	{
		cPoints.push_back(points[pIdxInside[i]] + normPoints[pIdxInside[i]]*FACE_DISTANCE);
	}
	arrayVec3i convexFace = findConvexHull(cPoints, nbBoundaryPoints);

	// Convert back to Problem
	addedPoints.clear();
	addedFaces.clear();
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

	// Modify surface
	TopologyModifier modifier;
// 	modifier.removeEdges(topo, alledgeRemove);
// 	modifier.removeFaces(topo, idxOfCollidedTris);
// 
 	modifier.addPoints(topo, addedPoints);
// 	modifier.addFaces(topo, addedFaces, preNbPoint);
	for (int i=idxOfCollidedTris.size()-1;i>=0;i--)
	{
		int rmIdx = idxOfCollidedTris[i];
		int changeIdx = objTris->size()-1;


		// Update BVH
		AABBNode* node = BVH->findLeafNode(changeIdx);
		if (node)
			node->IndexInLeafNode = rmIdx;
		node = BVH->findLeafNode(rmIdx);
		BVH->removeNode(node);

		modifier.removeFace(topo, idxOfCollidedTris[i]);
	}

	modifier.removeEdges(topo, alledgeRemove);
	int nbFace = objTris->size();

	for (int i=0; i<addedFaces.size(); i++)
	{
		modifier.addFace(topo, addedFaces[i], preNbPoint);
		AABBNode* newNode = new AABBNode;

		newNode->IndexInLeafNode = nbFace+i;
		Vec3f leftDown=Vec3f(MAX_INT,MAX_INT,MAX_INT);
		Vec3f rightUp=Vec3f(MIN_INT,MIN_INT,MIN_INT);
		Vec3i curTri = addedFaces[i];
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
	}

	obj->updateNormal();
	obj->updateBVH();

	newFaceIdx.clear();
	for (int i=nbFace; i<objTris->size(); i++)
	{
		newFaceIdx.push_back(i);
	}
}

arrayInt eTool::pointInsideObject( arrayVec3f* toolPoints, arrayVec3i* toolTris, SurfaceObj* obj )
{
	arrayInt ptInside;
	CollisionManager colMng;
	for (int i=0; i<toolPoints->size(); i++)
	{
		if (colMng.isPointInSurfObj(obj, toolPoints->at(i)))
		{
			ptInside.push_back(i);
		}
	}

	return ptInside;
}

arrayVec3i eTool::findConvexHull( arrayVec3f cPoints, int nbBoundaryPoints )
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

void eTool::addPointToConvex( arrayVec3f _cPoints, int pIdx, arrayVec3i& _cFace )
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

void eTool::drawCutInfo( SurfaceObj* obj )
{
	arrayInt triIdxToDraw = generatedIdx;

	surfObj = obj;
	objPoints = obj->point();
	objTris = obj->face();
	topo = obj->container();

	glColor3f(0.5,0,0.5);
	glBegin(GL_TRIANGLES);
	for (int i=0; i<triIdxToDraw.size(); i++)
	{
		Vec3i _tri = objTris->at(triIdxToDraw[i]);
		for (int j=0; j<3; j++)
		{
			glVertex3f((*objPoints)[_tri[j]][0], (*objPoints)[_tri[j]][1], (*objPoints)[_tri[j]][2]);
		}
	}
	glEnd();

	glColor3f(1,0,0);
	glBegin(GL_LINES);
	for (int i=0; i<triIdxToDraw.size(); i++)
	{
		Vec3i _tri = objTris->at(triIdxToDraw[i]);
		for (int j=0; j<3; j++)
		{
			glVertex3f((*objPoints)[_tri[j]][0], (*objPoints)[_tri[j]][1], (*objPoints)[_tri[j]][2]);
			glVertex3f((*objPoints)[_tri[(j+1)%3]][0], (*objPoints)[_tri[(j+1)%3]][1], (*objPoints)[_tri[(j+1)%3]][2]);
		}
	}
	glEnd();
}

void eTool::cut4( SurfaceObj* obj )
{
	surfObj = obj;
	AABBTreeTri* BVH = (AABBTreeTri*)obj->getBVH();
	objPoints = obj->point();
	objTris = obj->face();
	topo = obj->container();
	arrayVec3f* objPointNorm = obj->pointNormal();
	VectorFunc func;
	preNbPoint = objPoints->size();

	// Find intersection triangles
	idxOfCollidedTris.clear();

	arrayInt toolCollidedIdx;
	CollisionManager colMng;
	colMng.collisionBtwTrisAndTrisWithBVH(&points, &tris, objPoints, objTris, BVH, toolCollidedIdx, idxOfCollidedTris);
	func.arrangeVector(idxOfCollidedTris);

	// Incremental tetrahedral
	addedPoints.clear();
	addedFaces.clear();

	generatedIdx = idxOfCollidedTris;

// 	while(1)
// 	{
// 		bool bIntersected = false;
// 		for (int i=0; i<idxOfCollidedTris.size(); i++)
// 		{
// 			Vec3i tri = objTris->at(idxOfCollidedTris[i]);
// 			Vec3f triPoint[3];
// 			triPoint[0] = objPoints->at(tri[0]); 
// 			triPoint[1] = objPoints->at(tri[1]); 
// 			triPoint[2] = objPoints->at(tri[2]); 
// 			if (isTriIntersectWithTool(triPoint))
// 			{
// 				// We should choose the good one, but now, it's OK, we choose any thing
// 				bIntersected = true;
// 				generateTetrahedral(idxOfCollidedTris[i]);
// 				break;
// 			}
// 		}
// 
// 		if (!bIntersected)
// 		{
// 			break;
// 		}
// 	}
}

bool eTool::isTriIntersectWithTool( Vec3f _tri[] )
{
	GeometricFunc func;
	for (int i=0; i<tris.size(); i++)
	{
		Vec3f curTri[3];
		curTri[0] = points[tris[i][0]];
		curTri[1] = points[tris[i][1]];
		curTri[2] = points[tris[i][2]];
		if(func.collisionBtwTri(_tri, curTri))
			return true;
	}
	return false;
}

void eTool::generateTetrahedral(  int triIdx  )
{
	GeometricFunc func;
	VectorFunc VecFunc;
	std::vector<arrayInt>* edgeOnFace = topo->edgesInFace();
	arrayVec2i* edges = topo->edge();


	// First check if it can join with other points on surface
	Vec3i tri = objTris->at(triIdx);
	Vec3f normV = -func.computeNormal(objPoints, tri);
	Vec3f midPoint = (objPoints->at(tri[0]) + objPoints->at(tri[1]) + objPoints->at(tri[2]))/3;
	float triSize = (	(objPoints->at(tri[1])-objPoints->at(tri[0])).norm() +
		(objPoints->at(tri[2])-objPoints->at(tri[1])).norm() + 
		(objPoints->at(tri[0])-objPoints->at(tri[2])).norm() )/3.0;

	if (triSize*MAX_DISTANCE_RATIO > MAX_DISTANCE_TETRA)
	{
		triSize = MAX_DISTANCE_TETRA/MAX_DISTANCE_RATIO;
	}

	// If the normal line intersect with surface, it is force to join with neighbor
	Vec3f temporalP = midPoint+normV*triSize;
	CollisionManager colMng;
	bool intersectWithSurface = colMng.collisionBtwSurfAndLineSeg(surfObj, midPoint, temporalP);

	if (!intersectWithSurface)
	{
		arrayInt edgesOnIt = edgeOnFace->at(triIdx);
		for (int i=0; i<edgesOnIt.size(); i++)
		{
			int edgeIdx = edgesOnIt[i];
			Vec2i shareE = edges->at(edgeIdx);
			arrayInt faceAroundThisEdge = topo->facesAroundEdge(edgeIdx);

			int neighborIdx = faceAroundThisEdge[0]==triIdx? faceAroundThisEdge[1]:faceAroundThisEdge[0];
			Vec3i neighborPoint = (*objTris)[neighborIdx];
			int point4Idx = -1;
			for (int j=0; j<3; j++)
			{
				if (neighborPoint[j]!=shareE[0] && neighborPoint[j]!=shareE[1])
				{
					point4Idx = neighborPoint[j];
					break;
				}
			}
			ASSERT(point4Idx!=-1);

			Vec3f point4 = (*objPoints)[point4Idx];
			// Check if this tetrahedral have good shape
			Vec3f direct = point4 - midPoint;
			if (direct.norm() < MAX_DISTANCE_RATIO*triSize && COS_ANGLE(direct,normV)>MAX_ANGLE)
			{
				// Good tetrahedral
				arrayInt _removeIdx;
				arrayVec3i _addedFaces;
				_removeIdx.push_back(triIdx);
				_removeIdx.push_back(neighborIdx);

				int _point3LocalIdx = -1;
				for (int j=0; j<3; j++)
				{
					if (tri[j]!= shareE[0] && tri[j]!=shareE[1])
						_point3LocalIdx = j;
				}

				_addedFaces.push_back(Vec3i(tri[(_point3LocalIdx+2)%3], tri[_point3LocalIdx], point4Idx));
				_addedFaces.push_back(Vec3i(tri[_point3LocalIdx], tri[(_point3LocalIdx+1)%3], point4Idx));

				// Update topology
				objTopoModifer(_removeIdx, _addedFaces);
				return;
			}
		}
	}
	else // Force join with a surface point
	{
		arrayInt edgesOnIt = edgeOnFace->at(triIdx);
		float maxCosAngle = 0;
		int pointIdx = -1;

		Vec2i _shareE;
		int _neighborIdx;
		for (int i=0; i<edgesOnIt.size(); i++)
		{
			int edgeIdx = edgesOnIt[i];
			Vec2i shareE = edges->at(edgeIdx);
			arrayInt faceAroundThisEdge = topo->facesAroundEdge(edgeIdx);

			int neighborIdx = faceAroundThisEdge[0]==triIdx? faceAroundThisEdge[1]:faceAroundThisEdge[0];
			Vec3i neighborPoint = (*objTris)[neighborIdx];
			int point4Idx = -1;
			for (int j=0; j<3; j++)
			{
				if (neighborPoint[j]!=shareE[0] && neighborPoint[j]!=shareE[1])
				{
					point4Idx = neighborPoint[j];
					break;
				}
			}
			ASSERT(point4Idx!=-1);

			Vec3f point4 = (*objPoints)[point4Idx];
			// Check if this tetrahedral have good shape
			Vec3f direct = point4 - midPoint;
			if (maxCosAngle < COS_ANGLE(direct,normV))
			{
				maxCosAngle = COS_ANGLE(direct,normV);
				pointIdx = point4Idx;
				_shareE = shareE;
				_neighborIdx = neighborIdx;
			}

		}	

		if (pointIdx!= -1)
		{
			// Good tetrahedral
			arrayInt _removeIdx;
			arrayVec3i _addedFaces;
			_removeIdx.push_back(triIdx);
			_removeIdx.push_back(_neighborIdx);

			int _point3LocalIdx = -1;
			for (int j=0; j<3; j++)
			{
				if (tri[j]!= _shareE[0] && tri[j]!=_shareE[1])
					_point3LocalIdx = j;
			}

			_addedFaces.push_back(Vec3i(tri[(_point3LocalIdx+2)%3], tri[_point3LocalIdx], pointIdx));
			_addedFaces.push_back(Vec3i(tri[_point3LocalIdx], tri[(_point3LocalIdx+1)%3], pointIdx));

			// Update topology
			objTopoModifer(_removeIdx, _addedFaces);
			return;
		}
	}


	// Generate a tetrahedral
	Vec3f newP = midPoint + normV*triSize;

	arrayVec3f _addP;
	arrayInt _removeIdx;
	arrayVec3i _addedFaces;

	_addP.push_back(newP);
	_removeIdx.push_back(triIdx);
	int newPIdx = objPoints->size();
	for (int i=0; i<3; i++)
	{
		_addedFaces.push_back(Vec3i(tri[i], tri[(i+1)%3], newPIdx));
	}
	// Update topology
	objTopoModifer(_removeIdx, _addedFaces, &_addP);

	// Update affected idx
// 	VecFunc.removeElementValue(idxOfCollidedTris, triIdx);
// 	idxOfCollidedTris.push_back(objTris->size()-1);
// 	idxOfCollidedTris.push_back(objTris->size()-2);
// 	idxOfCollidedTris.push_back(objTris->size()-3);	
}

void eTool::objTopoModifer( arrayInt _removedTriIdx, arrayVec3i _addFace, arrayVec3f* _addPoint /*= NULL*/ )
{
	AABBTreeTri* BVH = (AABBTreeTri*)surfObj->getBVH();
	TopologyModifier modifier;
	int _preNbPoint = objPoints->size();
	VectorFunc vecFunc;

	// Edge will be removed
	VectorFunc func;
	alledgeRemove.clear();
	boundLoops.clear();
	findBoundaryLoop(topo, _removedTriIdx, boundLoops);
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
		// For idea 4
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
		Vec3f leftDown=Vec3f(MAX_INT,MAX_INT,MAX_INT);
		Vec3f rightUp=Vec3f(MIN_INT,MIN_INT,MIN_INT);
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

void eTool::testAdd4()
{
	bool bIntersected = false;
	for (int i=0; i<generatedIdx.size(); i++)
	{
		Vec3i tri = objTris->at(generatedIdx[i]);
		Vec3f triPoint[3];
		triPoint[0] = objPoints->at(tri[0]); 
		triPoint[1] = objPoints->at(tri[1]); 
		triPoint[2] = objPoints->at(tri[2]); 
		if (isTriIntersectWithTool(triPoint))
		{
			// We should choose the good one, but now, it's OK, we choose any thing
			bIntersected = true;
			generateTetrahedral(generatedIdx[i]);
			break;
		}
	}
}

void eTool::drawCutInfoDetail( SurfaceObj* obj, int mode )
{
	/*
	arrayInt triIdxToDraw = generatedIdx;

	surfObj = obj;
	objPoints = obj->point();
	objTris = obj->face();
	topo = obj->container();

	glColor3f(0,1,0);
	for (int i=0; i<triIdxToDraw.size(); i++)
	{
		Vec3i _tri = objTris->at(triIdxToDraw[i]);
		Vec3f midPoint;
		for (int j=0; j<3; j++)
			midPoint += (*objPoints)[_tri[j]];
		midPoint /= 3;

		Utility::printw(midPoint[0], midPoint[1], midPoint[2], " %d", triIdxToDraw[i]);
	}*/
	
	for (int i=0; i<cPoints.size(); i++)
	{
		Vec3f curP = cPoints[i];
		Utility::printw(curP[0], curP[1], curP[2], " %d", i);

		glColor3f(0,0,1);
		glPointSize(3.0);
		glBegin(GL_POINTS);
		glVertex3f(curP[0], curP[1], curP[2]);
		glEnd();
	}

	glColor3f(0.5,0.4,0);
	glBegin(GL_TRIANGLES);
	for (int i=0; i<cFace.size(); i++)
	{
		for (int j=0; j<3; j++)
		{
			Vec3f curP = cPoints[cFace[i][j]];
			glVertex3f(curP[0], curP[1], curP[2]);
		}
	}
	glEnd();

	glColor3f(0.0,0.4,0.3);
	glBegin(GL_LINES);
	for (int i=0; i<cFace.size(); i++)
	{
		for (int j=0; j<3; j++)
		{
			Vec3f curP = cPoints[cFace[i][j]];
			glVertex3f(curP[0], curP[1], curP[2]);
			curP = cPoints[cFace[i][(j+1)%3]];
			glVertex3f(curP[0], curP[1], curP[2]);
		}
	}
	glEnd();

}

void eTool::generateTetrahedralUsingEFGNode( int triIdx, Meshfree_GPU* _meshFreeObj)
{
	GeometricFunc func;
	VectorFunc VecFunc;
	std::vector<arrayInt>* edgeOnFace = topo->edgesInFace();
	arrayVec2i* edges = topo->edge();


	// First check if it can join with other points on surface
	Vec3i tri = objTris->at(triIdx);
	Vec3f normV = -func.computeNormal(objPoints, tri);
	Vec3f midPoint = (objPoints->at(tri[0]) + objPoints->at(tri[1]) + objPoints->at(tri[2]))/3;
	float triSize = (	(objPoints->at(tri[1])-objPoints->at(tri[0])).norm() +
		(objPoints->at(tri[2])-objPoints->at(tri[1])).norm() + 
		(objPoints->at(tri[0])-objPoints->at(tri[2])).norm() )/3.0;

	// If the normal line intersect with surface, it is force to join with neighbor
	Vec3f temporalP = midPoint+normV*triSize;
	CollisionManager colMng;
	bool intersectWithSurface = colMng.collisionBtwSurfAndLineSeg(surfObj, midPoint, temporalP);

	if (!intersectWithSurface)
	{
		arrayInt edgesOnIt = edgeOnFace->at(triIdx);
		for (int i=0; i<edgesOnIt.size(); i++)
		{
			int edgeIdx = edgesOnIt[i];
			Vec2i shareE = edges->at(edgeIdx);
			arrayInt faceAroundThisEdge = topo->facesAroundEdge(edgeIdx);

			int neighborIdx = faceAroundThisEdge[0]==triIdx? faceAroundThisEdge[1]:faceAroundThisEdge[0];
			Vec3i neighborPoint = (*objTris)[neighborIdx];
			int point4Idx = -1;
			for (int j=0; j<3; j++)
			{
				if (neighborPoint[j]!=shareE[0] && neighborPoint[j]!=shareE[1])
				{
					point4Idx = neighborPoint[j];
					break;
				}
			}
			ASSERT(point4Idx!=-1);

			Vec3f point4 = (*objPoints)[point4Idx];
			// Check if this tetrahedral have good shape
			Vec3f direct = point4 - midPoint;
			if (direct.norm() < MAX_DISTANCE_RATIO*triSize && COS_ANGLE(direct,normV)>MAX_ANGLE)
			{
				// Good tetrahedral
				arrayInt _removeIdx;
				arrayVec3i _addedFaces;
				_removeIdx.push_back(triIdx);
				_removeIdx.push_back(neighborIdx);

				int _point3LocalIdx = -1;
				for (int j=0; j<3; j++)
				{
					if (tri[j]!= shareE[0] && tri[j]!=shareE[1])
						_point3LocalIdx = j;
				}

				_addedFaces.push_back(Vec3i(tri[(_point3LocalIdx+2)%3], tri[_point3LocalIdx], point4Idx));
				_addedFaces.push_back(Vec3i(tri[_point3LocalIdx], tri[(_point3LocalIdx+1)%3], point4Idx));

				// Update topology
				objTopoModifer(_removeIdx, _addedFaces);
				return;
			}
		}
	}
	else // Force join with a surface point
	{
		arrayInt edgesOnIt = edgeOnFace->at(triIdx);
		float maxCosAngle = 0;
		int pointIdx = -1;
	
		Vec2i _shareE;
		int _neighborIdx;
		for (int i=0; i<edgesOnIt.size(); i++)
		{
			int edgeIdx = edgesOnIt[i];
			Vec2i shareE = edges->at(edgeIdx);
			arrayInt faceAroundThisEdge = topo->facesAroundEdge(edgeIdx);

			int neighborIdx = faceAroundThisEdge[0]==triIdx? faceAroundThisEdge[1]:faceAroundThisEdge[0];
			Vec3i neighborPoint = (*objTris)[neighborIdx];
			int point4Idx = -1;
			for (int j=0; j<3; j++)
			{
				if (neighborPoint[j]!=shareE[0] && neighborPoint[j]!=shareE[1])
				{
					point4Idx = neighborPoint[j];
					break;
				}
			}
			ASSERT(point4Idx!=-1);

			Vec3f point4 = (*objPoints)[point4Idx];
			// Check if this tetrahedral have good shape
			Vec3f direct = point4 - midPoint;
			if (maxCosAngle < COS_ANGLE(direct,normV))
			{
				maxCosAngle = COS_ANGLE(direct,normV);
				pointIdx = point4Idx;
				_shareE = shareE;
				_neighborIdx = neighborIdx;
			}

		}	

		if (pointIdx!= -1)
		{
			// Good tetrahedral
			arrayInt _removeIdx;
			arrayVec3i _addedFaces;
			_removeIdx.push_back(triIdx);
			_removeIdx.push_back(_neighborIdx);

			int _point3LocalIdx = -1;
			for (int j=0; j<3; j++)
			{
				if (tri[j]!= _shareE[0] && tri[j]!=_shareE[1])
					_point3LocalIdx = j;
			}

			_addedFaces.push_back(Vec3i(tri[(_point3LocalIdx+2)%3], tri[_point3LocalIdx], pointIdx));
			_addedFaces.push_back(Vec3i(tri[_point3LocalIdx], tri[(_point3LocalIdx+1)%3], pointIdx));

			// Update topology
			objTopoModifer(_removeIdx, _addedFaces);
			return;
		}
	}


	// Cannot join with surface. We check if it can join with EFG node
	// We may optimize with BVH tree later
	{
		EFG_CUDA_RUNTIME* efgObj  = _meshFreeObj->efgObj();
		arrayVec3f* efgNode = efgObj->nodePosVec();

		float maxAngle = 0;
		int nodeIdx = -1;
		for (int i=0; i<efgNode->size(); i++)
		{
			Vec3f direct = efgNode->at(i)-midPoint;
			if (direct.norm() < RADIO_TETRAHEDRAL)
			{
				if (maxAngle < direct*normV)
				{
					maxAngle = direct*normV;
					nodeIdx = i;
				}
			}
		}

		ASSERT(nodeIdx!=-1);

		// Add new tetra node
		Vec3f newP = efgNode->at(nodeIdx);

		// Check if the point is already added.
		// Now we dont care performance
		int existedIdx = -1;
		if (VecFunc.isElementInVector(mappedEfgNode, nodeIdx))
		{
			// Find corresponding point index, we may use a map, but we will need to update it
			for (int i=0; i<objPoints->size(); i++)
			{
				if ((objPoints->at(i) - newP).norm() < EPS)
				{
					existedIdx = i;
					break;
				}
			}
		}

		arrayVec3f _addP;
		arrayInt _removeIdx;
		arrayVec3i _addedFaces;

		int newPIdx = objPoints->size();

		if (existedIdx == -1)
		{
			_addP.push_back(newP);
			_removeIdx.push_back(triIdx);
			mappedEfgNode.push_back(nodeIdx);
		}
		else
		{
			_removeIdx.push_back(triIdx);
			newPIdx = existedIdx;
		}

		for (int i=0; i<3; i++)
		{
			_addedFaces.push_back(Vec3i(tri[i], tri[(i+1)%3], newPIdx));
		}
		// Update topology
		objTopoModifer(_removeIdx, _addedFaces, &_addP);
	}

}

void eTool::testAdd5( Meshfree_GPU* _meshFreeObj )
{
	bool bIntersected = false;
	for (int i=0; i<generatedIdx.size(); i++)
	{
		Vec3i tri = objTris->at(generatedIdx[i]);
		Vec3f triPoint[3];
		triPoint[0] = objPoints->at(tri[0]); 
		triPoint[1] = objPoints->at(tri[1]); 
		triPoint[2] = objPoints->at(tri[2]); 
		if (isTriIntersectWithTool(triPoint))
		{
			// We should choose the good one, but now, it's OK, we choose any thing
			bIntersected = true;
			generateTetrahedralUsingEFGNode(generatedIdx[i], _meshFreeObj);
			break;
		}
	}
}

void eTool::cut6( SurfaceObj* obj )
{
	surfObj = obj;
	AABBTreeTri* BVH = (AABBTreeTri*)obj->getBVH();
	objPoints = obj->point();
	objTris = obj->face();
	topo = obj->container();
	arrayVec3f* objPointNorm = obj->pointNormal();
	VectorFunc func;
	preNbPoint = objPoints->size();

	// Voxel
	volMng.init(&points, &tris);

	// intersection with volume
	idxOfCollidedTris.clear();
	volMng.collisionDetectionWithObject(obj, idxOfCollidedTris);
	generatedIdx = idxOfCollidedTris;
	
	// Boundary loop
	alledgeRemove.clear();
	boundLoops.clear();
	findBoundaryLoop(topo, idxOfCollidedTris, boundLoops);
	func.arrangeVector(alledgeRemove);

	// Find all point inside 
	arrayVec3f insidePoints = volMng.pointsInsideObj(obj);

	// Triangulate convex hull
	bound = boundLoops[0];
	// We separate the problem
	arrayVec3f cPoints;
	int nbBoundaryPoints;
	nbBoundaryPoints = bound.size();
	for (int i=0; i<nbBoundaryPoints; i++)
	{
		cPoints.push_back(objPoints->at(bound[i]));
	}

	for (int i=0; i<insidePoints.size(); i++)
	{
		cPoints.push_back(insidePoints[i]);
	}

	arrayVec3i convexFace = findConvexHull(cPoints, nbBoundaryPoints);

	// Convert back to Problem
	addedPoints.clear();
	addedFaces.clear();
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

	objTopoModifer(idxOfCollidedTris, addedFaces, &addedPoints);
}

void eTool::testAdd6()
{
	int nbBoundaryPoints;
	nbBoundaryPoints = bound.size();
	arrayVec3i convexFace = findConvexHull(cPoints, nbBoundaryPoints);

	// Convert back to Problem
	addedPoints.clear();
	addedFaces.clear();
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

	objTopoModifer(idxOfCollidedTris, addedFaces, &addedPoints);
}

bool eTool::isDirectFace( Vec3f pt, int triIdx, arrayVec3f& cPoints, arrayVec3i& cFace )
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

void eTool::debug6(SurfaceObj* obj)
{
	surfObj = obj;
	AABBTreeTri* BVH = (AABBTreeTri*)obj->getBVH();
	objPoints = obj->point();
	objTris = obj->face();
	topo = obj->container();
	arrayVec3f* objPointNorm = obj->pointNormal();
	VectorFunc func;
	preNbPoint = objPoints->size();

	// Voxel
	volMng.init(&points, &tris);

	// intersection with volume
	idxOfCollidedTris.clear();
	volMng.collisionDetectionWithObject(obj, idxOfCollidedTris);
	generatedIdx = idxOfCollidedTris;

	// Boundary loop
	alledgeRemove.clear();
	boundLoops.clear();
	findBoundaryLoop(topo, idxOfCollidedTris, boundLoops);
	func.arrangeVector(alledgeRemove);

	ASSERT(boundLoops.size()==1);
	if (boundLoops.size()<=0)
	{
		return;
	}
	bound = boundLoops[0];
	// Convex boundary loop with flip triangle algorithm
//	convexByRelocate(bound, obj);	

	// Find all point inside 
	arrayVec3f insidePoints = volMng.pointsInsideObj(surfObj);

	// Remove too close points
	GeometricFunc geofunc;
	for (int i=insidePoints.size()-1; i>=0; i--)
	{
		Vec3f curP = insidePoints[i];
		float minDis = 9999;
		for (int j=0; j<idxOfCollidedTris.size(); j++)
		{
			Vec3i _triIdx = (*objTris)[idxOfCollidedTris[j]];
			Vec3f _tri[3], sP;
			_tri[0] = objPoints->at(_triIdx[0]);
			_tri[1] = objPoints->at(_triIdx[1]);
			_tri[2] = objPoints->at(_triIdx[2]);

			if(geofunc.disBtwPointAndTri(curP, _tri, sP) < MIN_DISTANCE_TO_FACE)
			{
				insidePoints.erase(insidePoints.begin()+i);
				break;
			}
		}
	}

	// Triangulate convex hull
	cPoints.clear();
	cFace.clear();
	// We separate the problem
	nbBoundaryPoints = bound.size();
	for (int i=0; i<nbBoundaryPoints; i++)
	{
		cPoints.push_back(objPoints->at(bound[i]));
	}

	for (int i=0; i<insidePoints.size(); i++)
	{
		cPoints.push_back(insidePoints[i]);
	}


	currentPIdx = nbBoundaryPoints;
}

bool eTool::stepDebug6()
{
	if (currentPIdx == nbBoundaryPoints)
	{
		// Initialize first 
		for (int i=0; i<nbBoundaryPoints; i++)
		{
			cFace.push_back(Vec3i(nbBoundaryPoints, i, (i+1)%nbBoundaryPoints));
		}
	}
	else if (currentPIdx < cPoints.size())
	{
		addPointToConvex(cPoints, currentPIdx, cFace);
	}
	else if (currentPIdx == cPoints.size())
	{
		// Convert back to Problem
		addedPoints.clear();
		addedFaces.clear();
		for (int i=0; i<cFace.size(); i++)
		{
			Vec3i orgF = cFace[i];

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

		objTopoModifer(idxOfCollidedTris, addedFaces, &addedPoints);

		return true;
	}

	currentPIdx++;

	return false;
}

void eTool::convexByFlip( arrayInt &_bound, arrayInt &_areTriIdx, SurfaceObj* _obj )
{
	arrayVec3f* _points = _obj->point();
	arrayVec3f* _normP = _obj->pointNormal();
	std::vector<arrayInt> *faceAroundPoints = _obj->container()->facesAroundPoint();

	// Find the most non-Convex
	float minNonConvexAngle_Cos = -1;
	int pIdx = -1;
	int nbPoint = _bound.size();
	for (int i=0; i<nbPoint; i++)
	{
		Vec3f pt[3];
		pt[0] = _points->at(_bound[(i-1+nbPoint)%nbPoint]);
		pt[1] = _points->at(_bound[i]);
		pt[2] = _points->at(_bound[(i+1)%nbPoint]);

		Vec3f _normV = (*_normP)[_bound[i]];

		Vec3f p1 = pt[1] - pt[0];
		Vec3f p2 = pt[2] - pt[1];

		if ( (p1.cross(p2))*_normV < 0 ) // convex
		{
			if (minNonConvexAngle_Cos < COS_ANGLE(-p1, p2))
			{
				pIdx = i;
				minNonConvexAngle_Cos = COS_ANGLE(-p1, p2);
			}
		}
	}

	// Found, flip it
	if (pIdx>-1)
	{
		// Data for change topo
		arrayInt rmTriIdx;
		arrayVec3i addedTri;

		int pIdxInGlobal = _bound[pIdx];
		arrayInt faceAround = faceAroundPoints->at(_bound[pIdxInGlobal]);

		// boundary around the points
		std::vector<arrayInt> localBoundLoop;
		findBoundaryLoop(_obj->container(), faceAround, localBoundLoop);
		ASSERT(localBoundLoop.size()==1);
		arrayInt loop = localBoundLoop[0];
		
		// Divide bound loop into 2
		


		_bound.erase(_bound.begin()+pIdx);
	}
}

void eTool::convexByRelocate( arrayInt _bound, SurfaceObj* _obj )
{
	arrayVec3f* _points = _obj->point();
	arrayVec3f* _normP = _obj->pointNormal();
	std::vector<arrayInt> *faceAroundPoints = _obj->container()->facesAroundPoint();

	while(1)
	{
		// Find the most non-Convex
		float minNonConvexAngle_Cos = -1;
		int pIdx = -1;
		Vec3i triO;
		int nbPoint = _bound.size();
		for (int i=0; i<nbPoint; i++)
		{
			Vec3i tri; 
			tri[0]=_bound[(i-1+nbPoint)%nbPoint];
			tri[1] = _bound[i];
			tri[2] = _bound[(i+1)%nbPoint];

			Vec3f _normV = (*_normP)[_bound[i]];

			Vec3f p1 = _points->at(tri[1]) - _points->at(tri[0]);
			Vec3f p2 = _points->at(tri[2]) - _points->at(tri[1]);

			float test = (p1.cross(p2))*_normV;

			if ( (p1.cross(p2))*_normV<0 && !volMng.isCollidWithTri(_points, tri)) // convex
			{
				Vec3f p1o = p1*-1.0;
				if (minNonConvexAngle_Cos < COS_ANGLE(p1o, p2) )
				{
					pIdx = i;
					minNonConvexAngle_Cos = COS_ANGLE(p1o, p2);
					triO = tri;
				}
			}
		}

		if (pIdx>-1 && minNonConvexAngle_Cos > cos(150*PI/180))//Found
		{
			_points->at(triO[1]) = (_points->at(triO[0])+_points->at(triO[2]))/2.0;
		}
		else
			break;
	}

}

void eTool::cut8( SurfaceObj* obj )
{

}
