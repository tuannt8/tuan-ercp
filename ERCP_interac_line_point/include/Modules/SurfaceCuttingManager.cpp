#include "stdafx.h"
#include "SurfaceCuttingManager.h"

SurfaceCuttingManager::SurfaceCuttingManager(void)
{
	F=fopen("SurfaceCuttingManager.txt","w");
	FIRST_INTERSECTION=true;
	CutEdge=NULL;
	CutSurfContainer=NULL;

	NbUpdated=0;
	Count=0;
}	

SurfaceCuttingManager::~SurfaceCuttingManager(void)
{
	fprintf(F,"NbUpdatedAve: %f\n",(float)NbUpdated/(float)Count);
	fclose(F);
}

void SurfaceCuttingManager::cutting(SurfaceObj* obj, CuttingTool* tool, std::vector<int>* pointsOnCutFront)
{
	VectorFunc func;
	GeometricFunc geoFunc;
	CollisionManager collision;

	// 1. Surface object data
	Obj=obj;
	TopologyContainer* container=Obj->container();
	AABBTree* surfBVH=Obj->getBVH();

	std::vector<Vec3f>* surfNode=Obj->point();
	std::vector<Vec3i>* surfFace=Obj->face();

	// 2. Cut face information
	CutPoint=tool->pointTotal();
	CutSurf=tool->faceTotal();
	CutSurfNormal=tool->normalTotal();

	// 3. Cut face의 topology information
	CutSurfContainer=tool->container();
	CutEdge=CutSurfContainer->edge();

	// 4. Init data
	PreNbPoint=Obj->nbPoint();
	AddedPoint.clear();
	AddedPointNormal.clear();
	AddedFace.clear();

	// 4. Cut point, cut edge normal
	computeCutEdgeNormal();
	computeCutPointNormal();

	// 5. Cut surface data
	std::vector<std::vector<int>>* cutFacesAroundEdge=CutSurfContainer->facesAroundEdge();
	std::vector<std::vector<int>>* cutFacesAroundPoint=CutSurfContainer->facesAroundPoint();

	// 6. Collision information
	std::vector<int> colEdgeIdx;
	std::vector<int> colTriIdxInCutSurf;
	std::vector<int> colTriIdxInSurfObj;

	std::vector<Vec3i> addedFace;
	std::vector<Vec2i> addedEdge;

	// 7. Re-meshing 해야할 삼각형들의 정보를 저장
	std::vector<CutFace> cutFaceInSurfObj;
	std::vector<CutFace> cutFaceInCutFace;
	cutFaceInCutFace.resize(CutSurf->size());
	cutFaceInSurfObj.resize(surfFace->size());
	for(int i=0;i<cutFaceInCutFace.size();i++)
	{
		cutFaceInCutFace[i].faceIdx=-1;
		for(int j=0;j<3;j++)
			cutFaceInCutFace[i].face[j]=-1;
	}
	for(int i=0;i<cutFaceInSurfObj.size();i++)
		cutFaceInSurfObj[i].faceIdx=-1;

	///////////////////////////////////////////////
	//////////////Cutting Functions////////////////
	///////////////////////////////////////////////

	// 8. 먼저 collision된 삼각형들을 찾는다
	collision.collisionBtwTrisAndTrisWithBVH(CutPoint, CutSurf, surfNode, surfFace, surfBVH, colTriIdxInCutSurf, colTriIdxInSurfObj);
	func.arrangeVector(colTriIdxInSurfObj);

	// 9. Surface object의 edge와 cut surface와의 충돌검사를 하고 그 정보를 저장한다.
	intersectionBtwSurfEdgeAndCutSurf(colTriIdxInSurfObj, colTriIdxInCutSurf, cutFaceInSurfObj, cutFaceInCutFace, colEdgeIdx);

	// 10. Cut surface의 edge와 surface object와의 충돌검사를 하고 그 정보를 저장한다.
	intersectionBtwCutEdgeAndSurfObj(colTriIdxInSurfObj, colTriIdxInCutSurf, cutFaceInSurfObj, cutFaceInCutFace);

	// 11. Surface object 내부에 있는 cut surface의 point들을 추가한다. 
	addCutFacesInsideSurfObj(cutFaceInCutFace, pointsOnCutFront);

	// 12. Add points
	TopologyModifier modifier;
	modifier.addPoints(container, AddedPoint);

	// 13. Surface object의 cutFace들을 retriangulation한다.
	retriangulateSurfCutFace(cutFaceInSurfObj, pointsOnCutFront);

	// 14. Cut surface의 cutFace들을 retriangulation한다.
	retriangulatecutSurfCutFace(cutFaceInCutFace);

	// 15. Update topology of the surface
	modifier.removeFaces(container, colTriIdxInSurfObj);
	modifier.removeEdges(container, colEdgeIdx);
	modifier.addFaces(container, AddedFace, PreNbPoint);

	obj->updateNormal();
}

void SurfaceCuttingManager::cutting(SurfaceObj* obj, std::vector<Vec3f>* cutPoint, std::vector<Vec3i>* cutFace, std::vector<Vec3f>* cutFaceNormal, std::vector<int>* pointsOnCutFront)
{
	VectorFunc func;
	GeometricFunc geoFunc;
	CollisionManager collision;

	// 1. Surface object data
	Obj=obj;
	TopologyContainer* container=Obj->container();
	AABBTree* surfBVH=Obj->getBVH();

	std::vector<Vec3f>* surfNode=Obj->point();
	std::vector<Vec3i>* surfFace=Obj->face();

	// 2. Cut face information
	CutPoint=cutPoint;
	CutSurf=cutFace;
	CutSurfNormal=cutFaceNormal;

	// 3. Cut face의 topology information
	CutEdge=new std::vector<Vec2i>;
	CutSurfContainer=new TopologyContainer;
	CutSurfContainer->init(CutPoint, CutPoint, CutSurf, CutEdge);

	// 4. Init data
	PreNbPoint=Obj->nbPoint();
	AddedPoint.clear();
	AddedPointNormal.clear();
	AddedFace.clear();

	// 4. Cut point, cut edge normal
	computeCutEdgeNormal();
	computeCutPointNormal();
	
	// 5. Cut surface data
	std::vector<std::vector<int>>* cutFacesAroundEdge=CutSurfContainer->facesAroundEdge();
	std::vector<std::vector<int>>* cutFacesAroundPoint=CutSurfContainer->facesAroundPoint();

	// 6. Collision information
	std::vector<int> colEdgeIdx;
	std::vector<int> colTriIdxInCutSurf;
	std::vector<int> colTriIdxInSurfObj;

	std::vector<Vec3i> addedFace;
	std::vector<Vec2i> addedEdge;

	// 7. Re-meshing 해야할 삼각형들의 정보를 저장
	std::vector<CutFace> cutFaceInSurfObj;
	std::vector<CutFace> cutFaceInCutFace;
	cutFaceInCutFace.resize(CutSurf->size());
	cutFaceInSurfObj.resize(surfFace->size());
	for(int i=0;i<cutFaceInCutFace.size();i++)
	{
		cutFaceInCutFace[i].faceIdx=-1;
		for(int j=0;j<3;j++)
			cutFaceInCutFace[i].face[j]=-1;
	}
	for(int i=0;i<cutFaceInSurfObj.size();i++)
		cutFaceInSurfObj[i].faceIdx=-1;

	///////////////////////////////////////////////
	//////////////Cutting Functions////////////////
	///////////////////////////////////////////////
	
	// 8. 먼저 collision된 삼각형들을 찾는다
	collision.collisionBtwTrisAndTrisWithBVH(CutPoint, CutSurf, surfNode, surfFace, surfBVH, colTriIdxInCutSurf, colTriIdxInSurfObj);
	func.arrangeVector(colTriIdxInSurfObj);

	// 9. Surface object의 edge와 cut surface와의 충돌검사를 하고 그 정보를 저장한다.
	intersectionBtwSurfEdgeAndCutSurf(colTriIdxInSurfObj, colTriIdxInCutSurf, cutFaceInSurfObj, cutFaceInCutFace, colEdgeIdx);

	// 10. Cut surface의 edge와 surface object와의 충돌검사를 하고 그 정보를 저장한다.
	intersectionBtwCutEdgeAndSurfObj(colTriIdxInSurfObj, colTriIdxInCutSurf, cutFaceInSurfObj, cutFaceInCutFace);

	// 11. Surface object 내부에 있는 cut surface의 point들을 추가한다. 
	addCutFacesInsideSurfObj(cutFaceInCutFace, pointsOnCutFront);

	// 12. Add points
	TopologyModifier modifier;
	modifier.addPoints(container, AddedPoint);

	// 13. Surface object의 cutFace들을 retriangulation한다.
	retriangulateSurfCutFace(cutFaceInSurfObj, pointsOnCutFront);

	// 14. Cut surface의 cutFace들을 retriangulation한다.
	retriangulatecutSurfCutFace(cutFaceInCutFace);

	// 15. Update topology of the surface
	modifier.removeFaces(container, colTriIdxInSurfObj);
	modifier.removeEdges(container, colEdgeIdx);
	modifier.addFaces(container, AddedFace, PreNbPoint);
	
	obj->updateNormal();
}

void SurfaceCuttingManager::computeCutEdgeNormal()
{
	std::vector<std::vector<int>>* cutFacesAroundEdge=CutSurfContainer->facesAroundEdge();
	CutEdgeNormal.resize(CutEdge->size());
	for(int i=0;i<CutEdge->size();i++)
	{
		Vec3f normal;
		for(int j=0;j<(*cutFacesAroundEdge)[i].size();j++)
			normal+=(*CutSurfNormal)[(*cutFacesAroundEdge)[i][j]];
		normal.normalize();
		CutEdgeNormal[i]=normal;
	}
}
void SurfaceCuttingManager::computeCutPointNormal()
{
	std::vector<std::vector<int>>* cutFacesAroundPoint=CutSurfContainer->facesAroundPoint();
	CutPointNormal.resize(CutPoint->size());
	for(int i=0;i<CutPoint->size();i++)
	{
		Vec3f normal;
		for(int j=0;j<(*cutFacesAroundPoint)[i].size();j++)
			normal+=(*CutSurfNormal)[(*cutFacesAroundPoint)[i][j]];
		normal.normalize();
		CutPointNormal[i]=normal;
	}
}

void SurfaceCuttingManager::intersectionBtwSurfEdgeAndCutSurf(std::vector<int>& colTriIdxInSurfObj, std::vector<int>& colTriIdxInCutSurf, std::vector<CutFace>& cutFaceInSurfObj, std::vector<CutFace>& cutFaceInCutFace, std::vector<int>& colEdgeIdx)
{
	TopologyContainer* container=Obj->container();
	std::vector<Vec3f>* surfNode=Obj->point();
	std::vector<Vec2i>* surfEdge=Obj->edge();
	std::vector<std::vector<int>>* edgesInFace=container->edgesInFace();
	std::vector<std::vector<int>>* facesAroundEdge=container->facesAroundEdge();
	VectorFunc func;
	GeometricFunc geoFunc;

	// Collision이 발생할 수 있는 edge들을 찾아서 저장한다.
	std::vector<int> pColEdgeIdx;
	for(int i=0;i<colTriIdxInSurfObj.size();i++)
	{
		int idx=colTriIdxInSurfObj[i];
		for(int j=0;j<3;j++)
			pColEdgeIdx.push_back((*edgesInFace)[idx][j]);
	}
	func.arrangeVector(pColEdgeIdx);

	// surface object의 edge와 cut surface와의 intersection을 검사한다.
	for(int i=0;i<pColEdgeIdx.size();i++)
	{
		int idx=pColEdgeIdx[i];
		for(int j=0;j<colTriIdxInCutSurf.size();j++)
		{
			Vec3f intersection;
			Vec3f tri[3]; 
			tri[0]=(*CutPoint)[(*CutSurf)[colTriIdxInCutSurf[j]][0]];
			tri[1]=(*CutPoint)[(*CutSurf)[colTriIdxInCutSurf[j]][1]];
			tri[2]=(*CutPoint)[(*CutSurf)[colTriIdxInCutSurf[j]][2]];
			if(geoFunc.collisionBtwLinesegAndTri((*surfNode)[(*surfEdge)[idx][0]],(*surfNode)[(*surfEdge)[idx][1]], tri, intersection))
			{
				// Cut face와 intersection 되는 edge와 intersection point
				colEdgeIdx.push_back(idx);
				AddedPoint.push_back(intersection);
				AddedPointNormal.push_back((*CutSurfNormal)[colTriIdxInCutSurf[j]]);
				cutFaceInSurfObj[(*facesAroundEdge)[idx][0]].faceIdx=(*facesAroundEdge)[idx][0];
				cutFaceInSurfObj[(*facesAroundEdge)[idx][1]].faceIdx=(*facesAroundEdge)[idx][1];
				cutFaceInSurfObj[(*facesAroundEdge)[idx][0]].pointsOnEdgeNormalDirec.push_back(surfNode->size()+AddedPoint.size()-1);
				cutFaceInSurfObj[(*facesAroundEdge)[idx][1]].pointsOnEdgeNormalDirec.push_back(surfNode->size()+AddedPoint.size()-1);
				cutFaceInSurfObj[(*facesAroundEdge)[idx][0]].pointsOnEdgeNormal.push_back((*CutSurfNormal)[colTriIdxInCutSurf[j]]);
				cutFaceInSurfObj[(*facesAroundEdge)[idx][1]].pointsOnEdgeNormal.push_back((*CutSurfNormal)[colTriIdxInCutSurf[j]]);
				cutFaceInSurfObj[(*facesAroundEdge)[idx][0]].cutEdgeIdx.push_back(pColEdgeIdx[i]);
				cutFaceInSurfObj[(*facesAroundEdge)[idx][1]].cutEdgeIdx.push_back(pColEdgeIdx[i]);

				// Cut face in cut surface
				cutFaceInCutFace[colTriIdxInCutSurf[j]].faceIdx=colTriIdxInCutSurf[j];
				cutFaceInCutFace[colTriIdxInCutSurf[j]].pointsOnFaceNormalDirec.push_back(surfNode->size()+AddedPoint.size()-1);

				AddedPoint.push_back(intersection);
				AddedPointNormal.push_back(-(*CutSurfNormal)[colTriIdxInCutSurf[j]]);
				cutFaceInSurfObj[(*facesAroundEdge)[idx][0]].pointsOnEdgeCNormalDirec.push_back(surfNode->size()+AddedPoint.size()-1);
				cutFaceInSurfObj[(*facesAroundEdge)[idx][1]].pointsOnEdgeCNormalDirec.push_back(surfNode->size()+AddedPoint.size()-1);
				break;
			}
		}
	}
}

void SurfaceCuttingManager::intersectionBtwCutEdgeAndSurfObj(std::vector<int>& colTriIdxInSurfObj, std::vector<int>& colTriIdxInCutSurf, std::vector<CutFace>& cutFaceInSurfObj, std::vector<CutFace>& cutFaceInCutFace)
{
	VectorFunc func;
	GeometricFunc geoFunc;
	std::vector<int> pColEdgeIdx;

	std::vector<Vec3f>* surfNode=Obj->point();
	std::vector<Vec3i>* surfFace=Obj->face();
	std::vector<std::vector<int>>* edgesInCutFace=CutSurfContainer->edgesInFace();
	std::vector<std::vector<int>>* cutFacesAroundEdge=CutSurfContainer->facesAroundEdge();

	for(int i=0;i<colTriIdxInCutSurf.size();i++)
	{
		int idx=colTriIdxInCutSurf[i];
		for(int j=0;j<3;j++)
			pColEdgeIdx.push_back((*edgesInCutFace)[idx][j]);
	}
	func.arrangeVector(pColEdgeIdx);

	// cut surface의 edge와 surface objec의 face와의 intersection을 검사한다
	for(int i=0;i<pColEdgeIdx.size();i++)
	{
		Vec3f l1=(*CutPoint)[(*CutEdge)[pColEdgeIdx[i]][0]];
		Vec3f l2=(*CutPoint)[(*CutEdge)[pColEdgeIdx[i]][1]];
		for(int k=0;k<colTriIdxInSurfObj.size();k++)
		{
			Vec3f intersection;
			Vec3f tri[3]; 
			tri[0]=(*surfNode)[(*surfFace)[colTriIdxInSurfObj[k]][0]];
			tri[1]=(*surfNode)[(*surfFace)[colTriIdxInSurfObj[k]][1]];
			tri[2]=(*surfNode)[(*surfFace)[colTriIdxInSurfObj[k]][2]];
			if(geoFunc.collisionBtwLinesegAndTri(l1, l2, tri, intersection))
			{
				// Cut face의 edge와 intersection 되는 surface object의 face
				AddedPoint.push_back(intersection);
				AddedPointNormal.push_back(CutEdgeNormal[pColEdgeIdx[i]]);
				cutFaceInSurfObj[colTriIdxInSurfObj[k]].pointsOnFaceNormalDirec.push_back(surfNode->size()+AddedPoint.size()-1);
				cutFaceInSurfObj[colTriIdxInSurfObj[k]].pointsOnFaceNormal.push_back(CutEdgeNormal[pColEdgeIdx[i]]);

				// Cut face in cut surface
				for(int l=0;l<(*cutFacesAroundEdge)[pColEdgeIdx[i]].size();l++)
				{
					cutFaceInCutFace[(*cutFacesAroundEdge)[pColEdgeIdx[i]][l]].faceIdx=(*cutFacesAroundEdge)[pColEdgeIdx[i]][l];
					cutFaceInCutFace[(*cutFacesAroundEdge)[pColEdgeIdx[i]][l]].pointsOnEdgeNormalDirec.push_back(surfNode->size()+AddedPoint.size()-1);
				}

				AddedPoint.push_back(intersection);
				AddedPointNormal.push_back(-CutEdgeNormal[pColEdgeIdx[i]]);
				cutFaceInSurfObj[colTriIdxInSurfObj[k]].pointsOnFaceCNormalDirec.push_back(surfNode->size()+AddedPoint.size()-1);
				break;
			}
		}
	}
}

void SurfaceCuttingManager::addCutFacesInsideSurfObj(std::vector<CutFace>& cutFaceInCutFace, std::vector<int>* pointsOnCutFront)
{
	GeometricFunc geoFunc;
	ReTriangulation reTri;
	std::vector<Vec3f>* surfNode=Obj->point();
	std::vector<Vec3i>* surfFace=Obj->face();
	AABBTree* surfBVH=Obj->getBVH();

	std::vector<int> insidePointIdxLocal;
	std::vector<int> insidePointIdxGlobal;
	for(int i=0;i<CutPoint->size();i++)
	{
		if(geoFunc.isPointInSurf(surfNode, surfFace, surfBVH->root(), (*CutPoint)[i]))
		{
			AddedPoint.push_back((*CutPoint)[i]);
			AddedPointNormal.push_back(CutPointNormal[i]);
			insidePointIdxGlobal.push_back(PreNbPoint+AddedPoint.size()-1);
			AddedPoint.push_back((*CutPoint)[i]);
			AddedPointNormal.push_back(-CutPointNormal[i]);
			insidePointIdxLocal.push_back(i);

			// Cut Front 위에 있는 vertex 정보는 따로 저장한다.
			if(pointsOnCutFront)
			{
				for(int j=0;j<pointsOnCutFront->size();j++)
				{
					if(i==(*pointsOnCutFront)[j])
					{
						VertexIdxOnCutFront.push_back(insidePointIdxGlobal[insidePointIdxGlobal.size()-1]);
						break;
					}
				}
			}
		}
	}

	for(int i=0;i<CutSurf->size();i++)
	{
		int count=0;
		Vec3i f1, f2;
		Vec3f p[3];
		p[0]=(*CutPoint)[(*CutSurf)[i][0]];
		p[1]=(*CutPoint)[(*CutSurf)[i][1]];
		p[2]=(*CutPoint)[(*CutSurf)[i][2]];
		Vec3f n=(*CutSurfNormal)[i];
		for(int j=0;j<3;j++)
		{
			for(int k=0;k<insidePointIdxLocal.size();k++)
			{
				if((*CutSurf)[i][j]==insidePointIdxLocal[k])
				{
					f1[count]=insidePointIdxGlobal[k];
					f2[count]=insidePointIdxGlobal[k]+1;
					count++;

					if(cutFaceInCutFace[i].faceIdx>0)
					{
						cutFaceInCutFace[i].face[j]=insidePointIdxGlobal[k];
					}
					break;
				}
			}
		}
		if(count==3)
		{
			reTri.legalizeTriangle(p, f1, -n);
			reTri.legalizeTriangle(p, f2, n);
			AddedFace.push_back(f1);
			AddedFace.push_back(f2);
		}
	}
}

void SurfaceCuttingManager::retriangulateSurfCutFace(std::vector<CutFace>& cutFaceInSurfObj, std::vector<int>* pointsOnCutFront)
{
	std::vector<Vec3f>* surfNode=Obj->point();
	std::vector<Vec3i>* surfFace=Obj->face();
	VectorFunc func;
	GeometricFunc geoFunc;
	ReTriangulation reTri;

	for(int i=0;i<cutFaceInSurfObj.size();i++)
	{
		if(cutFaceInSurfObj[i].faceIdx>0)
		{
			if(cutFaceInSurfObj[i].pointsOnEdgeNormalDirec.size()==2)
			{
				Vec3i face=(*surfFace)[cutFaceInSurfObj[i].faceIdx];
				std::vector<int> idxNormal;
				std::vector<int> idxCNormal;
				for(int j=0;j<cutFaceInSurfObj[i].pointsOnEdgeNormalDirec.size();j++)
				{
					int idx=cutFaceInSurfObj[i].pointsOnEdgeNormalDirec[j]-PreNbPoint;
					Vec3f p=AddedPoint[idx];
					Vec3f normal=cutFaceInSurfObj[i].pointsOnEdgeNormal[j];

					// 삼각형의 vertex중 normal방향과 반대방향의 vertex를 나눈다.
					for(int k=0;k<3;k++)
					{
						Vec3f l1=(*surfNode)[face[k]];
						Vec3f l2=(*surfNode)[face[(k+1)%3]];
						if(geoFunc.isPointInLine(l1, l2, p))
						{
							if((l1-p)*normal>0)
							{
								idxNormal.push_back(face[k]);
								idxCNormal.push_back(face[(k+1)%3]);
								break;
							}
							else
							{
								idxCNormal.push_back(face[k]);
								idxNormal.push_back(face[(k+1)%3]);
								break;
							}
						}
					}
				}
				std::vector<Vec3i> triangles; 
				func.arrangeVector(idxNormal);
				func.arrangeVector(idxCNormal);
				reTri.delaunayTriangulation(surfNode, face, idxCNormal, cutFaceInSurfObj[i].pointsOnEdgeNormalDirec, cutFaceInSurfObj[i].pointsOnFaceNormalDirec, triangles, PreNbPoint);
				for(int j=0;j<triangles.size();j++)
					AddedFace.push_back(triangles[j]);

				triangles.clear();
				reTri.delaunayTriangulation(surfNode, face, idxNormal, cutFaceInSurfObj[i].pointsOnEdgeCNormalDirec, cutFaceInSurfObj[i].pointsOnFaceCNormalDirec, triangles, PreNbPoint);
				for(int j=0;j<triangles.size();j++)
					AddedFace.push_back(triangles[j]);
			}
			else if(cutFaceInSurfObj[i].pointsOnEdgeNormalDirec.size()==1)
			{
				Vec3i face=(*surfFace)[cutFaceInSurfObj[i].faceIdx];
				std::vector<int> idxNormal;
				std::vector<int> idxCNormal;

				int idx=cutFaceInSurfObj[i].pointsOnEdgeNormalDirec[0]-PreNbPoint;
				Vec3f p=AddedPoint[idx];
				Vec3f normal=cutFaceInSurfObj[i].pointsOnEdgeNormal[0];

				// 삼각형의 vertex중 normal방향과 반대방향의 vertex를 나눈다.
				for(int k=0;k<3;k++)
				{
					Vec3f l1=(*surfNode)[face[k]];
					Vec3f l2=(*surfNode)[face[(k+1)%3]];
					if(geoFunc.isPointInLine(l1, l2, p))
					{
						if((l1-p)*normal>0)
						{
							idxNormal.push_back(face[k]);
							idxCNormal.push_back(face[(k+1)%3]);
							break;
						}
						else
						{
							idxCNormal.push_back(face[k]);
							idxNormal.push_back(face[(k+1)%3]);
							break;
						}
					}
				}

				// Cut front 위에 있는 point를 찾는다.
				for(int j=0;j<cutFaceInSurfObj[i].pointsOnFaceNormalDirec.size();j++)
				{
					idx=cutFaceInSurfObj[i].pointsOnFaceNormalDirec[j]-PreNbPoint;
					p=AddedPoint[idx];
					for(int k=0;k<pointsOnCutFront->size()-1;k++)
					{
						Vec3f l1=(*CutPoint)[(*pointsOnCutFront)[k]];
						Vec3f l2=(*CutPoint)[(*pointsOnCutFront)[k+1]];
						if(geoFunc.isPointInLine(l1, l2, p))
						{
							VertexIdxOnCutFront.push_back(cutFaceInSurfObj[i].pointsOnFaceNormalDirec[j]);
							VertexIdxOnCutFrontSurf.push_back(cutFaceInSurfObj[i].pointsOnFaceNormalDirec[j]);
							NormalOnCutFront.push_back(cutFaceInSurfObj[i].pointsOnFaceNormal[j]);
							NormalOnCutFrontSurf.push_back(cutFaceInSurfObj[i].pointsOnFaceNormal[j]);
							break;
						}
					}
				}

				std::vector<Vec3i> triangles; 
				reTri.delaunayTriangulation(surfNode, face, idxCNormal, cutFaceInSurfObj[i].pointsOnEdgeNormalDirec, cutFaceInSurfObj[i].pointsOnFaceNormalDirec, triangles, PreNbPoint);
				for(int j=0;j<triangles.size();j++)
					AddedFace.push_back(triangles[j]);

				triangles.clear();
				reTri.delaunayTriangulation(surfNode, face, idxNormal, cutFaceInSurfObj[i].pointsOnEdgeCNormalDirec, cutFaceInSurfObj[i].pointsOnFaceCNormalDirec, triangles, PreNbPoint);
				for(int j=0;j<triangles.size();j++)
					AddedFace.push_back(triangles[j]);
			}
		}
	}
}

void SurfaceCuttingManager::retriangulatecutSurfCutFace(std::vector<CutFace>& cutFaceInCutFace)
{
	ReTriangulation reTri;
	std::vector<Vec3f>* surfNode=Obj->point();

	for(int i=0;i<cutFaceInCutFace.size();i++)
	{
		int faceIdx=cutFaceInCutFace[i].faceIdx;
		if(faceIdx>0)
		{
			Vec3f _tri[3];
			_tri[0]=(*CutPoint)[(*CutSurf)[faceIdx][0]];
			_tri[1]=(*CutPoint)[(*CutSurf)[faceIdx][1]];
			_tri[2]=(*CutPoint)[(*CutSurf)[faceIdx][2]];
			Vec3f normal=-(*CutSurfNormal)[faceIdx];

			std::vector<Vec3i> triangles; 
			reTri.delaunayTriangulation(surfNode, cutFaceInCutFace[i].face, _tri, cutFaceInCutFace[i].pointsOnEdgeNormalDirec, cutFaceInCutFace[i].pointsOnFaceNormalDirec, triangles, PreNbPoint);
			for(int j=0;j<triangles.size();j++)
			{
				reTri.legalizeTriangle(_tri, triangles[j], normal);
				AddedFace.push_back(triangles[j]);
				AddedFace.push_back(Vec3i(triangles[j][0]+1,triangles[j][2]+1,triangles[j][1]+1));
			}
		}
	}
}

void SurfaceCuttingManager::cuttingProg(SurfaceObj* obj, CuttingTool* tool, int toolPathIdx)
{
	if(toolPathIdx==0)
	{
		Obj=obj;
		Tool=tool;
		ToolPathIdx=toolPathIdx;

		// 1. Surface object의 data
		SurfContainer=obj->container();
		SurfBVH=obj->getBVH();

		SurfNode=obj->point();
		SurfFace=obj->face();
		SurfEdge=obj->edge();
		SurfNormal=obj->faceNormal();
		return;
	}
	ToolPathIdx=toolPathIdx;

	// Cut face information
	CutPoint=Tool->cutPoint();
	CutSurf=Tool->cutFace();
	CutSurfNormal=Tool->cutFaceNormal();

	// Cut face의 topology information
	if(CutEdge)
		delete CutEdge;
	CutEdge=new std::vector<Vec2i>;

	if(CutSurfContainer)
		delete CutSurfContainer;
	CutSurfContainer=new TopologyContainer;
	CutSurfContainer->init(CutPoint, CutPoint, CutSurf, CutEdge);

	PreNbPoint=SurfNode->size();
	AddedPoint.clear();
	AddedPointNormal.clear();
	AddedFace.clear();

	// Cut point, cut edge normal
	computeCutEdgeNormal();
	computeCutPointNormal();

	VectorFunc func;
	GeometricFunc geoFunc;
	CollisionManager collision;

	int nbCutFront=Tool->nbCutFront();
	std::vector<int>* pointsOnCutFront=new std::vector<int>;
	for(int i=0;i<nbCutFront;i++)
		pointsOnCutFront->push_back(i);

	CutFront.resize(pointsOnCutFront->size());
	CutFrontGlobal.resize(pointsOnCutFront->size());
	CutEnd.resize(pointsOnCutFront->size());
	CutEndGlobal.resize(pointsOnCutFront->size());

	std::vector<std::vector<int>>* edgesInFace=SurfContainer->edgesInFace();

	// Cut surface data
	std::vector<std::vector<int>>* cutFacesAroundEdge=CutSurfContainer->facesAroundEdge();
	std::vector<std::vector<int>>* cutFacesAroundPoint=CutSurfContainer->facesAroundPoint();

	std::vector<int> colEdgeIdx;
	std::vector<int> colTriIdxInCutSurf;
	std::vector<int> colTriIdxInSurfObj;

	// Re-meshing 해야할 삼각형들의 정보를 저장
	std::vector<CutFace> cutFaceInSurfObj;
	std::vector<CutFace> cutFaceInCutFace;
	cutFaceInCutFace.resize(CutSurf->size());
	cutFaceInSurfObj.resize(SurfFace->size());
	for(int i=0;i<cutFaceInCutFace.size();i++)
	{
		cutFaceInCutFace[i].faceIdx=-1;
		for(int j=0;j<3;j++)
			cutFaceInCutFace[i].face[j]=-1;
	}
	for(int i=0;i<cutFaceInSurfObj.size();i++)
	{
		cutFaceInSurfObj[i].faceIdx=-1;
		cutFaceInSurfObj[i].vtxIdxOnCutFrontCNormal=-1;
		cutFaceInSurfObj[i].vtxIdxOnCutFrontNormal=-1;
	}

	// cut face가 첫번째로 object와 intersection하는 것이면
	if(FIRST_INTERSECTION)
	{
		collision.collisionBtwTrisAndTrisWithBVH(CutPoint, CutSurf, SurfNode, SurfFace, SurfBVH, colTriIdxInCutSurf, colTriIdxInSurfObj);
		func.arrangeVector(colTriIdxInSurfObj);

		// Surface object의 edge와 cut surface와의 충돌검사를 하고 그 정보를 저장한다.
		intersectionBtwSurfEdgeAndCutSurf(colTriIdxInSurfObj, colTriIdxInCutSurf, cutFaceInSurfObj, cutFaceInCutFace, colEdgeIdx);

		// Cut surface의 edge와 surface object와의 충돌검사를 하고 그 정보를 저장한다.
		intersectionBtwCutEdgeAndSurfObj(colTriIdxInSurfObj, colTriIdxInCutSurf, cutFaceInSurfObj, cutFaceInCutFace);

		// surface object 내부에 있는 cut surface의 point들을 추가한다. 
		addCutFacesInsideSurfObjProg(cutFaceInCutFace, pointsOnCutFront);

		// add points
		TopologyModifier modifier;
		modifier.addPoints(SurfContainer, AddedPoint);

		// surface object의 cutFace들을 retriangulation한다.
		retriangulateSurfCutFace(cutFaceInSurfObj, pointsOnCutFront);

		// cut surface의 cutFace들을 retriangulation한다.
		retriangulatecutSurfCutFace(cutFaceInCutFace);

		// Update topology of the surface
		modifier.removeFaces(SurfContainer, colTriIdxInSurfObj);
		modifier.removeEdges(SurfContainer, colEdgeIdx);
		modifier.addFaces(SurfContainer, AddedFace, PreNbPoint);

		if(AddedFace.size()>0)
		{
			NbUpdated+=(AddedFace.size()+colTriIdxInSurfObj.size());
			Count++;
			fprintf(F,"NbUpdated: %d\n",(AddedFace.size()+colTriIdxInSurfObj.size()));
		}
		Obj->updateNormal();

		if(AddedPoint.size()>0)
			FIRST_INTERSECTION=false;
	}

	// 이전 step에서 이미 intersection 했었으면
	else
	{
		// 이전 step에서 cut front위의 점 주변의 삼각형과 cut surface와의 충돌을 검사하고 관련 정보를 저장한다
		intersectionBtwPartialCutFaceAndCutSurf(colTriIdxInSurfObj, colTriIdxInCutSurf, cutFaceInSurfObj, cutFaceInCutFace, colEdgeIdx);

		// Collision된 edge 주변의 삼각형들을 recursive하게 검사
		intersectionBtwSurfEdgeAndCutSurfProg(colTriIdxInSurfObj, colTriIdxInCutSurf, cutFaceInSurfObj, cutFaceInCutFace, colEdgeIdx);

		// Cut surface의 edge와 surface object와의 충돌검사를 하고 그 정보를 저장한다.
		intersectionBtwCutEdgeAndSurfObjProg(colTriIdxInSurfObj, colTriIdxInCutSurf, cutFaceInSurfObj, cutFaceInCutFace);

		// surface object 내부에 있는 cut surface의 point들을 추가한다. 
		addCutFacesInsideSurfObjProg(cutFaceInCutFace, pointsOnCutFront);

		// add points
		TopologyModifier modifier;
		modifier.addPoints(SurfContainer, AddedPoint);
		
		if(AddedPoint.empty())
			FIRST_INTERSECTION=true;

		// delete information in previous step
		VertexIdxOnCutFront.clear();
		VertexIdxOnCutFrontSurf.clear();
		NormalOnCutFront.clear();
		NormalOnCutFrontSurf.clear();

		// surface object의 cutFace들을 retriangulation한다.
		retriangulateSurfCutFaceProg(cutFaceInSurfObj, pointsOnCutFront, colEdgeIdx);

		// cut surface의 cutFace들을 retriangulation한다.
		retriangulatecutSurfCutFace(cutFaceInCutFace);

		func.arrangeVector(colTriIdxInSurfObj);
		func.arrangeVector(colEdgeIdx);

		// Update topology of the surface
		modifier.removeFaces(SurfContainer, colTriIdxInSurfObj);
		modifier.removeEdges(SurfContainer, colEdgeIdx);
		modifier.addFaces(SurfContainer, AddedFace, PreNbPoint);

		if(AddedFace.size()>0)
		{
			NbUpdated+=(AddedFace.size()+colTriIdxInSurfObj.size());
			Count++;
			fprintf(F,"NbUpdated: %d\n",(AddedFace.size()+colTriIdxInSurfObj.size()));
		}

		Obj->updateNormal();
	}
	fprintf(F,"%d\n",SurfNode->size());
	delete pointsOnCutFront;
}

void SurfaceCuttingManager::addCutFacesInsideSurfObjProg(std::vector<CutFace>& cutFaceInCutFace, std::vector<int>* pointsOnCutFront)
{
	GeometricFunc geoFunc;
	ReTriangulation reTri;

	CutFront.clear(); CutFront.resize(pointsOnCutFront->size());
	CutFrontGlobal.clear(); CutFrontGlobal.resize(pointsOnCutFront->size());

	for(int i=0;i<CutFront.size();i++)
	{
		CutFront[i]=-1;
		CutFrontGlobal[i]=-1;
	}

	for(int i=0;i<pointsOnCutFront->size();i++)
	{
		int pointIdx=(*pointsOnCutFront)[i];
		if(geoFunc.isPointInSurf(SurfNode, SurfFace, SurfBVH->root(), (*CutPoint)[pointIdx]))
		{
			AddedPoint.push_back((*CutPoint)[pointIdx]);
			AddedPointNormal.push_back(CutPointNormal[i]);

			CutFront[i]=(*pointsOnCutFront)[i];
			CutFrontGlobal[i]=PreNbPoint+AddedPoint.size()-1;

			AddedPoint.push_back((*CutPoint)[pointIdx]);
			AddedPointNormal.push_back(-CutPointNormal[i]);
		}
	}

	for(int i=0;i<CutSurf->size();i++)
	{
		int count=0;
		Vec3i f1, f2;
		Vec3f p[3];
		p[0]=(*CutPoint)[(*CutSurf)[i][0]];
		p[1]=(*CutPoint)[(*CutSurf)[i][1]];
		p[2]=(*CutPoint)[(*CutSurf)[i][2]];
		Vec3f n=(*CutSurfNormal)[i];
		for(int j=0;j<3;j++)
		{
			for(int k=0;k<CutFront.size();k++)
			{
				if((*CutSurf)[i][j]==CutFront[k])
				{
					f1[count]=CutFrontGlobal[k];
					f2[count]=CutFrontGlobal[k]+1;
					count++;

					if(cutFaceInCutFace[i].faceIdx>0)
						cutFaceInCutFace[i].face[j]=CutFrontGlobal[k];
					break;
				}

				if((*CutSurf)[i][j]==CutEnd[k])
				{
					f1[count]=CutEndGlobal[k];
					f2[count]=CutEndGlobal[k]+1;
					count++;

					if(cutFaceInCutFace[i].faceIdx>0)
						cutFaceInCutFace[i].face[j]=CutEndGlobal[k];
					break;
				}
			}
		}
		if(count==3)
		{
			reTri.legalizeTriangle(p, f1, -n);
			reTri.legalizeTriangle(p, f2, n);
			AddedFace.push_back(f1);
			AddedFace.push_back(f2);
		}
	}

	CutEnd.clear(); CutEnd.resize(pointsOnCutFront->size());
	CutEndGlobal.clear(); CutEndGlobal.resize(pointsOnCutFront->size());

	for(int i=0;i<CutEnd.size();i++)
	{
		if(CutFront[i]>0)
		{
			CutEnd[i]=CutFront[i]+CutFront.size();
			CutEndGlobal[i]=CutFrontGlobal[i];
		}
		else
		{
			CutEnd[i]=-1;
			CutEndGlobal[i]=-1;
		}
	}
}

void SurfaceCuttingManager::intersectionBtwCutEdgeAndSurfObjProg(std::vector<int>& colTriIdxInSurfObj, std::vector<int>& colTriIdxInCutSurf, std::vector<CutFace>& cutFaceInSurfObj, std::vector<CutFace>& cutFaceInCutFace)
{
	VectorFunc vecFunc;
	GeometricFunc geoFunc;

	// Cut surface의 data
	std::vector<std::vector<int>>* facesAroundEdge=CutSurfContainer->facesAroundEdge();

	for(int i=0;i<colTriIdxInSurfObj.size();i++)
	{
		int triIdx=colTriIdxInSurfObj[i];
		Vec3f tri[3]; tri[0]=(*SurfNode)[(*SurfFace)[colTriIdxInSurfObj[i]][0]]; tri[1]=(*SurfNode)[(*SurfFace)[colTriIdxInSurfObj[i]][1]]; tri[2]=(*SurfNode)[(*SurfFace)[colTriIdxInSurfObj[i]][2]];

		for(int j=0;j<CutEdge->size();j++)
		{
			int edgeIdx=j;
			Vec3f l1=(*CutPoint)[(*CutEdge)[edgeIdx][0]];
			Vec3f l2=(*CutPoint)[(*CutEdge)[edgeIdx][1]];
			Vec3f inter;
			if(geoFunc.collisionBtwLinesegAndTri(l1, l2, tri, inter))
			{
				AddedPoint.push_back(inter);
				AddedPointNormal.push_back(CutEdgeNormal[edgeIdx]);
				cutFaceInSurfObj[triIdx].faceIdx=triIdx;
				cutFaceInSurfObj[triIdx].pointsOnFaceNormalDirec.push_back(PreNbPoint+AddedPoint.size()-1);
				cutFaceInSurfObj[triIdx].pointsOnFaceNormal.push_back(CutEdgeNormal[edgeIdx]);

				for(int k=0;k<(*facesAroundEdge)[edgeIdx].size();k++)
				{
					int faceIdx=(*facesAroundEdge)[edgeIdx][k];
					cutFaceInCutFace[faceIdx].faceIdx=faceIdx;
					cutFaceInCutFace[faceIdx].pointsOnEdgeNormalDirec.push_back(PreNbPoint+AddedPoint.size()-1);
				}

				AddedPoint.push_back(inter);
				AddedPointNormal.push_back(CutEdgeNormal[edgeIdx]);
				cutFaceInSurfObj[triIdx].pointsOnFaceCNormalDirec.push_back(PreNbPoint+AddedPoint.size()-1);
			}
		}
	}
}

void SurfaceCuttingManager::intersectionBtwSurfEdgeAndCutSurfProg(std::vector<int>& colTriIdxInSurfObj, std::vector<int>& colTriIdxInCutSurf, std::vector<CutFace>& cutFaceInSurfObj, std::vector<CutFace>& cutFaceInCutFace, std::vector<int>& colEdgeIdx)
{
	VectorFunc vecFunc;
	GeometricFunc geoFunc;

	// Surface object의 topology information
	std::vector<std::vector<int>>* facesAroundPoint=SurfContainer->facesAroundPoint();
	std::vector<std::vector<int>>* facesAroundEdge=SurfContainer->facesAroundEdge();
	std::vector<std::vector<int>>* edgesInFace=SurfContainer->edgesInFace();

	for(int i=0;i<colEdgeIdx.size();i++)
	{
		int triIdx;
		for(int j=0;j<(*facesAroundEdge)[colEdgeIdx[i]].size();j++)
		{
			triIdx=(*facesAroundEdge)[colEdgeIdx[i]][j];
			bool flag=true;
			for(int k=0;k<colTriIdxInSurfObj.size();k++)
			{
				if(triIdx==colTriIdxInSurfObj[k])
				{
					flag=false;
					break;
				}
			}
			if(flag)
			{
				colTriIdxInSurfObj.push_back(triIdx);
				break;
			}
		}

		// 삼각형 내부의 edge와의 collision 검사
		for(int j=0;j<(*edgesInFace)[triIdx].size();j++)
		{
			int edgeIdx=(*edgesInFace)[triIdx][j];

			// 이미 충돌된 edge인지 검사
			bool flag=true;
			for(int k=0;k<colEdgeIdx.size();k++)
			{
				if(edgeIdx==colEdgeIdx[k])
				{
					flag=false;
					break;
				}
			}

			// 이미 충돌되지 않은 edge이면 충돌검사
			if(flag)
			{
				Vec3f l1=(*SurfNode)[(*SurfEdge)[edgeIdx][0]];
				Vec3f l2=(*SurfNode)[(*SurfEdge)[edgeIdx][1]];

				int triIdx1=-1;
				int triIdx2=-1;

				if((*facesAroundEdge)[edgeIdx].size()==1)
				{
					triIdx1=(*facesAroundEdge)[edgeIdx][0];
				}
				if((*facesAroundEdge)[edgeIdx].size()==2)
				{
					triIdx1=(*facesAroundEdge)[edgeIdx][0];
					triIdx2=(*facesAroundEdge)[edgeIdx][1];
				}

				for(int k=0;k<CutSurf->size();k++)
				{
					Vec3f tri[3]; tri[0]=(*CutPoint)[(*CutSurf)[k][0]]; tri[1]=(*CutPoint)[(*CutSurf)[k][1]]; tri[2]=(*CutPoint)[(*CutSurf)[k][2]];
					Vec3f inter;
					if(geoFunc.collisionBtwLinesegAndTri(l1, l2, tri, inter))
					{
						// collision triangle
						colTriIdxInCutSurf.push_back(k);
						colEdgeIdx.push_back(edgeIdx);

						// collision triangle information
						AddedPoint.push_back(inter);
						AddedPointNormal.push_back((*CutSurfNormal)[k]);
						cutFaceInSurfObj[triIdx1].faceIdx=triIdx1;
						cutFaceInSurfObj[triIdx1].pointsOnEdgeNormalDirec.push_back(PreNbPoint+AddedPoint.size()-1);
						cutFaceInSurfObj[triIdx1].pointsOnEdgeNormal.push_back((*CutSurfNormal)[k]);
						cutFaceInSurfObj[triIdx1].cutEdgeIdx.push_back(edgeIdx);

						if(triIdx2>0)
						{
							cutFaceInSurfObj[triIdx2].faceIdx=triIdx2;
							cutFaceInSurfObj[triIdx2].pointsOnEdgeNormalDirec.push_back(PreNbPoint+AddedPoint.size()-1);
							cutFaceInSurfObj[triIdx2].pointsOnEdgeNormal.push_back((*CutSurfNormal)[k]);
							cutFaceInSurfObj[triIdx2].cutEdgeIdx.push_back(edgeIdx);
						}

						cutFaceInCutFace[k].faceIdx=k;
						cutFaceInCutFace[k].pointsOnFaceNormalDirec.push_back(PreNbPoint+AddedPoint.size()-1);

						AddedPoint.push_back(inter);
						AddedPointNormal.push_back(-(*CutSurfNormal)[k]);
						cutFaceInSurfObj[triIdx1].pointsOnEdgeCNormalDirec.push_back(PreNbPoint+AddedPoint.size()-1);
						if(triIdx2>0)
							cutFaceInSurfObj[triIdx2].pointsOnEdgeCNormalDirec.push_back(PreNbPoint+AddedPoint.size()-1);
					}
				}
			}
		}
	}
}

void SurfaceCuttingManager::intersectionBtwPartialCutFaceAndCutSurf(std::vector<int>& colTriIdxInSurfObj, std::vector<int>& colTriIdxInCutSurf, std::vector<CutFace>& cutFaceInSurfObj, std::vector<CutFace>& cutFaceInCutFace, std::vector<int>& colEdgeIdx)
{
	VectorFunc vecFunc;
	GeometricFunc geoFunc;

	// Surface object의 topology information
	std::vector<std::vector<int>>* facesAroundPoint=SurfContainer->facesAroundPoint();
	std::vector<std::vector<int>>* facesAroundEdge=SurfContainer->facesAroundEdge();
	std::vector<std::vector<int>>* edgesInFace=SurfContainer->edgesInFace();

	// 이전 step에서 cut front위에 있던 vertex 주위의 삼각형을 검사한다.
	for(int i=0;i<VertexIdxOnCutFrontSurf.size();i++)
	{
		int idx=VertexIdxOnCutFrontSurf[i];
		std::vector<int> pEdgeIdx;
		for(int j=0;j<(*facesAroundPoint)[idx].size();j++)
		{
			int faceIdx=(*facesAroundPoint)[idx][j];
			for(int k=0;k<(*edgesInFace)[faceIdx].size();k++)
			{
				int edgeIdx=(*edgesInFace)[faceIdx][k];
				pEdgeIdx.push_back(edgeIdx);
			}
		}

		idx=VertexIdxOnCutFrontSurf[i]+1;
		for(int j=0;j<(*facesAroundPoint)[idx].size();j++)
		{
			int faceIdx=(*facesAroundPoint)[idx][j];
			for(int k=0;k<(*edgesInFace)[faceIdx].size();k++)
			{
				int edgeIdx=(*edgesInFace)[faceIdx][k];
				pEdgeIdx.push_back(edgeIdx);
			}
		}
		vecFunc.arrangeVector(pEdgeIdx);

		for(int j=0;j<pEdgeIdx.size();j++)
		{
			if(((*SurfEdge)[pEdgeIdx[j]][0]==VertexIdxOnCutFrontSurf[i])||((*SurfEdge)[pEdgeIdx[j]][1]==VertexIdxOnCutFrontSurf[i]))
			{
				vecFunc.removeElement(pEdgeIdx, j);
				j--;
			}
		}

		// Edge와 intersection test
		for(int j=0;j<pEdgeIdx.size();j++)
		{
			Vec3f l1=(*SurfNode)[(*SurfEdge)[pEdgeIdx[j]][0]];
			Vec3f l2=(*SurfNode)[(*SurfEdge)[pEdgeIdx[j]][1]];
			for(int k=0;k<CutSurf->size();k++)
			{
				Vec3f tri[3]; tri[0]=(*CutPoint)[(*CutSurf)[k][0]]; tri[1]=(*CutPoint)[(*CutSurf)[k][1]]; tri[2]=(*CutPoint)[(*CutSurf)[k][2]];
				Vec3f inter;
				if(geoFunc.collisionBtwLinesegAndTri(l1, l2, tri, inter))
				{
					// collision triangle
					colTriIdxInCutSurf.push_back(k);
					colEdgeIdx.push_back(pEdgeIdx[j]);

					AddedPoint.push_back(inter);
					AddedPointNormal.push_back((*CutSurfNormal)[k]);
					AddedPoint.push_back(inter);
					AddedPointNormal.push_back(-(*CutSurfNormal)[k]);

					cutFaceInCutFace[k].faceIdx=k;
					cutFaceInCutFace[k].pointsOnFaceNormalDirec.push_back(PreNbPoint+AddedPoint.size()-2);

					for(int l=0;l<(*facesAroundEdge)[pEdgeIdx[j]].size();l++)
					{
						int faceIdx=(*facesAroundEdge)[pEdgeIdx[j]][l];

						// collision triangle information
						cutFaceInSurfObj[faceIdx].faceIdx=faceIdx;
						cutFaceInSurfObj[faceIdx].pointsOnEdgeNormalDirec.push_back(PreNbPoint+AddedPoint.size()-2);
						cutFaceInSurfObj[faceIdx].cutEdgeIdx.push_back(pEdgeIdx[j]);
						cutFaceInSurfObj[faceIdx].pointsOnEdgeNormal.push_back((*CutSurfNormal)[k]);
						cutFaceInSurfObj[faceIdx].pointsOnEdgeCNormalDirec.push_back(PreNbPoint+AddedPoint.size()-1);

						for(int m=0;m<3;m++)
						{
							if((*SurfFace)[faceIdx][m]==VertexIdxOnCutFrontSurf[i])
							{
								cutFaceInSurfObj[faceIdx].vtxIdxOnCutFrontNormal=VertexIdxOnCutFrontSurf[i];
								colTriIdxInSurfObj.push_back(faceIdx);
								break;
							}
							if((*SurfFace)[faceIdx][m]==(VertexIdxOnCutFrontSurf[i]+1))
							{
								cutFaceInSurfObj[faceIdx].vtxIdxOnCutFrontCNormal=VertexIdxOnCutFrontSurf[i]+1;
								colTriIdxInSurfObj.push_back(faceIdx);
								break;
							}
						}
					}
				}
			}
		}
	}

	std::vector<std::vector<int>>* cutFacesAroundEdge=CutSurfContainer->facesAroundEdge();

	// VertexOnCutFront를 포함하는 edge
	for(int i=0;i<VertexIdxOnCutFrontSurf.size();i++)
	{
		Vec3f p=(*SurfNode)[VertexIdxOnCutFrontSurf[i]];
		for(int j=0;j<CutEdge->size();j++)
		{
			Vec3f l1=(*CutPoint)[(*CutEdge)[j][0]];
			Vec3f l2=(*CutPoint)[(*CutEdge)[j][1]];
			if(geoFunc.isPointInLine(l1, l2, p))
			{
				int faceIdx=(*cutFacesAroundEdge)[j][0];
				cutFaceInCutFace[faceIdx].faceIdx=faceIdx;
				cutFaceInCutFace[faceIdx].pointsOnEdgeNormalDirec.push_back(VertexIdxOnCutFrontSurf[i]);
			}

		}
	}
}

void SurfaceCuttingManager::retriangulateSurfCutFaceProg(std::vector<CutFace>& cutFaceInSurfObj, std::vector<int>* pointsOnCutFront, std::vector<int>& colEdgeIdx)
{
	std::vector<std::vector<int>>* edgesInFace=SurfContainer->edgesInFace();
	VectorFunc func;
	GeometricFunc geoFunc;
	ReTriangulation reTri;

	for(int i=0;i<cutFaceInSurfObj.size();i++)
	{
		if(cutFaceInSurfObj[i].faceIdx>0)
		{
			// 이전 cut front 위에 있는 삼각형들
			if((cutFaceInSurfObj[i].vtxIdxOnCutFrontNormal>0)||(cutFaceInSurfObj[i].vtxIdxOnCutFrontCNormal>0))
			{
				CutFace temp=cutFaceInSurfObj[i];
				Vec3i face=(*SurfFace)[cutFaceInSurfObj[i].faceIdx];
				if(cutFaceInSurfObj[i].vtxIdxOnCutFrontNormal>0)
				{
					std::vector<int> idxNormal;
					std::vector<int> idxCNormal;

					int idx=cutFaceInSurfObj[i].pointsOnEdgeNormalDirec[0]-PreNbPoint;
					Vec3f p=AddedPoint[idx];
					Vec3f normal=cutFaceInSurfObj[i].pointsOnEdgeNormal[0];

					// 삼각형의 vertex중 normal방향과 반대방향의 vertex를 나눈다.
					int edgeIdx=cutFaceInSurfObj[i].cutEdgeIdx[0];
					Vec3f l1=(*SurfNode)[(*SurfEdge)[edgeIdx][0]];
					Vec3f l2=(*SurfNode)[(*SurfEdge)[edgeIdx][1]];
					Vec3f v=l1-p; v.normalize();
					if(v*normal>0)
					{
						idxNormal.push_back((*SurfEdge)[edgeIdx][0]);
						idxCNormal.push_back((*SurfEdge)[edgeIdx][1]);
					}
					else
					{
						idxCNormal.push_back((*SurfEdge)[edgeIdx][0]);
						idxNormal.push_back((*SurfEdge)[edgeIdx][1]);
					}

					std::vector<Vec3i> triangles; 
					reTri.delaunayTriangulation(SurfNode, face, idxCNormal, cutFaceInSurfObj[i].pointsOnEdgeNormalDirec, cutFaceInSurfObj[i].pointsOnFaceNormalDirec, triangles, PreNbPoint);
					for(int j=0;j<triangles.size();j++)
						AddedFace.push_back(triangles[j]);

					triangles.clear();
					for(int k=0;k<3;k++)
					{
						if(face[k]==cutFaceInSurfObj[i].vtxIdxOnCutFrontNormal)
						{
							// remove될 edge idx를 찾는다
							Vec2i edge(face[k], idxCNormal[0]);
							for(int l=0;l<3;l++)
							{
								Vec2i _edge=(*SurfEdge)[(*edgesInFace)[i][l]];
								if(geoFunc.isEdgeSame(edge, _edge))
								{
									colEdgeIdx.push_back((*edgesInFace)[i][l]);
									break;
								}
							}
							face[k]=cutFaceInSurfObj[i].vtxIdxOnCutFrontNormal+1;
							break;
						}
					}
					reTri.delaunayTriangulation(SurfNode, face, idxNormal, cutFaceInSurfObj[i].pointsOnEdgeCNormalDirec, cutFaceInSurfObj[i].pointsOnFaceCNormalDirec, triangles, PreNbPoint);
					for(int j=0;j<triangles.size();j++)
						AddedFace.push_back(triangles[j]);
				}
				else
				{
					std::vector<int> idxNormal;
					std::vector<int> idxCNormal;

					int idx=cutFaceInSurfObj[i].pointsOnEdgeNormalDirec[0]-PreNbPoint;
					Vec3f p=AddedPoint[idx];
					Vec3f normal=cutFaceInSurfObj[i].pointsOnEdgeNormal[0];

					// 삼각형의 vertex중 normal방향과 반대방향의 vertex를 나눈다.
					int edgeIdx=cutFaceInSurfObj[i].cutEdgeIdx[0];
					Vec3f l1=(*SurfNode)[(*SurfEdge)[edgeIdx][0]];
					Vec3f l2=(*SurfNode)[(*SurfEdge)[edgeIdx][1]];
					Vec3f v=l1-p; v.normalize();
					if(v*normal>0)
					{
						idxNormal.push_back((*SurfEdge)[edgeIdx][0]);
						idxCNormal.push_back((*SurfEdge)[edgeIdx][1]);
					}
					else
					{
						idxCNormal.push_back((*SurfEdge)[edgeIdx][0]);
						idxNormal.push_back((*SurfEdge)[edgeIdx][1]);
					}

					std::vector<Vec3i> triangles; 
					reTri.delaunayTriangulation(SurfNode, face, idxNormal, cutFaceInSurfObj[i].pointsOnEdgeCNormalDirec, cutFaceInSurfObj[i].pointsOnFaceCNormalDirec, triangles, PreNbPoint);
					for(int j=0;j<triangles.size();j++)
						AddedFace.push_back(triangles[j]);

					triangles.clear();
					for(int k=0;k<3;k++)
					{
						if(face[k]==cutFaceInSurfObj[i].vtxIdxOnCutFrontCNormal)
						{
							// remove될 edge idx를 찾는다
							Vec2i edge(face[k], idxNormal[0]);
							for(int l=0;l<3;l++)
							{
								Vec2i _edge=(*SurfEdge)[(*edgesInFace)[i][l]];
								if(geoFunc.isEdgeSame(edge, _edge))
								{
									colEdgeIdx.push_back((*edgesInFace)[i][l]);
									break;
								}
							}
							face[k]=cutFaceInSurfObj[i].vtxIdxOnCutFrontCNormal-1;
							break;
						}
					}
					reTri.delaunayTriangulation(SurfNode, face, idxCNormal, cutFaceInSurfObj[i].pointsOnEdgeNormalDirec, cutFaceInSurfObj[i].pointsOnFaceNormalDirec, triangles, PreNbPoint);
					for(int j=0;j<triangles.size();j++)
						AddedFace.push_back(triangles[j]);
				}
			}
			else if(cutFaceInSurfObj[i].pointsOnEdgeNormalDirec.size()==2)
			{
				Vec3i face=(*SurfFace)[cutFaceInSurfObj[i].faceIdx];
				std::vector<int> idxNormal;
				std::vector<int> idxCNormal;
				for(int j=0;j<cutFaceInSurfObj[i].pointsOnEdgeNormalDirec.size();j++)
				{
					int idx=cutFaceInSurfObj[i].pointsOnEdgeNormalDirec[j]-PreNbPoint;
					Vec3f p=AddedPoint[idx];
					Vec3f normal=cutFaceInSurfObj[i].pointsOnEdgeNormal[j];

					// 삼각형의 vertex중 normal방향과 반대방향의 vertex를 나눈다.
					for(int k=0;k<3;k++)
					{
						Vec3f l1=(*SurfNode)[face[k]];
						Vec3f l2=(*SurfNode)[face[(k+1)%3]];
						if(geoFunc.isPointInLine(l1, l2, p))
						{
							if((l1-p)*normal>0)
							{
								idxNormal.push_back(face[k]);
								idxCNormal.push_back(face[(k+1)%3]);
								break;
							}
							else
							{
								idxCNormal.push_back(face[k]);
								idxNormal.push_back(face[(k+1)%3]);
								break;
							}
						}
					}
				}
				std::vector<Vec3i> triangles; 
				func.arrangeVector(idxNormal);
				func.arrangeVector(idxCNormal);
				reTri.delaunayTriangulation(SurfNode, face, idxCNormal, cutFaceInSurfObj[i].pointsOnEdgeNormalDirec, cutFaceInSurfObj[i].pointsOnFaceNormalDirec, triangles, PreNbPoint);
				for(int j=0;j<triangles.size();j++)
					AddedFace.push_back(triangles[j]);

				triangles.clear();
				reTri.delaunayTriangulation(SurfNode, face, idxNormal, cutFaceInSurfObj[i].pointsOnEdgeCNormalDirec, cutFaceInSurfObj[i].pointsOnFaceCNormalDirec, triangles, PreNbPoint);
				for(int j=0;j<triangles.size();j++)
					AddedFace.push_back(triangles[j]);
			}
			else if(cutFaceInSurfObj[i].pointsOnEdgeNormalDirec.size()==1)
			{
				CutFace temp=cutFaceInSurfObj[i];
				Vec3i face=(*SurfFace)[cutFaceInSurfObj[i].faceIdx];
				Vec3f faceNormal=(*SurfNormal)[cutFaceInSurfObj[i].faceIdx];
				int edgeIdx=cutFaceInSurfObj[i].cutEdgeIdx[0];
				Vec2i edge=(*SurfEdge)[cutFaceInSurfObj[i].cutEdgeIdx[0]];
				std::vector<int> idxNormal;
				std::vector<int> idxCNormal;

				int idx=cutFaceInSurfObj[i].pointsOnEdgeNormalDirec[0]-PreNbPoint;
				Vec3f p=AddedPoint[idx];
				Vec3f normal=cutFaceInSurfObj[i].pointsOnEdgeNormal[0];

				// 삼각형의 vertex중 normal방향과 반대방향의 vertex를 나눈다.
				for(int k=0;k<3;k++)
				{
					Vec3f l1=(*SurfNode)[face[k]];
					Vec3f l2=(*SurfNode)[face[(k+1)%3]];
					if(geoFunc.isPointInLine(l1, l2, p))
					{
						if((l1-p)*normal>0)
						{
							idxNormal.push_back(face[k]);
							idxCNormal.push_back(face[(k+1)%3]);
							break;
						}
						else
						{
							idxCNormal.push_back(face[k]);
							idxNormal.push_back(face[(k+1)%3]);
							break;
						}
					}
				}

				// Cut front 위에 있는 point를 찾는다.
				for(int j=0;j<cutFaceInSurfObj[i].pointsOnFaceNormalDirec.size();j++)
				{
					idx=cutFaceInSurfObj[i].pointsOnFaceNormalDirec[j]-PreNbPoint;
					p=AddedPoint[idx];
					for(int k=0;k<pointsOnCutFront->size()-1;k++)
					{
						Vec3f l1=(*CutPoint)[(*pointsOnCutFront)[k]];
						Vec3f l2=(*CutPoint)[(*pointsOnCutFront)[k+1]];
						if(geoFunc.isPointInLine(l1, l2, p))
						{
							VertexIdxOnCutFront.push_back(cutFaceInSurfObj[i].pointsOnFaceNormalDirec[j]);
							VertexIdxOnCutFrontSurf.push_back(cutFaceInSurfObj[i].pointsOnFaceNormalDirec[j]);
							NormalOnCutFront.push_back(cutFaceInSurfObj[i].pointsOnFaceNormal[j]);
							NormalOnCutFrontSurf.push_back(cutFaceInSurfObj[i].pointsOnFaceNormal[j]);
							break;
						}
					}
				}

				std::vector<Vec3i> triangles; 
				reTri.delaunayTriangulation(SurfNode, face, edge, idxCNormal, cutFaceInSurfObj[i].pointsOnEdgeNormalDirec[0], cutFaceInSurfObj[i].pointsOnFaceNormalDirec, triangles);
				for(int j=0;j<triangles.size();j++)
				{
					reTri.legalizeTriangle(SurfNode, triangles[j], faceNormal);
					AddedFace.push_back(triangles[j]);
				}

				triangles.clear();
				reTri.delaunayTriangulation(SurfNode, face, edge, idxNormal, cutFaceInSurfObj[i].pointsOnEdgeCNormalDirec[0], cutFaceInSurfObj[i].pointsOnFaceCNormalDirec, triangles);
				for(int j=0;j<triangles.size();j++)
				{
					reTri.legalizeTriangle(SurfNode, triangles[j], faceNormal);
					AddedFace.push_back(triangles[j]);
				}
			}
		}
	}
}