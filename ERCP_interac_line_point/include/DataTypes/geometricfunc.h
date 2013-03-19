/********************************************	
*		Original source code: SOFA			*
*		Modified by Hoeryong Jung			*
*		Date: 2009. 10. 21					*
*		contact: junghl80@kaist.ac.kr		*
/*******************************************/

#ifndef GEOMETRIC_FUNC_H
#define GEOMETRIC_FUNC_H

#include "Vec.h"
#include "Mat.h"
#include "../Modules/AABB.h"
#include <vector>
#include <float.h>
#include "DataTypes/Define.h"
#include "../../CuttingSimulation_GPU/releaseLog.h"
#define isnan _isnan

class GeometricFunc
{
public:
	GeometricFunc()
	{
	};
	~GeometricFunc()
	{
	};

	static bool GeometricFunc::intersectionBtwFaceAndLine(Vec3f faceNorm, Vec3f faceP, 
							Vec3f l1, Vec3f l2, Vec3f& intersection)
	{
		Vec3f lineDirect = l2-l1;
		lineDirect.normalize();

		float numerator = (faceP-l1)*faceNorm;
		float denominator = lineDirect*faceNorm;
		if (abs(denominator) < EPS )
		{
			return false;
		}
		else
		{
			intersection = l1 + lineDirect*(numerator/denominator);
			return true;
		}
		return false;
	}

	static float GeometricFunc::angleBtwVector(Vec3f v1, Vec3f v2)
	{
		return acos(v1*v2/(v1.norm()*v2.norm()));
	}

	bool GeometricFunc::isEdgeSame(Vec2i e1, Vec2i e2)
	{
		if((e1[0]==e2[0])&&(e1[1]==e2[1]))
			return true;
		if((e1[0]==e2[1])&&(e1[1]==e2[0]))
			return true;
		return false;
	}

	bool GeometricFunc::isPointInLine(Vec3f l1, Vec3f l2, Vec3f p)
	{
		Vec3d v1=p-l1; v1.normalize();
		Vec3d v2=p-l2; v2.normalize();
		double val=v1*v2;
		if((val+1)<EPS)
			return true;
		else
			return false;
	}

	bool GeometricFunc::isPointInTri(Vec3f point, Vec3f tri[])
	{
		//point가 사각형 내부에 있는지 검사하는 함수
		Vec3f cross[3];
		cross[0]=(tri[1]-tri[0]).cross(point-tri[0]);
		cross[1]=(tri[2]-tri[1]).cross(point-tri[1]);
		cross[2]=(tri[0]-tri[2]).cross(point-tri[2]);

		if(cross[0]*cross[1]>0)
		{
			if(cross[0]*cross[2]>0)
			{
				return true;
			}
		}
		return false;
	}
	float GeometricFunc::distanceBtwPointAndLine(Vec3f p, Vec3f l1, Vec3f l2, Vec3f* pl = NULL)
	{
		Vec3f L = l2-l1;
		float coff = (p-l1)*L/(L*L);

		Vec3f nearestP;

		if (coff < 0)
		{
			nearestP = l1;
		}
		else if(coff > 1)
		{
			nearestP = l2;
		}
		else
		{
			nearestP = l1+L*coff;
		}

		if (pl)
		{
			*pl = nearestP;
		}
		return (p-nearestP).norm();
	}
	bool isBoundCollid( Vec3f tri1[], Vec3f tri2[] ) 
	{
		for (int i=0; i<3; i++)
		{
			if (isPointInTri(tri1[i], tri2)
				|| isPointInTri(tri2[i], tri1))
			{
				return true;
			}
		}
		return false;
	}

	bool GeometricFunc::isPointInTri(std::vector<Vec3f>* point, Vec3i face, int pointIdx)
	{
		Vec3f intersection=(*point)[pointIdx];
		
		//point가 삼각형 내부에 있는지 검사하는 함수
		Vec3f cross[3];
		cross[0]=((*point)[face[1]]-(*point)[face[0]]).cross(intersection-(*point)[face[0]]);
		cross[1]=((*point)[face[2]]-(*point)[face[1]]).cross(intersection-(*point)[face[1]]);
		cross[2]=((*point)[face[0]]-(*point)[face[2]]).cross(intersection-(*point)[face[2]]);

		if(cross[0]*cross[1]>0)
		{
			if(cross[0]*cross[2]>0)
				return true;
		}
		return false;
	}

	bool GeometricFunc::isPointInRect(Vec3f point, Vec3f rect[])
	{
		//point가 사각형 내부에 있는지 검사하는 함수
		Vec3f cross[4];
		cross[0]=(rect[1]-rect[0]).cross(point-rect[0]);
		cross[1]=(rect[2]-rect[1]).cross(point-rect[1]);
		cross[2]=(rect[3]-rect[2]).cross(point-rect[2]);
		cross[3]=(rect[0]-rect[3]).cross(point-rect[3]);

		if(cross[0]*cross[1]>0)
		{
			if(cross[0]*cross[2]>0)
			{
				if(cross[0]*cross[3]>0)
					return true;
			}
		}
		return false;
	}

	bool GeometricFunc::isTriOnLine(Vec3f tri[])
	{
		Vec3f v1=tri[0]-tri[1]; v1.normalize();
		Vec3f v2=tri[0]-tri[2]; v2.normalize();
		if(fabs((v1*v2)-1)<EPS)
			return true;
		else
			return false;
	}

	bool GeometricFunc::isLineInBox(Vec3f& leftDown, Vec3f& rightUp, Vec3f line[])
	{
		if(isPointInBox(leftDown, rightUp, line[0])&&isPointInBox(leftDown, rightUp, line[1]))
			return true;
		else
			return false;
	}
	bool GeometricFunc::isTriInBox(Vec3f& leftDown, Vec3f& rightUp, Vec3f tri[])
	{
		if(isPointInBox(leftDown, rightUp, tri[0])&&isPointInBox(leftDown, rightUp, tri[1])
			&&isPointInBox(leftDown, rightUp, tri[1]))
			return true;
		else
			return false;
	}
	bool GeometricFunc::isBoxInBox(Vec3f& leftDown, Vec3f& rightUp, Vec3f _leftDown, Vec3f _RightUp)
	{
		if(isPointInBox(leftDown, rightUp, _leftDown)&&isPointInBox(leftDown, rightUp, _RightUp))
			return true;
		else
			return false;
	}

	bool GeometricFunc::isPointInBox(Vec3f& leftDown, Vec3f& rightUp, Vec3f& point)
	{
		if(leftDown[0]>point[0])
			return false;
		if(leftDown[1]>point[1])
			return false;
		if(leftDown[2]>point[2])
			return false;
		if(rightUp[0]<point[0])
			return false;
		if(rightUp[1]<point[1])
			return false;
		if(rightUp[2]<point[2])
			return false;
		return true;
	}

	bool GeometricFunc::isPointInSurf(std::vector<Vec3f>* point, std::vector<Vec3i>* face, AABBNode* root, Vec3f pos)
	{
		// 1-1. Culling using AABB
		Vec3i direc(1,0,0);
		Vec3f direcF(1,0,0);
		std::vector<int> pCollisionTriIdx;
		collisionBtwAABBAndAxisLine(root, pos, direc, pCollisionTriIdx);

		// 1-2. Compute intersection 
		int count=0;
		for(int i=0;i<pCollisionTriIdx.size();i++)
		{
			Tri tri;
			tri.p[0]=(*point)[(*face)[pCollisionTriIdx[i]][0]];
			tri.p[1]=(*point)[(*face)[pCollisionTriIdx[i]][1]];
			tri.p[2]=(*point)[(*face)[pCollisionTriIdx[i]][2]];
			tri.normal=computeNormal(tri.p);
			Vec3f _intersection;

			if(collisionBtwTriDirectionalLine(pos, direcF, tri, _intersection))
			{
				count++;
			}
		}
		if((count%2)==1)
			return true;
		else
			return false;
	}

	bool GeometricFunc::isBoxInSurf(std::vector<Vec3f>* point, std::vector<Vec3i>* face, AABBNode* root, Vec3f leftDown, Vec3f rightUp)
	{
		Vec3f p[8];
		p[0]=leftDown;
		p[1]=Vec3f(leftDown[0],leftDown[1],rightUp[2]);
		p[2]=Vec3f(rightUp[0],leftDown[1],rightUp[2]);
		p[3]=Vec3f(rightUp[0],leftDown[1],leftDown[2]);
		p[4]=Vec3f(leftDown[0],rightUp[1],leftDown[2]);
		p[5]=Vec3f(leftDown[0],rightUp[1],rightUp[2]);
		p[6]=rightUp;
		p[7]=Vec3f(rightUp[0],rightUp[1],leftDown[2]);

		for(int i=0;i<8;i++)
		{
			if(isPointInSurf(point,face,root,p[i]))
				return true;
		}
		return false;
	}

	bool GeometricFunc::collisionBtwAABBAndAxisLine(AABBNode* root, Vec3f pos, Vec3i direc, std::vector<int>& pCollisionTriIdx)
	{
		Box box;
		box.leftDown=root->LeftDown;
		box.rightUp=root->RightUp;
		box.center=(box.leftDown+box.rightUp)/2.0;

		if(collisionBtwBoxAndAxisLine(box, pos, direc))
		{
			if(root->End)
			{
				pCollisionTriIdx.push_back(root->IndexInLeafNode);
				return true;
			}
			else
			{
				collisionBtwAABBAndAxisLine(root->Left, pos, direc, pCollisionTriIdx);
				collisionBtwAABBAndAxisLine(root->Right, pos, direc, pCollisionTriIdx);
			}
		}
	}

	bool GeometricFunc::collisionBtwBoxAndAxisLine(Box box, Vec3f pos, Vec3i direc)
	{
		if(abs(direc[0])==1)
		{
			if(pos[1]<box.leftDown[1])
				return false;
			if(pos[1]>box.rightUp[1])
				return false;
			if(pos[2]<box.leftDown[2])
				return false;
			if(pos[2]>box.rightUp[2])
				return false;
			return true;
		}

		if(abs(direc[1])==1)
		{
			if(pos[0]<box.leftDown[0])
				return false;
			if(pos[0]>box.rightUp[0])
				return false;
			if(pos[2]<box.leftDown[2])
				return false;
			if(pos[2]>box.rightUp[2])
				return false;
			return true;
		}

		if(abs(direc[2])==1)
		{
			if(pos[0]<box.leftDown[0])
				return false;
			if(pos[0]>box.rightUp[0])
				return false;
			if(pos[1]<box.leftDown[1])
				return false;
			if(pos[1]>box.rightUp[1])
				return false;
			return true;
		}

		return false;
	}

	int GeometricFunc::GetIntersection( float fDst1, float fDst2, Vec3f P1, Vec3f P2, Vec3f &Hit) {
		if ( (fDst1 * fDst2) >= 0.0f) return 0;
		if ( fDst1 == fDst2) return 0; 
		Hit = P1 + (P2-P1) * ( -fDst1/(fDst2-fDst1) );
		return 1;
	}

	int GeometricFunc::InBox( Vec3f Hit, Vec3f B1, Vec3f B2, const int Axis) {
		if ( Axis==1 && Hit[2] > B1[2] && Hit[2] < B2[2] && Hit[1] > B1[1] && Hit[1] < B2[1]) return 1;
		if ( Axis==2 && Hit[2] > B1[2] && Hit[2] < B2[2] && Hit[0] > B1[0] && Hit[0] < B2[0]) return 1;
		if ( Axis==3 && Hit[0] > B1[0] && Hit[0] < B2[0] && Hit[1] > B1[1] && Hit[1] < B2[1]) return 1;
		return 0;
	}


	/*bool GeometricFunc::collisionBtwBoxAndLineSeg(Box box, Vec3f l1, Vec3f l2)
	{
		Vec3f b1=box.leftDown;
		Vec3f b2=box.rightUp;
		Vec3f hit;

		if (l2[0] < b1[0] && l1[0] < b1[0]) return false;
		if (l2[0] > b2[0] && l1[0] > b2[0]) return false;
		if (l2[1] < b1[1] && l1[1] < b1[1]) return false;
		if (l2[1] > b2[1] && l1[1] > b2[1]) return false;
		if (l2[2] < b1[2] && l1[2] < b1[2]) return false;
		if (l2[2] > b2[2] && l1[2] > b2[2]) return false;
		if (l1[0] > b1[0] && l1[0] < b2[0] &&
			l1[1] > b1[1] && l1[1] < b2[1] &&
			l1[2] > b1[2] && l1[2] < b2[2]) 
		{hit = l1; 
		return true;}

		if ( (GetIntersection( l1[0]-b1[0], l2[0]-b1[0], l1, l2, hit) && InBox( hit, b1, b2, 1 ))
			|| (GetIntersection( l1[1]-b1[1], l2[1]-b1[1], l1, l2, hit) && InBox( hit, b1, b2, 2 )) 
			|| (GetIntersection( l1[2]-b1[2], l2[2]-b1[2], l1, l2, hit) && InBox( hit, b1, b2, 3 )) 
			|| (GetIntersection( l1[0]-b2[0], l2[0]-b2[0], l1, l2, hit) && InBox( hit, b1, b2, 1 )) 
			|| (GetIntersection( l1[1]-b2[1], l2[1]-b2[1], l1, l2, hit) && InBox( hit, b1, b2, 2 )) 
			|| (GetIntersection( l1[2]-b2[2], l2[2]-b2[2], l1, l2, hit) && InBox( hit, b1, b2, 3 )))
			return true;

		return false;
	}*/

	bool GeometricFunc::collisionBtwBoxAndLineSeg(Box box, Vec3f l1, Vec3f l2)
	{
		float bmin, bmax, si, ei;
		for(int i=0;i<3;i++)
		{
			bmin=box.leftDown[i];
			bmax=box.rightUp[i];
			si=l1[i];
			ei=l2[i];

			if(si<ei)
			{
				if(si>bmax || ei<bmin)
					return false;
			}
			else
			{
				if(ei>bmax || si<bmin)
					return false;
			}
		}
		return true;
	}

	static Vec3f GeometricFunc::computeNormal(std::vector<Vec3f>* points, std::vector<int>& boundayLoop)
	{
		Vec3f totalV(0,0,0);
		int nbPoint = boundayLoop.size();
		for (unsigned i=0;i<nbPoint; i++)
		{
			totalV += points->at(boundayLoop[i]).cross(points->at(boundayLoop[(i+1)%nbPoint]));
		}
		totalV.normalize();
		return totalV;
	}

	Vec3f GeometricFunc::computeNormal(Vec3f p[])
	{
		Vec3f v1=p[1]-p[0];
		Vec3f v2=p[2]-p[0];
		Vec3f n=v1.cross(v2); n.normalize();
		return n;
	}

	static Vec3f GeometricFunc::computeNormal(Vec3f& p1,Vec3f& p2,Vec3f& p3)
	{
		Vec3f v1=p2-p1;
		Vec3f v2=p3-p1;
		Vec3f n=v1.cross(v2); n.normalize();
		return n;
	}

	Vec3f GeometricFunc::computeNormal(std::vector<Vec3f>& point, Vec3i face)
	{
		Vec3f vector1, vector2, normal;
		vector1=point[face[1]]-point[face[0]]; 
		vector2=point[face[2]]-point[face[0]]; 
		normal = vector1.cross(vector2);
		normal.normalize();
		return normal;
	}

	static Vec3f GeometricFunc::computeNormal(std::vector<Vec3f>* point, Vec3i face)
	{
		Vec3f vector1, vector2, normal;
		vector1=(*point)[face[1]]-(*point)[face[0]]; 
		vector2=(*point)[face[2]]-(*point)[face[0]]; 
		normal = vector1.cross(vector2);
		normal.normalize();
		return normal;
	}


	bool GeometricFunc::collisionBtwLinesegAndTri(Line& line, Tri& tri, Vec3f& intersection)
	{
		Vec3f direc=line.l2-line.l1;
		float t=tri.normal*(tri.p[0]-line.l1)/(tri.normal*direc);
		intersection=line.l1+direc*t;
		if(t>0 && t<1)
		{
			if(isPointInTri(intersection, tri.p))
				return true;
		}
		return false;
	}

	bool GeometricFunc::collisionBtwLinesegAndTri(Vec3f line[], Vec3f tri[], Vec3f& intersection)
	{
		Vec3f direc=line[1]-line[0];
		Vec3f normal=computeNormal(tri);
		float t=normal*(tri[0]-line[0])/(normal*direc);
		intersection=line[0]+direc*t;
		if(t>0 && t<1)
		{
			if(isPointInTri(intersection, tri))
				return true;
		}
		return false;
	}

	bool GeometricFunc::collisionBtwLinesegAndTri(Vec3f l1, Vec3f l2, Vec3f tri[], Vec3f& intersection)
	{
		Vec3f direc=l2-l1;
		Vec3f normal=computeNormal(tri);
		float t=normal*(tri[0]-l1)/(normal*direc);
		intersection=l1+direc*t;
		if(t>0 && t<1)
		{
			if(isPointInTri(intersection, tri))
				return true;
		}
		return false;
	}

	bool GeometricFunc::collisionBtwTriDirectionalLine(Vec3f pos, Vec3f direc, Tri& tri, Vec3f& intersection)
	{
		float t=tri.normal*(tri.p[0]-pos)/(tri.normal*direc);
		intersection=pos+direc*t;
		if(t>0)
		{
			if(isPointInTri(intersection, tri.p))
				return true;
		}
		return false;
	}

	bool GeometricFunc::collisionBtwLinesegAndRect(Line& line, Rect& rect, Vec3f& intersection)
	{
		Vec3f direc=line.l2-line.l1;
		float t=rect.normal*(rect.p[0]-line.l1)/(rect.normal*direc);
		intersection=line.l1+direc*t;
		if(t>=0 && t<=1)
		{
			if(isPointInRect(intersection, rect.p))
				return true;
		}
		return false;
	}

	bool GeometricFunc::collisionBtwLinesegAndRect(Vec3f line[], Vec3f rect[], Vec3f& intersection)
	{
		Vec3f direc=line[1]-line[0];
		Vec3f normal=computeNormal(rect);
		float t=normal*(rect[0]-line[0])/(normal*direc);
		intersection=line[0]+direc*t;
		if(t>=0 && t<=1)
		{
			if(isPointInRect(intersection, rect))
				return true;
		}
		return false;
	}

	bool GeometricFunc::collisionBtwLinesegAndRect(Vec3f l1, Vec3f l2, Vec3f rect[], Vec3f& intersection)
	{
		Vec3f direc=l2-l1;
		Vec3f normal=computeNormal(rect);
		float t=normal*(rect[0]-l1)/(normal*direc);
		intersection=l1+direc*t;
		if(t>=0 && t<=1)
		{
			if(isPointInRect(intersection, rect))
				return true;
		}
		return false;
	}

	bool GeometricFunc::collisionBtwTri(Vec3f tri1[], Vec3f tri2[])
	{
		for(int i=0;i<3;i++)
		{
			Vec3f intersection;
			if(collisionBtwLinesegAndTri(tri1[i], tri1[(i+1)%3], tri2, intersection))
				return true;
		}
		for(int i=0;i<3;i++)
		{
			Vec3f intersection;
			if(collisionBtwLinesegAndTri(tri2[i], tri2[(i+1)%3], tri1, intersection))
				return true;
		}
		return false;
	}

	bool GeometricFunc::collisionBtwBox(Box& box1, Box& box2)
	{
		Vec3f radius1=box1.center-box1.leftDown;
		Vec3f radius2=box2.center-box2.leftDown;
		Vec3f length=box2.center-box1.center;

		// 1. x-axis test
		if(fabs(length[0])>radius1[0]+radius2[0])
			return false;
		// 2. y-axis test
		if(fabs(length[1])>radius1[1]+radius2[1])
			return false;
		// 3. z-axis test
		if(fabs(length[2])>radius1[2]+radius2[2])
			return false;
		return true;
	}

	bool GeometricFunc::isTriangleContainEdge(Vec2i edge, Vec3i triangle)
	{
		int count=0;
		for(int i=0;i<3;i++)
		{
			if(edge[0]==triangle[i])
				count++;
			if(edge[1]==triangle[i])
				count++;
		}
		if(count==2)
			return true;
		else
			return false;
	}
	
	Vec3f GeometricFunc::computeCircumcenter(std::vector<Vec3f>& point, Vec3i face, Vec3f faceNormal)
	{
		Vec3f midPoint[2];
		Vec3f normal[2];
		midPoint[0]=(point[face[0]]+point[face[1]])/2.0;
		midPoint[1]=(point[face[0]]+point[face[2]])/2.0;
		normal[0]=(point[face[0]]-point[face[1]]).cross(faceNormal); normal[0].normalize();
		normal[1]=(point[face[0]]-point[face[2]]).cross(faceNormal); normal[1].normalize();
		return intersectionBtwLinesWithDirection(midPoint[0], normal[0], midPoint[1], normal[1]);
	}

	Vec3f GeometricFunc::computeCircumcenter(std::vector<Vec3f>* point, Vec3i face, Vec3f faceNormal)
	{
		Vec3f midPoint[2];
		Vec3f normal[2];
		midPoint[0]=((*point)[face[0]]+(*point)[face[1]])/2.0;
		midPoint[1]=((*point)[face[0]]+(*point)[face[2]])/2.0;
		normal[0]=((*point)[face[0]]-(*point)[face[1]]).cross(faceNormal); normal[0].normalize();
		normal[1]=((*point)[face[0]]-(*point)[face[2]]).cross(faceNormal); normal[1].normalize();
		return intersectionBtwLinesWithDirection(midPoint[0], normal[0], midPoint[1], normal[1]);
	}

	Vec3f GeometricFunc::intersectionBtwLinesWithDirection(Vec3f p1, Vec3f d1, Vec3f p2, Vec3f d2)
	{
		Vec3f intersection;
		Vec3f c=p2-p1;
		Vec3f cross=d1.cross(d2);
		float s=(c.cross(d2)*(d1.cross(d2)))/(d1.cross(d2)).norm2();
		intersection=p1+d1*s;
		return intersection;
	}

	void GeometricFunc::computeRotationMatrix(Vec3f axis, float angle, Mat3x3f& rot)
	{
		float x,y,z;
		x=axis[0];
		y=axis[1];
		z=axis[2];

		rot(0,0)=x*x+(y*y+z*z)*cos(angle);
		rot(1,1)=y*y+(x*x+z*z)*cos(angle);
		rot(2,2)=z*z+(x*x+y*y)*cos(angle);
		rot(0,1)=(1-cos(angle))*x*y+z*sin(angle);
		rot(1,0)=(1-cos(angle))*x*y-z*sin(angle);
		rot(0,2)=(1-cos(angle))*x*z-y*sin(angle);
		rot(2,0)=(1-cos(angle))*z*x+y*sin(angle);
		rot(1,2)=(1-cos(angle))*y*z+x*sin(angle);
		rot(2,1)=(1-cos(angle))*z*y-x*sin(angle);

		rot.transpose();
	}

	float GeometricFunc::disBtwBox(Box& box1, Box& box2)
	{
		Vec3f dis;
		Vec3f radius1=box1.center-box1.leftDown;
		Vec3f radius2=box2.center-box2.leftDown;
		Vec3f length=box2.center-box1.center;

		for(int i=0;i<3;i++)
		{
			if(fabs(length[i])<(radius1[i]+radius2[i]))
				dis[i]=0;
			else
				dis[i]=fabs(length[i])-(radius1[i]+radius2[i]);
		}
		return dis.norm();
	}

	float GeometricFunc::disBtwPointAndBox(Box& box, Vec3f point)
	{
		Vec3f dis;
		Vec3f radius=box.center-box.leftDown;
		Vec3f length=point-box.center;

		for(int i=0;i<3;i++)
		{
			if(fabs(length[i])<(radius[i]))
				dis[i]=0;
			else
				dis[i]=fabs(length[i])-(radius[i]);
		}
		return dis.norm();
	}

	float GeometricFunc::disBtwPointAndTri(Vec3f P, Vec3f S[3], Vec3f& Q)
	{
		Vec3f Sv[3];
		
		Sv[0]=S[1]-S[0];
		Sv[1]=S[2]-S[1];
		Sv[2]=S[0]-S[2];

		Vec3f Sn;
		float Snl;       
		Sn=Sv[0].cross(Sv[1]);	// Compute normal to S triangle
		Sn.normalize();
			
		Vec3f V, Z;
		V=P-S[0];
		Z=Sn.cross(Sv[0]);

		// 1. 가까운 point가 triangle의 face내부에 있을 경우
		if ((V*Z) > 0)
		{
			V=P-S[1];
			Z=Sn.cross(Sv[1]);
			if ((V*Z) > 0)
			{
				V=P-S[2];
				Z=Sn.cross(Sv[2]);
				if ((V*Z) > 0)
				{
					// T[point] passed the test - it's a closest point for 
					// the T triangle; the other point is on the face of S
					Vec3f temp=S[0]-P;
					Q=P+Sn*(Sn*temp);
					return (P-Q).norm();
				}
			}
		}
		
		// 2. 가까운 point가 triangle의 edge위에 있을 경우
		float minDis=100000000;
		bool flag=false;
		for(int i=0;i<3;i++)
		{
			Vec3f l1=S[i]; Vec3f l2=S[(i+1)%3];
			float t=-(l1-P)*(l2-l1)/(l2-l1).norm2();

			if((t>0) && (t<1))
			{
				Vec3f temp=l1+(l2-l1)*t;
				float dis=(P-temp).norm();
				if(minDis>dis)
				{
					minDis=dis;
					Q=temp;
				}
			}
		}

		// 3. 가까운 point가 triangle의 point일 경우
		for(int i=0;i<3;i++)
		{
			float dis=(P-S[i]).norm();
			if(minDis>dis)
			{
				minDis=dis;
				Q=S[i];
			}
		}
		return (P-Q).norm();
	}

	float GeometricFunc::disBtwTri(Vec3f S[3], Vec3f T[3], Vec3f& P, Vec3f& Q)
	{
		Vec3f Sv[3], Tv[3];
		Vec3f VEC;

		Sv[0]=S[1]-S[0];
		Sv[1]=S[2]-S[1];
		Sv[2]=S[0]-S[2];

		Tv[0]=T[1]-T[0];
		Tv[1]=T[2]-T[1];
		Tv[2]=T[0]-T[2];

		Vec3f V;
		Vec3f Z;
		Vec3f minP, minQ;
		float mindd;
		int shown_disjoint=0;

		mindd=(S[0]-T[0]).norm2()+1;

		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				// Find closest points on edges i & j, plus the 
				// vector (and distance squared) between these points

				SegPoints(VEC,P,Q,S[i],Sv[i],T[j],Tv[j]);

				V=Q-P;
				float dd = V*V;

				// Verify this closest point pair only if the distance 
				// squared is less than the minimum found thus far.

				if (dd <= mindd)
				{
					minP=P;
					minQ=Q;
					mindd = dd;

					Z=S[(i+2)%3]-P;
					float a = Z*VEC;
					Z=T[(j+2)%3]-Q;
					float b = Z*VEC;

					if ((a <= 0) && (b >= 0)) return sqrt(dd);

					float p = V*VEC;

					if (a < 0) a = 0;
					if (b > 0) b = 0;
					if ((p - a + b) > 0) shown_disjoint = 1;	
				}
			}
		}

		// No edge pairs contained the closest points.  
		// either:
		// 1. one of the closest points is a vertex, and the
		//    other point is interior to a face.
		// 2. the triangles are overlapping.
		// 3. an edge of one triangle is parallel to the other's face. If
		//    cases 1 and 2 are not true, then the closest points from the 9
		//    edge pairs checks above can be taken as closest points for the
		//    triangles.
		// 4. possibly, the triangles were degenerate.  When the 
		//    triangle points are nearly colinear or coincident, one 
		//    of above tests might fail even though the edges tested
		//    contain the closest points.

		// First check for case 1

		Vec3f Sn;
		float Snl;       
		Sn=Sv[0].cross(Sv[1]); // Compute normal to S triangle
		Snl=Sn*Sn;      // Compute square of length of normal

		// If cross product is long enough,

		if (Snl > 1e-15)  
		{
			// Get projection lengths of T points

			Vec3f Tp; 

			V=S[0]-T[0];
			Tp[0] = V*Sn;

			V=S[0]-T[1];
			Tp[1] = V*Sn;

			V=S[0]-T[2];
			Tp[2] = V*Sn;

			// If Sn is a separating direction,
			// find point with smallest projection

			int point = -1;
			if ((Tp[0] > 0) && (Tp[1] > 0) && (Tp[2] > 0))
			{
				if (Tp[0] < Tp[1]) point = 0; else point = 1;
				if (Tp[2] < Tp[point]) point = 2;
			}
			else if ((Tp[0] < 0) && (Tp[1] < 0) && (Tp[2] < 0))
			{
				if (Tp[0] > Tp[1]) point = 0; else point = 1;
				if (Tp[2] > Tp[point]) point = 2;
			}

			// If Sn is a separating direction, 

			if (point >= 0) 
			{
				shown_disjoint = 1;

				// Test whether the point found, when projected onto the 
				// other triangle, lies within the face.

				V=T[point]-S[0];
				Z=Sn.cross(Sv[0]);
				if ((V*Z) > 0)
				{
					V=T[point]-S[1];
					Z=Sn.cross(Sv[1]);
					if ((V*Z) > 0)
					{
						V=T[point]-S[2];
						Z=Sn.cross(Sv[2]);
						if ((V*Z) > 0)
						{
							// T[point] passed the test - it's a closest point for 
							// the T triangle; the other point is on the face of S

							P=T[point]+Sn*Tp[point]/Snl;
							Q=T[point];
							return (P-Q).norm();
						}
					}
				}
			}
		}

		Vec3f Tn;
		float Tnl;       
		Tn=Tv[0].cross(Tv[1]); 
		Tnl = Tn*Tn;      

		if (Tnl > 1e-15)  
		{
			Vec3f Sp; 

			V=T[0]-S[0];
			Sp[0] = V*Tn;

			V=T[0]-S[1];
			Sp[1] = V*Tn;

			V=T[0]-S[2];
			Sp[2] = V*Tn;

			int point = -1;
			if ((Sp[0] > 0) && (Sp[1] > 0) && (Sp[2] > 0))
			{
				if (Sp[0] < Sp[1]) point = 0; else point = 1;
				if (Sp[2] < Sp[point]) point = 2;
			}
			else if ((Sp[0] < 0) && (Sp[1] < 0) && (Sp[2] < 0))
			{
				if (Sp[0] > Sp[1]) point = 0; else point = 1;
				if (Sp[2] > Sp[point]) point = 2;
			}

			if (point >= 0) 
			{ 
				shown_disjoint = 1;

				V=S[point]-T[0];
				Z=Tn.cross(Tv[0]);
				if ((V*Z) > 0)
				{
					V=S[point]-T[1];
					Z=Tn.cross(Tv[1]);
					if ((V*Z) > 0)
					{
						V=S[point]-T[2];
						Z=Tn.cross(Tv[2]);
						if ((V*Z) > 0)
						{
							P=S[point];
							Q=S[point]+Tn*Sp[point]/Tnl;
							return (P-Q).norm();
						}
					}
				}
			}
		}

		// Case 1 can't be shown.
		// If one of these tests showed the triangles disjoint,
		// we assume case 3 or 4, otherwise we conclude case 2, 
		// that the triangles overlap.

		if (shown_disjoint)
		{
			P=minP;
			Q=minQ;
			return sqrt(mindd);
		}
		else return 0;
	}

	static void GeometricFunc::SegPoints(Vec3f& VEC, Vec3f& X, Vec3f& Y,		//closest points
		const Vec3f P, const Vec3f A,			// seg 1 origin, vector
		const Vec3f Q, const Vec3f B)			// seg 2 origin, vector
	{
		Vec3f T, TMP;
		float A_dot_A, B_dot_B, A_dot_B, A_dot_T, B_dot_T;

		T=Q-P;
		A_dot_A = A*A;
		B_dot_B = B*B;
		A_dot_B = A*B;
		A_dot_T = A*T;
		B_dot_T = B*T;

		// t parameterizes ray P,A 
		// u parameterizes ray Q,B 

		float t,u;

		// compute t for the closest point on ray P,A to
		// ray Q,B

		float denom = A_dot_A*B_dot_B - A_dot_B*A_dot_B;

		t = (A_dot_T*B_dot_B - B_dot_T*A_dot_B) / denom;

		// clamp result so t is on the segment P,A

		if ((t < 0) || isnan(t)) t = 0; else if (t > 1) t = 1;

		// find u for point on ray Q,B closest to point at t

		u = (t*A_dot_B - B_dot_T) / B_dot_B;

		// if u is on segment Q,B, t and u correspond to 
		// closest points, otherwise, clamp u, recompute and
		// clamp t 

		if ((u <= 0) || isnan(u)) {

			Y=Q;

			t = A_dot_T / A_dot_A;

			if ((t <= 0) || isnan(t)) {
				X=P;
				VEC=Q-P;
			}
			else if (t >= 1) {
				X=P+A;
				VEC=Q+X;
			}
			else {
				X=P+A*t;
				TMP=T.cross(A);
				VEC=A.cross(TMP);
			}
		}
		else if (u >= 1) {

			Y=Q+B;

			t = (A_dot_B + A_dot_T) / A_dot_A;

			if ((t <= 0) || isnan(t)) {
				X=P;
				VEC=Y-P;
			}
			else if (t >= 1) {
				X=P+A;
				VEC=Y-X;
			}
			else {
				X=P+A*t;
				T=Y-P;
				TMP=T.cross(A);
				VEC=A.cross(TMP);
			}
		}
		else {

			Y=Q+B*u;

			if ((t <= 0) || isnan(t)) {
				X=P;
				TMP=T.cross(B);
				VEC=B.cross(TMP);
			}
			else if (t >= 1) {
				X=P+A;
				T=Q-X;
				TMP=T.cross(B);
				VEC=B.cross(TMP);
			}
			else {
				X=P+A*t;
				VEC=A.cross(B);
				if ((VEC*T) < 0) {
					VEC=VEC*-1;
				}
			}
		}
	}

	static bool isEdge(int pt1, int pt2, std::vector<Vec2i>* edges, int &index)
	{
		for (int i=0; i<edges->size(); i++)
		{
			if( (pt1 == (*edges)[i][0] && pt2 == (*edges)[i][1])
				||(pt2 == (*edges)[i][0] && pt1 == (*edges)[i][1]) )
			{
				index = i;
				return true;
			}
		}
		return false;
	}

	static float distanceBtwLineAndLine(Vec3f l1, Vec3f l2, Vec3f p1, Vec3f p2)
	{
		Vec3f X,Y,V;
		SegPoints(V,X,Y,l1,l2-l1,p1,p2-p1);
		return (Y-X).norm();
	}

	// Note: This is line vs line segment
	static float distanceBtwLineAndLineSeg(Vec3f l1, Vec3f l2, Vec3f p1, Vec3f p2)
	{
		Vec3f d1 = l2-l1;
		Vec3f d2 = p2-p1;

		float a1 = d1*d1, b1 = -d2*d1, c1 = (p1-l1)*d1;
		float a2 = d1*d2, b2 = -d2*d2, c2 = (p1-l1)*d2;

		float detA = b2*a1-b1*a2;
		float det1 = c1*b2 - b1*c2;
		float det2 = a1*c2 - c1*a2;

		Vec3f pt1, pt2;
		if (abs(detA) < EPS)//parallel?
		{
			float s1 = (p1-l1)*d1 / (d1*d1);
			float s2 = (p2-l1)*d1 / (d1*d1);
			float ss1 = min(s1,s2), ss2 = max(s1,s2);
			if (ss1 > 1){
				pt1 = l2;
				pt2 = p1;
			}
			else if (ss2 < 0){
				pt1 = l1;
				pt2 = p2;
			}
			else{
				pt2 = p1;
				pt1 = l1+d1*s1;
			}
			return (pt2-pt1).norm();
		}
		else
		{
			float s1 = det1/detA;
			float s2 = det2/detA;

			pt1 = p1+d2*s2;
			if (s2>1)
				pt1 = p2;
			else if (s2<0)
				pt1 = p1;

			float s = (pt1-l1)*d1 / (d1*d1);//project point to line
			pt2 = l1+d1*s;
			return (pt2-pt1).norm();
		}

	}

	static float distanceBtwTriAndLine( arrayVec3f* points, Vec3i tri, Vec3f l1, Vec3f l2,
		Vec3f* pOnTri = NULL, Vec3f* pOnLine = NULL ) 
	{
		Vec3f triPoints;
		Vec3f linePoints;
		measureMinimumDistanceBetLineandTriangle(l1,l2,
			(*points)[tri[0]], (*points)[tri[1]], (*points)[tri[2]], triPoints,linePoints);
		if (pOnTri)
			(*pOnTri) = triPoints;

		if (pOnLine)
			(*pOnLine) = linePoints;

		return (triPoints - linePoints).norm();
	}

	//////////////////////////////////////////////////////////////////////////
	// Line - tri distance
	static void measureMinimumDistanceBetLineandTriangle(Vec3f p1, Vec3f p2,Vec3f tri1, Vec3f tri2, Vec3f tri3, Vec3f &collidedTri, Vec3f &collidedCyl)
	{
		Vec3f direc=(p2-p1);
		direc.normalize();
		double Length;
		Vec3f tripoint[3];

		Vec3f collidedTriVertex;
		Vec3f collidedTriEdge;
		Vec3f collidedTriPlane;
		Vec3f collidedTriPoint;
		Vec3f collidedCylinder;
		Vec3f measuredCylinder;
		tripoint[0]=tri1;
		tripoint[1]=tri2;
		tripoint[2]=tri3;

		double L;

		//Point and Line


		Length=1000000000000;

		for(int i=0;i<3;i++)
		{
			collidedCylinder=p1+direc*((tripoint[i]-p1)*direc);
			if((collidedCylinder-p1)*(collidedCylinder-p2)<0)
			{
				L=(collidedCylinder-tripoint[i]).norm();
				if(L<Length){
					measuredCylinder=collidedCylinder;
					collidedTriPoint=tripoint[i];
					Length=L;
				}
			}
		}


		//Line and Line
		int ii[2];

		for(int i=0;i<3;i++)
		{

			ii[0]=i%3;
			ii[1]=(i+1)%3;
			collidedTriEdge=findPointBetweenTwoline1(tripoint[ii[0]],tripoint[ii[1]],p1,p2);
			collidedCylinder=p1+direc*((collidedTriEdge-p1)*direc);
			if((collidedTriEdge-tripoint[ii[0]])*(collidedTriEdge-tripoint[ii[1]])<0)
			{
				if((collidedCylinder-p1)*(collidedCylinder-p2)<0)
				{
					L=(collidedCylinder-collidedTriEdge).norm();
					if(L<Length){
						Length=L;
						measuredCylinder=collidedCylinder;
						collidedTriPoint=collidedTriEdge;
					}
				}
			}
		}

		//end point of line element and triangle

		collidedTriPlane=findClosestPointInTri(p1,tri1,tri2,tri3);
		if((p1-collidedTriPlane).norm()<Length)
		{
			Length=(p1-collidedTriPlane).norm();
			measuredCylinder=p1;
			collidedTriPoint=collidedTriPlane;
		}      

		collidedTriPlane=findClosestPointInTri(p2,tri1,tri2,tri3);

		if((p2-collidedTriPlane).norm()<Length)
		{
			Length=(p2-collidedTriPlane).norm();
			measuredCylinder=p2;
			collidedTriPoint=collidedTriPlane;
		}

		//return

		collidedTri=collidedTriPoint;
		collidedCyl=measuredCylinder;

	}

	static Vec3f findPointBetweenTwoline1(Vec3f p1, Vec3f p2, Vec3f p3, Vec3f p4)
	{
		Vec3f direc=p4-p3; direc.normalize();
		Vec3f r0=p3;
		double ta,tb,da,db,t,s;
		Vec3f Qa,Qb,Pa,Pb,Pa2,P,P2;


		Pa=p1;  Pb=p2;        
		ta=projectOnLine(r0, direc, Pa);            
		tb=projectOnLine(r0, direc, Pb);
		Qa=r0+direc*ta;               Qb=r0+direc*tb;
		da=(Pa-Qa).norm();     db=(Pb-Qb).norm();
		Pa2=Pa+Qb-Qa;
		s=(Pa2-Pb).norm();
		t=(da*da-db*db+s*s)/(2*s*s);

		P=Pa+(Pb-Pa)*t;

		return P;     
	}




	static double projectOnLine(Vec3f r0, Vec3f direc, Vec3f P)
	{
		return (P*direc-r0*direc);
	}
	static double projectOnPlane(Vec3f Normal,Vec3f Center,Vec3f Point)
	{

		double dis=(Normal*(Point-Center))/(Normal.norm());
		if(dis>0)
		{
			return dis;
		}else{
			return -dis;
		}
	}


	static Vec3f findProjectedPointToPlane(Vec3f Point, Vec3f Normal, Vec3f Center)
	{
		Normal.normalize();
		double d=(Point-Center)*Normal;

		Point=Point-Normal*d;

		return Point;
	}

	static Vec3f findClosestPointInTri(Vec3f point, Vec3f tri1, Vec3f tri2, Vec3f tri3)
	{
		double Length=1000000000000000;

		Vec3f Normal=(tri2-tri1).cross(tri3-tri1);
		Normal.normalize();
		Vec3f Center=tri1;
		Vec3f ProjectedPoint=findProjectedPointToPlane(point,Normal,Center);
		Vec3f tri[3];
		int index=0;
		tri[0]=tri1;
		tri[1]=tri2;
		tri[2]=tri3;

		double flag1=((tri2-tri1).cross(ProjectedPoint-tri1))*((ProjectedPoint-tri1).cross(tri3-tri1));
		double flag2=((tri3-tri2).cross(ProjectedPoint-tri2))*((ProjectedPoint-tri2).cross(tri1-tri2));

		if(flag1>=0 && flag2>=0)
		{
			return ProjectedPoint;
		}
		else
		{
			for(int i=0;i<3;i++)
			{
				if((tri[i]-point).norm()<Length)
				{
					Length=(tri[i]-point).norm();
					index=i;
				}
			}

			return tri[index];
		}
	}
	/// Line - tri distance
};
#endif
