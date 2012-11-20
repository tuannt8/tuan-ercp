#ifndef RETRIANGULATION
#define RETRIANGULATION

#include "DataTypes/Define.h"
#include "DataTypes/Vec.h"
#include "DataTypes/Mat.h"
#include "DataTypes/GeometricFunc.h"
#include <afxwin.h>
#include <GL/glu.h>
#include <GL/gl.h>
#include <vector>

class ReTriangulation
{
public:
	ReTriangulation();
	~ReTriangulation();

public:
	void delaunayTriangulation(std::vector<Vec3f>* point, Vec3i face, Vec2i edge, std::vector<int>& idx, int pointOnEdge, std::vector<int>& pointOnFace, std::vector<Vec3i>& triangles);
	void delaunayTriangulation(std::vector<Vec3f>* point, Vec3i face, Vec3f tri[], std::vector<int>& pointOnEdge, std::vector<int>& pointOnFace, std::vector<Vec3i>& triangles, int preNbPoint);
	void delaunayTriangulation(std::vector<Vec3f>* point, Vec3i face, std::vector<int>& idx, std::vector<int>& pointOnEdge, std::vector<int>& pointOnFace, std::vector<Vec3i>& triangles, int preNbPoint);
	void legalizeTriangle(std::vector<Vec3f>* point, Vec3i& face, Vec3f& normal);
	void legalizeTriangle(Vec3f point[], Vec3i& face, Vec3f& normal);

private:
	void findIllegalEdge(std::vector<Vec3f>* point, std::vector<int>& pointOnEdge, std::vector<int>& pointOnFace, std::vector<Vec2i>& illegalEdge);
	int legalizeEdge(std::vector<Vec3f>& point, int pointIdx, Vec2i edge, int triIdx, std::vector<Vec3i>& triangles);
	int legalizeEdge(std::vector<Vec3f>* point, int pointIdx, Vec2i edge, int triIdx, std::vector<Vec3i>& triangles);
	void legalizeEdge(std::vector<Vec3f>* point, std::vector<Vec2i>& illegalEdge, std::vector<Vec3i>& triangles);
};

#endif