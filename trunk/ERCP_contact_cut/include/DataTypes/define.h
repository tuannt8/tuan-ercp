#ifndef DEFINE_H
#define DEFINE_H

#include "DataTypes/vec.h"
#include <vector>

#define PI 3.141592
#define EPS 0.000001
//#define GRAVITY 98
#define GRAVITY 98
#define MAX 100000000
#define MIN -100000000
#define MARGIN 5
#define GAUSS_QUADRATURE 0
#define NODAL_INTEGRATION 1
#define STRESS_POINT 2

#define SPLINE3 0		//3rd order spline
#define SPLINE4 1		//4th order spline
#define EXPONENTIAL 2

#define E		1000
#define NU		0.3
#define DENSITY	0.01 //0.02 for majorpapilla; 0.001 for liver; 0.1 for papilla
#define DAMPING	100

#define TOOL_RADIUS 3

typedef struct
{
	Vec3f leftDown;
	Vec3f rightUp;
	Vec3f center;
} Box;

typedef struct
{
	Vec3f p[4];
	Vec3f normal;
	Box box;
} Rect;

typedef struct
{
	Vec3f p[3];
	Vec3f normal;
	Box box;
} Tri;

typedef struct
{
	Vec3f l1;
	Vec3f l2;
	Vec3f center;
} Line;

typedef std::vector<Vec3f> arrayVec3f;
typedef std::vector<Vec3i> arrayVec3i;
typedef std::vector<int> arrayInt;
typedef std::vector<Vec2f> arrayVec2f;
typedef std::vector<Vec2i> arrayVec2i;

#endif