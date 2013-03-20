#pragma once
#include "define.h"
#include "stdafx.h"

class wireTest
{
public:
	wireTest(void);
	~wireTest(void);

	void init();
	void draw(int mode);
	void move(Vec3f offset);

	//variable
	arrayVec3f m_points;
	arrayVec3f m_velocity;
	float m_radius;
};
