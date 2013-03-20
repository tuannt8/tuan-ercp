#include "StdAfx.h"
#include "wireTest.h"
#include "GL/GL.h"

wireTest::wireTest(void)
{
}

wireTest::~wireTest(void)
{
}

void wireTest::draw( int mode )
{
	if (mode == 1) // draw line only
	{
		glColor3f(1,1,0);
		glBegin(GL_LINES);
		for (int i=0; i<m_points.size()-1; i++)
		{
			glVertex3f(m_points[i][0], m_points[i][1], m_points[i][2]);
			glVertex3f(m_points[i+1][0], m_points[i+1][1], m_points[i+1][2]);
		}
		glEnd();
	}

	if (mode == 1)
	{
		glColor3f(0,1,1);
		glPointSize(4.0);
		glBegin(GL_POINTS);
		for (int i=0; i<m_points.size(); i++)
		{
			glVertex3f(m_points[i][0], m_points[i][1], m_points[i][2]);
		}
		glEnd();
	}
}

void wireTest::move( Vec3f offset )
{
	for (int i=0; i<m_points.size(); i++)
	{
		m_points[i]+=offset;
	}
}

void wireTest::init()
{
	float segL = 20;
	for (int i=0; i<10; i++)
	{
		m_points.push_back(Vec3f(-segL*i, 0, 0));
	}

	m_velocity.resize(m_points.size());
	m_radius = 2;
}
