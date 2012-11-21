#include "StdAfx.h"
#include "LineTool.h"
#include "textureManager.h"

LineTool::LineTool(void)
{
	catheter.Load("../data/catheter.obj");
}

LineTool::~LineTool(void)
{
}

void LineTool::init( Vec3f pt1, Vec3f pt2 )
{
	m_prePoint.push_back(pt1);
	m_prePoint.push_back(pt2);

	m_curPoint = m_prePoint;
}

void LineTool::draw( int mode )
{
	if (mode == 3)
	{
		glPushMatrix();

		glColor3f(1,1,1);
		Vec3f p0 = m_curPoint[0];
		Vec3f p1 = m_curPoint[1];
		Vec3f direct = p1-p0;
		double alp = atan2(-direct[2], direct[0]);
		double beta = atan2(direct[1], sqrt(direct[0]*direct[0]+direct[2]*direct[2]));

		glTranslated(p0[0], p0[1], p0[2]);
		glRotated(alp,0,1,0);
		glRotated(beta,0,0,1);
		glRotated(180,0,0,1);
		glRotated(-3.6,0,1,0);

		double scal = direct.norm()/5.0;
		glScaled(scal,scal,scal);

		glEnable(GL_TEXTURE_2D);
		textureManager::maptexture(TEXTURE_CATHETER);
		catheter.Draw();
		glDisable(GL_TEXTURE_2D);

		glScaled(1,1,1);
		glPopMatrix();
	}

	arrayVec3f points = point();
	arrayVec3i faces = face();

	if (mode == 1)
	{
		glColor3f(0,1,0);
		glBegin(GL_TRIANGLES);
		for (int i=0; i<faces.size(); i++)
		{
			for(int j = 0; j < 3; j++)
			{
				Vec3f curNode = points[faces[i][j]];
				glVertex3d(curNode[0], curNode[1], curNode[2]);
			}
		}
		glEnd();
	}

	if (mode == 2)//cylinder tool
	{
		glColor3f(0.1,0.6,0.6);

		for (int i = 0; i < m_curPoint.size()-1; i++)
		{
			glPushMatrix();
			Vec3f zAxis = Vec3f(0,0,1.0);
			Vec3f newAxis = m_curPoint[i+1] - m_curPoint[i];
			Vec3f rotation_axis = zAxis.cross(newAxis);
			float rotation_angle = asin(rotation_axis.norm()/(zAxis.norm()*newAxis.norm()))*(180.0/PI);
			glTranslated(m_curPoint[i][0], m_curPoint[i][1], m_curPoint[i][2]);
			glRotated(rotation_angle, rotation_axis[0], rotation_axis[1], rotation_axis[2]);

			GLUquadricObj *quadObj = gluNewQuadric();
			gluCylinder(quadObj, TOOL_RADIUS, TOOL_RADIUS, (GLdouble)newAxis.norm(), 20, 20);
			glPopMatrix();
		}
	}
}

void LineTool::moveCurrentPoint( Vec3f _trans )
{
	m_curPoint[0] += _trans;
	m_curPoint[1] += _trans;
}

arrayVec3f LineTool::point()
{
	arrayVec3f points;
	points.insert(points.end(), m_curPoint.begin(), m_curPoint.end());
	points.insert(points.end(), m_prePoint.begin(), m_prePoint.end());
	return points;
}

arrayVec3i LineTool::face()
{
	arrayVec3i face;
	face.push_back(Vec3i(0,1,2));
	face.push_back(Vec3i(1,3,2));
	return face;
}

void LineTool::reset()
{
	m_prePoint = m_curPoint;
}

arrayVec3f* LineTool::frontPoint()
{
	return &m_curPoint;
}
