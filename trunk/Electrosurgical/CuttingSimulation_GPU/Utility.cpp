#include "StdAfx.h"
#include "Utility.h"
#include "GL/GL.h"
#include "GL/glut.h"


void Utility::printw (float x, float y, float z, char* format, ...)
{
	glDisable(GL_LIGHTING);
	glColor3f(0.6, 0.2, 0.2);

	GLvoid *font_style = GLUT_BITMAP_8_BY_13;

	va_list args;   //  Variable argument list
	int len;        // String length
	int i;          //  Iterator
	char * text;    // Text

	va_start(args, format);
	len = _vscprintf(format, args) + 1;
	text = new char[len];
	vsprintf_s(text, len, format, args);
	va_end(args);

	//  Specify the raster position for pixel operations.
	glRasterPos3f (x, y, z);
	for (i = 0; text[i] != '\0'; i++)
		glutBitmapCharacter(font_style, text[i]);
	delete []text;

	glEnable(GL_LIGHTING);
}

void Utility::drawBox( Vec3f leftDown, Vec3f rightUp )
{
	glPushMatrix();
	Vec3f center = (leftDown+rightUp)/2;
	Vec3f direct = rightUp-leftDown;

	glTranslatef(center[0], center[1], center[2]);
	glScalef(direct[0]/2.0, direct[1]/2.0, direct[2]/2.0);

	glBegin(GL_QUADS); 

	glVertex3f( 1.0f, 1.0f,-1.0f);          // Top Right Of The Quad (Top)
	glVertex3f(-1.0f, 1.0f,-1.0f);          // Top Left Of The Quad (Top)
	glVertex3f(-1.0f, 1.0f, 1.0f);          // Bottom Left Of The Quad (Top)
	glVertex3f( 1.0f, 1.0f, 1.0f);          // Bottom Right Of The Quad (Top)

	glVertex3f( 1.0f,-1.0f, 1.0f);          // Top Right Of The Quad (Bottom)
	glVertex3f(-1.0f,-1.0f, 1.0f);          // Top Left Of The Quad (Bottom)
	glVertex3f(-1.0f,-1.0f,-1.0f);          // Bottom Left Of The Quad (Bottom)
	glVertex3f( 1.0f,-1.0f,-1.0f);          // Bottom Right Of The Quad (Bottom)

	glVertex3f( 1.0f, 1.0f, 1.0f);          // Top Right Of The Quad (Front)
	glVertex3f(-1.0f, 1.0f, 1.0f);          // Top Left Of The Quad (Front)
	glVertex3f(-1.0f,-1.0f, 1.0f);          // Bottom Left Of The Quad (Front)
	glVertex3f( 1.0f,-1.0f, 1.0f);          // Bottom Right Of The Quad (Front)

	glVertex3f( 1.0f,-1.0f,-1.0f);          // Bottom Left Of The Quad (Back)
	glVertex3f(-1.0f,-1.0f,-1.0f);          // Bottom Right Of The Quad (Back)
	glVertex3f(-1.0f, 1.0f,-1.0f);          // Top Right Of The Quad (Back)
	glVertex3f( 1.0f, 1.0f,-1.0f);          // Top Left Of The Quad (Back)

	glVertex3f(-1.0f, 1.0f, 1.0f);          // Top Right Of The Quad (Left)
	glVertex3f(-1.0f, 1.0f,-1.0f);          // Top Left Of The Quad (Left)
	glVertex3f(-1.0f,-1.0f,-1.0f);          // Bottom Left Of The Quad (Left)
	glVertex3f(-1.0f,-1.0f, 1.0f);          // Bottom Right Of The Quad (Left)

	glVertex3f( 1.0f, 1.0f,-1.0f);          // Top Right Of The Quad (Right)
	glVertex3f( 1.0f, 1.0f, 1.0f);          // Top Left Of The Quad (Right)
	glVertex3f( 1.0f,-1.0f, 1.0f);          // Bottom Left Of The Quad (Right)
	glVertex3f( 1.0f,-1.0f,-1.0f);          // Bottom Right Of The Quad (Right)

	glEnd();

	glScalef(1,1,1);
	glPopMatrix();
}

void Utility::drawFace( arrayVec3f* points, arrayVec3i* tris , int mode)
{
	glBegin(GL_TRIANGLES);
	for (int i=0; i<tris->size(); i++)
	{
		Vec3i _tri = (*tris)[i];
		for (int j=0; j<3; j++)
		{
			glVertex3f((*points)[_tri[j]][0], (*points)[_tri[j]][1], (*points)[_tri[j]][2]);
		}
	}
	glEnd();

	glColor3f(0,0,1);
	glBegin(GL_LINES);
	for (int i=0; i<tris->size(); i++)
	{
		Vec3i _tri = (*tris)[i];
		for (int j=0; j<3; j++)
		{
			glVertex3f((*points)[_tri[j]][0], (*points)[_tri[j]][1], (*points)[_tri[j]][2]);
			glVertex3f((*points)[_tri[(j+1)%3]][0], (*points)[_tri[(j+1)%3]][1], (*points)[_tri[(j+1)%3]][2]);
		}
	}
	glEnd();
}

void Utility::drawFace( arrayVec3f* points, arrayVec3i* tris, arrayInt* idxToDraw )
{
	glBegin(GL_TRIANGLES);
	for (int i=0; i<idxToDraw->size(); i++)
	{
		Vec3i _tri = (*tris)[(*idxToDraw)[i]];
		for (int j=0; j<3; j++)
		{
			glVertex3f((*points)[_tri[j]][0], (*points)[_tri[j]][1], (*points)[_tri[j]][2]);
		}
	}
	glEnd();

	glColor3f(0,0,1);
	glBegin(GL_LINES);
	for (int i=0; i<idxToDraw->size(); i++)
	{
		Vec3i _tri = (*tris)[(*idxToDraw)[i]];
		for (int j=0; j<3; j++)
		{
			glVertex3f((*points)[_tri[j]][0], (*points)[_tri[j]][1], (*points)[_tri[j]][2]);
			glVertex3f((*points)[_tri[(j+1)%3]][0], (*points)[_tri[(j+1)%3]][1], (*points)[_tri[(j+1)%3]][2]);
		}
	}
	glEnd();
}

void Utility::urgentLog( char* format, ... )
{
	FILE* ff = fopen("../DebugLog/urgentLog.txt", "a+");
	if(!ff)
		return;

	va_list args;   //  Variable argument list
	int len;        // String length
	int i;          //  Iterator
	char * text;    // Text

	va_start(args, format);
	len = _vscprintf(format, args) + 1;
	text = new char[len];
	vsprintf_s(text, len, format, args);
	va_end(args);

	fprintf(ff, text);
	fprintf(ff, "\n");

	fclose(ff);
}
