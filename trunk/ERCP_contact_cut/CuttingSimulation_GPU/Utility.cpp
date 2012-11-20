#include "stdafx.h"
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

void Utility::Log( EFG_CUDA_RUNTIME* obj )
{
	FILE* f = fopen("../DebugLog/EFG_log.txt", "w");
	arrayVec3f *nodePos = obj->nodePosVec();
	for (int i = 0; i < nodePos->size(); i++)
	{
		fprintf(f, "%lf %lf %lf\n", (double)((*nodePos)[i][0]), (float)((*nodePos)[i][1]), (float)((*nodePos)[i][2]));
	}
	fclose(f);
}

void Utility::Log( float* arr, int rowSize, int colSize /*= 3*/ )
{
	FILE* f = fopen("../DebugLog/floatArray.txt", "w");
	fprintf(f, "Number of element: %d\n", rowSize);
	for (int i = 0; i < rowSize; i++)
	{
		fprintf(f, "%lf %lf %lf\n", arr[i*3], arr[i*3 + 1], arr[i*3 + 2]);
	}
	fclose(f);
}

void Utility::okMessageBox( char* message )
{
//	AfxMessageBox(message, MB_OK|MB_ICONINFORMATION);
}

void Utility::drawPlane( double x, double y, double z, double large, int res , 
						Vec3f direction /*= Vec3f(0,1,0)*/)
{
	double gap = large / res;
	glPushMatrix();
	glTranslatef(x,y,z);

	double alfa = (double)atan2(direction[1], direction[0])*180.0/PI;
	double beta = (double)acos(direction[2]/direction.norm())*180.0/PI;

	glRotated(alfa, 0, 0, 1);
	glRotated(beta, 0, 1, 0);

	glBegin(GL_LINES);
	for (int i = -res; i <= res; i++) 
	{
		glVertex3f(i*gap, -large, 0.0);// y
		glVertex3f(i*gap, large, 0.0);

		glVertex3f(-large, i*gap, 0.0);// x
		glVertex3f(large, i*gap, 0.0);
	}
	glEnd();
	glPopMatrix();
}

void Utility::exportToSTL( char* path, arrayVec3f* point, arrayVec3i* face )
{
	FILE* f = fopen(path, "w");
	if (f)
	{
		fprintf(f, "solid\n");
		for (int i = 0; i< face->size(); i++)
		{
			fprintf(f, " facet normal 1 1 1\n");
			fprintf(f, "  outer loop\n");
			Vec3i faceP = (*face)[i];
			for (int j = 0; j < 3; j++)
			{
				Vec3f curP = (*point)[faceP[j]];
				fprintf(f, "   vertex %f %f %f\n", curP[0], curP[1], curP[2]);
			}
			fprintf(f, "  endloop\n");
			fprintf(f, " endfacet\n");
		}
		fprintf(f, "endsolid\n");
		fclose(f);
	}
}
