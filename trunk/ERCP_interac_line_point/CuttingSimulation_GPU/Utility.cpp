#include "stdafx.h"
#include "Utility.h"
#include "GL/GL.h"
#include "GL/glut.h"

extern "C" int matrix_mul_cuda( float* h_C, float* h_A, float* h_B, int ra, int ca, int cb );


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
	FILE* f = fopen("../DebugLog/matrix.txt", "w");
	fprintf(f, "Number of element: %dx%d\n", rowSize, colSize);
	for (int i = 0; i < rowSize; i++)
	{
		for (int j=0; j<colSize; j++)
		{
			fprintf(f, "%lf\t", arr[i*colSize+j]);
		}
		fprintf(f, "\n");
	}
	fclose(f);
}

void Utility::Log( Vec3d* arr, int size , int mode)
{
	FILE* f = fopen("../DebugLog/vectorArray.txt", mode==0?"w":"a");
	fprintf(f, "Number of element: %d\n", size);
	for (int i = 0; i < size; i++)
	{
		for (int j=0; j<3; j++)
		{
			fprintf(f, "%lf\t", arr[i][j]);
		}
		fprintf(f, "\n");
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

void Utility::LogMatrix( arma::mat* A, char* fileName/*="matrix_log.txt"*/ )
{
	CString path;
	path.Format("../DebugLog/%s", (char*)fileName);
	FILE* temp = fopen(path.GetBuffer(), ("w"));
	if (temp)
	{
		fprintf(temp, "Log matrix: %dx%d\n", A->n_rows, A->n_cols);
		for (int i=0; i<A->n_rows; i++)
		{
			for (int j=0; j<A->n_cols; j++)
			{
				fprintf(temp, "%.08f\t", A->at(i,j));
			}
			fprintf(temp, "\n");
		}
		fclose(temp);
	}
}

arma::mat Utility::multiplyMat( arma::mat*mA, arma::mat*mB )
{
	int cuda_size = 16;

	int row_A = mA->n_rows;
	int col_A = mA->n_cols;
	int row_B = mB->n_rows;
	int col_B = mB->n_cols;

	// to 16
	int row_A_k = (row_A/cuda_size +1)*cuda_size;
	int col_A_k = (col_A/cuda_size +1)*cuda_size;
	int row_B_k = (row_B/cuda_size +1)*cuda_size;
	int col_B_k = (col_B/cuda_size +1)*cuda_size;

	float *A, *B, *C;
	A = new float[row_A_k*col_A_k];
	B = new float[row_B_k*col_B_k];
	C = new float[row_A_k*col_B_k];

	for (int i=0; i<row_A_k; i++)
	{
		for (int j=0; j<col_A_k; j++)
		{
			if (i<row_A && j<col_A)
				A[i*col_A_k + j] = mA->at(i,j);
			else
				A[i*col_A_k + j] = 0;
		}
	}

	for (int i=0; i<row_B_k; i++)
	{
		for (int j=0; j<col_B_k; j++)
		{
			if (i<row_B && j<col_B)
				B[i*col_B_k + j] = mB->at(i,j);
			else
				B[i*col_B_k + j] = 0;
		}
	}

	matrix_mul_cuda(C,A,B,row_A_k,col_A_k,col_B_k);

	arma:: mat matC;
	matC.set_size(row_A, col_B);
	for (int i=0; i<row_A; i++)
	{
		for (int j=0; j<col_B; j++)
		{
			matC(i,j) = C[i*col_B_k+j];
		}
	}

	delete A;
	delete B;
	delete C;

	return matC;
}

