#pragma once
#include "stdafx.h"
#include "../include/Modules/EFG_CUDA_RUNTIME.h"
#include "armadillo"

typedef enum _fileType
{
	FILE_TYPE_ANY			= 0x0000,
	FILE_TYPE_STL			= 0x0001, //STL
	FILE_TYPE_TXT			= 0x1000

}fileType;

namespace Utility
{
	// File
	void Log(EFG_CUDA_RUNTIME* obj);
	void Log(float* arr, int rowSize, int colSize = 3);

	void LogMatrix(arma::mat* A, char* fileName="matrix_log.txt");

	void exportToSTL(char* path, arrayVec3f* point, arrayVec3i* face);
	// Draw
	void printw (float x, float y, float z, char* format, ...);
	void drawPlane( double x, double y, double z, double large, int res , Vec3f direction = Vec3f(0,1,0));
	// util
	void okMessageBox(char* message);
};
