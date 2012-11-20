#pragma once
#include "stdafx.h"
#include "gl\GLAux.h"

#define CUT_SUR_TEXTURE 0
#define LIGHTTING_TEXTURE 1
#define LIGHTTING_STREAK 2
#define LIGHTTING_BIG_GLOW 3
#define TEXTURE_CATHETER 4

class textureManager
{
public:
	textureManager(void);
	~textureManager(void);

	static void loadAllTexture();
	static void deleteTexture();

	static void maptexture(int index);

	static AUX_RGBImageRec *LoadBMP(char *Filename);
	static int LoadGLTextures(char* path, int index);

	static GLuint texture[10];
};

extern textureManager texManager;
