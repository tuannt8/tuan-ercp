#include "StdAfx.h"
#include "textureManager.h"

textureManager::textureManager(void)
{
	static bool loaded = false;
	if (!loaded)
	{
		loaded = true;
		loadAllTexture();
	}
}

textureManager::~textureManager(void)
{
}


GLuint textureManager::texture[10];



AUX_RGBImageRec *textureManager::LoadBMP(char *Filename)				// Loads A Bitmap Image
{
	FILE *File=NULL;									// File Handle

	if (!Filename)										// Make Sure A Filename Was Given
	{
		return NULL;									// If Not Return NULL
	}

	File=fopen(Filename,"r");							// Check To See If The File Exists

	if (File)											// Does The File Exist?
	{
		fclose(File);									// Close The Handle
		return auxDIBImageLoad(Filename);				// Load The Bitmap And Return A Pointer
	}

	return NULL;										// If Load Failed Return NULL
}

int textureManager::LoadGLTextures(char* path, int index)									// Load Bitmaps And Convert To Textures
{
	int Status=FALSE;									// Status Indicator

	AUX_RGBImageRec *TextureImage[1];					// Create Storage Space For The Texture

	memset(TextureImage,0,sizeof(void *)*1);           	// Set The Pointer To NULL


	// Load The Bitmap, Check For Errors, If Bitmap's Not Found Quit
	if (TextureImage[0]=LoadBMP(path))
	{
		Status=TRUE;									// Set The Status To TRUE
		glGenTextures(1, &texture[index]);					// Create The Texture

		// Typical Texture Generation Using Data From The Bitmap
		glBindTexture(GL_TEXTURE_2D, texture[index]);
		glTexImage2D(GL_TEXTURE_2D, 0, 3, TextureImage[0]->sizeX, TextureImage[0]->sizeY, 0, GL_RGB, GL_UNSIGNED_BYTE, TextureImage[0]->data);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
	}

	if (TextureImage[0])									// If Texture Exists
	{
		if (TextureImage[0]->data)							// If Texture Image Exists
		{
			free(TextureImage[0]->data);					// Free The Texture Image Memory
		}

		free(TextureImage[0]);								// Free The Image Structure
	}

	return Status;										// Return The Status
}

void textureManager::loadAllTexture()
{
	LoadGLTextures("../data/textureTest.bmp", CUT_SUR_TEXTURE);
	LoadGLTextures("../image/HardGlow2.bmp", LIGHTTING_TEXTURE);
	LoadGLTextures("../image/Streaks4.bmp", LIGHTTING_STREAK);
	LoadGLTextures("../image/BigGlow3.bmp", LIGHTTING_BIG_GLOW);
	LoadGLTextures("../image/catheter.bmp", TEXTURE_CATHETER);
}

void textureManager::deleteTexture()
{

}

void textureManager::maptexture( int index )
{
//	glActiveTextureARB;
//	glActiveTexture;
	glBindTexture(GL_TEXTURE_2D, texture[index]);
}
