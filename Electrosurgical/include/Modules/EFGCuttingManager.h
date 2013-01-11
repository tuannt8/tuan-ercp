#ifndef EFG_CUTTING_MANAGER
#define EFG_CUTTING_MANAGER

#include "DataTypes/Define.h"
#include "DataTypes/Vec.h"
#include "DataTypes/Mat.h"
#include "Modules/CollisionManager.h"
#include "Modules/EFG_CPU.h"
#include "Modules/EFG_CUDA_RUNTIME.h"
#include "Modules/TimeTick.h"
#include <afxwin.h>
#include <GL/glu.h>
#include <GL/gl.h>
#include <vector>

class EFGCuttingManager
{

public:
	EFGCuttingManager(void);
	~EFGCuttingManager(void);

//functions
public:
	void cutting(EFG_CPU* obj, std::vector<Vec3f>* cutPoint, std::vector<Vec3i>* cutSurf);
	void cutting(EFG_CUDA_RUNTIME* obj, std::vector<Vec3f>* cutPoint, std::vector<Vec3i>* cutSurf);
};

#endif