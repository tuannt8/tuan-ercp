#ifndef CUTTING_MANAGER
#define CUTTING_MANAGER

#include "DataTypes/Define.h"
#include "DataTypes/Vec.h"
#include "DataTypes/Mat.h"
#include "Modules/CollisionManager.h"
#include "Graphics/CuttingTool.h"
#include "Modules/EFG_CUDA_RUNTIME.h"
#include "Modules/TimeTick.h"
#include <afxwin.h>
#include <GL/glu.h>
#include <GL/gl.h>
#include <vector>

class CuttingManager
{
public:
	CuttingManager(void);
	~CuttingManager(void);

//functions
public:
	void cutting(EFG_CUDA_RUNTIME* obj, CuttingTool* tool);
	void cuttingWithCell(EFG_CUDA_RUNTIME* obj, CuttingTool* tool);
};

#endif