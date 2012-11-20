// CuttingSimulation_GPU.h : main header file for the CuttingSimulation_GPU application
//
#pragma once

#ifndef __AFXWIN_H__
	#error "include 'stdafx.h' before including this file for PCH"
#endif

#include "resource.h"       // main symbols
#include "Graphics/OpenGL.h"
#include "Graphics/Camera.h"
#include "DataTypes/Vec.h"
#include "DataTypes/Mat.h"
#include "DataTypes/matrix.h"
#include "Modules/EFG_CUDA_RUNTIME.h"
#include "Modules/TimeTick.h"
#include "Graphics/SurfaceObj.h"
#include "Modules/Meshfree_CPU.h"
#include "Modules/CuttingTool.h"
#include "Modules/Meshfree_GPU.h"
#include "Modules/MeshfreeCuttingManager.h"
#include "Modules/MeshfreeContactManager.h"
#include "Modules/CollisionManager.h"
#include "Cavatar.h"
#include "CollisionResponse.h"
#include "armadillo"


using namespace arma;
using namespace std;


// CCuttingSimulation_GPUApp:
// See CuttingSimulation_GPU.cpp for the implementation of this class
//

class CCuttingSimulation_GPUApp : public CWinApp
{
public:
	CCuttingSimulation_GPUApp();


// Overrides
public:
	virtual BOOL InitInstance();

// Implementation
	afx_msg void OnAppAbout();
	DECLARE_MESSAGE_MAP()
};

extern CCuttingSimulation_GPUApp theApp;