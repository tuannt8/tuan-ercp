// CuttingSimulation_GPUView.h : interface of the CCuttingSimulation_GPUView class
//


#pragma once
#include "bound2D.h"
#include "eTool.h"
#include "eLineTool.h"

class CCuttingSimulation_GPUView : public CView
{
protected: // create from serialization only
	CCuttingSimulation_GPUView();
	DECLARE_DYNCREATE(CCuttingSimulation_GPUView)

// Attributes
public:
	CCuttingSimulation_GPUDoc* GetDocument() const;

// Operations
public:

// Overrides
public:
	virtual void OnDraw(CDC* pDC);  // overridden to draw this view
	virtual BOOL PreCreateWindow(CREATESTRUCT& cs);
protected:
	virtual BOOL OnPreparePrinting(CPrintInfo* pInfo);
	virtual void OnBeginPrinting(CDC* pDC, CPrintInfo* pInfo);
	virtual void OnEndPrinting(CDC* pDC, CPrintInfo* pInfo);

// Implementation
public:
	virtual ~CCuttingSimulation_GPUView();
#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

// functions
protected:
	void InitGL();

	//Drawing
	void DrawView();
	void SetupView();
	void UpdateView();

	void LiverInit(int res);
	void KidneyInit(int res);
	void LungInit(int res);
	void BoxInit(int res);
	void SphereInit(int res);
	void TubeInit(int res);
	void TorusInit(int res);

// variables
protected:
	FILE* f;
	CTimeTick TimeTick;

	HDC     m_hDC;
	HGLRC   m_hRC;
	GLuint	base;

	//shader id
	GLuint m_ShaderProg;

	//flag
	bool LEFT_DOWN;
	bool RIGHT_DOWN;
	bool START;
	bool TOOL_START;
	bool SWITCH;

	//window≈©±‚
	int m_WindowHeight;
	int m_WindowWidth;

	//camera
	CCamera m_Cam;

	//mouse position
	Vec3d m_MousePos;
	Vec3d m_PreMousePos;
	Vec3d m_DMousePos;

	CuttingTool m_Tool;
	Meshfree_GPU m_Meshfree;
	MeshfreeCuttingManager m_CuttingManger;
	MeshfreeContactManager m_ContactManger;
	SurfaceCuttingManager m_SurfCuttingManager;

	SurfaceObj m_Surf;
	SurfaceObj m_Box;
	SurfaceObj m_Plane;
	SurfaceObj m_Cylinder;
	SurfaceObj m_Sphere;
	SurfaceObj m_Tube;
	SurfaceObj m_Bowl;
	Vec3f m_Trans;
	int m_ToolPathIdx;

	bound2D boundTest;
	eTool tool;
	eLineTool lineTool;

	BOOL m_displayMode[10];
// Generated message map functions
protected:
	DECLARE_MESSAGE_MAP()
public:
	virtual void OnInitialUpdate();
	afx_msg void OnKeyDown(UINT nChar, UINT nRepCnt, UINT nFlags);
	afx_msg void OnLButtonDown(UINT nFlags, CPoint point);
	afx_msg void OnLButtonUp(UINT nFlags, CPoint point);
	afx_msg void OnRButtonDown(UINT nFlags, CPoint point);
	afx_msg void OnRButtonUp(UINT nFlags, CPoint point);
	afx_msg void OnMouseMove(UINT nFlags, CPoint point);
	afx_msg BOOL OnMouseWheel(UINT nFlags, short zDelta, CPoint pt);
	afx_msg void OnSize(UINT nType, int cx, int cy);
	afx_msg void OnTimer(UINT_PTR nIDEvent);
};

#ifndef _DEBUG  // debug version in CuttingSimulation_GPUView.cpp
inline CCuttingSimulation_GPUDoc* CCuttingSimulation_GPUView::GetDocument() const
   { return reinterpret_cast<CCuttingSimulation_GPUDoc*>(m_pDocument); }
#endif

