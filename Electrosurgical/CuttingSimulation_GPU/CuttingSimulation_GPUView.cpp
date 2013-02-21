// CuttingSimulation_GPUView.cpp : implementation of the CCuttingSimulation_GPUView class
//

#include "stdafx.h"
#include "CuttingSimulation_GPU.h"

#include "CuttingSimulation_GPUDoc.h"
#include "CuttingSimulation_GPUView.h"

#include "stl.h"
#include "voxel.h"
#include "Utility.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// CCuttingSimulation_GPUView

IMPLEMENT_DYNCREATE(CCuttingSimulation_GPUView, CView)

BEGIN_MESSAGE_MAP(CCuttingSimulation_GPUView, CView)
	// Standard printing commands
	ON_COMMAND(ID_FILE_PRINT, &CView::OnFilePrint)
	ON_COMMAND(ID_FILE_PRINT_DIRECT, &CView::OnFilePrint)
	ON_COMMAND(ID_FILE_PRINT_PREVIEW, &CView::OnFilePrintPreview)
	ON_WM_KEYDOWN()
	ON_WM_LBUTTONDOWN()
	ON_WM_LBUTTONUP()
	ON_WM_RBUTTONDOWN()
	ON_WM_RBUTTONUP()
	ON_WM_MOUSEMOVE()
	ON_WM_MOUSEWHEEL()
	ON_WM_SIZE()
	ON_WM_TIMER()
END_MESSAGE_MAP()

// CCuttingSimulation_GPUView construction/destruction

CCuttingSimulation_GPUView::CCuttingSimulation_GPUView()
{
	// TODO: add construction code here
	f=fopen("Time.txt","w");
}

CCuttingSimulation_GPUView::~CCuttingSimulation_GPUView()
{
	fclose(f);
}

BOOL CCuttingSimulation_GPUView::PreCreateWindow(CREATESTRUCT& cs)
{
	// TODO: Modify the Window class or styles here by modifying
	//  the CREATESTRUCT cs

	return CView::PreCreateWindow(cs);
}

// CCuttingSimulation_GPUView drawing

void CCuttingSimulation_GPUView::OnDraw(CDC* /*pDC*/)
{
	CCuttingSimulation_GPUDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	if (!pDoc)
		return;

	// TODO: add draw code for native data here
	wglMakeCurrent(m_hDC,m_hRC);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	DrawView();
	SwapBuffers(m_hDC);
}


// CCuttingSimulation_GPUView printing

BOOL CCuttingSimulation_GPUView::OnPreparePrinting(CPrintInfo* pInfo)
{
	// default preparation
	return DoPreparePrinting(pInfo);
}

void CCuttingSimulation_GPUView::OnBeginPrinting(CDC* /*pDC*/, CPrintInfo* /*pInfo*/)
{
	// TODO: add extra initialization before printing
}

void CCuttingSimulation_GPUView::OnEndPrinting(CDC* /*pDC*/, CPrintInfo* /*pInfo*/)
{
	// TODO: add cleanup after printing
}


// CCuttingSimulation_GPUView diagnostics

#ifdef _DEBUG
void CCuttingSimulation_GPUView::AssertValid() const
{
	CView::AssertValid();
}

void CCuttingSimulation_GPUView::Dump(CDumpContext& dc) const
{
	CView::Dump(dc);
}

CCuttingSimulation_GPUDoc* CCuttingSimulation_GPUView::GetDocument() const // non-debug version is inline
{
	ASSERT(m_pDocument->IsKindOf(RUNTIME_CLASS(CCuttingSimulation_GPUDoc)));
	return (CCuttingSimulation_GPUDoc*)m_pDocument;
}
#endif //_DEBUG


// CCuttingSimulation_GPUView message handlers

void CCuttingSimulation_GPUView::OnInitialUpdate()
{
	CView::OnInitialUpdate();

	// TODO: Add your specialized code here and/or call the base class
	LEFT_DOWN=false;
	RIGHT_DOWN=false;
	START=false;
	TOOL_START=false;
	SWITCH=true;
	m_ToolPathIdx=0;
	SetTimer(1,30,NULL);
	InitGL();

	int res=25;
	LiverInit(res);
}

void CCuttingSimulation_GPUView::OnKeyDown(UINT nChar, UINT nRepCnt, UINT nFlags)
{
	// TODO: Add your message handler code here and/or call default
	char lsChar;
	lsChar = char(nChar);

	if(lsChar=='Q')
	{
		if(START)
			START=false;
		else
			START=true;
	}
	if (lsChar == 'C')
	{
		lineTool.cut8(m_Meshfree.surfObj());
	}
	if (lsChar == 'R')
	{
		lineTool.stepDebug8();
	}
	if (lsChar == 'E')
	{
		lineTool.resetPath();
			
	}
	else if (lsChar == 'G')
	{
		char* source = ("C:\\Users\\tuan\\Desktop\\needle.stl");
		char* des = ("C:\\Users\\tuan\\Desktop\\needle.txt");

		CSTL surObj;
		if (surObj.ReadData(source))
		{
			surObj.WriteToObj(des);
			MessageBox("Convert successful");
		}
		else
			MessageBox("No file to stl file to convert!!!");
	}
	if (lsChar == 'S')
	{
		lineTool.smoothBoundary();
	}
	if (lsChar == 'T')
	{
		CTimeTick timeTick;
		timeTick.SetStart();
		Vec3f test = lineTool.invertMappingFunction(&m_Meshfree, Vec3f(0,100,0));
		timeTick.SetEnd();
		Utility::urgentLog("Time: %lf", timeTick.GetTick());
	}
	else if (nChar >= 48 && nChar <= 57   )
	{
		m_displayMode[nChar - 48] = ! m_displayMode[nChar - 48];
	}


	float speed = 5;
	if (lsChar == 'M')
	{
		lineTool.move(Vec3f(0,speed,0));
	}
	else if (lsChar == 'N')
	{
		lineTool.move(Vec3f(0,-speed,0));
	}
	switch (nChar)
	{
	case VK_UP:
		tool.move(Vec3f(speed,0,0));
		break;
	case VK_DOWN:
		tool.move(Vec3f(-speed,0,0));
		break;
	case VK_LEFT:
		tool.move(Vec3f(0, 0, speed));
		break;
	case VK_RIGHT:
		tool.move(Vec3f(0, 0, -speed));
		break;
	}

	switch (nChar)
	{
	case VK_UP:
		lineTool.move(Vec3f(speed,0,0));
		break;
	case VK_DOWN:
		lineTool.move(Vec3f(-speed,0,0));
		break;
	case VK_LEFT:
		lineTool.move(Vec3f(0, 0, speed));
		break;
	case VK_RIGHT:
		lineTool.move(Vec3f(0, 0, -speed));
		break;
	}

	CView::OnKeyDown(nChar, nRepCnt, nFlags);
}

void CCuttingSimulation_GPUView::OnLButtonDown(UINT nFlags, CPoint point)
{
	// TODO: Add your message handler code here and/or call default
	LEFT_DOWN=true;
	CView::OnLButtonDown(nFlags, point);
}

void CCuttingSimulation_GPUView::OnLButtonUp(UINT nFlags, CPoint point)
{
	// TODO: Add your message handler code here and/or call default
	LEFT_DOWN=false;
	CView::OnLButtonUp(nFlags, point);
}

void CCuttingSimulation_GPUView::OnMouseMove(UINT nFlags, CPoint point)
{
	// TODO: Add your message handler code here and/or call default
	m_MousePos[0]=point.x;
	m_MousePos[1]=-point.y;
	m_DMousePos=m_MousePos-m_PreMousePos;

	if(LEFT_DOWN)
		m_Cam.RotCamPos(m_DMousePos);
	if(RIGHT_DOWN)
		m_Cam.MoveCamPos(m_DMousePos);
	m_PreMousePos=m_MousePos;
	CView::OnMouseMove(nFlags, point);
}

BOOL CCuttingSimulation_GPUView::OnMouseWheel(UINT nFlags, short zDelta, CPoint pt)
{
	// TODO: Add your message handler code here and/or call default
	Vec3d temp;
	m_Cam.m_Distance+=zDelta*0.1;
	m_Cam.RotCamPos(temp);
	return CView::OnMouseWheel(nFlags, zDelta, pt);
}

void CCuttingSimulation_GPUView::OnRButtonDown(UINT nFlags, CPoint point)
{
	// TODO: Add your message handler code here and/or call default
	RIGHT_DOWN=true;
	CView::OnRButtonDown(nFlags, point);
}

void CCuttingSimulation_GPUView::OnRButtonUp(UINT nFlags, CPoint point)
{
	// TODO: Add your message handler code here and/or call default
	RIGHT_DOWN=false;
	CView::OnRButtonUp(nFlags, point);
}

void CCuttingSimulation_GPUView::OnSize(UINT nType, int cx, int cy)
{
	CView::OnSize(nType, cx, cy);

	// TODO: Add your message handler code here
	CSize size(cx,cy);
	m_WindowHeight=size.cy;
	m_WindowWidth=size.cx;
}

void CCuttingSimulation_GPUView::OnTimer(UINT_PTR nIDEvent)
{
	// TODO: Add your message handler code here and/or call default
	if(START)
		m_Meshfree.updatePositionExplicit(0.003, 10);
	InvalidateRect(NULL, FALSE);
	CView::OnTimer(nIDEvent);
}

void CCuttingSimulation_GPUView::InitGL()
{
	COpenGL Initgl;

	//Initgl에 windows handle을 넘겨준다
	Initgl.SetHWND(m_hWnd);
	Initgl.SetupPixelFormat();
	base=Initgl.base;

	m_hDC=Initgl.m_hDC;
	m_hRC=Initgl.m_hRC;

// 	Initgl.SetupShader("../shader/local");
// 	m_ShaderProg=Initgl.GetProgLog();
// 	glUniform3f(glGetUniformLocation(m_ShaderProg, "LightPosition"), 1000.0, 1000.0, 1000.0);
}

void CCuttingSimulation_GPUView::DrawView()
{
	GLUquadricObj *qobj = 0;
	qobj = gluNewQuadric();
	glPushAttrib(GL_CURRENT_BIT | GL_ENABLE_BIT | GL_LIGHTING_BIT);
	glPushMatrix();
	SetupView();
	UpdateView();

//	m_Meshfree.drawSurfObj(Vec3f(1,0,0),0);
//	boundTest.draw(1);

	if (m_displayMode[1])
		m_Meshfree.surfObj()->drawObject(Vec3f(1,0,0));
	
	if (m_displayMode[2])
		m_Meshfree.surfObj()->drawWireFrame(Vec3f(0,0,1));

	if (!m_displayMode[4])
	{
		m_Meshfree.surfObj()->drawPointIdx();
	}
	if (m_displayMode[5])
	{
		m_Meshfree.drawEFGObj(Vec3f(1,0,0), 2, 0);
		m_Meshfree.efgObj()->drawEdge();
	}

	if (m_displayMode[9])
	{
		lineTool.draw(0);
	}
	if(m_displayMode[8])
		lineTool.draw(1);

// 	if (m_displayMode[9])
// 		tool.draw();
// 	if (m_displayMode[8])
// 		tool.drawCutInfo(m_Meshfree.surfObj());
// 	if (!m_displayMode[7])
// 		tool.draw(1);
// 
// 	if (m_displayMode[6])
// 		tool.drawCutInfoDetail(m_Meshfree.surfObj(), 1);

	glPopMatrix();
	glPopAttrib();
}

void CCuttingSimulation_GPUView::SetupView()
{
	if (m_displayMode[0])
		glClearColor(0,0,0,0);
	else
		glClearColor(1,1,1,1);

	GLfloat diffuseLight[] = {0.4f,0.4f,0.4f,1.0f};
	GLfloat ambientLight[] = {0.2f,0.2f,0.2f,1.0f};
	GLfloat specular[] = {1.0f, 1.0f, 1.0f, 1.0f};

	glEnable(GL_DEPTH_TEST);                                        
	//glEnable(GL_CULL_FACE);
	//	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	//	glEnable(GL_BLEND);

	glEnable(GL_COLOR_MATERIAL);
	glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);

	glEnable(GL_LIGHTING);   
	glLightfv(GL_LIGHT0, GL_AMBIENT, ambientLight);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuseLight);
	glEnable(GL_LIGHT0);

	glFrontFace(GL_CCW);
	//glFrontFace(GL_CW);
	glShadeModel(GL_SMOOTH); 
	glPolygonMode(GL_FRONT, GL_FILL);
}

void CCuttingSimulation_GPUView::UpdateView()
{
	glViewport(0,0,m_WindowWidth,m_WindowHeight);
	float fovy=45;
	float aspect=float(m_WindowWidth)/float(m_WindowHeight);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	gluPerspective(fovy, aspect, 1, 10000);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(m_Cam.m_Pos[0],m_Cam.m_Pos[1],m_Cam.m_Pos[2],
		m_Cam.m_Center[0],m_Cam.m_Center[1],m_Cam.m_Center[2],
		m_Cam.m_Up[0],m_Cam.m_Up[1],m_Cam.m_Up[2]);
}

void CCuttingSimulation_GPUView::LiverInit(int res)
{
// 	m_Surf.readObjData("../data/liver2194.txt");
// 	m_Surf.constructAABBTree();

	m_Meshfree.loadSurfObj("../data/liver2194.txt");
	m_Meshfree.generateEFGObj(res, false);
	m_Meshfree.connectSurfAndEFG();
	m_Meshfree.boxConstraint(Vec3f(-500,-500,-500), Vec3f(-200,500,500));
	m_Meshfree.initFixedConstraintGPU();
// 	m_Tool.initEx2();
}
void CCuttingSimulation_GPUView::KidneyInit(int res)
{
}
void CCuttingSimulation_GPUView::LungInit(int res)
{
}

void CCuttingSimulation_GPUView::BoxInit(int res)
{
	m_Meshfree.loadSurfObj("../data/box3752.txt");
	m_Meshfree.generateEFGObj(res, false);
	m_Meshfree.boxConstraint(Vec3f(-300, -300, -300), Vec3f(-30, 300, 300));
	m_Meshfree.connectSurfAndEFG();
	m_Meshfree.initFixedConstraintGPU();
}

void CCuttingSimulation_GPUView::SphereInit(int res)
{
	m_Meshfree.loadSurfObj("../data/Sphere482.txt");
	m_Meshfree.generateEFGObj(res, true);
	m_Meshfree.boxConstraint(Vec3f(-300, -300, -300), Vec3f(-30, 300, 300));
	m_Meshfree.connectSurfAndEFG();
	m_Meshfree.initFixedConstraintGPU();
}

void CCuttingSimulation_GPUView::TubeInit(int res)
{
	m_Surf.readObjData("../data/tube.txt");
	m_Surf.constructAABBTree();
	m_Meshfree.loadSurfObj("../data/tube.txt");
	m_Meshfree.generateEFGObj(res, true);
	m_Meshfree.connectSurfAndEFG();
	//m_Meshfree.boxConstraint(Vec3f(-500,-500,-500), Vec3f(-100,500,500));
	//m_Meshfree.initFixedConstraintGPU();
	m_Tool.initEx1();
}

void CCuttingSimulation_GPUView::TorusInit(int res)
{
	m_Surf.readObjData("../data/torus.txt");
	m_Surf.constructAABBTree();
	m_Meshfree.loadSurfObj("../data/torus.txt");
	m_Meshfree.generateEFGObj(res, false);
	m_Meshfree.connectSurfAndEFG();
	m_Meshfree.boxConstraint(Vec3f(-500,100,-500), Vec3f(500,500,500));
	m_Meshfree.initFixedConstraintGPU();
	m_Tool.initEx1();
}