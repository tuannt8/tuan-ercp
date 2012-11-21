// CuttingSimulation_GPUView.cpp : implementation of the CCuttingSimulation_GPUView class
//

#include "stdafx.h"
#include <stdio.h>
#include "CuttingSimulation_GPU.h"
#include "MainFrm.h"

#include "CuttingSimulation_GPUDoc.h"
#include "CuttingSimulation_GPUView.h"
#include "simpleRemesh.h"
#include "eSurfaceCutting.h"
#include "meanValueCoord.h"
#include "textureManager.h"
#include "stl.h"

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
	ON_COMMAND(ID_BUTTON_UPDDATE, &CCuttingSimulation_GPUView::OnUpdateDebug)
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
	Collision=false;
	bCollisionMode = FALSE;
	m_ToolPathIdx=0;
	bCut = FALSE;
	bRemesh = FALSE;
	bSmoothBoundary = FALSE;
	bInitTexCoord = FALSE;
	SetTimer(1,20,NULL);
	InitGL();

	int res=15;
	//LiverInit(res);
	//SphereInit(res);

	majorPapillaInit();

	//m_Meshfree.makeMappingMatrix();
	m_tool.init(Vec3f(100,150,200),Vec3f(0,350,200),10);

//	m_lineTool.init(Vec3f(100,150,200),Vec3f(0,350,200));
}

void CCuttingSimulation_GPUView::OnKeyDown(UINT nChar, UINT nRepCnt, UINT nFlags)
{
	// TODO: Add your message handler code here and/or call default
	char lsChar;
	lsChar = char(nChar);
	double var=4;

	double speed = 1;

	if(lsChar=='Q')
	{
		START = !START;
	}
	else if(lsChar=='W')
	{
		bCollisionMode = !bCollisionMode;
	}
	else if(lsChar=='C')
	{
		bCut = TRUE;
	}
	else if(lsChar=='R')
	{
		bRemesh = TRUE;
	}
	else if(lsChar=='X')
	{
		bCut = FALSE;
	}
	else if(lsChar=='V')
	{
		bSmoothBoundary = TRUE;
	}
	else if(lsChar=='Z')
	{
		bInitTexCoord = TRUE;
	}
	else if (lsChar == 'G')
	{
		char* source = ("C:\\Users\\tuan\\Desktop\\testObj.stl");
		char* des = ("C:\\Users\\tuan\\Desktop\\testObj.txt");

		CSTL surObj;
		surObj.ReadData(source);
		surObj.WriteToObj(des);
	}
	else if(lsChar=='O')
	{
		m_lineTool.moveCurrentPoint(Vec3f(speed,0,0));
	}
	else if(lsChar=='P')
	{
		m_lineTool.moveCurrentPoint(Vec3f(-speed,0,0));
	}
	else if (nChar >= 48 && nChar <= 57   )
	{
		m_displayMode[nChar - 48] = ! m_displayMode[nChar - 48];
	}


	switch (nChar)
	{
	case VK_UP:
		m_lineTool.moveCurrentPoint(Vec3f(0,speed,0));
		break;
	case VK_DOWN:
		m_lineTool.moveCurrentPoint(Vec3f(0,-speed,0));
		break;
	case VK_LEFT:
		m_lineTool.moveCurrentPoint(Vec3f(0, 0, speed));
		break;
	case VK_RIGHT:
		m_lineTool.moveCurrentPoint(Vec3f(0, 0, -speed));
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
	float dt=0.03;
	int n=10;



	if(START)
	{
		if (bCollisionMode)
		{
			arrayVec3f* toolPoint = m_lineTool.frontPoint();
			Vec3f P1 = (*toolPoint)[0];
			Vec3f P2 = (*toolPoint)[1];		
			Collision=m_Collision.collisionBtwSurfAndLineSeg_return_Index_V2(m_Meshfree.surfObj(),P1,P2,TOOL_RADIUS,3);
			Response.ComputeforcefromComplianceV10(dt/n,n,&m_Meshfree,&m_Collision);

			m_Meshfree.efgObj()->synchronizeHostAndDevide(SYNC_HOST_TO_DEVICE);
		}
		else
		{
			if (bCut)
			{
				m_Meshfree.efgObj()->getBVH()->updateAABBTreeBottomUp(); // Cost quite a lot of time

				m_CuttingManger.cylinderCutting(&m_Meshfree, m_lineTool.frontPoint(), TOOL_RADIUS);

				simpleRemesh mesh;
				mesh.removeEarTri(m_Meshfree.surfObj(), eSurfaceCutting::cutFaceIdx);
			}
			if (bRemesh)
			{
				bRemesh = false;

				simpleRemesh mesh;
				mesh.remesh(m_Meshfree.surfObj(), eSurfaceCutting::cutFaceIdx, 20);
				arrayInt newPt = mesh.newPointIdx;
				eSurfaceCutting::cutFaceIdx = mesh.newFaceIdx;
				mesh.updateShapeFunc(&m_Meshfree, newPt);
			}
			for (int i=0; i<5; i++)
			{
				m_Meshfree.updatePositionExplicitFree(0.01);
			}

			m_Meshfree.efgObj()->synchronizeHostAndDevide(SYNC_DEVICE_TO_HOST);
		}
	}

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
	textureManager::loadAllTexture();
}
void CCuttingSimulation_GPUView::DrawText()
{
	CString text;
	if (!START)
		text.Format("Static");
	else
	{
		if (bCollisionMode)
			text.Format("Collision mode");
		else
		{
			if (bCut)
				text.Format("Deform mode-Cut");
			else
				text.Format("Free deform mode");
		}
	}


	glPushMatrix();
	glTranslatef(m_Cam.m_Center[0], m_Cam.m_Center[1],m_Cam.m_Center[2]);

	float textPosX = 0.45*(m_WindowWidth/m_WindowHeight)*m_Cam.m_Distance/1.4;
	float textPosY = 0.5*m_Cam.m_Distance/1.4;

	float textPosZ = 0.0*m_Cam.m_Distance;
	Vec3d textPos = Vec3d(textPosX,textPosY,textPosZ);

	Mat3x3f rotateM = m_Cam.m_RotMatrix;
	textPos = rotateM*(textPos);
	glColor3f(1.0,0.7,0.4);
	Utility::printw(textPos[0], textPos[1], textPos[2], text.GetBuffer());

	glPopMatrix();
}
void CCuttingSimulation_GPUView::DrawView()  
{
	GLUquadricObj *qobj = 0;
	qobj = gluNewQuadric();
	glPushAttrib(GL_CURRENT_BIT | GL_ENABLE_BIT | GL_LIGHTING_BIT);
	glPushMatrix();
	SetupView();
	UpdateView();

	textureTest();

	m_lineTool.draw(2);

	DrawText();
	drawDebug();

	if (m_displayMode[1])
		m_Meshfree.drawSurfObj(Vec3f(0.2,0.6,0.2),1);
	if (m_displayMode[2])
		m_Meshfree.drawSurfObj(Vec3f(0.8,0.0,0.3),0);

	if (!m_displayMode[4])
		m_Meshfree.surfObj()->drawBVH();

	if (!m_displayMode[5])
		m_Meshfree.efgObj()->drawEdge();

	if (!m_displayMode[6])
		m_Meshfree.drawEFGObj(Vec3f(1,0,0),2,0);

	if (!m_displayMode[9])
		m_Collision.drawCollisionInfo(Vec3d(0,1,0));

	if (!m_displayMode[8])
		m_lineTool.draw(3);

	glPopMatrix();
	glPopAttrib();
}

void CCuttingSimulation_GPUView::SetupView()
{
	glClearColor(1.0f, 1.0f, 1.0f, 0.5f);

	GLfloat diffuseLight[] = {0.4f,0.4f,0.4f,1.0f};
	GLfloat ambientLight[] = {0.8f,0.8f,0.8f,1.0f};
	GLfloat specular[] = {1.0f, 1.0f, 1.0f, 1.0f};

	glEnable(GL_DEPTH_TEST);                                        
//	glEnable(GL_CULL_FACE);
	//	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	//	glEnable(GL_BLEND);

	glEnable(GL_COLOR_MATERIAL);
	glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);

	glEnable(GL_LIGHTING);   
	glLightfv(GL_LIGHT0, GL_AMBIENT, ambientLight);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuseLight);
	glEnable(GL_LIGHT0);

	//glFrontFace(GL_CCW);
	glFrontFace(GL_CW);
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
	//papilla_highRes
	//liver2194
	m_Surf.readObjData("../data/liver2194.txt");
	m_Surf.constructAABBTree();
	m_Meshfree.loadSurfObj("../data/liver2194.txt");
	m_Meshfree.generateEFGObj(res, false);
	m_Meshfree.connectSurfAndEFG();
	m_Meshfree.boxConstraint(Vec3f(-500,-500,-500), Vec3f(-200,500,500));
	m_Meshfree.initFixedConstraintGPU();
	m_Tool.initEx2();

	m_lineTool.init(Vec3f(100,150,200),Vec3f(0,350,200));
}

void CCuttingSimulation_GPUView::ManInit(int res)
{
	m_Surf.readObjData("../data/body.obj");
	/*m_Surf.scale(1,1,1.3);
	m_Surf.writeObjData("../data/body.obj");*/
	m_Surf.constructAABBTree();
	m_Meshfree.loadSurfObj("../data/body.obj");
	m_Meshfree.generateEFGObj(res, false);
	m_Meshfree.connectSurfAndEFG();
	m_Meshfree.boxConstraint(Vec3f(-500,-500,-500), Vec3f(-200,500,500));
	m_Meshfree.initFixedConstraintGPU();
	m_Tool.initEx2();
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

void CCuttingSimulation_GPUView::CylinderInit(int res)
{
	m_Meshfree.loadSurfObj("../data/cylinder.txt");
	m_Meshfree.generateEFGObj(res, false);
	m_Meshfree.boxConstraint(Vec3f(-300, -300, -300), Vec3f(-30, 300, 300));
	m_Meshfree.connectSurfAndEFG();
	m_Meshfree.initFixedConstraintGPU();
}

void CCuttingSimulation_GPUView::SphereInit(int res)
{
	m_Meshfree.loadSurfObj("../data/sphere.txt");
	m_Meshfree.generateEFGObj(res, true);
	m_Meshfree.boxConstraint(Vec3f(-300, -300, -300), Vec3f(-30, 300, 300));
	m_Meshfree.connectSurfAndEFG();
	m_Meshfree.initFixedConstraintGPU();

	m_lineTool.init(Vec3f(100,150,200),Vec3f(0,350,200));
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

#pragma "New Function for cutting"

void CCuttingSimulation_GPUView::majorPapillaInit()
{
	m_Meshfree.loadSurfObj("../data/papilla_highRes.txt");
	m_Meshfree.generateEFGObj(10, false);
	m_Meshfree.connectSurfAndEFG();
	m_Meshfree.boxConstraint(Vec3f(-150, 100, -300), Vec3f(300, 300, 300));
	m_Meshfree.initFixedConstraintGPU();

// 	//	m_Meshfree.loadSurfObj("../data/mp_super_high.txt");
// 	//	m_Meshfree.loadSurfObj("../data/papilla_hr_1.txt");
// 	//	m_Meshfree.loadSurfObj("../data/mp2_h.txt");
// 	//	m_Meshfree.loadSurfObj("../data/mp_hr2.txt");
// 	m_Meshfree.loadSurfObj("../data/papilla_highRes.txt");
// 	//	m_Meshfree.loadSurfObj("../data/cube_hole_convex.txt");

	//m_lineTool.init(Vec3f(200,0,-200), Vec3f(200,0,200));
	m_lineTool.init(Vec3f(200,0,-200), Vec3f(-100,0,-200));
}

void CCuttingSimulation_GPUView::textureTest()
{
	static arrayVec2f uvPoint;
	static arrayVec3i uvTri;

	if (bSmoothBoundary)  
	{
		//Smooth boundary
		bSmoothBoundary = false;
		arrayVec3f* points = m_Meshfree.surfObj()->point();
		arrayVec3f* point0 = m_Meshfree.surfObj()->point0();
		arrayVec3i* face = m_Meshfree.surfObj()->face();

		arrayInt triArea = eSurfaceCutting::cutFaceIdx;
		eSurfaceCutting surfC;
		VectorFunc func;
		std::vector<arrayInt> allLoops;
		surfC.findBoundaryLoop(m_Meshfree.surfObj()->container(), triArea, allLoops);
		arrayInt loops = allLoops[0];

		int nbPoint = loops.size();
		arrayVec3f newPs;
		arrayVec3f newPs0;
		for (int i=0; i<loops.size(); i++)
		{
			Vec3f pt = (points->at(loops[(i-1+nbPoint)%nbPoint])+points->at(loops[i])+
				points->at(loops[(i+1)%nbPoint]))/3;
			Vec3f pt0 = (point0->at(loops[(i-1+nbPoint)%nbPoint])+point0->at(loops[i])+
				point0->at(loops[(i+1)%nbPoint]))/3;
			newPs.push_back(pt);
			newPs0.push_back(pt0);
		}
		for (int i=0; i<loops.size(); i++)
		{
			points->at(loops[i])=newPs[i];
			point0->at(loops[i])=newPs0[i];
		}
	}

	if (bInitTexCoord)
	{
		bInitTexCoord = false;
		arrayVec3f* points = m_Meshfree.surfObj()->point();
		arrayVec3f* point0 = m_Meshfree.surfObj()->point0();
		arrayVec3i* face = m_Meshfree.surfObj()->face();

		arrayInt triArea = eSurfaceCutting::cutFaceIdx;
		eSurfaceCutting surfC;
		VectorFunc func;

		meanValueCoord meanCoord;

		arrayVec3f pointArea;
		arrayVec3i faceArea;

		for (int i=0; i<triArea.size(); i++)
		{
			faceArea.push_back(face->at(triArea[i]));
		}

		arrayInt allPtIdx;
		for (int i=0; i<triArea.size(); i++)
		{
			for (int j=0; j<3; j++)
			{
				allPtIdx.push_back(face->at(triArea[i])[j]);
			}
		}
		func.arrangeVector(allPtIdx);

		for (int i=0; i<allPtIdx.size(); i++)
		{
			pointArea.push_back(points->at(allPtIdx[i]));
			for (int j=0; j<faceArea.size(); j++)
			{
				for (int k=0; k<3; k++)
				{
					if (faceArea[j][k] == allPtIdx[i])
					{
						faceArea[j][k] = i;
					}
				}
			}
		}

		meanCoord.init(pointArea, faceArea);

		uvTri = faceArea;
		uvPoint = meanCoord.m_planarPoint;
	}

	if (uvTri.size()>0)
	{
		arrayInt triArea = eSurfaceCutting::cutFaceIdx;

		arrayVec3f* points = m_Meshfree.surfObj()->point();
		arrayVec3f* norms = m_Meshfree.surfObj()->pointNormal();
		arrayVec3i* faces = m_Meshfree.surfObj()->face();

		glColor3f(1,1,1);
		glEnable(GL_TEXTURE_2D);
		textureManager::maptexture(CUT_SUR_TEXTURE);
		glBegin(GL_TRIANGLES);
		for (int i=0;i<triArea.size(); i++)
		{
			Vec3i faceTri = faces->at(triArea[i]);
			Vec3i uvF = uvTri[i];

			for (int j=0; j<3; j++)
			{
				Vec3f pt = points->at(faceTri[j]);
				Vec3f ptN = norms->at(faceTri[j]);
				Vec2f ptC = uvPoint[uvF[j]];

				glNormal3f(ptN[0], ptN[1], ptN[2]);
				glTexCoord2f(ptC[0], ptC[1]);
				glVertex3f(pt[0], pt[1], pt[2]);
			}
		}
		glEnd();
		glDisable(GL_TEXTURE_2D);
	}
}


void CCuttingSimulation_GPUView::OnUpdateDebug()
{
	CMainFrame* mainFrm = (CMainFrame*)AfxGetMainWnd();

	mode = mainFrm->m_comboBox.GetCurSel();

	CString text;
	mainFrm->m_editBox.GetWindowText(text);
	index =  atoi( (LPCTSTR) text );
}

void CCuttingSimulation_GPUView::drawDebug()
{
	if (index == -1)
	{
		return;
	}

	switch(mode)
	{
	case 0: // BVH bounding surface triangle
		{
			AABBNode* node = m_Meshfree.surfObj()->getBVH()->findLeafNode(index);
			m_Meshfree.surfObj()->getBVH()->drawBoundingBoxLeafNode(node);

			m_Meshfree.surfObj()->drawFace(index);
			break;
		}
	case 1: // BVH bounding EFG edge
		{
			AABBNode* node = m_Meshfree.efgObj()->getBVH()->findLeafNode(index);
			m_Meshfree.efgObj()->getBVH()->drawBoundingBoxLeafNode(node);

			m_Meshfree.efgObj()->drawEdge(index);
			break;
		}

	}
}
