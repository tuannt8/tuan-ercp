// MainFrm.cpp : implementation of the CMainFrame class
//

#include "stdafx.h"
#include "CuttingSimulation_GPU.h"

#include "MainFrm.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// CMainFrame

IMPLEMENT_DYNCREATE(CMainFrame, CFrameWnd)

BEGIN_MESSAGE_MAP(CMainFrame, CFrameWnd)
	ON_WM_CREATE()
END_MESSAGE_MAP()

static UINT indicators[] =
{
	ID_SEPARATOR,           // status line indicator
	ID_INDICATOR_CAPS,
	ID_INDICATOR_NUM,
	ID_INDICATOR_SCRL,
};


// CMainFrame construction/destruction

CMainFrame::CMainFrame()
{
	// TODO: add member initialization code here
}

CMainFrame::~CMainFrame()
{
}


int CMainFrame::OnCreate(LPCREATESTRUCT lpCreateStruct)
{
	if (CFrameWnd::OnCreate(lpCreateStruct) == -1)
		return -1;
	
	if (!m_wndToolBar.CreateEx(this, TBSTYLE_FLAT, WS_CHILD | WS_VISIBLE | CBRS_TOP
		| CBRS_GRIPPER | CBRS_TOOLTIPS | CBRS_FLYBY | CBRS_SIZE_DYNAMIC) ||
		!m_wndToolBar.LoadToolBar(IDR_MAINFRAME))
	{
		TRACE0("Failed to create toolbar\n");
		return -1;      // fail to create
	}

	if (!m_wndStatusBar.Create(this) ||
		!m_wndStatusBar.SetIndicators(indicators,
		  sizeof(indicators)/sizeof(UINT)))
	{
		TRACE0("Failed to create status bar\n");
		return -1;      // fail to create
	}

	// TODO: Delete these three lines if you don't want the toolbar to be dockable

	{
		CRect rect;

		int nIndex = m_wndToolBar.GetToolBarCtrl().CommandToIndex(ID_COMBO);
		m_wndToolBar.SetButtonInfo(nIndex, ID_COMBO, TBBS_SEPARATOR, 205);
		m_wndToolBar.GetToolBarCtrl().GetItemRect(nIndex, &rect);
		rect.top = 1;
		rect.bottom = rect.top + 250 /*drop height*/;

		if(!m_comboBox.Create(CBS_DROPDOWNLIST | CBS_SORT | WS_VISIBLE |
			WS_TABSTOP | WS_VSCROLL, rect, &m_wndToolBar, ID_COMBO))
		{
			TRACE(_T("Failed to create combo-box\n"));
			return FALSE;
		}


		m_comboBox.AddString("1. Surface Tri bounding box");
		m_comboBox.AddString("2. EFG edge bounding box");
		m_comboBox.AddString("3. Neighbor of surf point");

		m_comboBox.SetCurSel(0);
	}
	{
		CRect rect;

		int nIndex = m_wndToolBar.GetToolBarCtrl().CommandToIndex(ID_TEXT_BOX);
		m_wndToolBar.SetButtonInfo(nIndex, ID_TEXT_BOX, TBBS_SEPARATOR, 50);
		m_wndToolBar.GetToolBarCtrl().GetItemRect(nIndex, &rect);
		rect.top = 1;
		rect.bottom = rect.top + 250 /*drop height*/;

		if (!m_editBox.Create(ES_MULTILINE | WS_CHILD | WS_VISIBLE | WS_TABSTOP | WS_BORDER,
			rect, &m_wndToolBar, ID_TEXT_BOX))
		{
			TRACE(_T("Failed to create combo-box\n"));
			return FALSE;
		}
		m_editBox.SetWindowText(_T("-1"));
	}

	m_wndToolBar.EnableDocking(CBRS_ALIGN_ANY);
	EnableDocking(CBRS_ALIGN_ANY);
	DockControlBar(&m_wndToolBar);

	return 0;
}

BOOL CMainFrame::PreCreateWindow(CREATESTRUCT& cs)
{
	if( !CFrameWnd::PreCreateWindow(cs) )
		return FALSE;
	// TODO: Modify the Window class or styles here by modifying
	//  the CREATESTRUCT cs

	return TRUE;
}


// CMainFrame diagnostics

#ifdef _DEBUG
void CMainFrame::AssertValid() const
{
	CFrameWnd::AssertValid();
}

void CMainFrame::Dump(CDumpContext& dc) const
{
	CFrameWnd::Dump(dc);
}

#endif //_DEBUG


// CMainFrame message handlers



