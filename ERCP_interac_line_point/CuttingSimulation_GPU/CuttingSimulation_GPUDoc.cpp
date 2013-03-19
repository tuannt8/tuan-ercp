// CuttingSimulation_GPUDoc.cpp : implementation of the CCuttingSimulation_GPUDoc class
//

#include "stdafx.h"
#include "CuttingSimulation_GPU.h"

#include "CuttingSimulation_GPUDoc.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// CCuttingSimulation_GPUDoc

IMPLEMENT_DYNCREATE(CCuttingSimulation_GPUDoc, CDocument)

BEGIN_MESSAGE_MAP(CCuttingSimulation_GPUDoc, CDocument)
END_MESSAGE_MAP()


// CCuttingSimulation_GPUDoc construction/destruction

CCuttingSimulation_GPUDoc::CCuttingSimulation_GPUDoc()
{
	// TODO: add one-time construction code here

}

CCuttingSimulation_GPUDoc::~CCuttingSimulation_GPUDoc()
{
}

BOOL CCuttingSimulation_GPUDoc::OnNewDocument()
{
	if (!CDocument::OnNewDocument())
		return FALSE;

	// TODO: add reinitialization code here
	// (SDI documents will reuse this document)

	return TRUE;
}




// CCuttingSimulation_GPUDoc serialization

void CCuttingSimulation_GPUDoc::Serialize(CArchive& ar)
{
	if (ar.IsStoring())
	{
		// TODO: add storing code here
	}
	else
	{
		// TODO: add loading code here
	}
}


// CCuttingSimulation_GPUDoc diagnostics

#ifdef _DEBUG
void CCuttingSimulation_GPUDoc::AssertValid() const
{
	CDocument::AssertValid();
}

void CCuttingSimulation_GPUDoc::Dump(CDumpContext& dc) const
{
	CDocument::Dump(dc);
}
#endif //_DEBUG


// CCuttingSimulation_GPUDoc commands
