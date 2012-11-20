// CuttingSimulation_GPUDoc.h : interface of the CCuttingSimulation_GPUDoc class
//


#pragma once


class CCuttingSimulation_GPUDoc : public CDocument
{
protected: // create from serialization only
	CCuttingSimulation_GPUDoc();
	DECLARE_DYNCREATE(CCuttingSimulation_GPUDoc)

// Attributes
public:

// Operations
public:

// Overrides
public:
	virtual BOOL OnNewDocument();
	virtual void Serialize(CArchive& ar);

// Implementation
public:
	virtual ~CCuttingSimulation_GPUDoc();
#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

protected:

// Generated message map functions
protected:
	DECLARE_MESSAGE_MAP()
};


