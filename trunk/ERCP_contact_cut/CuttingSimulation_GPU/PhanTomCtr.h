// PhanTomCtr.h: interface for the CPhanTomCtr class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_PHANTOMCTR_H__E3198F94_B3CE_40F3_BB88_0186044656DE__INCLUDED_)
#define AFX_PHANTOMCTR_H__E3198F94_B3CE_40F3_BB88_0186044656DE__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include <HD/hd.h>
#include <HDU/hdu.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>

#include <stdio.h>
#include "./DataTypes/Vec.h"
#include "./DataTypes/Mat.h"

class CPhanTomCtr  
{
public:
	CPhanTomCtr();
	virtual ~CPhanTomCtr();

public:
	void InitHD();
	Vec3f getPos();
	Mat3x3f getRot();
	void setProxyPos(Vec3f pos);
	void setForce(Vec3d force);
	HDdouble Transform[16];
	hduVector3Dd Position;
	hduVector3Dd ProxPos;

	bool Collision;
};
extern CPhanTomCtr g_PhanTomCtr;

#endif // !defined(AFX_PHANTOMCTR_H__E3198F94_B3CE_40F3_BB88_0186044656DE__INCLUDED_)
