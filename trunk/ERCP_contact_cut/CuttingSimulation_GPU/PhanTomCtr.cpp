// PhanTomCtr.cpp: implementation of the CPhanTomCtr class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "PhanTomCtr.h"


#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif

HDCallbackCode HDCALLBACK touchScene(void *pUserData);

//Haptic handler
HHD hHD;
HDSchedulerHandle hUpdateDeviceCallback;

//global variable
CPhanTomCtr g_PhanTomCtr;

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CPhanTomCtr::CPhanTomCtr()
{
	Collision=false;
}

CPhanTomCtr::~CPhanTomCtr()
{

}

void CPhanTomCtr::InitHD()
{
    HDErrorInfo error;
    hHD = hdInitDevice(HD_DEFAULT_DEVICE);
    hdEnable(HD_FORCE_OUTPUT);

	if (HD_DEVICE_ERROR(error = hdGetError())) 
	{
		fprintf(stderr, "\nPress any key to quit.\n");
		getchar();
		exit(-1);
	}

	hdSetSchedulerRate(1000);    //Sampling rate: 1kHz 
	hUpdateDeviceCallback = hdScheduleAsynchronous(touchScene, 0, HD_MAX_SCHEDULER_PRIORITY);
	hdStartScheduler();
	if (HD_DEVICE_ERROR(error = hdGetError()))
	{
		exit(-1);
	}
}

Vec3f CPhanTomCtr::getPos()
{
	return Vec3f(Position[0],Position[1],Position[2]);
}

void CPhanTomCtr::setProxyPos(Vec3f pos)
{
	for(int i=0;i<3;i++)
		ProxPos[i]=pos[i];
}

void CPhanTomCtr::setForce(Vec3d force)
{

	hduVector3Dd Force;
	Force.set(0,0,0);
	
	for(int i=0;i<3;i++)
	{
		Force[i]=force[i];
	}
	hdSetDoublev(HD_CURRENT_FORCE,Force);
}

Mat3x3f CPhanTomCtr::getRot()
{
	Mat3x3f rot;
	rot(0,0)=Transform[0]; rot(0,1)=Transform[1]; rot(0,2)=Transform[2];
	rot(1,0)=Transform[4]; rot(1,1)=Transform[5]; rot(1,2)=Transform[6];
	rot(2,0)=Transform[8]; rot(2,1)=Transform[9]; rot(2,2)=Transform[10];
	rot.transpose();
	return rot;
}

HDCallbackCode HDCALLBACK touchScene(void *pUserData)
{
    hdBeginFrame(hHD);
    hdGetDoublev(HD_CURRENT_POSITION, g_PhanTomCtr.Position);
	hdGetDoublev(HD_CURRENT_TRANSFORM,g_PhanTomCtr.Transform);

	float check=g_PhanTomCtr.Position[0];
	hduVector3Dd force;

	hdSetDoublev(HD_CURRENT_FORCE,force);
	hdEndFrame(hHD);
    return HD_CALLBACK_CONTINUE;
}
