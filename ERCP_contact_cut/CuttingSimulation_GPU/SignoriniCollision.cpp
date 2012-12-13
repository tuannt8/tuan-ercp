#include "StdAfx.h"
#include "SignoriniCollision.h"
#include "DataStruct.h"
#include "Modules/CollisionManager.h"
#include "Utility.h"

SignoriniCollision::SignoriniCollision(void)
{
	ii=NULL;
	jj=NULL;
	newii=NULL;
	newjj=NULL;
	newB=NULL;
	InverseB=NULL;
	newBB=NULL;
	dP=NULL;
	DuplicatedPoint=NULL;
	Mapping=NULL;

	MC1=NULL;
	MC2=NULL;
	MCM=NULL;
	delta=NULL;
	Force=NULL;
	M1=NULL;
	M2=NULL;
	ReducedM1=NULL;
	ReducedM2=NULL;


	TriIndex=NULL;
	
	fp=fopen("../DebugLog/ContactModelTime.txt","w");
	Delta=fopen("../DebugLog/Delta2.txt","w");
}

SignoriniCollision::~SignoriniCollision(void)
{
	if(ii)
		delete [] ii;
	if(jj)
		delete [] jj;
	if(newii)
		delete [] newii;
	if(newjj)
		delete [] newjj;
	if(newB)
		for(int i=0;i<size;i++)
			delete [] newB[i];
	if(InverseB)
		for(int i=0;i<size;i++)
			delete [] InverseB[i];
	if(newBB)
		for(int i=0;i<size;i++)
			delete [] newBB[i];
	if(dP)
		delete [] dP;

	if(DuplicatedPoint)
		delete [] DuplicatedPoint;


	if(Mapping)
		for(int i=0;i<4;i++)
			delete [] Mapping[i];

	if(MC1)
		delete [] MC1;
	if(MC2)
		delete [] MC2;
	if(MCM)
		for(int i=0;i<NbCollide;i++)
			delete [] MCM[i];

	if(M1)
		for(int i=0;i<NbCollide;i++)
			delete [] M1[i];
	if(M2)
		for(int i=0;i<NbCollide;i++)
			delete [] M2[i];

	if(ReducedM1)
		delete [] ReducedM1;
	if(ReducedM2)
		delete [] ReducedM2;
	if(TriIndex)
		delete [] TriIndex;


	fclose(fp);
	fclose(Delta);
}

void SignoriniCollision::drawVec(Vec3d p1, Vec3d p2)
{
	glColor3f(0,0,1);
	glBegin(GL_LINES);
	glVertex3f((GLfloat)p1[0],(GLfloat)p1[1],(GLfloat)p1[2]);
	glVertex3f((GLfloat)p2[0],(GLfloat)p2[1],(GLfloat)p2[2]);
	glEnd();
}

void SignoriniCollision::drawCollisionInfo()
{

	for (int i=0; i<distance.size(); i++)
	{
		drawVec(distance[i].measuredPointInCylinder, distance[i].collidedTriPoint);
	}

}


// line equation: r=r0+tv, return t;
double SignoriniCollision::projectOnLine(Vec3d r0, Vec3d direc, Vec3d P)
{
	return (P*direc-r0*direc);
}
double SignoriniCollision::projectOnPlane(Vec3d Normal,Vec3d Center,Vec3d Point)
{

	double dis=(Normal*(Point-Center))/(Normal.norm());
	if(dis>0)
	{
		return dis;
	}else{
		return -dis;
	}
}

Vec3d SignoriniCollision::findProjectedPointToPlane(Vec3d Point, Vec3d Normal, Vec3d Center)
{
	Normal.normalize();
	double d=(Point-Center)*Normal;

	Point=Point-Normal*d;

	return Point;
}


double SignoriniCollision::computeDistanceBetweenTwoline(Vec3d p1, Vec3d p2, Vec3d p3, Vec3d p4)
{
	Vec3d direc=p4-p3; direc.normalize();
	Vec3d r0=p3;
	double ta,tb,da,db,t,s, dis;
	Vec3d Qa,Qb,Pa,Pb,Pa2,P,P2;


	Pa=p1;	Pb=p2;		
	ta=projectOnLine(r0, direc, Pa);		
	tb=projectOnLine(r0, direc, Pb);
	Qa=r0+direc*ta;		Qb=r0+direc*tb;
	da=(Pa-Qa).norm();	db=(Pb-Qb).norm();
	Pa2=Pa+Qb-Qa;
	s=(Pa2-Pb).norm();
	t=(da*da-db*db+s*s)/(2*s*s);

	P=Pa+(Pb-Pa)*t;
	P2=r0+direc*t;
	dis=(P2-P).norm();
	return dis;	
}

Vec3d SignoriniCollision::findPointBetweenTwoline1(Vec3d p1, Vec3d p2, Vec3d p3, Vec3d p4)
{
	Vec3d direc=p4-p3; direc.normalize();
	Vec3d r0=p3;
	double ta,tb,da,db,t,s;
	Vec3d Qa,Qb,Pa,Pb,Pa2,P,P2;


	Pa=p1;	Pb=p2;		
	ta=projectOnLine(r0, direc, Pa);		
	tb=projectOnLine(r0, direc, Pb);
	Qa=r0+direc*ta;		Qb=r0+direc*tb;
	da=(Pa-Qa).norm();	db=(Pb-Qb).norm();
	Pa2=Pa+Qb-Qa;
	s=(Pa2-Pb).norm();
	t=(da*da-db*db+s*s)/(2*s*s);

	P=Pa+(Pb-Pa)*t;

	return P;	
}

Vec3d SignoriniCollision::findPointBetweenTwoline2(Vec3d p1, Vec3d p2, Vec3d p3, Vec3d p4)
{
	Vec3d direc=p4-p3; direc.normalize();
	Vec3d r0=p3;
	double ta,tb,da,db,t,s;
	Vec3d Qa,Qb,Pa,Pb,Pa2,P,P2;


	Pa=p1;	Pb=p2;		
	ta=projectOnLine(r0, direc, Pa);		
	tb=projectOnLine(r0, direc, Pb);
	Qa=r0+direc*ta;		Qb=r0+direc*tb;
	da=(Pa-Qa).norm();	db=(Pb-Qb).norm();
	Pa2=Pa+Qb-Qa;
	s=(Pa2-Pb).norm();
	t=(da*da-db*db+s*s)/(2*s*s);

	P=Pa+(Pb-Pa)*t;
	P2=r0+direc*t;
	return P2;	
}



Vec3d SignoriniCollision::computeFaceNormal(Triangle& tri)
{
	Vec3d vector1, vector2, normal;
	vector1=tri.p2-tri.p1; 
	vector2=tri.p3-tri.p1;
	normal = vector1.cross(vector2);
	normal.normalize();
	return normal;
}



bool SignoriniCollision::CollisionDetectionUsingDistanceTest(MyFFD* surf,MyFFD* cont)
{
	Vec3d* SurfacePoint=surf->returnSurfacePoint();
	int NbSurfacePoint=surf->returnNbSurface();
	int NbCenterPoint=surf->returnNbCenter();
	int NbCircularPoint=surf->returnNbCircular();
	int NbFace=surf->returnNbFace();
	int* FaceIndex=surf->returnFaceIndex();
	Vec3i* Face=surf->returnFace();
	Vec3d* Points=cont->returnControlPoint();
	int NbPoint=cont->returnNbPoint();
	int minNb;
	int maxNb;

	int Margin=4;

	int face[2];
	int Nb[2];

	Vec3d collidedCylPoint;
	Vec3d collidedTriPoint;
	Vec3d MinCylPoint;
	Vec3d MinTriPoint;
	int CylIndex;
	double Length;
	double minimumDistance=1000000;

	distance.clear();

	minNb=10000000000;
	maxNb=-10000000000;
	for (int i=0;i<NbPoint-1;i++)
	{
		Nb[0]=surf->closestCenterPointIdx(Points[i]);
		Nb[1]=surf->closestCenterPointIdx(Points[i+1]);
		if(Nb[0]>=Nb[1]){
			int temp=Nb[1];
			Nb[1]=Nb[0];
			Nb[0]=temp;
		}
		if(Nb[0]<=minNb)
			minNb=Nb[0];
		if(Nb[1]>=maxNb)
			maxNb=Nb[1];
	}
	if(minNb==0)
	{
		face[0]=FaceIndex[minNb];
		face[1]=FaceIndex[maxNb+Margin];
	}else if(maxNb==NbCenterPoint-1){
		face[0]=FaceIndex[minNb-Margin];
		face[1]=FaceIndex[maxNb];
	}else{
		face[0]=FaceIndex[minNb-Margin/2];
		face[1]=FaceIndex[maxNb+Margin];
	}

	Distancefield dis;
	int i,j;
#pragma omp for private(i,j,CylIndex,MinCylPoint,MinTriPoint,dis)
	for (j= face[0];j<face[1];j++){
		Length=10000000000000;
		for(i=0;i<NbPoint-1;i++){
			measureMinimumDistanceBetLineandTriangle(Points[i],Points[i+1],SurfacePoint[Face[j][0]],SurfacePoint[Face[j][1]],SurfacePoint[Face[j][2]],collidedTriPoint,collidedCylPoint);
			if((collidedCylPoint-collidedTriPoint).norm()<Length)
			{
				Length=(collidedCylPoint-collidedTriPoint).norm();
				CylIndex=i;
				MinCylPoint=collidedCylPoint;
				MinTriPoint=collidedTriPoint;
			}	
		}
		if(Length<minimumDistance)
		{

			dis.triIdx=j;
			dis.cylinderIdx=CylIndex;
			Vec3d CollidedDirec=MinTriPoint-MinCylPoint;
			CollidedDirec.normalize();
			dis.measuredPointInCylinder=MinCylPoint;
			dis.collidedCylPoint=MinCylPoint+CollidedDirec*(minimumDistance-Length);
			dis.collidedTriPoint=MinTriPoint;
			dis.Distance=Length;
			dis.PenetratedDirec=-CollidedDirec;
			dis.Penetration=minimumDistance-Length;

			distance.push_back(dis);
		}
	}

	if(distance.size())
		return true;
	else
		return false;

}

bool SignoriniCollision::CollisionDetectionUsingDistanceEndoscope(MyFFD* surf,MyFFD* cont)
{
	Vec3d* SurfacePoint=surf->returnSurfacePoint();
	int NbSurfacePoint=surf->returnNbSurface();
	int NbCenterPoint=surf->returnNbCenter();
	int NbCircularPoint=surf->returnNbCircular();
	int NbFace=surf->returnNbFace();
	int* FaceIndex=surf->returnFaceIndex();
	Vec3i* Face=surf->returnFace();
	Vec3d* Points=cont->returnControlPoint();
	int NbPoint=cont->returnNbPoint();
	int minface;
	int maxface;

	int Margin=4;

	int face[2];
	int Nb[2];

	Vec3d collidedCylPoint;
	Vec3d collidedTriPoint;
	Vec3d MinCylPoint;
	Vec3d MinTriPoint;
	int CylIndex;
	double Length;
	double minimumDistance=cont->returnRadiusOfEndoscope();

	distance.clear();

	minface=10000000000000;
	maxface=-10000000000000;
	for (int i=0;i<NbPoint-1;i++)
	{
		Nb[0]=surf->closestCenterPointIdx(Points[i]);
		Nb[1]=surf->closestCenterPointIdx(Points[i+1]);
		if(Nb[0]>=Nb[1]){
			int temp=Nb[1];
			Nb[1]=Nb[0];
			Nb[0]=temp;
		}
		if(Nb[0]<=minface)
			minface=Nb[0];
		if(Nb[1]>=maxface)
			maxface=Nb[1];
	}
	if(minface==0)
	{
		face[0]=FaceIndex[minface];
		face[1]=FaceIndex[maxface+Margin];
	}else if(maxface==NbCenterPoint-1){
		face[0]=FaceIndex[minface-Margin];
		face[1]=FaceIndex[maxface];
	}else{
		face[0]=FaceIndex[minface-Margin/2];
		face[1]=FaceIndex[maxface+Margin/2];
	}

	Distancefield dis;
	int j,i;
//#pragma omp for private(i,j,CylIndex,MinCylPoint,MinTriPoint,dis)
	for (j= 0;j<NbFace;j++){
		Length=10000000000000;
		for(i=0;i<NbPoint-1;i++){
			measureMinimumDistanceBetLineandTriangle(Points[i],Points[i+1],SurfacePoint[Face[j][0]],SurfacePoint[Face[j][1]],SurfacePoint[Face[j][2]],collidedTriPoint,collidedCylPoint);
			if((collidedCylPoint-collidedTriPoint).norm()<Length)
			{
				Length=(collidedCylPoint-collidedTriPoint).norm();
				CylIndex=i;
				MinCylPoint=collidedCylPoint;
				MinTriPoint=collidedTriPoint;
			}	
		}
		if(Length<minimumDistance)
		{

			dis.triIdx=j;
			dis.cylinderIdx=CylIndex;
			Vec3d CollidedDirec=MinTriPoint-MinCylPoint;
			CollidedDirec.normalize();
			dis.measuredPointInCylinder=MinCylPoint;
			dis.collidedCylPoint=MinCylPoint+CollidedDirec*(minimumDistance-Length);
			dis.collidedTriPoint=MinTriPoint;
			dis.Distance=Length;
			dis.PenetratedDirec=-CollidedDirec;
			dis.Penetration=minimumDistance-Length;

			distance.push_back(dis);
		}
	}

	if(distance.size())
		return true;
	else
		return false;

}

bool SignoriniCollision::CollisionDetectionUsingDistanceEndoscopeV2(MyFFD* surf,MyFFD* cont)
{
	Vec3d* SurfacePoint=surf->returnSurfacePoint();
	int NbSurfacePoint=surf->returnNbSurface();
	int NbCenterPoint=surf->returnNbCenter();
	int NbCircularPoint=surf->returnNbCircular();
	int NbFace=surf->returnNbFace();
	int* FaceIndex=surf->returnFaceIndex();
	Vec3i* Face=surf->returnFace();
	Vec3d* Points=cont->returnControlPoint();
	int NbPoint=cont->returnNbPoint();
	int minface;
	int maxface;

	int Margin=4;

	int face[2];
	int Nb[2];

	Vec3d collidedCylPoint;
	Vec3d collidedTriPoint;
	Vec3d MinCylPoint;
	Vec3d MinTriPoint;
	int CylIndex;
	double Length;
	double minimumDistance=cont->returnRadiusOfEndoscope();

	distance.clear();

	minface=10000000000000;
	maxface=-10000000000000;
	for (int i=0;i<NbPoint-1;i++)
	{
		Nb[0]=surf->closestCenterPointIdx(Points[i]);
		Nb[1]=surf->closestCenterPointIdx(Points[i+1]);
		if(Nb[0]>=Nb[1]){
			int temp=Nb[1];
			Nb[1]=Nb[0];
			Nb[0]=temp;
		}
		if(Nb[0]<=minface)
			minface=Nb[0];
		if(Nb[1]>=maxface)
			maxface=Nb[1];
	}
	if(minface==0)
	{
		face[0]=FaceIndex[minface];
		face[1]=FaceIndex[maxface+Margin];
	}else if(maxface==NbCenterPoint-1){
		face[0]=FaceIndex[minface-Margin];
		face[1]=FaceIndex[maxface];
	}else{
		face[0]=FaceIndex[minface-Margin/2];
		face[1]=FaceIndex[maxface+Margin*2];
	}

	Distancefield dis;
	int j,i;
	//#pragma omp for private(i,j,CylIndex,MinCylPoint,MinTriPoint,dis)
	for (j= face[0];j<face[1];j++){
		Length=10000000000000;
		for(i=0;i<NbPoint-1;i++){
			measureMinimumDistanceBetLineandTriangle(Points[i],Points[i+1],SurfacePoint[Face[j][0]],SurfacePoint[Face[j][1]],SurfacePoint[Face[j][2]],collidedTriPoint,collidedCylPoint);
			if((collidedCylPoint-collidedTriPoint).norm()<Length)
			{
				Length=(collidedCylPoint-collidedTriPoint).norm();
				CylIndex=i;
				MinCylPoint=collidedCylPoint;
				MinTriPoint=collidedTriPoint;
			}	
		}
		if(Length<minimumDistance)
		{

			dis.triIdx=j;
			dis.cylinderIdx=CylIndex;
			Vec3d CollidedDirec=MinTriPoint-MinCylPoint;
			CollidedDirec.normalize();
			dis.measuredPointInCylinder=MinCylPoint;
			dis.collidedCylPoint=MinCylPoint+CollidedDirec*(minimumDistance-Length);
			dis.collidedTriPoint=MinTriPoint;
			dis.Distance=Length;
			dis.PenetratedDirec=-CollidedDirec;
			dis.Penetration=minimumDistance-Length;

			distance.push_back(dis);
		}
	}

	if(distance.size())
		return true;
	else
		return false;

}


bool SignoriniCollision::PotentialCollisionDetection(MyFFD* surf,MyFFD* cont)
{
	Vec3d* SurfacePoint=surf->returnSurfacePoint();
	int NbSurfacePoint=surf->returnNbSurface();
	int NbCenterPoint=surf->returnNbCenter();
	int NbCircularPoint=surf->returnNbCircular();
	int NbFace=surf->returnNbFace();
	int* FaceIndex=surf->returnFaceIndex();
	Vec3i* Face=surf->returnFace();
	Vec3d* Points=cont->returnControlPoint();
	int NbPoint=cont->returnNbPoint();


	int Margin=2;

	int face[2];
	int Nb[2];

	Vec3d collidedCylPoint;
	Vec3d collidedTriPoint;
	Vec3d MinCylPoint;
	Vec3d MinTriPoint;
	int CylIndex;
	double Length;
	double minimumDistance=cont->returnRadiusOfEndoscope()*1.1;
	Distancefield dis;


	double AvgPenetration=0.0;
	double MaxPenetration=1000000000000.0;



	distance.clear();
	InspectedTri.clear();

//#pragma omp parallel for
	for (int i=0;i<NbPoint-1;i++)
	{


		Nb[0]=surf->closestCenterPointIdx(Points[i]);
		Nb[1]=surf->closestCenterPointIdx(Points[i+1]);
		if(Nb[0]>=Nb[1]){
			int temp=Nb[1];
			Nb[1]=Nb[0];
			Nb[0]=temp;
		}
		if(i==0){
			if(Nb[0]-Margin<=0)
			{
				face[0]=FaceIndex[0];
				face[1]=FaceIndex[Nb[1]+Margin*4];
				
			}else if(Nb[1]+Margin*4>=NbCenterPoint-1){
				face[0]=FaceIndex[Nb[0]-Margin];
				face[1]=FaceIndex[NbCenterPoint-1];
			}else{
				face[0]=FaceIndex[Nb[0]-Margin];
				face[1]=FaceIndex[Nb[1]+Margin*4];
			}
		}else{
			if(Nb[0]-Margin<=0)
			{
				face[0]=FaceIndex[0];
				face[1]=FaceIndex[Nb[1]+Margin];

			}else if(Nb[1]+Margin>=NbCenterPoint-1){
				face[0]=FaceIndex[Nb[0]-Margin];
				face[1]=FaceIndex[NbCenterPoint-1];
			}else{
				face[0]=FaceIndex[Nb[0]-Margin];
				face[1]=FaceIndex[Nb[1]+Margin];
			}
		}

		for (int j= face[0];j<face[1];j++){		
			measureMinimumDistanceBetLineandTriangle(Points[i],Points[i+1],SurfacePoint[Face[j][0]],SurfacePoint[Face[j][1]],SurfacePoint[Face[j][2]],collidedTriPoint,collidedCylPoint);
			
			Length=(collidedCylPoint-collidedTriPoint).norm();
			CylIndex=i;
			MinCylPoint=collidedCylPoint;
			MinTriPoint=collidedTriPoint;

			if(Length<minimumDistance)
			{
				InspectedTri.push_back(j);
				AvgPenetration+=Length-cont->returnRadiusOfEndoscope();
				if(Length-cont->returnRadiusOfEndoscope()<MaxPenetration)
					MaxPenetration=Length-cont->returnRadiusOfEndoscope();
				dis.triIdx=j;
				dis.cylinderIdx=CylIndex;
				Vec3d CollidedDirec=MinTriPoint-MinCylPoint;
				CollidedDirec.normalize();
				dis.measuredPointInCylinder=MinCylPoint;
				dis.collidedCylPoint=MinCylPoint+CollidedDirec*(minimumDistance-Length);
				dis.collidedTriPoint=MinTriPoint;
				dis.Distance=Length;
				dis.PenetratedDirec=-CollidedDirec;
				dis.Penetration=Length-cont->returnRadiusOfEndoscope();

				distance.push_back(dis);
			}
		}
	}

	


	if(distance.size()){
		fprintf(Delta,"%f\n",MaxPenetration);
		return true;
	}else{
		return false;
	}

}

bool SignoriniCollision::PotentialCollisionDetection(MyFFD* surf,MyFFD* cont,int Margin, double boundary)
{
	Vec3d* SurfacePoint=surf->returnSurfacePoint();
	int NbSurfacePoint=surf->returnNbSurface();
	int NbCenterPoint=surf->returnNbCenter();
	int NbCircularPoint=surf->returnNbCircular();
	int NbFace=surf->returnNbFace();
	int* FaceIndex=surf->returnFaceIndex();
	Vec3i* Face=surf->returnFace();
	Vec3d* Points=cont->returnControlPoint();
	int NbPoint=cont->returnNbPoint();


	int face[2];
	int Nb[2];

	Vec3d collidedCylPoint;
	Vec3d collidedTriPoint;
	Vec3d MinCylPoint;
	Vec3d MinTriPoint;
	int CylIndex;
	double Length;
	double minimumDistance=cont->returnRadiusOfEndoscope()*boundary;
	Distancefield dis;


	double AvgPenetration=0.0;
	double MaxPenetration=1000000000000.0;



	distance.clear();
	InspectedTri.clear();

	int InsertedIndex=0;
	for(int i=0;i<NbPoint;i++)
		if((Points[i]-cont->returnInsertedPoint())*cont->returnInsertedDirec()>=0)
			InsertedIndex=i;


	for (int i=0;i<InsertedIndex;i++)
	{


		Nb[0]=surf->closestCenterPointIdx(Points[i]);
		Nb[1]=surf->closestCenterPointIdx(Points[i+1]);
		if(Nb[0]>=Nb[1]){
			int temp=Nb[1];
			Nb[1]=Nb[0];
			Nb[0]=temp;
		}
		if(i==0){
			if(Nb[0]-Margin<=0)
			{
				face[0]=FaceIndex[0];
				face[1]=FaceIndex[Nb[1]+Margin*4];

			}else if(Nb[1]+Margin*4>=NbCenterPoint-1){
				face[0]=FaceIndex[Nb[0]-Margin];
				face[1]=FaceIndex[NbCenterPoint-1];
			}else{
				face[0]=FaceIndex[Nb[0]-Margin];
				face[1]=FaceIndex[Nb[1]+Margin*4];
			}
		}else{
			if(Nb[0]-Margin<=0)
			{
				face[0]=FaceIndex[0];
				face[1]=FaceIndex[Nb[1]+Margin];

			}else if(Nb[1]+Margin>=NbCenterPoint-1){
				face[0]=FaceIndex[Nb[0]-Margin];
				face[1]=FaceIndex[NbCenterPoint-1];
			}else{
				face[0]=FaceIndex[Nb[0]-Margin];
				face[1]=FaceIndex[Nb[1]+Margin];
			}
		}

		for (int j= face[0];j<face[1];j++){		
			measureMinimumDistanceBetLineandTriangle(Points[i],Points[i+1],SurfacePoint[Face[j][0]],SurfacePoint[Face[j][1]],SurfacePoint[Face[j][2]],collidedTriPoint,collidedCylPoint);
			InspectedTri.push_back(j);
			Length=(collidedCylPoint-collidedTriPoint).norm();
			CylIndex=i;
			MinCylPoint=collidedCylPoint;
			MinTriPoint=collidedTriPoint;

			if(Length<minimumDistance)
			{
				
				AvgPenetration+=Length-cont->returnRadiusOfEndoscope();
				if(Length-cont->returnRadiusOfEndoscope()<MaxPenetration)
					MaxPenetration=Length-cont->returnRadiusOfEndoscope();
				dis.triIdx=j;
				dis.cylinderIdx=CylIndex;
				Vec3d CollidedDirec=MinTriPoint-MinCylPoint;
				CollidedDirec.normalize();
				dis.measuredPointInCylinder=MinCylPoint;
				dis.collidedCylPoint=MinCylPoint+CollidedDirec*(minimumDistance-Length);
				dis.collidedTriPoint=MinTriPoint;
				dis.Distance=Length;
				dis.PenetratedDirec=-CollidedDirec;
				dis.Penetration=Length-cont->returnRadiusOfEndoscope();

				distance.push_back(dis);
			}
		}
	}




	if(distance.size()){
		return true;
	}else{
		return false;
	}

}

bool SignoriniCollision::PotentialCollisionDetectionV2(MyFFD* surf,MyFFD* cont,int Margin, double boundary)
{
	Vec3d* SurfacePoint=surf->returnSurfacePoint();
	int NbSurfacePoint=surf->returnNbSurface();
	int NbCenterPoint=surf->returnNbCenter();
	int NbCircularPoint=surf->returnNbCircular();
	int NbFace=surf->returnNbFace();
	int* FaceIndex=surf->returnFaceIndex();
	Vec3i* Face=surf->returnFace();
	Vec3d* Points=cont->returnControlPoint();
	int NbPoint=cont->returnNbPoint();


	int face[2];
	int Nb[2];

	Vec3d collidedCylPoint;
	Vec3d collidedTriPoint;
	Vec3d MinCylPoint;
	Vec3d MinTriPoint;
	int CylIndex;
	double Length;
	double minimumDistance=cont->returnRadiusOfEndoscope()*boundary;
	Distancefield dis;


	double AvgPenetration=0.0;
	double MaxPenetration=1000000000000.0;



	distance.clear();
	InspectedTri.clear();


	for (int i=0;i<NbPoint-1;i++)
	{


		Nb[0]=surf->closestCenterPointIdx(Points[i]);
		Nb[1]=surf->closestCenterPointIdx(Points[i+1]);
		if(Nb[0]>=Nb[1]){
			int temp=Nb[1];
			Nb[1]=Nb[0];
			Nb[0]=temp;
		}
		if(i==0){
			if(Nb[0]-Margin<=0)
			{
				face[0]=FaceIndex[0];
				face[1]=FaceIndex[Nb[1]+Margin*4];

			}else if(Nb[1]+Margin*4>=NbCenterPoint-1){
				face[0]=FaceIndex[Nb[0]-Margin];
				face[1]=FaceIndex[NbCenterPoint-1];
			}else{
				face[0]=FaceIndex[Nb[0]-Margin];
				face[1]=FaceIndex[Nb[1]+Margin*4];
			}
		}else{
			if(Nb[0]-Margin<=0)
			{
				face[0]=FaceIndex[0];
				face[1]=FaceIndex[Nb[1]+Margin];

			}else if(Nb[1]+Margin>=NbCenterPoint-1){
				face[0]=FaceIndex[Nb[0]-Margin];
				face[1]=FaceIndex[NbCenterPoint-1];
			}else{
				face[0]=FaceIndex[Nb[0]-Margin];
				face[1]=FaceIndex[Nb[1]+Margin];
			}
		}

		for (int j= face[0];j<face[1];j++){		
			measureMinimumDistanceBetPointandTriangle(Points[i],SurfacePoint[Face[j][0]],SurfacePoint[Face[j][1]],SurfacePoint[Face[j][2]],collidedTriPoint,collidedCylPoint);
			InspectedTri.push_back(j);
			Length=(collidedCylPoint-collidedTriPoint).norm();
			CylIndex=i;
			MinCylPoint=collidedCylPoint;
			MinTriPoint=collidedTriPoint;

			if(Length<minimumDistance)
			{

				AvgPenetration+=Length-cont->returnRadiusOfEndoscope();
				if(Length-cont->returnRadiusOfEndoscope()<MaxPenetration)
					MaxPenetration=Length-cont->returnRadiusOfEndoscope();
				dis.triIdx=j;
				dis.cylinderIdx=CylIndex;
				Vec3d CollidedDirec=MinTriPoint-MinCylPoint;
				CollidedDirec.normalize();
				dis.measuredPointInCylinder=MinCylPoint;
				dis.collidedCylPoint=MinCylPoint+CollidedDirec*(minimumDistance-Length);
				dis.collidedTriPoint=MinTriPoint;
				dis.Distance=Length;
				dis.PenetratedDirec=-CollidedDirec;
				dis.Penetration=Length-cont->returnRadiusOfEndoscope();

				distance.push_back(dis);
			}
		}
	}




	if(distance.size()){
		fprintf(Delta,"%f\n",MaxPenetration);
		return true;
	}else{
		return false;
	}

}

bool SignoriniCollision::PotentialCollisionDetectionV3(MyFFD* surf,MyFFD* cont,int Margin, double boundary)
{
	Vec3d* SurfacePoint=surf->returnSurfacePoint();
	int NbSurfacePoint=surf->returnNbSurface();
	int NbCenterPoint=surf->returnNbCenter();
	int NbCircularPoint=surf->returnNbCircular();
	int NbFace=surf->returnNbFace();
	int* FaceIndex=surf->returnFaceIndex();
	Vec3i* Face=surf->returnFace();
	Vec3d* Points=cont->returnControlPoint();
	int NbPoint=cont->returnNbPoint();


	int face[2];
	int Nb[2];

	Vec3d collidedCylPoint;
	Vec3d collidedTriPoint;
	Vec3d MinCylPoint;
	Vec3d MinTriPoint;
	int CylIndex;
	double Length;
	double minimumDistance=cont->returnRadiusOfEndoscope()*boundary;
	Distancefield dis;


	double AvgPenetration=0.0;
	double MaxPenetration=1000000000000.0;



	distance.clear();
	InspectedTri.clear();




	Nb[0]=surf->closestCenterPointIdx(Points[NbPoint-2]);
	Nb[1]=surf->closestCenterPointIdx(Points[NbPoint-1]);
	if(Nb[0]>=Nb[1]){
		int temp=Nb[1];
		Nb[1]=Nb[0];
		Nb[0]=temp;
	}

	if(Nb[0]-Margin<=0)
	{
		face[0]=FaceIndex[0];
		face[1]=FaceIndex[Nb[1]+Margin*4];

	}else if(Nb[1]+Margin*4>=NbCenterPoint-1){
		face[0]=FaceIndex[Nb[0]-Margin];
		face[1]=FaceIndex[NbCenterPoint-1];
	}else{
		face[0]=FaceIndex[Nb[0]-Margin];
		face[1]=FaceIndex[Nb[1]+Margin*4];
	}
	

	for (int j= face[0];j<face[1];j++){		
		measureMinimumDistanceBetPointandTriangle(Points[0],SurfacePoint[Face[j][0]],SurfacePoint[Face[j][1]],SurfacePoint[Face[j][2]],collidedTriPoint,collidedCylPoint);
		InspectedTri.push_back(j);
		Length=(collidedCylPoint-collidedTriPoint).norm();
		CylIndex=0;
		MinCylPoint=collidedCylPoint;
		MinTriPoint=collidedTriPoint;

		if(Length<minimumDistance)
		{

			AvgPenetration+=Length-cont->returnRadiusOfEndoscope();
			if(Length-cont->returnRadiusOfEndoscope()<MaxPenetration)
				MaxPenetration=Length-cont->returnRadiusOfEndoscope();
			dis.triIdx=j;
			dis.cylinderIdx=CylIndex;
			Vec3d CollidedDirec=MinTriPoint-MinCylPoint;
			CollidedDirec.normalize();
			dis.measuredPointInCylinder=MinCylPoint;
			dis.collidedCylPoint=MinCylPoint+CollidedDirec*(minimumDistance-Length);
			dis.collidedTriPoint=MinTriPoint;
			dis.Distance=Length;
			dis.PenetratedDirec=-CollidedDirec;
			dis.Penetration=Length-cont->returnRadiusOfEndoscope();

			distance.push_back(dis);
		}
	}





	if(distance.size()){
		return true;
	}else{
		return false;
	}

}

void SignoriniCollision::PenaltyMethod(MyFFD* surf, MyFFD* cont, double K)
{
	Vec3d force;
	for(int i=0;i<distance.size();i++)
	{
		force=(distance[i].PenetratedDirec)*distance[i].Penetration*K;
		surf->addForcetoSurfaceTri(distance[i].triIdx, distance[i].collidedTriPoint,-force);
		cont->addForceBetweenPoint(distance[i].cylinderIdx,distance[i].cylinderIdx+1,distance[i].measuredPointInCylinder,force);
	}

}

bool SignoriniCollision::CollisionDetection(MyFFD* surf,MyFFD* cont)
{
	Vec3d* SurfacePoint=surf->returnSurfacePoint();
	int NbSurfacePoint=surf->returnNbSurface();
	int NbCenterPoint=surf->returnNbCenter();
	int NbCircularPoint=surf->returnNbCircular();
	int NbFace=surf->returnNbFace();
	int* FaceIndex=surf->returnFaceIndex();
	Vec3i* Face=surf->returnFace();
	Vec3d* Points=cont->returnControlPoint();
	int NbPoint=cont->returnNbPoint();


	int Margin=2;

	int face[2];
	int Nb[2];

	Vec3d collidedCylPoint;
	Vec3d collidedTriPoint;
	Vec3d MinCylPoint;
	Vec3d MinTriPoint;
	int CylIndex;
	double Length;
	double minimumDistance=cont->returnRadiusOfEndoscope()*1.0;
	Distancefield dis;


	distance.clear();
	InspectedTri.clear();


	for (int i=0;i<NbPoint-1;i++)
	{
		Nb[0]=surf->closestCenterPointIdx(Points[i]);
		Nb[1]=surf->closestCenterPointIdx(Points[i+1]);
		if(Nb[0]>=Nb[1]){
			int temp=Nb[1];
			Nb[1]=Nb[0];
			Nb[0]=temp;
		}
		if(i==0){
			if(Nb[0]-Margin<=0)
			{
				face[0]=FaceIndex[0];
				face[1]=FaceIndex[Nb[1]+Margin*4];

			}else if(Nb[1]+Margin*4>=NbCenterPoint-1){
				face[0]=FaceIndex[Nb[0]-Margin];
				face[1]=FaceIndex[NbCenterPoint-1];
			}else{
				face[0]=FaceIndex[Nb[0]-Margin];
				face[1]=FaceIndex[Nb[1]+Margin*4];
			}
		}else{
			if(Nb[0]-Margin<=0)
			{
				face[0]=FaceIndex[0];
				face[1]=FaceIndex[Nb[1]+Margin];

			}else if(Nb[1]+Margin>=NbCenterPoint-1){
				face[0]=FaceIndex[Nb[0]-Margin];
				face[1]=FaceIndex[NbCenterPoint-1];
			}else{
				face[0]=FaceIndex[Nb[0]-Margin];
				face[1]=FaceIndex[Nb[1]+Margin];
			}
		}

		for (int j= face[0];j<face[1];j++){
			Length=10000000000000;

			measureMinimumDistanceBetLineandTriangle(Points[i],Points[i+1],SurfacePoint[Face[j][0]],SurfacePoint[Face[j][1]],SurfacePoint[Face[j][2]],collidedTriPoint,collidedCylPoint);
			if((collidedCylPoint-collidedTriPoint).norm()<Length)
			{

				Length=(collidedCylPoint-collidedTriPoint).norm();
				CylIndex=i;
				MinCylPoint=collidedCylPoint;
				MinTriPoint=collidedTriPoint;
			}	

			if(Length<minimumDistance)
			{
				InspectedTri.push_back(j);
				dis.triIdx=j;
				dis.cylinderIdx=CylIndex;
				Vec3d CollidedDirec=MinTriPoint-MinCylPoint;
				CollidedDirec.normalize();
				dis.measuredPointInCylinder=MinCylPoint;
				dis.collidedCylPoint=MinCylPoint+CollidedDirec*(minimumDistance-Length);
				dis.collidedTriPoint=MinTriPoint;
				dis.Distance=Length;
				dis.PenetratedDirec=-CollidedDirec;
				dis.Penetration=Length-cont->returnRadiusOfEndoscope();

				distance.push_back(dis);
			}
		}

	}

	if(distance.size())
		return true;
	else
		return false;

}

bool SignoriniCollision::LatticeBasedCollisionDetectionUsingDistanceEndoscope(MyFFD* surf,MyFFD* cont)
{
	Vec3d* SurfacePoint=surf->returnSurfacePoint();
	int NbSurfacePoint=surf->returnNbSurface();
	int NbCenterPoint=surf->returnNbCenter();
	int NbCircularPoint=surf->returnNbCircular();
	int NbFace=surf->returnNbFace();
	int* FaceIndex=surf->returnFaceIndex();
	int NbCen=surf->returnNbCenter();
	Vec3i* Face=surf->returnFace();
	Vec3d* Points=cont->returnControlPoint();
	int NbPoint=cont->returnNbPoint();
	
	Vec3d InsertionPoint=cont->returnInsertedPoint();
	Vec3d InsertionDirec=cont->returnInsertedDirec();

	Vec3d p1,p2,p3,p4;
	int Margin=4;

	TriInHyper.clear();

	Vec3d collidedCylPoint;
	Vec3d collidedTriPoint;
	Vec3d MinCylPoint;
	Vec3d MinTriPoint;

	Vec3d normal1,normal2,normal3,normal4;
	int CylIndex;
	double Length;
	double minimumDistance=cont->returnRadiusOfEndoscope();
	
	bool flag;

	distance.clear();
	int InsertedIndex=0;
	for(int i=0;i<NbPoint;i++){
		if((Points[i]-InsertionPoint)*InsertionDirec>=0){
			InsertedIndex=i; 
		}else{
			break;
		}
	}


	for(int i=0;i<NbCen-2;i++)
	{
		for(int j=0;j<6;j++)
		{

			p1=surf->paraToSurfacePoint(i,j,0,0,1);
			p2=surf->paraToSurfacePoint(i,j,1,1,1);
			p3=surf->paraToSurfacePoint(i,j,1,0,1);
			normal1=(p3-p1).cross(p2-p1);
			p1=surf->paraToSurfacePoint(i,j,0,0,0);
			p2=surf->paraToSurfacePoint(i,j,1,1,0);
			p3=surf->paraToSurfacePoint(i,j,1,0,0);
			normal2=(p3-p1).cross(p2-p1);
			p1=surf->paraToSurfacePoint(i,j,0,0,1);
			p2=surf->paraToSurfacePoint(i,j,0,0,0);
			p3=surf->paraToSurfacePoint(i,j,0.5,0,0.5);
			normal3=(p3-p1).cross(p2-p1);
			p1=surf->paraToSurfacePoint(i,j,0,0,1);
			p2=surf->paraToSurfacePoint(i,j,0,0,0);
			p3=surf->paraToSurfacePoint(i,j,0.5,1,0.5);
			normal4=(p3-p1).cross(p2-p1);
			for(int k=0;k<InsertedIndex;k++)
			{
				flag=false;
				p1=surf->paraToSurfacePoint(i,j,0,0,1);
				p2=surf->paraToSurfacePoint(i,j,0,0,0);
				if(((Points[k]-p1)*normal1)<=0 && ((Points[k]-p2)*normal2)>=0){
					p1=surf->paraToSurfacePoint(i,j,0.5,0,0.5);
					p2=surf->paraToSurfacePoint(i,j,0.5,1,0.5);
					if(((Points[k]-p1)*normal3)>=0 && ((Points[k]-p2)*normal4)<=0)
						flag=true;
				}
				if(flag)
					TriInHyper.push_back(6*i+j);
			}
		}
	}

	Distancefield dis;
	int j,i;


	for (j= 0;j<TriInHyper.size();j++){
		for(int k=0;k<TriIndex[TriInHyper[j]].size();k++){
			Length=10000000000000;
		
			for(i=InsertedIndex;i<NbPoint-1;i++){
				measureMinimumDistanceBetLineandTriangle(Points[i],Points[i+1],SurfacePoint[Face[TriIndex[TriInHyper[j]][k]][0]],SurfacePoint[Face[TriIndex[TriInHyper[j]][k]][1]],SurfacePoint[Face[TriIndex[TriInHyper[j]][k]][2]],collidedTriPoint,collidedCylPoint);
				if((collidedCylPoint-collidedTriPoint).norm()<Length)
				{
					Length=(collidedCylPoint-collidedTriPoint).norm();
					CylIndex=i;
					MinCylPoint=collidedCylPoint;
					MinTriPoint=collidedTriPoint;
				}	
			}
			if(Length<minimumDistance)
			{

				dis.triIdx=j;
				dis.cylinderIdx=CylIndex;
				Vec3d CollidedDirec=MinTriPoint-MinCylPoint;
				CollidedDirec.normalize();
				dis.measuredPointInCylinder=MinCylPoint;
				dis.collidedCylPoint=MinCylPoint+CollidedDirec*(minimumDistance-Length);
				dis.collidedTriPoint=MinTriPoint;
				dis.Distance=Length;
				dis.PenetratedDirec=-CollidedDirec;
				dis.Penetration=minimumDistance-Length;

				distance.push_back(dis);
			}
		}
	}

	if(distance.size())
		return true;
	else
		return false;

}

bool SignoriniCollision::SimpleLatticeBasedCollisionDetectionUsingDistanceEndoscope(MyFFD* surf,MyFFD* cont)
{
	Vec3d* SurfacePoint=surf->returnSurfacePoint();
	int NbSurfacePoint=surf->returnNbSurface();
	int NbCenterPoint=surf->returnNbCenter();
	int NbCircularPoint=surf->returnNbCircular();
	int NbFace=surf->returnNbFace();
	int* FaceIndex=surf->returnFaceIndex();
	int NbCen=surf->returnNbCenter();
	Vec3i* Face=surf->returnFace();
	Vec3d* Points=cont->returnControlPoint();
	int NbPoint=cont->returnNbPoint();

	Vec3d InsertionPoint=cont->returnInsertedPoint();
	Vec3d InsertionDirec=cont->returnInsertedDirec();

	Vec3d p1,p2,p3,p4;
	int Margin=4;

	TriInHyper.clear();

	Vec3d collidedCylPoint;
	Vec3d collidedTriPoint;
	Vec3d MinCylPoint;
	Vec3d MinTriPoint;

	Vec3d normal1,normal2,normal3,normal4;
	int CylIndex;
	double Length;
	double minimumDistance=cont->returnRadiusOfEndoscope();

	bool flag;

	distance.clear();
	int InsertedIndex=0;
	for(int i=0;i<NbPoint;i++){
		if((Points[i]-InsertionPoint)*InsertionDirec>=0){
			InsertedIndex=i; 
		}else{
			break;
		}
	}


	for(int i=0;i<InsertedIndex;i++)
		TriInHyper.push_back(surf->findCloestCircularPointIndex(Points[i]));

	Distancefield dis;
	int j,i;


	for (j= 0;j<TriInHyper.size();j++){
		for(int k=0;k<TriIndex[TriInHyper[j]].size();k++){
			Length=10000000000000;

			for(i=InsertedIndex;i<NbPoint-1;i++){
				measureMinimumDistanceBetLineandTriangle(Points[i],Points[i+1],SurfacePoint[Face[TriIndex[TriInHyper[j]][k]][0]],SurfacePoint[Face[TriIndex[TriInHyper[j]][k]][1]],SurfacePoint[Face[TriIndex[TriInHyper[j]][k]][2]],collidedTriPoint,collidedCylPoint);
				if((collidedCylPoint-collidedTriPoint).norm()<Length)
				{
					Length=(collidedCylPoint-collidedTriPoint).norm();
					CylIndex=i;
					MinCylPoint=collidedCylPoint;
					MinTriPoint=collidedTriPoint;
				}	
			}
			if(Length<minimumDistance)
			{

				dis.triIdx=j;
				dis.cylinderIdx=CylIndex;
				Vec3d CollidedDirec=MinTriPoint-MinCylPoint;
				CollidedDirec.normalize();
				dis.measuredPointInCylinder=MinCylPoint;
				dis.collidedCylPoint=MinCylPoint+CollidedDirec*(minimumDistance-Length);
				dis.collidedTriPoint=MinTriPoint;
				dis.Distance=Length;
				dis.PenetratedDirec=-CollidedDirec;
				dis.Penetration=minimumDistance-Length;

				distance.push_back(dis);
			}
		}
	}

	if(distance.size())
		return true;
	else
		return false;

}

void SignoriniCollision::measureMinimumDistanceBetLineandTriangle(Vec3d p1, Vec3d p2,Vec3d tri1, Vec3d tri2, Vec3d tri3, Vec3d &collidedTri, Vec3d &collidedCyl)
{
	Vec3d direc=(p2-p1);
	direc.normalize();
	double Length;
	Vec3d tripoint[3];

	Vec3d collidedTriVertex;
	Vec3d collidedTriEdge;
	Vec3d collidedTriPlane;
	Vec3d collidedTriPoint;
	Vec3d collidedCylinder;
	Vec3d measuredCylinder;
	tripoint[0]=tri1;
	tripoint[1]=tri2;
	tripoint[2]=tri3;

	double L;

	//Point and Line


	Length=1000000000000;

	for(int i=0;i<3;i++)
	{
		collidedCylinder=p1+direc*((tripoint[i]-p1)*direc);
		if((collidedCylinder-p1)*(collidedCylinder-p2)<0)
		{
			L=(collidedCylinder-tripoint[i]).norm();
			if(L<Length){
				measuredCylinder=collidedCylinder;
				collidedTriPoint=tripoint[i];
				Length=L;
			}
		}
	}


	//Line and Line
	int ii[2];

	for(int i=0;i<3;i++)
	{

		ii[0]=i%3;
		ii[1]=(i+1)%3;
		collidedTriEdge=findPointBetweenTwoline1(tripoint[ii[0]],tripoint[ii[1]],p1,p2);
		collidedCylinder=p1+direc*((collidedTriEdge-p1)*direc);
		if((collidedTriEdge-tripoint[ii[0]])*(collidedTriEdge-tripoint[ii[1]])<0)
		{
			if((collidedCylinder-p1)*(collidedCylinder-p2)<0)
			{
				L=(collidedCylinder-collidedTriEdge).norm();
				if(L<Length){
					Length=L;
					measuredCylinder=collidedCylinder;
					collidedTriPoint=collidedTriEdge;
				}
			}
		}
	}

	//end point of line element and triangle

	collidedTriPlane=findClosestPointInTri(p1,tri1,tri2,tri3);
	if((p1-collidedTriPlane).norm()<Length)
	{
		Length=(p1-collidedTriPlane).norm();
		measuredCylinder=p1;
		collidedTriPoint=collidedTriPlane;
	}	

	collidedTriPlane=findClosestPointInTri(p2,tri1,tri2,tri3);

	if((p2-collidedTriPlane).norm()<Length)
	{
		Length=(p2-collidedTriPlane).norm();
		measuredCylinder=p2;
		collidedTriPoint=collidedTriPlane;
	}

	//return

	collidedTri=collidedTriPoint;
	collidedCyl=measuredCylinder;

}

void SignoriniCollision::measureMinimumDistanceBetPointandTriangle(Vec3d p1, Vec3d tri1, Vec3d tri2, Vec3d tri3, Vec3d &collidedTri, Vec3d &collidedCyl)
{
	double Length;
	Vec3d tripoint[3];

	Vec3d collidedTriPoint;
	Vec3d collidedTriPlane;
	Vec3d measuredCylinder;
	tripoint[0]=tri1;
	tripoint[1]=tri2;
	tripoint[2]=tri3;

	double L;

	//Point and Line


	Length=1000000000000;


	//end point of line element and triangle

	collidedTriPlane=findClosestPointInTri(p1,tri1,tri2,tri3);
	if((p1-collidedTriPlane).norm()<Length)
	{
		Length=(p1-collidedTriPlane).norm();
		measuredCylinder=p1;
		collidedTriPoint=collidedTriPlane;
	}	

	//return

	collidedTri=collidedTriPoint;
	collidedCyl=measuredCylinder;

}


Vec3d SignoriniCollision::findClosestPointInTri(Vec3d point, Vec3d tri1, Vec3d tri2, Vec3d tri3)
{
	double Length=1000000000000000;

	Vec3d Normal=(tri2-tri1).cross(tri3-tri1);
	Normal.normalize();
	Vec3d Center=tri1;
	Vec3d ProjectedPoint=findProjectedPointToPlane(point,Normal,Center);
	Vec3d tri[3];
	int index=0;
	tri[0]=tri1;
	tri[1]=tri2;
	tri[2]=tri3;
	//Á¡-¸é

	double flag1=((tri2-tri1).cross(ProjectedPoint-tri1))*((ProjectedPoint-tri1).cross(tri3-tri1));
	double flag2=((tri3-tri2).cross(ProjectedPoint-tri2))*((ProjectedPoint-tri2).cross(tri1-tri2));

	if(flag1>=0 && flag2>=0)
	{
		return ProjectedPoint;
	}
	else
	{
		for(int i=0;i<3;i++)
		{
			if((tri[i]-point).norm()<Length)
			{
				Length=(tri[i]-point).norm();
				index=i;
			}
		}

		return tri[index];
		
	}
}




void SignoriniCollision::ComputeforcefromComplianceV19(MyFFD* surf,MyFFD* cont)
{
	int Nb1=surf->returnNbPoint();
	int Nb2=cont->returnNbPoint();
	int nbCollide=distance.size();	

	mat m1,m2,C1,C2,mm1,mm2;
	LARGE_INTEGER freq,start,end;
	double time1,time2,time3,time4,time5,time6,time7;
	mat force;
	force.set_size(nbCollide);
	force.fill(0.0);

	if(QueryPerformanceFrequency(&freq)) 
	{
	}
	
	mat penetration;
	penetration.set_size(nbCollide);
	for(int i=0;i<nbCollide;i++)
		penetration(i)=distance[i].Penetration;

	m1.set_size(nbCollide,3*Nb1);
	m2.set_size(nbCollide,3*Nb2);

	mm1.set_size(nbCollide,3*Nb1);
	mm2.set_size(nbCollide,3*Nb2);

	m1.fill(0.0);
	m2.fill(0.0);

	mm1.fill(0.0);
	mm2.fill(0.0);

	mat mcm(nbCollide,nbCollide);
	mcm.fill(0.0);


	QueryPerformanceCounter(&start);
	Vec3d normal;

	for(int i=0;i<nbCollide;i++)
	{
		normal=distance[i].PenetratedDirec;
		m1.submat(i,0,i,3*Nb1-1)=trans(surf->returnMappingMatforOneSurface(distance[i].triIdx,distance[i].collidedTriPoint,normal));
		m2.submat(i,0,i,3*Nb2-1)=trans(cont->returnMappingMatforOneLattice(distance[i].cylinderIdx,distance[i].measuredPointInCylinder,normal));
	}
	
	QueryPerformanceCounter(&end);
	time1=(double)(end.QuadPart-start.QuadPart)/freq.QuadPart*1000;

	QueryPerformanceCounter(&start);
	mat InternalForce1,InternalForce2,InternalDis1,InternalDis2,Gravity1;
	InternalForce1.set_size(3*Nb1);
	InternalForce2.set_size(3*Nb2);
	InternalDis1.set_size(3*Nb1);
	InternalDis2.set_size(3*Nb2);


	surf->addForcetoAllPoint(Vec3d(0,0,-0.1));
	InternalForce1=surf->returnInternalExplicitForceOfUpper();
	InternalForce2=cont->returnInternalExplicitForceOfEndoscope();

	C1=surf->returnVertorFormExplicitCompliancematrixForUpper();
	C2=cont->returnVectorFormExplicitCompliancematrixForEndoscope();


	InternalDis1=C1%InternalForce1-surf->returnPreDisplacement();
	InternalDis2=C2%InternalForce2-cont->returnPreDisplacement();
	QueryPerformanceCounter(&end);
	time2=(double)(end.QuadPart-start.QuadPart)/freq.QuadPart*1000;

	QueryPerformanceCounter(&start);
	penetration+=-m1*InternalDis1+m2*InternalDis2;
	
	 
	//Gauss Seidel 

	int Iteration=15;
	for(int i=0;i<nbCollide;i++){
		mm1.submat(i,0,i,3*Nb1-1)=m1.submat(i,0,i,3*Nb1-1)%trans(C1);
		mm2.submat(i,0,i,3*Nb2-1)=m2.submat(i,0,i,3*Nb2-1)%trans(C2);
	}

	mcm=mm1*trans(m1)+mm2*trans(m2);
	
	
	QueryPerformanceCounter(&end);
	time3=(double)(end.QuadPart-start.QuadPart)/freq.QuadPart*1000;

	QueryPerformanceCounter(&start);
	mat Displacement1,Displacement2;
	Displacement1=InternalDis1;
	Displacement2=InternalDis2;
	if(nbCollide)
	{
		force=GaussSeidelMethod1(mcm,penetration,Iteration);
		Displacement1-=trans(mm1)*force;
		Displacement2+=trans(mm2)*force;
	}
	QueryPerformanceCounter(&end);
	time4=(double)(end.QuadPart-start.QuadPart)/freq.QuadPart*1000;
	

	QueryPerformanceCounter(&start);
	surf->addDisplacementToControlPoint(Displacement1);
	cont->addDisplacementToControlPoint(Displacement2);
	surf->updateSurfacePointusingMappingMatrix();
	QueryPerformanceCounter(&end);
	time5=(double)(end.QuadPart-start.QuadPart)/freq.QuadPart*1000;

	fprintf(fp,"%f %f%d\n",time2+time4,time1+time3+time5,nbCollide);

}

void SignoriniCollision::ComputeforceImplicit(MyFFD* surf,MyFFD* cont)
{
	int Nb1=surf->returnNbPoint();
	int Nb2=cont->returnNbPoint();
	int nbCollide=distance.size();	

	mat m1,m2,C1,C2;
	LARGE_INTEGER freq,start,end;
	double time1,time2,time3,time4,time5,time6,time7;
	mat force;
	force.set_size(nbCollide);
	force.fill(0.0);

	if(QueryPerformanceFrequency(&freq)) 
	{
	}

	mat penetration;
	penetration.set_size(nbCollide);
	for(int i=0;i<nbCollide;i++)
		penetration(i)=distance[i].Penetration;

	m1.set_size(nbCollide,3*Nb1);
	m2.set_size(nbCollide,3*Nb2);

	m1.fill(0.0);
	m2.fill(0.0);

	C1.set_size(3*Nb1,3*Nb1);
	C2.set_size(3*Nb2,3*Nb2);

	C1.fill(0.0);
	C2.fill(0.0);

	mat mcm(nbCollide,nbCollide);
	mcm.fill(0.0);


	QueryPerformanceCounter(&start);
	Vec3d normal;

	for(int i=0;i<nbCollide;i++)
	{
		normal=distance[i].PenetratedDirec;
		m1.submat(i,0,i,3*Nb1-1)=trans(surf->returnMappingMatforOneSurface(distance[i].triIdx,distance[i].collidedTriPoint,normal));
		m2.submat(i,0,i,3*Nb2-1)=trans(cont->returnMappingMatforOneLattice(distance[i].cylinderIdx,distance[i].measuredPointInCylinder,normal));
	}

	QueryPerformanceCounter(&end);
	time1=(double)(end.QuadPart-start.QuadPart)/freq.QuadPart*1000;

	QueryPerformanceCounter(&start);
	mat InternalForce1,InternalForce2,InternalDis1,InternalDis2,Gravity1;
	InternalForce1.set_size(3*Nb1);
	InternalForce2.set_size(3*Nb2);
	InternalDis1.set_size(3*Nb1);
	InternalDis2.set_size(3*Nb2);
	Gravity1.set_size(3*Nb1);
	Gravity1.fill(-50);

	InternalForce1=surf->returnInternalImplicitForceOfUpper()+Gravity1;
	InternalForce2=cont->returnInternalImplicitForceOfEndoscope();

	C1=surf->returnImplicitDynamicStiffnessMatrixForUpper();
	C2=cont->returnImplicitDynamicStiffnessMatrixForEndoscope();
	

	/*Rmat RC1,RC2;
	RC1.set_size(6,426);
	RC2.set_size(3,)*/
	InternalDis1=solve(C1,InternalForce1)-surf->returnPreDisplacement();
	InternalDis2=solve(C2,InternalForce2)-cont->returnPreDisplacement();
	QueryPerformanceCounter(&end);
	time2=(double)(end.QuadPart-start.QuadPart)/freq.QuadPart*1000;

	QueryPerformanceCounter(&start);
	penetration+=-m1*InternalDis1+m2*InternalDis2;


	//Gauss Seidel 

	int Iteration=15;
	

	mcm=m1*solve(C1,trans(m1))+m2*solve(C2,trans(m2));


	QueryPerformanceCounter(&end);
	time3=(double)(end.QuadPart-start.QuadPart)/freq.QuadPart*1000;

	QueryPerformanceCounter(&start);
	mat Displacement1,Displacement2;
	Displacement1=InternalDis1;
	Displacement2=InternalDis2;
	if(nbCollide)
	{
		force=GaussSeidelMethod1(mcm,penetration,Iteration);
		Displacement1-=solve(C1,trans(m1)*force);
		Displacement2+=solve(C2,trans(m2)*force);
	}
	QueryPerformanceCounter(&end);
	time4=(double)(end.QuadPart-start.QuadPart)/freq.QuadPart*1000;


	QueryPerformanceCounter(&start);
	surf->addDisplacementToControlPoint(Displacement1);
	cont->addDisplacementToControlPoint(Displacement2);
	surf->updateSurfacePointusingMappingMatrix();
	QueryPerformanceCounter(&end);
	time5=(double)(end.QuadPart-start.QuadPart)/freq.QuadPart*1000;

	fprintf(fp,"%f %f %f %f %d\n",time1,time2,time3,time4,nbCollide);

}

void SignoriniCollision::ComputeforcefromComplianceV20(MyFFD* surf,MyFFD* cont)
{
	int Nb1=surf->returnNbPoint();
	int Nb2=cont->returnNbPoint();
	int nbCollide=distance.size();	

	//Set Variables
	mat m1,m2,C1,C2;
	LARGE_INTEGER freq,start,end;
	double time1,time2,time3,time4,time5,time6,time7;
	mat force,InternalForce1,InternalForce2,InternalDis1,InternalDis2,Gravity1,penetration;
	force.set_size(nbCollide);
	force.fill(0.0);
	InternalForce1.set_size(3*Nb1);
	InternalForce2.set_size(3*Nb2);
	InternalDis1.set_size(3*Nb1);
	InternalDis2.set_size(3*Nb2);
	Gravity1.set_size(3*Nb1);
	Gravity1.fill(-50);
	m1.set_size(nbCollide,3*Nb1);
	m2.set_size(nbCollide,3*Nb2);
	m1.fill(0.0);
	m2.fill(0.0);


	mat mcm(nbCollide,nbCollide);
	mcm.fill(0.0);

	penetration.set_size(nbCollide);
	for(int i=0;i<nbCollide;i++)
		penetration(i)=distance[i].Penetration;

	if(QueryPerformanceFrequency(&freq)) 
	{
	}


	QueryPerformanceCounter(&start);
	Vec3d normal;

	for(int i=0;i<nbCollide;i++)
	{
		normal=distance[i].PenetratedDirec;
		m1.submat(i,0,i,3*Nb1-1)=trans(surf->returnMappingMatforOneSurface(distance[i].triIdx,distance[i].collidedTriPoint,normal));
		m2.submat(i,0,i,3*Nb2-1)=trans(cont->returnMappingMatforOneLattice(distance[i].cylinderIdx,distance[i].measuredPointInCylinder,normal));
	}

	QueryPerformanceCounter(&end);
	time1=(double)(end.QuadPart-start.QuadPart)/freq.QuadPart*1000;

	QueryPerformanceCounter(&start);
	

	InternalForce1=surf->returnInternalImplicitForceOfUpper()+Gravity1;
	InternalForce2=cont->returnInternalImplicitForceOfEndoscope();

	C1=surf->returnImplicitDynamicStiffnessMatrixForUpper();
	C2=cont->returnImplicitDynamicStiffnessMatrixForEndoscope();

	Rmat RC1,RC2;
	RC1.set(21,C1.n_rows);
	RC2.set(3,C2.n_rows);
	RC1.add(C1);
	RC2.add(C2);

	InternalDis1=RC1.Solve(InternalForce1)-surf->returnPreDisplacement();
	InternalDis2=RC2.Solve(InternalForce2)-cont->returnPreDisplacement();
	QueryPerformanceCounter(&end);
	time2=(double)(end.QuadPart-start.QuadPart)/freq.QuadPart*1000;
	
	QueryPerformanceCounter(&start);
	if(nbCollide)
		penetration+=-m1*InternalDis1+m2*InternalDis2;


	//Gauss Seidel 
	int Iteration=15;

	QueryPerformanceCounter(&end);
	time3=(double)(end.QuadPart-start.QuadPart)/freq.QuadPart*1000;

	QueryPerformanceCounter(&start);
	mat Displacement1,Displacement2;
	Displacement1=InternalDis1;
	Displacement2=InternalDis2;
	
	if(nbCollide)
	{
        mcm=m1*RC1.Resolve(trans(m1))+m2*RC2.Resolve(trans(m2));
		force=GaussSeidelMethod1(mcm,penetration,Iteration);
		Displacement1-=trans(m1)*force;
		Displacement2+=trans(m2)*force;
	}
	QueryPerformanceCounter(&end);
	time4=(double)(end.QuadPart-start.QuadPart)/freq.QuadPart*1000;


	QueryPerformanceCounter(&start);
	surf->addDisplacementToControlPoint(Displacement1);
	cont->addDisplacementToControlPoint(Displacement2);
	surf->updateSurfacePointusingMappingMatrix();
	QueryPerformanceCounter(&end);
	time5=(double)(end.QuadPart-start.QuadPart)/freq.QuadPart*1000;

	fprintf(fp,"%f %f %f %f %d\n",time1,time2,time3,time4,nbCollide);

}

void SignoriniCollision::ComputeforcefromComplianceV21(MyFFD* surf,MyFFD* cont)
{
	//Compute only force, doesn't update
	int Nb1=surf->returnNbPoint();
	int Nb2=cont->returnNbPoint();
	int nbCollide=distance.size();	

	mat m1,m2,C1,C2,mm1,mm2;
	LARGE_INTEGER freq,start,end;
	mat force;
	force.set_size(nbCollide);
	force.fill(0.0);


	mat penetration;
	penetration.set_size(nbCollide);
	for(int i=0;i<nbCollide;i++)
		penetration(i)=distance[i].Penetration;

	m1.set_size(nbCollide,3*Nb1);
	m2.set_size(nbCollide,3*Nb2);

	mm1.set_size(nbCollide,3*Nb1);
	mm2.set_size(nbCollide,3*Nb2);

	m1.fill(0.0);
	m2.fill(0.0);

	mm1.fill(0.0);
	mm2.fill(0.0);

	mat mcm(nbCollide,nbCollide);
	mcm.fill(0.0);

	Vec3d normal;
	for(int i=0;i<nbCollide;i++)
	{
		normal=distance[i].PenetratedDirec;
		m1.submat(i,0,i,3*Nb1-1)=trans(surf->returnMappingMatforOneSurface(distance[i].triIdx,distance[i].collidedTriPoint,normal));
		m2.submat(i,0,i,3*Nb2-1)=trans(cont->returnMappingMatforOneLattice(distance[i].cylinderIdx,distance[i].measuredPointInCylinder,normal));
	}


	C1=surf->returnVertorFormExplicitCompliancematrixForUpper();
	C2=cont->returnVectorFormExplicitCompliancematrixForEndoscope();



	//Gauss Seidel 

	int Iteration=15;
	for(int i=0;i<nbCollide;i++){
		mm1.submat(i,0,i,3*Nb1-1)=m1.submat(i,0,i,3*Nb1-1)%trans(C1);
		mm2.submat(i,0,i,3*Nb2-1)=m2.submat(i,0,i,3*Nb2-1)%trans(C2);
	}

	mcm=mm1*trans(m1)+mm2*trans(m2);

	mat force1,force2;

	if(nbCollide)
	{
		force=GaussSeidelMethod1(mcm,penetration,Iteration);
		force1=-trans(m1)*force;
		force2=trans(m2)*force;
	}


	surf->addForcetoAllPoint(force1);
	cont->addForcetoAllPoint(force2);




}

void SignoriniCollision::InteractionSimulation(MyFFD* surf,MyFFD* endo, MyFFD* cath)
{

	mat m1,m2,m3,C1,C2,C3,mm1,mm2,mm3;
	mat force1,force2;
	mat InternalForce1,InternalForce2,InternalForce3,InternalDis1,InternalDis2,InternalDis3,Gravity1;

	int Nb1=surf->returnNbPoint();
	int Nb2=endo->returnNbPoint();
	int Nb3=cath->returnNbPoint();
	int nbCollide;

	bool flag;
	mat penetration,mcm;
	Vec3d normal;
	int Iteration=15;

	InternalForce1.set_size(3*Nb1); InternalForce2.set_size(3*Nb2); InternalForce3.set_size(3*Nb3);
	InternalDis1.set_size(3*Nb1);   InternalDis2.set_size(3*Nb2);   InternalDis3.set_size(3*Nb3);

	surf->addForcetoAllPoint(Vec3d(0,0,-0.1));
	InternalForce1=surf->returnInternalExplicitForceOfUpper();
	InternalForce2=endo->returnInternalExplicitForceOfEndoscope();
	InternalForce3=cath->returnInternalExplicitForceOfEndoscope();

	C1=surf->returnVertorFormExplicitCompliancematrixForUpper();
	C2=endo->returnVectorFormExplicitCompliancematrixForEndoscope();
	C3=cath->returnVectorFormExplicitCompliancematrixForEndoscope();

	InternalDis1=C1%InternalForce1-surf->returnPreDisplacement();
	InternalDis2=C2%InternalForce2-endo->returnPreDisplacement();
	InternalDis3=C3%InternalForce3-cath->returnPreDisplacement();


	flag=PotentialCollisionDetection(surf,endo,5,1.3);
	if(flag)
	{
		nbCollide=distance.size();	
		force1.set_size(nbCollide); force1.fill(0.0);

		
		penetration.set_size(nbCollide);
		for(int i=0;i<nbCollide;i++)
			penetration(i)=distance[i].Penetration;

		m1.set_size(nbCollide,3*Nb1); m1.fill(0.0); mm1.set_size(nbCollide,3*Nb1); mm1.fill(0.0);
		m2.set_size(nbCollide,3*Nb2); m2.fill(0.0); mm2.set_size(nbCollide,3*Nb2); mm2.fill(0.0);

		

		
		for(int i=0;i<nbCollide;i++)
		{
			normal=distance[i].PenetratedDirec;
			m1.submat(i,0,i,3*Nb1-1)=trans(surf->returnMappingMatforOneSurface(distance[i].triIdx,distance[i].collidedTriPoint,normal));
			m2.submat(i,0,i,3*Nb2-1)=trans(endo->returnMappingMatforOneLattice(distance[i].cylinderIdx,distance[i].measuredPointInCylinder,normal));
		}

		penetration+=-m1*InternalDis1+m2*InternalDis2;


		//Gauss Seidel 
		
		for(int i=0;i<nbCollide;i++){
			mm1.submat(i,0,i,3*Nb1-1)=m1.submat(i,0,i,3*Nb1-1)%trans(C1);
			mm2.submat(i,0,i,3*Nb2-1)=m2.submat(i,0,i,3*Nb2-1)%trans(C2);
		}

		mcm.set_size(nbCollide,nbCollide);
		mcm.fill(0.0);
		mcm=mm1*trans(m1)+mm2*trans(m2);

		force1=GaussSeidelMethod1(mcm,penetration,Iteration);
		InternalDis1-=trans(mm1)*force1;
		InternalDis2+=trans(mm2)*force1;
	}
	
	if(PotentialCollisionDetection(surf,cath,1,5))
	{


		nbCollide=distance.size();	
		force2.set_size(nbCollide); force2.fill(0.0);

		penetration.set_size(nbCollide);
		for(int i=0;i<nbCollide;i++)
			penetration(i)=distance[i].Penetration;

		m1.set_size(nbCollide,3*Nb1); m1.fill(0.0); mm1.set_size(nbCollide,3*Nb1); mm1.fill(0.0);
		m2.set_size(nbCollide,3*Nb3); m2.fill(0.0); mm2.set_size(nbCollide,3*Nb3); mm2.fill(0.0);

		for(int i=0;i<nbCollide;i++)
		{
			normal=distance[i].PenetratedDirec;
			m1.submat(i,0,i,3*Nb1-1)=trans(surf->returnMappingMatforOneSurface(distance[i].triIdx,distance[i].collidedTriPoint,normal));
			m2.submat(i,0,i,3*Nb3-1)=trans(endo->returnMappingMatforOneLattice(distance[i].cylinderIdx,distance[i].measuredPointInCylinder,normal));
		}
		penetration+=-m1*InternalDis1+m2*InternalDis3;


		//Gauss Seidel 
		for(int i=0;i<nbCollide;i++){
			mm1.submat(i,0,i,3*Nb1-1)=m1.submat(i,0,i,3*Nb1-1)%trans(C1);
			mm2.submat(i,0,i,3*Nb3-1)=m2.submat(i,0,i,3*Nb3-1)%trans(C3);
		}

		mcm.set_size(nbCollide,nbCollide);
		mcm.fill(0.0);
		mcm=mm1*trans(m1)+mm2*trans(m2);


		force2=GaussSeidelMethod1(mcm,penetration,Iteration);
		InternalDis1-=trans(mm1)*force2;
		InternalDis3+=trans(mm2)*force2;
		
		mat transferForce=trans(m2)*force2;

		Vec3d Contactforce;
		Contactforce.clear();
		for(int i=0;i<Nb3;i++)
			for(int j=0;j<3;j++)
				Contactforce[j]+=transferForce(3*i+j);

	}

	surf->addDisplacementToControlPoint(InternalDis1);
	endo->addDisplacementToControlPoint(InternalDis2);
	cath->addDisplacementToControlPoint(InternalDis3);
	surf->updateSurfacePointusingMappingMatrix();
}

void SignoriniCollision::InteractionSimulationV2(MyFFD* surf,MyFFD* endo, MyFFD* cath)
{

	mat m1,m2,m3,C1,C2,C3,mm1,mm2,mm3;
	mat force1,force2;
	mat InternalForce1,InternalForce2,InternalForce3,InternalDis1,InternalDis2,InternalDis3,Gravity1;

	int Nb1=surf->returnNbPoint();
	int Nb2=endo->returnNbPoint();
	int Nb3=cath->returnNbPoint();
	int nbCollide;

	bool flag;
	mat penetration,mcm;
	Vec3d normal;
	int Iteration=15;

	InternalForce1.set_size(3*Nb1); InternalForce2.set_size(3*Nb2); InternalForce3.set_size(3*Nb3);
	InternalForce1.fill(0.0); InternalForce2.fill(0.0); InternalForce3.fill(0.0);	

	surf->addForcetoAllPoint(Vec3d(0,0,-0.1));

	C1=surf->returnVertorFormExplicitCompliancematrixForUpper();
	C2=endo->returnVectorFormExplicitCompliancematrixForEndoscope();
	C3=cath->returnVectorFormExplicitCompliancematrixForEndoscope();

	if(PotentialCollisionDetection(surf,endo,5,1.3))
	{
		nbCollide=distance.size();	
		force1.set_size(nbCollide); force1.fill(0.0);


		penetration.set_size(nbCollide);
		for(int i=0;i<nbCollide;i++)
			penetration(i)=distance[i].Penetration;

		//m1.set_size(nbCollide,3*Nb1); m1.fill(0.0); mm1.set_size(nbCollide,3*Nb1); mm1.fill(0.0);
		//m2.set_size(nbCollide,3*Nb2); m2.fill(0.0); mm2.set_size(nbCollide,3*Nb2); mm2.fill(0.0);

		//for(int i=0;i<nbCollide;i++)
		//{
		//	normal=distance[i].PenetratedDirec;
		//	m1.submat(i,0,i,3*Nb1-1)=trans(surf->returnMappingMatforOneSurface(distance[i].triIdx,distance[i].collidedTriPoint,normal));
		//	m2.submat(i,0,i,3*Nb2-1)=trans(endo->returnMappingMatforOneLattice(distance[i].cylinderIdx,distance[i].measuredPointInCylinder,normal));
		//}

		//

		//for(int i=0;i<nbCollide;i++){
		//	mm1.submat(i,0,i,3*Nb1-1)=m1.submat(i,0,i,3*Nb1-1)%trans(C1);
		//	mm2.submat(i,0,i,3*Nb2-1)=m2.submat(i,0,i,3*Nb2-1)%trans(C2);
		//}

		//mcm.set_size(nbCollide,nbCollide);
		//mcm.fill(0.0);
		//mcm=mm1*trans(m1)+mm2*trans(m2);

		////Gauss Seidel 
		//force1=GaussSeidelMethod1(mcm,penetration,Iteration);
		//InternalForce1-=trans(m1)*force1;
		//InternalForce2+=trans(m2)*force1;

		PenaltyMethod(surf,endo,100);
	}

	if(PotentialCollisionDetection(surf,cath,1,5))
	{


		nbCollide=distance.size();	
		force2.set_size(nbCollide); force2.fill(0.0);

		penetration.set_size(nbCollide);
		for(int i=0;i<nbCollide;i++)
			penetration(i)=distance[i].Penetration;

		//m1.set_size(nbCollide,3*Nb1); m1.fill(0.0); mm1.set_size(nbCollide,3*Nb1); mm1.fill(0.0);
		//m2.set_size(nbCollide,3*Nb3); m2.fill(0.0); mm2.set_size(nbCollide,3*Nb3); mm2.fill(0.0);

		//for(int i=0;i<nbCollide;i++)
		//{
		//	normal=distance[i].PenetratedDirec;
		//	m1.submat(i,0,i,3*Nb1-1)=trans(surf->returnMappingMatforOneSurface(distance[i].triIdx,distance[i].collidedTriPoint,normal));
		//	m2.submat(i,0,i,3*Nb3-1)=trans(endo->returnMappingMatforOneLattice(distance[i].cylinderIdx,distance[i].measuredPointInCylinder,normal));
		//}

		////Gauss Seidel 
		//for(int i=0;i<nbCollide;i++){
		//	mm1.submat(i,0,i,3*Nb1-1)=m1.submat(i,0,i,3*Nb1-1)%trans(C1);
		//	mm2.submat(i,0,i,3*Nb3-1)=m2.submat(i,0,i,3*Nb3-1)%trans(C3);
		//}

		//mcm.set_size(nbCollide,nbCollide);
		//mcm.fill(0.0);
		//mcm=mm1*trans(m1)+mm2*trans(m2);


		//force2=GaussSeidelMethod1(mcm,penetration,Iteration);
		////InternalForce1-=trans(m1)*force2;
		////InternalForce3+=trans(m2)*force2;

		//mat transferForce=trans(m2)*force2;

		//Vec3d Contactforce;
		//Contactforce.clear();
		//for(int i=0;i<Nb3;i++)
		//	for(int j=0;j<3;j++)
		//		Contactforce[j]+=transferForce(3*i+j);

		PenaltyMethod(surf,cath,100);

	}

//	surf->addForcetoAllPoint(InternalForce1);
//	endo->addForcetoAllPoint(InternalForce2);
//	cath->addForcetoAllPoint(InternalForce3);
	int n=20;
	for(int i=0;i<n;i++)
	{
		surf->updatePositionExplicit(0.030/n,Vec3d(0,0,-10));
		endo->updateEndoscopeExplicit(0.030/n,Vec3d(0,0,0));
		cath->updateEndoscopeExplicit(0.030/n,Vec3d(0,0,0));
	}
	surf->updateSurfacePointusingMappingMatrix();
}

void SignoriniCollision::InteractionSimulationV3(MyFFD* surf,MyFFD* endo, MyFFD* cath)
{
	mat InternalForce1,InternalForce2,InternalForce3,InternalDis1,InternalDis2,InternalDis3,Gravity1;

	int Nb1=surf->returnNbPoint();
	int Nb2=endo->returnNbPoint();
	int Nb3=cath->returnNbPoint();
	int nbCollide;

	mat penetration,mcm;
	Vec3d normal;
	int Iteration=15;

	InternalForce1.set_size(3*Nb1); InternalForce2.set_size(3*Nb2); InternalForce3.set_size(3*Nb3);
	InternalForce1.fill(0.0); InternalForce2.fill(0.0); InternalForce3.fill(0.0);	

	if(PotentialCollisionDetection(surf,endo,5,1.3))
	{
		nbCollide=distance.size();	

		penetration.set_size(nbCollide);
		for(int i=0;i<nbCollide;i++)
			penetration(i)=distance[i].Penetration;

		ComputeforceUsingPenalty(surf,endo,-1000);
	}

	if(PotentialCollisionDetection(surf,cath,1,5))
	{
		nbCollide=distance.size();	

		penetration.set_size(nbCollide);
		for(int i=0;i<nbCollide;i++)
			penetration(i)=distance[i].Penetration;

		ComputeforceUsingPenalty(surf,cath,-50000);
	}

	int n=10;
	for(int i=0;i<n;i++)
		surf->updatePositionExplicit(0.030/n,Vec3d(0,0,-10));
		
	n=30;
	for(int i=0;i<n;i++)
	{
		endo->updateEndoscopeExplicit(0.030/n,Vec3d(0,0,0));
		cath->updateEndoscopeExplicit(0.030/n,Vec3d(0,0,0));
	}

	/*endo->updateEndoscopeFastImplicit(0.030,Vec3d(0,0,0));
	cath->updateEndoscopeFastImplicit(0.030,Vec3d(0,0,0));*/
	surf->updateSurfacePointusingMappingMatrix();
}


void SignoriniCollision::InteractionSimulationV4(MyFFD* surf,MyFFD* endo, MyFFD* cath)
{

	mat m1,m2,m3,C1,C2,C3,mm1,mm2,mm3;
	mat force1,force2;
	mat InternalForce1,InternalForce2,InternalForce3,InternalDis1,InternalDis2,InternalDis3,Gravity1;

	int Nb1=surf->returnNbPoint();
	int Nb2=endo->returnNbPoint();
	int Nb3=cath->returnNbPoint();
	int nbCollide;

	mat penetration,mcm;
	Vec3d normal;
	int Iteration=15;

	InternalForce1.set_size(3*Nb1); InternalForce2.set_size(3*Nb2); InternalForce3.set_size(3*Nb3);
	InternalForce1.fill(0.0);		InternalForce2.fill(0.0);		InternalForce3.fill(0.0);
	InternalDis1.set_size(3*Nb1);   InternalDis2.set_size(3*Nb2);   InternalDis3.set_size(3*Nb3);
	InternalDis1.fill(0.0);			InternalDis2.fill(0.0);			InternalDis3.fill(0.0);
	

	
	//InternalForce3=cath->returnInternalExplicitForceOfEndoscope();

	C1=surf->returnVertorFormExplicitCompliancematrixForUpper();
	C2=endo->returnVectorFormExplicitCompliancematrixForEndoscope();
	C3=cath->returnVectorFormExplicitCompliancematrixForEndoscope();

	surf->addForcetoAllPoint(Vec3d(0,0,-0.1));
	InternalForce1+=surf->returnInternalExplicitForceOfFFD();
	InternalForce2+=endo->returnInternalExplicitForceOfEndoscope();
	

	if(PotentialCollisionDetection(surf,endo,5,1.3))
	{
		nbCollide=distance.size();	
		force1.set_size(nbCollide); force1.fill(0.0);


		penetration.set_size(nbCollide);
		for(int i=0;i<nbCollide;i++)
			penetration(i)=distance[i].Penetration;

		m1.set_size(nbCollide,3*Nb1); m1.fill(0.0); mm1.set_size(nbCollide,3*Nb1); mm1.fill(0.0);
		m2.set_size(nbCollide,3*Nb2); m2.fill(0.0); mm2.set_size(nbCollide,3*Nb2); mm2.fill(0.0);


		for(int i=0;i<nbCollide;i++)
		{
			normal=distance[i].PenetratedDirec;
			m1.submat(i,0,i,3*Nb1-1)=trans(surf->returnMappingMatforOneSurface(distance[i].triIdx,distance[i].collidedTriPoint,normal));
			m2.submat(i,0,i,3*Nb2-1)=trans(endo->returnMappingMatforOneLattice(distance[i].cylinderIdx,distance[i].measuredPointInCylinder,normal));
		}

		penetration+=-m1*InternalDis1+m2*InternalDis2;
		InternalDis1.fill(0.0);			InternalDis2.fill(0.0);			InternalDis3.fill(0.0);

		

		for(int i=0;i<nbCollide;i++){
			mm1.submat(i,0,i,3*Nb1-1)=m1.submat(i,0,i,3*Nb1-1)%trans(C1);
			mm2.submat(i,0,i,3*Nb2-1)=m2.submat(i,0,i,3*Nb2-1)%trans(C2);
		}

		mcm.set_size(nbCollide,nbCollide);
		mcm.fill(0.0);
		mcm=mm1*trans(m1)+mm2*trans(m2);

		//Gauss Seidel 
		force1=GaussSeidelMethod1(mcm,penetration,Iteration);
		InternalForce1-=trans(m1)*force1;
		InternalForce2+=trans(m2)*force1;
	}

	if(PotentialCollisionDetection(surf,cath,1,1))
	{
		nbCollide=distance.size();	

		Vec3d force;
		double penetration;
		double K=500;

		for(int i=0;i<nbCollide;i++)
		{
			normal=distance[i].PenetratedDirec;
			
			penetration=distance[i].Penetration;
			force=normal*penetration*K;

			surf->addForcetoSurfaceTri(distance[i].triIdx,distance[i].collidedTriPoint,force);
			cath->addForceBetweenPoint(distance[i].cylinderIdx,distance[i].cylinderIdx+1,distance[i].measuredPointInCylinder,-force);
			for(int j=0;j<3;j++)
				InternalForce2(j)-=(force[j]/2);
		}
	}

	

	InternalDis1=C1%InternalForce1-surf->returnPreDisplacement();
	InternalDis2=C2%InternalForce2-endo->returnPreDisplacement();
//	InternalDis3=C3%InternalForce3-cath->returnPreDisplacement();
	
    int	n=10;
	for(int i=0;i<n;i++)
		cath->updateEndoscopeExplicit(0.015/n,Vec3d(0,0,0));


	surf->addDisplacementToControlPoint(InternalDis1);
	endo->addDisplacementToControlPoint(InternalDis2);
//	cath->addDisplacementToControlPoint(InternalDis3);
	
	surf->updateSurfacePointusingMappingMatrix();
}


void SignoriniCollision::InteractionSimulationV5(MyFFD* surf,MyFFD* endo, MyFFD* cath)
{

	mat m1,m2,m3,C1,C2,C3,mm1,mm2,mm3;
	mat force1,force2;
	mat InternalForce1,InternalForce2,InternalForce3,InternalDis1,InternalDis2,InternalDis3,Gravity1;

	int Nb1=surf->returnNbPoint();
	int Nb2=endo->returnNbPoint();
	int Nb3=cath->returnNbPoint();
	int nbCollide;

	mat penetration,mcm;
	Vec3d normal;
	int Iteration=15;

	InternalForce1.set_size(3*Nb1); InternalForce2.set_size(3*Nb2); InternalForce3.set_size(3*Nb3);
	InternalForce1.fill(0.0);		InternalForce2.fill(0.0);		InternalForce3.fill(0.0);
	InternalDis1.set_size(3*Nb1);   InternalDis2.set_size(3*Nb2);   InternalDis3.set_size(3*Nb3);
	InternalDis1.fill(0.0);			InternalDis2.fill(0.0);			InternalDis3.fill(0.0);



	//InternalForce3=cath->returnInternalExplicitForceOfEndoscope();

	C1=surf->returnVertorFormExplicitCompliancematrixForUpper();
	C2=endo->returnVectorFormExplicitCompliancematrixForEndoscope();
	C3=cath->returnVectorFormExplicitCompliancematrixForEndoscope();

	surf->addForcetoAllPoint(Vec3d(0,0,-0.1));
	InternalForce1+=surf->returnInternalExplicitForceOfUpper();
	InternalForce2+=endo->returnInternalExplicitForceOfEndoscope();


	if(PotentialCollisionDetection(surf,endo,5,1.3))
	{
		nbCollide=distance.size();	
		force1.set_size(nbCollide); force1.fill(0.0);


		penetration.set_size(nbCollide);
		for(int i=0;i<nbCollide;i++)
			penetration(i)=distance[i].Penetration;

		m1.set_size(nbCollide,3*Nb1); m1.fill(0.0); mm1.set_size(nbCollide,3*Nb1); mm1.fill(0.0);
		m2.set_size(nbCollide,3*Nb2); m2.fill(0.0); mm2.set_size(nbCollide,3*Nb2); mm2.fill(0.0);


		for(int i=0;i<nbCollide;i++)
		{
			normal=distance[i].PenetratedDirec;
			m1.submat(i,0,i,3*Nb1-1)=trans(surf->returnMappingMatforOneSurface(distance[i].triIdx,distance[i].collidedTriPoint,normal));
			m2.submat(i,0,i,3*Nb2-1)=trans(endo->returnMappingMatforOneLattice(distance[i].cylinderIdx,distance[i].measuredPointInCylinder,normal));
		}

		penetration+=-m1*InternalDis1+m2*InternalDis2;
		InternalDis1.fill(0.0);			InternalDis2.fill(0.0);			InternalDis3.fill(0.0);



		for(int i=0;i<nbCollide;i++){
			mm1.submat(i,0,i,3*Nb1-1)=m1.submat(i,0,i,3*Nb1-1)%trans(C1);
			mm2.submat(i,0,i,3*Nb2-1)=m2.submat(i,0,i,3*Nb2-1)%trans(C2);
		}

		mcm.set_size(nbCollide,nbCollide);
		mcm.fill(0.0);
		mcm=mm1*trans(m1)+mm2*trans(m2);

		//Gauss Seidel 
		force1=GaussSeidelMethod1(mcm,penetration,Iteration);
		InternalForce1-=trans(m1)*force1;
		InternalForce2+=trans(m2)*force1;
	}

	if(PotentialCollisionDetection(surf,cath,1,1))
	{
		nbCollide=distance.size();	

		Vec3d force;
		double penetration;
		double K=500;

		for(int i=0;i<nbCollide;i++)
		{
			normal=distance[i].PenetratedDirec;

			penetration=distance[i].Penetration;
			force=normal*penetration*K;

			surf->addForcetoSurfaceTri(distance[i].triIdx,distance[i].collidedTriPoint,force);
			cath->addForceBetweenPoint(distance[i].cylinderIdx,distance[i].cylinderIdx+1,distance[i].measuredPointInCylinder,-force);
			for(int j=0;j<3;j++)
				InternalForce2(j)-=(force[j]/2);
		}
	}



	InternalDis1=C1%InternalForce1-surf->returnPreDisplacement();
	InternalDis2=C2%InternalForce2-endo->returnPreDisplacement();
	//	InternalDis3=C3%InternalForce3-cath->returnPreDisplacement();

	int	n=10;
	for(int i=0;i<n;i++)
		cath->updateEndoscopeExplicit(0.015/n,Vec3d(0,0,0));


	surf->addDisplacementToControlPoint(InternalDis1);
	endo->addDisplacementToControlPoint(InternalDis2);
	//	cath->addDisplacementToControlPoint(InternalDis3);

	surf->updateSurfacePointusingMappingMatrix();
}

void SignoriniCollision::InteractionSimulationV6(MyFFD* surf,MyFFD* endo)
{

	mat m1,m2,C1,C2,mm1,mm2;
	mat force1;
	mat InternalForce1,InternalForce2,InternalDis1,InternalDis2,Gravity1;

	int Nb1=surf->returnNbPoint();
	int Nb2=endo->returnNbPoint();
	int nbCollide;

	mat penetration,mcm;
	Vec3d normal;
	int Iteration=15;

	InternalForce1.set_size(3*Nb1); InternalForce2.set_size(3*Nb2); 
	InternalForce1.fill(0.0);		InternalForce2.fill(0.0);		
	InternalDis1.set_size(3*Nb1);   InternalDis2.set_size(3*Nb2);   
	InternalDis1.fill(0.0);			InternalDis2.fill(0.0);			


	C1=surf->returnVertorFormExplicitCompliancematrixForUpper();
	C2=endo->returnVectorFormExplicitCompliancematrixForEndoscope();
	
	InternalForce1+=surf->returnInternalExplicitForceOfFFD();
	InternalForce2+=endo->returnInternalExplicitForceOfEndoscope();


	if(PotentialCollisionDetection(surf,endo,5,1.3))
	{
		nbCollide=distance.size();	
		force1.set_size(nbCollide); force1.fill(0.0);


		penetration.set_size(nbCollide);
		for(int i=0;i<nbCollide;i++)
			penetration(i)=distance[i].Penetration;

		m1.set_size(nbCollide,3*Nb1); m1.fill(0.0); mm1.set_size(nbCollide,3*Nb1); mm1.fill(0.0);
		m2.set_size(nbCollide,3*Nb2); m2.fill(0.0); mm2.set_size(nbCollide,3*Nb2); mm2.fill(0.0);


		for(int i=0;i<nbCollide;i++)
		{
			normal=distance[i].PenetratedDirec;
			m1.submat(i,0,i,3*Nb1-1)=trans(surf->returnMappingMatforOneSurface(distance[i].triIdx,distance[i].collidedTriPoint,normal));
			m2.submat(i,0,i,3*Nb2-1)=trans(endo->returnMappingMatforOneLattice(distance[i].cylinderIdx,distance[i].measuredPointInCylinder,normal));
		}

		penetration+=-m1*InternalDis1+m2*InternalDis2;
		InternalDis1.fill(0.0);			InternalDis2.fill(0.0);			



		for(int i=0;i<nbCollide;i++){
			mm1.submat(i,0,i,3*Nb1-1)=m1.submat(i,0,i,3*Nb1-1)%trans(C1);
			mm2.submat(i,0,i,3*Nb2-1)=m2.submat(i,0,i,3*Nb2-1)%trans(C2);
		}

		mcm.set_size(nbCollide,nbCollide);
		mcm.fill(0.0);
		mcm=mm1*trans(m1)+mm2*trans(m2);

		//Gauss Seidel 
		force1=GaussSeidelMethod1(mcm,penetration,Iteration);
		InternalForce1-=trans(m1)*force1;
		InternalForce2+=trans(m2)*force1;
	}

	InternalDis1=C1%InternalForce1-surf->returnPreDisplacement();
	InternalDis2=C2%InternalForce2-endo->returnPreDisplacement();
	

	surf->addDisplacementToControlPoint(InternalDis1);
	endo->addDisplacementToControlPoint(InternalDis2);
	surf->updateSurfacePointusingMappingMatrix();
}


void SignoriniCollision::ComputeforceUsingPenalty(MyFFD* surf,MyFFD* cont,double K)
{
	Vec3d Force;
	Vec3d normal;
	double penetration;
	int nbCollide=distance.size();
	for(int i=0;i<nbCollide;i++)
	{
		
		Force.clear();
		normal=distance[i].PenetratedDirec;
		penetration=distance[i].Penetration;
		Force=normal*penetration*K;
		if(penetration<0)
		{
			surf->addForcetoSurfaceTri(distance[i].triIdx,distance[i].collidedTriPoint,-Force*10);
			cont->addForceBetweenPoint(distance[i].cylinderIdx,distance[i].cylinderIdx+1,distance[i].measuredPointInCylinder,Force);
		}
	}


}


mat SignoriniCollision::GaussSeidelMethod1(mat mcm,mat penetration,int Iteration)
{
	int nbCollide=mcm.n_rows;
	double displacement;
	mat force;
	force.set_size(nbCollide);
	force.fill(0.0);

	mat delta;
	delta.set_size(nbCollide);
	delta.fill(0.0);

	for(int it=0;it<Iteration;it++)
	{
		for(int i=0;i<nbCollide;i++)
		{
			displacement=0;

			for(int j=0;j<nbCollide;j++)
				if(i!=j)
					displacement+=mcm(i,j)*force(j);
			delta(i)=penetration(i)+displacement;

			if(delta(i)>0){
				force(i)=0;
			}else{
				force(i)=-delta(i)/(mcm(i,i));
			}
		}
	}
	
	return force;
}

mat SignoriniCollision::GaussSeidelMethod2(mat mcm,mat penetration,double precision)
{
	int nbCollide=mcm.n_rows;
	double displacement;
	double temp;
	mat preforce;
	preforce.set_size(nbCollide);
	preforce.fill(0.0);
	mat force;
	force.set_size(nbCollide);
	force.fill(1.0);
	mat delta;
	delta.set_size(nbCollide);
	delta.fill(0.0);
	double error=100;
	while(error>=precision){
		error=0;
		for(int i=0;i<nbCollide;i++)
		{
			for(int j=0;j<i;j++)
				delta(j)+=mcm(i,j)*force(j);

			for(int j=i+1;j<nbCollide;j++)
				delta(j)+=mcm(i,j)*preforce(j);

			temp=force(i);
			force(i)=preforce(i)-delta(i)/mcm(i,i);
			preforce(i)=temp;
			error+=abs(force(i)-preforce(i));
		}
	}

	return force;
}


void SignoriniCollision::SetLatticebasedCollision(MyFFD* surf)
{
	int NbCen=surf->returnNbCenter();
	int NbCircularLattice=6;
	int NbSurfacePoint=surf->returnNbSurface();
	int NbTri=surf->returnNbFace();
	Vec3i* Face=surf->returnFace();
	Vec3d* Surface=surf->returnSurfacePoint();
	Vec3d p1,p2,p3;
	bool flag;
	Vec3d normal1,normal2,normal3,normal4;
	TriIndex=new std::vector<int>[(NbCen-2)*6];
	for(int i=0;i<NbCen-2;i++)
	{
		for(int j=0;j<6;j++)
		{

			p1=surf->paraToSurfacePoint(i,j,0,0,1);
			p2=surf->paraToSurfacePoint(i,j,1,1,1);
			p3=surf->paraToSurfacePoint(i,j,1,0,1);
			normal1=(p3-p1).cross(p2-p1);
			p1=surf->paraToSurfacePoint(i,j,0,0,0);
			p2=surf->paraToSurfacePoint(i,j,1,1,0);
			p3=surf->paraToSurfacePoint(i,j,1,0,0);
			normal2=(p3-p1).cross(p2-p1);
			p1=surf->paraToSurfacePoint(i,j,0,0,1);
			p2=surf->paraToSurfacePoint(i,j,0,0,0);
			p3=surf->paraToSurfacePoint(i,j,0.5,0,0.5);
			normal3=(p3-p1).cross(p2-p1);
			p1=surf->paraToSurfacePoint(i,j,0,0,1);
			p2=surf->paraToSurfacePoint(i,j,0,0,0);
			p3=surf->paraToSurfacePoint(i,j,0.5,1,0.5);
			normal4=(p3-p1).cross(p2-p1);
			for(int k=0;k<NbTri;k++)
			{
				for(int l=0;l<3;l++){
					int kk=Face[k][l];
					flag=false;
					p1=surf->paraToSurfacePoint(i,j,0,0,1);
					p2=surf->paraToSurfacePoint(i,j,0,0,0);
					if(((Surface[kk]-p1)*normal1)<=0 && ((Surface[kk]-p2)*normal2)>=0){
						p1=surf->paraToSurfacePoint(i,j,0.5,0,0.5);
						p2=surf->paraToSurfacePoint(i,j,0.5,1,0.5);
						if(((Surface[kk]-p1)*normal3)>=0 && ((Surface[kk]-p2)*normal4)<=0)
							flag=true;
					}

				}
				if(flag)
					TriIndex[6*i+j].push_back(k);
			}
		}
	}
}

SparseMatrix SignoriniCollision::convertMatrixtoSparse(mat A)
{
	SparseMatrix SparseA(A.n_rows,A.n_cols,100);
	int maxterm=0;
	if(A.n_rows==1)
	{
		for(int j=0;j<A.n_cols;j++){
			if(A(j)){
				SparseA.storeMatrix(0,j,A(j));
				maxterm+=1;
			}
		}		
	}else if(A.n_cols==1)
	{
		for(int i=0;i<A.n_rows;i++){
			if(A(i)){
				SparseA.storeMatrix(i,0,A(i));
				maxterm+=1;
			}
		}
	}else
	{
		for(int i=0;i<A.n_rows;i++){
			for(int j=0;j<A.n_cols;j++){
				if(A(i,j)){
					SparseA.storeMatrix(i,j,A(i,j));
					maxterm+=1;
				}
			}
		}
	}
	
	SparseA.Terms=maxterm;

	return SparseA;

}

mat SignoriniCollision::convertSparsetoMatrix(SparseMatrix SparseA)
{
	mat A(SparseA.Rows,SparseA.Cols);
	A.fill(0.0);
	for(int i=0;i<SparseA.Terms;i++)
		A(SparseA.smArray[i].row,SparseA.smArray[i].col)=SparseA.smArray[i].value;

	return A;

}

void SignoriniCollision::interactionSimulation( MyFFD* _catheter, Meshfree_GPU* object, float dt, int itter)
{
	VectorFunc func;
	mat m1,m2,C1,C2,mm1,mm2;
	mat force1;
	mat InternalForce1,InternalForce2,InternalDis1,InternalDis2,Gravity1;

	mat penetration,mcm;
	Vec3d normal;
	int Iteration=15;

	int Nb1=object->efgObj()->nbNode(); // Nb of meshless nodes
	int Nb2=_catheter->returnNbPoint();
	int nbCollide;

	InternalForce1.set_size(3*Nb1); InternalForce2.set_size(3*Nb2); 
	InternalForce1.fill(0.0);		InternalForce2.fill(0.0);		
	InternalDis1.set_size(3*Nb1);   InternalDis2.set_size(3*Nb2);   
	InternalDis1.fill(0.0);			InternalDis2.fill(0.0);		

	// C1 matrix of mesh free nodes; We may not need this matrix
	float** DisAddress=object->efgObj()->returnPreDis(dt,itter);
	float*  nodeMass=object->efgObj()->nodeMass();
	double C=dt*dt*itter*(itter+1)/(2*nodeMass[0]);
	for(int i=0;i<Nb1*3;i++)
	{
		InternalDis1(i)=(*DisAddress)[i];
	}

	C2=_catheter->returnVectorFormExplicitCompliancematrixForEndoscope();

//	InternalForce2+=_catheter->returnInternalExplicitForceOfEndoscope();

	if (CollisionDetectionBtwCatheterAndMeshles(_catheter, object, 1.1))// Collision detection btw catheter and object
	{
		nbCollide=distance.size(); //
		force1.set_size(nbCollide); force1.fill(0.0); // Contact force

		penetration.set_size(nbCollide);
		for(int i=0;i<nbCollide;i++)
			penetration(i)=distance[i].Penetration;

		m1.set_size(nbCollide,3*Nb1); m1.fill(0.0); mm1.set_size(nbCollide,3*Nb1); mm1.fill(0.0);
		m2.set_size(nbCollide,3*Nb2); m2.fill(0.0); mm2.set_size(nbCollide,3*Nb2); mm2.fill(0.0);

		arrayInt affectedEFGNodes;
		std::vector<arrayInt>* neighborSurf = object->neighborNodeOfSurfVertice();
		std::vector<Vec3i>* face=object->surfObj()->face();
		for(int i=0;i<nbCollide;i++)
		{
			// For catheter
			normal=distance[i].PenetratedDirec;
			m2.submat(i,0,i,3*Nb2-1)=trans(_catheter->returnMappingMatforOneLattice(distance[i].cylinderIdx,distance[i].measuredPointInCylinder,normal));

			// For meshles object
			for (int j=0; j<3; j++)
			{
				int Index=(*face)[distance[i].triIdx][j];
				arrayInt neighborNode = neighborSurf->at(Index);
				for (int k=0; k<neighborNode.size(); k++)
				{
					if (!std::binary_search(affectedEFGNodes.begin(), affectedEFGNodes.end(), neighborNode[k]))
					{
						std::vector<int>::iterator it;
						it = std::lower_bound(affectedEFGNodes.begin(), affectedEFGNodes.end(), neighborNode[k]);
						affectedEFGNodes.insert(it, neighborNode[k]);
					}
				}
			}
		}
		// Optimize meshles matrix
		int nbAffectedNodes = affectedEFGNodes.size();
		//
		mat m1t;		m1t.set_size(nbCollide, 3*nbAffectedNodes); m1t.fill(0.0);
		//	penetration.print("pene1.txt");
		double* mapping;
		mapping=new double[3*Nb1];

		for(int i=0;i<nbCollide;i++)
		{
			arrayInt efgNodeIdx;
			normal=distance[i].PenetratedDirec;

			object->returnMappingMatrixWithNodeIdx(distance[i].triIdx,distance[i].collidedTriPoint,normal,mapping, efgNodeIdx);
			func.arrangeVector(efgNodeIdx);

			for (int j=0; j<efgNodeIdx.size(); j++)
			{
				int nodeIdx = efgNodeIdx[j];
				int colIdx = func.indexOfElement(&affectedEFGNodes, nodeIdx);
				if (colIdx <0 || colIdx > nbAffectedNodes)
				{
					int a = efgNodeIdx[0];
				}
				for (int k=0; k<3; k++)
				{
					m1t(i,colIdx*3+k)=mapping[nodeIdx*3+k];
				}
			}

			for(int j=0;j<3*Nb1;j++){
				m1(i,j)=mapping[j];
			}
		}



		delete [] mapping;

	//	penetration+=-m1*InternalDis1+m2*InternalDis2;
		penetration+=-m1*InternalDis1;
		InternalDis1.fill(0.0);			InternalDis2.fill(0.0);	

		for(int i=0;i<nbCollide;i++){
			mm2.submat(i,0,i,3*Nb2-1)=m2.submat(i,0,i,3*Nb2-1)%trans(C2);
		}
		
		mcm.set_size(nbCollide,nbCollide); // Do we need this?
		mcm.fill(0.0);
		mcm=(m1t*trans(m1t))*C+mm2*trans(m2);

		//Gauss Seidel 
		force1=GaussSeidelMethod1(mcm,penetration,Iteration);
		InternalForce1=-trans(m1)*force1; 
		InternalForce2+=trans(m2)*force1;
	}

// 	InternalDis2=C2%InternalForce2-_catheter->returnPreDisplacement();
// 	_catheter->addDisplacementToControlPoint(InternalDis2);


	//_catheter->updateCatheterExplicit(InternalForce2, Vec3d(0,-1,0));
	_catheter->updateCatheterExplicit(Vec3d(0,-1,0));
	InternalDis2=C2%InternalForce2;
	_catheter->addDisplacementToControlPoint(InternalDis2);



	// from force to displace ment of mesh free
	double* InternalForce;
	InternalForce=new double[3*Nb1];

	Vec3d transferdForce(0,0,100);

	for(int i=0;i<3*Nb1;i++)
	{
		InternalForce[i]=InternalForce1(i);
	}

	object->efgObj()->updatePosition(InternalForce,C,dt,itter);
	object->updateSurfPosition();

	delete [] InternalForce;
	InternalForce=NULL;
}

bool SignoriniCollision::CollisionDetectionBtwCatheterAndMeshles( MyFFD* _catheter, Meshfree_GPU* object, double marginRatio )
{
	VectorFunc func;
	Vec3d* Points=_catheter->returnControlPoint();
	int NbPoint=_catheter->returnNbPoint();
	
	int InsertedIndex=0;
	for(int i=0;i<NbPoint;i++)
		if((Points[i]-_catheter->returnInsertedPoint())*_catheter->returnInsertedDirec()>=0)
			InsertedIndex=i;

	distance.clear();
	InspectedTri.clear();

	if (InsertedIndex>1)
	{//Now we detect nearest
		int index = PotentialCollideSegment(_catheter, object);
	}

	std::vector<CollisionManager::Distancefield> distanceM;
	int index;
	for (int i=0;i<InsertedIndex;i++)
	{
		Vec3d point1 = Points[i];
		Vec3d point2 = Points[i+1];

		CollisionManager colM;
		colM.collisionBtwSurfAndLineSeg_part(object->surfObj(),point1,point2,_catheter->returnRadiusOfEndoscope(),marginRatio);
		std::vector<CollisionManager::Distancefield> test = colM.getDistance();
		if (test.size()>0 && test.size()>distanceM.size()/2)
		{
			distanceM = test;
			index = i;
		}
	}
	if (distanceM.size()>0)
	{
		for (int j=0; j<distanceM.size(); j++)
		{
			CollisionManager::Distancefield dColis = distanceM[j];

			SignoriniCollision::Distancefield newDistance;
			newDistance.triIdx = dColis.triIdx;
			newDistance.cylinderIdx = index;
			newDistance.measuredPointInCylinder = dColis.measuredPointInCylinder;
			newDistance.collidedCylPoint=dColis.collidedCylPoint;
			newDistance.collidedTriPoint=dColis.collidedTriPoint;
			newDistance.Distance=dColis.Penetration + _catheter->returnRadiusOfEndoscope();
			newDistance.PenetratedDirec=dColis.PenetratedDirec;
			newDistance.Penetration=dColis.Penetration;

			distance.push_back(newDistance);
			InspectedTri.push_back(dColis.triIdx);
		}
	}

	func.arrangeVector(InspectedTri);
	return distance.size()>0;
}

int SignoriniCollision::PotentialCollideSegment( MyFFD* _catheter, Meshfree_GPU* object )
{
	VectorFunc func;
	Vec3d* Points=_catheter->returnControlPoint();
	int NbPoint=_catheter->returnNbPoint();
	AABBNode* root=object->surfObj()->getBVH()->root();

	//Largest index that intersect
	for (int i=0; i<NbPoint; i++)
	{
		
	}

	return 0;
}
