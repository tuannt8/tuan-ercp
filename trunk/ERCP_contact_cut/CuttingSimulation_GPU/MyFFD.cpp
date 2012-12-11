#include "StdAfx.h"
#include "MyFFD.h"

MyFFD::MyFFD(void)
{
	Kglobal=NULL;
	Cglobal=NULL;
	Mglobal=NULL;
	Compliance=NULL;
	local=NULL;
	ControlPoint=NULL;
	PreControlPoint=NULL;
	Con=NULL;
	Force=NULL;
	Velocity=NULL;
	ControlPoint0=NULL;
	CenterPoint=NULL;
	SurfacePoint=NULL;
	SurfacePointPara=NULL;
	SurfacePointParaIdx=NULL;
	CenterPointPara=NULL;
	CenterPointParaIdx=NULL;
	Face=NULL;
	ControlPointOffline=NULL;
	Mapping=NULL;
	FaceIndex=NULL;
	AroundPoint=NULL;

	x=NULL;

	MappingMatrixforLattice=NULL;
	
	fp=fopen("../data/Check/FFD_Multi.txt","w");
}

MyFFD::~MyFFD(void)
{
	if(Kglobal)
	{
		for (int i=0;i<3*NbPoint;i++)
			delete [] Kglobal[i];
		delete Kglobal;
		Kglobal = NULL;
	}
	if(Cglobal)
	{
		for (int i=0;i<3*NbPoint;i++)
			delete [] Cglobal[i];
		delete Cglobal;
		Cglobal = NULL;
	}
	if(Mglobal)
	{
		for (int i=0;i<3*NbPoint;i++)
			delete [] Mglobal[i]; 
		delete Mglobal;
		Mglobal = NULL;
	}
	if(Compliance)
	{
		for (int i=0;i<3*NbPoint;i++)
			delete [] Compliance[i]; 
		delete Compliance;
		Compliance = NULL;
	}
	if (local)
	{
		for (int i=0;i<6;i++)
			delete [] local[i];
		delete local;
		local = NULL;
	}
	if(ControlPoint)
		delete [] ControlPoint;
	if(PreControlPoint)
		delete [] PreControlPoint;
	if (Con)
		delete [] Con;
	if(Force)
		delete [] Force;
	if(Velocity)
		delete [] Velocity;
	if(ControlPoint0)
		delete [] ControlPoint0;
	if(CenterPoint)
		delete [] CenterPoint;
	if(SurfacePoint)
		delete [] SurfacePoint;
	if(SurfacePointPara)
		delete [] SurfacePointPara;
	if(SurfacePointParaIdx)
		delete [] SurfacePointParaIdx;
	if(CenterPointPara)
		delete [] CenterPointPara;
	if(CenterPointParaIdx)
		delete [] CenterPointParaIdx;
	if(Face)
		delete [] Face;
	if (ControlPointOffline)
	{
		for(int i=0;i<Nbtimestep;i++)
			delete [] ControlPointOffline[i];
		delete ControlPointOffline;
		ControlPointOffline = NULL;
	}
	if(Mapping)
		delete [] Mapping;
	if(FaceIndex)
		delete [] FaceIndex;
	if(x)
		delete [] x;

	if(MappingMatrixforLattice)
	{
		for(int i=0;i<3*NbPoint;i++)
			delete [] MappingMatrixforLattice[i];
		delete MappingMatrixforLattice;
		MappingMatrixforLattice = NULL;
	}
	if(AroundPoint)
		delete [] AroundPoint;

	if (fp)
		fclose(fp);
}

void MyFFD::init(int _NbPoint, Vec3d* _ControlPoint, Vec3d* _surfCP)
{

}

void MyFFD::setOffline(int nbtimestep)
{
	Nbtimestep=nbtimestep;
	ControlPointOffline=new Vec3d*[Nbtimestep];
	for (int i=0;i<Nbtimestep;i++)
		ControlPointOffline[i]=new Vec3d[NbPoint];
}

void MyFFD::updateOffline(int timestep)
{
	for(int i=0;i<NbPoint;i++)
		ControlPointOffline[timestep][i]=ControlPoint[i];
}

void MyFFD::updateOfflinefordraw(int timestep)
{
	for(int i=0;i<NbPoint;i++)
		ControlPoint[i]=ControlPointOffline[timestep][i];
}

void MyFFD::makeSquareFFD(Vec3d Center, int a, int b, int c, double width1, double width2, double height)
{
	//make Connectivity
	NbCon=((a+1)*b+a*(b+1))*(c+1)+(a+1)*(b+1)*c;
	NbPoint=(a+1)*(b+1)*(c+1);
	
	Con=new Vec2i[NbCon];
	ControlPoint=new Vec3d[NbPoint];
	PreControlPoint=new Vec3d[NbPoint];
	ControlPoint0=new Vec3d[NbPoint];
	Force=new Vec3d[NbPoint];
	Velocity=new Vec3d[NbPoint];
	int Nb=0;
	for (int i=0;i<c+1;i++)
	{
		for (int j=0;j<b;j++)
		{
			for (int k=0;k<a;k++)
			{
				if (j==0 && k==0)
				{
					Con[Nb][0]=(a+1)*(b+1)*i+(a+1)*j+k;		Con[Nb][1]=(a+1)*(b+1)*i+(a+1)*j+k+1;		Nb+=1;
					Con[Nb][0]=(a+1)*(b+1)*i+(a+1)*j+k+1;	Con[Nb][1]=(a+1)*(b+1)*i+(a+1)*(j+1)+k+1;	Nb+=1;
					Con[Nb][0]=(a+1)*(b+1)*i+(a+1)*(j+1)+k+1;	Con[Nb][1]=(a+1)*(b+1)*i+(a+1)*(j+1)+k;	Nb+=1;
					Con[Nb][0]=(a+1)*(b+1)*i+(a+1)*(j+1)+k;	Con[Nb][1]=(a+1)*(b+1)*i+(a+1)*(j)+k;		Nb+=1;
					if(i!=c){
						Con[Nb][0]=(a+1)*(b+1)*i+(a+1)*j+k;		  Con[Nb][1]=(a+1)*(b+1)*(i+1)+(a+1)*j+k;		Nb+=1;
						Con[Nb][0]=(a+1)*(b+1)*i+(a+1)*j+k+1;	  Con[Nb][1]=(a+1)*(b+1)*(i+1)+(a+1)*j+k+1;		Nb+=1;
						Con[Nb][0]=(a+1)*(b+1)*i+(a+1)*(j+1)+k+1; Con[Nb][1]=(a+1)*(b+1)*(i+1)+(a+1)*(j+1)+k+1;	Nb+=1;
						Con[Nb][0]=(a+1)*(b+1)*i+(a+1)*(j+1)+k;	  Con[Nb][1]=(a+1)*(b+1)*(i+1)+(a+1)*(j+1)+k;		Nb+=1;
					}
				}else if(j==0 && k>0){
					Con[Nb][0]=(a+1)*(b+1)*i+(a+1)*j+k;		Con[Nb][1]=(a+1)*(b+1)*i+(a+1)*j+k+1;		Nb+=1;
					Con[Nb][0]=(a+1)*(b+1)*i+(a+1)*j+k+1;	Con[Nb][1]=(a+1)*(b+1)*i+(a+1)*(j+1)+k+1;	Nb+=1;
					Con[Nb][0]=(a+1)*(b+1)*i+(a+1)*(j+1)+k+1;	Con[Nb][1]=(a+1)*(b+1)*i+(a+1)*(j+1)+k;	Nb+=1;
					if(i!=c){
						Con[Nb][0]=(a+1)*(b+1)*i+(a+1)*j+k+1;	  Con[Nb][1]=(a+1)*(b+1)*(i+1)+(a+1)*j+k+1;		Nb+=1;
						Con[Nb][0]=(a+1)*(b+1)*i+(a+1)*(j+1)+k+1; Con[Nb][1]=(a+1)*(b+1)*(i+1)+(a+1)*(j+1)+k+1;	Nb+=1;
					}
				}else if(j>0 && k==0){
					Con[Nb][0]=(a+1)*(b+1)*i+(a+1)*j+k+1;	Con[Nb][1]=(a+1)*(b+1)*i+(a+1)*(j+1)+k+1;	Nb+=1;
					Con[Nb][0]=(a+1)*(b+1)*i+(a+1)*(j+1)+k+1;	Con[Nb][1]=(a+1)*(b+1)*i+(a+1)*(j+1)+k;	Nb+=1;
					Con[Nb][0]=(a+1)*(b+1)*i+(a+1)*(j+1)+k;	Con[Nb][1]=(a+1)*(b+1)*i+(a+1)*(j)+k;		Nb+=1;
					if(i!=c){
						Con[Nb][0]=(a+1)*(b+1)*i+(a+1)*(j+1)+k+1; Con[Nb][1]=(a+1)*(b+1)*(i+1)+(a+1)*(j+1)+k+1;	Nb+=1;
						Con[Nb][0]=(a+1)*(b+1)*i+(a+1)*(j+1)+k;	  Con[Nb][1]=(a+1)*(b+1)*(i+1)+(a+1)*(j+1)+k;		Nb+=1;
					}
				}else{
					Con[Nb][0]=(a+1)*(b+1)*i+(a+1)*j+k+1;	Con[Nb][1]=(a+1)*(b+1)*i+(a+1)*(j+1)+k+1;	Nb+=1;
					Con[Nb][0]=(a+1)*(b+1)*i+(a+1)*(j+1)+k+1;	Con[Nb][1]=(a+1)*(b+1)*i+(a+1)*(j+1)+k;	Nb+=1;
					if (i!=c)
					{
						Con[Nb][0]=(a+1)*(b+1)*i+(a+1)*(j+1)+k+1; Con[Nb][1]=(a+1)*(b+1)*(i+1)+(a+1)*(j+1)+k+1;	Nb+=1;
					}
				}
			}
		}
	}

	//make Control point
	Nb=0;

	
	for (int i=0;i<c+1;i++)
	{
		for (int j=0;j<b+1;j++)
		{
			for (int k=0;k<a+1;k++)
			{
				ControlPoint[Nb][0]=Center[0]+k*width1/a-width1/2;
				ControlPoint[Nb][1]=Center[1]+j*width2/b-width2/2;
				ControlPoint[Nb][2]=Center[2]-i*height/c+height/2;
				ControlPoint0[Nb]=ControlPoint[Nb];
				PreControlPoint[Nb]=ControlPoint[Nb];

				if(Nb<NbPoint-1)
					Nb+=1;
			}
		}
	}
}

void MyFFD::makePlaneFFD(Vec3d Center, int a, int b, double width1, double width2)
{
	//make Connectivity
	NbCon=((a+1)*b+a*(b+1));
	NbPoint=(a+1)*(b+1);

	Con=new Vec2i[NbCon];
	ControlPoint=new Vec3d[NbPoint];
	PreControlPoint=new Vec3d[NbPoint];
	ControlPoint0=new Vec3d[NbPoint];
	Force=new Vec3d[NbPoint];
	Velocity=new Vec3d[NbPoint];
	int Nb=0;
	int	i=0;
	for (int j=0;j<b;j++)
	{
		for (int k=0;k<a;k++)
		{
			if (j==0 && k==0)
			{
				Con[Nb][0]=(a+1)*(b+1)*i+(a+1)*j+k;			Con[Nb][1]=(a+1)*(b+1)*i+(a+1)*j+k+1;		Nb+=1;
				Con[Nb][0]=(a+1)*(b+1)*i+(a+1)*j+k+1;		Con[Nb][1]=(a+1)*(b+1)*i+(a+1)*(j+1)+k+1;	Nb+=1;
				Con[Nb][0]=(a+1)*(b+1)*i+(a+1)*(j+1)+k+1;	Con[Nb][1]=(a+1)*(b+1)*i+(a+1)*(j+1)+k;		Nb+=1;
				Con[Nb][0]=(a+1)*(b+1)*i+(a+1)*(j+1)+k;		Con[Nb][1]=(a+1)*(b+1)*i+(a+1)*(j)+k;		Nb+=1;
			}else if(j==0 && k>0){
				Con[Nb][0]=(a+1)*(b+1)*i+(a+1)*j+k;			Con[Nb][1]=(a+1)*(b+1)*i+(a+1)*j+k+1;		Nb+=1;
				Con[Nb][0]=(a+1)*(b+1)*i+(a+1)*j+k+1;		Con[Nb][1]=(a+1)*(b+1)*i+(a+1)*(j+1)+k+1;	Nb+=1;
				Con[Nb][0]=(a+1)*(b+1)*i+(a+1)*(j+1)+k+1;	Con[Nb][1]=(a+1)*(b+1)*i+(a+1)*(j+1)+k;		Nb+=1;
			}else if(j>0 && k==0){
				Con[Nb][0]=(a+1)*(b+1)*i+(a+1)*j+k+1;	Con[Nb][1]=(a+1)*(b+1)*i+(a+1)*(j+1)+k+1;	Nb+=1;
				Con[Nb][0]=(a+1)*(b+1)*i+(a+1)*(j+1)+k+1;	Con[Nb][1]=(a+1)*(b+1)*i+(a+1)*(j+1)+k;	Nb+=1;
				Con[Nb][0]=(a+1)*(b+1)*i+(a+1)*(j+1)+k;	Con[Nb][1]=(a+1)*(b+1)*i+(a+1)*(j)+k;		Nb+=1;
			}else{
				Con[Nb][0]=(a+1)*(b+1)*i+(a+1)*j+k+1;	Con[Nb][1]=(a+1)*(b+1)*i+(a+1)*(j+1)+k+1;	Nb+=1;
				Con[Nb][0]=(a+1)*(b+1)*i+(a+1)*(j+1)+k+1;	Con[Nb][1]=(a+1)*(b+1)*i+(a+1)*(j+1)+k;	
				if(Nb<NbCon-1)
					Nb+=1;
			}
		}
	}


	//make Control point
	Nb=0;


	i=0;
	for (int j=0;j<b+1;j++)
	{
		for (int k=0;k<a+1;k++)
		{
			ControlPoint[Nb][0]=Center[0]+k*width1/a-width1/2;
			ControlPoint[Nb][1]=Center[1]+j*width2/b-width2/2;
			ControlPoint[Nb][2]=Center[2]+0;
			ControlPoint0[Nb]=ControlPoint[Nb];
			PreControlPoint[Nb]=ControlPoint[Nb];
			if(Nb<NbPoint-1)
				Nb+=1;
		}
	}
	
}


void MyFFD::makeLineFFD(Vec3d Center, Vec3d direc, int nb)
{
	//make Connectivity
	NbCon=nb;
	NbPoint=nb+1;

	Con=new Vec2i[NbCon];
	ControlPoint=new Vec3d[NbPoint];
	PreControlPoint=new Vec3d[NbPoint];
	ControlPoint0=new Vec3d[NbPoint];
	Force=new Vec3d[NbPoint];
	Velocity=new Vec3d[NbPoint];

	int	i=0;
	for (int i=0;i<NbCon;i++)
	{
		Con[i][0]=i;
		Con[i][1]=i+1;
	}


	//make Control point
	for (int i=0;i<NbPoint;i++)
	{
		ControlPoint[i]=Center+direc*i;
		ControlPoint0[i]=ControlPoint[i];
		PreControlPoint[i]=ControlPoint[i];
	}

}

void MyFFD::makeEndoscope(Vec3d Center, Vec3d direc,double elementLength,double radius, int nb)
{
	//make Connectivity
	NbCon=nb;
	NbPoint=nb+1;

	direc.normalize();
	InsertedPoint=Center;
	InsertedDirec=direc;
	RadiusOfEndoscope=radius;
	Con=new Vec2i[NbCon];
	ControlPoint=new Vec3d[NbPoint];
	PreControlPoint=new Vec3d[NbPoint];
	ControlPoint0=new Vec3d[NbPoint];
	Force=new Vec3d[NbPoint];
	Velocity=new Vec3d[NbPoint];

	int	i=0;
	for (int i=0;i<NbCon;i++)
	{
		Con[i][0]=i;
		Con[i][1]=i+1;
	}


	//make Control point
	for (int i=0;i<NbPoint;i++)
	{
		ControlPoint[i]=Center-direc*elementLength*i;
		ControlPoint0[i]=ControlPoint[i];
		PreControlPoint[i]=ControlPoint[i];
	}

}

void MyFFD::makeCatheter(double elementLength,double radius, int nb)
{
	//make Connectivity
	NbCon=nb;
	NbPoint=nb+1;
	Vec3d direc(-1,0,0);
	Vec3d Center(70,-35,0);
	direc.normalize();

	InsertedPoint=Center;
	InsertedDirec=direc;

	RadiusOfEndoscope=radius;
	Con=new Vec2i[NbCon];
	ControlPoint=new Vec3d[NbPoint];
	PreControlPoint=new Vec3d[NbPoint];
	ControlPoint0=new Vec3d[NbPoint];
	Force=new Vec3d[NbPoint];
	Velocity=new Vec3d[NbPoint];

	int	i=0;
	for (int i=0;i<NbCon;i++)
	{
		Con[i][0]=i;
		Con[i][1]=i+1;
	}


	//make Control point
	for (int i=0;i<NbPoint;i++)
	{
		ControlPoint[i]=Center-direc*elementLength*i;
		ControlPoint0[i]=ControlPoint[i];
		PreControlPoint[i]=ControlPoint[i];
	}

}

void MyFFD::makePlaneConstraints(int a, int b)
{
	for(int i=0;i<b+1;i++){
		for (int j=0;j<a+1;j++)
		{
			if (i==0 || i==b)
			{
				addFixedConstraint(i*(a+1)+j);
			}else if((i!=0 && i!=b) && (j==0 || j==a)){
				addFixedConstraint(i*(a+1)+j);
			}
		}
	}
}



void MyFFD::drawControlPoint(double radius, double lineWidth)
{
	GLUquadricObj *qobj = 0;
	qobj = gluNewQuadric();
	

	//surface control point: blue
	glColor3f(0,0,1);
	
	for (int i=0;i<NbPoint;i++)
	{
		glPushMatrix();
		glTranslatef((GLfloat)ControlPoint[i][0],(GLfloat)ControlPoint[i][1],(GLfloat)ControlPoint[i][2]);
		gluSphere(qobj,radius,20,10);
		glPopMatrix();
	}
	
	
	//edges between CP: white
	glColor3f(1,1,1);
	glLineWidth((GLfloat)lineWidth);
	glBegin(GL_LINES);
	for (int i=0;i<NbCon;i++)
	{
		
		glVertex3f((GLfloat)ControlPoint[Con[i][0]][0],(GLfloat)ControlPoint[Con[i][0]][1],(GLfloat)ControlPoint[Con[i][0]][2]);
		glVertex3f((GLfloat)ControlPoint[Con[i][1]][0],(GLfloat)ControlPoint[Con[i][1]][1],(GLfloat)ControlPoint[Con[i][1]][2]);
	}
	glEnd();
	
}

void MyFFD::drawControlPoint0(double radius, double lineWidth)
{
	GLUquadricObj *qobj = 0;
	qobj = gluNewQuadric();


	//surface control point: blue
	glColor3f(0,0,1);

	for (int i=0;i<NbPoint;i++)
	{
		glPushMatrix();
		glTranslatef((GLfloat)ControlPoint0[i][0],(GLfloat)ControlPoint0[i][1],(GLfloat)ControlPoint0[i][2]);
		gluSphere(qobj,radius,20,10);
		glPopMatrix();
	}


	//edges between CP: white
	glColor3f(1,1,1);
	glLineWidth((GLfloat)lineWidth);
	glBegin(GL_LINES);
	for (int i=0;i<NbCon;i++)
	{

		glVertex3f((GLfloat)ControlPoint0[Con[i][0]][0],(GLfloat)ControlPoint0[Con[i][0]][1],(GLfloat)ControlPoint0[Con[i][0]][2]);
		glVertex3f((GLfloat)ControlPoint0[Con[i][1]][0],(GLfloat)ControlPoint0[Con[i][1]][1],(GLfloat)ControlPoint0[Con[i][1]][2]);
	}
	glEnd();

}

void MyFFD::drawPoint(Vec3d Point, double radius)
{
	GLUquadricObj *qobj = 0;
	qobj = gluNewQuadric();

	glPushMatrix();
	glTranslatef((GLfloat)Point[0],(GLfloat)Point[1],(GLfloat)Point[2]);
	gluSphere(qobj,radius,20,10);
	glPopMatrix();

}
void MyFFD::drawControlPoint(int Idx, double radius)
{
	GLUquadricObj *qobj = 0;
	qobj = gluNewQuadric();

	glPushMatrix();
	glTranslatef((GLfloat)ControlPoint[Idx][0],(GLfloat)ControlPoint[Idx][1],(GLfloat)ControlPoint[Idx][2]);
	gluSphere(qobj,radius,20,10);
	glPopMatrix();

}

void MyFFD::drawCenterPoint(int Idx, double radius)

{
	GLUquadricObj *qobj = 0;
	qobj = gluNewQuadric();

	glPushMatrix();
	glTranslatef((GLfloat)CenterPoint[Idx][0],(GLfloat)CenterPoint[Idx][1],(GLfloat)CenterPoint[Idx][2]);
	gluSphere(qobj,radius,20,10);
	glPopMatrix();

}

void MyFFD::drawSurfacePoint(int Idx, double radius)
{
	GLUquadricObj *qobj = 0;
	qobj = gluNewQuadric();

	glPushMatrix();
	glTranslatef((GLfloat)SurfacePoint[Idx][0],(GLfloat)SurfacePoint[Idx][1],(GLfloat)SurfacePoint[Idx][2]);
	gluSphere(qobj,radius,20,10);
	glPopMatrix();
}

void MyFFD::drawTri(int idx)
{
	glBegin(GL_TRIANGLES);
	glVertex3f((GLfloat)SurfacePoint[Face[idx][0]][0],(GLfloat)SurfacePoint[Face[idx][0]][1],(GLfloat)SurfacePoint[Face[idx][0]][2]);
	glVertex3f((GLfloat)SurfacePoint[Face[idx][1]][0],(GLfloat)SurfacePoint[Face[idx][1]][1],(GLfloat)SurfacePoint[Face[idx][1]][2]);
	glVertex3f((GLfloat)SurfacePoint[Face[idx][2]][0],(GLfloat)SurfacePoint[Face[idx][2]][1],(GLfloat)SurfacePoint[Face[idx][2]][2]);

	glVertex3f((GLfloat)SurfacePoint[Face[idx][0]][0],(GLfloat)SurfacePoint[Face[idx][0]][1],(GLfloat)SurfacePoint[Face[idx][0]][2]);
	glVertex3f((GLfloat)SurfacePoint[Face[idx][2]][0],(GLfloat)SurfacePoint[Face[idx][2]][1],(GLfloat)SurfacePoint[Face[idx][2]][2]);
	glVertex3f((GLfloat)SurfacePoint[Face[idx][1]][0],(GLfloat)SurfacePoint[Face[idx][1]][1],(GLfloat)SurfacePoint[Face[idx][1]][2]);
	glEnd();
}

void MyFFD::drawWire(double lineWidth)
{
	GLUquadricObj *qobj = 0;
	qobj = gluNewQuadric();
	glLineWidth((GLfloat)lineWidth);
	//edges between CP: white
	
	glBegin(GL_LINES);
	for (int i=0;i<NbCon;i++)
	{
		glVertex3f((GLfloat)ControlPoint[Con[i][0]][0],(GLfloat)ControlPoint[Con[i][0]][1],(GLfloat)ControlPoint[Con[i][0]][2]);
		glVertex3f((GLfloat)ControlPoint[Con[i][1]][0],(GLfloat)ControlPoint[Con[i][1]][1],(GLfloat)ControlPoint[Con[i][1]][2]);
	}
	glEnd();

}

void MyFFD::drawCylinder(double radius)
{
	GLUquadricObj *qobj = 0;
	qobj = gluNewQuadric();
	//edges between CP: white

	double Cos;
	Vec3d direc;
	Vec3d normal;
	Vec3d Zaxis(0,0,1);
	double L;

	for (int i=0;i<NbCon;i++)
	{
		
		L=(ControlPoint[Con[i][1]]-ControlPoint[Con[i][0]]).norm();
		direc=(ControlPoint[Con[i][1]]-ControlPoint[Con[i][0]]);
		direc.normalize();
		normal=Zaxis.cross(direc);
		normal.normalize();

		Cos=direc*Zaxis;
		Cos=180/PI*acos(Cos);
		
		
		if(i==0){
			glPushMatrix();
			glTranslatef((GLfloat)ControlPoint[Con[i][0]][0],(GLfloat)ControlPoint[Con[i][0]][1],(GLfloat)ControlPoint[Con[i][0]][2]);
			gluSphere(qobj,radius,20,10);
			glPopMatrix();
		}
		glPushMatrix();
		glTranslatef((GLfloat)ControlPoint[Con[i][1]][0],(GLfloat)ControlPoint[Con[i][1]][1],(GLfloat)ControlPoint[Con[i][1]][2]);
		gluSphere(qobj,radius,20,10);
		glPopMatrix();

		glPushMatrix();
		glTranslatef((GLfloat)ControlPoint[Con[i][0]][0],(GLfloat)ControlPoint[Con[i][0]][1],(GLfloat)ControlPoint[Con[i][0]][2]);
		glRotatef((GLfloat)Cos,(GLfloat)normal[0],(GLfloat)normal[1],(GLfloat)normal[2]);
		gluCylinder(qobj,(GLfloat)radius,(GLfloat)radius,L,20,20);
		glPopMatrix();		
	}
}

void MyFFD::drawEndoscope()
{
	GLUquadricObj *qobj = 0;
	qobj = gluNewQuadric();

	double Cos;
	Vec3d direc;
	Vec3d normal;
	Vec3d Zaxis(0,0,1);
	double L;
	Vec3d test;
	int index=0;
	double radius=RadiusOfEndoscope;
	for(int i=0;i<NbPoint;i++)
		if((ControlPoint[i]-InsertedPoint)*InsertedDirec>=0)
			index=i;

	glPushMatrix();
	glTranslatef((GLfloat)ControlPoint[0][0],(GLfloat)ControlPoint[0][1],(GLfloat)ControlPoint[0][2]);
	gluSphere(qobj,radius,20,10);
	glPopMatrix();

	for (int i=0;i<index;i++)
	{
		L=(ControlPoint[i+1]-ControlPoint[i]).norm();
		direc=(ControlPoint[i+1]-ControlPoint[i]);
		direc.normalize();
		normal=Zaxis.cross(direc);
		normal.normalize();

		Cos=direc*Zaxis;
		Cos=180/PI*acos(Cos);

		glPushMatrix();
		glTranslatef((GLfloat)ControlPoint[i+1][0],(GLfloat)ControlPoint[i+1][1],(GLfloat)ControlPoint[i+1][2]);
		gluSphere(qobj,radius,20,10);
		glPopMatrix();

		glPushMatrix();
		glTranslatef((GLfloat)ControlPoint[i][0],(GLfloat)ControlPoint[i][1],(GLfloat)ControlPoint[i][2]);
		glRotatef((GLfloat)Cos,(GLfloat)normal[0],(GLfloat)normal[1],(GLfloat)normal[2]);
		gluCylinder(qobj,(GLfloat)radius,(GLfloat)radius,L,20,20);
		glPopMatrix();
	}

	L=(InsertedPoint-ControlPoint[index]).norm();
	direc=(InsertedPoint-ControlPoint[index]);
	direc.normalize();
	normal=Zaxis.cross(direc);
	normal.normalize();

	Cos=direc*Zaxis;
	Cos=180/PI*acos(Cos);

	glPushMatrix();
	glTranslatef((GLfloat)InsertedPoint[0],(GLfloat)InsertedPoint[1],(GLfloat)InsertedPoint[2]);
	gluSphere(qobj,radius,20,10);
	glPopMatrix();

	glPushMatrix();
	glTranslatef((GLfloat)ControlPoint[index][0],(GLfloat)ControlPoint[index][1],(GLfloat)ControlPoint[index][2]);
	glRotatef((GLfloat)Cos,(GLfloat)normal[0],(GLfloat)normal[1],(GLfloat)normal[2]);
	gluCylinder(qobj,(GLfloat)radius,(GLfloat)radius,L,20,20);
	glPopMatrix();
}


void MyFFD::drawPenetrationAtSurface(int Idx,Vec3d penetration,double lineWidth)
{

	glLineWidth((GLfloat)lineWidth);

	Vec3d A;
	A=SurfacePoint[Idx]+penetration;
	glBegin(GL_LINES);
	glVertex3f((GLfloat)SurfacePoint[Idx][0],(GLfloat)SurfacePoint[Idx][1],(GLfloat)SurfacePoint[Idx][2]);
	glVertex3f((GLfloat)A[0],(GLfloat)A[1],(GLfloat)A[2]);
	glEnd();

}

void MyFFD::drawPenetrationAtControl(int Idx,Vec3d penetration,double lineWidth)
{

	glLineWidth((GLfloat)lineWidth);
	Vec3d A;
	A=ControlPoint[Idx]+penetration;
	glBegin(GL_LINES);
	glVertex3f((GLfloat)ControlPoint[Idx][0],(GLfloat)ControlPoint[Idx][1],(GLfloat)ControlPoint[Idx][2]);
	glVertex3f((GLfloat)A[0],(GLfloat)A[1],(GLfloat)A[2]);
	glEnd();

}

void MyFFD::drawLine(Vec3d P1,Vec3d P2,double lineWidth)
{

	glLineWidth((GLfloat)lineWidth);

	glBegin(GL_LINES);
	glVertex3f((GLfloat)P1[0],(GLfloat)P1[1],(GLfloat)P1[2]);
	glVertex3f((GLfloat)P2[0],(GLfloat)P2[1],(GLfloat)P2[2]);
	glEnd();

}

void MyFFD::drawBendingAxis()
{
	Vec3d v1,v2,v3;
	
	for(int i=0;i<NbPoint-2;i++)
	{
		v1=ControlPoint[i+1]-ControlPoint[i];
		v2=ControlPoint[i+2]-ControlPoint[i+1];
		v3=v1.cross(v2);
		v1=v3.cross(v1);
		v2=v3.cross(v2);
		v1.normalize();
		v2.normalize();
		drawLine(ControlPoint[i],ControlPoint[i]+v1*10,3.0);
		drawLine(ControlPoint[i+2],ControlPoint[i+2]+v2*10,3.0);
	}
}

void MyFFD::drawForce()
{
	for(int i=0;i<NbPoint;i++)
		drawLine(ControlPoint[i],ControlPoint[i]+Force[i]*100,3.0);
}

void MyFFD::drawPlane(double lineWidth,int a,int b)
{
	Vec3d Normal;
	GLUquadricObj *qobj = 0;
	qobj = gluNewQuadric();
	glBegin(GL_QUADS);
	for (int i=0;i<b;i++)
	{
		for (int j=0;j<a;j++)
		{
			Normal=computeNormal(ControlPoint[(a+1)*(i+1)+j],ControlPoint[(a+1)*(i+1)+j+1],ControlPoint[(a+1)*i+j+1]);
			Normal=-Normal;
			glNormal3f((GLfloat)Normal[0],(GLfloat)Normal[1],(GLfloat)Normal[2]);
			glVertex3f((GLfloat)ControlPoint[(a+1)*i+j][0],(GLfloat)ControlPoint[(a+1)*i+j][1],(GLfloat)ControlPoint[(a+1)*i+j][2]);					// Top left
			glVertex3f((GLfloat)ControlPoint[(a+1)*i+j+1][0],(GLfloat)ControlPoint[(a+1)*i+j+1][1],(GLfloat)ControlPoint[(a+1)*i+j+1][2]);				// Top Right
			glVertex3f((GLfloat)ControlPoint[(a+1)*(i+1)+j+1][0],(GLfloat)ControlPoint[(a+1)*(i+1)+j+1][1],(GLfloat)ControlPoint[(a+1)*(i+1)+j+1][2]);  // Bottom Right
			glVertex3f((GLfloat)ControlPoint[(a+1)*(i+1)+j][0],(GLfloat)ControlPoint[(a+1)*(i+1)+j][1],(GLfloat)ControlPoint[(a+1)*(i+1)+j][2]);        // Bottom Left
		}
	}

	for (int i=0;i<b;i++)
	{
		for (int j=0;j<a;j++)
		{
			Normal=computeNormal(ControlPoint[(a+1)*i+j+1],ControlPoint[(a+1)*(i+1)+j+1],ControlPoint[(a+1)*(i+1)+j]);
			Normal=-Normal;
			glNormal3f((GLfloat)Normal[0],(GLfloat)Normal[1],(GLfloat)Normal[2]);
			glVertex3f((GLfloat)ControlPoint[(a+1)*(i+1)+j][0],(GLfloat)ControlPoint[(a+1)*(i+1)+j][1],(GLfloat)ControlPoint[(a+1)*(i+1)+j][2]);        // Bottom Left
			glVertex3f((GLfloat)ControlPoint[(a+1)*(i+1)+j+1][0],(GLfloat)ControlPoint[(a+1)*(i+1)+j+1][1],(GLfloat)ControlPoint[(a+1)*(i+1)+j+1][2]);  // Bottom Right
			glVertex3f((GLfloat)ControlPoint[(a+1)*i+j+1][0],(GLfloat)ControlPoint[(a+1)*i+j+1][1],(GLfloat)ControlPoint[(a+1)*i+j+1][2]);				// Top Right
			glVertex3f((GLfloat)ControlPoint[(a+1)*i+j][0],(GLfloat)ControlPoint[(a+1)*i+j][1],(GLfloat)ControlPoint[(a+1)*i+j][2]);					// Top left
		}
	}
	glEnd();
}

void MyFFD::drawConstraintPoint(double radius)
{
	GLUquadricObj *qobj = 0;
	qobj = gluNewQuadric();
	for(unsigned int i=0;i<FixedConstraint.size();i++){
		glColor3f(1,0,0);
		glPushMatrix();
		glTranslatef((GLfloat)ControlPoint[FixedConstraint[i]][0],(GLfloat)ControlPoint[FixedConstraint[i]][1],(GLfloat)ControlPoint[FixedConstraint[i]][2]);
		gluSphere(qobj,radius,20,10);
		glPopMatrix();
	}
}

void MyFFD::drawSurface()
{
	GLUquadricObj *qobj = 0;
	qobj = gluNewQuadric();

	int i;
	
	glBegin(GL_LINES);
	for(i=0;i<NbFace;i++)
	{
		glVertex3f((GLfloat)SurfacePoint[Face[i][0]][0], (GLfloat)SurfacePoint[Face[i][0]][1], (GLfloat)SurfacePoint[Face[i][0]][2]);
		glVertex3f((GLfloat)SurfacePoint[Face[i][1]][0], (GLfloat)SurfacePoint[Face[i][1]][1], (GLfloat)SurfacePoint[Face[i][1]][2]);

		glVertex3f((GLfloat)SurfacePoint[Face[i][1]][0], (GLfloat)SurfacePoint[Face[i][1]][1], (GLfloat)SurfacePoint[Face[i][1]][2]);
		glVertex3f((GLfloat)SurfacePoint[Face[i][2]][0], (GLfloat)SurfacePoint[Face[i][2]][1], (GLfloat)SurfacePoint[Face[i][2]][2]);

		glVertex3f((GLfloat)SurfacePoint[Face[i][2]][0], (GLfloat)SurfacePoint[Face[i][2]][1], (GLfloat)SurfacePoint[Face[i][2]][2]);
		glVertex3f((GLfloat)SurfacePoint[Face[i][0]][0], (GLfloat)SurfacePoint[Face[i][0]][1], (GLfloat)SurfacePoint[Face[i][0]][2]);
	}
	glEnd();
}

void MyFFD::drawSurfaceTri()
{
	GLUquadricObj *qobj = 0;
	qobj = gluNewQuadric();

	int i;
	Vec3d pos[3];
	Vec3d normal;

	glBegin(GL_TRIANGLES);
	//glShadeModel(GL_SMOOTH);
	for(i=0;i<NbFace;i++)
	{
		pos[0]=SurfacePoint[Face[i][0]];
		pos[2]=SurfacePoint[Face[i][1]];
		pos[1]=SurfacePoint[Face[i][2]];
		normal=computeNormal(pos[0],pos[1],pos[2]);
		glNormal3f((GLfloat)normal[0],(GLfloat)normal[1],(GLfloat)normal[2]);
		glVertex3f((GLfloat)SurfacePoint[Face[i][0]][0],(GLfloat)SurfacePoint[Face[i][0]][1],(GLfloat)SurfacePoint[Face[i][0]][2]);
		glVertex3f((GLfloat)SurfacePoint[Face[i][1]][0],(GLfloat)SurfacePoint[Face[i][1]][1],(GLfloat)SurfacePoint[Face[i][1]][2]);
		glVertex3f((GLfloat)SurfacePoint[Face[i][2]][0],(GLfloat)SurfacePoint[Face[i][2]][1],(GLfloat)SurfacePoint[Face[i][2]][2]);


		glVertex3f((GLfloat)SurfacePoint[Face[i][0]][0],(GLfloat)SurfacePoint[Face[i][0]][1],(GLfloat)SurfacePoint[Face[i][0]][2]);
		glVertex3f((GLfloat)SurfacePoint[Face[i][2]][0],(GLfloat)SurfacePoint[Face[i][2]][1],(GLfloat)SurfacePoint[Face[i][2]][2]);
		glVertex3f((GLfloat)SurfacePoint[Face[i][1]][0],(GLfloat)SurfacePoint[Face[i][1]][1],(GLfloat)SurfacePoint[Face[i][1]][2]);
	}
	glEnd();
}

void MyFFD::drawSmoothSurface()
{
	GLUquadricObj *qobj = 0;
	qobj = gluNewQuadric();

	int i;
	mat PointNormal,Normal,Sum;
	PointNormal.set_size(NbSurfacePoint,3);
	Normal.set_size(NbFace,3);
	Sum.set_size(3);
	Vec3d pos[3];
	Vec3d normal;
	
	for(i=0;i<NbFace;i++)
	{
		pos[0]=SurfacePoint[Face[i][0]];
		pos[1]=SurfacePoint[Face[i][1]];
		pos[2]=SurfacePoint[Face[i][2]];
		normal=computeNormal(pos[0],pos[1],pos[2]);
		Normal(i,0)=normal[0];
		Normal(i,1)=normal[1];
		Normal(i,2)=normal[2];
	}


	for(i=0;i<NbSurfacePoint;i++)
	{
		if(AroundPoint[i].size())
		{
			Sum.fill(0.0);
			for(int j=0;j<AroundPoint[i].size();j++){
				Sum(0)+=Normal(AroundPoint[i][j],0);
				Sum(1)+=Normal(AroundPoint[i][j],1);
				Sum(2)+=Normal(AroundPoint[i][j],2);
			}
			PointNormal(i,0)=Sum(0)/AroundPoint[i].size();
			PointNormal(i,1)=Sum(1)/AroundPoint[i].size();
			PointNormal(i,2)=Sum(2)/AroundPoint[i].size();

		}
	}

	
	glBegin(GL_TRIANGLES);
	//glShadeModel(GL_SMOOTH);
	for(i=0;i<NbFace;i++)
	{
		glNormal3f((GLfloat)PointNormal(Face[i][0],0), (GLfloat)PointNormal(Face[i][0],1), (GLfloat)PointNormal(Face[i][0],2));
		glVertex3f((GLfloat)SurfacePoint[Face[i][0]][0],(GLfloat)SurfacePoint[Face[i][0]][1],(GLfloat)SurfacePoint[Face[i][0]][2]);
		glNormal3f((GLfloat)PointNormal(Face[i][1],0), (GLfloat)PointNormal(Face[i][1],1), (GLfloat)PointNormal(Face[i][1],2));
		glVertex3f((GLfloat)SurfacePoint[Face[i][1]][0],(GLfloat)SurfacePoint[Face[i][1]][1],(GLfloat)SurfacePoint[Face[i][1]][2]);
		glNormal3f((GLfloat)PointNormal(Face[i][2],0), (GLfloat)PointNormal(Face[i][2],1), (GLfloat)PointNormal(Face[i][2],2));
		glVertex3f((GLfloat)SurfacePoint[Face[i][2]][0],(GLfloat)SurfacePoint[Face[i][2]][1],(GLfloat)SurfacePoint[Face[i][2]][2]);
	}

	glEnd();
}

void MyFFD::drawTexturedSurface()
{
	GLUquadricObj *qobj = 0;
	qobj = gluNewQuadric();

	int i;
	mat PointNormal,Normal,Sum;
	PointNormal.set_size(NbSurfacePoint,3);
	Normal.set_size(NbFace,3);
	Sum.set_size(3);
	Vec3d pos[3];
	Vec3d normal;

	for(i=0;i<NbFace;i++)
	{
		pos[0]=SurfacePoint[Face[i][0]];
		pos[1]=SurfacePoint[Face[i][1]];
		pos[2]=SurfacePoint[Face[i][2]];
		normal=computeNormal(pos[0],pos[1],pos[2]);
		Normal(i,0)=normal[0];
		Normal(i,1)=normal[1];
		Normal(i,2)=normal[2];
	}


	for(i=0;i<NbSurfacePoint;i++)
	{
		if(AroundPoint[i].size())
		{
			Sum.fill(0.0);
			for(int j=0;j<AroundPoint[i].size();j++){
				Sum(0)+=Normal(AroundPoint[i][j],0);
				Sum(1)+=Normal(AroundPoint[i][j],1);
				Sum(2)+=Normal(AroundPoint[i][j],2);
			}
			PointNormal(i,0)=Sum(0)/AroundPoint[i].size();
			PointNormal(i,1)=Sum(1)/AroundPoint[i].size();
			PointNormal(i,2)=Sum(2)/AroundPoint[i].size();

		}
	}

	glEnable(GL_TEXTURE_2D);
	glBegin(GL_TRIANGLES);
	//glShadeModel(GL_SMOOTH);
	for(i=0;i<NbFace;i++)
	{
		//glNormal3f((GLfloat)Normal(i,0), (GLfloat)Normal(i,1), (GLfloat)Normal(i,2));
		for(int j=0;j<3;j++)
		{
			glNormal3f((GLfloat)PointNormal(Face[i][j],0), (GLfloat)PointNormal(Face[i][j],1), (GLfloat)PointNormal(Face[i][j],2));
			glTexCoord2f(texCoord(FaceofTex(i,j),0),texCoord(FaceofTex(i,j),1));
			glVertex3f((GLfloat)SurfacePoint[Face[i][j]][0],(GLfloat)SurfacePoint[Face[i][j]][1],(GLfloat)SurfacePoint[Face[i][j]][2]);
		}
	}
	glEnd();
	glDisable(GL_TEXTURE_2D);
}

void MyFFD::drawFaceNormalVector(double LineWidth)
{
	Vec3d pos[3];
	Vec3d Cen;
	Vec3d normal;
	for(int i=0;i<NbFace;i++)
	{
		pos[0]=SurfacePoint[Face[i][0]];
		pos[1]=SurfacePoint[Face[i][1]];
		pos[2]=SurfacePoint[Face[i][2]];
		Cen=(pos[0]+pos[1]+pos[2])/3;
		normal=computeNormal(pos[0],pos[1],pos[2]);
		drawLine(Cen,Cen+normal*5,LineWidth);
	}
}

void MyFFD::drawControlGroup(int centerIdx, int circualrIdx,double radius,double lineWidth)
{
	int ii[3];
	int jj[3];
	GLUquadricObj *qobj = 0;
	qobj = gluNewQuadric();
	ii[0]=centerIdx;
	ii[1]=centerIdx+1;
	ii[2]=centerIdx+2;


	jj[0] = (circualrIdx+6-1)%6;
	jj[1] = circualrIdx;
	jj[2] = (circualrIdx+1)%6;

	glColor3f(0,0,1);
	for(int i=0;i<3;i++)
	{
		glPushMatrix();
		glTranslatef((GLfloat)ControlPoint[7*ii[i]][0],(GLfloat)ControlPoint[7*ii[i]][1],(GLfloat)ControlPoint[7*ii[i]][2]);
		gluSphere(qobj,radius,20,10);
		glPopMatrix();
		for(int j=0;j<3;j++)
		{
			if(i==1 && j==1)
				glColor3f((GLfloat)0.8,(GLfloat)0,(GLfloat)0);
			glPushMatrix();
			glTranslatef((GLfloat)ControlPoint[7*ii[i]+jj[j]+1][0],(GLfloat)ControlPoint[7*ii[i]+jj[j]+1][1],(GLfloat)ControlPoint[7*ii[i]+jj[j]+1][2]);
			gluSphere(qobj,radius,20,10);
			glPopMatrix();
			glColor3f(0,0,1);
		}
	}
	glLineWidth((GLfloat)lineWidth);
	glBegin(GL_LINES);
	glColor3f(0.5,0,0);
	for(int i=0;i<3;i++)
	{
		for(int j=0;j<3;j++)
		{
			glVertex3f((GLfloat)ControlPoint[7*ii[i]][0], (GLfloat)ControlPoint[7*ii[i]][1], (GLfloat)ControlPoint[7*ii[i]][2]);
			glVertex3f((GLfloat)ControlPoint[7*ii[i]+jj[j]+1][0], (GLfloat)ControlPoint[7*ii[i]+jj[j]+1][1], (GLfloat)ControlPoint[7*ii[i]+jj[j]+1][2]);
		}
		for(int j=0;j<2;j++)
		{
			glVertex3f((GLfloat)ControlPoint[7*ii[i]+jj[j]+1][0], (GLfloat)ControlPoint[7*ii[i]+jj[j]+1][1], (GLfloat)ControlPoint[7*ii[i]+jj[j]+1][2]);
			glVertex3f((GLfloat)ControlPoint[7*ii[i]+jj[j+1]+1][0], (GLfloat)ControlPoint[7*ii[i]+jj[j+1]+1][1], (GLfloat)ControlPoint[7*ii[i]+jj[j+1]+1][2]);
		}
	}
	for(int i=0;i<2;i++)
	{
		glVertex3f((GLfloat)ControlPoint[7*ii[i]][0], (GLfloat)ControlPoint[7*ii[i]][1], (GLfloat)ControlPoint[7*ii[i]][2]);
		glVertex3f((GLfloat)ControlPoint[7*ii[i+1]][0], (GLfloat)ControlPoint[7*ii[i+1]][1], (GLfloat)ControlPoint[7*ii[i+1]][2]);
		for(int j=0;j<3;j++)
		{
			glVertex3f((GLfloat)ControlPoint[7*ii[i]+jj[j]+1][0], (GLfloat)ControlPoint[7*ii[i]+jj[j]+1][1], (GLfloat)ControlPoint[7*ii[i]+jj[j]+1][2]);
			glVertex3f((GLfloat)ControlPoint[7*ii[i+1]+jj[j]+1][0], (GLfloat)ControlPoint[7*ii[i+1]+jj[j]+1][1], (GLfloat)ControlPoint[7*ii[i+1]+jj[j]+1][2]);
		}
	}

	glEnd();
}

Vec3d MyFFD::paraToSurfacePoint(int centerIdx, int circualrIdx, double u, double v, double w)
{
	int m, n, ii[3], jj[3];
	Vec3d ret;
	double vv, ww;
	double funcX, funcY, funcZ, func;

	///////////////////
	//basis functions//
	///////////////////

	//linear basis function
	Vec2d L;

	//quadratic B spline
	Vec3d B1;
	Vec3d B2;
	Mat3x3d B12;


	/*if(centerIdx==0) 
	ii[0] = centerIdx;
	else ii[0] = centerIdx-1;
	ii[1] = centerIdx;
	ii[2] = centerIdx+1;*/

	ii[0]=centerIdx;
	ii[1]=centerIdx+1;
	ii[2]=centerIdx+2;


	jj[0] = (circualrIdx+6-1)%6;
	jj[1] = circualrIdx;
	jj[2] = (circualrIdx+1)%6;

	L[0] = 1.0 - u; 
	L[1] = u;

	vv = v*v;
	ww = w*w;

	B1[0] = (vv-2*v+1);
	B1[1] = (-2*vv+2*v+1);
	B1[2] = vv;

	B2[0] = (ww-2*w+1);
	B2[1] = (-2*ww+2*w+1);
	B2[2] = ww;

	for(m=0;m<3;m++)
	{
		for(n=0;n<3;n++)
		{
			B12[m][n] = B1[m]*B2[n];
		}	
	}

	funcX = funcY = funcZ = 0.0;
	for(n=0;n<3;n++)
	{
		func = 0.0;

		for(m=0;m<3;m++) 
			func += B12[m][n];

		func*=L[0];

		ret += ControlPoint[7*ii[n]]*func;	
	
	}
	for(m=0;m<3;m++)
		for(n=0;n<3;n++){
			func = L[1]*B12[m][n];
			ret += ControlPoint[ii[n]*7+jj[m]+1]*func;	
		}

	return ret * 0.25;
}



Vec3d MyFFD::computeNormal(Vec3d pos1, Vec3d pos2, Vec3d pos3)
{
	Vec3d normal;
	Vec3d vector1, vector2;

	vector1=pos1-pos2; 
	vector2=pos1-pos3; 
	normal = vector1.cross(vector2);
	normal.normalize();

	return normal;
}

void MyFFD::makeMappingMatrix()
{
	int m, n;
	Vec3i ii,jj;

	double vv, ww;
	double funcX, funcY, funcZ, func;
	double u,v,w;


	///////////////////
	//basis functions//
	///////////////////

	//linear basis function
	Vec2d L;

	//quadratic B spline
	Vec3d B1;
	Vec3d B2;
	Mat3x3d B12;

	Mapping=new Vec<12,double>[NbSurfacePoint];

	MappingMatrix.set_size(3*NbSurfacePoint,3*NbPoint);
	MappingMatrix.fill(0.0);

	MappingIndex.set_size(NbSurfacePoint,36);
	
	for(int i=0;i<NbSurfacePoint;i++)
	{
			u=SurfacePointPara[i][0];
			v=SurfacePointPara[i][1];
			w=SurfacePointPara[i][2];

			ii[0]=SurfacePointParaIdx[i][0];
			ii[1]=SurfacePointParaIdx[i][0]+1;
			ii[2]=SurfacePointParaIdx[i][0]+2;


			jj[0] = (SurfacePointParaIdx[i][1]+SurfCPNum-1)%SurfCPNum;
			jj[1] = SurfacePointParaIdx[i][1];
			jj[2] = (SurfacePointParaIdx[i][1]+1)%SurfCPNum;

			L[0] = 1.0 - u; 
			L[1] = u;

			vv = v*v;
			ww = w*w;

			B1[0] = (vv-2*v+1);
			B1[1] = (-2*vv+2*v+1);
			B1[2] = vv;

			B2[0] = (ww-2*w+1);
			B2[1] = (-2*ww+2*w+1);
			B2[2] = ww;

			for(m=0;m<3;m++)
				for(n=0;n<3;n++)
					B12[m][n] = B1[m]*B2[n];


			funcX = funcY = funcZ = 0.0;
			for(n=0;n<3;n++)
			{
				func = 0.0;

				for(m=0;m<3;m++) 
					func += B12[m][n];

				func*=L[0];

				Mapping[i][4*n] = func*1/4;	
		
			}
			for(m=0;m<3;m++)
			{
				for(n=0;n<3;n++)
				{
					func = L[1]*B12[n][m];
					Mapping[i][4*m+n+1]= func*1/4;	
				}
			}

		
	}
	int N;
	for(int i=0;i<NbSurfacePoint;i++)
	{
		ii[0]=SurfacePointParaIdx[i][0];
		ii[1]=SurfacePointParaIdx[i][0]+1;
		ii[2]=SurfacePointParaIdx[i][0]+2;


		jj[0] = (SurfacePointParaIdx[i][1]+SurfCPNum-1)%SurfCPNum;
		jj[1] = SurfacePointParaIdx[i][1];
		jj[2] = (SurfacePointParaIdx[i][1]+1)%SurfCPNum;

		N=0;
		int N2=0;
		for(int m=0;m<3;m++){
			MappingMatrix(3*i,3*(ii[m]*(SurfCPNum+1)))=Mapping[i][N];
			MappingMatrix(3*i+1,3*(ii[m]*(SurfCPNum+1))+1)=Mapping[i][N];
			MappingMatrix(3*i+2,3*(ii[m]*(SurfCPNum+1))+2)=Mapping[i][N];
			MappingIndex(i,N2)=3*(ii[m]*(SurfCPNum+1));		N2+=1;
			MappingIndex(i,N2)=3*(ii[m]*(SurfCPNum+1)+1);	N2+=1;
			MappingIndex(i,N2)=3*(ii[m]*(SurfCPNum+1)+2);	N2+=1;
			 

			N+=1;
			for(int n=0;n<3;n++){
				MappingMatrix(3*i,3*(ii[m]*(SurfCPNum+1)+jj[n]+1))=Mapping[i][N];
				MappingMatrix(3*i+1,3*(ii[m]*(SurfCPNum+1)+jj[n]+1)+1)=Mapping[i][N];
				MappingMatrix(3*i+2,3*(ii[m]*(SurfCPNum+1)+jj[n]+1)+2)=Mapping[i][N];
				MappingIndex(i,N2)=3*(ii[m]*(SurfCPNum+1)+jj[n]+1);		N2+=1;
				MappingIndex(i,N2)=3*(ii[m]*(SurfCPNum+1)+jj[n]+1)+1;	N2+=1;
				MappingIndex(i,N2)=3*(ii[m]*(SurfCPNum+1)+jj[n]+1)+2;	N2+=1;


				N+=1;
			}
		}
	}
}


mat MyFFD::returnMappingMatforOneSurface(int triIdx,Vec3d Pos)
{
	mat m;
	m.set_size(3*NbPoint);
	m.fill(0.0);

	double A;
	double Ai;
	double N;
	int ii[2];
	int Index;

	A=((SurfacePoint[Face[triIdx][1]]-SurfacePoint[Face[triIdx][0]]).cross(SurfacePoint[Face[triIdx][2]]-SurfacePoint[Face[triIdx][0]])).norm();

	for(int i=0;i<3;i++)
	{
		ii[0]=(i+1)%3;
		ii[1]=(i+2)%3;
		Ai=((SurfacePoint[Face[triIdx][ii[1]]]-SurfacePoint[Face[triIdx][ii[0]]]).cross(Pos-SurfacePoint[Face[triIdx][ii[0]]])).norm();
		N=Ai/A;
		Index=Face[triIdx][i];
		for(int k=0;k<3*NbPoint;k++)
			m(k)+=N*MappingMatrix(Index,k);
	}

	return m;
}

mat MyFFD::returnMappingMatforOneSurface(int triIdx,Vec3d Pos,Vec3d normal)
{
	mat m;
	m.set_size(3*NbPoint);
	m.fill(0.0);

	double A;
	double Ai;
	double N;
	int ii[2];
	int Index;

	A=((SurfacePoint[Face[triIdx][1]]-SurfacePoint[Face[triIdx][0]]).cross(SurfacePoint[Face[triIdx][2]]-SurfacePoint[Face[triIdx][0]])).norm();

	for(int i=0;i<3;i++)
	{
		ii[0]=(i+1)%3;
		ii[1]=(i+2)%3;
		Ai=((SurfacePoint[Face[triIdx][ii[1]]]-SurfacePoint[Face[triIdx][ii[0]]]).cross(Pos-SurfacePoint[Face[triIdx][ii[0]]])).norm();
		N=Ai/A;
		Index=Face[triIdx][i];
		for(int j=0;j<3;j++)
			for(int k=0;k<3*NbPoint;k++)
				m(k)+=N*MappingMatrix(3*Index+j,k)*normal[j];
	}

	return m;
}

mat MyFFD::returnSparseMappingMatforSurface(int triIdx,Vec3d Pos,Vec3d normal)
{
	mat m;
	m.set_size(3*NbPoint);
	m.fill(0.0);


	mat sparseM;

	double A;
	double Ai;
	double N;
	int ii[2];
	int Index;

	A=((SurfacePoint[Face[triIdx][1]]-SurfacePoint[Face[triIdx][0]]).cross(SurfacePoint[Face[triIdx][2]]-SurfacePoint[Face[triIdx][0]])).norm();

	for(int i=0;i<3;i++)
	{
		ii[0]=(i+1)%3;
		ii[1]=(i+2)%3;
		Ai=((SurfacePoint[Face[triIdx][ii[1]]]-SurfacePoint[Face[triIdx][ii[0]]]).cross(Pos-SurfacePoint[Face[triIdx][ii[0]]])).norm();
		N=Ai/A;
		Index=Face[triIdx][i];
		int MaxNumberOfNonZero;
		for(int j=0;j<3;j++)
			for(int k=0;k<3*NbPoint;k++)
					m(k)+=N*MappingMatrix(3*Index+j,k)*normal[j];

	}

	int idx=0;
	for(int k=0;k<3*NbPoint;k++)
		if(m(k)){
			sparseM(idx,0)=k;
			sparseM(idx,1)=m(k);
			idx+=1;
		}

	return sparseM;
}

mat MyFFD::returnSparseMappingMatforEndoscope(int idx1, Vec3d pos, Vec3d normal)
{
	mat m;
	m.set_size(3*NbPoint);
	m.fill(0.0);

	mat sparseM;

	double m1,m2;
	m2=(pos-ControlPoint[idx1]).norm()/(ControlPoint[idx1+1]-ControlPoint[idx1]).norm();
	m1=1-m2;


	for(int j=0;j<3;j++){
		sparseM(j,0)=3*idx1+j;
		sparseM(j,1)=m1*normal(j);
	}

	for(int j=0;j<3;j++){
		sparseM(j,0)=3*(idx1+1)+j;
		sparseM(j,1)=m2*normal(j);
	}

	return sparseM;
}

std::vector<int> MyFFD::returnMappingMatIndexforOneSurface(int triIdx,Vec3d Pos)
{
	mat m;
	m.set_size(3*NbPoint);
	m.fill(0.0);

	int ii[2];
	int Index;

	for(int i=0;i<3;i++)
	{
		ii[0]=(i+1)%3;
		ii[1]=(i+2)%3;

		Index=Face[triIdx][i];
		for(int j=0;j<3;j++)
			for(int k=0;k<3*NbPoint;k++)
				m(k)+=MappingMatrix(3*Index+j,k);
	}

	std::vector<int> index;

	for(int i=0;i<3*NbPoint;i++)
		if(m(i))
			index.push_back(i);

	return index;
}



mat MyFFD::returnMappingMatforOneLattice(int idx1, int idx2, Vec3d pos)
{
	mat m;
	m.set_size(3*NbPoint);
	m.fill(0.0);

	double m1,m2;
	m2=(pos-ControlPoint[idx1]).norm()/(ControlPoint[idx2]-ControlPoint[idx1]).norm();
	m1=1-m2;


	for(int j=0;j<3;j++)
		m(3*idx1+j)=m1;

	for(int j=0;j<3;j++)
		m(3*idx2+j)=m2;

	return m;
}

mat MyFFD::returnMappingMatforOneLattice(int idx1, Vec3d pos, Vec3d normal)
{
	mat m;
	m.set_size(3*NbPoint);
	m.fill(0.0);

	double m1,m2;
	m2=(pos-ControlPoint[idx1]).norm()/(ControlPoint[idx1+1]-ControlPoint[idx1]).norm();
	m1=1-m2;


	for(int j=0;j<3;j++)
		m(3*idx1+j)=m1*normal(j);

	for(int j=0;j<3;j++)
		m(3*(idx1+1)+j)=m2*normal(j);

	return m;
}



double MyFFD::returnRatioOfArea(int triIdx,Vec3d Pos, int index)
{

	double A;
	double Ai;
	double N;
	int ii[2];

	A=((SurfacePoint[Face[triIdx][1]]-SurfacePoint[Face[triIdx][0]]).cross(SurfacePoint[Face[triIdx][2]]-SurfacePoint[Face[triIdx][0]])).norm();
	
	ii[0]=(index+1)%3;
	ii[1]=(index+2)%3;
	Ai=((SurfacePoint[Face[triIdx][ii[1]]]-SurfacePoint[Face[triIdx][ii[0]]]).cross(Pos-SurfacePoint[Face[triIdx][ii[0]]])).norm();
	N=Ai/A;

	return N;
}

void MyFFD::translate(Vec3d trans)
{
	for(int i=0;i<NbPoint;i++){
		ControlPoint[i]=ControlPoint[i]+trans;
		ControlPoint0[i]=ControlPoint[i];
	}
};

void MyFFD::translate(double x, double y, double z)
{
	Vec3d trans(x,y,z);

	for(int i=0;i<NbPoint;i++){
		ControlPoint[i]=ControlPoint[i]+trans;
		ControlPoint0[i]=ControlPoint[i];
	}
}

void MyFFD::rotate(Vec3d axis, double angle)
{
	Mat3x3d rot;
	computeRotationMatrix(axis, angle, rot);

	for(int i=0;i<NbPoint;i++){
		ControlPoint[i]=rot*ControlPoint[i];
		ControlPoint0[i]=ControlPoint[i];
	}
}

void MyFFD::rotate(double* rot)
{
	Mat3x3d Rot;
	Rot(0,0)=rot[0]; Rot(0,1)=rot[1]; Rot(0,2)=rot[2];
	Rot(1,0)=rot[4]; Rot(1,1)=rot[5]; Rot(1,2)=rot[6];
	Rot(2,0)=rot[8]; Rot(2,1)=rot[9]; Rot(2,2)=rot[10];
	rotate(Rot);
}

void MyFFD::rotate(Mat3x3d rot)
{
	for(int i=0;i<NbPoint;i++){
		ControlPoint[i]=rot*ControlPoint[i];
		ControlPoint0[i]=ControlPoint[i];
	}
}
void MyFFD::scale(float x, float y, float z)
{
	for(int i=0;i<NbPoint;i++)
	{
		ControlPoint[i][0]=ControlPoint[i][0]*x;
		ControlPoint[i][1]=ControlPoint[i][1]*y;
		ControlPoint[i][2]=ControlPoint[i][2]*z;
		ControlPoint0[i]=ControlPoint[i];
	}
}

void MyFFD::scale(float scale)
{
	for(int i=0;i<NbPoint;i++){
		ControlPoint[i]=ControlPoint[i]*scale;
		ControlPoint0[i]=ControlPoint[i];
	}
}

void MyFFD::computeRotationMatrix(Vec3d axis, double angle, Mat3x3d& rot)
{
	double x,y,z;
	x=axis[0];
	y=axis[1];
	z=axis[2];

	rot(0,0)=x*x+(y*y+z*z)*cos(angle);
	rot(1,1)=y*y+(x*x+z*z)*cos(angle);
	rot(2,2)=z*z+(x*x+y*y)*cos(angle);
	rot(0,1)=(1-cos(angle))*x*y+z*sin(angle);
	rot(1,0)=(1-cos(angle))*x*y-z*sin(angle);
	rot(0,2)=(1-cos(angle))*x*z-y*sin(angle);
	rot(2,0)=(1-cos(angle))*z*x+y*sin(angle);
	rot(1,2)=(1-cos(angle))*y*z+x*sin(angle);
	rot(2,1)=(1-cos(angle))*z*y-x*sin(angle);

	rot.transpose();
}

void MyFFD::computeControlPointNormal()
{

}



void MyFFD::setMassSpringEndoscope(double mass, double ks, double kd, double kb, double cb, double ed, double _dt)
{
	int p1,p2;
	double length;

	Mass=mass;	Ks=ks;	Kd=kd;	Ed=ed;

	//1. center to surface point
	for(int i=0;i<NbCon;i++)
	{
		p1=Con[i][0];
		p2=Con[i][1];
		length=(ControlPoint[p1]-ControlPoint[p2]).norm();
		m_Spring.addSpring(p1,p2,ks,kd,length);
	}
	M=mass;
	K=ks;
	Kb=kb;
	Cb=cb;
	C=kd;
	dt=_dt;
	Ed2=ed;
	
	local=new double*[6];
	for(int i=0;i<6;i++)
		local[i]=new double[6];

	Kglobal=new double*[3*NbPoint];
	for (int i=0;i<3*NbPoint;i++)
		Kglobal[i]=new double[3*NbPoint];

	Cglobal=new double*[3*NbPoint];
	for (int i=0;i<3*NbPoint;i++)
		Cglobal[i]=new double[3*NbPoint];

	Mglobal=new double*[3*NbPoint];
	for (int i=0;i<3*NbPoint;i++)
		Mglobal[i]=new double[3*NbPoint];

	Compliance=new double*[3*NbPoint];
	for (int i=0;i<3*NbPoint;i++)
		Compliance[i]=new double[3*NbPoint];

	MappingMatrixforLattice=new double*[3*NbPoint];
	for(int i=0;i<3*NbPoint;i++)
		MappingMatrixforLattice[i]=new double[3*NbPoint];

	x=new double[3*NbPoint];

	MassMatrix.set_size(3*NbPoint,3*NbPoint);
	DampingMatrix.set_size(3*NbPoint,3*NbPoint);
	StiffnessMatrix.set_size(3*NbPoint,3*NbPoint);
	Local.set_size(6,6);
	Displacement.set_size(3*NbPoint);
	EndoscopeTipDisplacement.set_size(3*NbPoint);
	EndoscopeTipDisplacement.fill(0.0);

	PrePos.set_size(3*NbPoint);
	for(int i=0;i<NbPoint;i++)
		for(int j=0;j<3;j++)
			PrePos(3*i+j)=ControlPoint[i][j];


	MassMatrix=eye(3*NbPoint,3*NbPoint);

	makeDiagonalMassmatrix();

	makeEndoscopeStiffness(true);
	makeEndoscopeDamping();
	makeMassmatrix2();
	makeEndoscopeCompliance(true);


}



void MyFFD::setMassSpringDamper(double mass, double ks, double kd, double ed, double _dt)
{
	int p1,p2;
	double length;

	Mass=mass;	Ks=ks;	Kd=kd;	Ed=ed;

	//1. center to surface point
	for(int i=0;i<NbCon;i++)
	{
		p1=Con[i][0];
		p2=Con[i][1];
		length=(ControlPoint[p1]-ControlPoint[p2]).norm();
		m_Spring.addSpring(p1,p2,ks,kd,length);
	}
	M=mass;
	K=ks;
	C=kd;
	dt=_dt;
	Ed2=ed;

	local=new double*[6];
	for(int i=0;i<6;i++)
		local[i]=new double[6];

	Kglobal=new double*[3*NbPoint];
	for (int i=0;i<3*NbPoint;i++)
		Kglobal[i]=new double[3*NbPoint];

	Cglobal=new double*[3*NbPoint];
	for (int i=0;i<3*NbPoint;i++)
		Cglobal[i]=new double[3*NbPoint];

	Mglobal=new double*[3*NbPoint];
	for (int i=0;i<3*NbPoint;i++)
		Mglobal[i]=new double[3*NbPoint];

	Compliance=new double*[3*NbPoint];
	for (int i=0;i<3*NbPoint;i++)
		Compliance[i]=new double[3*NbPoint];

	MappingMatrixforLattice=new double*[3*NbPoint];
	for(int i=0;i<3*NbPoint;i++)
		MappingMatrixforLattice[i]=new double[3*NbPoint];

	x=new double[3*NbPoint];

	MassMatrix.set_size(3*NbPoint,3*NbPoint);
	DampingMatrix.set_size(3*NbPoint,3*NbPoint);
	StiffnessMatrix.set_size(3*NbPoint,3*NbPoint);
	Local.set_size(6,6);
	Displacement.set_size(3*NbPoint);
	PrePos.set_size(3*NbPoint);
	for(int i=0;i<NbPoint;i++)
		for(int j=0;j<3;j++)
			PrePos(3*i+j)=ControlPoint[i][j];


	MassMatrix=eye(3*NbPoint,3*NbPoint);


	makeDiagonalMassmatrix();


}

void MyFFD::updatePosition(double dt)
{
	/* add gravity */

	for(int i=0;i<NbPoint;i++)
	{
		Force[i][2]-=9.8*1;
	}


	/* add internal force */
	m_Spring.addForce(Force,ControlPoint,Velocity);

	/* add environment damping force */
	for(int i=0;i<NbPoint;i++)
	{
		Force[i]-=Velocity[i]*Ed;
	}

	//////////////////////////
	/* Explicit integration */
	//////////////////////////

	for(int i=0;i<NbPoint;i++)
	{
		Vec3d dv=Force[i]/Mass*dt;
		Velocity[i]+=dv;
		ControlPoint[i]+=(Velocity[i]*dt);
	}
	//Position Contraint

	//Fixed constraint
	for(unsigned int i=0;i<FixedConstraint.size();i++)
		ControlPoint[FixedConstraint[i]]=ControlPoint0[FixedConstraint[i]];


	/* reset force */
	for(int i=0;i<NbPoint;i++)
		Force[i].clear();
}

void MyFFD::updatePosition(double dt,bool gravity)
{
	/* add gravity */
	if(gravity)
		for(int i=0;i<NbPoint;i++)
			Force[i][2]-=9.8*6;



	/* add internal force */
	m_Spring.addForce(Force,ControlPoint,Velocity);

	/* add environment damping force */
	for(int i=0;i<NbPoint;i++)
	{
		Force[i]-=Velocity[i]*Ed;
	}

	//////////////////////////
	/* Explicit integration */
	//////////////////////////

	for(int i=0;i<NbPoint;i++)
	{
		Vec3d dv=Force[i]/Mass*dt;
		Velocity[i]+=dv;
		ControlPoint[i]+=(Velocity[i]*dt);
	}
	//Position Contraint

	//Fixed constraint
	for(unsigned int i=0;i<FixedConstraint.size();i++)
		ControlPoint[FixedConstraint[i]]=ControlPoint0[FixedConstraint[i]];


	/* reset force */
	for(int i=0;i<NbPoint;i++)
		Force[i].clear();
}


void MyFFD::updatePositionWithBendingAndForce(double dt,Vec3d Gravity)
{
	/* add gravity */
	for(int i=0;i<NbPoint;i++)
		Force[i]+=Gravity;



	/* add internal force */
	m_Spring.addForce(Force,ControlPoint,Velocity);

	/* add Bending force */
	addBendingForce2();

	/* add environment damping force */
	for(int i=0;i<NbPoint;i++)
		Force[i]-=Velocity[i]*Ed;

	//////////////////////////
	/* Explicit integration */
	//////////////////////////

	for(int i=0;i<NbPoint;i++)
	{
		Vec3d dv=Force[i]/Mass*dt;
		Velocity[i]+=dv;
		ControlPoint[i]+=(Velocity[i]*dt);
	}
	//Position Contraint

	//Fixed constraint
	for(unsigned int i=0;i<FixedConstraint.size();i++)
		ControlPoint[FixedConstraint[i]]=ControlPoint0[FixedConstraint[i]];


	/* reset force */
	for(int i=0;i<NbPoint;i++)
		Force[i].clear();
}


void MyFFD::updatePositionWithForce(double dt,Vec3d Gravity)
{
	/* add gravity */
	for(int i=0;i<NbPoint;i++)
		Force[i]+=Gravity;



	/* add internal force */
	m_Spring.addForce(Force,ControlPoint,Velocity);

	/* add environment damping force */
	for(int i=0;i<NbPoint;i++)
	{
		Force[i]-=Velocity[i]*Ed;
	}


	//////////////////////////
	/* Explicit integration */
	//////////////////////////

	for(int i=0;i<NbPoint;i++)
	{
		Vec3d dv=Force[i]/Mass*dt;
		Velocity[i]+=dv;
		ControlPoint[i]+=(Velocity[i]*dt);
	}
	//Position Contraint

	//Fixed constraint
	for(unsigned int i=0;i<FixedConstraint.size();i++)
		ControlPoint[FixedConstraint[i]]=ControlPoint0[FixedConstraint[i]];


	/* reset force */
	for(int i=0;i<NbPoint;i++)
		Force[i].clear();
}

void MyFFD::updatePositionImplicit(double dt,Vec3d Gravity)
{
	//////////////////////////
	/* Implicit integration */
	//////////////////////////


	/* add gravity */
	for(int i=0;i<NbPoint;i++)
		Force[i]+=Gravity;

	makeStiffnessWithoutPenalty();
	makeDamping();
	
	/* add internal force */
	int i,j,k,l;
	int nbPoint=NbPoint;
	//clock_t  time_start, time_end;
//	time_start = clock();

#pragma omp parallel for shared(nbPoint) private(i,j,k,l)
	for(i=0;i<nbPoint;i++)
		for(j=0;j<3;j++)
			for(k=0;k<nbPoint;k++)
				for(l=0;l<3;l++)
					Force[i][j]+=(Mglobal[3*i+j][3*k+l]/(dt*dt)+Cglobal[3*i+j][3*k+l]/dt)*(ControlPoint[k][l]-ControlPoint0[k][l])+(Mglobal[3*i+j][3*k+l]/dt)*Velocity[k][l]-Kglobal[3*i+j][3*k+l]*ControlPoint0[k][l];
	

	/* add environment damping force */

	for(i=0;i<NbPoint;i++)
		Force[i]-=Velocity[i]*Ed;

	for(i=0;i<NbPoint;i++)
		for(j=0;j<3;j++)
			x[3*i+j]=Force[i][j];

	
	

	for(i=0;i<NbPoint;i++)
		PreControlPoint[i]=ControlPoint[i];
	
	for (int i=0;i<3*NbPoint;i++)
		for (int j=0;j<3*NbPoint;j++)
			Kglobal[i][j]+=Mglobal[i][j]/(dt*dt)+Cglobal[i][j]/dt;

	double penalty=30000;
	//Constraints
	for (int i=0;i<FixedConstraint.size();i++)
		for (int j=0;j<3;j++)
			Kglobal[3*FixedConstraint[i]+j][3*FixedConstraint[i]+j]+=penalty;

	//Position update
	if(GSolve(Kglobal,3*NbPoint,x))
		for(i=0;i<nbPoint;i++)
			for(j=0;j<3;j++)
				ControlPoint[i][j]=ControlPoint0[i][j]+x[3*i+j];

	//time_end=clock();
	//fprintf(fp,"%f\n",double(time_end - time_start) /double(CLOCKS_PER_SEC)*1000);
	//Velocity update

	for(i=0;i<NbPoint;i++)
		Velocity[i]=(ControlPoint[i]-PreControlPoint[i])/dt;

	/* reset force */
	for(i=0;i<NbPoint;i++)
		Force[i].clear();
}

void MyFFD::updatePositionFastImplicit(double dt,Vec3d Gravity)
{
	//////////////////////////
	/* Implicit integration */
	//////////////////////////


	/* add gravity */
	for(int i=0;i<NbPoint;i++)
		Force[i]+=Gravity;

	makeStiffnessMatrixWithoutPenalty();
	makeDampingMatrix();

	/* add internal force */

	mat Pos0,PrePos,Pos,Vel,force;
	Pos0.set_size(3*NbPoint); Pos.set_size(3*NbPoint); PrePos.set_size(3*NbPoint); Vel.set_size(3*NbPoint); force.set_size(3*NbPoint);

	for(int i=0;i<NbPoint;i++){
		for(int j=0;j<3;j++){
			Pos0(3*i+j)=ControlPoint0[i][j];
			Pos(3*i+j)=ControlPoint[i][j];
			Vel(3*i+j)=Velocity[i][j];
			force(3*i+j)=Force[i][j];
		}
	}
	force+=(MassMatrix/(dt*dt)+DampingMatrix/dt)*(Pos-Pos0)+(MassMatrix/dt)*Vel-StiffnessMatrix*Pos0;

	/* add environment damping force */
	force-=Ed*Vel;

	// Position update
	PrePos=Pos;

	// Solve

	StiffnessMatrix+=MassMatrix/(dt*dt)+DampingMatrix/dt;

	double penalty=30000;
	//Constraints
	for (int i=0;i<FixedConstraint.size();i++)
		for (int j=0;j<3;j++)
			StiffnessMatrix(3*FixedConstraint[i]+j,3*FixedConstraint[i]+j)+=penalty;

	Displacement=solve(StiffnessMatrix,force);

	//Position update
	
	Pos=Pos0+Displacement;

	Vel=(Pos-PrePos)/dt;

	//Update
	for(int i=0;i<NbPoint;i++){
		for(int j=0;j<3;j++){
			ControlPoint[i][j]=Pos(3*i+j);
			Velocity[i][j]=Vel(3*i+j);
		}
	}
	for(int i=0;i<NbPoint;i++)
		Force[i].clear();
	
}

void MyFFD::updateUpperImplicit(double dt,Vec3d Gravity)
{
	//////////////////////////
	/* Implicit integration */
	//////////////////////////


	/* add gravity */
	for(int i=0;i<NbPoint;i++)
		Force[i]+=Gravity;

	makeStiffnessMatrixWithoutPenaltyForUpper();
	makeDampingMatrixForUpper();

	/* add internal force */

	mat Pos0,PrePos,Pos,Vel,force;
	Pos0.set_size(3*NbPoint); Pos.set_size(3*NbPoint); PrePos.set_size(3*NbPoint); Vel.set_size(3*NbPoint); force.set_size(3*NbPoint);

	for(int i=0;i<NbPoint;i++){
		for(int j=0;j<3;j++){
			Pos0(3*i+j)=ControlPoint0[i][j];
			Pos(3*i+j)=ControlPoint[i][j];
			Vel(3*i+j)=Velocity[i][j];
			force(3*i+j)=Force[i][j];
		}
	}
	force+=(MassMatrix/(dt*dt)+DampingMatrix/dt)*(Pos-Pos0)+(MassMatrix/dt)*Vel-StiffnessMatrix*Pos0;

	/* add environment damping force */
	force-=Ed*Vel;

	// Position update
	PrePos=Pos;

	// Solve

	StiffnessMatrix+=MassMatrix/(dt*dt)+DampingMatrix/dt;

	double penalty=30000;
	//Constraints
	for (int i=0;i<FixedConstraint.size();i++)
		for (int j=0;j<3;j++)
			StiffnessMatrix(3*FixedConstraint[i]+j,3*FixedConstraint[i]+j)+=penalty;

	Displacement=solve(StiffnessMatrix,force);

	//Position update

	Pos=Pos0+Displacement;

	Vel=(Pos-PrePos)/dt;

	//Update
	for(int i=0;i<NbPoint;i++){
		for(int j=0;j<3;j++){
			ControlPoint[i][j]=Pos(3*i+j);
			Velocity[i][j]=Vel(3*i+j);
		}
	}
	for(int i=0;i<NbPoint;i++)
		Force[i].clear();

	updateSurfacePointusingMappingMatrix();
}

void MyFFD::updateUpperExplicit(double dt,Vec3d Gravity)
{

	makeStiffnessMatrixWithoutPenalty();
	makeDampingMatrix();

	/* add internal force */

	mat Pos0,PreDis,Pos,Vel,force;
	Pos0.set_size(3*NbPoint); Pos.set_size(3*NbPoint); PreDis.set_size(3*NbPoint); Vel.set_size(3*NbPoint); force.set_size(3*NbPoint);

	/* add gravity */
	for(int i=0;i<NbPoint;i++)
		Force[i]+=Gravity;

	for(int i=0;i<NbPoint;i++){
		for(int j=0;j<3;j++){
			Pos0(3*i+j)=ControlPoint0[i][j];
			Pos(3*i+j)=ControlPoint[i][j];
			Vel(3*i+j)=Velocity[i][j];
			force(3*i+j)=Force[i][j];
		}
	}
	force-=Ed*Vel;
	force+=-(MassMatrix/(dt*dt)-DampingMatrix/dt+StiffnessMatrix)*(PrePos-Pos0)-(-2*MassMatrix/(dt*dt)+DampingMatrix/dt)*(Pos-Pos0)-StiffnessMatrix*Pos0;
	PrePos=Pos;

	double penalty=30000;
	mat mass;
	mass.set_size(3*NbPoint);
	mass.fill(M);

	//Constraints
	for (int i=0;i<FixedConstraint.size();i++)
		for (int j=0;j<3;j++)
			mass(3*FixedConstraint[i]+j)+=penalty;

	mass/=(dt*dt);


	mat Displacement=force/mass;

	Pos=Pos0+Displacement;

	Vel=(Pos-PrePos)/dt;

	//Update
	for(int i=0;i<NbPoint;i++){
		for(int j=0;j<3;j++){
			ControlPoint[i][j]=Pos(3*i+j);
			Velocity[i][j]=Vel(3*i+j);
		}
	}
	for(int i=0;i<NbPoint;i++)
		Force[i].clear();

	updateSurfacePointusingMappingMatrix();
}


void MyFFD::updatePositionExplicit(double dt,Vec3d Gravity)
{

	makeStiffnessMatrixWithoutPenalty();
	makeDampingMatrix();

	/* add internal force */

	mat Pos0,PreDis,Pos,Vel,force;
	Pos0.set_size(3*NbPoint); Pos.set_size(3*NbPoint); PreDis.set_size(3*NbPoint); Vel.set_size(3*NbPoint); force.set_size(3*NbPoint);

	/* add gravity */
	for(int i=0;i<NbPoint;i++)
		Force[i]+=Gravity;

	for(int i=0;i<NbPoint;i++){
		for(int j=0;j<3;j++){
			Pos0(3*i+j)=ControlPoint0[i][j];
			Pos(3*i+j)=ControlPoint[i][j];
			Vel(3*i+j)=Velocity[i][j];
			force(3*i+j)=Force[i][j];
		}
	}
	force-=Ed*Vel;
	force+=-(MassMatrix/(dt*dt)-DampingMatrix/dt+StiffnessMatrix)*(PrePos-Pos0)-(-2*MassMatrix/(dt*dt)+DampingMatrix/dt)*(Pos-Pos0)-StiffnessMatrix*Pos0;
	PrePos=Pos;

	double penalty=30000;
	mat mass;
	mass.set_size(3*NbPoint);
	mass.fill(M);

	//Constraints
	for (int i=0;i<FixedConstraint.size();i++)
		for (int j=0;j<3;j++)
			mass(3*FixedConstraint[i]+j)+=penalty;

	mass/=(dt*dt);


	mat Displacement=force/mass;

	Pos=Pos0+Displacement;

	Vel=(Pos-PrePos)/dt;

	//Update
	for(int i=0;i<NbPoint;i++){
		for(int j=0;j<3;j++){
			ControlPoint[i][j]=Pos(3*i+j);
			Velocity[i][j]=Vel(3*i+j);
		}
	}
	for(int i=0;i<NbPoint;i++)
		Force[i].clear();

	updateSurfacePointusingMappingMatrix();
}

void MyFFD::updatePositionUsingStiffness(double dt,bool gravity)
{
	/* add gravity */
	if(gravity)
		for(int i=0;i<NbPoint;i++)
			Force[i][2]-=9.8*6;



	/* add internal force */

	makeStiffness();
	for(int i=0;i<NbPoint;i++)
	{
		for(int j=0;j<3;j++)
		{
			for(int k=0;k<NbPoint;k++)
				for(int l=0;l<3;l++)
					Force[i][j]-=Kglobal[3*i+j][3*k+l]*(ControlPoint[k][l]-ControlPoint0[k][l]);
		}
	}

	makeDamping();
	for(int i=0;i<NbPoint;i++)
	{
		for(int j=0;j<3;j++)
		{
			for(int k=0;k<NbPoint;k++)
				for(int l=0;l<3;l++)
					Force[i][j]-=Cglobal[3*i+j][3*k+l]*Velocity[k][l];
		}
	}


	/* add environment damping force */
	for(int i=0;i<NbPoint;i++)
	{
		Force[i]-=Velocity[i]*Ed;
	}

	//////////////////////////
	/* Explicit integration */
	//////////////////////////

	for(int i=0;i<NbPoint;i++)
	{
		Vec3d dv=Force[i]/Mass*dt;
		Velocity[i]+=dv;
		ControlPoint[i]+=(Velocity[i]*dt);
	}
	//Position Contraint

	//Fixed constraint
	for(unsigned int i=0;i<FixedConstraint.size();i++)
		ControlPoint[FixedConstraint[i]]=ControlPoint0[FixedConstraint[i]];


	/* reset force */
	for(int i=0;i<NbPoint;i++)
		Force[i].clear();
}

void MyFFD::updatePositionUsingStiffness(double dt,Vec3d Gravity)
{
	/* add gravity */

	for(int i=0;i<NbPoint;i++)
		Force[i]+=Gravity;



	/* add internal force */

	makeStiffness();
	for(int i=0;i<NbPoint;i++)
	{
		for(int j=0;j<3;j++)
		{
			for(int k=0;k<NbPoint;k++)
				for(int l=0;l<3;l++)
					Force[i][j]-=Kglobal[3*i+j][3*k+l]*(ControlPoint[k][l]);
		}
	}

	makeDamping();
	for(int i=0;i<NbPoint;i++)
	{
		for(int j=0;j<3;j++)
		{
			for(int k=0;k<NbPoint;k++)
				for(int l=0;l<3;l++)
					Force[i][j]-=Cglobal[3*i+j][3*k+l]*Velocity[k][l];
		}
	}


	/* add environment damping force */
	for(int i=0;i<NbPoint;i++)
	{
		Force[i]-=Velocity[i]*Ed;
	}

	//////////////////////////
	/* Explicit integration */
	//////////////////////////

	for(int i=0;i<NbPoint;i++)
	{
		Vec3d dv=Force[i]/Mass*dt;
		Velocity[i]+=dv;
		ControlPoint[i]+=(Velocity[i]*dt);
	}
	//Position Contraint

	//Fixed constraint
	for(unsigned int i=0;i<FixedConstraint.size();i++)
		ControlPoint[FixedConstraint[i]]=ControlPoint0[FixedConstraint[i]];


	/* reset force */
	for(int i=0;i<NbPoint;i++)
		Force[i].clear();
}

void MyFFD::addFixedConstraint(int idx)
{
	FixedConstraint.push_back(idx);
}

void MyFFD::addElasticConstraint(int idx, double K)
{
	Vec2d Constraint;
	Constraint[0]=idx;
	Constraint[1]=K;
	ElasticConstraint.push_back(Constraint);
}


void MyFFD::makeStiffnessWithoutPenalty()
{
	matrixClear(3*NbPoint,Kglobal);
	Vec3d p1,p2,p10,p20;
	double l,l0;
	int N[6];

	for(int i=0;i<NbCon;i++)
	{
		N[0]=3*Con[i][0];
		N[1]=3*Con[i][0]+1;
		N[2]=3*Con[i][0]+2;
		N[3]=3*Con[i][1];
		N[4]=3*Con[i][1]+1;
		N[5]=3*Con[i][1]+2;

		p1=ControlPoint[Con[i][0]];
		p2=ControlPoint[Con[i][1]];
		p10=ControlPoint0[Con[i][0]];
		p20=ControlPoint0[Con[i][1]];
		l=(p1-p2).norm();
		l0=(p10-p20).norm();
		makeLocalstiffness(p1,p2,K*(1-l0/l));
		for (int j=0;j<6;j++)
			for (int k=0;k<6;k++)
				Kglobal[N[j]][N[k]]+=local[j][k];
	}
}

void MyFFD::makeStiffness()
{
	matrixClear(3*NbPoint,Kglobal);
	Vec3d p1,p2,p10,p20;
	double l,l0;
	int N[6];

	for(int i=0;i<NbCon;i++)
	{
		N[0]=3*Con[i][0];
		N[1]=3*Con[i][0]+1;
		N[2]=3*Con[i][0]+2;
		N[3]=3*Con[i][1];
		N[4]=3*Con[i][1]+1;
		N[5]=3*Con[i][1]+2;

		p1=ControlPoint[Con[i][0]];
		p2=ControlPoint[Con[i][1]];
		p10=ControlPoint0[Con[i][0]];
		p20=ControlPoint0[Con[i][1]];
		l=(p1-p2).norm();
		l0=(p10-p20).norm();
		makeLocalstiffness(p1,p2,K*(1-l0/l));
		for (int j=0;j<6;j++)
			for (int k=0;k<6;k++)
				Kglobal[N[j]][N[k]]+=local[j][k];
	}

	double penalty=30000;
	//Constraints
	for (int i=0;i<FixedConstraint.size();i++)
		for (int j=0;j<3;j++)
			Kglobal[3*FixedConstraint[i]+j][3*FixedConstraint[i]+j]+=penalty;
}


void MyFFD::makeBendingStiffness()
{
	Vec3d p1,p2,p3;
	double l;
	int N[9];

	for(int i=0;i<NbCon-1;i++)
	{
		N[0]=3*Con[i][0];
		N[1]=3*Con[i][0]+1;
		N[2]=3*Con[i][0]+2;
		N[3]=3*Con[i][1];
		N[4]=3*Con[i][1]+1;
		N[5]=3*Con[i][1]+2;
		N[6]=3*Con[i+1][1];
		N[7]=3*Con[i+1][1]+1;
		N[8]=3*Con[i+1][1]+2;


		p1=ControlPoint[Con[i][0]];
		p2=ControlPoint[Con[i][1]];
		p3=ControlPoint[Con[i+1][1]];

		makeLocalBendingstiffness1(p1,p2,p3,Kb);
		for (int j=0;j<6;j++)
			for (int k=0;k<6;k++)
				Kglobal[N[j]][N[k+3]]+=local[j][k];
		makeLocalBendingstiffness2(p1,p2,p3,Kb);
		for (int j=0;j<6;j++)
			for (int k=0;k<6;k++)
				Kglobal[N[j+3]][N[k]]+=local[j][k];
	}
}

void MyFFD::makeDamping()
{
	matrixClear(3*NbPoint,Cglobal);
	Vec3d p1,p2;

	int N[6];

	for(int i=0;i<NbCon;i++)
	{
		N[0]=3*Con[i][0];
		N[1]=3*Con[i][0]+1;
		N[2]=3*Con[i][0]+2;
		N[3]=3*Con[i][1];
		N[4]=3*Con[i][1]+1;
		N[5]=3*Con[i][1]+2;

		p1=ControlPoint[Con[i][0]];
		p2=ControlPoint[Con[i][1]];
		makeLocalstiffness(p1,p2,C);
		for (int j=0;j<6;j++)
			for (int k=0;k<6;k++)
				Cglobal[N[j]][N[k]]+=local[j][k];
	}
}

void MyFFD::makeBendingDamping()
{
	Vec3d p1,p2,p3;
	double l;
	int N[9];

	for(int i=0;i<NbCon-1;i++)
	{
		N[0]=3*Con[i][0];
		N[1]=3*Con[i][0]+1;
		N[2]=3*Con[i][0]+2;
		N[3]=3*Con[i][1];
		N[4]=3*Con[i][1]+1;
		N[5]=3*Con[i][1]+2;
		N[6]=3*Con[i+1][1];
		N[7]=3*Con[i+1][1]+1;
		N[8]=3*Con[i+1][1]+2;


		p1=ControlPoint[Con[i][0]];
		p2=ControlPoint[Con[i][1]];
		p3=ControlPoint[Con[i+1][1]];

		makeLocalBendingdamping1(p1,p2,p3,Cb);
		for (int j=0;j<6;j++)
			for (int k=0;k<6;k++)
				Cglobal[N[j]][N[k]]+=local[j][k];
		makeLocalBendingdamping2(p1,p2,p3,Cb);
		for (int j=0;j<6;j++)
			for (int k=0;k<6;k++)
				Cglobal[N[j+3]][N[k+3]]+=local[j][k];
	}
}

//void MyFFD::makeMassmatrix()
//{
//
//	matrixClear(3*NbPoint,Mglobal);
//
//	Vec3d p1,p2;
//
//	int N[6];
//
//	for(int i=0;i<NbCon;i++)
//	{
//		N[0]=3*Con[i][0];
//		N[1]=3*Con[i][0]+1;
//		N[2]=3*Con[i][0]+2;
//		N[3]=3*Con[i][1];
//		N[4]=3*Con[i][1]+1;
//		N[5]=3*Con[i][1]+2;
//
//		p1=ControlPoint[Con[i][0]];
//		p2=ControlPoint[Con[i][1]];
//		makeLocalmass2(p1,p2,M);
//		for (int j=0;j<6;j++)
//			for (int k=0;k<6;k++)
//				Mglobal[N[j]][N[k]]+=local[j][k];
//	}
//	
//}

void MyFFD::makeMassmatrix2()
{
	matrixClear(3*NbPoint,Mglobal);
	for (int i=0;i<3*NbPoint;i++)
	{
		for (int j=0;j<3*NbPoint;j++)
		{
			if (i==j)
				Mglobal[i][j]=M; 
			else
				Mglobal[i][j]=0;
		}
	}
}

void MyFFD::makeLocalstiffness(Vec3d p1, Vec3d p2, double K)
{
	double length;
	length=(p2-p1).norm();
	Vec3d Cos;
	for(int i=0;i<3;i++)
		Cos[i]=(p2[i]-p1[i])/length;
	

	int ii,jj;
	for (int i=0;i<6;i++)
	{
		for (int j=0;j<6;j++)
		{
			ii=i%3;	jj=j%3;

			if (i<3 && j<3)
			{
				local[i][j]=K*Cos[ii]*Cos[jj];
			} 
			else if(i<3 && j>2)
			{
				local[i][j]=-K*Cos[ii]*Cos[jj];
			}
			else if (i>2 && j<3)
			{
				local[i][j]=-K*Cos[ii]*Cos[jj];
			} 
			else if(i>2 && j>2)
			{
				local[i][j]=K*Cos[ii]*Cos[jj];
			}
		}
	}
}

void MyFFD::makeLocalBendingstiffness1(Vec3d p1, Vec3d p2, Vec3d p3, double K)
{
	Vec3d v1,v2;
	double length;
	v1=p2-p1;
	v2=p3-p2;
	length=v1.norm();
	length=length*length;	
	v2=v1.cross(v2);
	v1=v2.cross(v1);
	v1.normalize();

	Vec3d Cos;
	Cos=v1;

	int ii,jj;
	for (int i=0;i<6;i++)
	{
		for (int j=0;j<6;j++)
		{
			ii=i%3;	jj=j%3;

			if (i<3 && j<3)
			{
				local[i][j]=-K*Cos[ii]*Cos[jj];
			} 
			else if(i<3 && j>2)
			{
				local[i][j]=K*Cos[ii]*Cos[jj];
			}
			else if (i>2 && j<3)
			{
				local[i][j]=K*Cos[ii]*Cos[jj];
			} 
			else if(i>2 && j>2)
			{
				local[i][j]=-K*Cos[ii]*Cos[jj];
			}
		}
	}
}

void MyFFD::makeLocalBendingstiffness2(Vec3d p1, Vec3d p2, Vec3d p3, double K)
{
	Vec3d v1,v2;
	double length;
	v1=p2-p1;
	v2=p3-p2;
	length=v2.norm();
	length=length*length;

	v1.normalize();
	v2.normalize();


	
	
	v1=v1.cross(v2);
	v2=v1.cross(v2);
	v2.normalize();
	Vec3d Cos=v2;

	int ii,jj;
	for (int i=0;i<6;i++)
	{
		for (int j=0;j<6;j++)
		{
			ii=i%3;	jj=j%3;

			if (i<3 && j<3)
			{
				local[i][j]=-K*Cos[ii]*Cos[jj];
			} 
			else if(i<3 && j>2)
			{
				local[i][j]=K*Cos[ii]*Cos[jj];
			}
			else if (i>2 && j<3)
			{
				local[i][j]=K*Cos[ii]*Cos[jj];
			} 
			else if(i>2 && j>2)
			{
				local[i][j]=-K*Cos[ii]*Cos[jj];
			}
		}
	}
}


void MyFFD::makeLocalBendingdamping1(Vec3d p1, Vec3d p2, Vec3d p3, double C)
{
	Vec3d v1,v2;
	double length;
	v1=p2-p1;
	v2=p3-p2;
	length=v1.norm();
	length=length*length;	
	v2=v1.cross(v2);
	v1=v2.cross(v1);
	v1.normalize();

	Vec3d Cos;
	Cos=v1;

	int ii,jj;
	for (int i=0;i<6;i++)
	{
		for (int j=0;j<6;j++)
		{
			ii=i%3;	jj=j%3;

			if (i<3 && j<3)
			{
				local[i][j]=C*Cos[ii]*Cos[jj];
			} 
			else if(i<3 && j>2)
			{
				local[i][j]=0;
			}
			else if (i>2 && j<3)
			{
				local[i][j]=0;
			} 
			else if(i>2 && j>2)
			{
				local[i][j]=C*Cos[ii]*Cos[jj];
			}
		}
	}
}

void MyFFD::makeLocalBendingdamping2(Vec3d p1, Vec3d p2, Vec3d p3, double K)
{
	Vec3d v1,v2;
	double length;
	v1=p2-p1;
	v2=p3-p2;
	length=v2.norm();
	length=length*length;

	v1.normalize();
	v2.normalize();




	v1=v1.cross(v2);
	v2=v1.cross(v2);
	v2.normalize();
	Vec3d Cos=v2;

	int ii,jj;
	for (int i=0;i<6;i++)
	{
		for (int j=0;j<6;j++)
		{
			ii=i%3;	jj=j%3;

			if (i<3 && j<3)
			{
				local[i][j]=C*Cos[ii]*Cos[jj];
			} 
			else if(i<3 && j>2)
			{
				local[i][j]=0;
			}
			else if (i>2 && j<3)
			{
				local[i][j]=0;
			} 
			else if(i>2 && j>2)
			{
				local[i][j]=C*Cos[ii]*Cos[jj];
			}
		}
	}
}


void MyFFD::makeStiffnessMatrixWithoutPenaltyForUpper()
{
	StiffnessMatrix.fill(0.0);
	Vec3d p1,p2,p10,p20;
	double l,l0;
	int N[6];

	for(int i=0;i<NbCon;i++)
	{
		N[0]=3*Connectivity(i,0);
		N[1]=3*Connectivity(i,0)+1;
		N[2]=3*Connectivity(i,0)+2;
		N[3]=3*Connectivity(i,1);
		N[4]=3*Connectivity(i,1)+1;
		N[5]=3*Connectivity(i,1)+2;

		p1=ControlPoint[(int)Connectivity(i,0)];
		p2=ControlPoint[(int)Connectivity(i,1)];
		p10=ControlPoint0[(int)Connectivity(i,0)];
		p20=ControlPoint0[(int)Connectivity(i,1)];
		l=(p1-p2).norm();
		l0=(p10-p20).norm();
		makeLocalStiffnessMatrix(p1,p2,Connectivity(i,2)*(1-l0/l));
		for (int j=0;j<6;j++)
			for (int k=0;k<6;k++)
				StiffnessMatrix(N[j],N[k])+=Local(j,k);
	}
}

void MyFFD::makeStiffnessMatrixWithoutPenalty()
{
	StiffnessMatrix.fill(0.0);
	Vec3d p1,p2,p10,p20;
	double l,l0;
	int N[6];

	for(int i=0;i<NbCon;i++)
	{
		N[0]=3*Con[i][0];
		N[1]=3*Con[i][0]+1;
		N[2]=3*Con[i][0]+2;
		N[3]=3*Con[i][1];
		N[4]=3*Con[i][1]+1;
		N[5]=3*Con[i][1]+2;

		p1=ControlPoint[Con[i][0]];
		p2=ControlPoint[Con[i][1]];
		p10=ControlPoint0[Con[i][0]];
		p20=ControlPoint0[Con[i][1]];
		l=(p1-p2).norm();
		l0=(p10-p20).norm();
		makeLocalStiffnessMatrix(p1,p2,K*(1-l0/l));
		for (int j=0;j<6;j++)
			for (int k=0;k<6;k++)
				StiffnessMatrix(N[j],N[k])+=Local(j,k);
	}
}

void MyFFD::makeStiffnessMatrix()
{
	StiffnessMatrix.fill(0.0);
	Vec3d p1,p2,p10,p20;
	double l,l0;
	int N[6];

	for(int i=0;i<NbCon;i++)
	{
		N[0]=3*Con[i][0];
		N[1]=3*Con[i][0]+1;
		N[2]=3*Con[i][0]+2;
		N[3]=3*Con[i][1];
		N[4]=3*Con[i][1]+1;
		N[5]=3*Con[i][1]+2;

		p1=ControlPoint[Con[i][0]];
		p2=ControlPoint[Con[i][1]];
		p10=ControlPoint0[Con[i][0]];
		p20=ControlPoint0[Con[i][1]];
		l=(p1-p2).norm();
		l0=(p10-p20).norm();
		makeLocalStiffnessMatrix(p1,p2,K*(1-l0/l));
		for (int j=0;j<6;j++)
			for (int k=0;k<6;k++)
				StiffnessMatrix(N[j],N[k])+=Local(j,k);
	}

	double penalty=30000;
	//Constraints
	for (int i=0;i<FixedConstraint.size();i++)
		for (int j=0;j<3;j++)
			StiffnessMatrix(3*FixedConstraint[i]+j,3*FixedConstraint[i]+j)+=penalty;
}

void MyFFD::makeStiffnessMatrixForUpper(bool Constraint)
{
	StiffnessMatrix.fill(0.0);
	Vec3d p1,p2,p10,p20;
	double l,l0;
	int N[6];

	for(int i=0;i<NbCon;i++)
	{
		N[0]=3*Connectivity(i,0);
		N[1]=3*Connectivity(i,0)+1;
		N[2]=3*Connectivity(i,0)+2;
		N[3]=3*Connectivity(i,1);
		N[4]=3*Connectivity(i,1)+1;
		N[5]=3*Connectivity(i,1)+2;

		p1=ControlPoint[(int)Connectivity(i,0)];
		p2=ControlPoint[(int)Connectivity(i,1)];
		p10=ControlPoint0[(int)Connectivity(i,0)];
		p20=ControlPoint0[(int)Connectivity(i,1)];
		l=(p1-p2).norm();
		l0=(p10-p20).norm();
		makeLocalStiffnessMatrix(p1,p2,Connectivity(i,2)*(1-l0/l));
		for (int j=0;j<6;j++)
			for (int k=0;k<6;k++)
				StiffnessMatrix(N[j],N[k])+=Local(j,k);
	}
	if(Constraint){
		double penalty=30000;
		//Constraints
		for (int i=0;i<FixedConstraint.size();i++)
			for (int j=0;j<3;j++)
				StiffnessMatrix(3*FixedConstraint[i]+j,3*FixedConstraint[i]+j)+=penalty;
	}
}


void MyFFD::makeBendingStiffnessMatrix()
{
	Vec3d p1,p2,p3;
	double l;
	int N[9];

	for(int i=0;i<NbCon-1;i++)
	{
		N[0]=3*Con[i][0];
		N[1]=3*Con[i][0]+1;
		N[2]=3*Con[i][0]+2;
		N[3]=3*Con[i][1];
		N[4]=3*Con[i][1]+1;
		N[5]=3*Con[i][1]+2;
		N[6]=3*Con[i+1][1];
		N[7]=3*Con[i+1][1]+1;
		N[8]=3*Con[i+1][1]+2;


		p1=ControlPoint[Con[i][0]];
		p2=ControlPoint[Con[i][1]];
		p3=ControlPoint[Con[i+1][1]];

		l=(p2-p1).norm();
		makeLocalBendingStiffnessMatrix1(p1,p2,p3,Kb/(l*l));
		for (int j=0;j<6;j++)
			for (int k=0;k<6;k++)
				StiffnessMatrix(N[j],N[k+3])+=Local(j,k);

		l=(p3-p2).norm();
		makeLocalBendingStiffnessMatrix2(p1,p2,p3,Kb/(l*l));
		for (int j=0;j<6;j++)
			for (int k=0;k<6;k++)
				StiffnessMatrix(N[j+3],N[k])+=Local(j,k);
	}
}

void MyFFD::makeDampingMatrix()
{
	DampingMatrix.fill(0.0);
	Vec3d p1,p2;

	int N[6];

	for(int i=0;i<NbCon;i++)
	{
		N[0]=3*Con[i][0];
		N[1]=3*Con[i][0]+1;
		N[2]=3*Con[i][0]+2;
		N[3]=3*Con[i][1];
		N[4]=3*Con[i][1]+1;
		N[5]=3*Con[i][1]+2;

		p1=ControlPoint[Con[i][0]];
		p2=ControlPoint[Con[i][1]];
		makeLocalStiffnessMatrix(p1,p2,C);
		for (int j=0;j<6;j++)
			for (int k=0;k<6;k++)
				DampingMatrix(N[j],N[k])+=Local(j,k);
	}
}

void MyFFD::makeDampingMatrixForUpper()
{
	DampingMatrix.fill(0.0);
	Vec3d p1,p2;

	int N[6];

	for(int i=0;i<NbCon;i++)
	{
		N[0]=3*Connectivity(i,0);
		N[1]=3*Connectivity(i,0)+1;
		N[2]=3*Connectivity(i,0)+2;
		N[3]=3*Connectivity(i,1);
		N[4]=3*Connectivity(i,1)+1;
		N[5]=3*Connectivity(i,1)+2;

		p1=ControlPoint[(int)Connectivity(i,0)];
		p2=ControlPoint[(int)Connectivity(i,1)];
		makeLocalStiffnessMatrix(p1,p2,Connectivity(i,3));
		for (int j=0;j<6;j++)
			for (int k=0;k<6;k++)
				DampingMatrix(N[j],N[k])+=Local(j,k);
	}
}

void MyFFD::makeBendingDampingMatrix()
{
	Vec3d p1,p2,p3;
	double l;
	int N[9];

	for(int i=0;i<NbCon-1;i++)
	{
		N[0]=3*Con[i][0];
		N[1]=3*Con[i][0]+1;
		N[2]=3*Con[i][0]+2;
		N[3]=3*Con[i][1];
		N[4]=3*Con[i][1]+1;
		N[5]=3*Con[i][1]+2;
		N[6]=3*Con[i+1][1];
		N[7]=3*Con[i+1][1]+1;
		N[8]=3*Con[i+1][1]+2;


		p1=ControlPoint[Con[i][0]];
		p2=ControlPoint[Con[i][1]];
		p3=ControlPoint[Con[i+1][1]];
		l=(p2-p1).norm();
		makeLocalBendingDampingMatrix1(p1,p2,p3,Cb/(l*l));
		for (int j=0;j<6;j++)
			for (int k=0;k<6;k++)
				DampingMatrix(N[j],N[k])+=Local(j,k);
		l=(p2-p3).norm();
		makeLocalBendingDampingMatrix2(p1,p2,p3,Cb/(l*l));
		for (int j=0;j<6;j++)
			for (int k=0;k<6;k++)
				DampingMatrix(N[j+3],N[k+3])+=Local(j,k);
	}
}


void MyFFD::makeDiagonalMassmatrix()
{
	MassMatrix.fill(0.0);
	for (int i=0;i<3*NbPoint;i++)
	{
		for (int j=0;j<3*NbPoint;j++)
		{
			if (i==j)
				MassMatrix(i,j)=M; 
			else
				MassMatrix(i,j)=0;
		}
	}
}

void MyFFD::makeLocalStiffnessMatrix(Vec3d p1, Vec3d p2, double K)
{
	double length;
	length=(p2-p1).norm();
	Vec3d Cos;
	for(int i=0;i<3;i++)
		Cos[i]=(p2[i]-p1[i])/length;


	int ii,jj;
	for (int i=0;i<6;i++)
	{
		for (int j=0;j<6;j++)
		{
			ii=i%3;	jj=j%3;

			if (i<3 && j<3)
			{
				Local(i,j)=K*Cos[ii]*Cos[jj];
			} 
			else if(i<3 && j>2)
			{
				Local(i,j)=-K*Cos[ii]*Cos[jj];
			}
			else if (i>2 && j<3)
			{
				Local(i,j)=-K*Cos[ii]*Cos[jj];
			} 
			else if(i>2 && j>2)
			{
				Local(i,j)=K*Cos[ii]*Cos[jj];
			}
		}
	}
}

void MyFFD::makeLocalBendingStiffnessMatrix1(Vec3d p1, Vec3d p2, Vec3d p3, double K)
{
	Vec3d v1,v2;
	double length;
	v1=p2-p1;
	v2=p3-p2;
	length=v1.norm();
	length=length*length;	
	v2=v1.cross(v2);
	v1=v2.cross(v1);
	v1.normalize();

	Vec3d Cos;
	Cos=v1;

	int ii,jj;
	for (int i=0;i<6;i++)
	{
		for (int j=0;j<6;j++)
		{
			ii=i%3;	jj=j%3;

			if (i<3 && j<3)
			{
				Local(i,j)=-K*Cos[ii]*Cos[jj];
			} 
			else if(i<3 && j>2)
			{
				Local(i,j)=K*Cos[ii]*Cos[jj];
			}
			else if (i>2 && j<3)
			{
				Local(i,j)=K*Cos[ii]*Cos[jj];
			} 
			else if(i>2 && j>2)
			{
				Local(i,j)=-K*Cos[ii]*Cos[jj];
			}
		}
	}
}

void MyFFD::makeLocalBendingStiffnessMatrix2(Vec3d p1, Vec3d p2, Vec3d p3, double K)
{
	Vec3d v1,v2;
	double length;
	v1=p2-p1;
	v2=p3-p2;
	length=v2.norm();
	length=length*length;

	v1.normalize();
	v2.normalize();




	v1=v1.cross(v2);
	v2=v1.cross(v2);
	v2.normalize();
	Vec3d Cos=v2;

	int ii,jj;
	for (int i=0;i<6;i++)
	{
		for (int j=0;j<6;j++)
		{
			ii=i%3;	jj=j%3;

			if (i<3 && j<3)
			{
				Local(i,j)=-K*Cos[ii]*Cos[jj];
			} 
			else if(i<3 && j>2)
			{
				Local(i,j)=K*Cos[ii]*Cos[jj];
			}
			else if (i>2 && j<3)
			{
				Local(i,j)=K*Cos[ii]*Cos[jj];
			} 
			else if(i>2 && j>2)
			{
				Local(i,j)=-K*Cos[ii]*Cos[jj];
			}
		}
	}
}


void MyFFD::makeLocalBendingDampingMatrix1(Vec3d p1, Vec3d p2, Vec3d p3, double C)
{
	Vec3d v1,v2;
	double length;
	v1=p2-p1;
	v2=p3-p2;
	length=v1.norm();
	length=length*length;	
	v2=v1.cross(v2);
	v1=v2.cross(v1);
	v1.normalize();

	Vec3d Cos;
	Cos=v1;

	int ii,jj;
	for (int i=0;i<6;i++)
	{
		for (int j=0;j<6;j++)
		{
			ii=i%3;	jj=j%3;

			if (i<3 && j<3)
			{
				Local(i,j)=C*Cos[ii]*Cos[jj];
			} 
			else if(i<3 && j>2)
			{
				Local(i,j)=0;
			}
			else if (i>2 && j<3)
			{
				Local(i,j)=0;
			} 
			else if(i>2 && j>2)
			{
				Local(i,j)=C*Cos[ii]*Cos[jj];
			}
		}
	}
}

void MyFFD::makeLocalBendingDampingMatrix2(Vec3d p1, Vec3d p2, Vec3d p3, double K)
{
	Vec3d v1,v2;
	double length;
	v1=p2-p1;
	v2=p3-p2;
	length=v2.norm();
	length=length*length;

	v1.normalize();
	v2.normalize();




	v1=v1.cross(v2);
	v2=v1.cross(v2);
	v2.normalize();
	Vec3d Cos=v2;

	int ii,jj;
	for (int i=0;i<6;i++)
	{
		for (int j=0;j<6;j++)
		{
			ii=i%3;	jj=j%3;

			if (i<3 && j<3)
			{
				Local(i,j)=C*Cos[ii]*Cos[jj];
			} 
			else if(i<3 && j>2)
			{
				Local(i,j)=0;
			}
			else if (i>2 && j<3)
			{
				Local(i,j)=0;
			} 
			else if(i>2 && j>2)
			{
				Local(i,j)=C*Cos[ii]*Cos[jj];
			}
		}
	}
}

void MyFFD::makeLocalmass(Vec3d p1, Vec3d p2, double M)
{
	double length;
	length=(p2-p1).norm();
	Vec3d Cos;
	for(int i=0;i<3;i++)
		Cos[i]=(p2[i]-p1[i])/length;
	
	int ii,jj;
	for (int i=0;i<6;i++)
	{
		for (int j=0;j<6;j++)
		{
			ii=i%3;	jj=j%3;

			if (i<3 && j<3)
			{
				local[i][j]=2*M*Cos[ii]*Cos[jj];
			} 
			else if(i<3 && j>2)
			{
				local[i][j]=M*Cos[ii]*Cos[jj];
			}
			else if (i>2 && j<3)
			{
				local[i][j]=M*Cos[ii]*Cos[jj];
			} 
			else if(i>2 && j>2)
			{
				local[i][j]=2*M*Cos[ii]*Cos[jj];
			}
		}
	}
}

void MyFFD::makeLocalmass2(Vec3d p1, Vec3d p2, double M)
{
	double length;
	length=(p2-p1).norm();
	Vec3d Cos;
	for(int i=0;i<3;i++)
		Cos[i]=(p2[i]-p1[i])/length;

	int ii,jj;
	for (int i=0;i<6;i++)
	{
		for (int j=0;j<6;j++)
		{
			ii=i%3;	jj=j%3;

			if (i<3 && j<3)
			{
				local[i][j]=1*M*Cos[ii]*Cos[jj];
			} 
			else if(i<3 && j>2)
			{
				local[i][j]=0;
			}
			else if (i>2 && j<3)
			{
				local[i][j]=0;
			} 
			else if(i>2 && j>2)
			{
				local[i][j]=1*M*Cos[ii]*Cos[jj];
			}
		}
	}
}

void MyFFD::resetControlPoints()
{


}

bool MyFFD::GSolve(double** a,int n,double* &x)
{
	int maxrow;
	double tmp,tmp2,tmp3;
	int i,j,k;

	for (i=0;i<n;i++) {

		/* Find the row with the largest first value */
		maxrow = i;
		for (j=i+1;j<n;j++)
			if (abs(a[j][i]) > abs(a[maxrow][i]))
				maxrow = j;

		/* Swap the maxrow and ith row */
		tmp2= x[i];
		x[i]=x[maxrow];
		x[maxrow]=tmp2;
		for (k=i;k<n;k++) {
			tmp = a[i][k];
			a[i][k] = a[maxrow][k];
			a[maxrow][k] = tmp;
		}


		/* Singular matrix? */
		if (abs(a[i][i]) < EPS)
			return false;


		/* Eliminate the ith element of the jth row */
		for ( j=i+1;j<n;j++){ 
			tmp3=a[j][i] / a[i][i];
			for ( k=i;k<n;k++) 
				a[j][k] -= a[i][k] * tmp3;
			x[j] -= x[i] * tmp3;
		}
	}


	/* Do the back substitution */
	for (int j=n-1;j>=0;j--) {
		tmp = 0;
		for (int k=j+1;k<n;k++)
			tmp += a[j][k] * x[k];
		x[j] = (x[j] - tmp) / a[j][j];
	}
	return true;
}

bool MyFFD::GSolveParallel(double** a,int n,double* &x)
{
	int maxrow;
	double tmp,tmp2,tmp3;
	int i,j,k;
	bool flag=true;
	//#pragma omp parallel for private(i,j,k,tmp,tmp2,tmp3,maxrow) shared(n,flag)

	for (i=0;i<n;i++) {

		/* Find the row with the largest first value */
		maxrow = i;
		for (j=i+1;j<n;j++)
			if (abs(a[j][i]) > abs(a[maxrow][i]))
				maxrow = j;

		/* Swap the maxrow and ith row */
		tmp2= x[i];
		x[i]=x[maxrow];
		x[maxrow]=tmp2;
		for (k=i;k<n;k++) {
			tmp = a[i][k];
			a[i][k] = a[maxrow][k];
			a[maxrow][k] = tmp;
		}


		/* Singular matrix? */
		if (abs(a[i][i]) < EPS)
			flag=false;


		/* Eliminate the ith element of the jth row */
		for ( j=i+1;j<n;j++){ 
			tmp3=a[j][i] / a[i][i];
			for ( k=i;k<n;k++) 
				a[j][k] -= a[i][k] * tmp3;
			x[j] -= x[i] * tmp3;
		}
	}


	/* Do the back substitution */
	for (int j=n-1;j>=0;j--) {
		tmp = 0;
		for (int k=j+1;k<n;k++)
			tmp += a[j][k] * x[k];
		x[j] = (x[j] - tmp) / a[j][j];
	}
	return flag;
}

void MyFFD::matrixClear(int n, double** a)
{
	for (int i=0;i<n;i++)
		for(int j=0;j<n;j++)
			a[i][j]=0;

}

void MyFFD::matrixClear(int n, double* a)
{
	for (int i=0;i<n;i++)
		a[i]=0;

}

void MyFFD::makeCompliancematrix()
{
	makeStiffness();
	makeDamping();
	for (int i=0;i<3*NbPoint;i++)
		for (int j=0;j<3*NbPoint;j++)
			Compliance[i][j]=Mglobal[i][j]/(dt*dt)+Cglobal[i][j]/dt+Kglobal[i][j];
}

void MyFFD::makeCompliancematrix2()
{
	for (int i=0;i<3*NbPoint;i++)
		for (int j=0;j<3*NbPoint;j++)
			Compliance[i][j]=Mglobal[i][j]/(dt*dt)+Cglobal[i][j]/dt+Kglobal[i][j];
}

mat* MyFFD::returnaddressInverseCompliance()
{
	makeStiffness();
	makeDamping();
	mat C(3*NbPoint,3*NbPoint);
	mat	invC(3*NbPoint,3*NbPoint);
	for (int i=0;i<3*NbPoint;i++)
		for (int j=0;j<3*NbPoint;j++)
			C(i,j)=Mglobal[i][j]/(dt*dt)+Cglobal[i][j]/dt+Kglobal[i][j];
	invC=inv(C);
	return &invC;
}

mat MyFFD::returnInverseCompliance()
{
	makeStiffness();
	makeDamping();
	mat C(3*NbPoint,3*NbPoint);
	mat	invC(3*NbPoint,3*NbPoint);
	for (int i=0;i<3*NbPoint;i++)
		for (int j=0;j<3*NbPoint;j++)
			C(i,j)=Mglobal[i][j]/(dt*dt)+Cglobal[i][j]/dt+Kglobal[i][j];
	invC=inv(C);
	return invC;
}

mat MyFFD::returnImplicitCompliancematrix()
{
	makeStiffnessMatrix();
	makeDampingMatrix();
	mat C(3*NbPoint,3*NbPoint);
	C=MassMatrix/(dt*dt)+DampingMatrix/dt+StiffnessMatrix;
	C=inv(C);
	return C;

}

mat MyFFD::returnExplicitCompliancematrix()
{
	mat C(3*NbPoint,3*NbPoint);
	C=eye(3*NbPoint,3*NbPoint);
	C=C*dt*dt/M;
	return C;
}

mat MyFFD::returnExplicitCompliancematrixForUpper()
{
	mat C(3*NbPoint,3*NbPoint);
	C=eye(3*NbPoint,3*NbPoint);

	C=C*dt*dt/M;
	double penalty=30000;
	for(int i=0;i<FixedConstraint.size();i++)
		for(int j=0;j<3;j++)
			C(3*FixedConstraint[i]+j,3*FixedConstraint[i]+j)=dt*dt/(M+penalty);
	return C;

}

mat MyFFD::returnVertorFormExplicitCompliancematrixForUpper()
{
	mat C;
	C.set_size(3*NbPoint);
	C.fill(1.0);

	C=C*dt*dt/M;
	double penalty=30000;
	for(int i=0;i<FixedConstraint.size();i++)
		for(int j=0;j<3;j++)
			C(3*FixedConstraint[i]+j)=dt*dt/(M+penalty);
	return C;

}

mat MyFFD::returnExplicitDynamicStiffnessMatrixForUpper()
{
	mat C=MassMatrix;
	double penalty=30000;
	for(int i=0;i<FixedConstraint.size();i++)
		for(int j=0;j<3;j++)
			C(3*FixedConstraint[i]+j,3*FixedConstraint[i]+j)=+penalty;
	return C/(dt*dt);
}

mat MyFFD::returnExplicitDynamicStiffnessMatrixForFFD()
{
	mat C=MassMatrix;
	double penalty=30000;
	for(int i=0;i<FixedConstraint.size();i++)
		for(int j=0;j<3;j++)
			C(3*FixedConstraint[i]+j,3*FixedConstraint[i]+j)=+penalty;
	return C/(dt*dt);
}

mat MyFFD::returnExplicitCompliancematrixForEndoscope()
{
	mat C(3*NbPoint,3*NbPoint);
	C=eye(3*NbPoint,3*NbPoint);
	C*=dt*dt/M;
	int InsertedIndex=0;
	double penalty=30000;
	for(int i=0;i<FixedConstraint.size();i++)
		for(int j=0;j<3;j++)
			C(3*FixedConstraint[i]+j,3*FixedConstraint[i]+j)=dt*dt/(M+penalty);

	//Constraints
	for(int i=0;i<NbPoint;i++)
		if((ControlPoint[i]-InsertedPoint)*InsertedDirec>=0)
			InsertedIndex=i;

	if(InsertedIndex==NbPoint-1)
	{
		for (int j=0;j<3;j++){
			C(3*(NbPoint-2)+j,3*(NbPoint-2)+j)=dt*dt/(M+penalty);
			C(3*(NbPoint-1)+j,3*(NbPoint-1)+j)=dt*dt/(M+penalty);
		}
	}else{
		for(int i=InsertedIndex;i<NbPoint;i++)
			for (int j=0;j<3;j++)
				C(3*i+j,3*i+j)=dt*dt/(M+penalty);	
	}


	return C;
}

mat MyFFD::returnVectorFormExplicitCompliancematrixForEndoscope()
{
	mat C;
	C.set_size(3*NbPoint);
	C.fill(1.0);
	C*=dt*dt/M;
	int InsertedIndex=0;
	double penalty=30000;
	for(int i=0;i<FixedConstraint.size();i++)
		for(int j=0;j<3;j++)
			C(3*FixedConstraint[i]+j)=dt*dt/(M+penalty);

	//Constraints
	for(int i=0;i<NbPoint;i++)
		if((ControlPoint[i]-InsertedPoint)*InsertedDirec>=0)
			InsertedIndex=i;

	if(InsertedIndex==NbPoint-1)
	{
		for (int j=0;j<3;j++){
			C(3*(NbPoint-2)+j)=dt*dt/(M+penalty);
			C(3*(NbPoint-1)+j)=dt*dt/(M+penalty);
		}
	}else{
		for(int i=InsertedIndex;i<NbPoint;i++)
			for (int j=0;j<3;j++)
				C(3*i+j)=dt*dt/(M+penalty);	
	}


	return C;
}

mat MyFFD::returnImplicitCompliancematrixForEndoscope()
{
	makeEndoscopeStiffnessMatrix(true);
	makeEndoscopeDampingMatrix();
	mat C;
	C.set_size(3*NbPoint,3*NbPoint);
	C=MassMatrix/(dt*dt)+DampingMatrix/dt+StiffnessMatrix;
	C=inv(C);
	return C;
}

mat MyFFD::returnImplicitDynamicStiffnessMatrixForEndoscope()
{
	makeEndoscopeStiffnessMatrix(true);
	makeEndoscopeDampingMatrix();
	mat C;
	C.set_size(3*NbPoint,3*NbPoint);
	C=MassMatrix/(dt*dt)+DampingMatrix/dt+StiffnessMatrix;

	return C;
}

mat MyFFD::returnImplicitDynamicStiffnessMatrixForUpper()
{
	makeStiffnessMatrixForUpper(true);
	makeDampingMatrixForUpper();
	MassMatrix.save("Mass.txt",raw_ascii);
	DampingMatrix.save("Damping.txt",raw_ascii);
	StiffnessMatrix.save("Stiffness.txt",raw_ascii);
	mat C;
	C.set_size(3*NbPoint,3*NbPoint);
	C=MassMatrix/(dt*dt)+DampingMatrix/dt+StiffnessMatrix;
	C.save("C.txt",raw_ascii);

	return C;
}


mat MyFFD::returnImplicitCompliancematrixForUpper()
{
	makeStiffnessMatrixForUpper(true);
	makeDampingMatrixForUpper();
	mat C;
	C.set_size(3*NbPoint,3*NbPoint);
	C=MassMatrix/(dt*dt)+DampingMatrix/dt+StiffnessMatrix;
	C=inv(C);
	return C;
}

void MyFFD::addForcetoPoint(int idx,Vec3d force)
{
	Force[idx]+=force;
}

void MyFFD::addForcetoAllPoint(Vec3d force)
{
	for(int i=0;i<NbPoint;i++)
		Force[i]+=force;
}

void MyFFD::addForcetoAllPoint(mat force)
{
	//Update
	for(int i=0;i<NbPoint;i++){
		for(int j=0;j<3;j++){
			Force[i][j]=force(3*i+j);
		}
	}
}


void MyFFD::addForceBetweenPoint(int idx1,int idx2,Vec3d pos,Vec3d force)
{
	double m1,m2;
	m2=(pos-ControlPoint[idx1]).norm()/(ControlPoint[idx2]-ControlPoint[idx1]).norm();
	m1=1-m2;

	Force[idx1]+=force*m1;

	Force[idx2]+=force*m2;
}

void MyFFD::addForcetoSurfacePoint(int idx,Vec3d force)
{
	int ii[3];
	int jj[3];

	ii[0]=SurfacePointParaIdx[idx][0];
	ii[1]=SurfacePointParaIdx[idx][0]+1;
	ii[2]=SurfacePointParaIdx[idx][0]+2;


	jj[0] = (SurfacePointParaIdx[idx][1]+6-1)%6;
	jj[1] = SurfacePointParaIdx[idx][1];
	jj[2] = (SurfacePointParaIdx[idx][1]+1)%6;
	int N=0;
	for(int m=0;m<3;m++){
		addForcetoPoint(ii[m]*7,force*Mapping[idx][N]);	
		N+=1;
		for(int n=0;n<3;n++){
			addForcetoPoint(ii[m]*7+jj[n]+1,force*Mapping[idx][N]);	
			N+=1;
		}
	}
}

void MyFFD::addForcetoSurfacePoint(int triIdx, int trivertexIdx, Vec3d force)
{
	int ii[3];
	int jj[3];
	
	int idx=Face[triIdx][trivertexIdx];

	ii[0]=SurfacePointParaIdx[idx][0];
	ii[1]=SurfacePointParaIdx[idx][0]+1;
	ii[2]=SurfacePointParaIdx[idx][0]+2;


	jj[0] = (SurfacePointParaIdx[idx][1]+6-1)%6;
	jj[1] = SurfacePointParaIdx[idx][1];
	jj[2] = (SurfacePointParaIdx[idx][1]+1)%6;
	int N=0;
	for(int m=0;m<3;m++){
		addForcetoPoint(ii[m]*7,force*Mapping[idx][N]);	
		N+=1;
		for(int n=0;n<3;n++){
			addForcetoPoint(ii[m]*7+jj[n]+1,force*Mapping[idx][N]);	
			N+=1;
		}
	}
}

void MyFFD::addForcetoSurfaceTri(int triIdx, Vec3d Pos, Vec3d force)
{
	double A;
	double Ai;
	double N;
	int ii[2];
	
	A=((SurfacePoint[Face[triIdx][1]]-SurfacePoint[Face[triIdx][0]]).cross(SurfacePoint[Face[triIdx][2]]-SurfacePoint[Face[triIdx][0]])).norm();
	
	for(int i=0;i<3;i++)
	{
		ii[0]=(i+1)%3;
		ii[1]=(i+2)%3;
		Ai=((SurfacePoint[Face[triIdx][ii[1]]]-SurfacePoint[Face[triIdx][ii[0]]]).cross(Pos-SurfacePoint[Face[triIdx][ii[0]]])).norm();
		N=Ai/A;
		addForcetoSurfacePoint(triIdx,i,force*N);
	}
}

void MyFFD::addForcetoControlPoint(mat force)
{
	for(int i=0;i<NbPoint;i++)
		for(int j=0;j<3;j++)
			Force[i][j]+=force(3*i+j);
}

void MyFFD::addBendingForce()
{
	Vec3d force1,force2,force3;
	Vec3d v1,v2,v3;
	Vec3d p1,p2,p3;
	for(int i=0;i<NbCon-1;i++)
	{
		p1=ControlPoint[i];
		p2=ControlPoint[i+1];
		p3=ControlPoint[i+2];
		v1=p2-p1; v2=p3-p2;
		v1.normalize();
		v2.normalize();
		v3=v1.cross(v2);
		force1=v3.cross(v1);
		force3=v3.cross(v2);
		force1.normalize();
		force3.normalize();
		force1=force1*Kb*v3.norm()*(p2-p1).norm();
		force3=force3*Kb*v3.norm()*(p3-p2).norm();
		force2=-(force1+force3);
		Force[i]-=force1;
		Force[i+1]-=force2;
		Force[i+2]-=force3;
	}
}


void MyFFD::addBendingForce2()
{
	double l1,l2;

	Vec3d force1,force2,force3;
	Vec3d v1,v2,v3;
	Vec3d p1,p2,p3;
	double angle;
	for(int i=0;i<NbCon-1;i++)
	{
		p1=ControlPoint[i];
		p2=ControlPoint[i+1];
		p3=ControlPoint[i+2];
		l1=(p2-p1).norm();
		l2=(p3-p2).norm();
		v1=p2-p1; v2=p3-p2;
		v1.normalize();
		v2.normalize();
		v3=v1.cross(v2);
		angle=v3.norm();
		angle=asin(angle);
		force1=v3.cross(v1);
		force3=v3.cross(v2);
		force1.normalize();
		force3.normalize();
		force1=force1*Kb*angle/l1;
		force3=force3*Kb*angle/l2;
		force2=-(force1+force3);
		Force[i]-=force1;
		Force[i+1]-=force2;
		Force[i+2]-=force3;
	}
}

void MyFFD::setInitialVelocity(int Idx,Vec3d Vel)
{
	Velocity[Idx]=Vel;
}

void MyFFD::GLPrint(const char* text)
{
	glPushAttrib(GL_LIST_BIT);                        
	glListBase(1 - 32);                             
	glCallLists(strlen(text), GL_UNSIGNED_BYTE, text); 
	glPopAttrib();  
}

void MyFFD::printNodeNumber()
{
	glColor3f(1,0,0);
	for(int i=0;i<NbPoint;i++)
	{
		CString idx;
		char index[10];
		idx.Format(_T("%d"),i);
		glPushMatrix();
		glRasterPos3f(ControlPoint[i][0],ControlPoint[i][1],ControlPoint[i][2]);
		itoa(i,index,10);
		GLPrint(index);
		glPopMatrix();
	}
}


void MyFFD::loadStomachModel(char* filename)
{
	FILE *fp = fopen(filename,"r");
	int _axisCPNum;
	int _surfCPNum;
	Vec3d* _axisCP;
	Vec3d* _surfCP;
	int Nb=0;

	fscanf(fp,"%d %d\n",&_axisCPNum,&_surfCPNum);
	_axisCP=new Vec3d[_axisCPNum];
	_surfCP=new Vec3d[_axisCPNum*_surfCPNum];
	NbPoint=_axisCPNum*_surfCPNum+_axisCPNum;
	NbCon=12*_axisCPNum+(_surfCPNum+1)*(_axisCPNum-1);

	AxisCPNum=_axisCPNum;
	SurfCPNum=_surfCPNum;

	ControlPoint=new Vec3d[NbPoint]; 
	ControlPoint0=new Vec3d[NbPoint];
	Force=new Vec3d[NbPoint];
	Velocity=new Vec3d[NbPoint];
	Con=new Vec2i[NbCon];
	for(int i=0;i<_axisCPNum;i++)
	{
		fscanf(fp,"%lf %lf %lf\n",&_axisCP[i][0],&_axisCP[i][1],&_axisCP[i][2]);
		ControlPoint[Nb]=_axisCP[i];
		ControlPoint0[Nb]=ControlPoint[Nb];
		Nb+=1;
		for(int j=0;j<_surfCPNum;j++)
		{
			fscanf(fp,"%lf %lf %lf\n",&_surfCP[i*_surfCPNum+j][0],&_surfCP[i*_surfCPNum+j][1],&_surfCP[i*_surfCPNum+j][2]);
			ControlPoint[Nb]=_surfCP[i*_surfCPNum+j];
			ControlPoint0[Nb]=ControlPoint[Nb];
			Nb+=1;
		}
	}
	
	//Generate Connectivity
	Nb=0;
	for (int i=0;i<_axisCPNum;i++)
	{
		for (int j=0;j<_surfCPNum;j++)
		{
			Con[Nb][0]=i*(_surfCPNum+1);
			Con[Nb][1]=i*(_surfCPNum+1)+j+1;
			Nb+=1;
			if (j==_surfCPNum-1)
			{
				Con[Nb][0]=i*(_surfCPNum+1)+j+1;
				Con[Nb][1]=i*(_surfCPNum+1)+1;
				Nb+=1;

			}else{
				Con[Nb][0]=i*(_surfCPNum+1)+j+1;
				Con[Nb][1]=i*(_surfCPNum+1)+j+2;
				Nb+=1;
			}

			if (i!=_axisCPNum-1)
			{
				Con[Nb][0]=i*(_surfCPNum+1)+j+1;
				Con[Nb][1]=(i+1)*(_surfCPNum+1)+j+1;
				Nb+=1;
			}
		}
		if (i!=_axisCPNum-1)
		{
			Con[Nb][0]=i*(_surfCPNum+1);
			Con[Nb][1]=(i+1)*(_surfCPNum+1);
			Nb+=1;
		}
	}


	delete [] _axisCP;
	delete [] _surfCP;

	fscanf(fp,"%d\n",&NbCenterPoint);
	CenterPointPara=new Vec3d[NbCenterPoint];
	CenterPointParaIdx=new Vec2i[NbCenterPoint];

	for(int i=0;i<NbCenterPoint;i++)
		fscanf(fp,"%d %d %lf %lf %lf\n",&CenterPointParaIdx[i][0],&CenterPointParaIdx[i][1],&CenterPointPara[i][0],&CenterPointPara[i][1],&CenterPointPara[i][2]);



	fscanf(fp,"%d\n",&NbCircularPoint);
	NbSurfacePoint=NbCenterPoint*NbCircularPoint;
	SurfacePointPara=new Vec3d[NbSurfacePoint];
	SurfacePointParaIdx=new Vec2i[NbSurfacePoint];


	for(int i=0;i<NbCenterPoint;i++)
		for(int j=0;j<NbCircularPoint;j++)
			fscanf(fp,"%d %d %lf %lf %lf\n",&SurfacePointParaIdx[i*NbCircularPoint+j][0], &SurfacePointParaIdx[i*NbCircularPoint+j][1],&SurfacePointPara[i*NbCircularPoint+j][0], &SurfacePointPara[i*NbCircularPoint+j][1], &SurfacePointPara[i*NbCircularPoint+j][2]);


	CenterPoint=new Vec3d[NbCenterPoint];
	SurfacePoint=new Vec3d[NbSurfacePoint];

	Nb=0;
	for(int i=0;i<NbCenterPoint-1;i++){
		CenterPoint[i]=paraToSurfacePoint(CenterPointParaIdx[i][0],CenterPointParaIdx[i][1],CenterPointPara[i][0],CenterPointPara[i][1],CenterPointPara[i][2]);
		for(int j=0;j<NbCircularPoint;j++)
			SurfacePoint[i*NbCircularPoint+j]=paraToSurfacePoint(SurfacePointParaIdx[i*NbCircularPoint+j][0],SurfacePointParaIdx[i*NbCircularPoint+j][1],SurfacePointPara[i*NbCircularPoint+j][0],SurfacePointPara[i*NbCircularPoint+j][1],SurfacePointPara[i*NbCircularPoint+j][2]);
	}
	
	//Load face information
	fscanf(fp,"%d\n",&NbFace);

	Face=new Vec3i[NbFace];
	Nb=0;
	for (int i=0;i<NbFace;i++)
		fscanf(fp,"%d %d %d\n",&Face[i][0],&Face[i][1],&Face[i][2]);

	//Generate face index information
	FaceIndex=new int[NbCenterPoint];
	for(int i=0;i<NbCenterPoint;i++)
		FaceIndex[i]=NbCircularPoint*2*i;

	fclose(fp);

	computeAroundPoint();
};


void MyFFD::writeStomachModel(char* filename)
{
	FILE *fp = fopen(filename,"w");
	int Nb=0;

	fprintf(fp,"%d %d\n",AxisCPNum,SurfCPNum);

	//Write control point information
	for(int i=0;i<AxisCPNum;i++)
	{
		fprintf(fp,"%lf %lf %lf\n",ControlPoint[Nb][0],ControlPoint[Nb][1],ControlPoint[Nb][2]);

		Nb+=1;
		for(int j=0;j<SurfCPNum;j++)
		{
			fprintf(fp,"%lf %lf %lf\n",ControlPoint[Nb][0],ControlPoint[Nb][1],ControlPoint[Nb][2]);
			Nb+=1;
		}
	}


	//Write center point information
	fprintf(fp,"%d\n",NbCenterPoint);


	for(int i=0;i<NbCenterPoint;i++)
		fprintf(fp,"%d %d %lf %lf %lf\n",CenterPointParaIdx[i][0],CenterPointParaIdx[i][1],CenterPointPara[i][0],CenterPointPara[i][1],CenterPointPara[i][2]);


	//Write circular information
	fprintf(fp,"%d\n",NbCircularPoint);

	for(int i=0;i<NbCenterPoint;i++)
		for(int j=0;j<NbCircularPoint;j++)
			fprintf(fp,"%d %d %lf %lf %lf\n",SurfacePointParaIdx[i*NbCircularPoint+j][0], SurfacePointParaIdx[i*NbCircularPoint+j][1],SurfacePointPara[i*NbCircularPoint+j][0], SurfacePointPara[i*NbCircularPoint+j][1], SurfacePointPara[i*NbCircularPoint+j][2]);

	//Write face information
	fprintf(fp,"%d\n",NbFace);

	Face=new Vec3i[NbFace];
	Nb=0;
	for (int i=0;i<NbFace;i++)
		fprintf(fp,"%d %d %d\n",&Face[i][0],&Face[i][1],&Face[i][2]);

	fclose(fp);
};



void MyFFD::loadCenterlineBasedCP(char* filename)
{
	FILE *fp = fopen(filename,"r");
	int _axisCPNum;
	int _surfCPNum;
	Vec3d* _axisCP;
	Vec3d* _surfCP;
	int Nb=0;
	fscanf(fp,"%d %d\n",&_axisCPNum,&_surfCPNum);
	_axisCP=new Vec3d[_axisCPNum];
	_surfCP=new Vec3d[_axisCPNum*_surfCPNum];
	NbPoint=_axisCPNum*_surfCPNum+_axisCPNum;
	NbCon=12*_axisCPNum+7*(_axisCPNum-1);
	
	AxisCPNum=_axisCPNum;
	SurfCPNum=_surfCPNum;

	ControlPoint=new Vec3d[NbPoint]; 
	ControlPoint0=new Vec3d[NbPoint];
	Force=new Vec3d[NbPoint];
	Velocity=new Vec3d[NbPoint];
	Con=new Vec2i[NbCon];
	for(int i=0;i<_axisCPNum;i++)
	{
		fscanf(fp,"%lf %lf %lf\n",&_axisCP[i][0],&_axisCP[i][1],&_axisCP[i][2]);
		ControlPoint[Nb]=_axisCP[i];
		ControlPoint0[Nb]=ControlPoint[Nb];
		Nb+=1;
		for(int j=0;j<_surfCPNum;j++)
		{
			fscanf(fp,"%lf %lf %lf\n",&_surfCP[i*_surfCPNum+j][0],&_surfCP[i*_surfCPNum+j][1],&_surfCP[i*_surfCPNum+j][2]);
			ControlPoint[Nb]=_surfCP[i*_surfCPNum+j];
			ControlPoint0[Nb]=ControlPoint[Nb];
			Nb+=1;
		}
	}
	fclose(fp);

	Nb=0;
	for (int i=0;i<_axisCPNum;i++)
	{
		for (int j=0;j<_surfCPNum;j++)
		{
			Con[Nb][0]=i*(_surfCPNum+1);
			Con[Nb][1]=i*(_surfCPNum+1)+j+1;
			Nb+=1;
			if (j==_surfCPNum-1)
			{
				Con[Nb][0]=i*(_surfCPNum+1)+j+1;
				Con[Nb][1]=i*(_surfCPNum+1)+1;
				Nb+=1;

			}else{
				Con[Nb][0]=i*(_surfCPNum+1)+j+1;
				Con[Nb][1]=i*(_surfCPNum+1)+j+2;
				Nb+=1;
			}

			if (i!=_axisCPNum-1)
			{
				Con[Nb][0]=i*(_surfCPNum+1)+j+1;
				Con[Nb][1]=(i+1)*(_surfCPNum+1)+j+1;
				Nb+=1;
			}
		}
		if (i!=_axisCPNum-1)
		{
			Con[Nb][0]=i*(_surfCPNum+1);
			Con[Nb][1]=(i+1)*(_surfCPNum+1);
			Nb+=1;
		}
	}


	delete [] _axisCP;
	delete [] _surfCP;
};

void  MyFFD::loadParameter(char* filename)
{
	FILE *fp = fopen(filename,"r");

	fscanf(fp,"%d\n",&NbCenterPoint);
	CenterPointPara=new Vec3d[NbCenterPoint];
	CenterPointParaIdx=new Vec2i[NbCenterPoint];

	for(int i=0;i<NbCenterPoint;i++)
		fscanf(fp,"%lf %lf %lf %d %d\n",&CenterPointPara[i][0],&CenterPointPara[i][1],&CenterPointPara[i][2],&CenterPointParaIdx[i][0],&CenterPointParaIdx[i][1]);

	

	fscanf(fp,"%d\n",&NbCircularPoint);
	NbSurfacePoint=NbCenterPoint*NbCircularPoint;
	SurfacePointPara=new Vec3d[NbSurfacePoint];
	SurfacePointParaIdx=new Vec2i[NbSurfacePoint];

	int Nb=0;

	for(int i=0;i<NbCenterPoint;i++)
		for(int j=0;j<NbCircularPoint;j++)
			fscanf(fp,"%lf %lf %lf %d %d\n",&SurfacePointPara[i*NbCircularPoint+j][0], &SurfacePointPara[i*NbCircularPoint+j][1], &SurfacePointPara[i*NbCircularPoint+j][2], &SurfacePointParaIdx[i*NbCircularPoint+j][0], &SurfacePointParaIdx[i*NbCircularPoint+j][1]);
	
	
	CenterPoint=new Vec3d[NbCenterPoint];
	SurfacePoint=new Vec3d[NbSurfacePoint];

	Nb=0;
	for(int i=0;i<NbCenterPoint-1;i++){
		CenterPoint[i]=paraToSurfacePoint(CenterPointParaIdx[i][0],CenterPointParaIdx[i][1],CenterPointPara[i][0],CenterPointPara[i][1],CenterPointPara[i][2]);
		for(int j=0;j<NbCircularPoint;j++)
			SurfacePoint[i*NbCircularPoint+j]=paraToSurfacePoint(SurfacePointParaIdx[i*NbCircularPoint+j][0],SurfacePointParaIdx[i*NbCircularPoint+j][1],SurfacePointPara[i*NbCircularPoint+j][0],SurfacePointPara[i*NbCircularPoint+j][1],SurfacePointPara[i*NbCircularPoint+j][2]);
	}

	//generate Face
	NbFace=2*NbCircularPoint*(NbCenterPoint-1);
	Face=new Vec3i[NbFace];
	Nb=0;
	for (int i=0;i<NbCenterPoint-2;i++)
	{
		for (int j=0;j<NbCircularPoint;j++)
		{
			if(j!=NbCircularPoint-1){
				Face[Nb][0]=i*NbCircularPoint+j;
				Face[Nb][1]=(i+1)*NbCircularPoint+j;
				Face[Nb][2]=(i+1)*NbCircularPoint+j+1;
				Nb+=1;
				Face[Nb][0]=i*NbCircularPoint+j;
				Face[Nb][1]=(i+1)*NbCircularPoint+j+1;
				Face[Nb][2]=i*NbCircularPoint+j+1;
				Nb+=1;
			}else{
				Face[Nb][0]=i*NbCircularPoint+NbCircularPoint-1;
				Face[Nb][1]=(i+1)*NbCircularPoint+NbCircularPoint-1;
				Face[Nb][2]=(i+1)*NbCircularPoint;
				Nb+=1;
				Face[Nb][0]=i*NbCircularPoint+NbCircularPoint-1;
				Face[Nb][1]=(i+1)*NbCircularPoint;
				Face[Nb][2]=i*NbCircularPoint;
				Nb+=1;
			}
		}
	}
	FaceIndex=new int[NbCenterPoint];
	for(int i=0;i<NbCenterPoint;i++)
		FaceIndex[i]=NbCircularPoint*2*i;


	fclose(fp);
}


void  MyFFD::loadSurfaceVar(char* filename)
{
	FILE *fp = fopen(filename,"r");

	fscanf(fp,"%d\n",&NbCenterPoint);
	CenterPoint=new Vec3d[NbCenterPoint];
	

	for(int i=0;i<NbCenterPoint;i++)
		fscanf(fp,"%lf %lf %lf\n",&CenterPoint[i][0],&CenterPoint[i][1],&CenterPoint[i][2]);
	
	NbCircularPointVar=new int[NbCenterPoint];
	NbSurfacePoint=0;
	for(int i=0;i<NbCenterPoint;i++){
		fscanf(fp,"%d\n",&NbCircularPointVar[i]);
		NbSurfacePoint+=NbCircularPointVar[i];
	}
	SurfacePoint=new Vec3d[NbSurfacePoint];

	for(int i=0;i<NbSurfacePoint;i++)
		fscanf(fp,"%lf %lf %lf\n",&SurfacePoint[i][0], &SurfacePoint[i][1], &SurfacePoint[i][2]);

	
	fscanf(fp,"%d\n",&NbFace);
	int a[3];
	int b[3];
	int* NB;
	NB=new int[NbCenterPoint];
	for (int i=0;i<NbCenterPoint;i++)
	{
		NB[i]=0;
		for(int j=0;j<i;j++)
			NB[i]+=NbCircularPointVar[j];
	}
	
	Face=new Vec3i[NbFace];
	for(int i=0;i<NbFace;i++){
		fscanf(fp,"%d %d %d %d %d %d\n",&a[0],&b[0],&a[1],&b[1],&a[2],&b[2]);
		for(int j=0;j<3;j++)
			Face[i][j]=NB[a[j]]+b[j];
	}


	delete [] NB;
	fclose(fp);
}

void  MyFFD::loadParameterVar(char* filename)
{
	FILE *fp = fopen(filename,"r");

	fscanf(fp,"%d\n",&NbCenterPoint);
	CenterPointPara=new Vec3d[NbCenterPoint];
	CenterPointParaIdx=new Vec2i[NbCenterPoint];

	for(int i=0;i<NbCenterPoint;i++)
		fscanf(fp,"%lf %lf %lf %d %d\n",&CenterPointPara[i][0],&CenterPointPara[i][1],&CenterPointPara[i][2],&CenterPointParaIdx[i][0],&CenterPointParaIdx[i][1]);

	NbCircularPointVar=new int[NbCenterPoint];
	NbSurfacePoint=0;
	for(int i=0;i<NbCenterPoint;i++){
		fscanf(fp,"%d\n",&NbCircularPointVar[i]);
		NbSurfacePoint+=NbCircularPointVar[i];
	}


	SurfacePointPara=new Vec3d[NbSurfacePoint];
	SurfacePointParaIdx=new Vec2i[NbSurfacePoint];

	for(int i=0;i<NbSurfacePoint;i++)
		fscanf(fp,"%lf %lf %lf %d %d\n",&SurfacePointPara[i][0], &SurfacePointPara[i][1], &SurfacePointPara[i][2], &SurfacePointParaIdx[i][0], &SurfacePointParaIdx[i][1]);

	CenterPoint=new Vec3d[NbCenterPoint];
	SurfacePoint=new Vec3d[NbSurfacePoint];

	for(int i=0;i<NbCenterPoint;i++)
		CenterPoint[i]=paraToSurfacePoint(CenterPointParaIdx[i][0],CenterPointParaIdx[i][1],CenterPointPara[i][0],CenterPointPara[i][1],CenterPointPara[i][2]);

	for(int i=0;i<NbSurfacePoint;i++)
		SurfacePoint[i]=paraToSurfacePoint(SurfacePointParaIdx[i][0],SurfacePointParaIdx[i][1],SurfacePointPara[i][0],SurfacePointPara[i][1],SurfacePointPara[i][2]);

	fscanf(fp,"%d\n",&NbFace);
	int a[3];
	int b[3];
	int* NB;
	NB=new int[NbCenterPoint];
	for (int i=0;i<NbCenterPoint;i++)
	{
		NB[i]=0;
		for(int j=0;j<i;j++)
			NB[i]+=NbCircularPointVar[j];
	}

	Face=new Vec3i[NbFace];
	for(int i=0;i<NbFace;i++){
		fscanf(fp,"%d %d %d %d %d %d\n",&a[0],&b[0],&a[1],&b[1],&a[2],&b[2]);
		for(int j=0;j<3;j++)
			Face[i][j]=NB[a[j]]+b[j];
	}


	delete [] NB;
	fclose(fp);
}

void MyFFD::updateSurfacePoint()
{

	for(int i=0;i<NbCenterPoint;i++)
		CenterPoint[i]=paraToSurfacePoint(CenterPointParaIdx[i][0],CenterPointParaIdx[i][1],CenterPointPara[i][0],CenterPointPara[i][1],CenterPointPara[i][2]);
	
	for(int j=0;j<NbSurfacePoint;j++)
		SurfacePoint[j]=paraToSurfacePoint(SurfacePointParaIdx[j][0],SurfacePointParaIdx[j][1],SurfacePointPara[j][0],SurfacePointPara[j][1],SurfacePointPara[j][2]);
}

void MyFFD::updateSurfacePointusingMappingMatrix()
{

	int Nb=0;
	int ii[3],jj[3];
	int N;
	Vec3d temp;
	for(int i=0;i<NbCenterPoint;i++)
		CenterPoint[i]=paraToSurfacePoint(CenterPointParaIdx[i][0],CenterPointParaIdx[i][1],CenterPointPara[i][0],CenterPointPara[i][1],CenterPointPara[i][2]);
	
	for(int j=0;j<NbSurfacePoint;j++){


			ii[0]=SurfacePointParaIdx[j][0];
			ii[1]=SurfacePointParaIdx[j][0]+1;
			ii[2]=SurfacePointParaIdx[j][0]+2;


			jj[0] = (SurfacePointParaIdx[j][1]+6-1)%6;
			jj[1] = SurfacePointParaIdx[j][1];
			jj[2] = (SurfacePointParaIdx[j][1]+1)%6;
			N=0;
			for(int m=0;m<3;m++){
				temp +=ControlPoint[ii[m]*7]*Mapping[j][N];	
				N+=1;
				for(int n=0;n<3;n++){
					temp +=ControlPoint[ii[m]*7+jj[n]+1]*Mapping[j][N];	
					N+=1;
				}
			}
			SurfacePoint[j]=temp;
			temp.clear();
		}
	
}

void MyFFD::moveControlPoint( int Idx,Vec3d trans )
{
	ControlPoint[Idx]+=trans;
	ControlPoint0[Idx]+=trans;
}

int MyFFD::closestCenterPointIdx(Vec3d Point)
{
	int Idx=0;
	double min;
	min=10000000000000;
	for(int i=0;i<NbCenterPoint;i++)
	{
		if((Point-CenterPoint[i]).norm()<min){
			min=(Point-CenterPoint[i]).norm();
			Idx=i;
		}
	}

	return Idx;
}

int MyFFD::closestControlCenterPointIdx(Vec3d Point)
{
	int Idx;
	double min;
	min=10000000000000;
	for(int i=0;i<NbCenterPoint;i++)
	{
		if((Point-ControlPoint[7*i]).norm()<min){
			min=(Point-ControlPoint[7*i]).norm();
			Idx=i;
		}
	}

	return Idx;
}

int MyFFD::closestCircularPointIdx(int AxisIdx,Vec3d Point)
{
	int Idx;
	double min;
	min=10000000000000;
	for(int i=0;i<6;i++)
	{
		if((Point-ControlPoint[AxisIdx*7+i+1]).norm()<min)
		{
			min=(Point-ControlPoint[AxisIdx*7+i+1]).norm();
			Idx=i;
		}
	}
	return Idx;
}

void MyFFD::drawHyperPatch()
{
	for(int i=0;i<AxisCPNum-2;i++)
	{
		for(int j=0;j<SurfCPNum;j++)
			drawHyperPatch(i,j);
	}
}

void MyFFD::drawHyperPatchLine()
{
	for(int i=0;i<AxisCPNum-2;i++)
	{
		for(int j=0;j<SurfCPNum;j++)
			drawHyperPatchLine(i,j);
	}
}

void MyFFD::drawHyperPatch(int centerIdx, int circularIdx)
{
	GLUquadricObj *qobj = 0;
	qobj = gluNewQuadric();

	Vec3d pos[4];
	Vec3d normal;
	double v,w,dv,dw;
	int nbV=20;
	int nbW=20;

	//side 1
	w=0;
	dw=1.0/nbW;
	

	glBegin(GL_TRIANGLES);
	for(int i=0;i<nbV;i++)
	{
		pos[0]=paraToSurfacePoint(centerIdx, circularIdx, 0,0,w);
		pos[1]=paraToSurfacePoint(centerIdx, circularIdx, 1,0,w);
		pos[2]=paraToSurfacePoint(centerIdx, circularIdx, 1,0,w+dw);
		normal=computeNormal(pos[0],pos[1],pos[2]);
		glNormal3f((GLfloat)normal[0],(GLfloat)normal[1],(GLfloat)normal[2]);
		glVertex3f((GLfloat)pos[0][0],(GLfloat)pos[0][1],(GLfloat)pos[0][2]);
		glVertex3f((GLfloat)pos[1][0],(GLfloat)pos[1][1],(GLfloat)pos[1][2]);
		glVertex3f((GLfloat)pos[2][0],(GLfloat)pos[2][1],(GLfloat)pos[2][2]);

		pos[0]=paraToSurfacePoint(centerIdx, circularIdx, 1,0,w+dw);
		pos[1]=paraToSurfacePoint(centerIdx, circularIdx, 0,0,w+dw);
		pos[2]=paraToSurfacePoint(centerIdx, circularIdx, 0,0,w);
		normal=computeNormal(pos[0],pos[1],pos[2]);
		glNormal3f((GLfloat)normal[0],(GLfloat)normal[1],(GLfloat)normal[2]);
		glVertex3f((GLfloat)pos[0][0],(GLfloat)pos[0][1],(GLfloat)pos[0][2]);
		glVertex3f((GLfloat)pos[1][0],(GLfloat)pos[1][1],(GLfloat)pos[1][2]);
		glVertex3f((GLfloat)pos[2][0],(GLfloat)pos[2][1],(GLfloat)pos[2][2]);

		w+=dw;
	}
	glEnd();

	//side 2
	w=0;
	glBegin(GL_TRIANGLES);
	for(int i=0;i<nbV;i++)
	{
		pos[0]=paraToSurfacePoint(centerIdx, circularIdx, 0,1,w);
		pos[2]=paraToSurfacePoint(centerIdx, circularIdx, 1,1,w);
		pos[1]=paraToSurfacePoint(centerIdx, circularIdx, 1,1,w+dw);
		normal=computeNormal(pos[0],pos[1],pos[2]);
		glNormal3f((GLfloat)normal[0],(GLfloat)normal[1],(GLfloat)normal[2]);
		glVertex3f((GLfloat)pos[0][0],(GLfloat)pos[0][1],(GLfloat)pos[0][2]);
		glVertex3f((GLfloat)pos[1][0],(GLfloat)pos[1][1],(GLfloat)pos[1][2]);
		glVertex3f((GLfloat)pos[2][0],(GLfloat)pos[2][1],(GLfloat)pos[2][2]);

		pos[0]=paraToSurfacePoint(centerIdx, circularIdx, 1,1,w+dw);
		pos[2]=paraToSurfacePoint(centerIdx, circularIdx, 0,1,w+dw);
		pos[1]=paraToSurfacePoint(centerIdx, circularIdx, 0,1,w);
		normal=computeNormal(pos[0],pos[1],pos[2]);
		glNormal3f((GLfloat)normal[0],(GLfloat)normal[1],(GLfloat)normal[2]);
		glVertex3f((GLfloat)pos[0][0],(GLfloat)pos[0][1],(GLfloat)pos[0][2]);
		glVertex3f((GLfloat)pos[1][0],(GLfloat)pos[1][1],(GLfloat)pos[1][2]);
		glVertex3f((GLfloat)pos[2][0],(GLfloat)pos[2][1],(GLfloat)pos[2][2]);

		w+=dw;
	}
	glEnd();

	//up plate
	v=0;
	dv=1.0/nbV;
	pos[0]=paraToSurfacePoint(centerIdx, circularIdx, 0,0,1);

	glBegin(GL_TRIANGLES);
	for(int i=0;i<nbV;i++)
	{
		pos[1]=paraToSurfacePoint(centerIdx, circularIdx, 1, v, 1);
		pos[2]=paraToSurfacePoint(centerIdx, circularIdx, 1, v+dv,1);
		normal=computeNormal(pos[0],pos[1],pos[2]);
		glNormal3f((GLfloat)normal[0],(GLfloat)normal[1],(GLfloat)normal[2]);
		glVertex3f((GLfloat)pos[0][0],(GLfloat)pos[0][1],(GLfloat)pos[0][2]);
		glVertex3f((GLfloat)pos[1][0],(GLfloat)pos[1][1],(GLfloat)pos[1][2]);
		glVertex3f((GLfloat)pos[2][0],(GLfloat)pos[2][1],(GLfloat)pos[2][2]);
		v+=dv;
	}
	glEnd();

	//down plate
	v=0;
	pos[0]=paraToSurfacePoint(centerIdx, circularIdx, 0,0,0);

	glBegin(GL_TRIANGLES);
	for(int i=0;i<nbV;i++)
	{
		pos[2]=paraToSurfacePoint(centerIdx, circularIdx, 1, v, 0);
		pos[1]=paraToSurfacePoint(centerIdx, circularIdx, 1, v+dv,0);
		normal=computeNormal(pos[0],pos[1],pos[2]);
		glNormal3f((GLfloat)normal[0],(GLfloat)normal[1],(GLfloat)normal[2]);
		glVertex3f((GLfloat)pos[0][0],(GLfloat)pos[0][1],(GLfloat)pos[0][2]);
		glVertex3f((GLfloat)pos[1][0],(GLfloat)pos[1][1],(GLfloat)pos[1][2]);
		glVertex3f((GLfloat)pos[2][0],(GLfloat)pos[2][1],(GLfloat)pos[2][2]);
		v+=dv;
	}
	glEnd();

	//front
	w=0;
	dw=1.0/nbW;
	v=0;

	glBegin(GL_QUADS);
	for(int i=0;i<nbV;i++)
	{
		w=0;
		for(int j=0;j<nbW;j++)
		{
			pos[0]=paraToSurfacePoint(centerIdx, circularIdx,1,v,w);
			pos[1]=paraToSurfacePoint(centerIdx, circularIdx,1,v+dv,w);
			pos[2]=paraToSurfacePoint(centerIdx, circularIdx,1,v+dv,w+dw);
			pos[3]=paraToSurfacePoint(centerIdx, circularIdx,1,v,w+dw);
			normal=computeNormal(pos[0],pos[1],pos[2]);
			glNormal3f((GLfloat)normal[0],(GLfloat)normal[1],(GLfloat)normal[2]);
			glVertex3f((GLfloat)pos[0][0],(GLfloat)pos[0][1],(GLfloat)pos[0][2]);
			glVertex3f((GLfloat)pos[1][0],(GLfloat)pos[1][1],(GLfloat)pos[1][2]);
			glVertex3f((GLfloat)pos[2][0],(GLfloat)pos[2][1],(GLfloat)pos[2][2]);
			glVertex3f((GLfloat)pos[3][0],(GLfloat)pos[3][1],(GLfloat)pos[3][2]);
			w+=dw;
		}
		v+=dv;
	}
	glEnd();
}

void MyFFD::drawHyperPatchLine(int centerIdx, int circularIdx)
{
	GLUquadricObj *qobj = 0;
	qobj = gluNewQuadric();

	Vec3d pos[4];
	Vec3d normal;

	int nbV=20;
	int nbW=20;

	//side 1
	pos[0]=paraToSurfacePoint(centerIdx, circularIdx, 0,0,0);
	pos[1]=paraToSurfacePoint(centerIdx, circularIdx, 1,0,0);
	pos[2]=paraToSurfacePoint(centerIdx, circularIdx, 1,0,1);
	pos[3]=paraToSurfacePoint(centerIdx, circularIdx, 0,0,1);

	glBegin(GL_LINES);
	glVertex3f((GLfloat)pos[0][0],(GLfloat)pos[0][1],(GLfloat)pos[0][2]);
	glVertex3f((GLfloat)pos[1][0],(GLfloat)pos[1][1],(GLfloat)pos[1][2]);

	glVertex3f((GLfloat)pos[1][0],(GLfloat)pos[1][1],(GLfloat)pos[1][2]);
	glVertex3f((GLfloat)pos[2][0],(GLfloat)pos[2][1],(GLfloat)pos[2][2]);

	glVertex3f((GLfloat)pos[2][0],(GLfloat)pos[2][1],(GLfloat)pos[2][2]);
	glVertex3f((GLfloat)pos[3][0],(GLfloat)pos[3][1],(GLfloat)pos[3][2]);

	glVertex3f((GLfloat)pos[3][0],(GLfloat)pos[3][1],(GLfloat)pos[3][2]);
	glVertex3f((GLfloat)pos[0][0],(GLfloat)pos[0][1],(GLfloat)pos[0][2]);

	//side 2
	pos[0]=paraToSurfacePoint(centerIdx, circularIdx, 0,1,0);
	pos[1]=paraToSurfacePoint(centerIdx, circularIdx, 0,1,1);
	pos[2]=paraToSurfacePoint(centerIdx, circularIdx, 1,1,1);
	pos[3]=paraToSurfacePoint(centerIdx, circularIdx, 1,1,0);

	glVertex3f((GLfloat)pos[0][0],(GLfloat)pos[0][1],(GLfloat)pos[0][2]);
	glVertex3f((GLfloat)pos[1][0],(GLfloat)pos[1][1],(GLfloat)pos[1][2]);

	glVertex3f((GLfloat)pos[1][0],(GLfloat)pos[1][1],(GLfloat)pos[1][2]);
	glVertex3f((GLfloat)pos[2][0],(GLfloat)pos[2][1],(GLfloat)pos[2][2]);

	glVertex3f((GLfloat)pos[2][0],(GLfloat)pos[2][1],(GLfloat)pos[2][2]);
	glVertex3f((GLfloat)pos[3][0],(GLfloat)pos[3][1],(GLfloat)pos[3][2]);

	glVertex3f((GLfloat)pos[3][0],(GLfloat)pos[3][1],(GLfloat)pos[3][2]);
	glVertex3f((GLfloat)pos[0][0],(GLfloat)pos[0][1],(GLfloat)pos[0][2]);


	//up plate
	double v=0;
	double dv=1.0/nbV;

	for(int i=0;i<nbV;i++)
	{
		pos[1]=paraToSurfacePoint(centerIdx, circularIdx, 1, v, 1);
		pos[2]=paraToSurfacePoint(centerIdx, circularIdx, 1, v+dv,1);

		glVertex3f((GLfloat)pos[1][0],(GLfloat)pos[1][1],(GLfloat)pos[1][2]);
		glVertex3f((GLfloat)pos[2][0],(GLfloat)pos[2][1],(GLfloat)pos[2][2]);

		v+=dv;
	}
	//down plate

	v=0;
	for(int i=0;i<nbV;i++)
	{
		pos[1]=paraToSurfacePoint(centerIdx, circularIdx, 1, v, 0);
		pos[2]=paraToSurfacePoint(centerIdx, circularIdx, 1, v+dv,0);

		glVertex3f((GLfloat)pos[1][0],(GLfloat)pos[1][1],(GLfloat)pos[1][2]);
		glVertex3f((GLfloat)pos[2][0],(GLfloat)pos[2][1],(GLfloat)pos[2][2]);

		v+=dv;
	}
	glEnd();
}

void MyFFD::makeCenterlinebasedSurface(int NbV,int NbW,int NbCen,double radius, Vec3d startpoint,Vec3d endpoint)
{
	Vec3d direc=endpoint-startpoint;
	direc.normalize();
	double length=(endpoint-startpoint).norm();
	Vec3d Xdirec,Ydirec,Zdirec;

	Xdirec[0]=1;
	Ydirec[2]=1;
	Zdirec[1]=1;


	
	int _axisCPNum=NbCen;
	int _surfCPNum=6;

	NbPoint=_axisCPNum*(_surfCPNum+1);
	NbCon=12*_axisCPNum+7*(_axisCPNum-1);

	ControlPoint=new Vec3d[NbPoint]; 
	PreControlPoint=new Vec3d[NbPoint];
	ControlPoint0=new Vec3d[NbPoint];
	Force=new Vec3d[NbPoint];
	Velocity=new Vec3d[NbPoint];
	Con=new Vec2i[NbCon];


	//make lattice
	int Nb=0;
	for(int i=0;i<_axisCPNum;i++)
	{
		ControlPoint[Nb]=startpoint+Zdirec*i*(length)/(NbCen-1);
		ControlPoint0[Nb]=ControlPoint[Nb];
		Nb=Nb+1;
		for(int j=0;j<_surfCPNum;j++)
		{
			ControlPoint[Nb]=startpoint+Zdirec*i*(length)/(NbCen-1)+Xdirec*cos(j*2*PI/6)*radius+Ydirec*sin(j*2*PI/6)*radius;
			ControlPoint0[Nb]=ControlPoint[Nb];
			PreControlPoint[Nb]=ControlPoint[Nb];
			Nb=Nb+1;
		}
	}

	Nb=0;
	for (int i=0;i<_axisCPNum;i++)
	{
		for (int j=0;j<_surfCPNum;j++)
		{
			Con[Nb][0]=i*(_surfCPNum+1);
			Con[Nb][1]=i*(_surfCPNum+1)+j+1;
			Nb+=1;
			if (j==_surfCPNum-1)
			{
				Con[Nb][0]=i*(_surfCPNum+1)+j+1;
				Con[Nb][1]=i*(_surfCPNum+1)+1;
				Nb+=1;

			}else{
				Con[Nb][0]=i*(_surfCPNum+1)+j+1;
				Con[Nb][1]=i*(_surfCPNum+1)+j+2;
				Nb+=1;
			}

			if (i!=_axisCPNum-1)
			{
				Con[Nb][0]=i*(_surfCPNum+1)+j+1;
				Con[Nb][1]=(i+1)*(_surfCPNum+1)+j+1;
				Nb+=1;
			}
		}
		if (i!=_axisCPNum-1)
		{
			Con[Nb][0]=i*(_surfCPNum+1);
			Con[Nb][1]=(i+1)*(_surfCPNum+1);
			Nb+=1;
		}
	}



	//make surface
	NbCenterPoint=NbW*(NbCen-2);
	CenterPointPara=new Vec3d[NbCenterPoint];
	CenterPointParaIdx=new Vec2i[NbCenterPoint];
	Nb=0;
	for(int i=0;i<NbCen-2;i++){
		for(int j=0;j<NbW;j++){
			CenterPointPara[Nb][0]=0;
			CenterPointPara[Nb][1]=0;
			CenterPointPara[Nb][2]=(double)j/NbW;
			CenterPointParaIdx[Nb][0]=i;
			CenterPointParaIdx[Nb][1]=0;
			Nb+=1;
		}
	}
	NbCircularPoint=_surfCPNum*NbV;
	NbSurfacePoint=NbCenterPoint*NbCircularPoint;
	SurfacePointPara=new Vec3d[NbSurfacePoint];
	SurfacePointParaIdx=new Vec2i[NbSurfacePoint];

	Nb=0;

	for(int i=0;i<NbCen-2;i++)
		for(int j=0;j<NbW;j++)
			for(int k=0;k<_surfCPNum;k++)
				for(int l=0;l<NbV;l++)
				{
					SurfacePointPara[Nb][0]=1;
					SurfacePointPara[Nb][1]=(double)l/NbV;
					SurfacePointPara[Nb][2]=(double)j/NbW;
					SurfacePointParaIdx[Nb][0]=i;
					SurfacePointParaIdx[Nb][1]=k;
					Nb+=1;
				}
			


	CenterPoint=new Vec3d[NbCenterPoint];
	SurfacePoint=new Vec3d[NbSurfacePoint];

	for(int i=0;i<NbCenterPoint;i++){
		CenterPoint[i]=paraToSurfacePoint(CenterPointParaIdx[i][0],CenterPointParaIdx[i][1],CenterPointPara[i][0],CenterPointPara[i][1],CenterPointPara[i][2]);
		for(int j=0;j<NbCircularPoint;j++)
			SurfacePoint[i*NbCircularPoint+j]=paraToSurfacePoint(SurfacePointParaIdx[i*NbCircularPoint+j][0],SurfacePointParaIdx[i*NbCircularPoint+j][1],SurfacePointPara[i*NbCircularPoint+j][0],SurfacePointPara[i*NbCircularPoint+j][1],SurfacePointPara[i*NbCircularPoint+j][2]);
	}

	//generate Face
	NbFace=2*NbCircularPoint*(NbCenterPoint-1);
	Face=new Vec3i[NbFace];
	Nb=0;
	for (int i=0;i<NbCenterPoint-1;i++)
	{
		for (int j=0;j<NbCircularPoint;j++)
		{
			if(j!=NbCircularPoint-1){
				Face[Nb][0]=i*NbCircularPoint+j;
				Face[Nb][1]=(i+1)*NbCircularPoint+j;
				Face[Nb][2]=(i+1)*NbCircularPoint+j+1;
				Nb+=1;
				Face[Nb][0]=i*NbCircularPoint+j;
				Face[Nb][1]=(i+1)*NbCircularPoint+j+1;
				Face[Nb][2]=i*NbCircularPoint+j+1;
				Nb+=1;
			}else{
				Face[Nb][0]=i*NbCircularPoint+NbCircularPoint-1;
				Face[Nb][1]=(i+1)*NbCircularPoint+NbCircularPoint-1;
				Face[Nb][2]=(i+1)*NbCircularPoint;
				Nb+=1;
				Face[Nb][0]=i*NbCircularPoint+NbCircularPoint-1;
				Face[Nb][1]=(i+1)*NbCircularPoint;
				Face[Nb][2]=i*NbCircularPoint;
				Nb+=1;
			}
		}
	}
	FaceIndex=new int[NbCenterPoint];
	for(int i=0;i<NbCenterPoint;i++)
		FaceIndex[i]=NbCircularPoint*2*i;
	FILE* check=fopen("../data/Check/checkcenterlinebased.txt","w");

	for(int i=0;i<NbCenterPoint;i++){
		fprintf(check,"%d %d %f %f %f\n",CenterPointParaIdx[i][0],CenterPointParaIdx[i][1],CenterPointPara[i][0],CenterPointPara[i][1],CenterPointPara[i][2]);
		for(int j=0;j<NbCircularPoint;j++)
			fprintf(check,"%d %d %f %f %f\n",SurfacePointParaIdx[i*NbCircularPoint+j][0],SurfacePointParaIdx[i*NbCircularPoint+j][1],SurfacePointPara[i*NbCircularPoint+j][0],SurfacePointPara[i*NbCircularPoint+j][1],SurfacePointPara[i*NbCircularPoint+j][2]);
	}
	computeAroundPoint();
}


void MyFFD::makeCenterlinebasedSurface(int NbV,int NbW,int NbCen,double radius, Vec3d startpoint,double length)
{
	Vec3d direc(1,0,0);
	direc.normalize();
	Vec3d Xdirec,Ydirec,Zdirec;

	Xdirec[0]=1;
	Ydirec[2]=1;
	Zdirec[1]=1;



	int _axisCPNum=NbCen;
	int _surfCPNum=6;

	NbPoint=_axisCPNum*(_surfCPNum+1);
	NbCon=12*_axisCPNum+7*(_axisCPNum-1);

	ControlPoint=new Vec3d[NbPoint]; 
	PreControlPoint=new Vec3d[NbPoint];
	ControlPoint0=new Vec3d[NbPoint];
	Force=new Vec3d[NbPoint];
	Velocity=new Vec3d[NbPoint];
	Con=new Vec2i[NbCon];


	//make lattice
	int Nb=0;
	for(int i=0;i<_axisCPNum;i++)
	{
		ControlPoint[Nb]=startpoint+Zdirec*i*(length)/(NbCen-1);
		ControlPoint0[Nb]=ControlPoint[Nb];
		Nb=Nb+1;
		for(int j=0;j<_surfCPNum;j++)
		{
			ControlPoint[Nb]=startpoint+Zdirec*i*(length)/(NbCen-1)+Xdirec*cos(j*2*PI/6)*radius+Ydirec*sin(j*2*PI/6)*radius;
			ControlPoint0[Nb]=ControlPoint[Nb];
			PreControlPoint[Nb]=ControlPoint[Nb];
			Nb=Nb+1;
		}
	}

	Nb=0;
	for (int i=0;i<_axisCPNum;i++)
	{
		for (int j=0;j<_surfCPNum;j++)
		{
			Con[Nb][0]=i*(_surfCPNum+1);
			Con[Nb][1]=i*(_surfCPNum+1)+j+1;
			Nb+=1;
			if (j==_surfCPNum-1)
			{
				Con[Nb][0]=i*(_surfCPNum+1)+j+1;
				Con[Nb][1]=i*(_surfCPNum+1)+1;
				Nb+=1;

			}else{
				Con[Nb][0]=i*(_surfCPNum+1)+j+1;
				Con[Nb][1]=i*(_surfCPNum+1)+j+2;
				Nb+=1;
			}

			if (i!=_axisCPNum-1)
			{
				Con[Nb][0]=i*(_surfCPNum+1)+j+1;
				Con[Nb][1]=(i+1)*(_surfCPNum+1)+j+1;
				Nb+=1;
			}
		}
		if (i!=_axisCPNum-1)
		{
			Con[Nb][0]=i*(_surfCPNum+1);
			Con[Nb][1]=(i+1)*(_surfCPNum+1);
			Nb+=1;
		}
	}



	//make surface
	NbCenterPoint=NbW*(NbCen-2);
	CenterPointPara=new Vec3d[NbCenterPoint];
	CenterPointParaIdx=new Vec2i[NbCenterPoint];
	Nb=0;
	for(int i=0;i<NbCen-2;i++){
		for(int j=0;j<NbW;j++){
			CenterPointPara[Nb][0]=0;
			CenterPointPara[Nb][1]=0;
			CenterPointPara[Nb][2]=(double)j/NbW;
			CenterPointParaIdx[Nb][0]=i;
			CenterPointParaIdx[Nb][1]=0;
			Nb+=1;
		}
	}
	NbCircularPoint=_surfCPNum*NbV;
	NbSurfacePoint=NbCenterPoint*NbCircularPoint;
	SurfacePointPara=new Vec3d[NbSurfacePoint];
	SurfacePointParaIdx=new Vec2i[NbSurfacePoint];

	Nb=0;

	for(int i=0;i<NbCen-2;i++)
		for(int j=0;j<NbW;j++)
			for(int k=0;k<_surfCPNum;k++)
				for(int l=0;l<NbV;l++)
				{
					SurfacePointPara[Nb][0]=1;
					SurfacePointPara[Nb][1]=(double)l/NbV;
					SurfacePointPara[Nb][2]=(double)j/NbW;
					SurfacePointParaIdx[Nb][0]=i;
					SurfacePointParaIdx[Nb][1]=k;
					Nb+=1;
				}



				CenterPoint=new Vec3d[NbCenterPoint];
				SurfacePoint=new Vec3d[NbSurfacePoint];

				for(int i=0;i<NbCenterPoint;i++){
					CenterPoint[i]=paraToSurfacePoint(CenterPointParaIdx[i][0],CenterPointParaIdx[i][1],CenterPointPara[i][0],CenterPointPara[i][1],CenterPointPara[i][2]);
					for(int j=0;j<NbCircularPoint;j++)
						SurfacePoint[i*NbCircularPoint+j]=paraToSurfacePoint(SurfacePointParaIdx[i*NbCircularPoint+j][0],SurfacePointParaIdx[i*NbCircularPoint+j][1],SurfacePointPara[i*NbCircularPoint+j][0],SurfacePointPara[i*NbCircularPoint+j][1],SurfacePointPara[i*NbCircularPoint+j][2]);
				}

				//generate Face
				NbFace=2*NbCircularPoint*(NbCenterPoint-1);
				Face=new Vec3i[NbFace];
				Nb=0;
				for (int i=0;i<NbCenterPoint-1;i++)
				{
					for (int j=0;j<NbCircularPoint;j++)
					{
						if(j!=NbCircularPoint-1){
							Face[Nb][0]=i*NbCircularPoint+j;
							Face[Nb][1]=(i+1)*NbCircularPoint+j;
							Face[Nb][2]=(i+1)*NbCircularPoint+j+1;
							Nb+=1;
							Face[Nb][0]=i*NbCircularPoint+j;
							Face[Nb][1]=(i+1)*NbCircularPoint+j+1;
							Face[Nb][2]=i*NbCircularPoint+j+1;
							Nb+=1;
						}else{
							Face[Nb][0]=i*NbCircularPoint+NbCircularPoint-1;
							Face[Nb][1]=(i+1)*NbCircularPoint+NbCircularPoint-1;
							Face[Nb][2]=(i+1)*NbCircularPoint;
							Nb+=1;
							Face[Nb][0]=i*NbCircularPoint+NbCircularPoint-1;
							Face[Nb][1]=(i+1)*NbCircularPoint;
							Face[Nb][2]=i*NbCircularPoint;
							Nb+=1;
						}
					}
				}
				FaceIndex=new int[NbCenterPoint];
				for(int i=0;i<NbCenterPoint;i++)
					FaceIndex[i]=NbCircularPoint*2*i;
				FILE* check=fopen("../data/Check/checkcenterlinebased.txt","w");

				for(int i=0;i<NbCenterPoint;i++){
					fprintf(check,"%d %d %f %f %f\n",CenterPointParaIdx[i][0],CenterPointParaIdx[i][1],CenterPointPara[i][0],CenterPointPara[i][1],CenterPointPara[i][2]);
					for(int j=0;j<NbCircularPoint;j++)
						fprintf(check,"%d %d %f %f %f\n",SurfacePointParaIdx[i*NbCircularPoint+j][0],SurfacePointParaIdx[i*NbCircularPoint+j][1],SurfacePointPara[i*NbCircularPoint+j][0],SurfacePointPara[i*NbCircularPoint+j][1],SurfacePointPara[i*NbCircularPoint+j][2]);
				}

	computeAroundPoint();
}

void MyFFD::makeCenterlinebasedSurface(char* filename,int NbV,int NbW,double radius)
{
	FILE *fp = fopen(filename,"r");
	int _axisCPNum;
	int	_surfCPNum;
	Vec3d* _axisCP;
	Vec3d* _surfCP;
	
	fscanf(fp,"%d %d\n",&_axisCPNum,&_surfCPNum);
	int NbCen=_axisCPNum;
	

	NbPoint=_axisCPNum*(_surfCPNum+1);
	NbCon=12*_axisCPNum+7*(_axisCPNum-1);

	_axisCP=new Vec3d[_axisCPNum];
	_surfCP=new Vec3d[NbPoint];
	ControlPoint=new Vec3d[NbPoint]; 
	PreControlPoint=new Vec3d[NbPoint];
	ControlPoint0=new Vec3d[NbPoint];
	Force=new Vec3d[NbPoint];
	Velocity=new Vec3d[NbPoint];
	Con=new Vec2i[NbCon];
	int Nb=0;
	for(int i=0;i<_axisCPNum;i++)
	{
		fscanf(fp,"%lf %lf %lf\n",&_axisCP[i][0],&_axisCP[i][1],&_axisCP[i][2]);
		ControlPoint[Nb]=_axisCP[i];
		ControlPoint0[Nb]=ControlPoint[Nb];
		Nb+=1;
		for(int j=0;j<_surfCPNum;j++)
		{
			fscanf(fp,"%lf %lf %lf\n",&_surfCP[i*_surfCPNum+j][0],&_surfCP[i*_surfCPNum+j][1],&_surfCP[i*_surfCPNum+j][2]);
			ControlPoint[Nb]=_surfCP[i*_surfCPNum+j];
			ControlPoint0[Nb]=ControlPoint[Nb];
			Nb+=1;
		}
	}
	fclose(fp);

	delete [] _axisCP;
	delete [] _surfCP;

	//make lattice


	Nb=0;
	for (int i=0;i<_axisCPNum;i++)
	{
		for (int j=0;j<_surfCPNum;j++)
		{
			Con[Nb][0]=i*(_surfCPNum+1);
			Con[Nb][1]=i*(_surfCPNum+1)+j+1;
			Nb+=1;
			if (j==_surfCPNum-1)
			{
				Con[Nb][0]=i*(_surfCPNum+1)+j+1;
				Con[Nb][1]=i*(_surfCPNum+1)+1;
				Nb+=1;

			}else{
				Con[Nb][0]=i*(_surfCPNum+1)+j+1;
				Con[Nb][1]=i*(_surfCPNum+1)+j+2;
				Nb+=1;
			}

			if (i!=_axisCPNum-1)
			{
				Con[Nb][0]=i*(_surfCPNum+1)+j+1;
				Con[Nb][1]=(i+1)*(_surfCPNum+1)+j+1;
				Nb+=1;
			}
		}
		if (i!=_axisCPNum-1)
		{
			Con[Nb][0]=i*(_surfCPNum+1);
			Con[Nb][1]=(i+1)*(_surfCPNum+1);
			Nb+=1;
		}
	}



	//make surface
	NbCenterPoint=NbW*(NbCen-2);
	CenterPointPara=new Vec3d[NbCenterPoint];
	CenterPointParaIdx=new Vec2i[NbCenterPoint];
	Nb=0;
	for(int i=0;i<NbCen-2;i++){
		for(int j=0;j<NbW;j++){
			CenterPointPara[Nb][0]=0;
			CenterPointPara[Nb][1]=0;
			CenterPointPara[Nb][2]=(double)j/NbW;
			CenterPointParaIdx[Nb][0]=i;
			CenterPointParaIdx[Nb][1]=0;
			Nb+=1;
		}
	}
	NbCircularPoint=_surfCPNum*NbV;
	NbSurfacePoint=NbCenterPoint*NbCircularPoint;
	SurfacePointPara=new Vec3d[NbSurfacePoint];
	SurfacePointParaIdx=new Vec2i[NbSurfacePoint];

	Nb=0;

	for(int i=0;i<NbCen-2;i++){
		for(int j=0;j<NbW;j++){
			for(int k=0;k<_surfCPNum;k++){
				for(int l=0;l<NbV;l++)
				{
					SurfacePointPara[Nb][0]=radius;
					SurfacePointPara[Nb][1]=(double)l/NbV;
					SurfacePointPara[Nb][2]=(double)j/NbW;
					SurfacePointParaIdx[Nb][0]=i;
					SurfacePointParaIdx[Nb][1]=k;
					Nb+=1;
				}
			}
		}
	}




				
	CenterPoint=new Vec3d[NbCenterPoint];
	SurfacePoint=new Vec3d[NbSurfacePoint];

	for(int i=0;i<NbCenterPoint;i++){
		CenterPoint[i]=paraToSurfacePoint(CenterPointParaIdx[i][0],CenterPointParaIdx[i][1],CenterPointPara[i][0],CenterPointPara[i][1],CenterPointPara[i][2]);
		for(int j=0;j<NbCircularPoint;j++)
			SurfacePoint[i*NbCircularPoint+j]=paraToSurfacePoint(SurfacePointParaIdx[i*NbCircularPoint+j][0],SurfacePointParaIdx[i*NbCircularPoint+j][1],SurfacePointPara[i*NbCircularPoint+j][0],SurfacePointPara[i*NbCircularPoint+j][1],SurfacePointPara[i*NbCircularPoint+j][2]);
	}

	//generate Face
	NbFace=2*NbCircularPoint*(NbCenterPoint-1);
	Face=new Vec3i[NbFace];
	Nb=0;
	for (int i=0;i<NbCenterPoint-1;i++)
	{
		for (int j=0;j<NbCircularPoint;j++)
		{
			if(j!=NbCircularPoint-1){
				Face[Nb][0]=i*NbCircularPoint+j;
				Face[Nb][1]=(i+1)*NbCircularPoint+j;
				Face[Nb][2]=(i+1)*NbCircularPoint+j+1;
				Nb+=1;
				Face[Nb][0]=i*NbCircularPoint+j;
				Face[Nb][1]=(i+1)*NbCircularPoint+j+1;
				Face[Nb][2]=i*NbCircularPoint+j+1;
				Nb+=1;
			}else{
				Face[Nb][0]=i*NbCircularPoint+NbCircularPoint-1;
				Face[Nb][1]=(i+1)*NbCircularPoint+NbCircularPoint-1;
				Face[Nb][2]=(i+1)*NbCircularPoint;
				Nb+=1;
				Face[Nb][0]=i*NbCircularPoint+NbCircularPoint-1;
				Face[Nb][1]=(i+1)*NbCircularPoint;
				Face[Nb][2]=i*NbCircularPoint;
				Nb+=1;
			}
		}
	}
	FaceIndex=new int[NbCenterPoint];
	for(int i=0;i<NbCenterPoint;i++)
		FaceIndex[i]=NbCircularPoint*2*i;	

	computeAroundPoint();
}

void MyFFD::makeUppergastrointestinalSurface(char* filename,int NbV,int NbW,double radius)
{
	FILE *fp = fopen(filename,"r");
	int _axisCPNum;
	int	_surfCPNum;
	Vec3d* _axisCP;
	Vec3d* _surfCP;

	fscanf(fp,"%d %d\n",&_axisCPNum,&_surfCPNum);
	int NbCen=_axisCPNum;

	int nbPoint=_axisCPNum*(_surfCPNum+1);
	int nbCon=12*_axisCPNum+7*(_axisCPNum-1);
	NbPoint=nbPoint+37;
	NbCon=nbCon+30+4+6+1+1+1+1;

	double K1,K2,K3;
	double C1,C2,C3;
	//Parameter1
	K1=50;
	K2=8000;
	K3=600;
	//Parameter2
	/*K1=600;
	K2=8000;
	K3=600;*/

	//Parameter3
	/*K1=600;
	K2=8000;
	K3=800;*/


	C1=K1*0.1;
	C2=K2*0.1;
	C3=K3*0.1;



	

	_axisCP=new Vec3d[_axisCPNum];
	_surfCP=new Vec3d[NbPoint];
	ControlPoint=new Vec3d[NbPoint]; 
	PreControlPoint=new Vec3d[NbPoint];
	ControlPoint0=new Vec3d[NbPoint];
	Force=new Vec3d[NbPoint];
	Velocity=new Vec3d[NbPoint];
	Con=new Vec2i[NbCon];

	Connectivity.set_size(NbCon,4);

	int Nb=0;
	for(int i=0;i<6;i++)
	{
		fscanf(fp,"%lf %lf %lf\n",&_axisCP[i][0],&_axisCP[i][1],&_axisCP[i][2]);
		ControlPoint[Nb]=_axisCP[i];
		ControlPoint[Nb][0]*=0.6;
		ControlPoint[Nb][1]*=0.6;
		ControlPoint0[Nb]=ControlPoint[Nb];
		Nb+=1;
		for(int j=0;j<_surfCPNum;j++)
		{
			fscanf(fp,"%lf %lf %lf\n",&_surfCP[i*_surfCPNum+j][0],&_surfCP[i*_surfCPNum+j][1],&_surfCP[i*_surfCPNum+j][2]);
			ControlPoint[Nb]=_surfCP[i*_surfCPNum+j];
			ControlPoint[Nb][0]*=0.6;
			ControlPoint[Nb][1]*=0.6;

			ControlPoint0[Nb]=ControlPoint[Nb];
			Nb+=1;
		}
	}

	for(int i=6;i<_axisCPNum-4;i++)
	{
		fscanf(fp,"%lf %lf %lf\n",&_axisCP[i][0],&_axisCP[i][1],&_axisCP[i][2]);
		ControlPoint[Nb]=_axisCP[i];
		ControlPoint0[Nb]=ControlPoint[Nb];
		Nb+=1;
		for(int j=0;j<_surfCPNum;j++)
		{
			fscanf(fp,"%lf %lf %lf\n",&_surfCP[i*_surfCPNum+j][0],&_surfCP[i*_surfCPNum+j][1],&_surfCP[i*_surfCPNum+j][2]);
			ControlPoint[Nb]=_surfCP[i*_surfCPNum+j];
			ControlPoint0[Nb]=ControlPoint[Nb];
			Nb+=1;
		}
	}

	for(int i=_axisCPNum-4;i<_axisCPNum;i++)
	{
		fscanf(fp,"%lf %lf %lf\n",&_axisCP[i][0],&_axisCP[i][1],&_axisCP[i][2]);
		ControlPoint[Nb]=_axisCP[i];
		ControlPoint[Nb][0]*=0.6;
		ControlPoint[Nb][1]*=0.6;
		ControlPoint0[Nb]=ControlPoint[Nb];
		Nb+=1;
		for(int j=0;j<_surfCPNum;j++)
		{
			fscanf(fp,"%lf %lf %lf\n",&_surfCP[i*_surfCPNum+j][0],&_surfCP[i*_surfCPNum+j][1],&_surfCP[i*_surfCPNum+j][2]);
			ControlPoint[Nb]=_surfCP[i*_surfCPNum+j];
			ControlPoint[Nb][0]*=0.6;
			ControlPoint[Nb][1]*=0.6;
			ControlPoint0[Nb]=ControlPoint[Nb];
			Nb+=1;
		}
	}

	//Constraint Point 

	for(int i=0;i<5;i++)
	{
		for(int j=0;j<6;j++)
		{
			ControlPoint[Nb]=ControlPoint[7*i+j+1]*2-ControlPoint[7*i];
			ControlPoint0[Nb]=ControlPoint[Nb];
			addFixedConstraint(Nb);
			Nb+=1;
		}
	}
	for(int i=0;i<4;i++){
		ControlPoint[Nb]=ControlPoint[7*(i+6)+1]*3-ControlPoint[7*(i+6)]*2;
		ControlPoint0[Nb]=ControlPoint[Nb];
		addFixedConstraint(Nb);
		Nb+=1;
	}


	ControlPoint[Nb]=ControlPoint[45]*2-ControlPoint[42];
	ControlPoint0[Nb]=ControlPoint[Nb];
	addFixedConstraint(Nb);
	Nb+=1;

	ControlPoint[Nb]=ControlPoint[86]*2.0-ControlPoint[84];
	ControlPoint0[Nb]=ControlPoint[Nb];
	addFixedConstraint(Nb);
	Nb+=1;

	ControlPoint[Nb]=ControlPoint[76]*2.0-ControlPoint[70];
	ControlPoint0[Nb]=ControlPoint[Nb];
	addFixedConstraint(Nb);
	Nb+=1;

	fclose(fp);

	delete [] _axisCP;
	delete [] _surfCP;

	//make lattice


	Nb=0;
	for (int i=0;i<_axisCPNum;i++)
	{
		for (int j=0;j<_surfCPNum;j++)
		{
			Con[Nb][0]=i*(_surfCPNum+1);
			Con[Nb][1]=i*(_surfCPNum+1)+j+1;
			Connectivity(Nb,0)=Con[Nb][0];
			Connectivity(Nb,1)=Con[Nb][1];
			Connectivity(Nb,2)=K1;
			Connectivity(Nb,3)=C1;
			Nb+=1;
			if (j==_surfCPNum-1)
			{
				Con[Nb][0]=i*(_surfCPNum+1)+j+1;
				Con[Nb][1]=i*(_surfCPNum+1)+1;
				Connectivity(Nb,0)=Con[Nb][0];
				Connectivity(Nb,1)=Con[Nb][1];
				Connectivity(Nb,2)=K1;
				Connectivity(Nb,3)=C1;
				Nb+=1;

			}else{
				Con[Nb][0]=i*(_surfCPNum+1)+j+1;
				Con[Nb][1]=i*(_surfCPNum+1)+j+2;
				Connectivity(Nb,0)=Con[Nb][0];
				Connectivity(Nb,1)=Con[Nb][1];
				Connectivity(Nb,2)=K1;
				Connectivity(Nb,3)=C1;
				Nb+=1;
			}

			if (i!=_axisCPNum-1)
			{
				Con[Nb][0]=i*(_surfCPNum+1)+j+1;
				Con[Nb][1]=(i+1)*(_surfCPNum+1)+j+1;
				Connectivity(Nb,0)=Con[Nb][0];
				Connectivity(Nb,1)=Con[Nb][1];
				Connectivity(Nb,2)=K1;
				Connectivity(Nb,3)=C1;
				Nb+=1;
			}
		}
		if (i!=_axisCPNum-1)
		{
			Con[Nb][0]=i*(_surfCPNum+1);
			Con[Nb][1]=(i+1)*(_surfCPNum+1);
			Connectivity(Nb,0)=Con[Nb][0];
			Connectivity(Nb,1)=Con[Nb][1];
			Connectivity(Nb,2)=K1;
			Connectivity(Nb,3)=C1;
			Nb+=1;
		}
	}

	for(int i=0;i<5;i++)
	{
		for(int j=0;j<6;j++)
		{
			Con[Nb][0]=(7*i+j+1);
			Con[Nb][1]=(nbPoint-1)+(6*i+j+1);
			Connectivity(Nb,0)=Con[Nb][0];
			Connectivity(Nb,1)=Con[Nb][1];
			Connectivity(Nb,2)=K2;
			Connectivity(Nb,3)=C2;
			Nb+=1;
		}
	}

	for(int i=0;i<4;i++)
	{
		Con[Nb][0]=(7*(i+6)+1);
		Con[Nb][1]=(nbPoint)+30+i;
		Connectivity(Nb,0)=Con[Nb][0];
		Connectivity(Nb,1)=Con[Nb][1];
		Connectivity(Nb,2)=K3;
		Connectivity(Nb,3)=C3;
		Nb+=1;
	}

	for(int i=0;i<6;i++)
	{
		Con[Nb][0]=45+7*i;
		Con[Nb][1]=(nbPoint)+33+1;
		Connectivity(Nb,0)=Con[Nb][0];
		Connectivity(Nb,1)=Con[Nb][1];
		Connectivity(Nb,2)=K2;
		Connectivity(Nb,3)=C2;
		Nb+=1;
	}

	Con[Nb][0]=86;
	Con[Nb][1]=(nbPoint)+33+2;
	Connectivity(Nb,0)=Con[Nb][0];
	Connectivity(Nb,1)=Con[Nb][1];
	Connectivity(Nb,2)=K2;
	Connectivity(Nb,3)=C2;
	Nb+=1;


	Con[Nb][0]=93;
	Con[Nb][1]=(nbPoint)+33+2;
	Connectivity(Nb,0)=Con[Nb][0];
	Connectivity(Nb,1)=Con[Nb][1];
	Connectivity(Nb,2)=K2;
	Connectivity(Nb,3)=C2;
	Nb+=1;


	Con[Nb][0]=79;
	Con[Nb][1]=(nbPoint)+33+1;
	Connectivity(Nb,0)=Con[Nb][0];
	Connectivity(Nb,1)=Con[Nb][1];
	Connectivity(Nb,2)=K2;
	Connectivity(Nb,3)=C2;
	Nb+=1;

	Con[Nb][0]=76;
	Con[Nb][1]=nbPoint+30+4+1+1;
	Connectivity(Nb,0)=Con[Nb][0];
	Connectivity(Nb,1)=Con[Nb][1];
	Connectivity(Nb,2)=K2;
	Connectivity(Nb,3)=C2;
	Nb+=1;


	//make surface
	NbCenterPoint=NbW*(NbCen-2);
	CenterPointPara=new Vec3d[NbCenterPoint];
	CenterPointParaIdx=new Vec2i[NbCenterPoint];
	Nb=0;
	for(int i=0;i<NbCen-2;i++){
		for(int j=0;j<NbW;j++){
			CenterPointPara[Nb][0]=0;
			CenterPointPara[Nb][1]=0;
			CenterPointPara[Nb][2]=(double)j/NbW;
			CenterPointParaIdx[Nb][0]=i;
			CenterPointParaIdx[Nb][1]=0;
			Nb+=1;
		}
	}
	NbCircularPoint=_surfCPNum*NbV;
	NbSurfacePoint=NbCenterPoint*NbCircularPoint;
	SurfacePointPara=new Vec3d[NbSurfacePoint];
	SurfacePointParaIdx=new Vec2i[NbSurfacePoint];

	Nb=0;
	double var,var2;
	var=randn()*0.005;

	for(int i=0;i<NbCen-2;i++){
		
		for(int j=0;j<NbW;j++){
			
			for(int k=0;k<_surfCPNum;k++){
				var=randn()*0.03;
				for(int l=0;l<NbV;l++)
				{
					SurfacePointPara[Nb][0]=radius-var;
					SurfacePointPara[Nb][1]=(double)l/NbV;
					SurfacePointPara[Nb][2]=(double)j/NbW;
					SurfacePointParaIdx[Nb][0]=i;
					SurfacePointParaIdx[Nb][1]=k;
					Nb+=1;
				}
			}
		}
	}





	CenterPoint=new Vec3d[NbCenterPoint];
	SurfacePoint=new Vec3d[NbSurfacePoint];

	for(int i=0;i<NbCenterPoint;i++){
		CenterPoint[i]=paraToSurfacePoint(CenterPointParaIdx[i][0],CenterPointParaIdx[i][1],CenterPointPara[i][0],CenterPointPara[i][1],CenterPointPara[i][2]);
		for(int j=0;j<NbCircularPoint;j++)
			SurfacePoint[i*NbCircularPoint+j]=paraToSurfacePoint(SurfacePointParaIdx[i*NbCircularPoint+j][0],SurfacePointParaIdx[i*NbCircularPoint+j][1],SurfacePointPara[i*NbCircularPoint+j][0],SurfacePointPara[i*NbCircularPoint+j][1],SurfacePointPara[i*NbCircularPoint+j][2]);
	}

	//generate Face
	NbFace=2*NbCircularPoint*(NbCenterPoint-1);
	Face=new Vec3i[NbFace];
	Nb=0;
	for (int i=0;i<NbCenterPoint-1;i++)
	{
		for (int j=0;j<NbCircularPoint;j++)
		{
			if(j!=NbCircularPoint-1){
				Face[Nb][0]=i*NbCircularPoint+j;
				Face[Nb][1]=(i+1)*NbCircularPoint+j;
				Face[Nb][2]=(i+1)*NbCircularPoint+j+1;
				Nb+=1;
				Face[Nb][0]=i*NbCircularPoint+j;
				Face[Nb][1]=(i+1)*NbCircularPoint+j+1;
				Face[Nb][2]=i*NbCircularPoint+j+1;
				Nb+=1;
			}else{
				Face[Nb][0]=i*NbCircularPoint+NbCircularPoint-1;
				Face[Nb][1]=(i+1)*NbCircularPoint+NbCircularPoint-1;
				Face[Nb][2]=(i+1)*NbCircularPoint;
				Nb+=1;
				Face[Nb][0]=i*NbCircularPoint+NbCircularPoint-1;
				Face[Nb][1]=(i+1)*NbCircularPoint;
				Face[Nb][2]=i*NbCircularPoint;
				Nb+=1;
			}
		}
	}
	FaceIndex=new int[NbCenterPoint];
	for(int i=0;i<NbCenterPoint;i++)
		FaceIndex[i]=NbCircularPoint*2*i;	
	FILE* Upper=fopen("../data/Check/Upper.txt","w");
	for(int i=0;i<NbFace;i++)
	{
		fprintf(Upper,"%d %d %d\n",Face[i][0],Face[i][1],Face[i][2]);
	}
	fclose(Upper);

	computeAroundPoint();
}

void MyFFD::makeUppergastrointestinalSurfaceV2(char* filename,int NbV,int NbW,double radius)
{
	FILE *fp = fopen(filename,"r");
	int _axisCPNum;
	int	_surfCPNum;
	Vec3d* _axisCP;
	Vec3d* _surfCP;

	fscanf(fp,"%d %d\n",&_axisCPNum,&_surfCPNum);
	int NbCen=_axisCPNum;

	int nbPoint=_axisCPNum*(_surfCPNum+1);
	int nbCon=12*_axisCPNum+7*(_axisCPNum-1);
	NbPoint=nbPoint;
	NbCon=nbCon;

	double K1,K2,K3;
	double C1,C2,C3;

	//Parameter1
	K1=50;
	K2=8000;
	K3=600;

	C1=K1*0.1;
	C2=K2*0.1;
	C3=K3*0.1;



	_axisCP=new Vec3d[_axisCPNum];
	_surfCP=new Vec3d[NbPoint];
	ControlPoint=new Vec3d[NbPoint]; 
	PreControlPoint=new Vec3d[NbPoint];
	ControlPoint0=new Vec3d[NbPoint];
	Force=new Vec3d[NbPoint];
	Velocity=new Vec3d[NbPoint];
	Con=new Vec2i[NbCon];

	Connectivity.set_size(NbCon,4);

	int Nb=0;
	for(int i=0;i<6;i++)
	{
		fscanf(fp,"%lf %lf %lf\n",&_axisCP[i][0],&_axisCP[i][1],&_axisCP[i][2]);
		ControlPoint[Nb]=_axisCP[i];
		ControlPoint[Nb][0]*=0.6;
		ControlPoint[Nb][1]*=0.6;
		ControlPoint0[Nb]=ControlPoint[Nb];
		Nb+=1;
		for(int j=0;j<_surfCPNum;j++)
		{
			fscanf(fp,"%lf %lf %lf\n",&_surfCP[i*_surfCPNum+j][0],&_surfCP[i*_surfCPNum+j][1],&_surfCP[i*_surfCPNum+j][2]);
			ControlPoint[Nb]=_surfCP[i*_surfCPNum+j];
			ControlPoint[Nb][0]*=0.6;
			ControlPoint[Nb][1]*=0.6;

			ControlPoint0[Nb]=ControlPoint[Nb];
			Nb+=1;
		}
	}

	for(int i=6;i<_axisCPNum-4;i++)
	{
		fscanf(fp,"%lf %lf %lf\n",&_axisCP[i][0],&_axisCP[i][1],&_axisCP[i][2]);
		ControlPoint[Nb]=_axisCP[i];
		ControlPoint0[Nb]=ControlPoint[Nb];
		Nb+=1;
		for(int j=0;j<_surfCPNum;j++)
		{
			fscanf(fp,"%lf %lf %lf\n",&_surfCP[i*_surfCPNum+j][0],&_surfCP[i*_surfCPNum+j][1],&_surfCP[i*_surfCPNum+j][2]);
			ControlPoint[Nb]=_surfCP[i*_surfCPNum+j];
			ControlPoint0[Nb]=ControlPoint[Nb];
			Nb+=1;
		}
	}

	for(int i=_axisCPNum-4;i<_axisCPNum;i++)
	{
		fscanf(fp,"%lf %lf %lf\n",&_axisCP[i][0],&_axisCP[i][1],&_axisCP[i][2]);
		ControlPoint[Nb]=_axisCP[i];
		ControlPoint[Nb][0]*=0.6;
		ControlPoint[Nb][1]*=0.6;
		ControlPoint0[Nb]=ControlPoint[Nb];
		Nb+=1;
		for(int j=0;j<_surfCPNum;j++)
		{
			fscanf(fp,"%lf %lf %lf\n",&_surfCP[i*_surfCPNum+j][0],&_surfCP[i*_surfCPNum+j][1],&_surfCP[i*_surfCPNum+j][2]);
			ControlPoint[Nb]=_surfCP[i*_surfCPNum+j];
			ControlPoint[Nb][0]*=0.6;
			ControlPoint[Nb][1]*=0.6;
			ControlPoint0[Nb]=ControlPoint[Nb];
			Nb+=1;
		}
	}


	fclose(fp);

	delete [] _axisCP;
	delete [] _surfCP;

	//make lattice


	Nb=0;
	for (int i=0;i<_axisCPNum;i++)
	{
		for (int j=0;j<_surfCPNum;j++)
		{
			Con[Nb][0]=i*(_surfCPNum+1);
			Con[Nb][1]=i*(_surfCPNum+1)+j+1;
			Connectivity(Nb,0)=Con[Nb][0];
			Connectivity(Nb,1)=Con[Nb][1];
			Connectivity(Nb,2)=K1;
			Connectivity(Nb,3)=C1;
			Nb+=1;
			if (j==_surfCPNum-1)
			{
				Con[Nb][0]=i*(_surfCPNum+1)+j+1;
				Con[Nb][1]=i*(_surfCPNum+1)+1;
				Connectivity(Nb,0)=Con[Nb][0];
				Connectivity(Nb,1)=Con[Nb][1];
				Connectivity(Nb,2)=K1;
				Connectivity(Nb,3)=C1;
				Nb+=1;

			}else{
				Con[Nb][0]=i*(_surfCPNum+1)+j+1;
				Con[Nb][1]=i*(_surfCPNum+1)+j+2;
				Connectivity(Nb,0)=Con[Nb][0];
				Connectivity(Nb,1)=Con[Nb][1];
				Connectivity(Nb,2)=K1;
				Connectivity(Nb,3)=C1;
				Nb+=1;
			}

			if (i!=_axisCPNum-1)
			{
				Con[Nb][0]=i*(_surfCPNum+1)+j+1;
				Con[Nb][1]=(i+1)*(_surfCPNum+1)+j+1;
				Connectivity(Nb,0)=Con[Nb][0];
				Connectivity(Nb,1)=Con[Nb][1];
				Connectivity(Nb,2)=K1;
				Connectivity(Nb,3)=C1;
				Nb+=1;
			}
		}
		if (i!=_axisCPNum-1)
		{
			Con[Nb][0]=i*(_surfCPNum+1);
			Con[Nb][1]=(i+1)*(_surfCPNum+1);
			Connectivity(Nb,0)=Con[Nb][0];
			Connectivity(Nb,1)=Con[Nb][1];
			Connectivity(Nb,2)=K1;
			Connectivity(Nb,3)=C1;
			Nb+=1;
		}
	}



	//make surface
	NbCenterPoint=NbW*(NbCen-2);
	CenterPointPara=new Vec3d[NbCenterPoint];
	CenterPointParaIdx=new Vec2i[NbCenterPoint];
	Nb=0;
	for(int i=0;i<NbCen-2;i++){
		for(int j=0;j<NbW;j++){
			CenterPointPara[Nb][0]=0;
			CenterPointPara[Nb][1]=0;
			CenterPointPara[Nb][2]=(double)j/NbW;
			CenterPointParaIdx[Nb][0]=i;
			CenterPointParaIdx[Nb][1]=0;
			Nb+=1;
		}
	}
	NbCircularPoint=_surfCPNum*NbV;
	NbSurfacePoint=NbCenterPoint*NbCircularPoint;
	SurfacePointPara=new Vec3d[NbSurfacePoint];
	SurfacePointParaIdx=new Vec2i[NbSurfacePoint];

	Nb=0;
	double var,var2;
	var=randn()*0.005;

	for(int i=0;i<NbCen-2;i++){
		
		for(int j=0;j<NbW;j++){
			
			for(int k=0;k<_surfCPNum;k++){
				var=randn()*0.03;
				for(int l=0;l<NbV;l++)
				{
					SurfacePointPara[Nb][0]=radius-var;
					SurfacePointPara[Nb][1]=(double)l/NbV;
					SurfacePointPara[Nb][2]=(double)j/NbW;
					SurfacePointParaIdx[Nb][0]=i;
					SurfacePointParaIdx[Nb][1]=k;
					Nb+=1;
				}
			}
		}
	}





	CenterPoint=new Vec3d[NbCenterPoint];
	SurfacePoint=new Vec3d[NbSurfacePoint];

	for(int i=0;i<NbCenterPoint;i++){
		CenterPoint[i]=paraToSurfacePoint(CenterPointParaIdx[i][0],CenterPointParaIdx[i][1],CenterPointPara[i][0],CenterPointPara[i][1],CenterPointPara[i][2]);
		for(int j=0;j<NbCircularPoint;j++)
			SurfacePoint[i*NbCircularPoint+j]=paraToSurfacePoint(SurfacePointParaIdx[i*NbCircularPoint+j][0],SurfacePointParaIdx[i*NbCircularPoint+j][1],SurfacePointPara[i*NbCircularPoint+j][0],SurfacePointPara[i*NbCircularPoint+j][1],SurfacePointPara[i*NbCircularPoint+j][2]);
	}

	//generate Face
	NbFace=2*NbCircularPoint*(NbCenterPoint-1);
	Face=new Vec3i[NbFace];
	Nb=0;
	for (int i=0;i<NbCenterPoint-1;i++)
	{
		for (int j=0;j<NbCircularPoint;j++)
		{
			if(j!=NbCircularPoint-1){
				Face[Nb][0]=i*NbCircularPoint+j;
				Face[Nb][1]=(i+1)*NbCircularPoint+j;
				Face[Nb][2]=(i+1)*NbCircularPoint+j+1;
				Nb+=1;
				Face[Nb][0]=i*NbCircularPoint+j;
				Face[Nb][1]=(i+1)*NbCircularPoint+j+1;
				Face[Nb][2]=i*NbCircularPoint+j+1;
				Nb+=1;
			}else{
				Face[Nb][0]=i*NbCircularPoint+NbCircularPoint-1;
				Face[Nb][1]=(i+1)*NbCircularPoint+NbCircularPoint-1;
				Face[Nb][2]=(i+1)*NbCircularPoint;
				Nb+=1;
				Face[Nb][0]=i*NbCircularPoint+NbCircularPoint-1;
				Face[Nb][1]=(i+1)*NbCircularPoint;
				Face[Nb][2]=i*NbCircularPoint;
				Nb+=1;
			}
		}
	}
	FaceIndex=new int[NbCenterPoint];
	for(int i=0;i<NbCenterPoint;i++)
		FaceIndex[i]=NbCircularPoint*2*i;	

	computeAroundPoint();
}


void MyFFD::makeUppergastrointestinalSurfaceV3(char* filename,int NbV,int NbW,double radius)
{
	FILE *fp = fopen(filename,"r");
	int _axisCPNum;
	int	_surfCPNum;
	Vec3d* _axisCP;
	Vec3d* _surfCP;

	fscanf(fp,"%d %d\n",&_axisCPNum,&_surfCPNum);
	int NbCen=_axisCPNum;

	int nbPoint=_axisCPNum*(_surfCPNum+1);
	int nbCon=12*_axisCPNum+7*(_axisCPNum-1);
	NbPoint=nbPoint;
	NbCon=nbCon;

	double K1,K2,K3;
	double C1,C2,C3;

	//Parameter1
	K1=500;
	K2=8000;
	K3=600;

	C1=K1*0.1;
	C2=K2*0.1;
	C3=K3*0.1;


	_axisCP=new Vec3d[_axisCPNum];
	_surfCP=new Vec3d[NbPoint];
	ControlPoint=new Vec3d[NbPoint]; 
	PreControlPoint=new Vec3d[NbPoint];
	ControlPoint0=new Vec3d[NbPoint];
	Force=new Vec3d[NbPoint];
	Velocity=new Vec3d[NbPoint];
	Con=new Vec2i[NbCon];

	Connectivity.set_size(NbCon,4);

	int Nb=0;


	for(int i=0;i<_axisCPNum;i++)
	{
		fscanf(fp,"%lf %lf %lf\n",&_axisCP[i][0],&_axisCP[i][1],&_axisCP[i][2]);
		ControlPoint[Nb]=_axisCP[i];
		ControlPoint0[Nb]=ControlPoint[Nb];
		Nb+=1;
		for(int j=0;j<_surfCPNum;j++)
		{
			fscanf(fp,"%lf %lf %lf\n",&_surfCP[i*_surfCPNum+j][0],&_surfCP[i*_surfCPNum+j][1],&_surfCP[i*_surfCPNum+j][2]);
			ControlPoint[Nb]=_surfCP[i*_surfCPNum+j];
			ControlPoint0[Nb]=ControlPoint[Nb];
			Nb+=1;
		}
	}



	fclose(fp);

	delete [] _axisCP;
	delete [] _surfCP;

	//make lattice


	Nb=0;
	for (int i=0;i<_axisCPNum;i++)
	{
		for (int j=0;j<_surfCPNum;j++)
		{
			Con[Nb][0]=i*(_surfCPNum+1);
			Con[Nb][1]=i*(_surfCPNum+1)+j+1;
			Connectivity(Nb,0)=Con[Nb][0];
			Connectivity(Nb,1)=Con[Nb][1];
			Connectivity(Nb,2)=K1;
			Connectivity(Nb,3)=C1;
			Nb+=1;
			if (j==_surfCPNum-1)
			{
				Con[Nb][0]=i*(_surfCPNum+1)+j+1;
				Con[Nb][1]=i*(_surfCPNum+1)+1;
				Connectivity(Nb,0)=Con[Nb][0];
				Connectivity(Nb,1)=Con[Nb][1];
				Connectivity(Nb,2)=K1;
				Connectivity(Nb,3)=C1;
				Nb+=1;

			}else{
				Con[Nb][0]=i*(_surfCPNum+1)+j+1;
				Con[Nb][1]=i*(_surfCPNum+1)+j+2;
				Connectivity(Nb,0)=Con[Nb][0];
				Connectivity(Nb,1)=Con[Nb][1];
				Connectivity(Nb,2)=K1;
				Connectivity(Nb,3)=C1;
				Nb+=1;
			}

			if (i!=_axisCPNum-1)
			{
				Con[Nb][0]=i*(_surfCPNum+1)+j+1;
				Con[Nb][1]=(i+1)*(_surfCPNum+1)+j+1;
				Connectivity(Nb,0)=Con[Nb][0];
				Connectivity(Nb,1)=Con[Nb][1];
				Connectivity(Nb,2)=K1;
				Connectivity(Nb,3)=C1;
				Nb+=1;
			}
		}
		if (i!=_axisCPNum-1)
		{
			Con[Nb][0]=i*(_surfCPNum+1);
			Con[Nb][1]=(i+1)*(_surfCPNum+1);
			Connectivity(Nb,0)=Con[Nb][0];
			Connectivity(Nb,1)=Con[Nb][1];
			Connectivity(Nb,2)=K1;
			Connectivity(Nb,3)=C1;
			Nb+=1;
		}
	}



	//make surface
	NbCenterPoint=NbW*(NbCen-2);
	CenterPointPara=new Vec3d[NbCenterPoint];
	CenterPointParaIdx=new Vec2i[NbCenterPoint];
	Nb=0;
	for(int i=0;i<NbCen-2;i++){
		for(int j=0;j<NbW;j++){
			CenterPointPara[Nb][0]=0;
			CenterPointPara[Nb][1]=0;
			CenterPointPara[Nb][2]=(double)j/NbW;
			CenterPointParaIdx[Nb][0]=i;
			CenterPointParaIdx[Nb][1]=0;
			Nb+=1;
		}
	}
	NbCircularPoint=_surfCPNum*NbV;
	NbSurfacePoint=NbCenterPoint*NbCircularPoint;
	SurfacePointPara=new Vec3d[NbSurfacePoint];
	SurfacePointParaIdx=new Vec2i[NbSurfacePoint];

	Nb=0;
	double var,var2;
	var=randn()*0.005;

	for(int i=0;i<NbCen-2;i++){

		for(int j=0;j<NbW;j++){

			for(int k=0;k<_surfCPNum;k++){
				
				for(int l=0;l<NbV;l++)
				{
					var=randn()*0.04;
					SurfacePointPara[Nb][0]=radius-var;
					SurfacePointPara[Nb][1]=(double)l/NbV;
					SurfacePointPara[Nb][2]=(double)j/NbW;
					SurfacePointParaIdx[Nb][0]=i;
					SurfacePointParaIdx[Nb][1]=k;
					Nb+=1;
				}
			}
		}
	}

	CenterPoint=new Vec3d[NbCenterPoint];
	SurfacePoint=new Vec3d[NbSurfacePoint];

	for(int i=0;i<NbCenterPoint;i++){
		CenterPoint[i]=paraToSurfacePoint(CenterPointParaIdx[i][0],CenterPointParaIdx[i][1],CenterPointPara[i][0],CenterPointPara[i][1],CenterPointPara[i][2]);
		for(int j=0;j<NbCircularPoint;j++)
			SurfacePoint[i*NbCircularPoint+j]=paraToSurfacePoint(SurfacePointParaIdx[i*NbCircularPoint+j][0],SurfacePointParaIdx[i*NbCircularPoint+j][1],SurfacePointPara[i*NbCircularPoint+j][0],SurfacePointPara[i*NbCircularPoint+j][1],SurfacePointPara[i*NbCircularPoint+j][2]);
	}

	//generate Face
	NbFace=2*NbCircularPoint*(NbCenterPoint-1);
	Face=new Vec3i[NbFace];
	Nb=0;
	for (int i=0;i<NbCenterPoint-1;i++)
	{
		for (int j=0;j<NbCircularPoint;j++)
		{
			if(j!=NbCircularPoint-1){
				Face[Nb][0]=i*NbCircularPoint+j;
				Face[Nb][1]=(i+1)*NbCircularPoint+j;
				Face[Nb][2]=(i+1)*NbCircularPoint+j+1;
				Nb+=1;
				Face[Nb][0]=i*NbCircularPoint+j;
				Face[Nb][1]=(i+1)*NbCircularPoint+j+1;
				Face[Nb][2]=i*NbCircularPoint+j+1;
				Nb+=1;
			}else{
				Face[Nb][0]=i*NbCircularPoint+NbCircularPoint-1;
				Face[Nb][1]=(i+1)*NbCircularPoint+NbCircularPoint-1;
				Face[Nb][2]=(i+1)*NbCircularPoint;
				Nb+=1;
				Face[Nb][0]=i*NbCircularPoint+NbCircularPoint-1;
				Face[Nb][1]=(i+1)*NbCircularPoint;
				Face[Nb][2]=i*NbCircularPoint;
				Nb+=1;
			}
		}
	}
	FaceIndex=new int[NbCenterPoint];
	for(int i=0;i<NbCenterPoint;i++)
		FaceIndex[i]=NbCircularPoint*2*i;	

	computeAroundPoint();
}




void MyFFD::InsertEndoscope(double dv)
{
	int InsertedIndex=0;
	for(int i=0;i<NbPoint;i++){
		ControlPoint0[i]+=InsertedDirec*dv;
	}

	for(int i=0;i<NbPoint;i++)
		if((ControlPoint[i]-InsertedPoint)*InsertedDirec>=0)
			InsertedIndex=i;

	if(InsertedIndex==NbPoint-1)
	{
		ControlPoint[NbPoint-1]+=InsertedDirec*dv;
		ControlPoint[NbPoint-2]+=InsertedDirec*dv;
	}else{
		for(int i=InsertedIndex;i<NbPoint;i++){
			ControlPoint[i]+=InsertedDirec*dv;
		}	
	}

	for (unsigned int i=0;i<FixedConstraint.size();i++)
		for (int j=0;j<3;j++)
			ControlPoint[FixedConstraint[i]]+=InsertedDirec*dv;
	
}

void MyFFD::ControlEndsocope(double dx,double dy)
{
	Vec3d Subdirec(1,0,0);
	Vec3d TorqueDirec1=ControlPoint[0]-ControlPoint[1];
	Vec3d TorqueDirec2;

	TorqueDirec1=TorqueDirec1.cross(Subdirec);
	TorqueDirec2=TorqueDirec1.cross(ControlPoint[0]-ControlPoint[1]);

	TorqueDirec1.normalize();
	TorqueDirec2.normalize();

	double Torque1=Kb*sin(dx*PI/180);
	double Torque2=Kb*sin(dy*PI/180);

	for(int i=1;i<5;i++)
	{
		addTorqueToPoint(TorqueDirec1,Torque1*(1+0.5*i),i);
		addTorqueToPoint(TorqueDirec2,Torque1*(1+0.5*i),i);
	}
	

}

void MyFFD::addTorqueToPoint(Vec3d direc,double Torque,int Idx)
{
	direc.normalize();
	Vec3d p1,p2,p3;
	Vec3d Subdirec1,Subdirec2;

	p1=ControlPoint[Idx-1];
	p2=ControlPoint[Idx];
	p3=ControlPoint[Idx+1];

	Subdirec1=direc.cross(p1-p2);
	Subdirec2=direc.cross(p2-p3);
	
	Subdirec1.normalize();
	Subdirec2.normalize();

	Force[Idx-1]+=Subdirec1*Torque/(p1-p2).norm();
	Force[Idx]-=Subdirec2*Torque/(p2-p3).norm()+Subdirec1*Torque/(p1-p2).norm();
	Force[Idx+1]+=Subdirec2*Torque/(p2-p3).norm();
}


void MyFFD::ControlEndsocopeByDisplacement(double Angle1,double Angle2)
{
	Vec3d Subdirec(1,0,0);
	Vec3d Direc1=ControlPoint[0]-ControlPoint[1];
	Vec3d Direc2;
	Vec3d p1,p2;
	Vec3d SinDirec,CosDirec;

	EndoscopeTipDisplacement.fill(0.0);
	

	Direc1=Direc1.cross(Subdirec);
	Direc2=Direc1.cross(ControlPoint[0]-ControlPoint[1]);

	Direc1.normalize();
	Direc2.normalize();

	Angle1*=PI/180;
	Angle2*=PI/180;

	Vec3d Dis;
	double l;
	Dis.clear();
	for(int i=0;i<1;i++)
	{
		p1=ControlPoint[i];
		p2=ControlPoint[i+1];
		l=(p1-p2).norm();
		CosDirec=p2-p1;
		CosDirec.normalize();
		SinDirec=Direc1.cross(p1-p2);
		SinDirec.normalize();

		Dis+=SinDirec*sin(Angle1*(i+1))*l+CosDirec*(1-cos(Angle1*(i+1)))*l;

		SinDirec=Direc2.cross(p1-p2);
		SinDirec.normalize();

		Dis+=SinDirec*sin(Angle2*(i+1))*l+CosDirec*(1-cos(Angle2*(i+1)))*l;


		for(int j=0;j<3;j++)
			EndoscopeTipDisplacement(3*i+j)=Dis[j];
	}
	

}

void MyFFD::HoldEndoscope()
{
	for(int i=0;i<NbPoint;i++)
		ControlPoint0[i]=ControlPoint[i];

	for(int i=0;i<NbPoint;i++)
		addFixedConstraint(i);
}

void MyFFD::ReleaseEndoscope()
{
	FixedConstraint.clear();
}


void MyFFD::updateEndoscopeImplicit(double dt,Vec3d Gravity)
{
	//////////////////////////
	/* Implicit integration */
	//////////////////////////
	int InsertedIndex=0;

	/* add gravity */
	for(int i=0;i<NbPoint;i++)
		Force[i]+=Gravity;

	makeEndoscopeStiffness(false);
	makeDamping();

	/* add internal force */
	for(int i=0;i<NbPoint;i++)
		for(int j=0;j<3;j++)
			for(int k=0;k<NbPoint;k++)
				for(int l=0;l<3;l++)
					Force[i][j]+=(Mglobal[3*i+j][3*k+l]/(dt*dt)+Cglobal[3*i+j][3*k+l]/dt)*(ControlPoint[k][l]-ControlPoint0[k][l])+(Mglobal[3*i+j][3*k+l]/dt)*Velocity[k][l]-Kglobal[3*i+j][3*k+l]*ControlPoint0[k][l];


	/* add environment damping force */
	for(int i=0;i<NbPoint;i++)
		Force[i]-=Velocity[i]*Ed;


	for(int i=0;i<NbPoint;i++)
		for(int j=0;j<3;j++)
			x[3*i+j]=Force[i][j];


	for (int i=0;i<3*NbPoint;i++)
		for (int j=0;j<3*NbPoint;j++)
			Kglobal[i][j]+=Mglobal[i][j]/(dt*dt)+Cglobal[i][j]/dt;

	double penalty=30000;
	//Constraints
	for(int i=0;i<NbPoint;i++)
		if((ControlPoint[i]-InsertedPoint)*InsertedDirec>=0)
			InsertedIndex=i;

	if(InsertedIndex==NbPoint-1)
	{
		for (int j=0;j<3;j++){
			Kglobal[3*(NbPoint-2)+j][3*(NbPoint-2)+j]+=penalty;
			Kglobal[3*(NbPoint-1)+j][3*(NbPoint-1)+j]+=penalty;
		}
	}else{
		for(int i=InsertedIndex;i<NbPoint;i++)
			for (int j=0;j<3;j++)
				Kglobal[3*(i)+j][3*(i)+j]+=penalty;	
	}
	for (int i=0;i<FixedConstraint.size();i++)
		for (int j=0;j<3;j++)
			Kglobal[3*FixedConstraint[i]+j][3*FixedConstraint[i]+j]+=penalty;

	for(int i=0;i<NbPoint;i++)
		PreControlPoint[i]=ControlPoint[i];

	//Position update
	if(GSolve(Kglobal,3*NbPoint,x))
		for(int i=0;i<NbPoint;i++)
			for(int j=0;j<3;j++)
				ControlPoint[i][j]=ControlPoint0[i][j]+x[3*i+j];

	//Velocity update
	for(int i=0;i<NbPoint;i++)
		Velocity[i]=(ControlPoint[i]-PreControlPoint[i])/dt;

	/* reset force */
	for(int i=0;i<NbPoint;i++)
		Force[i].clear();
}

void MyFFD::updateEndoscopeFastImplicit(double dt,Vec3d Gravity)
{
	//////////////////////////
	/* Implicit integration */
	//////////////////////////
	int InsertedIndex=0;

	/* add gravity */
	for(int i=0;i<NbPoint;i++)
		Force[i]+=Gravity;

	makeEndoscopeStiffnessMatrix(false);
	makeEndoscopeDampingMatrix();


	
	mat Pos0,PrePos,Pos,Vel,force;
	Pos0.set_size(3*NbPoint); Pos.set_size(3*NbPoint); PrePos.set_size(3*NbPoint); Vel.set_size(3*NbPoint); force.set_size(3*NbPoint);

	for(int i=0;i<NbPoint;i++){
		for(int j=0;j<3;j++){
			Pos0(3*i+j)=ControlPoint0[i][j];
			Pos(3*i+j)=ControlPoint[i][j];
			Vel(3*i+j)=Velocity[i][j];
			force(3*i+j)=Force[i][j];
		}
	}
	force+=(MassMatrix/(dt*dt)+DampingMatrix/dt)*(Pos-Pos0)+(MassMatrix/dt)*Vel-StiffnessMatrix*Pos0;

	/* add environment damping force */
	force-=Ed*Vel;

	// Position update
	PrePos=Pos;

	// Solve

	StiffnessMatrix+=MassMatrix/(dt*dt)+DampingMatrix/dt;


	double penalty=30000;
	//Constraints
	for(int i=0;i<NbPoint;i++)
		if((ControlPoint[i]-InsertedPoint)*InsertedDirec>=0)
			InsertedIndex=i;

	if(InsertedIndex==NbPoint-1)
	{
		for (int j=0;j<3;j++){
			StiffnessMatrix(3*(NbPoint-2)+j,3*(NbPoint-2)+j)+=penalty;
			StiffnessMatrix(3*(NbPoint-1)+j,3*(NbPoint-1)+j)+=penalty;
		}
	}else{
		for(int i=InsertedIndex;i<NbPoint;i++)
			for (int j=0;j<3;j++)
				StiffnessMatrix(3*(i)+j,3*(i)+j)+=penalty;	
	}
	for (int i=0;i<FixedConstraint.size();i++)
		for (int j=0;j<3;j++)
			StiffnessMatrix(3*FixedConstraint[i]+j,3*FixedConstraint[i]+j)+=penalty;

	Displacement=solve(StiffnessMatrix,force);

	//Position update

	Pos=Pos0+Displacement;

	Vel=(Pos-PrePos)/dt;

	//Update
	for(int i=0;i<NbPoint;i++){
		for(int j=0;j<3;j++){
			ControlPoint[i][j]=Pos(3*i+j);
			Velocity[i][j]=Vel(3*i+j);
		}
	}
	for(int i=0;i<NbPoint;i++)
		Force[i].clear();
}

void MyFFD::updateEndoscopeExplicit(double dt,Vec3d Gravity)
{
	//////////////////////////
	/* Implicit integration */
	//////////////////////////

	int InsertedIndex=0;

	/* add gravity */
	for(int i=0;i<NbPoint;i++)
		Force[i]+=Gravity;

	/* add internal force */
	m_Spring.addForce(Force,ControlPoint,Velocity);

	/* add Bending force */
	addBendingForce();

	/* add environment damping force */
	for(int i=0;i<NbPoint;i++)
		Force[i]-=Velocity[i]*Ed;

	//////////////////////////
	/* Explicit integration */
	//////////////////////////

	for(int i=0;i<NbPoint;i++)
		if((ControlPoint[i]-InsertedPoint)*InsertedDirec>=0)
			InsertedIndex=i;


	for(int i=0;i<NbPoint;i++)
	{
		Vec3d dv=Force[i]/Mass*dt;
		Velocity[i]+=dv;
		ControlPoint[i]+=(Velocity[i]*dt);
	}


	//Constraints
	

	if(InsertedIndex==NbPoint-1)
	{
		ControlPoint[(NbPoint-2)]=ControlPoint0[(NbPoint-2)];
		ControlPoint[(NbPoint-1)]=ControlPoint0[(NbPoint-1)];
	}else{
		for(int i=InsertedIndex;i<NbPoint;i++)
			ControlPoint[i]=ControlPoint0[i];	
	}
	/*for (int i=0;i<FixedConstraint.size();i++)
		ControlPoint[FixedConstraint[i]]=ControlPoint0[FixedConstraint[i]];*/


	for(int i=0;i<NbPoint;i++)
		Force[i].clear();
}


void MyFFD::makeEndoscopeStiffness(bool Constraint)
{
	matrixClear(3*NbPoint,Kglobal);
	Vec3d p1,p2,p10,p20;
	double l,l0;
	int InsertedIndex=0;
	int N[6];

	for(int i=0;i<NbCon;i++)
	{
		N[0]=3*Con[i][0];
		N[1]=3*Con[i][0]+1;
		N[2]=3*Con[i][0]+2;
		N[3]=3*Con[i][1];
		N[4]=3*Con[i][1]+1;
		N[5]=3*Con[i][1]+2;

		p1=ControlPoint[Con[i][0]];
		p2=ControlPoint[Con[i][1]];
		p10=ControlPoint0[Con[i][0]];
		p20=ControlPoint0[Con[i][1]];
		l=(p1-p2).norm();
		l0=(p10-p20).norm();
		makeLocalstiffness(p1,p2,K*(1-l0/l));
		for (int j=0;j<6;j++)
			for (int k=0;k<6;k++)
				Kglobal[N[j]][N[k]]+=local[j][k];
	}

	makeBendingStiffness();

	if(Constraint)
	{
		double penalty=30000;
		//Constraints
		for(int i=0;i<NbPoint;i++)
			if((ControlPoint[i]-InsertedPoint)*InsertedDirec>=0)
				InsertedIndex=i;
		
		if(InsertedIndex==NbPoint-1)
		{
			for (int j=0;j<3;j++){
				Kglobal[3*(NbPoint-2)+j][3*(NbPoint-2)+j]+=penalty;
				Kglobal[3*(NbPoint-1)+j][3*(NbPoint-1)+j]+=penalty;
			}
		}else{
			for(int i=InsertedIndex;i<NbPoint;i++)
				for (int j=0;j<3;j++)
					Kglobal[3*(i)+j][3*(i)+j]+=penalty;	
		}
		for (int i=0;i<FixedConstraint.size();i++)
			for (int j=0;j<3;j++)
				Kglobal[3*FixedConstraint[i]+j][3*FixedConstraint[i]+j]+=penalty;
	}

}


void MyFFD::makeEndoscopeStiffnessMatrix(bool Constraint)
{
	StiffnessMatrix.fill(0.0);
	Vec3d p1,p2,p10,p20;
	double l,l0;
	int InsertedIndex=0;
	int N[6];

	for(int i=0;i<NbCon;i++)
	{
		N[0]=3*Con[i][0];
		N[1]=3*Con[i][0]+1;
		N[2]=3*Con[i][0]+2;
		N[3]=3*Con[i][1];
		N[4]=3*Con[i][1]+1;
		N[5]=3*Con[i][1]+2;

		p1=ControlPoint[Con[i][0]];
		p2=ControlPoint[Con[i][1]];
		p10=ControlPoint0[Con[i][0]];
		p20=ControlPoint0[Con[i][1]];
		l=(p1-p2).norm();
		l0=(p10-p20).norm();
		makeLocalStiffnessMatrix(p1,p2,K*(1-l0/l));
		for (int j=0;j<6;j++)
			for (int k=0;k<6;k++)
				StiffnessMatrix(N[j],N[k])+=Local(j,k);
	}

	makeBendingStiffnessMatrix();

	if(Constraint)
	{
		double penalty=30000;
		//Constraints
		for(int i=0;i<NbPoint;i++)
			if((ControlPoint[i]-InsertedPoint)*InsertedDirec>=0)
				InsertedIndex=i;

		if(InsertedIndex==NbPoint-1)
		{
			for (int j=0;j<3;j++){
				StiffnessMatrix(3*(NbPoint-2)+j,3*(NbPoint-2)+j)+=penalty;
				StiffnessMatrix(3*(NbPoint-1)+j,3*(NbPoint-1)+j)+=penalty;
			}
		}else{
			for(int i=InsertedIndex;i<NbPoint;i++)
				for (int j=0;j<3;j++)
					StiffnessMatrix(3*(i)+j,3*(i)+j)+=penalty;	
		}
		for (int i=0;i<FixedConstraint.size();i++)
			for (int j=0;j<3;j++)
				StiffnessMatrix(3*FixedConstraint[i]+j,3*FixedConstraint[i]+j)+=penalty;
	}

}

void MyFFD::makeEndoscopeDamping()
{
	matrixClear(3*NbPoint,Cglobal);
	Vec3d p1,p2;

	int N[6];

	for(int i=0;i<NbCon;i++)
	{
		N[0]=3*Con[i][0];
		N[1]=3*Con[i][0]+1;
		N[2]=3*Con[i][0]+2;
		N[3]=3*Con[i][1];
		N[4]=3*Con[i][1]+1;
		N[5]=3*Con[i][1]+2;

		p1=ControlPoint[Con[i][0]];
		p2=ControlPoint[Con[i][1]];
		makeLocalstiffness(p1,p2,C);
		for (int j=0;j<6;j++)
			for (int k=0;k<6;k++)
				Cglobal[N[j]][N[k]]+=local[j][k];
	}
	makeBendingDamping();
}


void MyFFD::makeEndoscopeDampingMatrix()
{
	DampingMatrix.fill(0.0);
	Vec3d p1,p2;

	int N[6];

	for(int i=0;i<NbCon;i++)
	{
		N[0]=3*Con[i][0];
		N[1]=3*Con[i][0]+1;
		N[2]=3*Con[i][0]+2;
		N[3]=3*Con[i][1];
		N[4]=3*Con[i][1]+1;
		N[5]=3*Con[i][1]+2;

		p1=ControlPoint[Con[i][0]];
		p2=ControlPoint[Con[i][1]];
		makeLocalStiffnessMatrix(p1,p2,C);
		for (int j=0;j<6;j++)
			for (int k=0;k<6;k++)
				DampingMatrix(N[j],N[k])+=Local(j,k);
	}
	makeBendingDampingMatrix();
}

void MyFFD::makeEndoscopeCompliance(bool Constraint)
{
	makeEndoscopeStiffness(Constraint);
	makeEndoscopeDamping();

	for (int i=0;i<3*NbPoint;i++)
		for (int j=0;j<3*NbPoint;j++)
			Compliance[i][j]=Mglobal[i][j]/(dt*dt)+Cglobal[i][j]/dt+Kglobal[i][j];

}

void MyFFD::makeEndoscopeCompliance2()
{
	for (int i=0;i<3*NbPoint;i++)
		for (int j=0;j<3*NbPoint;j++)
			Compliance[i][j]=Mglobal[i][j]/(dt*dt)+Cglobal[i][j]/dt+Kglobal[i][j];

}

int MyFFD::findCloestCenterPointIndex(Vec3d Pos)
{
	double min=1000000000;
	int index=0;
	for(int i=0;i<NbPoint/7;i++)
	{
		if((ControlPoint[7*i]-Pos).norm()<min)
		{
			min=(ControlPoint[7*i]-Pos).norm();
			index=i;
		}
	}
	return index;

}

int MyFFD::findCloestCircularPointIndex(Vec3d Pos)
{
	double min=1000000000;
	int index=0;
	int index2=0;
	
	index=findCloestCenterPointIndex(Pos);
	min=1000000000;
	for(int i=0;i<6;i++)
	{
		if((ControlPoint[7*index+i+1]-Pos).norm()<min)
		{
			min=(ControlPoint[7*index+i+1]-Pos).norm();
			index2=i;
		}
	}
	return 7*index+index2+1;

}

mat MyFFD::returnInternalImplicitForceOfEndoscope()
{

	makeEndoscopeStiffnessMatrix(false);
	makeEndoscopeDampingMatrix();



	mat Pos0,PrePos,Pos,Vel,force;
	Pos0.set_size(3*NbPoint); Pos.set_size(3*NbPoint); PrePos.set_size(3*NbPoint); Vel.set_size(3*NbPoint); force.set_size(3*NbPoint);

	for(int i=0;i<NbPoint;i++){
		for(int j=0;j<3;j++){
			Pos0(3*i+j)=ControlPoint0[i][j];
			Pos(3*i+j)=ControlPoint[i][j];
			Vel(3*i+j)=Velocity[i][j];
			force(3*i+j)=Force[i][j];
		}
	}

	for(int i=0;i<NbPoint;i++)
		Force[i].clear();

	force+=(MassMatrix/(dt*dt)+DampingMatrix/dt)*(Pos-Pos0)+(MassMatrix/dt)*Vel-StiffnessMatrix*Pos0;

	/* add environment damping force */
	force-=Ed*Vel;

	return force;
}

mat MyFFD::returnInternalExplicitForceOfEndoscope()
{

	makeEndoscopeStiffnessMatrix(false);
	makeEndoscopeDampingMatrix();



	mat Pos0,Pos,Vel,force;
	Pos0.set_size(3*NbPoint); Pos.set_size(3*NbPoint); Vel.set_size(3*NbPoint); force.set_size(3*NbPoint);


	for(int i=0;i<NbPoint;i++){
		for(int j=0;j<3;j++){
			Pos0(3*i+j)=ControlPoint0[i][j];
			Pos(3*i+j)=ControlPoint[i][j];
			Vel(3*i+j)=Velocity[i][j];
			force(3*i+j)=Force[i][j];
		}
	}
	force-=Ed*Vel;
	force+=-(MassMatrix/(dt*dt)-DampingMatrix/dt+StiffnessMatrix)*(PrePos-Pos0)-(-2*MassMatrix/(dt*dt)+DampingMatrix/dt)*(Pos-Pos0)-StiffnessMatrix*Pos0;
	
	PrePos=Pos;


	for(int i=0;i<NbPoint;i++)
		Force[i].clear();

	return force;
}

mat MyFFD::returnInternalImplicitForceOfFFD()
{

	makeStiffnessMatrixWithoutPenalty();
	makeDampingMatrix();

	/* add internal force */

	mat Pos0,PrePos,Pos,Vel,force;
	Pos0.set_size(3*NbPoint); Pos.set_size(3*NbPoint); PrePos.set_size(3*NbPoint); Vel.set_size(3*NbPoint); force.set_size(3*NbPoint);

	for(int i=0;i<NbPoint;i++){
		for(int j=0;j<3;j++){
			Pos0(3*i+j)=ControlPoint0[i][j];
			Pos(3*i+j)=ControlPoint[i][j];
			Vel(3*i+j)=Velocity[i][j];
			force(3*i+j)=Force[i][j];
		}
	}
	force+=(MassMatrix/(dt*dt)+DampingMatrix/dt)*(Pos-Pos0)+(MassMatrix/dt)*Vel-StiffnessMatrix*Pos0;

	/* add environment damping force */
	force-=Ed*Vel;

	return force;

}

mat MyFFD::returnInternalExplicitForceOfFFD()
{

	makeStiffnessMatrixWithoutPenalty();
	makeDampingMatrix();

	/* add internal force */

	mat Pos0,PreDis,Pos,Vel,force;
	Pos0.set_size(3*NbPoint); Pos.set_size(3*NbPoint); PreDis.set_size(3*NbPoint); Vel.set_size(3*NbPoint); force.set_size(3*NbPoint);


	for(int i=0;i<NbPoint;i++){
		for(int j=0;j<3;j++){
			Pos0(3*i+j)=ControlPoint0[i][j];
			Pos(3*i+j)=ControlPoint[i][j];
			Vel(3*i+j)=Velocity[i][j];
			force(3*i+j)=Force[i][j];
		}
	}
	force-=Ed*Vel;
	force+=-(MassMatrix/(dt*dt)-DampingMatrix/dt+StiffnessMatrix)*(PrePos-Pos0)-(-2*MassMatrix/(dt*dt)+DampingMatrix/dt)*(Pos-Pos0)-StiffnessMatrix*Pos0;
	PrePos=Pos;
	return force;

	for(int i=0;i<NbPoint;i++)
		Force[i].clear();

}

mat MyFFD::returnInternalImplicitForceOfUpper()
{

	makeStiffnessMatrixForUpper(false);
	makeDampingMatrixForUpper();

	/* add internal force */

	mat Pos0,PrePos,Pos,Vel,force;
	Pos0.set_size(3*NbPoint); Pos.set_size(3*NbPoint); PrePos.set_size(3*NbPoint); Vel.set_size(3*NbPoint); force.set_size(3*NbPoint);

	for(int i=0;i<NbPoint;i++){
		for(int j=0;j<3;j++){
			Pos0(3*i+j)=ControlPoint0[i][j];
			Pos(3*i+j)=ControlPoint[i][j];
			Vel(3*i+j)=Velocity[i][j];
			force(3*i+j)=Force[i][j];
		}
	}
	force+=(MassMatrix/(dt*dt)+DampingMatrix/dt)*(Pos-Pos0)+(MassMatrix/dt)*Vel-StiffnessMatrix*Pos0;

	/* add environment damping force */
	force-=Ed*Vel;

	return force;

	for(int i=0;i<NbPoint;i++)
		Force[i].clear();

}

mat MyFFD::returnInternalExplicitForceOfUpper()
{

	makeStiffnessMatrixForUpper(false);
	makeDampingMatrixForUpper();

	/* add internal force */

	mat Pos0,PreDis,Pos,Vel,force;
	Pos0.set_size(3*NbPoint); Pos.set_size(3*NbPoint); PreDis.set_size(3*NbPoint); Vel.set_size(3*NbPoint); force.set_size(3*NbPoint);

	applyElasticConstraintForce();

	for(int i=0;i<NbPoint;i++){
		for(int j=0;j<3;j++){
			Pos0(3*i+j)=ControlPoint0[i][j];
			Pos(3*i+j)=ControlPoint[i][j];
			Vel(3*i+j)=Velocity[i][j];
			force(3*i+j)=Force[i][j];
		}
	}
	force-=Ed*Vel;
	force+=-(MassMatrix/(dt*dt)-DampingMatrix/dt+StiffnessMatrix)*(PrePos-Pos0)-(-2*MassMatrix/(dt*dt)+DampingMatrix/dt)*(Pos-Pos0)-StiffnessMatrix*Pos0;
	PrePos=Pos;
	return force;

	for(int i=0;i<NbPoint;i++)
		Force[i].clear();
}

mat MyFFD::returnInternalImplicitDisplacementOfFFD(Vec3d Gravity)
{

	makeStiffnessMatrixWithoutPenalty();
	makeDampingMatrix();

	/* add internal force */

	mat Pos0,PreDis,Pos,Vel,force;
	Pos0.set_size(3*NbPoint); Pos.set_size(3*NbPoint); PreDis.set_size(3*NbPoint); Vel.set_size(3*NbPoint); force.set_size(3*NbPoint);

	for(int i=0;i<NbPoint;i++)
		Force[i]+=Gravity;


	for(int i=0;i<NbPoint;i++){
		for(int j=0;j<3;j++){
			Pos0(3*i+j)=ControlPoint0[i][j];
			Pos(3*i+j)=ControlPoint[i][j];
			Vel(3*i+j)=Velocity[i][j];
			force(3*i+j)=Force[i][j];
		}
	}

	PreDis=Pos-Pos0;

	force+=(MassMatrix/(dt*dt)+DampingMatrix/dt)*(Pos-Pos0)+(MassMatrix/dt)*Vel-StiffnessMatrix*Pos0;

	/* add environment damping force */
	force-=Ed*Vel;

	for(int i=0;i<NbPoint;i++)
		Force[i].clear();

	double penalty=30000;
	//Constraints
	for (int i=0;i<FixedConstraint.size();i++)
		for (int j=0;j<3;j++)
			StiffnessMatrix(3*FixedConstraint[i]+j,3*FixedConstraint[i]+j)+=penalty;

	mat Displacement=solve(MassMatrix/(dt*dt)+DampingMatrix/dt+StiffnessMatrix,force);
	Displacement-=PreDis;

	return Displacement;
}


mat MyFFD::returnInternalImplicitDisplacementOfEndoscope(Vec3d Gravity)
{

	makeEndoscopeStiffnessMatrix(false);
	makeEndoscopeDampingMatrix();

	/* add internal force */

	mat Pos0,PreDis,Pos,Vel,force;
	Pos0.set_size(3*NbPoint); Pos.set_size(3*NbPoint); PreDis.set_size(3*NbPoint); Vel.set_size(3*NbPoint); force.set_size(3*NbPoint);

	for(int i=0;i<NbPoint;i++)
		Force[i]+=Gravity;


	for(int i=0;i<NbPoint;i++){
		for(int j=0;j<3;j++){
			Pos0(3*i+j)=ControlPoint0[i][j];
			Pos(3*i+j)=ControlPoint[i][j];
			Vel(3*i+j)=Velocity[i][j];
			force(3*i+j)=Force[i][j];
		}
	}

	PreDis=Pos-Pos0;

	force+=(MassMatrix/(dt*dt)+DampingMatrix/dt)*(Pos-Pos0)+(MassMatrix/dt)*Vel-StiffnessMatrix*Pos0;

	/* add environment damping force */
	force-=Ed*Vel;

	for(int i=0;i<NbPoint;i++)
		Force[i].clear();
	
	int InsertedIndex=0;
	double penalty=30000;
	//Constraints
	for(int i=0;i<NbPoint;i++)
		if((ControlPoint[i]-InsertedPoint)*InsertedDirec>=0)
			InsertedIndex=i;

	if(InsertedIndex==NbPoint-1)
	{
		for (int j=0;j<3;j++){
			StiffnessMatrix(3*(NbPoint-2)+j,3*(NbPoint-2)+j)+=penalty;
			StiffnessMatrix(3*(NbPoint-1)+j,3*(NbPoint-1)+j)+=penalty;
		}
	}else{
		for(int i=InsertedIndex;i<NbPoint;i++)
			for (int j=0;j<3;j++)
				StiffnessMatrix(3*(i)+j,3*(i)+j)+=penalty;	
	}
	for (int i=0;i<FixedConstraint.size();i++)
		for (int j=0;j<3;j++)
			StiffnessMatrix(3*FixedConstraint[i]+j,3*FixedConstraint[i]+j)+=penalty;

	mat Displacement=solve(MassMatrix/(dt*dt)+DampingMatrix/dt+StiffnessMatrix,force);
	Displacement-=PreDis;

	return Displacement;
}

mat MyFFD::returnInternalExplicitDisplacementOfFFD(Vec3d Gravity)
{

	makeStiffnessMatrixWithoutPenalty();
	makeDampingMatrix();

	/* add internal force */

	mat Pos0,PreDis,Pos,Vel,force;
	Pos0.set_size(3*NbPoint); Pos.set_size(3*NbPoint); PreDis.set_size(3*NbPoint); Vel.set_size(3*NbPoint); force.set_size(3*NbPoint);

	for(int i=0;i<NbPoint;i++)
		Force[i]+=Gravity;

	/* add environment damping force */
	for(int i=0;i<NbPoint;i++)
		Force[i]-=Velocity[i]*Ed;



	for(int i=0;i<NbPoint;i++){
		for(int j=0;j<3;j++){
			Pos0(3*i+j)=ControlPoint0[i][j];
			Pos(3*i+j)=ControlPoint[i][j];
			Vel(3*i+j)=Velocity[i][j];
			force(3*i+j)=Force[i][j];
		}
	}
	PreDis=Pos-Pos0;
	force+=-(MassMatrix/(dt*dt)-DampingMatrix/dt+StiffnessMatrix)*(PrePos-Pos0)-(-2*MassMatrix/(dt*dt)+DampingMatrix/dt)*(Pos-Pos0)-StiffnessMatrix*Pos0;
	
	PrePos=Pos;

	double penalty=30000;
	mat mass;
	mass.set_size(3*NbPoint);
	mass.fill(M);

	//Constraints
	for (int i=0;i<FixedConstraint.size();i++)
		for (int j=0;j<3;j++)
			mass(3*FixedConstraint[i]+j)+=penalty;

	mass/=(dt*dt);


	mat Displacement=force/mass;
	Displacement-=PreDis;

	return Displacement;

	for(int i=0;i<NbPoint;i++)
		Force[i].clear();
}

mat MyFFD::returnInternalExplicitDisplacementOfUpper(Vec3d Gravity)
{


	makeStiffnessMatrixForUpper(false);
	makeDampingMatrixForUpper();

	/* add internal force */

	mat Pos0,PreDis,Pos,Vel,force;
	Pos0.set_size(3*NbPoint); Pos.set_size(3*NbPoint); PreDis.set_size(3*NbPoint); Vel.set_size(3*NbPoint); force.set_size(3*NbPoint);

	for(int i=0;i<NbPoint;i++)
		Force[i]+=Gravity;


	for(int i=0;i<NbPoint;i++){
		for(int j=0;j<3;j++){
			Pos0(3*i+j)=ControlPoint0[i][j];
			Pos(3*i+j)=ControlPoint[i][j];
			Vel(3*i+j)=Velocity[i][j];
			force(3*i+j)=Force[i][j];
		}
	}
	force-=Ed*Vel;
	force+=-(MassMatrix/(dt*dt)-DampingMatrix/dt+StiffnessMatrix)*(PrePos-Pos0)-(-2*MassMatrix/(dt*dt)+DampingMatrix/dt)*(Pos-Pos0)-StiffnessMatrix*Pos0;
	PreDis=Pos-Pos0;
	PrePos=Pos;

	double penalty=30000;
	mat mass;
	mass.set_size(3*NbPoint);
	mass.fill(M);

	//Constraints
	for (int i=0;i<FixedConstraint.size();i++)
		for (int j=0;j<3;j++)
			mass(3*FixedConstraint[i]+j)+=penalty;

	mass/=(dt*dt);


	mat Displacement=force/mass;
	Displacement-=PreDis;

	return Displacement;

	for(int i=0;i<NbPoint;i++)
		Force[i].clear();

}



mat MyFFD::returnInternalExplicitDisplacementOfEndoscope(Vec3d Gravity)
{

	makeEndoscopeStiffnessMatrix(false);
	makeEndoscopeDampingMatrix();

	/* add internal force */

	mat Pos0,PreDis,Pos,Vel,force;
	Pos0.set_size(3*NbPoint); Pos.set_size(3*NbPoint); PreDis.set_size(3*NbPoint); Vel.set_size(3*NbPoint); force.set_size(3*NbPoint);

	for(int i=0;i<NbPoint;i++)
		Force[i]+=Gravity;


	for(int i=0;i<NbPoint;i++){
		for(int j=0;j<3;j++){
			Pos0(3*i+j)=ControlPoint0[i][j];
			Pos(3*i+j)=ControlPoint[i][j];
			Vel(3*i+j)=Velocity[i][j];
			force(3*i+j)=Force[i][j];
		}
	}

	PreDis=Pos-Pos0;

	force+=(MassMatrix/(dt*dt)+DampingMatrix/dt)*(Pos-Pos0)+(MassMatrix/dt)*Vel-StiffnessMatrix*Pos0;

	/* add environment damping force */
	force-=Ed*Vel;

	for(int i=0;i<NbPoint;i++)
		Force[i].clear();

	int InsertedIndex=0;
	double penalty=30000;
	//Constraints
	for(int i=0;i<NbPoint;i++)
		if((ControlPoint[i]-InsertedPoint)*InsertedDirec>=0)
			InsertedIndex=i;

	if(InsertedIndex==NbPoint-1)
	{
		for (int j=0;j<3;j++){
			StiffnessMatrix(3*(NbPoint-2)+j,3*(NbPoint-2)+j)+=penalty;
			StiffnessMatrix(3*(NbPoint-1)+j,3*(NbPoint-1)+j)+=penalty;
		}
	}else{
		for(int i=InsertedIndex;i<NbPoint;i++)
			for (int j=0;j<3;j++)
				StiffnessMatrix(3*(i)+j,3*(i)+j)+=penalty;	
	}
	for (int i=0;i<FixedConstraint.size();i++)
		for (int j=0;j<3;j++)
			StiffnessMatrix(3*FixedConstraint[i]+j,3*FixedConstraint[i]+j)+=penalty;

	mat Displacement=solve(MassMatrix/(dt*dt)+DampingMatrix/dt+StiffnessMatrix,force);
	Displacement-=PreDis;

	return Displacement;
}

void MyFFD::addDisplacementToControlPoint(mat Dis)
{
	mat Pos,Vel;
	Pos.set_size(3*NbPoint); Vel.set_size(3*NbPoint);

	for(int i=0;i<NbPoint;i++)
		for(int j=0;j<3;j++)
			Pos(3*i+j)=ControlPoint[i][j];


	// Position update
	Pos+=Dis;
	Vel=(Dis)/dt;

	//Update
	for(int i=0;i<NbPoint;i++){
		for(int j=0;j<3;j++){
			ControlPoint[i][j]=Pos(3*i+j);
			Velocity[i][j]=Vel(3*i+j);
		}
	}
}

mat MyFFD::returnPreDisplacement()
{
	mat Dis;
	Dis.set_size(3*NbPoint);

	for(int i=0;i<NbPoint;i++)
		for(int j=0;j<3;j++)
			Dis(3*i+j)=ControlPoint[i][j]-ControlPoint0[i][j];

	return Dis;
}

void MyFFD::applyMassAndConstraint(mat &mapping)
{
	mapping*=(dt*dt/M);
	int n=mapping.n_rows;
	for(int i=0;i<FixedConstraint.size();i++)
		for(int j=0;j<3;j++)
			for(int k=0;k<n;k++)
				mapping(k,3*FixedConstraint[i]+j)*=(M/(M+30000));

}

void MyFFD::computeAroundPoint()
{
	AroundPoint=new std::vector<int>[NbSurfacePoint];
	for(int i=0;i<NbFace;i++)
	{
		AroundPoint[Face[i][0]].push_back(i);
		AroundPoint[Face[i][1]].push_back(i);
		AroundPoint[Face[i][2]].push_back(i);
	}
}

void MyFFD::applyElasticConstraintForce()
{
	int idx;
	double K;
	for(int i=0;i<ElasticConstraint.size();i++)
	{
		idx=ElasticConstraint[i][0];
		K=ElasticConstraint[i][1];
		Force[idx]-=(ControlPoint[idx]-ControlPoint0[idx])*K;
	}
}

void MyFFD::updateCatheterInsertionPosition(Vec3d Pos, Vec3d Direc)
{
	Direc.normalize();
	

	//Move to origin
	for(int i=0;i<NbPoint;i++){
		ControlPoint[i]-=InsertedPoint;
		ControlPoint0[i]-=InsertedPoint;
	}
	


	//Set Inserted Point and Direction
	InsertedPoint=Pos;
	InsertedDirec=Direc;

	//Rotate
	Vec3d V=ControlPoint[0]-ControlPoint[1];
	V.normalize();
	Vec3d V2=V.cross(Direc);

	double angle=asin(V2.norm());
	if(angle>0.01){
		V2.normalize();
		Mat3x3d rot;
		computeRotationMatrix(V2, angle, rot);

		for(int i=0;i<NbPoint;i++){
			ControlPoint[i]=rot*ControlPoint[i];
			ControlPoint0[i]=rot*ControlPoint0[i];
		}
	}


	//Move to tip of endoscope 
	for(int i=0;i<NbPoint;i++){
		ControlPoint[i]+=Pos;
		ControlPoint0[i]+=Pos;
	}


}

void MyFFD::updateCatheter(Vec3d Pos, Vec3d Direc, Vec3d basis, double angle)
{
	Direc.normalize();


	//Move to origin
	for(int i=0;i<NbPoint;i++){
		ControlPoint[i]-=InsertedPoint;
		ControlPoint0[i]-=InsertedPoint;
	}



	//Set Inserted Point and Direction
	InsertedPoint=Pos;
	InsertedDirec=Direc;
	rotateCatheter(basis,angle);

	//Rotate
	Vec3d V=ControlPoint[0]-ControlPoint[1];
	V.normalize();
	Vec3d V2=V.cross(InsertedDirec);

	angle=asin(V2.norm());
	if(angle>0.01){
		V2.normalize();
		Mat3x3d rot;
		computeRotationMatrix(V2, angle, rot);

		for(int i=0;i<NbPoint;i++){
			ControlPoint[i]=rot*ControlPoint[i];
			ControlPoint0[i]=rot*ControlPoint0[i];
		}
	}


	//Move to tip of endoscope 
	for(int i=0;i<NbPoint;i++){
		ControlPoint[i]+=Pos;
		ControlPoint0[i]+=Pos;
	}


}

void MyFFD::rotateCatheter( Vec3d basis, double angle)
{
	basis.normalize();

	Mat3x3d rot;
	computeRotationMatrix(basis, angle, rot);

	InsertedDirec=rot*InsertedDirec;
}

void MyFFD::loadTextureCoordinate( char* filename )
{
	FILE *fp = fopen(filename,"r");
	
	int Nb;

	fscanf(fp,"%d\n",&Nb);
	texCoord.set_size(Nb,2);
	for(int i=0;i<Nb;i++)
		fscanf(fp,"%lf %lf\n",&texCoord(i,0),&texCoord(i,1));

	fscanf(fp,"%d\n",&Nb);
	FaceofTex.set_size(Nb,3);
	for(int i=0;i<Nb;i++)
		fscanf(fp,"%lf %lf %lf\n",&FaceofTex(i,0),&FaceofTex(i,1),&FaceofTex(i,2));

	fclose(fp);

}

void MyFFD::loadConstraints( char* filename )
{
	FILE *fp = fopen(filename,"r");

	int Nb,Node;

	fscanf(fp,"%d\n",&Nb);

	for(int i=0;i<Nb;i++){
		fscanf(fp,"%d\n",&Node);
		addFixedConstraint(Node);
	}


	fclose(fp);
}

void MyFFD::moveCatheter( Vec3d distance )
{
	Vec3d newPos = InsertedPoint + distance;
	updateCatheterInsertionPosition(newPos, InsertedDirec);
}

void MyFFD::drawCylinder( Vec3f a, Vec3f b, float radius )
{
	// This is the default direction for the cylinders to face in OpenGL
	Vec3f z = Vec3f(0,0,1);         
	// Get diff between two points you want cylinder along
	Vec3f p = (a - b);                               
	// Get CROSS product (the axis of rotation)
	Vec3f t = z.cross(p); 

	// Get angle. LENGTH is magnitude of the vector
	double angle = 180 / PI * acos (z*p/ p.norm());

	glPushMatrix();
	glTranslated(b[0],b[1],b[2]);
	glRotated(angle,t[0],t[1],t[2]);

	GLUquadricObj *quadratic = gluNewQuadric();
	gluCylinder(quadratic, radius, radius, p.norm(), 10, 5);
	glPopMatrix();
}

void MyFFD::drawCatheter( int mode )
{
	glColor3f(0.0,0.0,1.0);
	glBegin(GL_LINES);
	for (int i=0; i<NbPoint-1; i++)
	{
		Vec3d point1 = ControlPoint[i];
		Vec3d point2 = ControlPoint[i+1];

		glVertex3f(point1[0], point1[1], point1[2]);
		glVertex3f(point2[0], point2[1], point2[2]);
	}
	glEnd();

	glPointSize(4.0);
	glBegin(GL_POINTS);
	for (int i=0; i<NbPoint; i++)
	{
		Vec3d point1 = ControlPoint[i];
		glVertex3f(point1[0], point1[1], point1[2]);
	}
	glEnd();


	if (mode == 2)
	{
		for (int i=0; i<NbPoint-1; i++)
		{
			Vec3d point1 = ControlPoint[i];
			Vec3d point2 = ControlPoint[i+1];

			drawCylinder(point1, point2, RadiusOfEndoscope);
		}
	}




	float arrowLength = 10;
	glColor3f(1.0,0.0,0.0);
	glPointSize(3);
	glBegin(GL_LINES);
	glVertex3f(InsertedPoint[0], InsertedPoint[1], InsertedPoint[2]);
	Vec3d point2 = InsertedPoint+InsertedDirec*arrowLength;
	glVertex3f(point2[0], point2[1], point2[2]);
	glEnd();
}


