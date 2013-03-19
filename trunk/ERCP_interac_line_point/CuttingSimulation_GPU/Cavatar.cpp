#include "StdAfx.h"
#include "Cavatar.h"

Cavatar::Cavatar(void)
{
}

Cavatar::~Cavatar(void)
{
}

void Cavatar::init( Vec3f _P1, Vec3f _P2, double _radius )
{
	P1=_P1;
	P2=_P2;
	radius=_radius;
	length=(P2-P1).norm();
	Direc=(P2-P1);
	Direc.normalize();
}

void Cavatar::update( Vec3f _P1, Vec3f _P2 , double dt )
{
	preP1=P1;
	preP2=P2;

	P1=_P1;
	P2=_P2;
	Vel1=(P1-preP1)/dt;
	Vel2=(P2-preP2)/dt;

	Direc=(P2-P1);
	Direc.normalize();

	length=(P2-P1).norm();

}

void Cavatar::move( double x,double y,double z )
{
	preP1=P1;
	preP2=P2;

	P1[0]+=x;
	P1[1]+=y;
	P1[2]+=z;

	P2[0]+=x;
	P2[1]+=y;
	P2[2]+=z;


	Direc=(P2-P1);
	Direc.normalize();


}

void Cavatar::draw( Vec3f color )
{
	GLUquadricObj *qobj = 0;
	qobj = gluNewQuadric();

	double Cos;
	Vec3d normal;
	Vec3d Zaxis(0,0,1);

	normal=Zaxis.cross(Direc);
	normal.normalize();

	Cos=Direc*Zaxis;	
	Cos=180/PI*acos(Cos);

	glColor3f(color[0],color[1],color[2]);


	glPushMatrix();
	glTranslatef((GLfloat)P1[0],(GLfloat)P1[1],(GLfloat)P1[2]);
	gluSphere(qobj,radius,20,10);
	glPopMatrix();

	glPushMatrix();
	glTranslatef((GLfloat)P2[0],(GLfloat)P2[1],(GLfloat)P2[2]);
	gluSphere(qobj,radius,20,10);
	glPopMatrix();

	glPushMatrix();
	glTranslatef((GLfloat)P1[0],(GLfloat)P1[1],(GLfloat)P1[2]);
	glRotatef((GLfloat)Cos,(GLfloat)normal[0],(GLfloat)normal[1],(GLfloat)normal[2]);
	gluCylinder(qobj,(GLfloat)radius,(GLfloat)radius,length,20,20);
	glPopMatrix();


}

void Cavatar::drawLine( Vec3f color )
{

	glColor3f(color[0],color[1],color[2]);


	glLineWidth((GLfloat)3);

	glBegin(GL_LINES);
	glVertex3f((GLfloat)P1[0],(GLfloat)P1[1],(GLfloat)P1[2]);
	glVertex3f((GLfloat)P2[0],(GLfloat)P2[1],(GLfloat)P2[2]);
	glEnd();


}


