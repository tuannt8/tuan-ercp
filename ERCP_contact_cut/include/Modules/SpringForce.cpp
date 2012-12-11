#include "StdAfx.h"
#include "SpringForce.h"

SpringForce::SpringForce(void)
{
	ks=0;
	kd=0;
}

SpringForce::SpringForce(double _ks, double _kd)
{
	ks=_ks;
	kd=_kd;
}

SpringForce::~SpringForce(void)
{
}

void SpringForce::init(double _ks, double _kd)
{
	ks=_ks;
	kd=_kd;
}

double SpringForce::getStiffness() 
{ 
	return ks; 
}

double SpringForce::getDamping()	
{ 
	return kd; 
}

int SpringForce::getNbSprings()	
{ 
	return (int)springs.size(); 
}

void SpringForce::getPointIdxOfSpring(int springIdx, int& idx1, int& idx2)
{
	idx1=springs[springIdx].m1;
	idx2=springs[springIdx].m2;
}

void SpringForce::setStiffness(double _ks)
{ 
	ks=_ks; 
}

void SpringForce::setDamping(double _kd)
{ 
	kd=_kd; 
}

void SpringForce::loadObjData(char* filename)
{

}

void SpringForce::addForce(Vec3d* f, Vec3d* x, Vec3d* v)
{
	potentialEnergy = 0;
	for (unsigned int i=0; i<springs.size(); i++)
	{
		addSpringForce(potentialEnergy,f, x, v, springs[i]);
	}
}

void SpringForce::addSpringForce(double& ener, Vec3d* f, Vec3d* p, Vec3d* v, const Spring& spring)
{
	int a = spring.m1;
	int b = spring.m2;

	Vec3d u = p[b]-p[a];
	double d = u.norm();
	double inverseLength = 1.0f/d;
	
	//if( d<1.0e-4 ) // null length => no force
	//	return;

	u *= inverseLength;
	double elongation = (double)(d - spring.initpos);
	ener += elongation * elongation * spring.ks /2;
	Vec3d relativeVelocity = v[b]-v[a];
	double elongationVelocity = dot(u,relativeVelocity);
	double forceIntensity = (double)(spring.ks*elongation+spring.kd*elongationVelocity);
	Vec3d force = u*forceIntensity;
	f[a]+=force;
	f[b]-=force;
}

void SpringForce::addSprings(const std::vector<FixedArray<int,2>>& edges, const std::vector<Vec3d>& points, double ks, double kd)
{
	for(unsigned int i=0;i<edges.size();i++)
	{
		double initlen=(points[edges[i][0]]-points[edges[i][1]]).norm();
		springs.push_back(Spring(edges[i][0],edges[i][1],ks,kd,initlen));
	}
}

void SpringForce::addSpring(int m1, int m2, double ks, double kd, double initlen)
{
	springs.push_back(Spring(m1,m2,ks,kd,initlen));
}

void SpringForce::clear()
{
	springs.clear();
}

void SpringForce::draw()
{
}

double SpringForce::getPotentialEnergy()
{
	return potentialEnergy;
}