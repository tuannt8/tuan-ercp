#ifndef SPRING_FORCE_H
#define SPRING_FORCE_H

#include "../DataTypes/Vec.h"
#include "../DataTypes/Mat.h"
#include "../DataTypes/Newmat/Newmat.h"
#include <vector>

using namespace NEWMAT;

class Spring
{
public:
	int		m1, m2;  ///< the two extremities of the spring: masses m1 and m2
	double	ks;      ///< spring stiffness
	double	kd;      ///< damping factor
	double	initpos; ///< rest length of the spring

	Spring(int m1=0, int m2=0, double ks=0.0, double kd=0.0, double initpos=0.0)
		: m1(m1), m2(m2), ks((double)ks), kd((double)kd), initpos((double)initpos)
	{
	}

	Spring(int m1, int m2, float ks, float kd=0, float initpos=0)
		: m1(m1), m2(m2), ks((double)ks), kd((double)kd), initpos((double)initpos)
	{
	}
};

class SpringForce
{
public:
	SpringForce(void);
	SpringForce(double _ks, double _kd);
	~SpringForce(void);

public:
	/* initialize */
	void init(double _ks, double _kd);

	/* Obj file load */
	void loadObjData(char* filename);

	/* Get parameters */
	double getStiffness(); 
	double getDamping();
	int getNbSprings();
	void getPointIdxOfSpring(int springIdx, int& idx1, int& idx2);
	double getPotentialEnergy();

	/* set parameters */
	void setStiffness(double _ks);
	void setDamping(double _kd);

	/* add force */
	void addForce(Vec3d* f, Vec3d* x, Vec3d* v);

	/* add springs from set of edges */
	void addSprings(const std::vector<FixedArray<int,2>>& edges, const std::vector<Vec3d>& points, double ks, double kd);

	/* add spring */
	void addSpring(int m1, int m2, double ks, double kd, double initlen);

	/* draw */
	void draw();

private:
	/* add forces caused by one spring */
	void addSpringForce(double& ener, Vec3d* f, Vec3d* p, Vec3d* v, const Spring& spring);

	void computePotentialEnergy();

	/* clear memory */
	void clear();

private:
	/*********************/
	/* System parameters */
	/*********************/
	double ks;								// stiffness factor
	double kd;								// damping factor
	std::vector<Spring> springs;		// springs
	double potentialEnergy;
};

#endif