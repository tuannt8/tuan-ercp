#include "./DataTypes/Define.h"
#include "./DataTypes/Vec.h"
#include "./DataTypes/Mat.h"
#include "./DataTypes/Quat.h"
#include "./Modules/TimeTick.h"

#include <GL/glu.h>
#include <GL/gl.h>
#include <vector>


class Cavatar
{
public:
	Cavatar(void);
	~Cavatar(void);

	void init(Vec3f _P1, Vec3f _P2, double _radius);
	void update(Vec3f _P1, Vec3f _P2,double dt);
	void move(double x,double y,double z);

	void draw(Vec3f color);
	void drawLine( Vec3f color );


	Vec3f getPoint1(){return P1;};
	Vec3f getPoint2(){return P2;};
	double getRadius(){return radius;};

private:
	//Variables
	Vec3f P1, P2;
	Vec3f preP1, preP2;
	Vec3f Direc;
	double length;
	double radius;
	Vec3f Vel1, Vel2;
	Vec3f Acc1, Acc2;


};
