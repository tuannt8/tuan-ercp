#include "stdafx.h"
#include "EFG_CPU.h"

EFG_CPU::EFG_CPU(void)
{
	GlobalStiffness=NULL;
	F=fopen("EFG_CPU.txt","w");
	NbNodeInside=0;
	ConstrainedNodeIdx=-1;

	float length=2;
	X=Vec3f(length,0,0);
	Y=Vec3f(0,length,0);
	Z=Vec3f(0,0,length);
}

EFG_CPU::~EFG_CPU(void)
{
	if(GlobalStiffness)
		delete GlobalStiffness;
	fclose(F);
}

void EFG_CPU::writeEFGFile(char* filename)
{
	FILE* f=fopen(filename,"w");
	fprintf(f,"%d %f\n", NbNode, SupportRadius);

	for(int i=0;i<NbNode;i++)
		fprintf(f,"%f %f %f %f\n", NodePos0[i][0], NodePos0[i][1], NodePos0[i][2], NodeVolume[i]);
	fclose(f);
}

void EFG_CPU::init(char* filename)
{
	FILE* f=fopen(filename,"r");
	fscanf(f,"%d %f", &NbNode, &SupportRadius);
	Density=DENSITY;

	//Memory allocation
	NodePos0.resize(NbNode);
	NodePos.resize(NbNode);
	NodePosX.resize(NbNode);
	NodePosY.resize(NbNode);
	NodePosZ.resize(NbNode);
	NodeVel.resize(NbNode);
	NodeForce.resize(NbNode);
	NodeDis.resize(NbNode);
	NodeStress.resize(NbNode);
	NodeStrain.resize(NbNode);
	NodeVolume.resize(NbNode);

	//Data copy
	for(int i=0;i<NbNode;i++)
	{
		float temp;
		Vec3f nodePosVec;
		fscanf(f,"%f %f %f %f", &NodePos0[i][0], &NodePos0[i][1], &NodePos0[i][2], &NodeVolume[i]);
		NodePos[i]=NodePos0[i];
	}

	// 1. construct material stiffness matrix
	constructMaterialStiffness();

	// 2. neighbor information 
	initNeighborInformation();

	// 3. compute node mass and volume
	computeNodeVolume();

	// 4. init shape function value
	initShapeFuncValue();

	// 5. init bvh
	BVHAABB.init(&NodePos, &Edge);
	BVHAABB.constructAABBTree();
}

void EFG_CPU::init(std::vector<Vec3f>* nodePos, float nodeVolume, float supportRadius, int nbNodeInside)
{
	NbNode=nodePos->size();
	SupportRadius=supportRadius;
	NbNodeInside=nbNodeInside;
	Density=DENSITY;

	//Memory allocation
	NodePos0.resize(NbNode);
	NodePos.resize(NbNode);
	NodePosX.resize(NbNode);
	NodePosY.resize(NbNode);
	NodePosZ.resize(NbNode);
	NodeVel.resize(NbNode);
	NodeForce.resize(NbNode);
	NodeDis.resize(NbNode);
	NodeStress.resize(NbNode);
	NodeStrain.resize(NbNode);
	NodeVolume.resize(NbNode);

	//Data copy
	for(int i=0;i<NbNode;i++)
	{
		NodePos[i]=NodePos0[i]=(*nodePos)[i];
		NodeVolume[i]=nodeVolume;
	}

	// 1. construct material stiffness matrix
	constructMaterialStiffness();

	// 2. neighbor information 
	initNeighborInformation();

	// 3. compute node mass and volume
	computeNodeVolume();

	// 4. init shape function value
	initShapeFuncValue();

	// 5. init bvh
	BVHAABB.init(&NodePos, &Edge);
	BVHAABB.constructAABBTree();
}

void EFG_CPU::init(std::vector<Vec3f>* nodePos, float nodeVolume, float supportRadius)
{
	NbNode=nodePos->size();
	SupportRadius=supportRadius;
	Density=DENSITY;

	//Memory allocation
	NodePos0.resize(NbNode);
	NodePos.resize(NbNode);
	NodePosX.resize(NbNode);
	NodePosY.resize(NbNode);
	NodePosZ.resize(NbNode);
	NodeVel.resize(NbNode);
	NodeForce.resize(NbNode);
	NodeDis.resize(NbNode);
	NodeStress.resize(NbNode);
	NodeStrain.resize(NbNode);
	NodeVolume.resize(NbNode);

	//Data copy
	for(int i=0;i<NbNode;i++)
	{
		NodePos[i]=NodePos0[i]=(*nodePos)[i];
		NodeVolume[i]=nodeVolume;
	}

	// 1. construct material stiffness matrix
	constructMaterialStiffness();

	// 2. neighbor information 
	initNeighborInformation();

	// 3. compute node mass and volume
	computeNodeVolume();

	// 4. init shape function value
	initShapeFuncValue();

	// 5. init bvh
	BVHAABB.init(&NodePos, &Edge);
	BVHAABB.constructAABBTree();
}

void EFG_CPU::init(std::vector<Vec3f>* nodePos, float nodeVolume, float supportRadius, float density)
{
	NbNode=nodePos->size();
	SupportRadius=supportRadius;
	Density=density;

	//Memory allocation
	NodePos0.resize(NbNode);
	NodePos.resize(NbNode);
	NodePosX.resize(NbNode);
	NodePosY.resize(NbNode);
	NodePosZ.resize(NbNode);
	NodeVel.resize(NbNode);
	NodeForce.resize(NbNode);
	NodeDis.resize(NbNode);
	NodeStress.resize(NbNode);
	NodeStrain.resize(NbNode);
	NodeVolume.resize(NbNode);

	//Data copy
	for(int i=0;i<NbNode;i++)
	{
		NodePos[i]=NodePos0[i]=(*nodePos)[i];
		NodeVolume[i]=nodeVolume;
	}

	// 1. construct material stiffness matrix
	constructMaterialStiffness();

	// 2. neighbor information 
	initNeighborInformation();

	// 3. compute node mass and volume
	computeNodeVolume();

	// 4. init shape function value
	initShapeFuncValue();
}

void EFG_CPU::readNodeVolume(char* filename)
{
	FILE* f=fopen(filename,"r");
	NodeVolume.clear();
	NodeVolume.resize(NbNode);

	//Data copy
	for(int i=0;i<NbNode;i++)
	{
		fscanf(f,"%f", &NodeVolume[i]);
	}
}

void EFG_CPU::constructMaterialStiffness()
{
	// 1. construct material stiffness matrix
	MaterialStiffness[0][0] = MaterialStiffness[1][1] = MaterialStiffness[2][2] = 1;
	MaterialStiffness[0][1] = MaterialStiffness[0][2] = MaterialStiffness[1][0] = MaterialStiffness[1][2] = MaterialStiffness[2][0] = MaterialStiffness[2][1] = NU/(1-NU);
	MaterialStiffness[0][3] = MaterialStiffness[0][4] = MaterialStiffness[0][5] = 0;
	MaterialStiffness[1][3] = MaterialStiffness[1][4] = MaterialStiffness[1][5] = 0;
	MaterialStiffness[2][3] = MaterialStiffness[2][4] = MaterialStiffness[2][5] = 0;
	MaterialStiffness[3][0] = MaterialStiffness[3][1] = MaterialStiffness[3][2] = MaterialStiffness[3][4] = MaterialStiffness[3][5] = 0;
	MaterialStiffness[4][0] = MaterialStiffness[4][1] = MaterialStiffness[4][2] = MaterialStiffness[4][3] = MaterialStiffness[4][5] = 0;
	MaterialStiffness[5][0] = MaterialStiffness[5][1] = MaterialStiffness[5][2] = MaterialStiffness[5][3] = MaterialStiffness[5][4] = 0;
	MaterialStiffness[3][3] = MaterialStiffness[4][4] = MaterialStiffness[5][5] = (1-2*NU)/(2*(1-NU));
	MaterialStiffness*= (E*(1-NU))/((1+NU)*(1-2*NU));
}

void EFG_CPU::computeNodeVolume()
{
	NodeMass.resize(NbNode);

	//compute mass matrix
	for(int i=0;i<NbNode;i++)
		NodeMass[i]=Density*NodeVolume[i];
}

void EFG_CPU::computeNodeRotation()
{
	updatePositionAtNodeCoord();

	if(NodeRot.empty())
		NodeRot.resize(NbNode);

	std::vector<Mat3x3f> rot; rot.resize(NbNode);
	for(int i=0;i<NbNode;i++)
	{
		Vec3f x=NodePosX[i]-NodePos[i]; x.normalize();
		Vec3f y=NodePosY[i]-NodePos[i]; y.normalize();
		Vec3f z=x.cross(y);
		y=z.cross(x);
		for(int j=0;j<3;j++)
		{
			rot[i](j,0)=x[j];
			rot[i](j,1)=y[j];
			rot[i](j,2)=z[j];
		}
	}
	for(int i=0;i<NbNode;i++)
	{
		for(int j=0;j<NeighborNodeIdx[i].size();j++)
		{
			int idx=NeighborNodeIdx[i][j];
			NodeRot[i]+=rot[idx];
		}
		NodeRot[i].normalize();
	}
}

void EFG_CPU::initNeighborInformation()
{
	NeighborNodeIdx.resize(NbNode);
	EdgeAroundNode.resize(NbNode);
	for(int i=0;i<NbNode;i++)
	{
		for(int j=0;j<NbNode;j++)
		{
			if((NodePos[i]-NodePos[j]).norm()<SupportRadius)
			{
				NeighborNodeIdx[i].push_back(j);
				Vec2i edge(i,j);
				Edge.push_back(edge);
				EdgeAroundNode[i].push_back(Edge.size()-1);
			}
		}
	}
}

void EFG_CPU::initShapeFuncValue()
{
	//1. Init MLS shape function
	ShapeFunc.init(&NodePos0, SupportRadius, WEIGHT_FUNC_TYPE);

	//2. Init vector to neighbor node
	VecToNeighbor.resize(NbNode);
	for(int i=0;i<NbNode;i++)
	{
		VecToNeighbor[i].resize(NeighborNodeIdx[i].size());
		for(int j=0;j<NeighborNodeIdx[i].size();j++)
			VecToNeighbor[i][j]=NodePos[NeighborNodeIdx[i][j]]-NodePos[i];
	}

	//2. Compute moment matrix at each node
	MomentMatrixAtNode.resize(NbNode);
	MomentMatrixAtNodeX.resize(NbNode);
	MomentMatrixAtNodeY.resize(NbNode);
	MomentMatrixAtNodeZ.resize(NbNode);
	for(int i=0;i<NbNode;i++)
	{
		ShapeFunc.computeMomentMatrixAtNode(MomentMatrixAtNode[i], NeighborNodeIdx[i], i);
		ShapeFunc.computeMomentMatrix(MomentMatrixAtNodeX[i], NeighborNodeIdx[i], NodePos0[i]+X);
		ShapeFunc.computeMomentMatrix(MomentMatrixAtNodeY[i], NeighborNodeIdx[i], NodePos0[i]+Y);
		ShapeFunc.computeMomentMatrix(MomentMatrixAtNodeZ[i], NeighborNodeIdx[i], NodePos0[i]+Z);
	}

	//3. Compute moment matrix inverse at each node
	MomentMatrixInvAtNode.resize(NbNode);
	MomentMatrixInvAtNodeX.resize(NbNode);
	MomentMatrixInvAtNodeY.resize(NbNode);
	MomentMatrixInvAtNodeZ.resize(NbNode);
	for(int i=0;i<NbNode;i++)
	{
		if(!invertMatrix(MomentMatrixInvAtNode[i], MomentMatrixAtNode[i]))
		{
			fprintf(F,"Moment matrix %d is not invertible!!\n",i);
			fprintf(F,"NbNeighbor = %d\n",NeighborNodeIdx[i].size());
		}
		if(!invertMatrix(MomentMatrixInvAtNodeX[i], MomentMatrixAtNodeX[i]))
		{
			fprintf(F,"Moment matrix %d is not invertible!!\n",i);
			fprintf(F,"NbNeighbor = %d\n",NeighborNodeIdx[i].size());
		}
		if(!invertMatrix(MomentMatrixInvAtNodeY[i], MomentMatrixAtNodeY[i]))
		{
			fprintf(F,"Moment matrix %d is not invertible!!\n",i);
			fprintf(F,"NbNeighbor = %d\n",NeighborNodeIdx[i].size());
		}
		if(!invertMatrix(MomentMatrixInvAtNodeZ[i], MomentMatrixAtNodeZ[i]))
		{
			fprintf(F,"Moment matrix %d is not invertible!!\n",i);
			fprintf(F,"NbNeighbor = %d\n",NeighborNodeIdx[i].size());
		}
	}

	//4. Compute shape function at node
	ShapeFuncAtNode.resize(NbNode);
	ShapeFuncAtNodeX.resize(NbNode);
	ShapeFuncAtNodeY.resize(NbNode);
	ShapeFuncAtNodeZ.resize(NbNode);

	for(int i=0;i<NbNode;i++)
	{
		int nbNeighbor=NeighborNodeIdx[i].size();
		ShapeFuncAtNode[i].resize(nbNeighbor);
		ShapeFuncAtNodeX[i].resize(nbNeighbor);
		ShapeFuncAtNodeY[i].resize(nbNeighbor);
		ShapeFuncAtNodeZ[i].resize(nbNeighbor);

		for(unsigned int j=0;j<nbNeighbor;j++)
		{
			ShapeFuncAtNode[i][j]=ShapeFunc.computeShapeFuncValue(i, NodePos0[NeighborNodeIdx[i][j]], MomentMatrixInvAtNode[NeighborNodeIdx[i][j]]);
			ShapeFuncAtNodeX[i][j]=ShapeFunc.computeShapeFuncValue(NeighborNodeIdx[i][j], NodePos0[i]+X, MomentMatrixInvAtNodeX[i]);
			ShapeFuncAtNodeY[i][j]=ShapeFunc.computeShapeFuncValue(NeighborNodeIdx[i][j], NodePos0[i]+Y, MomentMatrixInvAtNodeY[i]);
			ShapeFuncAtNodeZ[i][j]=ShapeFunc.computeShapeFuncValue(NeighborNodeIdx[i][j], NodePos0[i]+Z, MomentMatrixInvAtNodeZ[i]);
		}
	}

	//5. Compute shape function drvX at node
	MomentMatrixDrvXAtNode.resize(NbNode);
	ShapeFuncDerivXAtNode.resize(NbNode);
	ShapeFuncDerivXAtNodeInv.resize(NbNode);
	for(int i=0;i<NbNode;i++)
		ShapeFunc.computeMomentMatrixDrvXAtNode(i, MomentMatrixDrvXAtNode[i], NeighborNodeIdx[i]);
	for(int i=0;i<NbNode;i++)
	{
		int nbNeighbor=NeighborNodeIdx[i].size();
		ShapeFuncDerivXAtNode[i].resize(nbNeighbor);
		ShapeFuncDerivXAtNodeInv[i].resize(nbNeighbor);
		for(unsigned int j=0;j<nbNeighbor;j++)
		{
			ShapeFuncDerivXAtNode[i][j]=ShapeFunc.computeShapeFuncDrvX(i, NodePos0[NeighborNodeIdx[i][j]], MomentMatrixInvAtNode[NeighborNodeIdx[i][j]], MomentMatrixDrvXAtNode[NeighborNodeIdx[i][j]]);
			ShapeFuncDerivXAtNodeInv[i][j]=ShapeFunc.computeShapeFuncDrvX(NeighborNodeIdx[i][j], NodePos0[i], MomentMatrixInvAtNode[i], MomentMatrixDrvXAtNode[i]);
		}
	}

	//6. Compute shape function drvY at node
	MomentMatrixDrvYAtNode.resize(NbNode);
	ShapeFuncDerivYAtNode.resize(NbNode);
	ShapeFuncDerivYAtNodeInv.resize(NbNode);
	for(int i=0;i<NbNode;i++)
		ShapeFunc.computeMomentMatrixDrvYAtNode(i, MomentMatrixDrvYAtNode[i], NeighborNodeIdx[i]);
	for(int i=0;i<NbNode;i++)
	{
		int nbNeighbor=NeighborNodeIdx[i].size();
		ShapeFuncDerivYAtNode[i].resize(nbNeighbor);
		ShapeFuncDerivYAtNodeInv[i].resize(nbNeighbor);
		for(unsigned int j=0;j<nbNeighbor;j++)
		{
			ShapeFuncDerivYAtNode[i][j]=ShapeFunc.computeShapeFuncDrvY(i, NodePos0[NeighborNodeIdx[i][j]], MomentMatrixInvAtNode[NeighborNodeIdx[i][j]], MomentMatrixDrvYAtNode[NeighborNodeIdx[i][j]]);
			ShapeFuncDerivYAtNodeInv[i][j]=ShapeFunc.computeShapeFuncDrvY(NeighborNodeIdx[i][j], NodePos0[i], MomentMatrixInvAtNode[i], MomentMatrixDrvYAtNode[i]);
		}
	}

	//7. Compute shape function drvZ at node
	MomentMatrixDrvZAtNode.resize(NbNode);
	ShapeFuncDerivZAtNode.resize(NbNode);
	ShapeFuncDerivZAtNodeInv.resize(NbNode);
	for(int i=0;i<NbNode;i++)
		ShapeFunc.computeMomentMatrixDrvZAtNode(i, MomentMatrixDrvZAtNode[i], NeighborNodeIdx[i]);
	for(int i=0;i<NbNode;i++)
	{
		int nbNeighbor=NeighborNodeIdx[i].size();
		ShapeFuncDerivZAtNode[i].resize(nbNeighbor);
		ShapeFuncDerivZAtNodeInv[i].resize(nbNeighbor);
		for(unsigned int j=0;j<nbNeighbor;j++)
		{
			ShapeFuncDerivZAtNode[i][j]=ShapeFunc.computeShapeFuncDrvZ(i, NodePos0[NeighborNodeIdx[i][j]], MomentMatrixInvAtNode[NeighborNodeIdx[i][j]], MomentMatrixDrvZAtNode[NeighborNodeIdx[i][j]]);
			ShapeFuncDerivZAtNodeInv[i][j]=ShapeFunc.computeShapeFuncDrvZ(NeighborNodeIdx[i][j], NodePos0[i], MomentMatrixInvAtNode[i], MomentMatrixDrvZAtNode[i]);
		}
	}
}

void EFG_CPU::reInitShapeFuncValue()
{
	//1. Init MLS shape function
	ShapeFunc.init(&NodePos0, SupportRadius, WEIGHT_FUNC_TYPE);

	//2. Compute moment matrix at each node
	MomentMatrixAtNode.clear();
	MomentMatrixAtNode.resize(NbNode);
	for(int i=0;i<NbNode;i++)
		ShapeFunc.computeMomentMatrixAtNode(MomentMatrixAtNode[i], NeighborNodeIdx[i], i);

	//3. Compute moment matrix inverse at each node
	MomentMatrixInvAtNode.clear();
	MomentMatrixInvAtNode.resize(NbNode);
	for(int i=0;i<NbNode;i++)
	{
		if(!invertMatrix(MomentMatrixInvAtNode[i], MomentMatrixAtNode[i]))
		{
			fprintf(F,"Moment matrix %d is not invertible!!\n",i);
			fprintf(F,"NbNeighbor = %d\n",NeighborNodeIdx[i].size());
		}
	}

	//4. Compute shape function at node
	ShapeFuncAtNode.clear();
	ShapeFuncAtNode.resize(NbNode);
	for(int i=0;i<NbNode;i++)
	{
		int nbNeighbor=NeighborNodeIdx[i].size();
		ShapeFuncAtNode[i].resize(nbNeighbor);
		for(unsigned int j=0;j<nbNeighbor;j++)
		{
			ShapeFuncAtNode[i][j]=ShapeFunc.computeShapeFuncValue(i, NodePos0[NeighborNodeIdx[i][j]], MomentMatrixInvAtNode[NeighborNodeIdx[i][j]]);
		}
	}

	//5. Compute shape function drvX at node
	MomentMatrixDrvXAtNode.clear();
	ShapeFuncDerivXAtNode.clear();
	ShapeFuncDerivXAtNodeInv.clear();

	MomentMatrixDrvXAtNode.resize(NbNode);
	ShapeFuncDerivXAtNode.resize(NbNode);
	ShapeFuncDerivXAtNodeInv.resize(NbNode);
	for(int i=0;i<NbNode;i++)
		ShapeFunc.computeMomentMatrixDrvXAtNode(i, MomentMatrixDrvXAtNode[i], NeighborNodeIdx[i]);
	for(int i=0;i<NbNode;i++)
	{
		int nbNeighbor=NeighborNodeIdx[i].size();
		ShapeFuncDerivXAtNode[i].resize(nbNeighbor);
		ShapeFuncDerivXAtNodeInv[i].resize(nbNeighbor);
		for(unsigned int j=0;j<nbNeighbor;j++)
		{
			ShapeFuncDerivXAtNode[i][j]=ShapeFunc.computeShapeFuncDrvX(i, NodePos0[NeighborNodeIdx[i][j]], MomentMatrixInvAtNode[NeighborNodeIdx[i][j]], MomentMatrixDrvXAtNode[NeighborNodeIdx[i][j]]);
			ShapeFuncDerivXAtNodeInv[i][j]=ShapeFunc.computeShapeFuncDrvX(NeighborNodeIdx[i][j], NodePos0[i], MomentMatrixInvAtNode[i], MomentMatrixDrvXAtNode[i]);
		}
	}

	//6. Compute shape function drvY at node
	MomentMatrixDrvYAtNode.clear();
	ShapeFuncDerivYAtNode.clear();
	ShapeFuncDerivYAtNodeInv.clear();

	MomentMatrixDrvYAtNode.resize(NbNode);
	ShapeFuncDerivYAtNode.resize(NbNode);
	ShapeFuncDerivYAtNodeInv.resize(NbNode);
	for(int i=0;i<NbNode;i++)
		ShapeFunc.computeMomentMatrixDrvYAtNode(i, MomentMatrixDrvYAtNode[i], NeighborNodeIdx[i]);
	for(int i=0;i<NbNode;i++)
	{
		int nbNeighbor=NeighborNodeIdx[i].size();
		ShapeFuncDerivYAtNode[i].resize(nbNeighbor);
		ShapeFuncDerivYAtNodeInv[i].resize(nbNeighbor);
		for(unsigned int j=0;j<nbNeighbor;j++)
		{
			ShapeFuncDerivYAtNode[i][j]=ShapeFunc.computeShapeFuncDrvY(i, NodePos0[NeighborNodeIdx[i][j]], MomentMatrixInvAtNode[NeighborNodeIdx[i][j]], MomentMatrixDrvYAtNode[NeighborNodeIdx[i][j]]);
			ShapeFuncDerivYAtNodeInv[i][j]=ShapeFunc.computeShapeFuncDrvY(NeighborNodeIdx[i][j], NodePos0[i], MomentMatrixInvAtNode[i], MomentMatrixDrvYAtNode[i]);
		}
	}

	//7. Compute shape function drvZ at node
	MomentMatrixDrvZAtNode.clear();
	ShapeFuncDerivZAtNode.clear();
	ShapeFuncDerivZAtNodeInv.clear();

	MomentMatrixDrvZAtNode.resize(NbNode);
	ShapeFuncDerivZAtNode.resize(NbNode);
	ShapeFuncDerivZAtNodeInv.resize(NbNode);
	for(int i=0;i<NbNode;i++)
		ShapeFunc.computeMomentMatrixDrvZAtNode(i, MomentMatrixDrvZAtNode[i], NeighborNodeIdx[i]);
	for(int i=0;i<NbNode;i++)
	{
		int nbNeighbor=NeighborNodeIdx[i].size();
		ShapeFuncDerivZAtNode[i].resize(nbNeighbor);
		ShapeFuncDerivZAtNodeInv[i].resize(nbNeighbor);
		for(unsigned int j=0;j<nbNeighbor;j++)
		{
			ShapeFuncDerivZAtNode[i][j]=ShapeFunc.computeShapeFuncDrvZ(i, NodePos0[NeighborNodeIdx[i][j]], MomentMatrixInvAtNode[NeighborNodeIdx[i][j]], MomentMatrixDrvZAtNode[NeighborNodeIdx[i][j]]);
			ShapeFuncDerivZAtNodeInv[i][j]=ShapeFunc.computeShapeFuncDrvZ(NeighborNodeIdx[i][j], NodePos0[i], MomentMatrixInvAtNode[i], MomentMatrixDrvZAtNode[i]);
		}
	}
}

void EFG_CPU::boxConstraint(Vec3f LeftDown, Vec3f RightUp)
{
	for(int i=0;i<NbNode;i++)
	{
		if((NodePos0[i][0]>LeftDown[0])&&(NodePos0[i][0]<RightUp[0]))
		{
			if((NodePos0[i][1]>LeftDown[1])&&(NodePos0[i][1]<RightUp[1]))
			{
				if((NodePos0[i][2]>LeftDown[2])&&(NodePos0[i][2]<RightUp[2]))
				{
					FixedNodeIdx.push_back(i);
				}
			}
		}
	}
}

void EFG_CPU::draw(Vec3f color, int radius, int mode)
{
	if(mode==0)
	{
		glColor3f(color[0],color[1],color[2]);
		GLUquadricObj *qobj = 0;
		qobj = gluNewQuadric();
		for(int i=0;i<NodePos.size();i++)
		{
			glPushMatrix();
			glTranslatef((GLfloat)NodePos[i][0],(GLfloat)NodePos[i][1],(GLfloat)NodePos[i][2]);
			gluSphere(qobj,radius,20,20);
			glPopMatrix();
		}

		for(int i=0;i<FixedNodeIdx.size();i++)
		{
			glColor3f(1,0,0);
			glPushMatrix();
			glTranslatef((GLfloat)NodePos[FixedNodeIdx[i]][0],(GLfloat)NodePos[FixedNodeIdx[i]][1],(GLfloat)NodePos[FixedNodeIdx[i]][2]);
			//gluSphere(qobj,radius*1.2,20,20);
			glPopMatrix();
		}
	}
	if(mode==1)
	{
		glColor3f(color[0],color[1],color[2]);
		glPointSize(radius);
		glBegin(GL_POINTS);
		for(int i=0;i<NodePos.size();i++)
		{
			glVertex3f((GLfloat)NodePos[i][0],(GLfloat)NodePos[i][1],(GLfloat)NodePos[i][2]);
		}
		glEnd();
	}
}

void EFG_CPU::drawNodeInside(Vec3f color, int radius)
{
	glColor3f(color[0],color[1],color[2]);
	GLUquadricObj *qobj = 0;
	qobj = gluNewQuadric();
	for(int i=0;i<NbNodeInside;i++)
	{
		glPushMatrix();
		glTranslatef((GLfloat)NodePos[i][0],(GLfloat)NodePos[i][1],(GLfloat)NodePos[i][2]);
		gluSphere(qobj,radius,20,20);
		glPopMatrix();
	}
}

void EFG_CPU::drawEdge()
{
	for(int i=0;i<Edge.size();i++)
		drawEdge(i);
}

void EFG_CPU::drawBVH()
{
	BVHAABB.drawBoundingBox();
}

void EFG_CPU::drawNodeOrientation()
{
	computeNodeRotation();
	if(!NodeRot.empty())
	{
		for(int i=0;i<NodePos.size();i++)
			drawCoord(NodePos[i], NodeRot[i]);
	}
}

void EFG_CPU::drawCoord(Vec3f pos, Mat3x3f rot)
{
	float length=10;

	Vec3f x(length,0,0);
	Vec3f y(0,length,0);
	Vec3f z(0,0,length);

	x=rot*x;
	y=rot*y;
	z=rot*z;

	glBegin(GL_LINES);
	glColor3f(1,0,0);
	glVertex3f(pos[0],pos[1],pos[2]);
	glVertex3f(pos[0]+x[0],pos[1]+x[1],pos[2]+x[2]);

	glColor3f(0,1,0);
	glVertex3f(pos[0],pos[1],pos[2]);
	glVertex3f(pos[0]+y[0],pos[1]+y[1],pos[2]+y[2]);

	glColor3f(0,0,1);
	glVertex3f(pos[0],pos[1],pos[2]);
	glVertex3f(pos[0]+z[0],pos[1]+z[1],pos[2]+z[2]);
	glEnd();
}

void EFG_CPU::drawNodeCoord()
{
	float length=10;
	for(int i=0;i<NbNode;i++)
	{
		Vec3f pos=NodePos[i];
		Vec3f x=(NodePosX[i]-NodePos[i])*length;
		Vec3f y=(NodePosY[i]-NodePos[i])*length;
		Vec3f z=(NodePosZ[i]-NodePos[i])*length;

		glBegin(GL_LINES);
		glColor3f(1,0,0);
		glVertex3f(pos[0],pos[1],pos[2]);
		glVertex3f(pos[0]+x[0],pos[1]+x[1],pos[2]+x[2]);

		glColor3f(0,1,0);
		glVertex3f(pos[0],pos[1],pos[2]);
		glVertex3f(pos[0]+y[0],pos[1]+y[1],pos[2]+y[2]);

		glColor3f(0,0,1);
		glVertex3f(pos[0],pos[1],pos[2]);
		glVertex3f(pos[0]+z[0],pos[1]+z[1],pos[2]+z[2]);
		glEnd();
	}
}

void EFG_CPU::drawEdge(int idx)
{
	glBegin(GL_LINES);
	glVertex3f(NodePos[Edge[idx][0]][0],NodePos[Edge[idx][0]][1],NodePos[Edge[idx][0]][2]);
	glVertex3f(NodePos[Edge[idx][1]][0],NodePos[Edge[idx][1]][1],NodePos[Edge[idx][1]][2]);
	glEnd();
}


void EFG_CPU::addInternalForce()
{
	if(GlobalStiffness)
	{
		float* dis=new float[NodePos.size()*3];
		for(int i=0;i<NodePos.size();i++)
		{
			for(int j=0;j<3;j++)
				dis[i*3+j]=NodePos0[i][j]-NodePos[i][j];
		}
		for(int i=0;i<NodePos.size();i++)
		{
			for(int j=0;j<NodePos.size()*3;j++)
			{
				NodeForce[i][0]+=(*GlobalStiffness)(i*3,j)*dis[j];
				NodeForce[i][1]+=(*GlobalStiffness)(i*3+1,j)*dis[j];
				NodeForce[i][2]+=(*GlobalStiffness)(i*3+2,j)*dis[j];
			}
		}
		delete [] dis;
	}
	else
	{
		computeStrain();
		computeStress();
		computeForce();
	}
}

void EFG_CPU::computeStrain()
{
	for(int i=0;i<NbNode;i++)
		NodeStrain[i].clear();

	if(COROTATION)
	{
		// 3. Co-rotational method
		computeNodeRotation();

		for(int i=0;i<NbNode;i++)
		{
			int nbNeighbor=NeighborNodeIdx[i].size();
			for(int j=0;j<nbNeighbor;j++)
			{
				Vec3f dis;
				dis=NodeRot[i].transposed()*(NodePos[NeighborNodeIdx[i][j]]-NodePos[i])-(NodePos0[NeighborNodeIdx[i][j]]-NodePos0[i]);

				NodeStrain[i][0]+=ShapeFuncDerivXAtNodeInv[i][j]*dis[0];
				NodeStrain[i][1]+=ShapeFuncDerivYAtNodeInv[i][j]*dis[1];
				NodeStrain[i][2]+=ShapeFuncDerivZAtNodeInv[i][j]*dis[2];

				NodeStrain[i][3]+=(ShapeFuncDerivYAtNodeInv[i][j]*dis[0]+ShapeFuncDerivXAtNodeInv[i][j]*dis[1]);
				NodeStrain[i][4]+=(ShapeFuncDerivZAtNodeInv[i][j]*dis[1]+ShapeFuncDerivYAtNodeInv[i][j]*dis[2]);
				NodeStrain[i][5]+=(ShapeFuncDerivZAtNodeInv[i][j]*dis[0]+ShapeFuncDerivXAtNodeInv[i][j]*dis[2]);
			}
		}
	}
	else
	{
		for(int i=NbNodeInside;i<NbNode;i++)
		{
			int nbNeighbor=NeighborNodeIdx[i].size();
			for(int j=0;j<nbNeighbor;j++)
			{
				Vec3f dis;
				dis=NodeDis[NeighborNodeIdx[i][j]]-NodeDis[i];

				NodeStrain[i][0]+=ShapeFuncDerivXAtNodeInv[i][j]*dis[0];
				NodeStrain[i][1]+=ShapeFuncDerivYAtNodeInv[i][j]*dis[1];
				NodeStrain[i][2]+=ShapeFuncDerivZAtNodeInv[i][j]*dis[2];

				NodeStrain[i][3]+=(ShapeFuncDerivYAtNodeInv[i][j]*dis[0]+ShapeFuncDerivXAtNodeInv[i][j]*dis[1]);
				NodeStrain[i][4]+=(ShapeFuncDerivZAtNodeInv[i][j]*dis[1]+ShapeFuncDerivYAtNodeInv[i][j]*dis[2]);
				NodeStrain[i][5]+=(ShapeFuncDerivZAtNodeInv[i][j]*dis[0]+ShapeFuncDerivXAtNodeInv[i][j]*dis[2]);
			}
		}
	}

	// 2. green strain 
	/*for(int i=0;i<NbNode;i++)
	{
		int nbNeighbor=NeighborNodeIdx[i].size();
		double uxdx=0; double uydx=0; double uzdx=0;
		double uxdy=0; double uydy=0; double uzdy=0;
		double uxdz=0; double uydz=0; double uzdz=0;
		
		for(int j=0;j<nbNeighbor;j++)
		{
			Vec3f dis;
			dis=NodeDis[NeighborNodeIdx[i][j]]-NodeDis[i];

			uxdx+=ShapeFuncDerivXAtNodeInv[i][j]*dis[0];
			uydx+=ShapeFuncDerivXAtNodeInv[i][j]*dis[1];
			uzdx+=ShapeFuncDerivXAtNodeInv[i][j]*dis[2];

			uxdy+=ShapeFuncDerivYAtNodeInv[i][j]*dis[0];
			uydy+=ShapeFuncDerivYAtNodeInv[i][j]*dis[1];
			uzdy+=ShapeFuncDerivYAtNodeInv[i][j]*dis[2];

			uxdz+=ShapeFuncDerivZAtNodeInv[i][j]*dis[0];
			uydz+=ShapeFuncDerivZAtNodeInv[i][j]*dis[1];
			uzdz+=ShapeFuncDerivZAtNodeInv[i][j]*dis[2];
		}
		NodeStrain[i][0]=uxdx+(uxdx*uxdx+uydx*uydx+uzdx*uzdx)/2;
		NodeStrain[i][1]=uydy+(uxdy*uxdy+uydy*uydy+uzdy*uzdy)/2;
		NodeStrain[i][2]=uzdz+(uxdz*uxdz+uydz*uydz+uzdz*uzdz)/2;

		NodeStrain[i][3]=(uxdy+uydx+uxdx*uxdy+uydx*uydy+uzdx*uzdy)/2.0;
		NodeStrain[i][4]=(uydz+uzdy+uxdy*uxdz+uydy*uydz+uzdy*uzdz)/2.0;
		NodeStrain[i][5]=(uxdz+uzdx+uxdz*uxdx+uydz*uydx+uzdz*uzdx)/2.0;
	}*/
}
void EFG_CPU::computeStress()
{
	for(int i=NbNodeInside;i<NbNode;i++)
	{
		NodeStress[i]=MaterialStiffness*NodeStrain[i];
	}
}

void EFG_CPU::computeForce()
{
	if(COROTATION)
	{
		for(int i=0;i<NbNode;i++)
		{
			int nbNeighbor=NeighborNodeIdx[i].size();
			for(int j=0;j<nbNeighbor;j++)
			{
				int idx=NeighborNodeIdx[i][j];
				const float shapeFuncDrvX=ShapeFuncDerivXAtNode[i][j];
				const float shapeFuncDrvY=ShapeFuncDerivYAtNode[i][j];
				const float shapeFuncDrvZ=ShapeFuncDerivZAtNode[i][j];
				Vec6f stress=NodeStress[NeighborNodeIdx[i][j]];

				Vec3f force;
				force[0]=(shapeFuncDrvX*stress[0]+shapeFuncDrvY*stress[3]+shapeFuncDrvZ*stress[5])*NodeVolume[idx];
				force[1]=(shapeFuncDrvY*stress[1]+shapeFuncDrvX*stress[3]+shapeFuncDrvZ*stress[4])*NodeVolume[idx];
				force[2]=(shapeFuncDrvZ*stress[2]+shapeFuncDrvY*stress[4]+shapeFuncDrvX*stress[5])*NodeVolume[idx];
				NodeForce[i]-=NodeRot[idx]*force;
			}
		}
	}
	else
	{
		for(int i=NbNodeInside;i<NbNode;i++)
		{
			int nbNeighbor=NeighborNodeIdx[i].size();
			for(int j=0;j<nbNeighbor;j++)
			{
				Vec3f force;
				int idx=NeighborNodeIdx[i][j];
				const float shapeFuncDrvX=ShapeFuncDerivXAtNode[i][j];
				const float shapeFuncDrvY=ShapeFuncDerivYAtNode[i][j];
				const float shapeFuncDrvZ=ShapeFuncDerivZAtNode[i][j];
				Vec6f stress=NodeStress[NeighborNodeIdx[i][j]];

				force[0]-=(shapeFuncDrvX*stress[0]+shapeFuncDrvY*stress[3]+shapeFuncDrvZ*stress[5])*NodeVolume[idx];
				force[1]-=(shapeFuncDrvY*stress[1]+shapeFuncDrvX*stress[3]+shapeFuncDrvZ*stress[4])*NodeVolume[idx];
				force[2]-=(shapeFuncDrvZ*stress[2]+shapeFuncDrvY*stress[4]+shapeFuncDrvX*stress[5])*NodeVolume[idx];

				NodeForce[i]+=force;
			}
		}
	}
}

void EFG_CPU::updatePositionExplicit(float dt, int itter)
{
	for(int k=0;k<itter;k++)
	{
		for(int i=NbNodeInside;i<NbNode;i++)
		{
			NodeForce[i].clear();
		}

		//add internal force
		addInternalForce();

		//add gravity
		for(int i=NbNodeInside;i<NbNode;i++)
		{
			NodeForce[i][1]-=GRAVITY*NodeMass[i];
		}

		//add damping force
		for(int i=NbNodeInside;i<NbNode;i++)
		{
			NodeForce[i]-=NodeVel[i]*DAMPING;
		}

		//Explicit integration
		for(int i=NbNodeInside;i<NbNode;i++)
		{
			Vec3f dv=NodeForce[i]/NodeMass[i]*dt;
			NodeVel[i]+=dv;
			NodeDis[i]+=(NodeVel[i]*dt);
			NodePos[i]=NodePos0[i]+NodeDis[i];
		}

		//Fixed Constraint
		for(unsigned int i=0;i<FixedNodeIdx.size();i++)
		{
			NodePos[FixedNodeIdx[i]]=NodePos0[FixedNodeIdx[i]];
			NodeDis[FixedNodeIdx[i]].clear();
		}

		//Essential BC
		if(ConstrainedNodeIdx>0)
		{
			NodeDis[ConstrainedNodeIdx]=ConstrainedDis;
			NodePos[ConstrainedNodeIdx]=NodePos0[ConstrainedNodeIdx]+ConstrainedDis;
		}
	}
	BVHAABB.updateAABBTreeBottomUp();
}

void EFG_CPU::removeEdges(std::vector<int>& idx)
{
	VectorFunc vectorFunc;
	vectorFunc.arrangeVector(idx);

	removeEdgesInPhysicalModel(idx);
	removeEdgesInCollisionModel(idx);
}
void EFG_CPU::removeEdgesInPhysicalModel(std::vector<int>& idx)
{
	VectorFunc func;
	std::vector<int> updatedNodeIdx;

	// 1. update neighbor information
	for(int i=idx.size()-1;i>=0;i--)
	{
		// update neighbor information
		Vec2i edge=Edge[idx[i]];
		for(int j=0;j<2;j++)
		{
			updatedNodeIdx.push_back(edge[j]);	
			for(int k=0;k<NeighborNodeIdx[edge[j]].size();k++)
			{
				if(NeighborNodeIdx[edge[j]][k]==edge[(j+1)%2])
					func.removeElement(NeighborNodeIdx[edge[j]], k);
			}
		}
	}
	func.arrangeVector(updatedNodeIdx);

	// 2. update shape functions
	updateShapeFuncValue(updatedNodeIdx);
}
void EFG_CPU::removeEdgesInCollisionModel(std::vector<int>& idx)
{
	for(int i=idx.size()-1;i>=0;i--)
	{
		int edgeIdxEnd=Edge.size()-1;

		// 1. Update BVH
		AABBNode* node=BVHAABB.findLeafNode(idx[i]);
		if(node)
			BVHAABB.removeNode(node);
		node=BVHAABB.findLeafNode(edgeIdxEnd);
		if(node)
			node->IndexInLeafNode=idx[i];

		// 2. Remove edge
		Edge[idx[i]]=Edge[edgeIdxEnd];
		Edge.pop_back();
	}
}

void EFG_CPU::updateShapeFuncValue(std::vector<int>& updatedNodeIdx)
{
	//1. recompute moment matrix at updated node
	for(int i=0;i<updatedNodeIdx.size();i++)
	{
		ShapeFunc.computeMomentMatrixAtNode(MomentMatrixAtNode[updatedNodeIdx[i]], NeighborNodeIdx[updatedNodeIdx[i]], updatedNodeIdx[i]);
		invertMatrix(MomentMatrixInvAtNode[updatedNodeIdx[i]], MomentMatrixAtNode[updatedNodeIdx[i]]);
	}

	//3. Compute moment matrix drv at node
	for(int i=0;i<updatedNodeIdx.size();i++)
	{
		ShapeFunc.computeMomentMatrixDrvXAtNode(updatedNodeIdx[i], MomentMatrixDrvXAtNode[updatedNodeIdx[i]], NeighborNodeIdx[updatedNodeIdx[i]]);
		ShapeFunc.computeMomentMatrixDrvYAtNode(updatedNodeIdx[i], MomentMatrixDrvYAtNode[updatedNodeIdx[i]], NeighborNodeIdx[updatedNodeIdx[i]]);
		ShapeFunc.computeMomentMatrixDrvZAtNode(updatedNodeIdx[i], MomentMatrixDrvZAtNode[updatedNodeIdx[i]], NeighborNodeIdx[updatedNodeIdx[i]]);
	}

	//4. Compute shape function drv at node
	for(int i=0;i<updatedNodeIdx.size();i++)
	{
		int idx=updatedNodeIdx[i];
		int nbNeighbor=NeighborNodeIdx[idx].size();
		for(unsigned int j=0;j<nbNeighbor;j++)
		{
			int neiIdx=NeighborNodeIdx[idx][j];
			int idxInv;
			for(int k=0;k<NeighborNodeIdx[neiIdx].size();k++)
			{
				if(NeighborNodeIdx[neiIdx][k]==idx)
				{
					idxInv=k;
					break;
				}
			}
			ShapeFuncAtNode[idx][j]=ShapeFunc.computeShapeFuncValue(idx, NodePos0[neiIdx], MomentMatrixInvAtNode[neiIdx]);
			ShapeFuncAtNode[neiIdx][idxInv]=ShapeFunc.computeShapeFuncValue(neiIdx, NodePos0[idx], MomentMatrixInvAtNode[idx]);

			ShapeFuncDerivXAtNode[idx][j]=ShapeFunc.computeShapeFuncDrvX(idx, NodePos0[neiIdx], MomentMatrixInvAtNode[neiIdx], MomentMatrixDrvXAtNode[neiIdx]);
			ShapeFuncDerivYAtNode[idx][j]=ShapeFunc.computeShapeFuncDrvY(idx, NodePos0[neiIdx], MomentMatrixInvAtNode[neiIdx], MomentMatrixDrvYAtNode[neiIdx]);
			ShapeFuncDerivZAtNode[idx][j]=ShapeFunc.computeShapeFuncDrvZ(idx, NodePos0[neiIdx], MomentMatrixInvAtNode[neiIdx], MomentMatrixDrvZAtNode[neiIdx]);
			ShapeFuncDerivXAtNode[neiIdx][idxInv]=ShapeFunc.computeShapeFuncDrvX(neiIdx, NodePos0[idx], MomentMatrixInvAtNode[idx], MomentMatrixDrvXAtNode[idx]);
			ShapeFuncDerivYAtNode[neiIdx][idxInv]=ShapeFunc.computeShapeFuncDrvY(neiIdx, NodePos0[idx], MomentMatrixInvAtNode[idx], MomentMatrixDrvYAtNode[idx]);
			ShapeFuncDerivZAtNode[neiIdx][idxInv]=ShapeFunc.computeShapeFuncDrvZ(neiIdx, NodePos0[idx], MomentMatrixInvAtNode[idx], MomentMatrixDrvZAtNode[idx]);

			ShapeFuncDerivXAtNodeInv[idx][j]=ShapeFuncDerivXAtNode[neiIdx][idxInv];
			ShapeFuncDerivYAtNodeInv[idx][j]=ShapeFuncDerivYAtNode[neiIdx][idxInv];
			ShapeFuncDerivZAtNodeInv[idx][j]=ShapeFuncDerivZAtNode[neiIdx][idxInv];
			ShapeFuncDerivXAtNodeInv[neiIdx][idxInv]=ShapeFuncDerivXAtNode[idx][j];
			ShapeFuncDerivYAtNodeInv[neiIdx][idxInv]=ShapeFuncDerivYAtNode[idx][j];
			ShapeFuncDerivZAtNodeInv[neiIdx][idxInv]=ShapeFuncDerivZAtNode[idx][j];
		}
	}
}

std::vector<Vec3f>* EFG_CPU::nodePos0()
{
	return &NodePos0;
}
std::vector<Vec3f>* EFG_CPU::nodePos()
{
	return &NodePos;
}
std::vector<Vec3f>* EFG_CPU::nodeVel()
{
	return &NodeVel;
}
std::vector<Vec3f>* EFG_CPU::nodeForce()
{
	return &NodeForce;
}
std::vector<Vec3f>* EFG_CPU::nodeDis()
{
	return &NodeDis;
}
float EFG_CPU::supportRadius()
{
	return SupportRadius;
}

std::vector<float>* EFG_CPU::nodeMass()
{
	return &NodeMass;
}

int EFG_CPU::nbNode()
{
	return NodePos.size();
}

int EFG_CPU::nbNodeInside()
{
	return NbNodeInside;
}
std::vector<Vec2i>* EFG_CPU::edge()
{
	return &Edge;
}

void EFG_CPU::assembleGlobalStiffness()
{
	std::vector<std::vector<Mat<6,3,float>>> P; P.resize(NodePos.size());
	std::vector<std::vector<Mat<3,6,float>>> PT; PT.resize(NodePos.size());
	std::vector<std::vector<Mat<3,6,float>>> PTD; PTD.resize(NodePos.size());
	
	for(int i=0;i<NodePos.size();i++)
	{
		for(int j=0;j<NeighborNodeIdx[i].size();j++)
		{
			Mat<6,3,float> p;
			Mat<3,6,float> pt;
			Mat<3,6,float> ptd;
			p[0][0]=ShapeFuncDerivXAtNode[i][j];
			p[1][1]=ShapeFuncDerivYAtNode[i][j];
			p[2][2]=ShapeFuncDerivZAtNode[i][j];
			p[3][0]=ShapeFuncDerivYAtNode[i][j];
			p[3][1]=ShapeFuncDerivXAtNode[i][j];
			p[4][1]=ShapeFuncDerivZAtNode[i][j];
			p[4][2]=ShapeFuncDerivYAtNode[i][j];
			p[5][0]=ShapeFuncDerivZAtNode[i][j];
			p[5][2]=ShapeFuncDerivXAtNode[i][j];
			pt=p.transposed();
			ptd=pt*MaterialStiffness;

			P[i].push_back(p);
			PT[i].push_back(pt);
			PTD[i].push_back(ptd);
		}
	}

	if(GlobalStiffness)
		delete GlobalStiffness;

	GlobalStiffness=new Matrix(NodePos.size()*3,NodePos.size()*3);
	for(int i=0;i<NodePos.size();i++)
	{
		for(int j=0;j<NodePos.size();j++)
		{
			Mat3x3f PTDP;
			for(int k=0;k<NeighborNodeIdx[i].size();k++)
			{
				int idx=NeighborNodeIdx[i][k];
				for(int l=0;l<NeighborNodeIdx[j].size();l++)
				{
					if(idx==NeighborNodeIdx[j][l])
					{
						PTDP+=PTD[i][k]*P[j][l]*NodeVolume[i];
						break;
					}
				}
			}
			
			int offsetR=i*3;
			int offsetC=j*3;
			for(int k=0;k<3;k++)
			{
				for(int l=0;l<3;l++)
					(*GlobalStiffness)(offsetR+k,offsetC+l)=PTDP[k][l];
			}
		}
	}
}

void EFG_CPU::writeGlobalStiffnessMatrix(char* filename)
{
	FILE *fp;
	fp = fopen(filename,"w");

	for(int i=0;i<GlobalStiffness->row();i++)
	{
		for(int j=0;j<GlobalStiffness->column();j++)
			fprintf(fp,"%f ",(*GlobalStiffness)(i,j));
		fprintf(fp,"\n");
	}
}

int EFG_CPU::nbNeighborNodeMax()
{
	int max=-100000000;
	for(int i=0;i<NodePos.size();i++)
	{
		int n=NeighborNodeIdx[i].size();
		if(max<n)
			max=n;
	}
	return max;
}

int EFG_CPU::nbNeighborNodeMin()
{
	int min=100000000;
	for(int i=0;i<NodePos.size();i++)
	{
		if(min>NeighborNodeIdx[i].size())
			min=NeighborNodeIdx[i].size();
	}
	return min;
}

float EFG_CPU::nbNeighborNodeAve()
{
	float ave=0;
	for(int i=0;i<NodePos.size();i++)
	{
		ave+=NeighborNodeIdx[i].size();
	}
	ave/=(float)NodePos.size();
	return ave;
}

void EFG_CPU::rot(Mat3x3f mat)
{
	for(int i=0;i<NodePos.size();i++)
	{
		NodePos[i]=mat*NodePos0[i];
	}
}

void EFG_CPU::updatePositionAtNodeCoord()
{
	for(int i=0;i<NbNode;i++)
	{
		Vec3f disX; Vec3f disY; Vec3f disZ;
		for(int j=0;j<NeighborNodeIdx[i].size();j++)
		{
			disX+=NodeDis[NeighborNodeIdx[i][j]]*ShapeFuncAtNodeX[i][j];
			disY+=NodeDis[NeighborNodeIdx[i][j]]*ShapeFuncAtNodeY[i][j];
			disZ+=NodeDis[NeighborNodeIdx[i][j]]*ShapeFuncAtNodeZ[i][j];
		}
		NodePosX[i]=NodePos0[i]+X+disX;
		NodePosY[i]=NodePos0[i]+Y+disY;
		NodePosZ[i]=NodePos0[i]+Z+disZ;
	}
}

MLSShapeFunc* EFG_CPU::shapeFunc()
{
	return &ShapeFunc;
}

AABBTreeEdge* EFG_CPU::getBVH()
{
	return &BVHAABB;
}