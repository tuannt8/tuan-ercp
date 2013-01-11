#include "stdafx.h"
#include "MeshfreeNodeGenerator.h"

MeshfreeNodeGenerator::MeshfreeNodeGenerator(void)
{
}

MeshfreeNodeGenerator::~MeshfreeNodeGenerator(void)
{
}

void MeshfreeNodeGenerator::setSurfObj(SurfaceObj* obj)
{
	Obj=obj;
}

void MeshfreeNodeGenerator::generateUniformNode(int res)
{
	// 1. Bounding box generation
	generateBoundingBox(Obj->point());

	// 2. Set resolution
	Vec3f box=(RightUpPoint-LeftDownPoint);
	float min=100000000;
	int idx=-1;
	for(int i=0;i<3;i++)
	{
		if(box[i]<min)
		{
			min=box[i];
			idx=i;
		}
	}
	Vec3i _res;
	float length=box[idx]/(float)res;
	for(int i=0;i<3;i++)
	{
		if(!(i==idx))
			_res[i]=box[i]/length;
	}
	_res[idx]=res;
	
	//3. Node generation
	generateNodeInBoundingBox(_res);

	//4. Compute node volume
	computeNodeVolume();

	//5. Remove nodes outside object
	removeNodeOutsideObject();
}

void MeshfreeNodeGenerator::generateUniformNodeWithStressPoint(int res)
{
	// 1. Bounding box generation
	generateBoundingBox(Obj->point());

	// 2. Set resolution
	Vec3f box=(RightUpPoint-LeftDownPoint);
	float min=100000000;
	int idx=-1;
	for(int i=0;i<3;i++)
	{
		if(box[i]<min)
		{
			min=box[i];
			idx=i;
		}
	}
	Vec3i _res;
	float length=box[idx]/(float)res;
	for(int i=0;i<3;i++)
	{
		if(!(i==idx))
			_res[i]=box[i]/length;
	}
	_res[idx]=res;

	//3. Node generation
	generateNodeInBoundingBoxWithStressPoint(_res);

	//4. Compute node volume
	computeNodeVolumeWithStressPoint();

	//5. Remove nodes outside object
	removeNodeOutsideObjectWithStressPoint();
}

void MeshfreeNodeGenerator::generateNodeInBoundingBox(Vec3i res)
{
	Vec3f ds;
	for(int i=0;i<3;i++)
		ds[i]=(RightUpPoint[i]-LeftDownPoint[i])/float(res[i]);

	for(int i=0;i<=res[1];i++)
	{
		for(int j=0;j<=res[0];j++)
		{
			for(int k=0;k<=res[2];k++)
			{
				Vec3d pos=LeftDownPoint+Vec3d(ds[0]*j,ds[1]*i,ds[2]*k);
				NodePos.push_back(pos+ds/2.0);
			}
		}
	}
	BoxSize=ds;
}

void MeshfreeNodeGenerator::generateNodeInBoundingBoxWithStressPoint(Vec3i res)
{
	Vec3f ds;
	for(int i=0;i<3;i++)
		ds[i]=(RightUpPoint[i]-LeftDownPoint[i])/float(res[i]);

	for(int i=0;i<=(res[1]+1);i++)
	{
		for(int j=0;j<=(res[0]+1);j++)
		{
			for(int k=0;k<=(res[2]+1);k++)
			{
				Vec3f pos=LeftDownPoint+Vec3d(ds[0]*j,ds[1]*i,ds[2]*k);
				if(!(i==(res[1]+1)))
				{
					if(!(j==(res[0]+1)))
					{
						if(!(k==(res[2]+1)))
							NodePos.push_back(pos+ds/2.0);
					}
				}
				StressPoint.push_back(pos);
			}
		}
	}
	BoxSize=ds;
}

void MeshfreeNodeGenerator::generateBoundingBox(std::vector<Vec3f>* point)
{
	float offset=1.001;
	Vec3d minPoint(MAX, MAX, MAX);
	Vec3d maxPoint(MIN, MIN, MIN);

	for(int i=0;i<point->size();i++)
	{
		for(int j=0;j<3;j++)
		{
			if((*point)[i][j]<minPoint[j])
				minPoint[j]=(*point)[i][j];
			if((*point)[i][j]>maxPoint[j])
				maxPoint[j]=(*point)[i][j];
		}
	}
	LeftDownPoint=minPoint-Vec3d(offset,offset,offset);
	RightUpPoint=maxPoint+Vec3d(offset,offset,offset);
}

void MeshfreeNodeGenerator::removeNodeOutsideObject()
{
	if(!Obj->getBVH())
		Obj->constructAABBTree();
	Vec3i direc(0,0,1);
	CollisionManager collision;
	for(int i=0;i<NodePos.size();i++)
	{
		if(!collision.isPointInSurfObj(Obj, NodePos[i]))
		{
			NodePos[i]=NodePos[NodePos.size()-1];
			NodePos.pop_back();
			i--;
		}
	}
}

void MeshfreeNodeGenerator::removeNodeOutsideObjectWithStressPoint()
{
	if(!Obj->getBVH())
		Obj->constructAABBTree();
	Vec3i direc(0,0,1);
	CollisionManager collision;
	for(int i=0;i<NodePos.size();i++)
	{
		if(!collision.isPointInSurfObj(Obj, NodePos[i]))
		{
			NodePos[i]=NodePos[NodePos.size()-1];
			NodePos.pop_back();
			i--;
		}
	}

	for(int i=0;i<StressPoint.size();i++)
	{
		if(!collision.isPointInSurfObj(Obj, StressPoint[i]))
		{
			StressPoint[i]=StressPoint[StressPoint.size()-1];
			StressPoint.pop_back();
			i--;
		}
	}
}

void MeshfreeNodeGenerator::computeNodeVolume()
{
	float volume=(RightUpPoint[0]-LeftDownPoint[0])*(RightUpPoint[1]-LeftDownPoint[1])*(RightUpPoint[2]-LeftDownPoint[2]);
	int nbNode=NodePos.size();
	NodeVolume=volume/(float)nbNode;
}

void MeshfreeNodeGenerator::computeNodeVolumeWithStressPoint()
{
	float volume=(RightUpPoint[0]-LeftDownPoint[0])*(RightUpPoint[1]-LeftDownPoint[1])*(RightUpPoint[2]-LeftDownPoint[2]);
	int nbNode=NodePos.size()+StressPoint.size();
	NodeVolume=volume/(float)nbNode;
}

int MeshfreeNodeGenerator::findNodeNearSurface(float radius)
{
	VectorFunc func;
	std::vector<Vec3f> nodeNearSurf;
	std::vector<Vec3f>* pos=Obj->point();
	for(int i=0;i<Obj->nbPoint();i++)
	{
		for(int j=0;j<NodePos.size();j++)
		{
			if(((*pos)[i]-NodePos[j]).norm()<radius)
			{
				nodeNearSurf.push_back(NodePos[j]);
				func.removeElement(NodePos, j);
				j--;
			}
		}
	}
	int nbNodeInside=NodePos.size();
	for(int i=0;i<nodeNearSurf.size();i++)
		NodePos.push_back(nodeNearSurf[i]);
	return nbNodeInside;
}