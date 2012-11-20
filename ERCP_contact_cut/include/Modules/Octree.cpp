#include "stdafx.h"
#include "Octree.h"

OctreeNode::OctreeNode(void)
{
	End=false;
	for(int i=0;i<8;i++)
		Descendant[i]=NULL;
	NodeIdx=-1;
}

OctreeNode::~OctreeNode(void)
{
	for(int i=0;i<8;i++)
	{
		if(Descendant[i])
			delete Descendant[i];
	}
}

void OctreeNode::drawBoundingBox()
{
	glBegin(GL_LINES);
	glVertex3f((GLfloat)LeftDown[0], (GLfloat)LeftDown[1], (GLfloat)LeftDown[2]);
	glVertex3f((GLfloat)RightUp[0], (GLfloat)LeftDown[1], (GLfloat)LeftDown[2]);
	glVertex3f((GLfloat)RightUp[0], (GLfloat)LeftDown[1], (GLfloat)LeftDown[2]);
	glVertex3f((GLfloat)RightUp[0], (GLfloat)LeftDown[1], (GLfloat)RightUp[2]);
	glVertex3f((GLfloat)RightUp[0], (GLfloat)LeftDown[1], (GLfloat)RightUp[2]);
	glVertex3f((GLfloat)LeftDown[0], (GLfloat)LeftDown[1], (GLfloat)RightUp[2]);
	glVertex3f((GLfloat)LeftDown[0], (GLfloat)LeftDown[1], (GLfloat)RightUp[2]);
	glVertex3f((GLfloat)LeftDown[0], (GLfloat)LeftDown[1], (GLfloat)LeftDown[2]);

	glVertex3f((GLfloat)LeftDown[0], (GLfloat)RightUp[1], (GLfloat)LeftDown[2]);
	glVertex3f((GLfloat)RightUp[0], (GLfloat)RightUp[1], (GLfloat)LeftDown[2]);
	glVertex3f((GLfloat)RightUp[0], (GLfloat)RightUp[1], (GLfloat)LeftDown[2]);
	glVertex3f((GLfloat)RightUp[0], (GLfloat)RightUp[1], (GLfloat)RightUp[2]);
	glVertex3f((GLfloat)RightUp[0], (GLfloat)RightUp[1], (GLfloat)RightUp[2]);
	glVertex3f((GLfloat)LeftDown[0], (GLfloat)RightUp[1], (GLfloat)RightUp[2]);
	glVertex3f((GLfloat)LeftDown[0], (GLfloat)RightUp[1], (GLfloat)RightUp[2]);
	glVertex3f((GLfloat)LeftDown[0], (GLfloat)RightUp[1], (GLfloat)LeftDown[2]);

	glVertex3f((GLfloat)LeftDown[0], (GLfloat)LeftDown[1], (GLfloat)LeftDown[2]);
	glVertex3f((GLfloat)LeftDown[0], (GLfloat)RightUp[1], (GLfloat)LeftDown[2]);
	glVertex3f((GLfloat)RightUp[0], (GLfloat)LeftDown[1], (GLfloat)RightUp[2]);
	glVertex3f((GLfloat)RightUp[0], (GLfloat)RightUp[1], (GLfloat)RightUp[2]);
	glVertex3f((GLfloat)LeftDown[0], (GLfloat)LeftDown[1], (GLfloat)RightUp[2]);
	glVertex3f((GLfloat)LeftDown[0], (GLfloat)RightUp[1], (GLfloat)RightUp[2]);
	glVertex3f((GLfloat)RightUp[0], (GLfloat)LeftDown[1], (GLfloat)LeftDown[2]);
	glVertex3f((GLfloat)RightUp[0], (GLfloat)RightUp[1], (GLfloat)LeftDown[2]);
	glEnd();
}

void OctreeNode::setBoundingBox(Vec3f& leftDown, Vec3f& rightUp)
{
	LeftDown=leftDown;
	RightUp=rightUp;
}

Octree::Octree(void)
{
	Root=NULL;
}

Octree::Octree(std::vector<Vec3f>* node, int nbNode)
{
	Root=NULL;
	Node=node;
	NbNode=nbNode;
	Mode=OCTREE_NORMAL;
}

Octree::Octree(std::vector<Vec3f>* node, int nbNode, int mode)
{
	Root=NULL;
	Node=node;
	NbNode=nbNode;
	Mode=mode;
}

Octree::~Octree(void)
{
	if(Root)
		delete Root;
}

void Octree::constructOctree(int depth)
{
	//1. bounding box generation
	Vec3f leftDown(MAX, MAX, MAX);
	Vec3f rightUp(MIN, MIN, MIN);
	Vec3f mid;

	for(int i=0;i<NbNode;i++)
	{
		for(int j=0;j<3;j++)
		{
			if(leftDown[j]>(*Node)[i][j])
				leftDown[j]=(*Node)[i][j];
			if(rightUp[j]<(*Node)[i][j])
				rightUp[j]=(*Node)[i][j];
		}
	}
	mid=(rightUp+leftDown)/2.0;

	//2. Root node
	Root=new OctreeNode;
	Root->setBoundingBox(leftDown, rightUp);
	Root->Depth=0;

	//3. Descendant
	OctreeNode** descendant=new OctreeNode*[8];
	for(int i=0;i<8;i++)
	{
		descendant[i]=new OctreeNode;
		descendant[i]->Depth=Root->Depth+1;
		Root->Descendant[i]=descendant[i];
	}

	//4. Descendant generation
	std::vector<int> nodeIdx[8];
	for(int i=0;i<NbNode;i++)
	{
		if((*Node)[i][0]>mid[0])
		{
			if((*Node)[i][1]>mid[1])
			{
				if((*Node)[i][2]>mid[2])
				{
					//RightTopFront
					nodeIdx[0].push_back(i);
				}
				else
				{
					//RightTopBehind
					nodeIdx[1].push_back(i);
				}
			}
			else
			{
				if((*Node)[i][2]>mid[2])
				{
					//RightBottomFront
					nodeIdx[2].push_back(i);
				}
				else
				{
					//RightBottomBehind
					nodeIdx[3].push_back(i);
				}
			}
		}
		else
		{
			if((*Node)[i][1]>mid[1])
			{
				if((*Node)[i][2]>mid[2])
				{
					//LeftTopFront
					nodeIdx[4].push_back(i);
				}
				else
				{
					//LeftTopBehind
					nodeIdx[5].push_back(i);
				}
			}
			else
			{
				if((*Node)[i][2]>mid[2])
				{
					//LeftBottomFront
					nodeIdx[6].push_back(i);
				}
				else
				{
					//LeftBottomBehind
					nodeIdx[7].push_back(i);
				}
			}
		}
	}

	//RightTopFront
	Vec3f p1=mid;
	Vec3f p2=rightUp;
	genOctree(descendant[0], p1, p2, nodeIdx[0], depth);  

	//RightTopBehind
	p1=Vec3f(mid[0], mid[1], leftDown[2]);
	p2=Vec3f(rightUp[0], rightUp[1], mid[2]);
	genOctree(descendant[1], p1, p2, nodeIdx[1], depth);  

	//RightBottomFront
	p1=Vec3f(mid[0], leftDown[1], mid[2]);
	p2=Vec3f(rightUp[0], mid[1], rightUp[2]) ;
	genOctree(descendant[2], p1, p2, nodeIdx[2], depth);

	//RightBottomBehind
	p1=Vec3f(mid[0],leftDown[1],leftDown[2]);
	p2=Vec3f(rightUp[0], mid[1], mid[2]);
	genOctree(descendant[3], p1, p2, nodeIdx[3], depth);

	//LeftTopFront
	p1=Vec3d(leftDown[0], mid[1], mid[2]);
	p2=Vec3d(mid[0], rightUp[1], rightUp[2]);
	genOctree(descendant[4], p1, p2, nodeIdx[4], depth);

	//LeftTopBehind
	p1=Vec3f(leftDown[0], mid[1], leftDown[2]);
	p2=Vec3f(mid[0], rightUp[1], mid[2]);
	genOctree(descendant[5], p1, p2, nodeIdx[5], depth);

	//LeftBottomFront
	p1=Vec3f(leftDown[0], leftDown[1], mid[2]);
	p2=Vec3f(mid[0], mid[1], rightUp[2]);
	genOctree(descendant[6], p1, p2, nodeIdx[6], depth);

	//LeftBottomBehind
	p1=leftDown;
	p2=(leftDown+rightUp)/2.0;
	genOctree(descendant[7], p1, p2, nodeIdx[7], depth);

	delete [] descendant;
}

int Octree::genOctree(OctreeNode* root, Vec3f& leftDown, Vec3f& rightUp, std::vector<int>& nodeIdx, int maxDepth)
{
	//1. Set bounding box
	root->setBoundingBox(leftDown, rightUp);

	//2. Stop if there is only one node in the box
	if(Mode==OCTREE_NORMAL)
	{
		if(nodeIdx.size()==0)
		{
			root->End=true;
			return 0;
		}
		if(root->Depth==maxDepth)
		{
			root->End=true;
			return 0;
		}
	}
	if(Mode==OCTREE_UNIFORM)
	{
		if(root->Depth==maxDepth)
		{
			root->End=true;
			return 0;
		}
	}
	if(Mode==OCTREE_MULTILEVEL)
	{
		if(root->Depth==maxDepth)
		{
			if(nodeIdx.size()==0)
			{
				root->End=true;
				return 0;
			}
		}
		if(root->Depth==(maxDepth+1))
		{
			root->End=true;
			return 0;
		}
	}

	//3. Descendant
	OctreeNode** descendant=new OctreeNode*[8];
	for(int i=0;i<8;i++)
	{
		descendant[i]=new OctreeNode;
		descendant[i]->Depth=root->Depth+1;
		root->Descendant[i]=descendant[i];
		if(nodeIdx.size()>0)
			descendant[i]->NodeIdx=1;
	}

	//4. Descendant generation
	Vec3f mid=(leftDown+rightUp)/2.0;
	std::vector<int> _nodeIdx[8];
	for(int i=0;i<nodeIdx.size();i++)
	{
		if((*Node)[nodeIdx[i]][0]>mid[0])
		{
			if((*Node)[nodeIdx[i]][1]>mid[1])
			{
				if((*Node)[nodeIdx[i]][2]>mid[2])
				{
					//RightTopFront
					_nodeIdx[0].push_back(nodeIdx[i]);
				}
				else
				{
					//RightTopBehind
					_nodeIdx[1].push_back(nodeIdx[i]);
				}
			}
			else
			{
				if((*Node)[nodeIdx[i]][2]>mid[2])
				{
					//RightBottomFront
					_nodeIdx[2].push_back(nodeIdx[i]);
				}
				else
				{
					//RightBottomBehind
					_nodeIdx[3].push_back(nodeIdx[i]);
				}
			}
		}
		else
		{
			if((*Node)[nodeIdx[i]][1]>mid[1])
			{
				if((*Node)[nodeIdx[i]][2]>mid[2])
				{
					//LeftTopFront
					_nodeIdx[4].push_back(nodeIdx[i]);
				}
				else
				{
					//LeftTopBehind
					_nodeIdx[5].push_back(nodeIdx[i]);
				}
			}
			else
			{
				if((*Node)[nodeIdx[i]][2]>mid[2])
				{
					//LeftBottomFront
					_nodeIdx[6].push_back(nodeIdx[i]);
				}
				else
				{
					//LeftBottomBehind
					_nodeIdx[7].push_back(nodeIdx[i]);
				}
			}
		}
	}

	Vec3f p1;
	Vec3f p2;

	//RightTopFront
	p1=mid;
	p2=rightUp;
	genOctree(descendant[0], p1, p2, _nodeIdx[0], maxDepth);  

	//RightTopBehind
	p1=Vec3f(mid[0], mid[1], leftDown[2]);
	p2=Vec3f(rightUp[0], rightUp[1], mid[2]);
	genOctree(descendant[1], p1, p2, _nodeIdx[1], maxDepth);  

	//RightBottomFront
	p1=Vec3f(mid[0], leftDown[1], mid[2]);
	p2=Vec3f(rightUp[0], mid[1], rightUp[2]) ;
	genOctree(descendant[2], p1, p2, _nodeIdx[2], maxDepth);

	//RightBottomBehind
	p1=Vec3f(mid[0],leftDown[1],leftDown[2]);
	p2=Vec3f(rightUp[0], mid[1], mid[2]);
	genOctree(descendant[3], p1, p2, _nodeIdx[3], maxDepth);

	//LeftTopFront
	p1=Vec3d(leftDown[0], mid[1], mid[2]);
	p2=Vec3d(mid[0], rightUp[1], rightUp[2]);
	genOctree(descendant[4], p1, p2, _nodeIdx[4], maxDepth);

	//LeftTopBehind
	p1=Vec3f(leftDown[0], mid[1], leftDown[2]);
	p2=Vec3f(mid[0], rightUp[1], mid[2]);
	genOctree(descendant[5], p1, p2, _nodeIdx[5], maxDepth);

	//LeftBottomFront
	p1=Vec3f(leftDown[0], leftDown[1], mid[2]);
	p2=Vec3f(mid[0], mid[1], rightUp[2]);
	genOctree(descendant[6], p1, p2, _nodeIdx[6], maxDepth);

	//LeftBottomBehind
	p1=leftDown;
	p2=mid;
	genOctree(descendant[7], p1, p2, _nodeIdx[7], maxDepth);

	delete [] descendant;
	return 0;
}

void Octree::drawBoundingBox(int depth)
{
	drawBoundingBox(Root, depth);
}

void Octree::drawBoundingBox(OctreeNode* root, int depth)
{
	if(root)
		root->drawBoundingBox();
	if(root->End)
		return;
	if(depth==0)
		return;
	else
	{
		depth--;
		for(int i=0;i<8;i++)
			drawBoundingBox(root->Descendant[i], depth);
	}
}

void Octree::drawBoundingBox()
{
	drawBoundingBox(Root);
}

void Octree::drawBoundingBox(OctreeNode* root)
{
	if(root)
		root->drawBoundingBox();
	if(root->End)
		return;
	for(int i=0;i<8;i++)
		drawBoundingBox(root->Descendant[i]);
}