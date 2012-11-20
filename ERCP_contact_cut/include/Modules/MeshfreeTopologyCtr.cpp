#include "stdafx.h"
#include "MeshfreeTopologyCtr.h"

MeshfreeTopologyCtr::MeshfreeTopologyCtr(void)
{
}

MeshfreeTopologyCtr::~MeshfreeTopologyCtr(void)
{
	for(int i=0;i<NodeCtr.size();i++)
		delete NodeCtr[i];
}

void MeshfreeTopologyCtr::init(SurfaceObj* obj, std::vector<Vec3f>* nodePos, Vec3f boxSize)
{
	// 1. set surface object
	SurfObj=obj;

	// 2. set node position
	for(int i=0;i<nodePos->size();i++)
		NodePos.push_back((*nodePos)[i]);

	// 3. set meshfree node information
	NodeCtr.resize(nodePos->size());
	for(int i=0;i<nodePos->size();i++)
	{
		NodeCtr[i]=new MeshfreeNode(NULL, NodePos[i], boxSize/2.0);
		NodeCtr[i]->setNodeIdx(i);
		NodeCtr[i]->generateChildNode(obj);
	}

	// 4. compute support radius
	float max=-10000;
	for(int i=0;i<3;i++)
	{
		if(boxSize[i]>max)
			max=boxSize[i];
	}
	SupportRadius=max*1.01;

	// 5. init neighbor information
	initNeighbor();

	// 6. neighbor node의 수가 7개 이상이 되도록 한다
	std::vector<int> illegalNodeIdx;
	for(int i=0;i<NeighborNodeIdx.size();i++)
	{
		if(NeighborNodeIdx[i].size()<7)
			illegalNodeIdx.push_back(i);
	}
	count=0;
	if(illegalNodeIdx.size()>0)
		legalizeNeighbor(illegalNodeIdx);
}

void MeshfreeTopologyCtr::initNeighbor()
{
	NeighborNodeIdx.clear();
	NeighborNodeIdx.resize(NodePos.size());

	for(int i=0;i<NodePos.size();i++)
	{
		for(int j=0;j<NodePos.size();j++)
		{
			if((NodePos[i]-NodePos[j]).norm()<SupportRadius)
			{
				NeighborNodeIdx[i].push_back(j);
			}
		}
	}	
}

void MeshfreeTopologyCtr::legalizeNeighbor(std::vector<int>& illegalNodeIdx)
{	
	// 1. Child node를 생성한다
	std::vector<MeshfreeNode*> childPtr;

	for(int i=0;i<illegalNodeIdx.size();i++)
	{
		int idx=illegalNodeIdx[i];
		std::vector<MeshfreeNode*> activeNode;
		if(NodeCtr[idx]->getActiveChildNode(activeNode))
		{
			for(int j=0;j<activeNode.size();j++)
				childPtr.push_back(activeNode[j]);
		}
		else
			NodeCtr[idx]->generateChildNode(SurfObj);
	}

	// 3. 자식노드의 neighbor를 찾는다
	int nbIlligalNodeMax=-1;
	MeshfreeNode* addedNodePtr;
	for(int i=0;i<childPtr.size();i++)
	{
		std::vector<int> neighbor;
		childPtr[i]->findNeighborNode(NeighborNodeIdx, neighbor, &NodePos, SupportRadius);
		int nbIlligalNode=0;
		for(int j=0;j<neighbor.size();j++)
		{
			int idx=neighbor[j];
			if(NeighborNodeIdx[idx].size()<7)
				nbIlligalNode++;
		}
		if(nbIlligalNode>nbIlligalNodeMax)
		{
			nbIlligalNodeMax=nbIlligalNode;
			addedNodePtr=childPtr[i];
		}
	}

	// 4. Node를 add한다
	NodePos.push_back(addedNodePtr->nodePos());
	addedNodePtr->setNodeIdx(NodePos.size()-1);

	// 5. Neighbor를 update한다
	std::vector<int> neighbor;
	addedNodePtr->findNeighborNode(NeighborNodeIdx, neighbor, &NodePos, SupportRadius);

	VectorFunc func;
	
	// 6. Illigal node index를 update한다
	for(int i=0;i<illegalNodeIdx.size();i++)
	{
		int idx=illegalNodeIdx[i];
		if(NeighborNodeIdx[idx].size()>=7)
		{
			func.removeElement(illegalNodeIdx, i);
			i--;
		}
	}

	count++;

	if(count>100)
		return;

	if(illegalNodeIdx.empty())
		return;
	else
		legalizeNeighbor(illegalNodeIdx);
}

void MeshfreeTopologyCtr::glPrint(const char* text)
{
	glPushAttrib(GL_LIST_BIT);                        
	glListBase(1 - 32);                             
	glCallLists(strlen(text), GL_UNSIGNED_BYTE, text); 
	glPopAttrib();  
}

void MeshfreeTopologyCtr::printNbNeighbor()
{
	glColor3f(1,0,0);
	for(int i=0;i<NeighborNodeIdx.size();i++)
	{
		CString idx;
		char index[10];
		idx.Format(_T("%d"),NeighborNodeIdx.size());
		glPushMatrix();
		Vec3f nodePos=NodePos[i];
		glRasterPos3f(nodePos[0],nodePos[1],nodePos[2]);
		itoa(NeighborNodeIdx[i].size(),index,10);
		glPrint(index);
		glPopMatrix();
	}
}

void MeshfreeTopologyCtr::drawNodeBox(Vec3f color, float radius)
{
	for(int i=0;i<NodeCtr.size();i++)
	{
		NodeCtr[i]->drawNode(color, radius);
		NodeCtr[i]->drawActiveChildNode(Vec3f(0,0,1),radius);
	}
}

MeshfreeNode::MeshfreeNode(MeshfreeNode* parent, Vec3f nodePos, Vec3f boxSize)
{
	Parent=parent;
	for(int i=0;i<8;i++)
		Child[i]=NULL;

	NodePos=nodePos;
	LeftDown=nodePos-boxSize;
	RightUp=nodePos+boxSize;

	Volume=0;
	NodeIdx=-1;
	Active=true;
}

MeshfreeNode::~MeshfreeNode()
{
	for(int i=0;i<8;i++)
	{
		if(Child[i])
			delete Child[i];
	}
}

// Node idx가 0보다 크면 node로 할당된것을 의미
void MeshfreeNode::setNodeIdx(int idx)
{
	NodeIdx=idx;
}

void MeshfreeNode::drawNode(Vec3f color, float radius)
{
	GLUquadricObj *qobj = 0;
	qobj = gluNewQuadric();

	glPushMatrix();
	glTranslatef(NodePos[0], NodePos[1], NodePos[2]);
	gluSphere(qobj, radius, 20, 20);
	glPopMatrix();
	drawBox();
}

void MeshfreeNode::drawActiveChildNode(Vec3f color, float radius)
{
	std::vector<MeshfreeNode*> node;
	getActiveChildNode(node);
	for(int i=0;i<node.size();i++)
		node[i]->drawNode(color, radius);
}

void MeshfreeNode::drawBox()
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

void MeshfreeNode::findNeighborNode(std::vector<std::vector<int>>& neighbor, std::vector<int>& dest, std::vector<Vec3f>* nodePos, float support)
{
	std::vector<MeshfreeNode*> root;
	rootNode(root);
	
	VectorFunc func;
	int nodeIdx=root[0]->nodeIdx();
	std::vector<int> pNeighbor;
	for(int i=0;i<neighbor[nodeIdx].size();i++)
		dest.push_back(neighbor[nodeIdx][i]);
}
void MeshfreeNode::generateChildNode(SurfaceObj* obj)
{
	Vec3f nodePos;
	Vec3f boxSize=(RightUp-LeftDown)/4.0;

	if(Child[0])
	{
		for(int i=0;i<8;i++)
			Child[i]->generateChildNode(obj);
	}
	else
	{
		//RightTopFront
		nodePos=(RightUp+NodePos)/2.0;
		Child[0]=new MeshfreeNode(this, nodePos, boxSize);

		//RightTopBehind
		nodePos=(Vec3f(RightUp[0], RightUp[1], NodePos[2])+Vec3f(NodePos[0], NodePos[1], LeftDown[2]))/2.0;
		Child[1]=new MeshfreeNode(this, nodePos, boxSize);

		//RightBottomFront
		nodePos=(Vec3f(NodePos[0], LeftDown[1], NodePos[2])+Vec3f(RightUp[0], NodePos[1], RightUp[2]))/2.0;
		Child[2]=new MeshfreeNode(this, nodePos, boxSize);

		//RightBottomBehind
		nodePos=(Vec3f(NodePos[0], LeftDown[1], LeftDown[2])+Vec3f(RightUp[0], NodePos[1], NodePos[2]))/2.0;
		Child[3]=new MeshfreeNode(this, nodePos, boxSize);

		//LeftTopFront
		nodePos=(Vec3d(LeftDown[0], NodePos[1], NodePos[2])+Vec3d(NodePos[0], RightUp[1], RightUp[2]))/2.0;
		Child[4]=new MeshfreeNode(this, nodePos, boxSize);

		//LeftTopBehind
		nodePos=(Vec3f(LeftDown[0], NodePos[1], LeftDown[2])+Vec3f(NodePos[0], RightUp[1], NodePos[2]))/2.0;
		Child[5]=new MeshfreeNode(this, nodePos, boxSize);

		//LeftBottomFront
		nodePos=(Vec3f(LeftDown[0], LeftDown[1], NodePos[2])+Vec3f(NodePos[0], NodePos[1], RightUp[2]))/2.0;
		Child[6]=new MeshfreeNode(this, nodePos, boxSize);

		//LeftBottomBehind
		nodePos=(LeftDown+NodePos)/2.0;
		Child[7]=new MeshfreeNode(this, nodePos, boxSize);

		//Surface object 내부의 node만 activate 시킨다.
		activateChildNode(obj);
	}
}

void MeshfreeNode::activateChildNode(SurfaceObj* obj)
{
	CollisionManager collision;
	for(int i=0;i<8;i++)
	{
		Child[i]->activate();
		Vec3f leftDown=Child[i]->leftDown();
		Vec3f rightUp=Child[i]->rightUp();
		Vec3f p[8];
		p[0]=leftDown;
		p[1]=rightUp;
		p[2]=Vec3f(leftDown[0],rightUp[1],leftDown[2]);
		p[3]=Vec3f(leftDown[0],rightUp[1],rightUp[2]);
		p[4]=Vec3f(leftDown[0],leftDown[1],rightUp[2]);
		p[5]=Vec3f(rightUp[0],rightUp[1],leftDown[2]);
		p[6]=Vec3f(rightUp[0],rightUp[1],rightUp[2]);
		p[7]=Vec3f(rightUp[0],leftDown[1],rightUp[2]);
		for(int j=0;j<8;j++)
		{
			if(!collision.isPointInSurfObj(obj, p[j]))
			{
				Child[i]->deAtivate();
				break;
			}
		}
	}
}

bool MeshfreeNode::hasChild()
{
	if(Child[0])
		return true;
	else
		return false;
}

void MeshfreeNode::activate()
{
	Active=true;
}
void MeshfreeNode::deAtivate()
{
	Active=false;
}
bool MeshfreeNode::isActive()
{
	if(NodeIdx>=0)
		return false;
	return Active;
}

bool MeshfreeNode::getActiveChildNode(std::vector<MeshfreeNode*>& node)
{
	for(int i=0;i<8;i++)
	{
		if(Child[i])
		{
			if(Child[i]->isActive())
				node.push_back(Child[i]);
			Child[i]->getActiveChildNode(node);
		}
	}
	if(node.empty())
		return false;
	else
		return true;
}

void MeshfreeNode::rootNode(std::vector<MeshfreeNode*>& root)
{
	if(Parent)
		Parent->rootNode(root);
	else
		root.push_back(this);
}