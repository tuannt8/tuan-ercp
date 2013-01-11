#include "stdafx.h"
#include "AABB.h"
#include "geometricfunc.h"

#define copy_box(box, aabb) box.leftDown = aabb->LeftDown; \
	box.rightUp = aabb->RightUp; \
	box.center = (aabb->LeftDown+aabb->RightUp)/2

AABBNode::AABBNode(void)
{
	End=false;
	Updated=false;
	Left=NULL;
	Right=NULL;
	Parent=NULL;
	Depth=-1;
}

AABBNode::~AABBNode(void)
{
	if(Left)
		delete Left;
	if(Right)
		delete Right;
}

void AABBNode::setBoundingBox(Vec3f leftDown, Vec3f rightUp)
{
	LeftDown=leftDown;
	RightUp=rightUp;
}

void  AABBNode::copy(AABBNode* node)
{
	node->LeftDown=LeftDown;
	node->RightUp=RightUp;
	node->Depth=Depth;
	node->End=End;
	node->Updated=Updated;
	node->IndexInLeafNode=IndexInLeafNode;
}

void AABBNode::drawBoundingBox()
{
	float offset=0.0;
	glColor3f(0,0,1);
	glBegin(GL_LINES);
	glVertex3f((GLfloat)LeftDown[0]-offset, (GLfloat)LeftDown[1]-offset, (GLfloat)LeftDown[2]-offset);
	glVertex3f((GLfloat)RightUp[0]+offset, (GLfloat)LeftDown[1]-offset, (GLfloat)LeftDown[2]-offset);
	glVertex3f((GLfloat)RightUp[0]+offset, (GLfloat)LeftDown[1]-offset, (GLfloat)LeftDown[2]-offset);
	glVertex3f((GLfloat)RightUp[0]+offset, (GLfloat)LeftDown[1]-offset, (GLfloat)RightUp[2]+offset);
	glVertex3f((GLfloat)RightUp[0]+offset, (GLfloat)LeftDown[1]-offset, (GLfloat)RightUp[2]+offset);
	glVertex3f((GLfloat)LeftDown[0]-offset, (GLfloat)LeftDown[1]-offset, (GLfloat)RightUp[2]+offset);
	glVertex3f((GLfloat)LeftDown[0]-offset, (GLfloat)LeftDown[1]-offset, (GLfloat)RightUp[2]+offset);
	glVertex3f((GLfloat)LeftDown[0]-offset, (GLfloat)LeftDown[1]-offset, (GLfloat)LeftDown[2]-offset);

	glVertex3f((GLfloat)LeftDown[0]-offset, (GLfloat)RightUp[1], (GLfloat)LeftDown[2]-offset);
	glVertex3f((GLfloat)RightUp[0]+offset, (GLfloat)RightUp[1]+offset, (GLfloat)LeftDown[2]-offset);
	glVertex3f((GLfloat)RightUp[0]+offset, (GLfloat)RightUp[1]+offset, (GLfloat)LeftDown[2]-offset);
	glVertex3f((GLfloat)RightUp[0]+offset, (GLfloat)RightUp[1]+offset, (GLfloat)RightUp[2]+offset);
	glVertex3f((GLfloat)RightUp[0]+offset, (GLfloat)RightUp[1]+offset, (GLfloat)RightUp[2]+offset);
	glVertex3f((GLfloat)LeftDown[0]-offset, (GLfloat)RightUp[1]+offset, (GLfloat)RightUp[2]+offset);
	glVertex3f((GLfloat)LeftDown[0]-offset, (GLfloat)RightUp[1]+offset, (GLfloat)RightUp[2]+offset);
	glVertex3f((GLfloat)LeftDown[0]-offset, (GLfloat)RightUp[1]+offset, (GLfloat)LeftDown[2]-offset);

	glVertex3f((GLfloat)LeftDown[0]-offset, (GLfloat)LeftDown[1]-offset, (GLfloat)LeftDown[2]-offset);
	glVertex3f((GLfloat)LeftDown[0]-offset, (GLfloat)RightUp[1]+offset, (GLfloat)LeftDown[2]-offset);
	glVertex3f((GLfloat)RightUp[0]+offset, (GLfloat)LeftDown[1]-offset, (GLfloat)RightUp[2]+offset);
	glVertex3f((GLfloat)RightUp[0]+offset, (GLfloat)RightUp[1]+offset, (GLfloat)RightUp[2]+offset);
	glVertex3f((GLfloat)LeftDown[0]-offset, (GLfloat)LeftDown[1]-offset, (GLfloat)RightUp[2]+offset);
	glVertex3f((GLfloat)LeftDown[0]-offset, (GLfloat)RightUp[1]+offset, (GLfloat)RightUp[2]+offset);
	glVertex3f((GLfloat)RightUp[0]+offset, (GLfloat)LeftDown[1]-offset, (GLfloat)LeftDown[2]-offset);
	glVertex3f((GLfloat)RightUp[0]+offset, (GLfloat)RightUp[1]+offset, (GLfloat)LeftDown[2]-offset);
/*	glVertex3f((GLfloat)LeftDown[0], (GLfloat)LeftDown[1], (GLfloat)LeftDown[2]);
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
	glVertex3f((GLfloat)RightUp[0], (GLfloat)RightUp[1], (GLfloat)LeftDown[2]);*/
	glEnd();
}

AABBTree::AABBTree(void)
{
	Root=NULL;
}

AABBTree::~AABBTree(void)
{
	if(Root)
		delete Root;
}

void AABBTree::constructAABBTree()
{
}

void AABBTree::constructAABBTree(std::vector<int>& idx)
{
}

void AABBTree::updateAABBTreeBottomUp()
{
}

void AABBTree::removeNode(AABBNode* node)
{
	if (!node)
		return;

	AABBNode* parent=node->Parent;
	AABBNode* grandParent=parent->Parent;
	if(parent->Left==node)
	{
		parent->Left=NULL;
		if(grandParent)
		{
			if(grandParent->Left==parent)
			{
				grandParent->Left=parent->Right;
				parent->Right->Parent=grandParent;
				parent->Right=NULL;
				updateDepth(grandParent->Left);
				delete parent;
			}
			else if(grandParent->Right==parent)
			{
				grandParent->Right=parent->Right;
				parent->Right->Parent=grandParent;
				parent->Right=NULL;
				updateDepth(grandParent->Right);
				delete parent;
			}
		}
		else
		{
			Root=parent->Right;
			Root->Parent=NULL;
			parent->Right=NULL;
			updateDepth(Root);
			delete parent;
		}
		delete node;
		return;
	}
	if(parent->Right==node)
	{
		parent->Right=NULL;
		if(grandParent)
		{
			if(grandParent->Left==parent)
			{
				grandParent->Left=parent->Left;
				parent->Left->Parent=grandParent;
				parent->Left=NULL;
				updateDepth(grandParent->Left);
				delete parent;
			}
			else if(grandParent->Right==parent)
			{
				grandParent->Right=parent->Left;
				parent->Left->Parent=grandParent;
				parent->Left=NULL;
				updateDepth(grandParent->Right);
				delete parent;
			}
		}
		else
		{
			Root=parent->Left;
			Root->Parent=NULL;
			parent->Left=NULL;
			updateDepth(Root);
			delete parent;
		}
		delete node;
		return;
	}
}

void AABBTree::addNode(AABBNode* node)
{
	addNode(Root, node);	
}

void AABBTree::addNode( AABBNode* _root, AABBNode* newNode )
{
	bool eneded = _root->End;
	if (_root->End == true)
	{
		AABBNode* newChildNode=new AABBNode;
		_root->copy(newChildNode);

		newChildNode->End = true;
		newChildNode->Depth = _root->Depth+1;
		newChildNode->Parent = _root;
		newNode->End = true;
		newNode->Depth = _root->Depth+1;
		newNode->Parent = _root;

		_root->End = false;
		_root->Left = newChildNode;
		_root->Right = newNode;

		for (int i=0; i<3; i++)
		{
			if (_root->LeftDown[i] > newNode->LeftDown[i])
			{
				_root->LeftDown[i] = newNode->LeftDown[i];
			}
			if (_root->RightUp[i] < newNode->RightUp[i])
			{
				_root->RightUp[i] = newNode->RightUp[i];
			}
		}
	}
	else
	{
		GeometricFunc funcs;

		AABBNode* _left = _root->Left;
		AABBNode* _right = _root->Right;

		if (!_left)
			addNode(_right, newNode);
		else if (!_right)
			addNode(_left, newNode);
		else
		{
			Box leftBox, righBox, newBox;
			copy_box(leftBox, _left);
			copy_box(righBox, _right);
			copy_box(newBox, newNode);

			float leftDis = funcs.disBtwBox(leftBox, newBox);
			float rightDis = funcs.disBtwBox(righBox, newBox);
			if (leftDis < rightDis)
			{
				addNode(_left, newNode);
			}
			else
				addNode(_right, newNode);
		}
	}
}

AABBNode* AABBTree::findLeafNode(int idx)
{
	return NULL;
}

bool AABBTree::findCellIncludingEdge(int edgeIdx, std::vector<int>& cellIdx)
{
	return false;
}

void AABBTree::updateDepth(AABBNode* root)
{
	if(root)
	{
		if(root->Parent)
		{
			root->Depth=root->Parent->Depth+1;
			if(root->End)
				return;
			else
			{
				updateDepth(root->Left);
				updateDepth(root->Right);
			}
		}
		else
		{
			root->Depth=0;
			if(root->End)
				return;
			else
			{
				updateDepth(root->Left);
				updateDepth(root->Right);
			}
		}
	}
}

void AABBTree::getIndexInLeafNode(std::vector<int>& index)
{
	getIndexInLeafNode(Root, index);
}

void AABBTree::getIndexInLeafNode(AABBNode* root, std::vector<int>& index)
{
	if(root)
	{
		if(root->End)
		{
			index.push_back(root->IndexInLeafNode);
			return;
		}
		else
		{
			getIndexInLeafNode(root->Left, index);
			getIndexInLeafNode(root->Right, index);
		}
	}
}

AABBNode* AABBTree::root()
{
	return Root;
}

void AABBTree::drawBoundingBox()
{
	drawBoundingBox(Root);
}

void AABBTree::drawBoundingBox(AABBNode* root)
{
	if(root)
	{
		root->drawBoundingBox();
		if(root->End)
			return;
		drawBoundingBox(root->Left);
		drawBoundingBox(root->Right);
	}
}

void AABBTree::drawBoundingBoxTo(int depth)
{
	drawBoundingBoxTo(depth, Root);
}
void AABBTree::drawBoundingBoxTo(int depth, AABBNode* root)
{
	if(root)
	{
		if(root->Depth<=depth)
			root->drawBoundingBox();
		if(root->End)
			return;
		drawBoundingBoxTo(depth, root->Left);
		drawBoundingBoxTo(depth, root->Right);
	}
}
void AABBTree::drawBoundingBoxAt(int depth)
{
	drawBoundingBoxTo(depth, Root);
}
void AABBTree::drawBoundingBoxAt(int depth, AABBNode* root)
{
	if(root)
	{
		if(root->Depth==depth)
			root->drawBoundingBox();
		if(root->End)
			return;
		drawBoundingBoxAt(depth, root->Left);
		drawBoundingBoxAt(depth, root->Right);
	}
}

void AABBTree::drawBoundingBoxLeafNode()
{
	drawBoundingBoxLeafNode(Root);
}

void AABBTree::drawBoundingBoxLeafNode(AABBNode* root)
{
	if(root)
	{
		if(root->End)
		{
			root->drawBoundingBox();
			return;
		}
		else
		{
			drawBoundingBoxLeafNode(root->Left);
			drawBoundingBoxLeafNode(root->Right);
		}
	}
}

void AABBTree::copy(AABBTree* tree)
{
	tree->Root=new AABBNode;
	copy(Root, tree, tree->Root);	
}

void AABBTree::copy(AABBNode* root, AABBTree* tree, AABBNode* rootCopy)
{
	if(root)
	{
		if(root->End)
		{
			root->copy(rootCopy);
			//tree->LeafNodeIdx.push_back(root->IndexInLeafNode);
			//tree->LeafNodePtr.push_back(rootCopy);
			//tree->LeafNode.insert(std::pair<int,AABBNode*>(root->IndexInLeafNode,rootCopy));
			return;
		}
		else
		{
			root->copy(rootCopy);
			rootCopy->Left=new AABBNode;
			rootCopy->Left->Parent=rootCopy;
			rootCopy->Right=new AABBNode;
			rootCopy->Right->Parent=rootCopy;
			copy(root->Left, tree, rootCopy->Left);
			copy(root->Right, tree, rootCopy->Right);
			return;
		}	
	}
}

/*void AABBTree::changeIdxInLeafNode(int from, int to)
{
	std::map<int,AABBNode*>::iterator iter=LeafNode.find(from);
	if(iter!=LeafNode.end())
	{
		AABBNode* node=iter->second;
		node->IndexInLeafNode=to;
		iter=LeafNode.find(to);
		if(iter!=LeafNode.end())
			iter->second=node;
		else
			LeafNode.insert(std::pair<int,AABBNode*>(to,node));
	}

	AABBNode* node;
	for(int i=0;i<LeafNodeIdx.size();i++)
	{
		if(LeafNodeIdx[i]==from)
		{
			node=LeafNodePtr[i];
			break;
		}
	}
	for(int i=0;i<LeafNodeIdx.size();i++)
	{
		if(LeafNodeIdx[i]==to)
		{
			LeafNodePtr[i]=node;
			node->IndexInLeafNode=to;
		}
	}
}*/