#include "stdafx.h"
#include "AABBPoint.h"


AABBTreePoint::AABBTreePoint(void)
{
	Node=NULL;
}

AABBTreePoint::AABBTreePoint(std::vector<Vec3f>* node, int nbNodeInBox)
{
	Node=node;
	NbNodeInBox=nbNodeInBox;
}

AABBTreePoint::~AABBTreePoint(void)
{
}

void AABBTreePoint::init(std::vector<Vec3f>* node, int nbNodeInBox)
{
	Node=node;
	NbNodeInBox=nbNodeInBox;
}

void AABBTreePoint::constructAABBTree()
{
	Root=new AABBNode;
	generateBoundingBox(Root);
}

void AABBTreePoint::constructAABBTree(std::vector<int>& nodeIdx)
{
	Root=new AABBNode;
	generateBoundingBox(nodeIdx, Root);
}

void AABBTreePoint::generateBoundingBox(AABBNode* root)
{
	//1. find bounding box
	Vec3f leftDown=Vec3f(MAX,MAX,MAX);
	Vec3f rightUp=Vec3f(MIN,MIN,MIN);

	for(int i=0;i<Node->size();i++)
	{
		for(int j=0;j<3;j++)
		{
			if((*Node)[i][j]<leftDown[j])
				leftDown[j]=(*Node)[i][j];
			if((*Node)[i][j]>rightUp[j])
				rightUp[j]=(*Node)[i][j];
		}
	}
	root->setBoundingBox(leftDown, rightUp);
	root->Depth=0;

	//2. find longest axis
	Vec3f ds=rightUp-leftDown;
	Vec3f mid=(rightUp+leftDown)/2.0;
	int maxIdx=-1;
	if(ds[0]>ds[1])
	{
		if(ds[0]>ds[2])
			maxIdx=0;
		else
			maxIdx=2;
	}
	else
	{
		if(ds[1]>ds[2])
			maxIdx=1;
		else
			maxIdx=2;
	}

	//3. divide the bounding box
	Vec3f leftDown1=leftDown;
	Vec3f rightUp1=rightUp;
	rightUp1[maxIdx]=mid[maxIdx];

	Vec3f leftDown2=leftDown;
	Vec3f rightUp2=rightUp;
	leftDown2[maxIdx]=mid[maxIdx];

	//4. find the nodes included in each bounding box;
	std::vector<int> nodeIdx1;
	std::vector<int> nodeIdx2;

	for(int i=0;i<Node->size();i++)
	{
		if((*Node)[i][maxIdx]<mid[maxIdx])
			nodeIdx1.push_back(i);
		else
			nodeIdx2.push_back(i);
	}

	//5. generate descendant node
	AABBNode* left=new AABBNode;
	AABBNode* right=new AABBNode;

	root->Left=left;
	root->Right=right;
	left->Parent=root;
	right->Parent=root;

	//6. generate bounding box
	generateBoundingBox(nodeIdx1, left);
	generateBoundingBox(nodeIdx2, right);
}

void AABBTreePoint::generateBoundingBox(std::vector<int>& nodeIdx, AABBNode* root)
{
	// 1. generate bounding box
	Vec3f leftDown(MAX, MAX, MAX);
	Vec3f rightUp(MIN, MIN, MIN);

	for(int i=0;i<nodeIdx.size();i++)
	{
		for(int j=0;j<3;j++)
		{
			if((*Node)[nodeIdx[i]][j]<leftDown[j])
				leftDown[j]=(*Node)[nodeIdx[i]][j];
			if((*Node)[nodeIdx[i]][j]>rightUp[j])
				rightUp[j]=(*Node)[nodeIdx[i]][j];
		}
	}
	root->setBoundingBox(leftDown, rightUp);
	if(root->Parent)
		root->Depth=root->Parent->Depth+1;
	else
		root->Depth=0;

	// 2. out if the node is leaf node
	if(nodeIdx.size()<NbNodeInBox)
	{
		root->End=true;
		for(int i=0;i<nodeIdx.size();i++)
			root->IndicesInLeafNode.push_back(nodeIdx[i]);
		return;
	}

	// 3. find the longest axis
	Vec3f ds=rightUp-leftDown;
	Vec3f mid=(rightUp+leftDown)/2.0;
	int maxIdx=-1;
	if(ds[0]>ds[1])
	{
		if(ds[0]>ds[2])
			maxIdx=0;
		else
			maxIdx=2;
	}
	else
	{
		if(ds[1]>ds[2])
			maxIdx=1;
		else
			maxIdx=2;
	}

	//3. divide the bounding box
	Vec3f leftDown1=leftDown;
	Vec3f rightUp1=rightUp;
	rightUp1[maxIdx]=mid[maxIdx];

	Vec3f leftDown2=leftDown;
	Vec3f rightUp2=rightUp;
	leftDown2[maxIdx]=mid[maxIdx];

	//4. find the triangles included in each bounding box;
	std::vector<int> nodeIdx1;
	std::vector<int> nodeIdx2;
	std::vector<int> edgeIdxInBoundary1;
	std::vector<int> edgeIdxInBoundary2;

	for(int i=0;i<nodeIdx.size();i++)
	{
		if((*Node)[nodeIdx[i]][maxIdx]<mid[maxIdx])
			nodeIdx1.push_back(nodeIdx[i]);
		else
			nodeIdx2.push_back(nodeIdx[i]);
	}

	if(nodeIdx1.size()==0)
	{
		nodeIdx1.push_back(nodeIdx2[nodeIdx2.size()-1]);
		nodeIdx2.pop_back();
	}
	if(nodeIdx2.size()==0)
	{
		nodeIdx2.push_back(nodeIdx1[nodeIdx1.size()-1]);
		nodeIdx1.pop_back();
	}

	//5. generate descendant node
	AABBNode* left=new AABBNode;
	AABBNode* right=new AABBNode;

	root->Left=left;
	root->Right=right;
	left->Parent=root;
	right->Parent=root;

	//6. generate bounding box
	generateBoundingBox(nodeIdx1, left);
	generateBoundingBox(nodeIdx2, right);
}

void AABBTreePoint::updateAABBTreeBottomUp()
{
	// 1. Transverse to the leaf node
	updateAABBLeafNode(Root);

	// 2. Update bounding box bottom up
	updateBoundingBoxBottomUp(Root);
}

void AABBTreePoint::updateBoundingBoxBottomUp(AABBNode* root)
{
	Vec3f leftDown=Vec3f(MAX, MAX, MAX);
	Vec3f rightUp=Vec3f(MIN, MIN, MIN);

	if(root)
	{
		if(!root->Left->Updated)
			updateBoundingBoxBottomUp(root->Left);
		if(!root->Right->Updated)
			updateBoundingBoxBottomUp(root->Right);

		for(int i=0;i<3;i++)
		{
			if(leftDown[i]>root->Left->LeftDown[i])
				leftDown[i]=root->Left->LeftDown[i];
			if(leftDown[i]>root->Right->LeftDown[i])
				leftDown[i]=root->Right->LeftDown[i];

			if(rightUp[i]<root->Left->RightUp[i])
				rightUp[i]=root->Left->RightUp[i];
			if(rightUp[i]<root->Right->RightUp[i])
				rightUp[i]=root->Right->RightUp[i];
		}
		root->setBoundingBox(leftDown, rightUp);
		root->Updated=true;
	}
}

void AABBTreePoint::updateAABBLeafNode(AABBNode* root)
{
	if(root)
	{
		if(root->End)
		{
			std::vector<int>* nodeIdx=root->indicesInLeafNode();
			Vec3f leftDown=Vec3f(MAX, MAX, MAX);
			Vec3f rightUp=Vec3f(MIN, MIN, MIN);
			
			for(int i=0;i<nodeIdx->size();i++)
			{
				int idx=(*nodeIdx)[i];
				for(int j=0;j<3;j++)
				{
					if((*Node)[idx][j]<leftDown[j])
						leftDown[j]=(*Node)[idx][j];
					if((*Node)[idx][j]>rightUp[j])
						rightUp[j]=(*Node)[idx][j];
				}
			}
			root->setBoundingBox(leftDown, rightUp);
			root->Updated=true;
		}
		else
		{
			root->Updated=false;
			updateAABBLeafNode(root->Left);
			updateAABBLeafNode(root->Right);
		}
	}
}