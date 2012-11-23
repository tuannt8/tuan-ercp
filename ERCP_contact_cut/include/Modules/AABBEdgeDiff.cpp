#include "stdafx.h"
#include "AABBEdgeDiff.h"


AABBTreeEdgeDiff::AABBTreeEdgeDiff(void)
{
	Point=NULL;
	Node=NULL;
	Edge=NULL;
}

AABBTreeEdgeDiff::AABBTreeEdgeDiff(std::vector<Vec3f>* point, std::vector<Vec3f>* node, std::vector<Vec2i>* edge)
{
	Point=point;
	Node=node;
	Edge=edge;
}

AABBTreeEdgeDiff::~AABBTreeEdgeDiff(void)
{
}

void AABBTreeEdgeDiff::init(std::vector<Vec3f>* point, std::vector<Vec3f>* node, std::vector<Vec2i>* edge)
{
	Point=point;
	Node=node;
	Edge=edge;
}

void AABBTreeEdgeDiff::constructAABBTree()
{
	Root=new AABBNode;
	generateBoundingBox(Root);
}

void AABBTreeEdgeDiff::constructAABBTree(std::vector<int>& edgeIdx)
{
	Root=new AABBNode;
	generateBoundingBox(edgeIdx, Root);
}

void AABBTreeEdgeDiff::generateBoundingBox(AABBNode* root)
{
	//1. find bounding box
	Vec3f leftDown=Vec3f(MAX,MAX,MAX);
	Vec3f rightUp=Vec3f(MIN,MIN,MIN);

	for(int i=0;i<Point->size();i++)
	{
		for(int j=0;j<3;j++)
		{
			if((*Point)[i][j]<leftDown[j])
				leftDown[j]=(*Point)[i][j];
			if((*Point)[i][j]>rightUp[j])
				rightUp[j]=(*Point)[i][j];
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
	std::vector<int> edgeIdx1;
	std::vector<int> edgeIdx2;

	for(int i=0;i<Edge->size();i++)
	{
		float meanLength=0;

		meanLength+=((*Point)[(*Edge)[i][0]][maxIdx]-mid[maxIdx]);
		meanLength+=((*Node)[(*Edge)[i][1]][maxIdx]-mid[maxIdx]);

		if(meanLength<0)
			edgeIdx1.push_back(i);
		else
			edgeIdx2.push_back(i);
	}

	//5. generate descendant node
	AABBNode* left=new AABBNode;
	AABBNode* right=new AABBNode;

	root->Left=left;
	root->Right=right;
	left->Parent=root;
	right->Parent=root;

	//6. generate bounding box
	generateBoundingBox(edgeIdx1, left);
	generateBoundingBox(edgeIdx2, right);
}

void AABBTreeEdgeDiff::generateBoundingBox(std::vector<int>& edgeIdx, AABBNode* root)
{
	// 1. generate bounding box
	Vec3f leftDown(MAX, MAX, MAX);
	Vec3f rightUp(MIN, MIN, MIN);

	for(int i=0;i<edgeIdx.size();i++)
	{
		for(int j=0;j<3;j++)
		{
			if((*Node)[(*Edge)[edgeIdx[i]][1]][j]<leftDown[j])
				leftDown[j]=(*Node)[(*Edge)[edgeIdx[i]][1]][j];
			if((*Node)[(*Edge)[edgeIdx[i]][1]][j]>rightUp[j])
				rightUp[j]=(*Node)[(*Edge)[edgeIdx[i]][1]][j];

			if((*Point)[(*Edge)[edgeIdx[i]][0]][j]<leftDown[j])
				leftDown[j]=(*Point)[(*Edge)[edgeIdx[i]][0]][j];
			if((*Point)[(*Edge)[edgeIdx[i]][0]][j]>rightUp[j])
				rightUp[j]=(*Point)[(*Edge)[edgeIdx[i]][0]][j];
		}
	}
	root->setBoundingBox(leftDown, rightUp);
	if(root->Parent)
		root->Depth=root->Parent->Depth+1;
	else
		root->Depth=0;

	// 2. out if the node is leaf node
	if(edgeIdx.size()==1)
	{
		root->End=true;
		root->IndexInLeafNode=edgeIdx[0];
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
	std::vector<int> edgeIdx1;
	std::vector<int> edgeIdx2;
	std::vector<int> edgeIdxInBoundary1;
	std::vector<int> edgeIdxInBoundary2;

	for(int i=0;i<edgeIdx.size();i++)
	{
		float meanLength=0;
		meanLength+=((*Point)[(*Edge)[edgeIdx[i]][0]][maxIdx]-mid[maxIdx]);
		meanLength+=((*Node)[(*Edge)[edgeIdx[i]][1]][maxIdx]-mid[maxIdx]);

		if(meanLength<0)
			edgeIdx1.push_back(edgeIdx[i]);
		else
			edgeIdx2.push_back(edgeIdx[i]);
	}

	if(edgeIdx1.size()==0)
	{
		edgeIdx1.push_back(edgeIdx2[edgeIdx2.size()-1]);
		edgeIdx2.pop_back();
	}
	if(edgeIdx2.size()==0)
	{
		edgeIdx2.push_back(edgeIdx1[edgeIdx1.size()-1]);
		edgeIdx1.pop_back();
	}

	//5. generate descendant node
	AABBNode* left=new AABBNode;
	AABBNode* right=new AABBNode;

	root->Left=left;
	root->Right=right;
	left->Parent=root;
	right->Parent=root;

	//6. generate bounding box
	generateBoundingBox(edgeIdx1, left);
	generateBoundingBox(edgeIdx2, right);
}

void AABBTreeEdgeDiff::updateAABBTreeBottomUp()
{
	// 1. Transverse to the leaf node
	updateAABBLeafNode(Root);

	// 2. Update bounding box bottom up
	updateBoundingBoxBottomUp(Root);
}

void AABBTreeEdgeDiff::updateBoundingBoxBottomUp(AABBNode* root)
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

void AABBTreeEdgeDiff::updateAABBLeafNode(AABBNode* root)
{
	if(root)
	{
		if(root->End)
		{
			int edgeIdx=root->IndexInLeafNode;
			Vec3f leftDown=Vec3f(MAX, MAX, MAX);
			Vec3f rightUp=Vec3f(MIN, MIN, MIN);
			Vec2i edge=(*Edge)[edgeIdx];
			for(int i=0;i<3;i++)
			{
				if((*Point)[edge[0]][i]<leftDown[i])
					leftDown[i]=(*Point)[edge[0]][i];
				if((*Point)[edge[0]][i]>rightUp[i])
					rightUp[i]=(*Point)[edge[0]][i];

				if((*Node)[edge[1]][i]<leftDown[i])
					leftDown[i]=(*Node)[edge[1]][i];
				if((*Node)[edge[1]][i]>rightUp[i])
					rightUp[i]=(*Node)[edge[1]][i];
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

AABBNode* AABBTreeEdgeDiff::findLeafNode(int edgeIdx)
{
// 	std::vector<AABBNode*> node;
// 	Vec3f edge[2]; 
// 	edge[0]=(*Point)[(*Edge)[edgeIdx][0]];
// 	edge[1]=(*Node)[(*Edge)[edgeIdx][1]];
// 	GeometricFunc func;
// 	if(func.isLineInBox(Root->LeftDown, Root->RightUp, edge))
// 	{
// 		findLeafNode(Root->Left, edgeIdx, node);
// 		findLeafNode(Root->Right, edgeIdx, node);
// 	}
// 	if(node.size()==0)
// 		return NULL;
// 	else
// 		return node[0];

	std::vector<AABBNode*> nodes;
	Vec3f edge[2]; 
	edge[0]=(*Point)[(*Edge)[edgeIdx][0]];
	edge[1]=(*Node)[(*Edge)[edgeIdx][1]];

	findLeafNode(Root, edgeIdx, nodes);

	for(int i=0; i<nodes.size(); i++)
	{
		if (nodes[i]->IndexInLeafNode == edgeIdx)
		{
			return nodes[i];
		}
	}
	return NULL;
}


void AABBTreeEdgeDiff::findLeafNode(AABBNode* root, int edgeIdx, std::vector<AABBNode*>& node)
{
	if(root)
	{
		Vec3f edge[2]; 
		edge[0]=(*Point)[(*Edge)[edgeIdx][0]];
		edge[1]=(*Node)[(*Edge)[edgeIdx][1]];
		GeometricFunc func;
		if(func.isLineInBox(root->LeftDown, root->RightUp, edge))
		{
			if(root->End)
			{
				node.push_back(root);
			}
			else
			{
				findLeafNode(root->Left, edgeIdx, node);
				findLeafNode(root->Right, edgeIdx, node);
			}
		}
	}
	return;
}