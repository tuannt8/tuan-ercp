#include "stdafx.h"
#include "AABBEdge.h"


AABBTreeEdge::AABBTreeEdge(void)
{
	Node=NULL;
	Edge=NULL;
}

AABBTreeEdge::AABBTreeEdge(std::vector<Vec3f>* node, std::vector<Vec2i>* edge)
{
	Node=node;
	Edge=edge;
}

AABBTreeEdge::~AABBTreeEdge(void)
{
}

void AABBTreeEdge::init(std::vector<Vec3f>* node, std::vector<Vec2i>* edge)
{
	Node=node;
	Edge=edge;
}

void AABBTreeEdge::constructAABBTree()
{
	Root=new AABBNode;
	generateBoundingBox(Root);
}

void AABBTreeEdge::constructAABBTree(std::vector<int>& edgeIdx)
{
	Root=new AABBNode;
	generateBoundingBox(edgeIdx, Root);
}

void AABBTreeEdge::generateBoundingBox(AABBNode* root)
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
	std::vector<int> edgeIdx1;
	std::vector<int> edgeIdx2;
	std::vector<int> edgeIdxInBoundary1;
	std::vector<int> edgeIdxInBoundary2;

	for(int i=0;i<Edge->size();i++)
	{
		float meanLength=0;
		for(int j=0;j<2;j++)
			meanLength+=((*Node)[(*Edge)[i][j]][maxIdx]-mid[maxIdx]);
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

void AABBTreeEdge::generateBoundingBox(std::vector<int>& edgeIdx, AABBNode* root)
{
	// 1. generate bounding box
	Vec3f leftDown(MAX, MAX, MAX);
	Vec3f rightUp(MIN, MIN, MIN);

	for(int i=0;i<edgeIdx.size();i++)
	{
		for(int j=0;j<2;j++)
		{
			for(int k=0;k<3;k++)
			{
				if((*Node)[(*Edge)[edgeIdx[i]][j]][k]<leftDown[k])
					leftDown[k]=(*Node)[(*Edge)[edgeIdx[i]][j]][k];
				if((*Node)[(*Edge)[edgeIdx[i]][j]][k]>rightUp[k])
					rightUp[k]=(*Node)[(*Edge)[edgeIdx[i]][j]][k];
			}
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
		//LeafNode.insert(std::pair<int,AABBNode*>(edgeIdx[0],root));
		//LeafNodeIdx.push_back(edgeIdx[0]);
		//LeafNodePtr.push_back(root);
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
		for(int j=0;j<2;j++)
			meanLength+=((*Node)[(*Edge)[edgeIdx[i]][j]][maxIdx]-mid[maxIdx]);
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

void AABBTreeEdge::updateAABBTreeBottomUp()
{
	// 1. Transverse to the leaf node
	updateAABBLeafNode(Root);

	// 2. Update bounding box bottom up
	updateBoundingBoxBottomUp(Root);
}

void AABBTreeEdge::updateBoundingBoxBottomUp(AABBNode* root)
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

void AABBTreeEdge::updateAABBLeafNode(AABBNode* root)
{
	if(root)
	{
		if(root->End)
		{
			int edgeIdx=root->IndexInLeafNode;
			Vec3f leftDown=Vec3f(MAX, MAX, MAX);
			Vec3f rightUp=Vec3f(MIN, MIN, MIN);
			Vec2i edge=(*Edge)[edgeIdx];
			for(int i=0;i<2;i++)
			{
				for(int j=0;j<3;j++)
				{
					if((*Node)[edge[i]][j]<leftDown[j])
						leftDown[j]=(*Node)[edge[i]][j];
					if((*Node)[edge[i]][j]>rightUp[j])
						rightUp[j]=(*Node)[edge[i]][j];
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

AABBNode* AABBTreeEdge::findLeafNode(int edgeIdx)
{
/*	for(int i=0;i<LeafNodeIdx.size();i++)
	{
		if(LeafNodeIdx[i]==edgeIdx)
			return LeafNodePtr[i];
	}
	return NULL;
	std::map<int,AABBNode*>::iterator iter=LeafNode.find(edgeIdx);
	if(iter!=LeafNode.end())
		return iter->second;
	else
		return NULL;*/

// 	std::vector<AABBNode*> node;
// 	Vec3f edge[2]; 
// 	edge[0]=(*Node)[(*Edge)[edgeIdx][0]];
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

void AABBTreeEdge::findLeafNode(AABBNode* root, int edgeIdx, std::vector<AABBNode*>& node)
{
	if(root)
	{
		Vec3f edge[2]; 
		edge[0]=(*Node)[(*Edge)[edgeIdx][0]];
		edge[1]=(*Node)[(*Edge)[edgeIdx][1]];
		GeometricFunc func;
		if(func.isLineInBox(root->LeftDown, root->RightUp, edge))
		{
			if(root->End)
			{
				if(root->IndexInLeafNode==edgeIdx)
				{
					node.push_back(root);
					return;
				}
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