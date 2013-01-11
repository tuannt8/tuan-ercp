#include "stdafx.h"
#include "AABBTri.h"


AABBTreeTri::AABBTreeTri(void)
{
}

AABBTreeTri::AABBTreeTri(std::vector<Vec3f>* point, std::vector<Vec3i>* face, std::vector<Vec2i>* edge)
{
	Point=point;
	Face=face;
	Edge=edge;
}

AABBTreeTri::~AABBTreeTri(void)
{
}

void AABBTreeTri::init(std::vector<Vec3f>* point, std::vector<Vec3i>* face, std::vector<Vec2i>* edge)
{
	Point=point;
	Face=face;
	Edge=edge;
}

void AABBTreeTri::constructAABBTree()
{
	Root=new AABBNode;
	generateBoundingBox(Root);
}

void AABBTreeTri::constructAABBTree(std::vector<int>& faceIdx)
{
	Root=new AABBNode;
	generateBoundingBox(faceIdx, Root);
}

void AABBTreeTri::generateBoundingBox(AABBNode* root)
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
	std::vector<int> faceIdx1;
	std::vector<int> faceIdx2;

	for(int i=0;i<Face->size();i++)
	{
		float meanLength=0;
		for(int j=0;j<3;j++)
			meanLength+=((*Point)[(*Face)[i][j]][maxIdx]-mid[maxIdx]);

		if(meanLength<0)
			faceIdx1.push_back(i);
		else
			faceIdx2.push_back(i);
	}

	//5. generate descendant node
	AABBNode* left=new AABBNode;
	AABBNode* right=new AABBNode;

	root->Left=left;
	root->Right=right;
	left->Parent=root;
	right->Parent=root;

	//6. generate bounding box
	generateBoundingBox(faceIdx1, left);
	generateBoundingBox(faceIdx2, right);
}

void AABBTreeTri::generateBoundingBox(std::vector<int>& faceIdx, AABBNode* root)
{
	// 1. generate bounding box
	Vec3f leftDown(MAX, MAX, MAX);
	Vec3f rightUp(MIN, MIN, MIN);

	for(int i=0;i<faceIdx.size();i++)
	{
		for(int j=0;j<3;j++)
		{
			for(int k=0;k<3;k++)
			{
				if((*Point)[(*Face)[faceIdx[i]][j]][k]<leftDown[k])
					leftDown[k]=(*Point)[(*Face)[faceIdx[i]][j]][k];
				if((*Point)[(*Face)[faceIdx[i]][j]][k]>rightUp[k])
					rightUp[k]=(*Point)[(*Face)[faceIdx[i]][j]][k];
			}
		}
	}
	root->setBoundingBox(leftDown, rightUp);

	if(root->Parent)
		root->Depth=root->Parent->Depth+1;
	else
		root->Depth=0;

	// 2. out if the node is leaf node
	if(faceIdx.size()==1)
	{
		root->End=true;
		root->IndexInLeafNode=faceIdx[0];
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
	std::vector<int> faceIdx1;
	std::vector<int> faceIdx2;

	for(int i=0;i<faceIdx.size();i++)
	{
		float meanLength=0;
		for(int j=0;j<3;j++)
			meanLength+=((*Point)[(*Face)[faceIdx[i]][j]][maxIdx]-mid[maxIdx]);
		if(meanLength<0)
			faceIdx1.push_back(faceIdx[i]);
		else
			faceIdx2.push_back(faceIdx[i]);
	}
	if(faceIdx1.size()==0)
	{
		faceIdx1.push_back(faceIdx2[faceIdx2.size()-1]);
		faceIdx2.pop_back();
	}
	if(faceIdx2.size()==0)
	{
		faceIdx2.push_back(faceIdx1[faceIdx1.size()-1]);
		faceIdx1.pop_back();
	}

	//5. generate descendant node
	AABBNode* left=new AABBNode;
	AABBNode* right=new AABBNode;

	root->Left=left;
	root->Right=right;
	left->Parent=root;
	right->Parent=root;

	//6. generate bounding box
	generateBoundingBox(faceIdx1, left);
	generateBoundingBox(faceIdx2, right);
}

void AABBTreeTri::updateAABBTreeBottomUp()
{
	// 1. Transverse to the leaf node
	updateAABBLeafNode(Root);

	// 2. Update bounding box bottom up
	updateBoundingBoxBottomUp(Root);
}

void AABBTreeTri::updateAABBLeafNode(AABBNode* root)
{
	if(root)
	{
		if(root->End)
		{
			int triIdx=root->IndexInLeafNode;
			Vec3f leftDown=Vec3f(MAX, MAX, MAX);
			Vec3f rightUp=Vec3f(MIN, MIN, MIN);
			Vec3i face=(*Face)[triIdx];
			for(int i=0;i<3;i++)
			{
				for(int j=0;j<3;j++)
				{
					if((*Point)[face[i]][j]<leftDown[j])
						leftDown[j]=(*Point)[face[i]][j];
					if((*Point)[face[i]][j]>rightUp[j])
						rightUp[j]=(*Point)[face[i]][j];
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

void AABBTreeTri::updateBoundingBoxBottomUp(AABBNode* root)
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

AABBNode* AABBTreeTri::findLeafNode( int triIdx )
{
	if (triIdx > Face->size())
		return NULL;

	Vec3f tri[3];
	tri[0] = Point->at(Face->at(triIdx)[0]);
	tri[1] = Point->at(Face->at(triIdx)[1]);
	tri[2] = Point->at(Face->at(triIdx)[2]);

	std::vector<AABBNode*> potentialNode;
	findLeafNode(Root, tri, &potentialNode);

	for (int i=0; i<potentialNode.size(); i++)
	{
		if (potentialNode[i]->IndexInLeafNode == triIdx)
		{
			return potentialNode[i];
		}
	}
	return NULL;
}

void AABBTreeTri::findLeafNode( AABBNode* _root, Vec3f* tri, std::vector<AABBNode*>* nodes )
{
	GeometricFunc func;
	if (func.isTriInBox(_root->LeftDown, _root->RightUp, tri))
	{
		if (_root->End)
		{
			nodes->push_back(_root);
		}
		else
		{
			findLeafNode(_root->Left, tri, nodes);
			findLeafNode(_root->Right, tri, nodes);
		}
	}
	return;
}
