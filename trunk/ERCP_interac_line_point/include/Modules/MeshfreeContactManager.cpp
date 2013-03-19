#include "stdafx.h"
#include "MeshfreeContactManager.h"

MeshfreeContactManager::MeshfreeContactManager(void)
{
	F=fopen("MeshfreeContactManger.txt","w");
}

MeshfreeContactManager::~MeshfreeContactManager(void)
{
	fclose(F);
}

void MeshfreeContactManager::contactResolutionWithSurfObj(Meshfree_GPU* obj, SurfaceObj* environment, float dt)
{
	SurfaceObj* surfObj=obj->surfObj();
	std::vector<Vec3f>* surfNode=surfObj->point();
	std::vector<std::vector<int>>* neighborNodeIdx=obj->neighborNodeOfSurfVertice();
	std::vector<std::vector<float>>* shapeFuncValue=obj->shapeFuncValueAtSurfPoint();

	EFG_CUDA_RUNTIME* efgObj=obj->efgObj();
	int nbNode=efgObj->nbNode();
	float* nodeMass=efgObj->nodeMass();
	float* nodeForce=efgObj->nodeForce();
	float* nodePos0=efgObj->nodePos0();
	std::vector<int> colPointIdx;
	std::vector<Vec3f> colNormal;
	std::vector<float> deltaFree;

	// 1. Collision detection
	CollisionManager collision;
	float minGap=5;
	collision.PQBtwSurfaceObj(surfObj, environment, colPointIdx, colNormal, deltaFree, minGap);
	if(colPointIdx.size()>0)
		float a=0;
	
	// 2. Construct mapping matrix
	Matrix HCHT, HT;
	std::vector<int> relatedNodeIdx;
	constructMappingMatrixSparse(obj, colPointIdx, colNormal, relatedNodeIdx, HT, HCHT, dt);

	// 3. LCP solver
	Matrix contactForce;
	GaussSeidelMethod(HCHT, deltaFree, contactForce, 10);

	// 4. Add force
	Matrix f=HT*contactForce;
	for(int i=0;i<nbNode;i++)
	{
		for(int j=0;j<3;j++)
		{
			nodeForce[i*3+j]=0;
		}
	}
	for(int i=0;i<relatedNodeIdx.size();i++)
	{
		for(int j=0;j<3;j++)
			nodeForce[relatedNodeIdx[i]*3+j]+=f(i*3+j,0);
	}
}

void MeshfreeContactManager::contactResolutionWithSurfObj(Meshfree_CPU* obj, SurfaceObj* environment, float dt)
{
	Obj=obj;
	SurfaceObj* surfObj=obj->surfObj();
	std::vector<Vec3f>* surfNode=surfObj->point();
	std::vector<std::vector<int>>* neighborNodeIdx=obj->neighborNodeOfSurfVertice();
	std::vector<std::vector<float>>* shapeFuncValue=obj->shapeFuncValueAtSurfPoint();

	EFG_CPU* efgObj=obj->efgObj();
	std::vector<float>* nodeMass=efgObj->nodeMass();
	std::vector<Vec3f>* nodeForce=efgObj->nodeForce();
	std::vector<int> colPointIdx;
	std::vector<Vec3f> colNormal;
	std::vector<float> deltaFree;

	// 1. Collision detection
	CollisionManager collision;
	float minGap=5;
	collision.PQBtwSurfaceObj(surfObj, environment, colPointIdx, colNormal, deltaFree, minGap);
	for(int i=0;i<colPointIdx.size();i++)
	{
		fprintf(F,"%d %f %f %f %f\n",i, deltaFree[i], colNormal[i][0],colNormal[i][1],colNormal[i][2]);
	}
	
	// 2. Construct mapping matrix
	Matrix HCHT, HT;
	std::vector<int> relatedNodeIdx;
	constructMappingMatrixSparse(obj, colPointIdx, colNormal, relatedNodeIdx, HT, HCHT, dt);
	//constructMappingMatrix(obj, colNodeIdx, colNormal, relatedNodeIdx, HT, HCHT, dt);

	// 3. LCP solver
	Matrix contactForce;
	GaussSeidelMethod(HCHT, deltaFree, contactForce, 30);

	// 4. Add force
	Matrix f=HT*contactForce;
	for(int i=0;i<relatedNodeIdx.size();i++)
	{
		for(int j=0;j<3;j++)
			(*nodeForce)[relatedNodeIdx[i]][j]+=f(i*3+j,0);
	}
}

void MeshfreeContactManager::contactResolutionWithPlane(Meshfree_CPU* obj, float dt)
{
	Obj=obj;
	SurfaceObj* surfObj=obj->surfObj();
	std::vector<Vec3f>* surfNode=surfObj->point();
	std::vector<std::vector<int>>* neighborNodeIdx=obj->neighborNodeOfSurfVertice();
	std::vector<std::vector<float>>* shapeFuncValue=obj->shapeFuncValueAtSurfPoint();

	EFG_CPU* efgObj=obj->efgObj();
	std::vector<float>* nodeMass=efgObj->nodeMass();
	std::vector<Vec3f>* nodeForce=efgObj->nodeForce();
	std::vector<int> colNodeIdx;
	std::vector<Vec3f> colNormal;
	std::vector<float> deltaFree;
	
	// 1. Collision detection
	for(int i=0;i<nodeForce->size();i++)
		(*nodeForce)[i].clear();

	for(int i=0;i<surfNode->size();i++)
	{
		if((*surfNode)[i][1]<-180)
		{
			colNodeIdx.push_back(i);
			deltaFree.push_back(((*surfNode)[i][1]+180));
			colNormal.push_back(Vec3f(0,1,0));
		}
	}

	// 2. Construct mapping matrix
	Matrix HCHT, HT;
	std::vector<int> relatedNodeIdx;
	constructMappingMatrixSparse(obj, colNodeIdx, colNormal, relatedNodeIdx, HT, HCHT, dt);
	//constructMappingMatrix(obj, colNodeIdx, colNormal, relatedNodeIdx, HT, HCHT, dt);

	// 3. LCP solver
	Matrix contactForce;
	GaussSeidelMethod(HCHT, deltaFree, contactForce, 30);

	// 4. Add force
	Matrix f=HT*contactForce;
	for(int i=0;i<relatedNodeIdx.size();i++)
	{
		for(int j=0;j<3;j++)
			(*nodeForce)[relatedNodeIdx[i]][j]+=f(i*3+j,0);
	}
}

void MeshfreeContactManager::contactResolutionWithTool(Meshfree_CPU* obj, CollisionManager* collision, float dt)
{
	//Obj=obj;
	//std::vector<CollisionManager::Distancefield> distance;
	//distance=collision->getDistance();

	//SurfaceObj* surfObj=obj->surfObj();
	//std::vector<Vec3f>* surfNode=surfObj->point();
	//std::vector<std::vector<int>>* neighborNodeIdx=obj->neighborNodeOfSurfVertice();
	//std::vector<std::vector<float>>* shapeFuncValue=obj->shapeFuncValueAtSurfPoint();

	//EFG_CPU* efgObj=obj->efgObj();
	//std::vector<float>* nodeMass=efgObj->nodeMass();
	//std::vector<Vec3f>* nodeForce=efgObj->nodeForce();
	//std::vector<int> colPointIdx;
	//std::vector<Vec3f> colNormal;
	//std::vector<float> deltaFree;

	//// 1. Collision detection
	////CollisionManager collision;
	////float minGap=5;
	////collision.PQBtwSurfaceObj(surfObj, environment, colPointIdx, colNormal, deltaFree, minGap);
	////for(int i=0;i<colPointIdx.size();i++)
	////{
	////	fprintf(F,"%d %f %f %f %f\n",i, deltaFree[i], colNormal[i][0],colNormal[i][1],colNormal[i][2]);
	////}

	//for(int i=0;i<distance.size();i++)
	//{
	//	colPointIdx.push_back(distance[i].)
	//	deltaFree.push_back(distance[i].Penetration);
	//	colNormal.push_back(distance[i].PenetratedDirec);
	//}

	//// 2. Construct mapping matrix
	//Matrix HCHT, HT;
	//std::vector<int> relatedNodeIdx;
	//constructMappingMatrixSparse(obj, colPointIdx, colNormal, relatedNodeIdx, HT, HCHT, dt);
	////constructMappingMatrix(obj, colNodeIdx, colNormal, relatedNodeIdx, HT, HCHT, dt);

	//// 3. LCP solver
	//Matrix contactForce;
	//GaussSeidelMethod(HCHT, deltaFree, contactForce, 30);

	//// 4. Add force
	//Matrix f=HT*contactForce;
	//for(int i=0;i<relatedNodeIdx.size();i++)
	//{
	//	for(int j=0;j<3;j++)
	//		(*nodeForce)[relatedNodeIdx[i]][j]+=f(i*3+j,0);
	//}
}

void MeshfreeContactManager::constructMappingMatrix(Meshfree_CPU* obj, std::vector<int>& colNodeIdx, std::vector<Vec3f>& colNormal, std::vector<int>& relatedNodeIdx, Matrix& HT, Matrix& HCHT, float dt)
{
	// 1. Get data
	VectorFunc func; 
	
	SurfaceObj* surfObj=obj->surfObj();
	std::vector<Vec3f>* surfNode=surfObj->point();
	std::vector<std::vector<int>>* neighborNodeIdx=obj->neighborNodeOfSurfVertice();
	std::vector<std::vector<float>>* shapeFuncValue=obj->shapeFuncValueAtSurfPoint();

	EFG_CPU* efgObj=obj->efgObj();
	std::vector<float>* nodeMass=efgObj->nodeMass();
	std::vector<Vec3f>* nodeForce=efgObj->nodeForce();
	
	// 2. Collision等 surface node客 包访等 EFG node甫 沥纺
	for(int i=0;i<colNodeIdx.size();i++)
	{
		for(int j=0;j<(*neighborNodeIdx)[colNodeIdx[i]].size();j++)
		{
			relatedNodeIdx.push_back((*neighborNodeIdx)[colNodeIdx[i]][j]);
		}
	}
	func.arrangeVector(relatedNodeIdx);
	
	// 3. Local index map 积己
	std::map<int,int> indexMap;
	for(int i=0;i<relatedNodeIdx.size();i++)
		indexMap.insert(std::pair<int,int>(relatedNodeIdx[i],i));

	// 4. H matrix 备己
	Matrix H(colNodeIdx.size(),relatedNodeIdx.size()*3);
	for(int i=0;i<colNodeIdx.size();i++)
	{
		for(int j=0;j<(*neighborNodeIdx)[colNodeIdx[i]].size();j++)
		{
			int neiIdx=(*neighborNodeIdx)[colNodeIdx[i]][j];
			int localIdx=indexMap[neiIdx];
			for(int k=0;k<3;k++)
				H(i,localIdx*3+k)=(*shapeFuncValue)[colNodeIdx[i]][j]*colNormal[i][k];
		}
	}

	// 5. HCHT matrix 备己
	HT=H.transpose();
	Matrix _HT=HT;
	for(int i=0;i<relatedNodeIdx.size();i++)
	{
		for(int j=0;j<HT.column();j++)
		{
			_HT(i*3,j)=HT(i*3,j)/(*nodeMass)[relatedNodeIdx[i]]*dt*dt;
			_HT(i*3+1,j)=HT(i*3+1,j)/(*nodeMass)[relatedNodeIdx[i]]*dt*dt;
			_HT(i*3+2,j)=HT(i*3+2,j)/(*nodeMass)[relatedNodeIdx[i]]*dt*dt;
		}
	}
	HCHT=H*_HT;
}

void MeshfreeContactManager::constructMappingMatrixSparse(Meshfree_CPU* obj, std::vector<int>& colNodeIdx, std::vector<Vec3f>& colNormal, std::vector<int>& relatedNodeIdx, Matrix& HT, Matrix& HCHT, float dt)
{
	// 1. Get data
	VectorFunc func;

	SurfaceObj* surfObj=obj->surfObj();
	std::vector<Vec3f>* surfNode=surfObj->point();
	std::vector<std::vector<int>>* neighborNodeIdx=obj->neighborNodeOfSurfVertice();
	std::vector<std::vector<float>>* shapeFuncValue=obj->shapeFuncValueAtSurfPoint();

	EFG_CPU* efgObj=obj->efgObj();
	std::vector<float>* nodeMass=efgObj->nodeMass();
	std::vector<Vec3f>* nodeForce=efgObj->nodeForce();

	// 2. Collision等 surface node客 包访等 EFG node甫 沥纺
	int nbElement=0;
	for(int i=0;i<colNodeIdx.size();i++)
	{
		for(int j=0;j<(*neighborNodeIdx)[colNodeIdx[i]].size();j++)
		{
			relatedNodeIdx.push_back((*neighborNodeIdx)[colNodeIdx[i]][j]);
			nbElement++;
		}
	}
	func.arrangeVector(relatedNodeIdx);

	// 3. Local index map 积己
	std::map<int,int> indexMap;
	for(int i=0;i<relatedNodeIdx.size();i++)
		indexMap.insert(std::pair<int,int>(relatedNodeIdx[i],i));

	// 4. H matrix 备己
	HT.init(relatedNodeIdx.size()*3,colNodeIdx.size());
	SparseMatrix H_S(colNodeIdx.size(),relatedNodeIdx.size()*3,nbElement*3);
	SparseMatrix HT_S(relatedNodeIdx.size()*3,colNodeIdx.size(),nbElement*3);
	for(int i=0;i<colNodeIdx.size();i++)
	{
		for(int j=0;j<(*neighborNodeIdx)[colNodeIdx[i]].size();j++)
		{
			int neiIdx=(*neighborNodeIdx)[colNodeIdx[i]][j];
			int localIdx=indexMap[neiIdx];
			
			for(int k=0;k<3;k++)
			{
				float value=(*shapeFuncValue)[colNodeIdx[i]][j]*colNormal[i][k];
				H_S.storeMatrix(i,localIdx*3+k,value);
				HT(localIdx*3+k,i)=value;
				HT_S.storeMatrix(localIdx*3+k,i,value/(*nodeMass)[neiIdx]*dt*dt);
			}
		}
	}

	// 5. HCHT matrix 备己
	HCHT.init(colNodeIdx.size(),colNodeIdx.size());
	SparseMatrix HCHT_S;
	HCHT_S=H_S.Multiply(HT_S);
	for(int i=0;i<HCHT_S.Terms;i++)
	{
		int rowIdx=HCHT_S.smArray[i].row;
		int colIdx=HCHT_S.smArray[i].col;
		float value=HCHT_S.smArray[i].value;
		HCHT(rowIdx,colIdx)=value;
	}
}

void MeshfreeContactManager::constructMappingMatrixSparseV2(Meshfree_CPU* obj, std::vector<int>& colTriIndex, std::vector<Vec3f>& colNormal, std::vector<int>& relatedNodeIdx, Matrix& HT, Matrix& HCHT, float dt)
{
	// 1. Get data
	VectorFunc func;

	SurfaceObj* surfObj=obj->surfObj();
	std::vector<Vec3f>* surfNode=surfObj->point();
	std::vector<std::vector<int>>* neighborNodeIdx=obj->neighborNodeOfSurfVertice();
	std::vector<std::vector<float>>* shapeFuncValue=obj->shapeFuncValueAtSurfPoint();
	std::vector<Vec3i>* FaceIndex=surfObj->face();

	EFG_CPU* efgObj=obj->efgObj();
	std::vector<float>* nodeMass=efgObj->nodeMass();
	std::vector<Vec3f>* nodeForce=efgObj->nodeForce();

	// 2. Collision等 surface node客 包访等 EFG node甫 沥纺
	int nbElement=0;
	for(int i=0;i<colTriIndex.size();i++)
	{
		for(int j=0;j<3;j++)
		{
			int NodeIndex=(*FaceIndex)[colTriIndex[i]][j];
		
			for(int k=0;k<(*neighborNodeIdx)[NodeIndex].size();k++)
			{
				relatedNodeIdx.push_back((*neighborNodeIdx)[NodeIndex][k]);
				nbElement++;
			}
		}
	}
	func.arrangeVector(relatedNodeIdx);

	// 3. Local index map 积己
	std::map<int,int> indexMap;
	for(int i=0;i<relatedNodeIdx.size();i++)
		indexMap.insert(std::pair<int,int>(relatedNodeIdx[i],i));

	int NbColNode=colTriIndex.size();
	// 4. H matrix 备己
	HT.init(relatedNodeIdx.size()*3,NbColNode);
	SparseMatrix H_S(NbColNode,relatedNodeIdx.size()*3,nbElement*3);
	SparseMatrix HT_S(relatedNodeIdx.size()*3,NbColNode,nbElement*3);
	/*for(int i=0;i<NbColNode;i++)
	{
		NodeIndex
		for(int j=0;j<(*neighborNodeIdx)[colNodeIdx[i]].size();j++)
		{
			int neiIdx=(*neighborNodeIdx)[colNodeIdx[i]][j];
			int localIdx=indexMap[neiIdx];

			for(int k=0;k<3;k++)
			{
				float value=(*shapeFuncValue)[colNodeIdx[i]][j]*colNormal[i][k];
				H_S.storeMatrix(i,localIdx*3+k,value);
				HT(localIdx*3+k,i)=value;
				HT_S.storeMatrix(localIdx*3+k,i,value/(*nodeMass)[neiIdx]*dt*dt);
			}
		}
	}*/

	//double A;
	//double Ai;
	//double N;
	//int ii[2];
	//int Index;
	//int NodeIndex;
	//int triIdx;

	////阿阿狼 伙阿屈俊 措秦
	//for(int I=0;I<NbColNode;I++)
	//{
	//	triIdx=colTriIndex[I];
	//	


	//	A=(((*surfNode)[(*FaceIndex)[triIdx][1]]-(*surfNode)[(*FaceIndex)[triIdx][0]]).cross((*surfNode)[(*FaceIndex)[triIdx][2]]-(*surfNode)[(*FaceIndex)[triIdx][0]])).norm();

	//	//伙阿屈 郴何狼 阿 3痢俊 措秦
	//	for(int i=0;i<3;i++)
	//	{
	//		ii[0]=(i+1)%3;
	//		ii[1]=(i+2)%3;
	//		//伙阿屈 厚啦
	//		Ai=(((*surfNode)[(*FaceIndex)[triIdx][ii[1]]]-(*surfNode)[(*FaceIndex)[triIdx][ii[0]]]).cross(Pos-(*surfNode)[(*FaceIndex)[triIdx][ii[0]]])).norm();
	//		N=Ai/A;

	//		//痢 锅龋
	//		Index=(*FaceIndex)[triIdx][i];

	//		for(int j=0;j<(*neighborNodeIdx)[Index].size();j++)
	//		{	
	//			NodeIndex=(*neighborNodeIdx)[Index][j];

	//			//m[NodeIndex*3]+=ShapeFuncValueAtSurfPoint[Index][j]*normal[0]*N;
	//			//m[NodeIndex*3+1]+=ShapeFuncValueAtSurfPoint[Index][j]*normal[1]*N;
	//			//m[NodeIndex*3+2]+=ShapeFuncValueAtSurfPoint[Index][j]*normal[2]*N;

	//			float value=(*shapeFuncValue)[Index][j]*colNormal[i][k];
	//			H_S.storeMatrix(i,localIdx*3+k,value);
	//			HT(localIdx*3+k,i)=value;
	//			HT_S.storeMatrix(localIdx*3+k,i,value/(*nodeMass)[neiIdx]*dt*dt);
	//		}

	//	}

	//}

	//// 5. HCHT matrix 备己
	//HCHT.init(colNodeIdx.size(),colNodeIdx.size());
	//SparseMatrix HCHT_S;
	//HCHT_S=H_S.Multiply(HT_S);
	//for(int i=0;i<HCHT_S.Terms;i++)
	//{
	//	int rowIdx=HCHT_S.smArray[i].row;
	//	int colIdx=HCHT_S.smArray[i].col;
	//	float value=HCHT_S.smArray[i].value;
	//	HCHT(rowIdx,colIdx)=value;
	//}
}

void MeshfreeContactManager::constructMappingMatrixSparse(Meshfree_GPU* obj, std::vector<int>& colNodeIdx, std::vector<Vec3f>& colNormal, std::vector<int>& relatedNodeIdx, Matrix& HT, Matrix& HCHT, float dt)
{
	// 1. Get data
	VectorFunc func;

	SurfaceObj* surfObj=obj->surfObj();
	std::vector<Vec3f>* surfNode=surfObj->point();
	std::vector<std::vector<int>>* neighborNodeIdx=obj->neighborNodeOfSurfVertice();
	std::vector<std::vector<float>>* shapeFuncValue=obj->shapeFuncValueAtSurfPoint();

	EFG_CUDA_RUNTIME* efgObj=obj->efgObj();
	float* nodeMass=efgObj->nodeMass();

	// 2. Collision等 surface node客 包访等 EFG node甫 沥纺
	int nbElement=0;
	for(int i=0;i<colNodeIdx.size();i++)
	{
		for(int j=0;j<(*neighborNodeIdx)[colNodeIdx[i]].size();j++)
		{
			relatedNodeIdx.push_back((*neighborNodeIdx)[colNodeIdx[i]][j]);
			nbElement++;
		}
	}
	func.arrangeVector(relatedNodeIdx);

	// 3. Local index map 积己
	std::map<int,int> indexMap;
	for(int i=0;i<relatedNodeIdx.size();i++)
		indexMap.insert(std::pair<int,int>(relatedNodeIdx[i],i));

	// 4. H matrix 备己
	HT.init(relatedNodeIdx.size()*3,colNodeIdx.size());
	SparseMatrix H_S(colNodeIdx.size(),relatedNodeIdx.size()*3,nbElement*3);
	SparseMatrix HT_S(relatedNodeIdx.size()*3,colNodeIdx.size(),nbElement*3);
	for(int i=0;i<colNodeIdx.size();i++)
	{
		for(int j=0;j<(*neighborNodeIdx)[colNodeIdx[i]].size();j++)
		{
			int neiIdx=(*neighborNodeIdx)[colNodeIdx[i]][j];
			int localIdx=indexMap[neiIdx];

			for(int k=0;k<3;k++)
			{
				float value=(*shapeFuncValue)[colNodeIdx[i]][j]*colNormal[i][k];
				H_S.storeMatrix(i,localIdx*3+k,value);
				HT(localIdx*3+k,i)=value;
				HT_S.storeMatrix(localIdx*3+k,i,value/nodeMass[neiIdx]*dt*dt);
			}
		}
	}

	// 5. HCHT matrix 备己
	HCHT.init(colNodeIdx.size(),colNodeIdx.size());
	SparseMatrix HCHT_S;
	HCHT_S=H_S.Multiply(HT_S);
	for(int i=0;i<HCHT_S.Terms;i++)
	{
		int rowIdx=HCHT_S.smArray[i].row;
		int colIdx=HCHT_S.smArray[i].col;
		float value=HCHT_S.smArray[i].value;
		HCHT(rowIdx,colIdx)=value;
	}
}

void MeshfreeContactManager::GaussSeidelMethod(Matrix& hcht, std::vector<float>& deltaFree, Matrix& force, int Iteration)
{
	int nbCollide=hcht.row();
	float displacement;

	force.init(nbCollide,1);
	std::vector<float> delta;
	delta.resize(nbCollide);
	for(int i=0;i<nbCollide;i++)
	{
		delta[i]=0;
	}

	for(int it=0;it<Iteration;it++)
	{
		for(int i=0;i<nbCollide;i++)
		{
			displacement=0;

			for(int j=0;j<nbCollide;j++)
				if(i!=j)
					displacement+=hcht(i,j)*force(j,0);
			delta[i]=deltaFree[i]+displacement;

			if(delta[i]>0){
				force(i,0)=0;
			}else{
				force(i,0)=-delta[i]/(hcht(i,i));
			}
		}
	}
}
