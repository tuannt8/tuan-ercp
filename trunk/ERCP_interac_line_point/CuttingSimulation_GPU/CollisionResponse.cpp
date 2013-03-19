#include "StdAfx.h"
#include "CollisionResponse.h"


CollisionResponse::CollisionResponse(void)
{
	c=NULL;
}

CollisionResponse::~CollisionResponse(void)
{
	delete [] c;
	c=NULL;
}

void CollisionResponse::ComputeforcefromComplianceV2(float dt, int itter, Meshfree_GPU* surf,CollisionManager* collision)
{
	
	std::vector<CollisionManager::Distancefield> distance;
	distance=collision->getDistance();

	int Nb1=surf->efgObj()->nbNode();
	int Nb2=2;
	int nbCollide=distance.size();	

	Vec3d normal;

	Cmat InternalDis1(3*Nb1);
	Cmat C1(3*Nb1);


	Cmat m1,mm1;	
	Cmat mcm;
	Cmat force;
	Cmat penetration;
	Cmat delta;
	

	

	//Internal Displacement
	float** DisAddress=surf->efgObj()->returnPreDis(dt,itter);
	
	
	if(nbCollide){

		m1.init(nbCollide,3*Nb1);
		mm1.init(nbCollide,3*Nb1);
		mcm.init(nbCollide,nbCollide);
		force.init(nbCollide);
		penetration.init(nbCollide);
		delta.init(nbCollide);
		
		for(int i=0;i<Nb1*3;i++)
		{
			InternalDis1(i)=(*DisAddress)[i];
		}

	//	InternalDis1.print("Internal.txt");

		for(int i=0;i<nbCollide;i++){
			penetration(i)=distance[i].Penetration;
		}
	//	penetration.print("pene1.txt");
		double* mapping;
		mapping=new double[3*Nb1];
		for(int i=0;i<3*Nb1;i++)
		{
			mapping[i]=0;
		}

	//	SparseMatrix sM;

		for(int i=0;i<nbCollide;i++)
		{
			normal=distance[i].PenetratedDirec;

			surf->returnMappingMatrix(distance[i].triIdx,distance[i].collidedTriPoint,normal,mapping);
			for(int j=0;j<3*Nb1;j++){
				m1(i,j)=mapping[j];
		//		sM.storeMatrix(i,j,mapping[j]);
			}

		}

		delete [] mapping;
		mapping=NULL;
	

		penetration-=m1*(InternalDis1);
	//	penetration.print("pene2.txt");

		//Construct MCM
		static bool flag=true;
		if(flag)
		{
			c=new double[3*Nb1];
			surf->efgObj()->returnVertorFormExplicitCompliancematrix(dt,itter,c);
			flag=false;
		}
		
		C1.set(c);
	//	
	//	m1.print("m1.txt");
		mm1=m1%C1;

	//	m1.print("m1.txt");
	//	mm1.print("mm1.txt");


	//	//mm1.fill(1);
	//	//m1.fill(1);
		mcm=mm1.transMul2(m1);
	//	mcm.print("mcm.txt");


		//Gauss Seidel 
		int Iteration=10;

		double displacement;
		

		for(int it=0;it<Iteration;it++)
		{
			for(int i=0;i<nbCollide;i++)
			{
				displacement=0;

				for(int j=0;j<nbCollide;j++)
					if(i!=j)
						displacement+=mcm(i,j)*force(j);
				delta(i)=penetration(i)+displacement;

				if(delta(i)>0){
					force(i)=0;
				}else{
					if(abs(mcm(i,i))>0.00000000001){
						force(i)=-delta(i)/(mcm(i,i));
					}else
					{
						force(i)=-delta(i)/0.00000000001;
					}
					
				}
			}
		}

	//	force.print("force.txt");
	//	
		InternalDis1=-mm1.transMul(force);
	//	InternalDis1.print("dis.txt");
//		InternalDis1.fill(0);
	}

	surf->efgObj()->updatePosition(InternalDis1.returnVectorForm());
	surf->updateSurfPosition();

}

void CollisionResponse::ComputeforcefromComplianceV3(float dt, int itter, Meshfree_GPU* surf,CollisionManager* collision)
{

	std::vector<CollisionManager::Distancefield> distance;
	distance=collision->getDistance();

	int Nb1=surf->efgObj()->nbNode();
	int Nb2=2;
	int nbCollide=distance.size();	

	Vec3d normal;

	Cmat Displacement1(3*Nb1);
	Cmat InternalDis1(3*Nb1);
	Cmat C1(3*Nb1);


	Cmat m1,mm1;	
	Cmat mcm;
	Cmat force;
	Cmat penetration;





	//Internal Displacement

	float** DisAddress=surf->efgObj()->returnPreDis(dt,itter);



	//	Displacement1.print("dis.txt");


	if(nbCollide){

		m1.init(nbCollide,3*Nb1);
		mm1.init(nbCollide,3*Nb1);
		mcm.init(nbCollide,nbCollide);
		force.init(nbCollide);
		penetration.init(nbCollide);
		InternalDis1.init(3*Nb1);


		for(int i=0;i<nbCollide;i++){
			penetration(i)=distance[i].Penetration;
		}
		penetration.print("pene1.txt");

		std::vector<Vec2d> mapping;	
		mapping.clear();
		std::vector<int> Index;
		Index.clear();


		int start=0;
		int nb=0;
		//Reduced mapping
		for(int i=0;i<nbCollide;i++)
		{

			normal=distance[i].PenetratedDirec;
			surf->returnVectorMappingMatrix(distance[i].triIdx,distance[i].collidedTriPoint,normal,mapping);
			Index.push_back(nb);
			nb=mapping.size();
			
		}
		Index.push_back(nb);

		Cmat check(nbCollide,3*Nb1);
		Cmat check2(nb,3);
		nb=0;
		int check212=Index.size();

		for(int i=start;i<nbCollide;i++)
		{
			for(int j=0;j<Index[i+1]-Index[i];j++)
			{
				check(i,mapping[Index[i]+j][0])=check(i,mapping[Index[i]+j][0])+mapping[Index[i]+j][1];
				check2(nb,0)=i;
				check2(nb,1)=mapping[Index[i]+j][0];
				check2(nb++,2)=mapping[Index[i]+j][1];
			}
		}
		check.print("check.txt");

		
		check2.print("Cmapping.txt");
		//full mapping
		double* Mapping;
		Mapping=new double[3*Nb1];
		for(int i=0;i<3*Nb1;i++)
		{
			Mapping[i]=0;
		}

		for(int i=start;i<nbCollide;i++)
		{
			normal=distance[i].PenetratedDirec;

			surf->returnMappingMatrix(distance[i].triIdx,distance[i].collidedTriPoint,normal,Mapping);
			for(int j=0;j<3*Nb1;j++)
			{
				m1(i,j)=Mapping[j];
			}

		}

		delete [] Mapping;
		Mapping=NULL;

		Cmat check3;
		check3=check-m1;
		check3.print("checkSame.txt");
		m1.print("m1.txt");

		
		int temp;
		double tempV,tempV2;
		
		//임시
		penetration.fill(0);
		//
		

		for(int i=0;i<Nb1*3;i++)
		{
			InternalDis1(i)=(*DisAddress)[i];
		}


		penetration=-m1*InternalDis1;

		penetration.print("pene2.txt");

		
		//임시
		penetration.fill(0);
		//

		for(int i=0;i<nbCollide;i++)
		{
			for(int j=0;j<Index[i+1]-Index[i];j++)
			{
				temp=mapping[Index[i]+j][0];
				tempV=mapping[Index[i]+j][1];

				penetration(i)-=tempV*(*DisAddress)[temp];
			}
		}


		penetration.print("pene3.txt");


		

		//Construct MCM


		for(int i=0;i<nbCollide;i++)
		{
			for(int j=0;j<nbCollide;j++)
			{
				for(unsigned int k=0;k<Index[i+1]-Index[i];k++)
				{
					if(i==j)
					{
						temp=mapping[Index[i]+k][0];
						tempV=mapping[Index[i]+k][1];
						tempV2=mapping[Index[i]+k][1];
						mcm(i,j)+=tempV*tempV2*dt*dt*itter*(itter+1)/2;;
					}
					else
					{
						for(unsigned int l=0;l<Index[j+1]-Index[j];l++)
						{
							temp=mapping[Index[i]+k][0];
							if(temp==mapping[Index[j]+l][0])
							{
								tempV=mapping[Index[i]+k][1];
								tempV2=mapping[Index[j]+l][1];
								mcm(i,j)+=tempV*tempV2*dt*dt*itter*(itter+1)/2;;

							}
						}

					}
					
				}
			}
		}
		mcm.print("mcm.txt");

		//Gauss Seidel 
		int Iteration=10;

		double displacement;
		Cmat delta(nbCollide);

		for(int it=0;it<Iteration;it++)
		{
			for(int i=0;i<nbCollide;i++)
			{
				displacement=0;

				for(int j=0;j<nbCollide;j++)
					if(i!=j)
						displacement+=mcm(i,j)*force(j);
				delta(i)=penetration(i)+displacement;

				if(delta(i)>0){
					force(i)=0;
				}else{
					force(i)=-delta(i)/(mcm(i,i));
				}
			}
		}

		force.print("force.txt");


		for(int i=0;i<nbCollide;i++)
		{
			for(int j=0;j<Index[i+1]-Index[i];j++)
			{
				temp=mapping[Index[i]+j][0];
				tempV=-mapping[Index[i]+j][1]*C1(temp)*force(i);
				if(tempV)
				{
					surf->efgObj()->addDisplacement(temp,tempV);
				}
			}
		}

		
		
	}

	

	surf->efgObj()->updatePosition();
	surf->updateSurfPosition();

}

void CollisionResponse::ComputeforcefromComplianceV4(float dt, int itter, Meshfree_GPU* surf,CollisionManager* collision)
{

	std::vector<CollisionManager::Distancefield> distance;
	distance=collision->getDistance();

	int Nb1=surf->efgObj()->nbNode();
	int Nb2=2;
	int nbCollide=distance.size();	

	Vec3d normal;

	Cmat Displacement1(3*Nb1);
	Cmat InternalDis1(3*Nb1);
	Cmat C1(3*Nb1);


	Cmat m1,mm1;	
	Cmat mcm;
	Cmat force;
	Cmat penetration;





	//Internal Displacement

	float** DisAddress=surf->efgObj()->returnPreDis(dt,itter);





	if(nbCollide){

		m1.init(nbCollide,3*Nb1);
		mm1.init(nbCollide,3*Nb1);
		mcm.init(nbCollide,nbCollide);
		force.init(nbCollide);
		penetration.init(nbCollide);

		for(int i=0;i<Nb1*3;i++)
		{
			InternalDis1(i)=(*DisAddress)[i];
		}

		InternalDis1.print("Internal.txt");

		for(int i=0;i<nbCollide;i++){
			penetration(i)=distance[i].Penetration;
		}
		penetration.print("pene1.txt");


		for(int i=0;i<nbCollide;i++)
		{
			normal=distance[i].PenetratedDirec;

			double* mapping=surf->returnMappingMatrix(distance[i].triIdx,distance[i].collidedTriPoint,normal);
			for(int j=0;j<3*Nb1;j++){
				m1(i,j)=mapping[j];
			}

		}


		penetration-=m1*(InternalDis1);
		penetration.print("pene2.txt");



		//Construct MCM
		C1.set(surf->efgObj()->returnVertorFormExplicitCompliancematrix(dt,itter));
		C1.print("C1.txt");
		m1.print("m1.txt");
		mm1=(m1.T())%C1;
		mcm=mm1.T()*m1.T();
		mm1.print("mm1.txt");
		mcm.print("mcm.txt");

		//Gauss Seidel 
		int Iteration=15;

		double displacement;
		Cmat delta(nbCollide);

		for(int it=0;it<Iteration;it++)
		{
			for(int i=0;i<nbCollide;i++)
			{
				displacement=0;

				for(int j=0;j<nbCollide;j++)
					if(i!=j)
						displacement+=mcm(i,j)*force(j);
				delta(i)=penetration(i)+displacement;

				if(delta(i)>0){
					force(i)=0;
				}else{
					force(i)=-delta(i)/(mcm(i,i));
				}
			}
		}

		force.print("force.txt");

		InternalDis1=mm1*force;
		InternalDis1.print("dis.txt");



	}

	double* Temp=InternalDis1.returnVectorForm();
	double a=Temp[5];

	surf->efgObj()->updatePosition(InternalDis1.returnVectorForm());
	surf->updateSurfPosition();

}

void CollisionResponse::ComputeforcefromComplianceV5(float dt, int itter, Meshfree_GPU* surf,CollisionManager* collision)
{

	std::vector<CollisionManager::Distancefield> distance;
	distance=collision->getDistance();

	int Nb1=surf->efgObj()->nbNode();
	int Nb2=2;
	int nbCollide=distance.size();	

	Vec3d normal;

	Cmat InternalDis1(3*Nb1);
	Cmat C1(3*Nb1);


	Cmat m1,mcm,force,penetration,delta;




	//Internal Displacement
	float** DisAddress=surf->efgObj()->returnPreDis(dt,itter);


	if(nbCollide){

		m1.init(nbCollide,3*Nb1);
		mcm.init(nbCollide,nbCollide);
		force.init(nbCollide);
		penetration.init(nbCollide);
		delta.init(nbCollide);

		for(int i=0;i<Nb1*3;i++)
		{
			InternalDis1(i)=(*DisAddress)[i];
		}

		for(int i=0;i<nbCollide;i++){
			penetration(i)=distance[i].Penetration;
		}

		double* mapping;
		mapping=new double[3*Nb1];
		for(int i=0;i<3*Nb1;i++)
		{
			mapping[i]=0;
		}

		//	SparseMatrix sM;

		for(int i=0;i<nbCollide;i++)
		{
			normal=distance[i].PenetratedDirec;

			surf->returnMappingMatrix(distance[i].triIdx,distance[i].collidedTriPoint,normal,mapping);
			for(int j=0;j<3*Nb1;j++){
				m1(i,j)=mapping[j];
				//		sM.storeMatrix(i,j,mapping[j]);
			}

		}

		delete [] mapping;
		mapping=NULL;
		penetration-=m1*(InternalDis1);


		//Construct MCM
		mcm=(m1.transMul2(m1))*dt*dt*itter*(itter+1)/2;



		//Gauss Seidel 
		int Iteration=10;

		double displacement;



		for(int it=0;it<Iteration;it++)
		{
			for(int i=0;i<nbCollide;i++)
			{
				displacement=0;

				for(int j=0;j<nbCollide;j++)
					if(i!=j)
						displacement+=mcm(i,j)*force(j);
				delta(i)=penetration(i)+displacement;

				if(delta(i)>0){
					force(i)=0;
				}else{
					force(i)=-delta(i)/(mcm(i,i));

				}
			}
		}

		InternalDis1=(-m1.transMul(force))*dt*dt*itter*(itter+1)/2;
	}

	surf->efgObj()->updatePosition(InternalDis1.returnVectorForm());
	surf->updateSurfPosition();

}

void CollisionResponse::ComputeforcefromComplianceV6(float dt, int itter, Meshfree_GPU* surf,CollisionManager* collision)
{

	std::vector<CollisionManager::Distancefield> distance;
	distance=collision->getDistance();

	int Nb1=surf->efgObj()->nbNode();
	int nbCollide=distance.size();
	float* nodeForce=surf->efgObj()->nodeForce();
	float* nodeMass=surf->efgObj()->nodeMass();
	std::vector<std::vector<int>>* neighborNodeIdx=surf->neighborNodeOfSurfVertice();
	std::vector<Vec3i>* face=surf->surfObj()->face();
	std::vector<float> deltaFree;
	
	VectorFunc func;
	Vec3d normal;

	std::vector<int> relatedNodeIdx;



	Matrix HCHT, HT;
	HCHT.init(nbCollide,nbCollide);	
	HT.init(3*Nb1,nbCollide);


	//Cmat check(3*Nb1);
	//check.set(nodeMass);
	//check.print("NodeMass.txt");

		
	for(int i=0;i<nbCollide;i++){	
		deltaFree.push_back(distance[i].Penetration);
	}


	int nbElement=0;
	int NodeIndex;
	for(int i=0;i<nbCollide;i++)
	{
		for(int k=0;k<3;k++)
		{
			NodeIndex=(*face)[i][k];
			for(int j=0;j<(*neighborNodeIdx)[NodeIndex].size();j++)
			{
				relatedNodeIdx.push_back((*neighborNodeIdx)[NodeIndex][j]);
				nbElement++;
			}
		}
	}


	func.arrangeVector(relatedNodeIdx);

	SparseMatrix H_S(nbCollide,relatedNodeIdx.size()*3,nbElement*3);
	SparseMatrix HT_S(relatedNodeIdx.size()*3,nbCollide,nbElement*3);


	std::vector<Vec2f> Map;
	for(int i=0;i<nbCollide;i++)
	{
		normal=distance[i].PenetratedDirec;
		Map.clear();
		surf->returnMappingMatrix(distance[i].triIdx,distance[i].collidedTriPoint,normal,Map);
			
		func.arrangeVector(Map);
		Matrix check(Map.size(),2);
		for(int kk=0;kk<Map.size();kk++)
		{
			check(kk,0)=Map[kk][0];
			check(kk,1)=Map[kk][1];
		}
		check.Print("Mapping check1.txt");

		for(int j=0;j<Map.size();j++){	
				
			int neiIdx=Map[j][0]/3;
			H_S.storeMatrix(i,Map[j][0],Map[j][1]);
			float C;
			C=Map[j][1];
			C=Map[j][1]*dt*dt*itter*(itter+1)/(2*nodeMass[0])*1000000;
			
			C=dt*dt*itter*(itter+1)/(2*nodeMass[0])*1000000;
			
			HT(Map[j][0],i)=Map[j][1];
			HT_S.storeMatrix(Map[j][0],i,Map[j][1]*C);
		}
	}
		
	//HT.Print("HT.txt");
	


	//Construct MCM
	SparseMatrix HCHT_S(nbCollide,nbCollide,nbCollide*nbCollide);
	HCHT_S=H_S.Multiply(HT_S);
	for(int i=0;i<nbCollide*nbCollide;i++)
	{
		int rowIdx=HCHT_S.smArray[i].row;
		int colIdx=HCHT_S.smArray[i].col;
		float value=HCHT_S.smArray[i].value;
		HCHT(rowIdx,colIdx)=value;
	}
	//HCHT.Print("HCHT.txt");
		

	//Gauss Seidel 
	int Iteration=10;

	Matrix contactForce;
	GaussSeidelMethod(HCHT, deltaFree, contactForce, 30);
	//contactForce.Print("contactForce.txt");
	// 4. Add force

	Matrix f=HT*contactForce;
	for(int i=0;i<Nb1;i++)
	{
		for(int j=0;j<3;j++)
		{
			nodeForce[i*3+j]=0;
		}
	}
	int NbRelated=relatedNodeIdx.size();

	/*Cmat check(relatedNodeIdx.size());
	for(int kk=0;kk<relatedNodeIdx.size();kk++)
	{
		check(kk)=relatedNodeIdx[kk];

	}*/
	//check.print("relatedNodeIdx.txt");

	for(int i=0;i<NbRelated;i++)
	{
		for(int j=0;j<3;j++){
			nodeForce[relatedNodeIdx[i]*3+j]+=f(relatedNodeIdx[i]*3+j,0)/1000000;
		}
	}
}

void CollisionResponse::ComputeforcefromComplianceV7(float dt, int itter, Meshfree_GPU* surf,CollisionManager* collision)
{

	std::vector<CollisionManager::Distancefield> distance;
	distance=collision->getDistance();

	int Nb1=surf->efgObj()->nbNode();
	int Nb2=2;
	int nbCollide=distance.size();	

	Vec3d normal;

	Cmat C1(3*Nb1);


	Cmat m1,mm1;	
	Cmat mcm;
	Cmat force;
	Cmat penetration;
	Cmat delta;


	float* nodeForce=surf->efgObj()->nodeForce();



	//Internal Displacement
	


	
	m1.init(nbCollide,3*Nb1);
	mm1.init(nbCollide,3*Nb1);
	mcm.init(nbCollide,nbCollide);
	force.init(nbCollide);
	penetration.init(nbCollide);
	delta.init(nbCollide);


	//	InternalDis1.print("Internal.txt");

	for(int i=0;i<nbCollide;i++){
		penetration(i)=distance[i].Penetration;
	}
		//	penetration.print("pene1.txt");
		double* mapping;
		mapping=new double[3*Nb1];
		for(int i=0;i<3*Nb1;i++)
		{
			mapping[i]=0;
		}

		//	SparseMatrix sM;

		for(int i=0;i<nbCollide;i++)
		{
			normal=distance[i].PenetratedDirec;

			surf->returnMappingMatrix(distance[i].triIdx,distance[i].collidedTriPoint,normal,mapping);
			for(int j=0;j<3*Nb1;j++){
				m1(i,j)=mapping[j];
				//		sM.storeMatrix(i,j,mapping[j]);
			}

		}

		delete [] mapping;
		mapping=NULL;

		//Construct MCM
		static bool flag=true;
		if(flag)
		{
			c=new double[3*Nb1];
			surf->efgObj()->returnVertorFormExplicitCompliancematrix(dt,itter,c);
			flag=false;
		}

		C1.set(c);
		//	
		//	m1.print("m1.txt");
		mm1=m1%C1;

		//	m1.print("m1.txt");
		//	mm1.print("mm1.txt");


		//	//mm1.fill(1);
		//	//m1.fill(1);
		mcm=mm1.transMul2(m1);
		//	mcm.print("mcm.txt");


		//Gauss Seidel 
		int Iteration=10;

		double displacement;


		for(int it=0;it<Iteration;it++)
		{
			for(int i=0;i<nbCollide;i++)
			{
				displacement=0;

				for(int j=0;j<nbCollide;j++)
					if(i!=j)
						displacement+=mcm(i,j)*force(j);
				delta(i)=penetration(i)+displacement;

				if(delta(i)>0){
					force(i)=0;
				}else{
					if(abs(mcm(i,i))>0.00000000001){
						force(i)=-delta(i)/(mcm(i,i));
					}else
					{
						force(i)=-delta(i)/0.00000000001;
					}

				}
			}
		}


	for(int i=0;i<Nb1;i++)
	{
		for(int j=0;j<3;j++)
		{
			nodeForce[i*3+j]=0;
		}
	}

	penetration=m1.transMul(force);

	for(int i=0;i<Nb1;i++)
	{
		for(int j=0;j<3;j++){
			nodeForce[i*3+j]=penetration(i*3+j,0);
		}
	}

		
	

}

void CollisionResponse::ComputeforcefromComplianceV8(float dt, int itter, Meshfree_GPU* surf,CollisionManager* collision)
{
	std::vector<CollisionManager::Distancefield> distance;
	distance=collision->getDistance();

	int Nb1=surf->efgObj()->nbNode();
	int Nb2=2;
	int nbCollide=distance.size();	

	Vec3d normal;



	mat m1;				m1.set_size(nbCollide,3*Nb1); m1.fill(0.0);
	mat mm1;			mm1.set_size(nbCollide,3*Nb1); mm1.fill(0.0);
	mat mcm;			mcm.set_size(nbCollide,nbCollide); mcm.fill(0.0);
	mat force;			force.set_size(nbCollide); force.fill(0.0);
	mat penetration;	penetration.set_size(nbCollide); penetration.fill(0.0);

	
	
	
	
	

	float* nodeMass=surf->efgObj()->nodeMass();
	float* nodeForce=surf->efgObj()->nodeForce();



	for(int i=0;i<nbCollide;i++){
		penetration(i)=distance[i].Penetration;
	}
	//penetration.save("penetration.txt",raw_ascii);
	double* mapping;
	mapping=new double[3*Nb1];
	for(int i=0;i<3*Nb1;i++)
	{
		mapping[i]=0;
	}


	for(int i=0;i<nbCollide;i++)
	{
		normal=distance[i].PenetratedDirec;

		surf->returnMappingMatrix(distance[i].triIdx,distance[i].collidedTriPoint,normal,mapping);
		for(int j=0;j<3*Nb1;j++){
			m1(i,j)=mapping[j];
		}
	}

	delete [] mapping;
	mapping=NULL;

	double C=dt*dt*itter*(itter+1)/(2*nodeMass[0]);

	mcm=(m1*trans(m1))*C;
	//mcm.save("mcm.txt",raw_ascii);


	//Gauss Seidel 
	int Iteration=20;

	force=GaussSeidelMethod1(mcm,penetration,Iteration);

	//force.save("force.txt",raw_ascii);

	for(int i=0;i<Nb1;i++)
	{
		for(int j=0;j<3;j++)
		{
			nodeForce[i*3+j]=0;
		}
	}

	force=trans(m1)*force;
	force.save("force.txt",raw_ascii);

	for(int i=0;i<Nb1;i++)
	{
		for(int j=0;j<3;j++){
			nodeForce[i*3+j]=-force(i*3+j,0);
		}
	}




}

void CollisionResponse::ComputeforcefromComplianceV9(float dt, int itter, Meshfree_GPU* surf,CollisionManager* collision)
{

	std::vector<CollisionManager::Distancefield> distance;
	distance=collision->getDistance();

	int Nb1=surf->efgObj()->nbNode();
	int Nb2=2;
	int nbCollide=distance.size();	

	Vec3d normal;

	mat m1;				m1.set_size(nbCollide,3*Nb1); m1.fill(0.0);
	mat mm1;			mm1.set_size(nbCollide,3*Nb1); mm1.fill(0.0);
	mat mcm;			mcm.set_size(nbCollide,nbCollide); mcm.fill(0.0);
	mat force;			force.set_size(nbCollide); force.fill(0.0);
	mat penetration;	penetration.set_size(nbCollide); penetration.fill(0.0);
	mat InternalDis;	InternalDis.set_size(3*Nb1),InternalDis.fill(0.0);

	//Internal Displacement
	float** DisAddress=surf->efgObj()->returnPreDis(dt,itter);
	float*  nodeMass=surf->efgObj()->nodeMass();

	double C=dt*dt*itter*(itter+1)/(2*nodeMass[0]);

	if(nbCollide){
		for(int i=0;i<Nb1*3;i++)
		{
			InternalDis(i)=(*DisAddress)[i];
		}


		for(int i=0;i<nbCollide;i++){
			penetration(i)=distance[i].Penetration;
		}
		//	penetration.print("pene1.txt");
		double* mapping;
		mapping=new double[3*Nb1];
		for(int i=0;i<3*Nb1;i++)
		{
			mapping[i]=0;
		}

		//	SparseMatrix sM;

		for(int i=0;i<nbCollide;i++)
		{
			normal=distance[i].PenetratedDirec;

			surf->returnMappingMatrix(distance[i].triIdx,distance[i].collidedTriPoint,normal,mapping);
			for(int j=0;j<3*Nb1;j++){
				m1(i,j)=mapping[j];
				//		sM.storeMatrix(i,j,mapping[j]);
			}

		}

		delete [] mapping;
		mapping=NULL;


		penetration-=m1*(InternalDis);
		//	penetration.print("pene2.txt");

		//Construct MCM
		

		mcm=(m1*trans(m1))*C;

		//Gauss Seidel 
		int Iteration=20;
		force=GaussSeidelMethod1(mcm,penetration,Iteration);

		//Internal Force
		InternalDis=-trans(m1)*force;
	}
	double* InternalForce;
	InternalForce=new double[3*Nb1];

	for(int i=0;i<3*Nb1;i++)
	{
		InternalForce[i]=InternalDis(i);
	}

	surf->efgObj()->updatePosition(InternalForce,C,dt,itter);
	surf->updateSurfPosition();
	delete [] InternalForce;
	InternalForce=NULL;
	

}

void CollisionResponse::ComputeforcefromComplianceV10(float dt, int itter, Meshfree_GPU* surf,CollisionManager* collision)
{
	std::vector<CollisionManager::Distancefield> distance;
	distance=collision->getDistance();

	int Nb1=surf->efgObj()->nbNode();
	int Nb2=2;
	int nbCollide=distance.size();	

	Vec3d normal;

	mat m1;				m1.set_size(nbCollide,3*Nb1); m1.fill(0.0);
	mat mm1;			mm1.set_size(nbCollide,3*Nb1); mm1.fill(0.0);
	mat mcm;			mcm.set_size(nbCollide,nbCollide); mcm.fill(0.0);
	mat force;			force.set_size(nbCollide); force.fill(0.0);
	mat penetration;	penetration.set_size(nbCollide); penetration.fill(0.0);
	mat InternalDis;	InternalDis.set_size(3*Nb1),InternalDis.fill(0.0);


	//Internal Displacement
	float** DisAddress=surf->efgObj()->returnPreDis(dt,itter);
	float*  nodeMass=surf->efgObj()->nodeMass();



	double C=dt*dt*itter*(itter+1)/(2*nodeMass[0]);



	if(nbCollide){
		for(int i=0;i<Nb1*3;i++)
		{
			InternalDis(i)=(*DisAddress)[i];
		}


		for(int i=0;i<nbCollide;i++){
			penetration(i)=distance[i].Penetration;
		}
		//	penetration.print("pene1.txt");
		double* mapping;
		mapping=new double[3*Nb1];
		

		//	SparseMatrix sM;

		for(int i=0;i<nbCollide;i++)
		{
			normal=distance[i].PenetratedDirec;

			surf->returnMappingMatrix(distance[i].triIdx,distance[i].collidedTriPoint,normal,mapping);
			for(int j=0;j<3*Nb1;j++){
				m1(i,j)=mapping[j];
				//		sM.storeMatrix(i,j,mapping[j]);
			}

		}


		delete [] mapping;
		mapping=NULL;

		penetration-=m1*(InternalDis);

		//Construct MCM
		mcm=(m1*trans(m1))*C;

		//Gauss Seidel 
		int Iteration=20;
		force=GaussSeidelMethod1(mcm,penetration,Iteration);

		//Internal Force
		InternalDis=-trans(m1)*force;
	}


	double* InternalForce;
	InternalForce=new double[3*Nb1];

	Vec3d transferdForce(0,0,100);

	for(int i=0;i<3*Nb1;i++)
	{
		InternalForce[i]=InternalDis(i);
	}
		
	surf->efgObj()->updatePosition(InternalForce,C,dt,itter);
	surf->updateSurfPosition();

	delete [] InternalForce;
	InternalForce=NULL;
}

void CollisionResponse::ComputeforcefromComplianceV11(float dt, int itter, Meshfree_GPU* surf,CollisionManager* collision)
{
	VectorFunc func;
	std::vector<CollisionManager::Distancefield> distance;
	distance=collision->getDistance();

	int Nb1=surf->efgObj()->nbNode();
	int Nb2=2;
	int nbCollide=distance.size();	

	Vec3d normal;

	mat m1;				m1.set_size(nbCollide,3*Nb1); m1.fill(0.0);
	mat mm1;			mm1.set_size(nbCollide,3*Nb1); mm1.fill(0.0);
	mat mcm;			mcm.set_size(nbCollide,nbCollide); mcm.fill(0.0);
	mat force;			force.set_size(nbCollide); force.fill(0.0);
	mat penetration;	penetration.set_size(nbCollide); penetration.fill(0.0);
	mat InternalDis;	InternalDis.set_size(3*Nb1),InternalDis.fill(0.0);


	//Internal Displacement
	float** DisAddress=surf->efgObj()->returnPreDis(dt,itter);
	float*  nodeMass=surf->efgObj()->nodeMass();

	double C=dt*dt*itter*(itter+1)/(2*nodeMass[0]);



	if(nbCollide){

		for(int i=0;i<Nb1*3;i++)
		{
			InternalDis(i)=(*DisAddress)[i];
		}

		for(int i=0;i<nbCollide;i++){
			penetration(i)=distance[i].Penetration;
		}


		//
		arrayInt affectedEFGNodes;
		std::vector<arrayInt>* neighborSurf = surf->neighborNodeOfSurfVertice();
		std::vector<Vec3i>* face=surf->surfObj()->face();
		for(int i=0;i<nbCollide;i++)
		{
			for (int j=0; j<3; j++)
			{
				int Index=(*face)[distance[i].triIdx][j];
				arrayInt neighborNode = neighborSurf->at(Index);
				for (int k=0; k<neighborNode.size(); k++)
				{
					if (!std::binary_search(affectedEFGNodes.begin(), affectedEFGNodes.end(), neighborNode[k]))
					{
						std::vector<int>::iterator it;
						it = std::lower_bound(affectedEFGNodes.begin(), affectedEFGNodes.end(), neighborNode[k]);
						affectedEFGNodes.insert(it, neighborNode[k]);
					}
				}
			}
		}
		int nbAffectedNodes = affectedEFGNodes.size();
		//
		mat m1t;		m1t.set_size(nbCollide, 3*nbAffectedNodes);
		//	penetration.print("pene1.txt");
		double* mapping;
		mapping=new double[3*Nb1];

		for(int i=0;i<nbCollide;i++)
		{
			arrayInt efgNodeIdx;
			normal=distance[i].PenetratedDirec;

			surf->returnMappingMatrixWithNodeIdx(distance[i].triIdx,distance[i].collidedTriPoint,normal,mapping, efgNodeIdx);
			func.arrangeVector(efgNodeIdx);

			for (int j=0; j<efgNodeIdx.size(); j++)
			{
				int nodeIdx = efgNodeIdx[j];
				int colIdx = func.indexOfElement(&affectedEFGNodes, nodeIdx);
				if (colIdx <0 || colIdx > nbAffectedNodes)
				{
					int a = efgNodeIdx[0];
				}
				for (int k=0; k<3; k++)
				{
					m1t(i,colIdx*3+k)=mapping[nodeIdx*3+k];
				}
			}

			for(int j=0;j<3*Nb1;j++){
				m1(i,j)=mapping[j];
			}
		}

		delete [] mapping;
		mapping=NULL;

		penetration-=m1*(InternalDis);

		//Construct MCM
	//	mcm=(m1*trans(m1))*C;
		mcm = (m1t*trans(m1t))*C;

		//Gauss Seidel 
		int Iteration=20;
		force=GaussSeidelMethod1(mcm,penetration,Iteration);

		//Internal Force
		InternalDis=-trans(m1)*force;
	}

	double* InternalForce;
	InternalForce=new double[3*Nb1];

	Vec3d transferdForce(0,0,100);

	for(int i=0;i<3*Nb1;i++)
	{
		InternalForce[i]=InternalDis(i);
	}

	surf->efgObj()->updatePosition(InternalForce,C,dt,itter);
	surf->updateSurfPosition();

	delete [] InternalForce;
	InternalForce=NULL;
}

void CollisionResponse::PenaltyMethod(Meshfree_GPU* surf,CollisionManager* collision,double K)
{

	std::vector<CollisionManager::Distancefield> distance;
	distance=collision->getDistance();
	
	Vec3d force;
	Vec3d forcet;
	forcet.clear();
	
	for(int i=0;i<distance.size();i++)
	{
		if(distance[i].Penetration<0)
		{
			force=(distance[i].PenetratedDirec)*distance[i].Penetration*K/distance.size();
			surf->addForceToSurfaceTri(distance[i].triIdx, distance[i].collidedTriPoint,-force);
			forcet-=force;
		}
	}
	//g_PhanTomCtr.setForce(forcet);
	
	

}

Cmat CollisionResponse::GaussSeidelMethod1(Cmat mcm,Cmat penetration,int Iteration)
{
	int nbCollide=mcm.nbR();

	double displacement;
	Cmat force(nbCollide);
	Cmat delta(nbCollide);

	for(int it=0;it<Iteration;it++)
	{
		for(int i=0;i<nbCollide;i++)
		{
			displacement=0;

			for(int j=0;j<nbCollide;j++)
				if(i!=j)
					displacement+=mcm(i,j)*force(j);
			delta(i)=penetration(i)+displacement;

			if(delta(i)>0){
				force(i)=0;
			}else{
				force(i)=-delta(i)/(mcm(i,i));
			}
		}
	}

	return force;
}

mat CollisionResponse::GaussSeidelMethod1(mat mcm,mat penetration,int Iteration)
{
	int nbCollide=mcm.n_rows;
	double displacement;
	mat force;
	force.set_size(nbCollide);
	force.fill(0.0);

	mat delta;
	delta.set_size(nbCollide);
	delta.fill(0.0);

	for(int it=0;it<Iteration;it++)
	{
		for(int i=0;i<nbCollide;i++)
		{
			displacement=0;

			for(int j=0;j<nbCollide;j++)
				if(i!=j)
					displacement+=mcm(i,j)*force(j);
			delta(i)=penetration(i)+displacement;

			if(delta(i)>0){
				force(i)=0;
			}else{
				force(i)=-delta(i)/(mcm(i,i));
			}
		}
	}

	return force;
}

void CollisionResponse::GaussSeidelMethod(Matrix& hcht, std::vector<float>& deltaFree, Matrix& force, int Iteration)
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

void CollisionResponse::ComputeforcefromComplianceV12( float dt, int itter, Meshfree_GPU* surf,CollisionManager* collision )
{
	VectorFunc func;
	std::vector<CollisionManager::Distancefield> distance;
	distance=collision->getDistance();

	int Nb1=surf->efgObj()->nbNode();
	int Nb2=2;
	int nbCollide=distance.size();	

	Vec3d normal;

	mat m1;				m1.set_size(nbCollide,3*Nb1); m1.fill(0.0);
	mat mm1;			mm1.set_size(nbCollide,3*Nb1); mm1.fill(0.0);
	mat mcm;			mcm.set_size(nbCollide,nbCollide); mcm.fill(0.0);
	mat force;			force.set_size(nbCollide); force.fill(0.0);
	mat penetration;	penetration.set_size(nbCollide); penetration.fill(0.0);
	mat InternalDis;	InternalDis.set_size(3*Nb1),InternalDis.fill(0.0);


	//Internal Displacement
	float** DisAddress=surf->efgObj()->returnPreDisNoDeform();
	float*  nodeMass=surf->efgObj()->nodeMass();

	double C=dt*dt*itter*(itter+1)/(2*nodeMass[0]);



	if(nbCollide){

		for(int i=0;i<Nb1*3;i++)
		{
			InternalDis(i)=(*DisAddress)[i];
		}

		for(int i=0;i<nbCollide;i++){
			penetration(i)=distance[i].Penetration;
		}


		//
		arrayInt affectedEFGNodes;
		std::vector<arrayInt>* neighborSurf = surf->neighborNodeOfSurfVertice();
		std::vector<Vec3i>* face=surf->surfObj()->face();
		for(int i=0;i<nbCollide;i++)
		{
			for (int j=0; j<3; j++)
			{
				int Index=(*face)[distance[i].triIdx][j];
				arrayInt neighborNode = neighborSurf->at(Index);
				for (int k=0; k<neighborNode.size(); k++)
				{
					if (!std::binary_search(affectedEFGNodes.begin(), affectedEFGNodes.end(), neighborNode[k]))
					{
						std::vector<int>::iterator it;
						it = std::lower_bound(affectedEFGNodes.begin(), affectedEFGNodes.end(), neighborNode[k]);
						affectedEFGNodes.insert(it, neighborNode[k]);
					}
				}
			}
		}
		int nbAffectedNodes = affectedEFGNodes.size();
		//
		mat m1t;		m1t.set_size(nbCollide, 3*nbAffectedNodes);
		//	penetration.print("pene1.txt");
		double* mapping;
		mapping=new double[3*Nb1];

		for(int i=0;i<nbCollide;i++)
		{
			arrayInt efgNodeIdx;
			normal=distance[i].PenetratedDirec;

			surf->returnMappingMatrixWithNodeIdx(distance[i].triIdx,distance[i].collidedTriPoint,normal,mapping, efgNodeIdx);
			func.arrangeVector(efgNodeIdx);

			for (int j=0; j<efgNodeIdx.size(); j++)
			{
				int nodeIdx = efgNodeIdx[j];
				int colIdx = func.indexOfElement(&affectedEFGNodes, nodeIdx);
				if (colIdx <0 || colIdx > nbAffectedNodes)
				{
					int a = efgNodeIdx[0];
				}
				for (int k=0; k<3; k++)
				{
					m1t(i,colIdx*3+k)=mapping[nodeIdx*3+k];
				}
			}

			for(int j=0;j<3*Nb1;j++){
				m1(i,j)=mapping[j];
			}
		}

		delete [] mapping;
		mapping=NULL;

		penetration-=m1*(InternalDis);

		//Construct MCM
		//	mcm=(m1*trans(m1))*C;
		mcm = (m1t*trans(m1t))*C;

		//Gauss Seidel 
		int Iteration=20;
		force=GaussSeidelMethod1(mcm,penetration,Iteration);

		//Internal Force
		InternalDis=-trans(m1)*force;
	}

	double* InternalForce;
	InternalForce=new double[3*Nb1];

	Vec3d transferdForce(0,0,100);

	for(int i=0;i<3*Nb1;i++)
	{
		InternalForce[i]=InternalDis(i);
	}

	surf->efgObj()->updatePosition(InternalForce,C,dt,itter);
	surf->updateSurfPosition();

	delete [] InternalForce;
	InternalForce=NULL;
}
