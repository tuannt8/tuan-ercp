
#include <stdio.h>

class Cmat
{
public:
	Cmat(void)
	{
		matrix=NULL;
		submatrix=NULL;
		Vec=NULL;
		returnMat=NULL;
	};

	Cmat(int nbR, int nbC)
	{
		NbR=nbR;
		NbC=nbC;

		matrix=new double*[NbR];
		for(int i=0;i<NbR;i++){
			matrix[i]=new double[NbC];
		}

		this->fill(0);

		submatrix=NULL;
		Vec=NULL;
		returnMat=NULL;

	};

	Cmat(int nbR)
	{
		NbR=nbR;
		NbC=1;


		matrix=new double*[NbR];
		for(int i=0;i<NbR;i++){
			matrix[i]=new double[NbC];
		}
		this->fill(0);
		submatrix=NULL;
		Vec=NULL;
		returnMat=NULL;
	};

	~Cmat(void)
	{
		if(matrix){
			for (int i=0;i<NbR;i++){
				delete [] matrix[i];
			}
			delete [] matrix;
		}

		if(submatrix){
			for (int i=0;i<SubR;i++){
				delete [] submatrix[i];
			}
			delete [] submatrix;
		}

		if(Vec){
			delete [] Vec;
		}
		if(returnMat)
		{
			delete returnMat;
		}

		returnMat=NULL;
		matrix=NULL;
		submatrix=NULL;
		Vec=NULL;
	};
	void init(int nbR, int nbC)
	{
		NbR=nbR;
		NbC=nbC;


		matrix=new double*[NbR];
		for(int i=0;i<NbR;i++){
			matrix[i]=new double[NbC];
		}

		this->fill(0);
	};

	void init(int nbR)
	{
		NbR=nbR;
		NbC=1;


		matrix=new double*[NbR];
		for(int i=0;i<NbR;i++){
			matrix[i]=new double[NbC];
		}

		this->fill(0);
	};

	void deleteMatrix()
	{
		if(matrix){
			for (int i=0;i<NbR;i++){
				delete [] matrix[i];
			}
			delete [] matrix;
		}

		if(submatrix){
			for (int i=0;i<SubR;i++){
				delete [] submatrix[i];
			}
			delete [] submatrix;
		}

		if(Vec){
			delete [] Vec;
		}


		matrix=NULL;
		submatrix=NULL;
		Vec=NULL;
	};

	double** get(){return matrix;};
	double** getSubMatrix(){return submatrix;};



	double get(int i,int j)
	{
		double result=matrix[i][j];
		return result;
	};
	double* getVec()
	{
		Vec=NULL;
		Vec=new double[NbR];
		for(int i=0;i<NbR;i++)
		{
			Vec[i]=matrix[i][0];
		}
		return Vec;
	};
	int nbR(){return NbR;};
	int nbC(){return NbC;};

	double** getSubMatrix(int startNbR,int startNbC,int endNbR,int endNbC)
	{
		int subNbR;
		int subNbC;
		subNbR=endNbR-startNbR+1;
		subNbC=endNbC-startNbC+1;

		initSub(subNbR,subNbC);

		for(int i=0;i<subNbR;i++){
			for(int j=0;j<subNbC;j++){
				submatrix[i][j]=matrix[startNbR+i][startNbC+j];
			}
		}

		return submatrix;

		deleteSub();
	};

	double* getSubMatrix(int startNbR)
	{
		double* Sub;
		Sub=new double[SubR];


		for(int j=0;j<NbC;j++){
			Sub[j]=matrix[startNbR][j];
		}


		return Sub;

		delete [] Sub;
	};

	void add(int I,int J,double value)
	{
		matrix[I][J]+=value;
	};

	void add(int I,double value)
	{
		matrix[I][0]+=value;
	};

	void add(double* A)
	{
		for(int i=0;i<NbR;i++){
			matrix[i][0]+=A[i];
		}
	};

	void add(double** A)
	{
		for(int i=0;i<NbR;i++){
			for(int j=0;j<NbC;j++)
			{
				matrix[i][j]+=A[i][j];
			}
		}
	};

	void minus(int I,int J,double value)
	{
		matrix[I][J]-=value;
	};

	void minus(int I,double value)
	{
		matrix[I][0]-=value;
	};

	void minus(double* A)
	{
		for(int i=0;i<NbR;i++){
			matrix[i][0]-=A[i];
		}
	};

	void minus(double** A)
	{
		for(int i=0;i<NbR;i++){
			for(int j=0;j<NbC;j++)
			{
				matrix[i][j]-=A[i][j];
			}
		}
	};

	void set(int i,int j,double A)
	{
		matrix[i][j]=A;

	};


	void set(double* A)
	{
		for(int i=0;i<NbR;i++){
			matrix[i][0]=A[i];
		}
	};

	void set(float* A)
	{
		for(int i=0;i<NbR;i++){
			matrix[i][0]=A[i];
		}
	};

	void set(double** A)
	{
		for(int i=0;i<NbR;i++){
			for(int j=0;j<NbC;j++)
			{
				matrix[i][j]=A[i][j];
			}
		}
	};

	void mul(double* A)
	{
		for(int i=0;i<NbR;i++){
			matrix[i][0]*=A[i];
		}
	};

	void mul(double A)
	{
		for(int i=0;i<NbR;i++){
			for(int j=0;j<NbC;j++)
			{
				matrix[i][j]*=A;
			}
		}
	};

	double* returnMul(double* A)
	{
		for(int i=0;i<NbR;i++){
			A[i]*=matrix[i][0];
		}
		return A;
	};

	double* returnMul(double* A,double* B,int Nb)
	{
		for(int i=0;i<Nb;i++){
			A[i]*=B[i];
		}
		return A;
	};

	double** returnNormalForm()
	{
		return matrix;
	};

	double* returnVectorForm()
	{
		if(Vec)
		{
			delete [] Vec;
		}
		Vec=NULL;
		Vec=new double[NbR];
		for(int i=0;i<NbR;i++)
		{
			Vec[i]=matrix[i][0];
		}
		return Vec;
	};

	void fill(double value)
	{
		for(int i=0;i<NbR;i++){
			for(int j=0;j<NbC;j++)
			{
				matrix[i][j]=value;
			}
		}
	};


	void setSubMatrix(int startNbR,int startNbC,Cmat A)
	{
		int endNbR=startNbR+A.nbR();
		int endNbC=startNbC+A.nbC();

		for(int i=startNbR;i<endNbR;i++){
			for(int j=startNbC;j<endNbC;j++){
				matrix[i][j]=A.get()[i-startNbR][j-startNbC];
			}
		}
	};

	void setSubMatrix(int i,int j,double A)
	{
		submatrix[i][j]=A;
	};

	void setSubMatrix(int startNbR,double* A)
	{
		for(int i=0;i<NbC;i++){
			matrix[startNbR][i]=A[i];
		}
	};

	void setSubMatrix(int startNbR,double** A)
	{

		for(int i=startNbR;i<NbR;i++){
			for(int j=0;j<NbC;j++){
				matrix[i][j]=A[i-startNbR][j];
			}
		}
	};

	void addSubMatrix(int startNbR,int startNbC,Cmat A)
	{
		int endNbR=startNbR+A.nbR();
		int endNbC=startNbC+A.nbC();

		for(int i=startNbR;i<endNbR;i++){
			for(int j=startNbC;j<endNbC;j++){
				matrix[i][j]+=A.get()[i-startNbR][j-startNbC];
			}
		}
	};

	Cmat& operator+(Cmat &B)
	{
		for(int i=0;i<NbR;i++){
			for(int j=0;j<NbC;j++)
			{
				this->matrix[i][j]=this->matrix[i][j]+B(i,j);
			}
		}
		return *this;
	};

	Cmat& operator-(Cmat &B)
	{


		for(int i=0;i<NbR;i++){
			for(int j=0;j<NbC;j++)
			{
				this->matrix[i][j]=this->matrix[i][j]-B(i,j);
			}
		}

		return *this;

	};


	void operator=(Cmat &B)
	{
		this->deleteMatrix();
		this->init(B.nbR(),B.nbC());

		for(int i=0;i<B.nbR();i++){
			for(int j=0;j<B.nbC();j++)
			{
				this->set(i,j,B(i,j));
			}
		}
	};

	

	Cmat& operator* (Cmat &B)
	{
		if(returnMat)
		{
			delete returnMat;
			returnMat=NULL;
		}
		returnMat=new Cmat;
		returnMat->init(this->nbR(),B.nbC());

		for(int i=0;i<this->nbR();i++)
		{
			for(int j=0;j<B.nbC();j++)
			{
				for(int k=0;k<this->nbC();k++)
				{
					returnMat->add(i,j,this->get(i,k)*B(k,j));
				}
			}
		}
		return *returnMat;
	};

	Cmat& operator%(Cmat &B)
	{
		if(returnMat)
		{
			delete returnMat;
			returnMat=NULL;
		}
		returnMat=new Cmat;
		returnMat->init(this->nbR(),this->nbC());

		
		if(NbR>=NbC)
		{
			for(int j=0;j<NbC;j++)
			{
				for(int i=0;i<NbR;i++)
				{
					returnMat->set(i,j,this->matrix[i][j]*B(i));
				}
			}
		}else
		{
			for(int i=0;i<NbR;i++)
			{
				for(int j=0;j<NbC;j++)
				{
					returnMat->set(i,j,this->matrix[i][j]*B(j));
				}
			}
		}

		return *returnMat;
	};

	Cmat& operator+(double B)
	{
		for(int i=0;i<NbR;i++){
			for(int j=0;j<NbC;j++)
			{
				this->matrix[i][j]=this->matrix[i][j]+B;
			}
		}

		return *this;
	};

	Cmat& operator-(double B)
	{
		for(int i=0;i<NbR;i++){
			for(int j=0;j<NbC;j++)
			{
				this->matrix[i][j]=this->matrix[i][j]-B;
			}
		}

		return *this;
	};

	Cmat& operator*(double B)
	{

		for(int i=0;i<NbR;i++){
			for(int j=0;j<NbC;j++)
			{
				this->matrix[i][j]=this->matrix[i][j]*B;
			}
		}

		return *this;

	};

	Cmat& operator/(double B)
	{

		for(int i=0;i<NbR;i++){
			for(int j=0;j<NbC;j++)
			{
				this->matrix[i][j]=this->matrix[i][j]/B;
			}
		}

		return *this;
	};

	void operator+=(double B)
	{
		for(int i=0;i<NbR;i++){
			for(int j=0;j<NbC;j++)
			{
				matrix[i][j]+=B;
			}
		}
	};

	void operator-=(double B)
	{
		for(int i=0;i<NbR;i++){
			for(int j=0;j<NbC;j++)
			{
				matrix[i][j]-=B;
			}
		}
	};

	void operator+=(Cmat &B)
	{
		for(int i=0;i<NbR;i++){
			for(int j=0;j<NbC;j++)
			{
				matrix[i][j]+=B(i,j);
			}
		}
	};

	void operator-=(Cmat &B)
	{
		for(int i=0;i<NbR;i++){
			for(int j=0;j<NbC;j++)
			{
				matrix[i][j]-=B(i,j);
			}
		}
	};

	void operator*=(double B)
	{

		for(int i=0;i<NbR;i++){
			for(int j=0;j<NbC;j++)
			{
				matrix[i][j]*=B;
			}
		}
	};

	void operator/=(double B)
	{
		for(int i=0;i<NbR;i++){
			for(int j=0;j<NbC;j++)
			{
				matrix[i][j]/=B;
			}
		}
	};

	double& operator()(int i,int j)
	{
		return matrix[i][j];
	};


	double& operator()(int i)
	{
		return matrix[i][0];
	};


	Cmat& operator-()
	{
		
		
		for(int i=0;i<this->nbR();i++)
		{
			for(int j=0;j<this->nbC();j++)
			{
				this->set(i,j,-this->get(i,j));
					
			}
		}

		return *this;

	};


	/*void operator()(int i,int j)=(double V)
	{
		matrix[i][j]=V;
	};


	void operator()(int i)=(double V)
	{
		matrix[i][0]=V;
	};

	void operator()(int i,int j)+=(double V)
	{
		matrix[i][j]+=V;
	};


	void operator()(int i)+=(double V)
	{
		matrix[i][0]+=V;
	};


	void operator()(int i,int j)-=(double V)
	{
		matrix[i][j]-=V;
	};


	void operator()(int i)-=(double V)
	{
		matrix[i][0]-=V;
	};


	void operator()(int i,int j)*=(double V)
	{
		matrix[i][j]*=V;
	};


	void operator()(int i)*=(double V)
	{
		matrix[i][0]*=V;
	};


	void operator()(int i,int j)/=(double V)
	{
		matrix[i][j]/=V;
	};


	void operator()(int i)/=(double V)
	{
		matrix[i][0]/=V;
	};*/



	double** trans()
	{
		double** Tmatrix;
		Tmatrix=new double*[NbC];
		for(int i=0;i<NbC;i++)
			Tmatrix[i]=new double[NbR];

		for(int i=0;i<NbR;i++)
		{
			for(int j=0;j<NbC;j++)
			{
				Tmatrix[j][i]=matrix[i][j];
			}
		}
		
		return Tmatrix;

		if(Tmatrix){
			for (int i=0;i<NbC;i++){
				delete [] Tmatrix[i];
			}
		}
	}

	Cmat& T()
	{
		this->deleteSub();
		this->initSub(NbC,NbR);


		for(int i=0;i<NbR;i++)
		{
			for(int j=0;j<NbC;j++)
			{
				this->setSubMatrix(j,i,this->get(i,j));
			}
		}

		this->init(NbC,NbR);
		this->set(this->submatrix);
		this->deleteSub();

		return *this;

	}

	void t()
	{
		double** Tmatrix;

		int nbC,nbR;
		nbC=NbC;
		nbR=NbR;


		Tmatrix=new double*[nbC];
		for(int i=0;i<nbC;i++)
			Tmatrix[i]=new double[nbR];

		for(int i=0;i<nbR;i++)
		{
			for(int j=0;j<nbC;j++)
			{
				Tmatrix[j][i]=matrix[i][j];
			}
		}

		deleteMatrix();

		init(nbC,nbR);

		for(int i=0;i<nbC;i++)
		{
			for(int j=0;j<nbR;j++)
			{
				matrix[i][j]=Tmatrix[i][j];
			}
		}



		if(Tmatrix){
			for (int i=0;i<nbC;i++){
				delete [] Tmatrix[i];
			}
			delete [] Tmatrix;
		}
		Tmatrix=NULL;
	}

	Cmat& transMul(Cmat &B)
	{
		if(returnMat)
		{
			delete returnMat;
			returnMat=NULL;
		}
		returnMat=new Cmat;
		returnMat->init(this->nbC(),B.nbC());

		for(int i=0;i<this->nbC();i++)
		{
			for(int j=0;j<B.nbC();j++)
			{
				for(int k=0;k<this->nbR();k++)
				{
					returnMat->add(i,j,this->get(k,i)*B(k,j));
				}
				returnMat->print("check3.txt");
			}
		}
		return *returnMat;
	};

	Cmat& transMul2(Cmat &B)
	{
		if(returnMat)
		{
			delete returnMat;
			returnMat=NULL;
		}
		returnMat=new Cmat;
		returnMat->init(this->nbR(),B.nbR());

		for(int i=0;i<this->nbR();i++)
		{
			for(int j=0;j<B.nbR();j++)
			{
				for(int k=0;k<this->nbC();k++)
				{
					returnMat->add(i,j,this->get(i,k)*B(j,k));
				}
			}
		}
		return *returnMat;
	};

	void print()
	{
		for(int i=0;i<NbR;i++)
		{
			for(int j=0;j<NbC;j++)
			{
				printf("%f ",this->get(i,j));
			}
			printf("\n");
		}
		printf("\n");
	};

	void print(char* filename)
	{
		FILE* f=fopen(filename,"w");
		for(int i=0;i<NbR;i++)
		{
			for(int j=0;j<NbC;j++)
			{
				fprintf(f,"%f ",matrix[i][j]);
			}
			fprintf(f,"\n");
		}

		fclose(f);

	};






private:

	void initSub(int nbR, int nbC)
	{
		SubR=nbR;
		SubC=nbC;
		submatrix=new double*[SubR];
		for(int i=0;i<SubR;i++){
			submatrix[i]=new double[SubC];
		}

		for(int i=0;i<SubR;i++){
			for(int j=0;j<SubC;j++)
			{
				submatrix[i][j]=0;
			}
		}

	};


	void deleteSub()
	{
		if(submatrix){
			for (int i=0;i<SubR;i++){
				delete [] submatrix[i];
			}
			delete [] submatrix;
		}
		submatrix=NULL;
	};

	double* Vec;
	double** matrix;
	double** submatrix;
	int NbR,NbC;
	int SubR,SubC;

	Cmat* returnMat;

};


