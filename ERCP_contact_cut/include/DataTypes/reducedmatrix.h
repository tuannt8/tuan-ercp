#ifndef REDUCED_MATRIX
#define REDUCED_MATRIX
#include "armadillo"
#include <functional>

using namespace arma;
using namespace std;

//=======================================================
// this class is developed for block-tridiagonal matrix  
//=======================================================

class Rmat
{

public:
	Rmat(int _blocksize,int _size)
	{
		blocksize=_blocksize; size=_size;
		numberofblock=size/blocksize;
		reduced.set_size(size,2*blocksize);
		reduced.fill(0.0);
	}; 
	Rmat(void)
	{
	};  
	~Rmat(void)
	{
	};

public:
	int blocksize,size,numberofblock;
	mat reduced;


public:
	void add(mat A)
	{
		int idx1,idx2,idx3;

		for(int i=0;i<numberofblock-1;i++){
			idx1=i*blocksize;
			idx2=(i+1)*blocksize;
			idx3=(i+2)*blocksize;
			reduced.submat(idx1,0,idx2-1,blocksize-1)+=A.submat(idx1,idx1,idx2-1,idx2-1);
			reduced.submat(idx1,blocksize,idx2-1,2*blocksize-1)+=A.submat(idx1,idx2,idx2-1,idx3-1);
		}
		idx1=(numberofblock-1)*blocksize;
		idx2=numberofblock*blocksize;
		reduced.submat(idx1,0,idx2-1,blocksize-1)+=A.submat(idx1,idx1,idx2-1,idx2-1);
	};

	Rmat operator+(Rmat &B)
	{
		Rmat A(blocksize,size);
		A.reduced=reduced+B.reduced;
		return A;
	};

	Rmat operator-(Rmat &B)
	{
		Rmat A(blocksize,size);
		A.reduced=reduced-B.reduced;
		return A;
	};

	Rmat operator+(double B)
	{
		Rmat A(blocksize,size);
		A.reduced=reduced+B;
		return A;
	};

	Rmat operator-(double B)
	{
		Rmat A(blocksize,size);
		A.reduced=reduced-B;
		return A;
	};

	Rmat operator*(double B)
	{
		Rmat A(blocksize,size);
		A.reduced=reduced*B;
		return A;
	};

	Rmat operator/(double B)
	{
		Rmat A(blocksize,size);
		reduced=reduced/B;
		return A;
	};

	void operator+=(double B)
	{
		reduced+=B;
	};

	void operator-=(double B)
	{

		reduced-=B;
	};

	void operator*=(double B)
	{

		reduced*=B;
	};

	void operator/=(double B)
	{
		reduced/=B;
	};



	void add(int i,int j,double A)
	{
		vec2 a;
		a(0)=i;a(1)=j;
		a=IndexMapping(a);
		if(a(1)>=0 && a(1)<2*blocksize)
			reduced(a(0),a(1))+=A;
	};


	void add(Rmat A)
	{
		reduced+=A.reduced;
	};

	void subtract(Rmat A)
	{
		reduced-=A.reduced;
	};

	void set_value(int i,int j,double A)
	{
		vec2 a;
		a(0)=i;a(1)=j;
		a=IndexMapping(a);
		if(a(1)>=0 && a(1)<2*blocksize)
			reduced(a(0),a(1))=A;
	};

	double return_value(int i,int j)
	{
		vec2 a;
		a(0)=i;a(1)=j;
		a=IndexMapping(a);
		if(a(1)>=0 && a(1)<2*blocksize)
		{
			return reduced(a(0),a(1));
		}
		else
		{
			return 0;
		}
	};

	void set(int _blocksize,int _size)
	{
		blocksize=_blocksize; size=_size;
		numberofblock=size/blocksize;
		reduced.set_size(size,2*blocksize);
		reduced.fill(0.0);
	};

	mat multiply(mat A)
	{
		int A_r,A_c;
		A_r=A.n_rows;
		A_c=A.n_cols;
		mat result(A_r,A_c);
		result.fill(0.0);

		mat subReduced;
		mat subA;
		for(int i=0;i<numberofblock;i++){
			if(i>0)
			{
				int ii=i-1;

				subReduced=reduced.submat(blocksize*ii,blocksize,blocksize*(ii+1)-1,blocksize*2-1);
				subReduced=trans(subReduced);
				//subReduced.print();
				subA=A.submat(blocksize*(i-1),0,blocksize*i-1,A_c-1);
				//subA.print();
				result.submat(blocksize*i,0,blocksize*(i+1)-1,A_c-1)+=subReduced*subA;
			}
			for(int j=0;j<2;j++){

				int jj=i+j;

				if(jj<numberofblock)
				{
					//printf("%d %d\n",i,jj);

					subReduced=reduced.submat(blocksize*i,blocksize*j,blocksize*(i+1)-1,blocksize*(j+1)-1);
					//subReduced.print();
					subA=A.submat(blocksize*jj,0,blocksize*(jj+1)-1,A_c-1);
					//subA.print();
					result.submat(blocksize*i,0,blocksize*(i+1)-1,A_c-1)+=subReduced*subA;
				}	
			}

		}
		return result;

	};

	void multiply(double A)
	{
		reduced*=A;
	};



	void printReducedform()
	{
		reduced.print();
	};

	void printFullform()
	{
		mat A=returnNormalForm();
		A.print();
	};

	vec2 Rmat::IndexMapping(vec2 a)
	{
		//Normal to Reduced
		vec b=a;
		int row,shift;
		row=a(0)/blocksize;



		shift=row;

		b(1)-=blocksize*shift;

		return b;
	};

	int Rmat::Mapping(int i,int j)
	{
		//Reduced to Normal
		int b=j;
		int shift;
		shift=i/blocksize;

		b+=blocksize*shift;

		return b;
	};

	vec2 Rmat::IndexInverseMapping(vec2 a)
	{
		//Reduced to Normal
		vec b=a;
		int row,shift;
		row=a(0)/blocksize;


		shift=row;

		b(1)+=blocksize*shift;

		return b;
	};


	mat returnNormalForm()
	{
		vec2 a;
		int idx1,idx2,idx3;
		mat A(size,size);
		A.fill(0.0);
		for(int i=0;i<numberofblock-1;i++){
			idx1=i*blocksize;
			idx2=(i+1)*blocksize;
			idx3=(i+2)*blocksize;
			A.submat(idx1,idx1,idx2-1,idx2-1)=reduced.submat(idx1,0,idx2-1,blocksize-1);
			A.submat(idx1,idx2,idx2-1,idx3-1)=reduced.submat(idx1,blocksize,idx2-1,2*blocksize-1);
			A.submat(idx2,idx1,idx3-1,idx2-1)=trans(reduced.submat(idx1,blocksize,idx2-1,2*blocksize-1));
		}
		idx1=(numberofblock-1)*blocksize;
		idx2=numberofblock*blocksize;
		A.submat(idx1,idx1,idx2-1,idx2-1)=reduced.submat(idx1,0,idx2-1,blocksize-1);

		return A;

	};

	mat Solve(mat B)
	{
		int idx0,idx1,idx2,idx3;
		int Ncol=B.n_cols-1;

		//Decompose
		for(int i=1;i<numberofblock;i++){
			idx0=(i-1)*blocksize;
			idx1=i*blocksize;
			idx2=(i+1)*blocksize;
			//Lt i-1
			reduced.submat(idx0,blocksize,idx1-1,2*blocksize-1)=solve(reduced.submat(idx0,0,idx1-1,blocksize-1),reduced.submat(idx0,blocksize,idx1-1,2*blocksize-1));

			// D i
			reduced.submat(idx1,0,idx2-1,blocksize-1)-=(trans(reduced.submat(idx0,blocksize,idx1-1,2*blocksize-1)))*reduced.submat(idx0,0,idx1-1,blocksize-1)*reduced.submat(idx0,blocksize,idx1-1,2*blocksize-1);
		}
		
		//Forward substitution
		for(int i=1;i<numberofblock;i++){
			idx0=(i-1)*blocksize;
			idx1=i*blocksize;
			idx2=(i+1)*blocksize;
			B.submat(idx1,0,idx2-1,Ncol)-=((reduced.submat(idx0,blocksize,idx1-1,2*blocksize-1)).t())*B.submat(idx0,0,idx1-1,Ncol);
		}


		//Backward substitution
		idx1=(numberofblock-1)*blocksize;
		idx2=numberofblock*blocksize;
		B.submat(idx1,0,idx2-1,Ncol)=solve(reduced.submat(idx1,0,idx2-1,blocksize-1),B.submat(idx1,0,idx2-1,Ncol));
		for(int i=numberofblock-2;i>-1;i--)
		{
			idx1=i*blocksize;
			idx2=(i+1)*blocksize;
			idx3=(i+2)*blocksize;			
			B.submat(idx1,0,idx2-1,Ncol)=solve(reduced.submat(idx1,0,idx2-1,blocksize-1),B.submat(idx1,0,idx2-1,Ncol))-(reduced.submat(idx1,blocksize,idx2-1,2*blocksize-1))*B.submat(idx2,0,idx3-1,Ncol);
		}

		return B;
	};

	mat Resolve(mat B)
	{
		int idx0,idx1,idx2,idx3;
		int Ncol=B.n_cols-1;

		//Forward substitution
		for(int i=1;i<numberofblock;i++){
			idx0=(i-1)*blocksize;
			idx1=i*blocksize;
			idx2=(i+1)*blocksize;
			B.submat(idx1,0,idx2-1,Ncol)-=((reduced.submat(idx0,blocksize,idx1-1,2*blocksize-1)).t())*B.submat(idx0,0,idx1-1,Ncol);
		}


		//Backward substitution
		idx1=(numberofblock-1)*blocksize;
		idx2=numberofblock*blocksize;
		B.submat(idx1,0,idx2-1,Ncol)=solve(reduced.submat(idx1,0,idx2-1,blocksize-1),B.submat(idx1,0,idx2-1,Ncol));
		for(int i=numberofblock-2;i>-1;i--)
		{
			idx1=i*blocksize;
			idx2=(i+1)*blocksize;
			idx3=(i+2)*blocksize;			
			B.submat(idx1,0,idx2-1,Ncol)=solve(reduced.submat(idx1,0,idx2-1,blocksize-1),B.submat(idx1,0,idx2-1,Ncol))-(reduced.submat(idx1,blocksize,idx2-1,2*blocksize-1))*B.submat(idx2,0,idx3-1,Ncol);
		}

		return B;
	};

};

#endif