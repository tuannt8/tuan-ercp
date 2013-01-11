#if !defined(_DATASTRUCT_H_)
#define _DATASTRUCT_H_

#include <Float.h>
#include <math.h>
#include <assert.h>
#include <iostream>

#define VEC_EPS float(10e-6)
#define SWAP(x, y, t) ( (t) = (x), (x) = (y), (y)= (t) )

class sort {
public:
	sort()
	{
	};

	~sort()
	{
	};

	void Sort(int list[], int Size)
	{
		QuickSort(list, 0, Size-1);
	};

	void Sort(double list[], int Size)
	{
		QuickSort(list, 0, Size-1);
	};

private:
	void QuickSort(int list[], int left, int right)
	{
		if(left<right)
		{
			int q=Partition(list, left, right);

			QuickSort(list, left, q-1);
			QuickSort(list, q+1,right);
		}
	};

	void QuickSort(double list[], int left, int right)
	{
		if(left<right)
		{
			int q=Partition(list, left, right);

			QuickSort(list, left, q-1);
			QuickSort(list, q+1,right);
		}
	};

	int Partition(int list[], int left, int right)
	{
		int pivot, temp;
		int low, high;

		low=left;
		high=right+1;

		pivot=list[left];

		do
		{
			do
				low++;
			while(list[low]<pivot);

			do
				high--;
			while(list[high]>pivot);

		if(low<high)
			SWAP(list[low], list[high], temp);
		
		}while(low<high);
		
		SWAP(list[left], list[high], temp);

		return high;
	};

	int Partition(double list[], int left, int right)
	{
		double pivot, temp;
		int low, high;

		low=left;
		high=right+1;

		pivot=list[left];

		do
		{
			do
				low++;
			while(list[low]<pivot);

			do
				high--;
			while(list[high]>pivot);

		if(low<high)
			SWAP(list[low], list[high], temp);
		
		}while(low<high);
		
		SWAP(list[left], list[high], temp);

		return high;
	};
};

class vec3f {
public:
	float  v[3];

	vec3f() 
	{
		v[0]=0; v[1]=0; v[2]=0;
	};

	vec3f(float initx, float inity,float initz)
	{ 
		v[0]=initx; v[1]=inity; v[2]=initz; 
	};
	
	void setValue(float xx, float yy, float zz)	
	{
		v[0]=xx; v[1]=yy; v[2]=zz; 
	};

	void setValue(vec3f init) 
	{
		v[0]=init.v[0]; v[1]=init.v[1]; v[2]=init.v[2];
	};

	float & operator [] ( int i ) 
	{ 
		return v[i]; 
	};

	const float & operator [] ( int i ) const 
	{ 
		return v[i]; 
	};

	vec3f & operator += ( const vec3f & u )
	{ 
		for(int i = 0; i < 3; i++) 
			v[i] += u.v[i]; 
		return *this;
	};

	vec3f operator + ( const vec3f &v) const
	{ 
		vec3f rt(*this); 
		return rt += v; 
	};

	vec3f & operator -= ( const vec3f & u )
	{ 
		for(int i = 0; i < 3; i++) 
			v[i] -= u.v[i]; 
		return *this;
	};

	vec3f operator - ( const vec3f &v) const
	{ 
		vec3f rt(*this); 
		return rt -= v; 
	};

	vec3f & operator *= ( float d )
	{ 
		for(int i = 0; i < 3; i++) 
			v[i] *= d; 
		return *this;
	};

	vec3f operator * ( float d) const
	{ 
		vec3f rt(*this); 
		return rt *= d; 
	};

	bool operator==(vec3f point) const
	{
		if((v[0]==point.v[0]) && (v[1]==point.v[1]) && (v[2]==point.v[2]))
			return true;
		else
			return false;
	};

	vec3f operator*(vec3f multiply) const
	{
		vec3f ret;	ret.v[0]=v[0]*multiply.v[0];	ret.v[1]=v[1]*multiply.v[1];	ret.v[2]=v[2]*multiply.v[2];	
		return ret;
	};
	
	vec3f operator/(const float& n) const
	{
		vec3f ret;ret.v[0]=v[0]/n;ret.v[1]=v[1]/n;ret.v[2]=v[2]/n;
		return ret;
	};

	vec3f cross(vec3f point) const
	{
		vec3f ret;ret.v[0]=v[1]*point.v[2]-v[2]*point.v[1]; ret.v[1]=v[2]*point.v[0]-v[0]*point.v[2];ret.v[2]=v[0]*point.v[1]-v[1]*point.v[0];
		return ret;
	};

	float dot(vec3f p) const
	{
		float dot;dot = v[0]*p.v[0] + v[1]*p.v[1] + v[2]*p.v[2];
		return dot;
	};

	float mag()
	{
		float m=(float)sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]); 
		return m;
	};

	vec3f mulscal(float s) 
	{
		vec3f ret; ret.v[0]=s*v[0]; ret.v[1]=s*v[1]; ret.v[2]=s*v[2]; 
		return ret;
	};

	float normalize() 
	{ 
		float sum(0);
		
		for(int i = 0; i < 3; i++) 
			sum += v[i]*v[i];

		sum = float(sqrt(sum));
		if (sum > VEC_EPS)
			for(int i = 0; i < 3; i++) 
                    v[i] /= sum;
		return sum;
	};
};

class vec3d {
public:
	union {
		struct {
		double x, y, z;
		};
		struct {
		double v[3];
		};
	};

	vec3d ()
	{x=0; y=0; z=0;}

	vec3d(vec3f v)
	{
		x = v[0];
		y = v[1];
		z = v[2];
	};
	
	vec3d(const float * v)
	{
		x = v[0];
		y = v[1];
		z = v[2];
	};

	void setValue(float xx, float yy, float zz)	
	{
		x=xx;y=yy;z=zz;
	};

	void setValue(vec3d init) 
	{
		x=init.x;y=init.y;z=init.z;
	};

	const double & operator [] ( int i ) const
	{
		switch (i) {
		case 0: return x;
		case 1: return y;
		case 2: return z;
		default: assert(0); return x;
		}
	};

	vec3d(double x, double y, double z)
	{
		this->x = x;
		this->y = y;
		this->z = z;
	};

	vec3d operator+ (const vec3d &v) const
	{
		return vec3d(x+v.x, y+v.y, z+v.z);
	};

	vec3d operator- (const vec3d &v) const
	{
		return vec3d(x-v.x, y-v.y, z-v.z);
	};

	vec3d operator *(float t) const
	{
		return vec3d(x*t, y*t, z*t);
	};

	vec3d operator/ (float n) const
	{
		return vec3d(x/n, y/n, z/n);
	};

	bool operator==(vec3d point) const
	{
		double eps=0.0001;
		if((fabs(x-point.x)<eps) && (fabs(y-point.y)<eps) && (fabs(z-point.z)<eps))
			return true;
		else
			return false;
	};

    // cross product
    const vec3d cross(const vec3d &vec) const
    {
         return vec3d(y*vec.z - z*vec.y, z*vec.x - x*vec.z, x*vec.y - y*vec.x);
    };

	//dot product
	float dot(vec3d p) const
	{
		float dot;dot = x*p.x + y*p.y + z*p.z;
		return dot;
	};

	float mag()
	{
		float m=(float)sqrt(x*x+y*y+z*z); 
		return m;
	};

	vec3d mulscal(float s) 
	{
		vec3d ret; ret.x=s*x; ret.y=s*y; ret.z=s*z; 
		return ret;
	};

	float normalize()
	{
		float sum=mag(); 
		if (sum > VEC_EPS)
		{
			x=x/sum; y=y/sum; z=z/sum;
		}
		return sum;
	};

	void vrand(float mag)
	{
	//	srand(time(NULL));
		x = rand() % 10 + 1;
		y = rand() % 10 + 1;
		z = rand() % 10 + 1;

		x=1/x;
		y=1/y;
		z=1/z;

		this->normalize();
		x*=mag;
		z*=mag;
		y*=mag;
	};
};

class vec3i {
public:
	int ii, jj, kk;
	vec3i(int i, int j, int k)
	{
		ii = i; jj = j; kk = k;
	}
	vec3i() {ii = jj = kk = 0;};
	int & operator [] ( int i )
	{
		switch (i) {
		case 0: return ii;
		case 1: return jj;
		case 2: return kk;
		default: assert(0); return ii;
		}
	};
};

class vec4i {
public:
	int ii, jj, kk, ww;
	vec4i() {ii = jj = kk = ww = 0;};
};

class matrix{
private:
	//행렬식 저장
	double determinant; 
	double* data;
	int col;
	int row;

public:

	matrix()
	{
		col=0; row=0; determinant=0; data=NULL;
	}
	matrix(int Row, int Col)
	{
		col=Col;
		row=Row;
		data=new double[Row*Col];
		for(int i=0;i<row*col;i++)
			data[i]=0.0;
		determinant=0;
	};

	matrix(const matrix & aRef)
	{
		data=NULL;
		determinant=aRef.determinant;
		col=aRef.col;
		row=aRef.row;
		New(aRef.row, aRef.col);
		std::copy (aRef.begin(), aRef.end(), begin());
	};

	~matrix()
	{
		if(data)
		{
			delete [] data;
			data=NULL;
		}
	};

	void init()
	{
		for(int i=0;i<row*col;i++)
			data[i]=0.0;
	};

	void init(int Row, int Col)
	{
		col=Col;
		row=Row;
		if(!data)
			data=new double[Row*Col];
		for(int i=0;i<row*col;i++)
			data[i]=0.0;
		determinant=0;
	};

	void reinit(int Row, int Col)
	{
		col=Col;
		row=Row;
		if(data)
		{
			delete [] data;
			data=NULL;
		}
		data=new double[Row*Col];
		for(int i=0;i<row*col;i++)
			data[i]=0.0;
		determinant=0;
	};

	void New(int Row, int Col)
	{
		double *newData = new double [Row*Col];
		for(int i=0;i<Row*Col;i++)
			newData[i]=0;
		if(data)
			delete [] data;
		data = newData;
		row = Row;
		col = Col;
	};

	/// 시작과 끝 포인터를 리턴 
	double *begin () { return data; };
	double *end () { return data+(row*col); };

	const double *begin () const { return data; };
	const double *end () const { return data+(row*col); };

	int Row() {return row;};
	int Col() {return col;};
	double* getMatrix(){if(data) return data;};
	double getDeterminant(){ return determinant;};


	// 할당 연산자 : = 
	matrix & operator=(const matrix & mat)
	{
		New (mat.row, mat.col);
		std::copy (mat.begin(), mat.end(), begin());

		determinant = mat.determinant;
		return *this;
	};

	double  operator [] (int index) const {return data[index];};
	double& operator [] (int index) {return data[index];};

	double & operator() (int Row, int Col)
	{ 
		ASSERT(Row<row && Col<col);
		return data[Row*col+Col]; 
	};

	matrix & operator*=(const double & n) 
	{
		int i,j;
		for(i=0;i<row;i++)
		{
			for(j=0;j<col;j++)
				data[i*col+j]*=n;
		}
		return *this;
	};

	matrix operator*(const double & n) const
	{
		matrix rt(*this);
		return rt*=n;
	};

	matrix operator*(const matrix & mat) const 
	{
		int i,j,k;
		matrix ref(row, mat.col);
		if(!(col==mat.row))
			return ref;

		for(i=0;i<row;i++)
		{
			for(j=0;j<mat.col;j++)
			{
				for(k=0;k<col;k++)
					ref(i,j)+=data[i*col+k]*mat.data[k*mat.col+j];
			}
		}
		return ref;
	};

	matrix & operator/=(const double & n) 
	{
		int i,j;
		for(i=0;i<row;i++)
		{
			for(j=0;j<col;j++)
				data[i*col+j]/=n;
		}
		return *this;
	};

	matrix operator/(const double & n) const
	{
		matrix rt(*this);
		return rt/=n;
	};

	matrix & operator+=(const matrix &v)
	{
		int i,j;
		if(!(row==v.row) || !(col==v.col))
			return *this;

		for(i=0;i<row;i++)
		{
			for(j=0;j<col;j++)
				data[i*col+j]+=v.data[i*col+j];
		}
		return *this;
	};

	matrix operator + (const matrix &ref) const
	{
		matrix rt(*this);
		return rt+=ref;
	};

	matrix & operator-=(const matrix &v)
	{
		int i,j;
		if(!(row==v.row) || !(col==v.col))
			return *this;

		for(i=0;i<row;i++)
		{
			for(j=0;j<col;j++)
				data[i*col+j]-=v.data[i*col+j];
		}
		return *this;
	};

	matrix operator - (const matrix &ref) const
	{
		matrix rt(*this);
		return rt-=ref;
	};

	// 두행렬 일치성 검사 
	bool operator == (const matrix & mat) const
	{
		if (!(this->row == mat.row && this->col==mat.col))
			return false;

		return std::equal (begin(), end(), mat.begin());
	};

	matrix returnSubMatrix(int rowIndex, int colIndex, int rowWidth, int colWidth)
	{
		int i,j;
		matrix mat(rowWidth,colWidth);
		for(i=0;i<rowWidth;i++)
		{
			for(j=0;j<colWidth;j++)
				mat(i,j)=(*this)(rowIndex+i,colIndex+j);
		}

		return mat;
	}

	void plusSubMatrix(matrix mat, int rowIndex, int colIndex, int rowWidth, int colWidth)
	{
		int i,j;
		for(i=0;i<rowWidth;i++)
		{
			for(j=0;j<colWidth;j++)
				(*this)(rowIndex+i,colIndex+j)+=mat(i,j);
		}
	}

	// mat 객체를 this 객체로 복사 
	void copyFrom (const matrix & mat)
	{
		New (mat.row, mat.col);
		std::copy (mat.begin(), mat.end(), begin());
		determinant = mat.determinant;
	};

	// double *를 this 객체로 복사
	void copyFrom (double *Data, int Row, int Col)
	{
		New (Row, Col);
		std::copy (Data, Data+(Row*Col), begin());
	};

	// this 객체를 mat으로 복사 
	void copyTo (matrix & mat)
	{
		mat.New (row, col);
		std::copy (begin(), end(), mat.begin());
	};

	// row vector를 normal vector로 만든다
	void normalizeRowVector()
	{
		int i,j;
		float det;
		for(i=0;i<row;i++)
		{
			det=0;
			for(j=0;j<col;j++)
				det+=data[i*col+j]*data[i*col+j];
			det=sqrt(det);
			for(j=0;j<col;j++)
				data[i*col+j]/=det;
		}
	};

	// column vector를 normal vector로 만든다
	void normalizeColVector()
	{
		int i,j;
		float det;
		for(i=0;i<col;i++)
		{
			det=0;
			for(j=0;j<row;j++)
				det+=data[i+col*j]*data[i+col*j];
			det=sqrt(det);
			for(j=0;j<row;j++)
				data[i+col*j]/=det;
		}
	};
	
	// 전치 행렬을 구한다. 
	matrix transpose()
	{
		matrix matC(row, col);

		// 행과 열을 바꾼다. 
		matC.row = col;
		matC.col = row;

		for (int r=0; r<row; r++)
			for (int c=0; c<col; c++)
				matC(c, r) = (*this)(r, c); /* (aij)^t = aji */

		return matC;
	};

	// 전치 행렬을 구한다. 
	void transposeSelf()
	{
		matrix matC(row, col);

		// 행과 열을 바꾼다. 
		matC.row = col;
		matC.col = row;

		for (int r=0; r<row; r++)
			for (int c=0; c<col; c++)
				matC(c, r) = (*this)(r, c); 
		copyFrom(matC);
	};

	//identity 행렬로 만든다
	void identity()
	{
		int i,j;
		if(row==col)
		{
			for(i=0;i<row;i++)
			{
				for(j=0;j<col;j++)
				{
					if(i==j)
						(*this)(i,j)=1;
					else
						(*this)(i,j)=0;
				}
			}
		}
	}

	// 역행렬 계산 (알고리즘: Gauss-Jordan) 
	matrix inverse () //eps=1.0e-6
	{
		double eps=1.0e-6;
		matrix matC;
		matC.copyFrom (*this);

		int *work;
		int i, j, k, r, iw, s, t, u, v;
		double w, wmax, pivot, api, w1;

		work = new int [row];
		w1 = 1.0;
		for ( i=0; i<row; i++ ) work[i] = i;

		for ( k=0; k<row; k++ )  {
			wmax = -999999999.0;
			for ( i=k; i<row; i++ ) {
				w = fabs ( matC[i*col+k] );
				if( w > wmax ) {
					wmax = w;
					r = i;
				}
			}
			pivot = matC [r*col+k];
			api = fabs (pivot);

			// 피봇요소가 e보다 작을 정도로 0에 가까울 때 
			if ( api <= eps ) {
				matC.determinant = determinant = w1;
				delete [] work;
				return matC;
			}
			w1 *= pivot;
			u = k * col;
			v = r * col;

			// pivoting 
			if (r != k) {
				w1 = -w;
				iw = work[k];
				work[k] = work[r];
				work[r] = iw;
				for ( j=0; j<row; j++ ) {
					s = u + j;
					t = v + j;
					w = matC[s];
					matC[s] = matC[t];
					matC[t] = w;
				}
			}
			for ( i=0; i<row; i++ )
				matC[u+i] /= pivot;
			for ( i=0; i<row; i++ ) {
				if ( i != k) {
					v= i*col;
					s = v+k;
					w = matC[s];
					if ( w != 0.0 ) {
						for ( j=0; j<row; j++ )
							if (j != k) matC[v+j] -= w * matC[u+j];
						matC[s] = -w / pivot;
					}
				}
			}
			matC[u+k] = 1.0 / pivot;
		}

		for ( i=0; i<row; i++ ) {
			while (1) {
				k = work[i];
				if (k == i) break;
				iw = work[k];
				work[k] = work[i];
				work[i] = iw;
				for ( j=0; j<row; j++ ) {
					u = j*col;
					s = u + i;
					t = u + k;
					w = matC[s];
					matC[s] = matC[t];
					matC[t] = w;
				}
			}
		}

		// 행렬식은 역행렬을 구할 때 얻어짐!!! 
		matC.determinant = determinant = w1;
		delete [] work;

		return matC;
	};

	int GaussElimination(matrix* f, matrix* u)
	{
		//square matrix가 아니면 종료
		if(!(col==row))
			return -1;

		//vector와 matrix의 dimension이 같지 않으면 종료
		if(!(row==f->row))
			return -1;
		if(!(row==u->row))
			return -1;

		int i,j;
		int n=col;

		//er=-1일 경우 singular matrix로 계산이 정지 된다
		int er=0;

		//pivot 요소가 0에 가까우면 singular
		double eps=1.0e-6;

		//계수중 가장큰 요소를 저장하는 배열
		double* s=new double[n];

		//계수중 가장큰 요소를 찾는다
		for(i=0;i<n;i++)
		{
			s[i]=abs((*this)(i,0));
			for(j=1;j<n;j++)
			{
				if(abs((*this)(i,j))>s[i])
					s[i]=abs((*this)(i,j));
			}
		}
		er=Eliminate(s,n,f,eps);

		if(er==-1)
		{
			delete [] s;
			return -1;
		}
		else
		{
			delete [] s;
			return Substitute(n,f,u);
		}
	};

	int Eliminate(double* s, int n, matrix* f,  double eps)
	{
		int i,j,k;
		double factor;
		for(k=0;k<n-1;k++)
		{
			Pivot(s, n, f, k);
			double a=fabs((*this)(k,k)/s[k]);
			if(fabs((*this)(k,k)/s[k])<eps)
				return -1;
			for(i=k+1;i<n;i++)
			{
				factor=(*this)(i,k)/(*this)(k,k);
				for(j=k+1;j<n;j++)
					(*this)(i,j)=(*this)(i,j)-factor*(*this)(k,j);
				(*f)(i,0)=(*f)(i,0)-factor*(*f)(k,0);
			}
		}
		if(fabs((*this)(k,k)/s[k])<eps)
			return -1;
		return 1;
	};

	void Pivot(double* s, int n, matrix* f, int k)
	{
		int i;
		double dummy;
		int p=k;

		//정규화, 행백터의 가장큰 요소를 1이 되도록 한다
		double big=fabs((*this)(k,k)/s[k]);

		for(i=k+1;i<n;i++)
		{
			dummy=fabs((*this)(i,k)/s[i]);
			if(dummy>big)
			{
				big=dummy;
				p=i;
			}
		}

		//pivot이 필요한 경우
		if(!(p==k))
		{
			for(i=k;i<n;i++)
			{
				dummy=(*this)(p,i);
				(*this)(p,i)=(*this)(k,i);
				(*this)(k,i)=dummy;
			}
			dummy=(*f)(p,0);
			(*f)(p,0)=(*f)(k,0);
			(*f)(k,0)=dummy;

			dummy=s[p];
			s[p]=s[k];
			s[k]=dummy;
		}
	};

	int Substitute(int n, matrix* f, matrix* u)
	{
		int i,j;
		(*u)(n-1,0)=(*f)(n-1,0)/(*this)(n-1,n-1);
		for(i=n-2;i>-1;i--)
		{
			double sum=0;
			for(j=i+1;j<n;j++)
				sum=sum+(*this)(i,j)*(*u)(j,0);
			(*u)(i,0)=((*f)(i,0)-sum)/(*this)(i,i);
		}
		return 1;
	};

	vec3d mulVector(vec3d p)
	{
		vec3d ref;
		if(row==3 && col==3)
		{
			ref.x=(*this)(0,0)*p.x+(*this)(0,1)*p.y+(*this)(0,2)*p.z;
			ref.y=(*this)(1,0)*p.x+(*this)(1,1)*p.y+(*this)(1,2)*p.z;
			ref.z=(*this)(2,0)*p.x+(*this)(2,1)*p.y+(*this)(2,2)*p.z;
		}
		return ref;
	};

	void Print(char* filename)
	{
		int i,j;
		FILE* f=fopen(filename,"w");
		fprintf(f,"%d %d\n",row,col);
		for(i=0;i<row;i++)
		{
			for(j=0;j<col;j++)
				fprintf(f,"%f ",(*this)(i,j));
			fprintf(f,"\n");
		}
		fclose(f);
	};

	void Read(char* filename)
	{
		int i,j;
		int Row,Col;
		FILE* f=fopen(filename,"r");
		fscanf(f,"%d %d",&Row,&Col);
		New(Row,Col);
		for(i=0;i<row;i++)
		{
			for(j=0;j<col;j++)
				fscanf(f,"%lf",&(*this)(i,j));
		}
		fclose(f);
	};
};

#endif