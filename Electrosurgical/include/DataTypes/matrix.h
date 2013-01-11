
#ifndef MATRIX_H
#define MATRIX_H

class Matrix
{
private:
	//행렬식 저장
	float Determinant; 
	float* Elem;
	int Column;
	int Row;

public:
	Matrix()
	{
		Column=0; Row=0; Determinant=0; Elem=NULL;
	}
	Matrix(int row, int col)
	{
		Column=col;
		Row=row;
		Elem=new float[row*col];
		for(int i=0;i<row*col;i++)
			Elem[i]=0.0;
		Determinant=0;
	};

	Matrix(const Matrix & aRef)
	{
		Elem=NULL;
		Determinant=aRef.determinant();
		Column=aRef.column();
		Row=aRef.row();
		New(aRef.row(), aRef.column());
		std::copy (aRef.begin(), aRef.end(), begin());
	};

	~Matrix()
	{
		if(Elem)
		{
			delete [] Elem;
			Elem=NULL;
		}
	};

	void init(int row, int col)
	{
		Column=col;
		Row=row;
		if(!Elem)
			Elem=new float[row*col];
		for(int i=0;i<row*col;i++)
			Elem[i]=0.0;
		Determinant=0;
	};

	void New(int row, int col)
	{
		float *newData = new float [row*col];
		for(int i=0;i<row*col;i++)
			newData[i]=0;
		if(Elem)
			delete [] Elem;
		Elem = newData;
		Row = row;
		Column = col;
	};

	/// 시작과 끝 포인터를 리턴 
	float *begin () { return Elem; };
	float *end () { return Elem+(Row*Column); };

	const float *begin () const { return Elem; };
	const float *end () const { return Elem+(Row*Column); };

	int row() {return Row;};
	int column() {return Column;};
	const int row() const {return Row;};
	const int column() const {return Column;};
	float* getMatrix(){if(Elem) return Elem;};
	float determinant(){return Determinant;};
	const float determinant() const {return Determinant;};


	// 할당 연산자 : = 
	Matrix & operator=(const Matrix & mat)
	{
		New (mat.row(), mat.column());
		std::copy (mat.begin(), mat.end(), begin());

		Determinant = mat.determinant();
		return *this;
	};

	float  operator [] (int index) const {return Elem[index];};
	float& operator [] (int index) {return Elem[index];};

	float & operator() (int row, int col)
	{ 
		return Elem[row*Column+col]; 
	};

	Matrix & operator*=(const float & n) 
	{
		int i,j;
		for(i=0;i<Row;i++)
		{
			for(j=0;j<Column;j++)
				Elem[i*Column+j]*=n;
		}
		return *this;
	};

	Matrix operator*(const float & n) const
	{
		Matrix rt(*this);
		return rt*=n;
	};

	Matrix operator*(const Matrix & mat) const 
	{
		int i,j,k;
		Matrix ref(Row, mat.column());
		if(!(Column==mat.row()))
			return ref;

		const float* elem=mat.Elem;
		const int _column=mat.Column;
		for(i=0;i<Row;i++)
		{
			//for(j=0;j<mat.Column;j++)
			for(j=0;j<_column;j++)
			{
				for(k=0;k<Column;k++)
					ref.Elem[i*ref.Column+j]+=Elem[i*Column+k]*elem[k*_column+j];
					//ref.Elem[i*ref.Column+j]+=Elem[i*Column+k]*mat.Elem[k*mat.Column+j];
					//ref(i,j)+=Elem[i*Column+k]*mat.Elem[k*mat.column()+j];
			}
		}
		return ref;
	};

	Matrix & operator/=(const float & n) 
	{
		int i,j;
		for(i=0;i<Row;i++)
		{
			for(j=0;j<Column;j++)
				Elem[i*Column+j]/=n;
		}
		return *this;
	};

	Matrix operator/(const float & n) const
	{
		Matrix rt(*this);
		return rt/=n;
	};

	Matrix & operator+=(const Matrix &v)
	{
		int i,j;
		if(!(Row==v.Row) || !(Column==v.Column))
			return *this;

		for(i=0;i<Row;i++)
		{
			for(j=0;j<Column;j++)
				Elem[i*Column+j]+=v.Elem[i*Column+j];
		}
		return *this;
	};

	Matrix operator + (const Matrix &ref) const
	{
		Matrix rt(*this);
		return rt+=ref;
	};

	Matrix & operator-=(const Matrix &v)
	{
		int i,j;
		if(!(Row==v.Row) || !(Column==v.Column))
			return *this;

		for(i=0;i<Row;i++)
		{
			for(j=0;j<Column;j++)
				Elem[i*Column+j]-=v.Elem[i*Column+j];
		}
		return *this;
	};

	Matrix operator - (const Matrix &ref) const
	{
		Matrix rt(*this);
		return rt-=ref;
	};

	// 두행렬 일치성 검사 
	bool operator == (const Matrix & mat) const
	{
		if (!(this->Row == mat.Row && this->Column==mat.Column))
			return false;

		return std::equal (begin(), end(), mat.begin());
	};


	// 전치 행렬을 구한다. 
	Matrix transpose()
	{
		Matrix matC(Row, Column);

		// 행과 열을 바꾼다. 
		matC.Row = Column;
		matC.Column = Row;

		for (int r=0; r<Row; r++)
			for (int c=0; c<Column; c++)
				matC(c, r) = (*this)(r, c); 
		return matC;
	};

	void transposeSelf()
	{ 
		for (int i=0;i<Row;i++)
			for (int j=i+1;j<Column;j++)
			{
				float t = this->Elem[i*Column+j];
				this->Elem[i*Column+j] = this->Elem[j*Column+i];
				this->Elem[j*Column+i] = t;
			}
	};

	//identity 행렬로 만든다
/*	void identity()
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
	Matrix inverse () //eps=1.0e-6
	{
		float eps=1.0e-6;
		Matrix matC;
		matC.copyFrom (*this);

		int *work;
		int i, j, k, r, iw, s, t, u, v;
		float w, wmax, pivot, api, w1;

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

	int GaussElimination(Matrix* f, Matrix* u)
	{
		//square Matrix가 아니면 종료
		if(!(col==row))
			return -1;

		//vector와 Matrix의 dimension이 같지 않으면 종료
		if(!(row==f->row))
			return -1;
		if(!(row==u->row))
			return -1;

		int i,j;
		int n=col;

		//er=-1일 경우 singular Matrix로 계산이 정지 된다
		int er=0;

		//pivot 요소가 0에 가까우면 singular
		float eps=1.0e-6;

		//계수중 가장큰 요소를 저장하는 배열
		float* s=new float[n];

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

	int Eliminate(float* s, int n, Matrix* f,  float eps)
	{
		int i,j,k;
		float factor;
		for(k=0;k<n-1;k++)
		{
			Pivot(s, n, f, k);
			float a=fabs((*this)(k,k)/s[k]);
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

	void Pivot(float* s, int n, Matrix* f, int k)
	{
		int i;
		float dummy;
		int p=k;

		//정규화, 행백터의 가장큰 요소를 1이 되도록 한다
		float big=fabs((*this)(k,k)/s[k]);

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

	int Substitute(int n, Matrix* f, Matrix* u)
	{
		int i,j;
		(*u)(n-1,0)=(*f)(n-1,0)/(*this)(n-1,n-1);
		for(i=n-2;i>-1;i--)
		{
			float sum=0;
			for(j=i+1;j<n;j++)
				sum=sum+(*this)(i,j)*(*u)(j,0);
			(*u)(i,0)=((*f)(i,0)-sum)/(*this)(i,i);
		}
		return 1;
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
	};*/
};

#endif
