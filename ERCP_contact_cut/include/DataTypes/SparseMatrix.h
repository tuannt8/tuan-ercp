#ifndef SPARSE_MATRIX
#define SPARSE_MATRIX

const int MaxTerm=10000;

class MatrixTerm
{
public:
	friend class SparseMatrix;
	int col,row;
	float value;
};

class SparseMatrix
{

public:
	SparseMatrix(int MaxRow,int MaxCol,int term)
	{
		Rows=MaxRow;Cols=MaxCol;Terms=term;
		idx=0;
	}; 
	SparseMatrix(void)
	{
	};  
	~SparseMatrix(void)
	{
	};

public:
	int Rows,Cols,Terms,idx;
	MatrixTerm smArray[MaxTerm];

public:
	SparseMatrix Transpose()
	{
		SparseMatrix b;
		b.Rows=Cols;
		b.Cols=Rows;
		b.Terms=Terms;
		if(Terms>0)
		{
			int CurrentB=0;
			for(int c=0;c<Cols;c++)
				for(int i=0;i<Terms;i++)
					if(smArray[i].col==c){
						b.smArray[CurrentB].row=c;
						b.smArray[CurrentB].col=smArray[i].row;
						b.smArray[CurrentB].value=smArray[i].value;
						CurrentB++;
					}
		}
		return b;
	};

	void storeMatrix(int r,int c,float v)
	{
		smArray[idx].row=r;
		smArray[idx].col=c;
		smArray[idx++].value=v;
	};
	
	int SparseMatrix::StoreSum(float sum, int& LastInResult, int r, int c)
	{
		if(sum!=0){
			if(LastInResult<MaxTerm-1){
				LastInResult++;
				smArray[LastInResult].row=r;
				smArray[LastInResult].col=c;
				smArray[LastInResult].value=sum;
				return 0;
			}
			else{
				return 1;
			}
		}
		else return 0;
	}; 

	SparseMatrix Multiply(SparseMatrix b)
	{
		SparseMatrix bXpose = b.Transpose();
		SparseMatrix result;

		int currRowIndex = 0, LastInResult = -1, currRowBegin = 0, currRowA = smArray[0].row;
		smArray[Terms].row = Rows;
		bXpose.smArray[b.Terms].row = b.Cols;
		bXpose.smArray[b.Terms].col = -1;
		float sum = 0;
		while ( currRowIndex < Terms ) {
			int currColB = bXpose.smArray[0].row;
			int currColIndex = 0;
			while ( currColIndex <= b.Terms ) {
				if ( smArray[currRowIndex].row != currRowA ) {
					if ( result.StoreSum(sum, LastInResult, currRowA, currColB) ){}
					//     return EmptyMatrix();
					else sum = 0;
					currRowIndex = currRowBegin;
					while ( bXpose.smArray[currColIndex].row == currColB ) currColIndex++;
					currColB = bXpose.smArray[currColIndex].row;
				}
				else if ( bXpose.smArray[currColIndex].row != currColB ) {
					if ( result.StoreSum(sum, LastInResult, currRowA, currColB) ){}
					//     return EmptyMatrix();
					else sum = 0;
					currRowIndex = currRowBegin;
					currColB = bXpose.smArray[currColIndex].row;
				}
				else
					switch ( compare(smArray[currRowIndex].col, bXpose.smArray[currColIndex].col) ){
					case '<':
						currRowIndex++; break;
					case '=':
						sum += smArray[currRowIndex].value * bXpose.smArray[currColIndex].value;
						currRowIndex++; currColIndex++; break;
					case '>':
						currColIndex++; break;
									}
			}
			while ( smArray[currRowIndex].row == currRowA ) currRowIndex++;
			currRowBegin = currRowIndex;
			currRowA = smArray[currRowIndex].row;
		}
		result.Rows = Rows; result.Cols = b.Cols; result.Terms = LastInResult + 1;
		return result;
	}; 
	
	char compare(int x,int y)
	{
		if(x>y) return '>';
		else if(x<y) return '<';
		else return '=';
	}; 
};

#endif