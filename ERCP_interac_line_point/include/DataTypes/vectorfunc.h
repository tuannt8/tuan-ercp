/********************************************	
*		Original source code: SOFA			*
*		Modified by Hoeryong Jung			*
*		Date: 2009. 10. 21					*
*		contact: junghl80@kaist.ac.kr		*
/*******************************************/

#ifndef VECTOR_FUNC_H
#define VECTOR_FUNC_H

#include <vector>
#include "DataTypes/Vec.h"
#include "DataTypes/Mat.h"

class VectorFunc
{
public:
	VectorFunc()
	{
	};
	~VectorFunc()
	{
	};
	
	bool isElementInVector(std::vector<int>& v, int value)
	{
		for(int i=0;i<v.size();i++)
		{
			if(v[i]==value)
				return true;
		}
		return false;
	};
	bool isElementInVector(std::vector<int>* v, int value)
	{
		for(int i=0;i<v->size();i++)
		{
			if((*v)[i]==value)
				return true;
		}
		return false;
	};

	void removeElementValue(std::vector<int>& v, int eValue)
	{
		for(int i=0;i<v.size();i++)
		{
			if(v[i]==eValue)
			{
				removeElement(v,i);
				break;
			}
		}
	}
	void removeElement(std::vector<int>& v, int localIdx)
	{
		v[localIdx]=v[v.size()-1];
		v.pop_back();
	};
	void removeElement(std::vector<float>& v, int localIdx)
	{
		v[localIdx]=v[v.size()-1];
		v.pop_back();
	};
	void removeElement(std::vector<Vec4i>& v, int localIdx)
	{
		v[localIdx]=v[v.size()-1];
		v.pop_back();
	};
	int indexOfElement(std::vector<int>& v, int eValue)
	{
		for(int i=0;i<v.size();i++)
		{
			if(v[i]==eValue)
			{
				return i;
			}
		}
		return -1;
	}
	int indexOfElement(std::vector<int>* v, int value)
	{
		std::vector< int >::iterator location=v->end();

		location = std::find( v->begin(), v->end(), value );

		if (location != v->end())
		{
			return std::distance(v->begin(), location);
		}
		
		return -1;
	}
	void removeElement(std::vector<Vec3f>& v, int localIdx)
	{
		v[localIdx]=v[v.size()-1];
		v.pop_back();
	};
	void arrangeVector(std::vector<int>& v)
	{
		if(v.size()>1)
		{
			std::vector<int> temp;
			quickSort(v, 0, v.size()-1);
			for(int i=0;i<v.size()-1;i++)
			{
				if(!(v[i]==v[i+1]))
					temp.push_back(v[i]);
			}
			temp.push_back(v[v.size()-1]);
			v.clear();
			for(int i=0;i<temp.size();i++)
				v.push_back(temp[i]);
		}
	};
	void quickSort(std::vector<int>& num, int left, int right)
	{
		if(left<right)
		{
			int pivotIdx=(left+right)/2;
			pivotIdx=partition(num, left, right, pivotIdx);
			quickSort(num,left,pivotIdx-1);
			quickSort(num,pivotIdx+1, right);
		}
	};
	int partition(std::vector<int>& num, int left, int right, int pivotIdx)
	{
		int pivotValue=num[pivotIdx];
		num[pivotIdx]=num[right];
		num[right]=pivotValue;
		int storeIdx=left;
		for(int i=left;i<right;i++)
		{
			if(num[i]<pivotValue)
			{
				int temp=num[storeIdx];
				num[storeIdx]=num[i];
				num[i]=temp;
				storeIdx=storeIdx+1;
			}
		}
		int temp=num[storeIdx];
		num[storeIdx]=num[right];
		num[right]=temp;
		return storeIdx;
	};

	void arrangeVector(std::vector<Vec2f>& v)
	{
		if(v.size()>1)
		{
			std::vector<Vec2f> temp;
			temp.clear();
			quickSort(v, 0, v.size()-1);
			temp.push_back(v[0]);

			for(int i=1;i<v.size();i++)
			{
				if(!(v[i-1][0]==v[i][0])){
					if(v[i][1])
					{
						temp.push_back(v[i]);
					}
				}else{
					if(v[i][1])
					{
						temp[temp.size()-1][1]+=v[i][1];
					}
				}
				
			}
			
			v.clear();
			for(int i=0;i<temp.size();i++)
				v.push_back(temp[i]);
		}
	};
	void quickSort(std::vector<Vec2f>& num, int left, int right)
	{
		if(left<right)
		{
			int pivotIdx=(left+right)/2;
			pivotIdx=partition(num, left, right, pivotIdx);
			quickSort(num,left,pivotIdx-1);
			quickSort(num,pivotIdx+1, right);
		}
	};
	int partition(std::vector<Vec2f>& num, int left, int right, int pivotIdx)
	{
		int pivotValue=num[pivotIdx][0];
		num[pivotIdx][0]=num[right][0];
		num[right][0]=pivotValue;
		int storeIdx=left;
		for(int i=left;i<right;i++)
		{
			if(num[i][0]<pivotValue)
			{
				int temp=num[storeIdx][0];
				float temp2=num[storeIdx][1];
				num[storeIdx][0]=num[i][0];
				num[i][0]=temp;
				num[storeIdx][1]=num[i][1];
				num[i][1]=temp2;
				storeIdx=storeIdx+1;
			}
		}
		int temp=num[storeIdx][0];
		float temp2=num[storeIdx][1];
		num[storeIdx][0]=num[right][0];
		num[right][0]=temp;
		num[storeIdx][1]=num[right][1];
		num[right][1]=temp2;
		return storeIdx;
	};

	void quickSort(std::vector<float>& num, int left, int right)
	{
		if(left<right)
		{
			int pivotIdx=(left+right)/2;
			pivotIdx=partition(num, left, right, pivotIdx);
			quickSort(num,left,pivotIdx-1);
			quickSort(num,pivotIdx+1, right);
		}
	};
	int partition(std::vector<float>& num, int left, int right, int pivotIdx)
	{
		float pivotValue=num[pivotIdx];
		num[pivotIdx]=num[right];
		num[right]=pivotValue;
		int storeIdx=left;
		for(int i=left;i<right;i++)
		{
			if(num[i]<pivotValue)
			{
				float temp=num[storeIdx];
				num[storeIdx]=num[i];
				num[i]=temp;
				storeIdx=storeIdx+1;
			}
		}
		float temp=num[storeIdx];
		num[storeIdx]=num[right];
		num[right]=temp;
		return storeIdx;
	};
	
	void constructMatFromVec(Vec3f v1, Vec3f v2, Mat3x3f& mat)
	{
		for(int i=0;i<3;i++)
		{
			for(int j=0;j<3;j++)
				mat[i][j]=v1[i]*v2[j];
		}
	};
};
#endif
