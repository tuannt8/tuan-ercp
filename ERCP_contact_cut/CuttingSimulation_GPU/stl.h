#if !defined(_STL_H_)
#define _STL_H_

#include "DataStruct.h"

#define MAX		100000000
#define MIN		-100000000

class CSTL  
{
//hash table
typedef struct
{
	vec3d Point[10000];
	int PointIndex[10000];
	int PointNum;
}	Hash;

private:
	vec3d*		m_Point;
	vec3d*		m_FaceNormal;
	vec3d*		m_PointNormal;
	vec3i*		m_Face;

	int			m_PointNum;
	int			m_FaceNum;

	Hash*		m_Hash;

public:
	CSTL()
	{
		m_Point=NULL;
		m_Face=NULL;
		m_FaceNormal=NULL;
		m_PointNormal=NULL;
		m_Hash=NULL;

		m_PointNum=0;
		m_FaceNum=0;
	};

	~CSTL()
	{
		if(m_Point){ delete [] m_Point;	m_Point=NULL;}
		if(m_Face){ delete [] m_Face; m_Face=NULL;}
		if(m_FaceNormal){ delete [] m_FaceNormal; m_FaceNormal=NULL;}
		if(m_PointNormal){ delete [] m_PointNormal;	m_PointNormal=NULL;}
	};

	int ReadData(char* filename)
	{
		FILE *fp;
		FILE *f;
		int i;

		fp = fopen(filename,"r");
		f=fopen("et.txt","w");

		if (!fp)
		{
			return 0;
		}

		//file
		char str[100];
		CString temp;
		CString normal="normal";
		CString vertex="vertex";
		CString endloop="endloop";
		CString endsolid="endsolid";

		//
		int faceCount=0;
		int maxX,maxY,maxZ;
		int minX,minY,minZ;
		int hashLength, hashMin, hashIndex;
		CString hashDirec;
		vec3d tempPoint;

		maxX=maxY=maxZ=MIN;
		minX=minY=minZ=MAX;

		while(1)
		{
			fscanf(fp, "%s\n", str);
			temp=str;
			if(temp==endloop)
				faceCount++;
			
			//point
			if(temp==vertex)
			{
				//x
				fscanf(fp, "%s\n", str);
				temp=str;
				tempPoint.x=atof(temp);
				if(tempPoint.x>maxX)
					maxX=tempPoint.x;
				if(tempPoint.x<minX)
					minX=tempPoint.x;

				//y
				fscanf(fp, "%s\n", str);
				temp=str;
				tempPoint.y=atof(temp);
				if(tempPoint.y>maxY)
					maxY=tempPoint.y;
				if(tempPoint.y<minY)
					minY=tempPoint.y;

				//z
				fscanf(fp, "%s\n", str);
				temp=str;
				tempPoint.z=atof(temp);
				if(tempPoint.z>maxZ)
					maxZ=tempPoint.z;
				if(tempPoint.z<minZ)
					minZ=tempPoint.z;
			}

			//file
			if(temp==endsolid)
			{
				//x,y,z
				if((maxX-minX)>(maxY-minY))
				{
					if((maxX-minX)>(maxZ-minZ))
					{
						hashLength=maxX-minX;
						hashMin=minX;
						hashDirec="x";
					}
					else
					{
						hashLength=maxZ-minZ;
						hashMin=minZ;
						hashDirec="z";
					}
				}
				else
				{
					if((maxY-minY)>(maxZ-minZ))
					{
						hashLength=maxY-minY;
						hashMin=minY;
						hashDirec="y";
					}
					else
					{
						hashLength=maxZ-minZ;
						hashMin=minZ;
						hashDirec="z";
					}
				}
				break;
			}
		}
		fclose(fp);
		m_FaceNum=faceCount;
		hashLength+=1;

		fp=fopen(filename,"r");
		
		//m_Face m_FaceNormal
		m_Face=new vec3i[m_FaceNum];
		m_FaceNormal=new vec3d[m_FaceNum];

		//hash table�� �޸𸮸� �Ҵ��Ѵ�
		m_Hash=new Hash[hashLength];

		//hash table�ʱ�ȭ
		for(i=0;i<hashLength;i++)
			m_Hash[i].PointNum=0;

		//�ӽ÷� point��ǥ�� �����ϴ� �迭�� memory�� �Ҵ��Ѵ�
		vec3d *ptempPoint=new vec3d[m_FaceNum];
		
		//face normal vector�� point��ǥ�� �����Ѵ�
		int pointCount=0;
		faceCount=0;

		//���� ����Ǿ���� point�� ���� ������� ǥ��
		CString currentSavePoint="ii";
		CString ii="ii"; CString jj="jj"; CString kk="kk";
		while(1)
		{
			fscanf(fp, "%s\n", str);
			temp=str;

			//face normal vector�� �����Ѵ�
			if(temp==normal)
			{
				//x direction
				fscanf(fp, "%s\n", str);
				temp=str;
				m_FaceNormal[faceCount].x=atof(temp);

				//y direction
				fscanf(fp, "%s\n", str);
				temp=str;
				m_FaceNormal[faceCount].y=atof(temp);

				//z direction
				fscanf(fp, "%s\n", str);
				temp=str;
				m_FaceNormal[faceCount].z=atof(temp);

				faceCount++;
			}

			//vertex��ǥ�� �����Ѵ�
			vec3d comparePoint;
			int pointIndex;
			BOOL isSamePoint;
			if(temp==vertex)
			{
				//x ��ǥ
				fscanf(fp, "%s\n", str);
				temp=str;
				comparePoint.x=atof(temp);

				//y ��ǥ
				fscanf(fp, "%s\n", str);
				temp=str;
				comparePoint.y=atof(temp);

				//z ��ǥ
				fscanf(fp, "%s\n", str);
				temp=str;
				comparePoint.z=atof(temp);
				
				isSamePoint=false;

				//�о�� point�� ����Ǿ���� hash table�� index�� ���Ѵ�
				if(hashDirec=="x")
					hashIndex=comparePoint.x-hashMin;
				if(hashDirec=="y")
					hashIndex=comparePoint.y-hashMin;
				if(hashDirec=="z")
					hashIndex=comparePoint.z-hashMin;

			
				//�̹� ����� point��� ���� point���� �˻�, hash table�ȿ��� �˻�
				for(i=0;i<m_Hash[hashIndex].PointNum;i++)
				{
					if(m_Hash[hashIndex].Point[i]==comparePoint)
					{
						pointIndex=m_Hash[hashIndex].PointIndex[i];
						isSamePoint=true;
						break;
					}
				}

				//�̹� ����� point��� �ٸ����� point�� �߰��Ѵ�
				if(!isSamePoint)
				{
					//�ӽ� point������ġ�� point�� �����Ѵ�
					ptempPoint[pointCount]=comparePoint;

					//hash table�� point�� �����Ѵ�
					m_Hash[hashIndex].Point[m_Hash[hashIndex].PointNum]=comparePoint;
					m_Hash[hashIndex].PointIndex[m_Hash[hashIndex].PointNum]=pointCount;
					m_Hash[hashIndex].PointNum++;
					
					if(currentSavePoint==ii)
					{
						m_Face[faceCount-1].ii=pointCount;
						currentSavePoint="jj";
					}
					else if(currentSavePoint==jj)
					{
						m_Face[faceCount-1].jj=pointCount;
						currentSavePoint="kk";
					}
					else
					{
						m_Face[faceCount-1].kk=pointCount;
						currentSavePoint="ii";
					}
					pointCount++;
				}

				//�̹� ����� point�̸� point���� �߰����� �ʰ� ���������� point��ȣ�� ����
				else
				{
					if(currentSavePoint==ii)
					{
						m_Face[faceCount-1].ii=pointIndex;
						currentSavePoint="jj";
					}
					else if(currentSavePoint==jj)
					{
						m_Face[faceCount-1].jj=pointIndex;
						currentSavePoint="kk";
					}
					else
					{
						m_Face[faceCount-1].kk=pointIndex;
						currentSavePoint="ii";
					}
				}

			}

			//file�� ������ ������ �������´�
			if(temp==endsolid)
			{
				//point��ǥ�� �����Ѵ�
				m_PointNum=pointCount;
				m_Point=new vec3d[m_PointNum];
				for(i=0;i<m_PointNum;i++)
					m_Point[i]=ptempPoint[i];
				break;
			}
		}
		fclose(fp);
		
		//hash table�� memory ����
		if(m_Hash)
		{
			delete [] m_Hash;
			m_Hash=NULL;
		}

		if(ptempPoint)
		{
			delete []ptempPoint;
			ptempPoint = NULL;
		}

		return 1;
	};

	int WriteToObj(char* filename)
	{
		int i;
		FILE* fp;
		fp=fopen(filename,"w");
		
		if(m_Point && m_Face)
		{
			fprintf(fp,"%d\n",m_PointNum);
			for(i=0;i<m_PointNum;i++)
				fprintf(fp,"%f %f %f\n",m_Point[i].x,m_Point[i].y,m_Point[i].z);
			fprintf(fp,"%d\n",m_FaceNum);
			for(i=0;i<m_FaceNum;i++)
				fprintf(fp,"%d %d %d\n",m_Face[i].ii,m_Face[i].jj,m_Face[i].kk);

			fclose(fp);
			return 1;
		}
		else
		{
			fclose(fp);
			return 0;
		}
	};

	vec3d* GetPoint()
	{
		if(m_Point)
			return m_Point;
		else
			return NULL;
	};

	vec3i* GetFace()
	{
		if(m_Face)
			return m_Face;
		else
			return NULL;
	};

	vec3d* GetFaceNormal()
	{
		if(m_FaceNormal)
			return m_FaceNormal;
		else
			return NULL;
	};

	vec3d* GetPointNormal()
	{
		if(m_PointNormal)
			return m_PointNormal;
		else
			return NULL;
	};

	int GetFaceNum()
	{
		return m_FaceNum;
	};

	int GetPointNum()
	{
		return m_PointNum;
	};
};

#endif
