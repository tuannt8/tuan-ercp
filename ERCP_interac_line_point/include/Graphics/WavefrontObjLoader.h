// WavefrontObjLoader.h: interface for the CObj class.
//
//////////////////////////////////////////////////////////////////////
#ifndef WAVEFRONT_OBJ_LOADER_H
#define WAVEFRONT_OBJ_LOADER_H

#include "./DataTypes/Vec.h"
#include "./DataTypes/Mat.h"
#include <afxwin.h>
#include <vector>

class WaveFrontObjLoader  
{

private:
	Vec3d* m_Point;
	Vec3i* m_Face;
	int m_PointNum;
	int m_FaceNum;

public:
	WaveFrontObjLoader()
	{
		m_Point=NULL;
		m_Face=NULL;
	};

	~WaveFrontObjLoader()
	{
		if(m_Point)
			delete [] m_Point;
		if(m_Face)
			delete [] m_Face;
	};

	int ReadData(char* filename)
	{
		FILE *fp;
		fp = fopen(filename,"r");
		
		std::vector<Vec3d> Point;
		std::vector<Vec3i> Face;
		Vec3d point;
		Vec3i face;
		char temp[100];
		CString trash;
		
		while(1)
		{
			fscanf(fp,"%s",&temp);
			if(((temp[0])=='e')&&((temp[1])=='n')&&((temp[2])=='d')&&((temp[3])=='o')&&((temp[4])=='f')&&((temp[5])=='f')&&((temp[6])=='i')&&((temp[7])=='l')&&((temp[8])=='e'))
				break;
			if(int(temp[1]==0))
			{
				if(temp[0]=='v')
				{
					fscanf(fp,"%lf %lf %lf", &point[0], &point[1], &point[2]);
					Point.push_back(point);
				}
				if(temp[0]=='f')
				{
					fscanf(fp,"%d %s %d %s %d %s", &face[0], &temp, &face[1], &temp, &face[2], &temp);
					//fscanf(fp,"%d %d %d", &face[0], &face[1], &face[2]);
					face[0]--; face[1]--; face[2]--;
					Face.push_back(face);
				}
			}
		}
		m_PointNum=(int)Point.size();
		m_FaceNum=(int)Face.size();
		m_Point=new Vec3d[m_PointNum];
		m_Face=new Vec3i[m_FaceNum];

		for(int i=0;i<m_PointNum;i++)
			m_Point[i]=Point[i];
		for(int i=0;i<m_FaceNum;i++)
			m_Face[i]=Face[i];
		return 0;
	};

	int WriteObjData(char* filename)
	{
		FILE *fp;
		int i;

		fp = fopen(filename,"w");
		
		fprintf(fp,"%d\n",m_PointNum);
		
		for(i=0;i<m_PointNum;i++)
			fprintf(fp,"%f %f %f\n",m_Point[i][0], m_Point[i][1], m_Point[i][2]);
		
		fprintf(fp,"%d\n",m_FaceNum);

		for(i=0;i<m_FaceNum;i++)
			fprintf(fp,"%d %d %d\n",m_Face[i][0], m_Face[i][1], m_Face[i][2]);

		fclose(fp);
		return 0;
	};
};

#endif






