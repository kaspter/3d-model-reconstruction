#include "stdafx.h"

namespace imp
{
	cv::Mat DiskMatrix (unsigned int radius, int type)
	{
		unsigned int diam = 2 * radius + 1;
		cv::Mat res = cv::Mat(diam, diam, type);
		for(unsigned int iMax = diam*diam, i=0; i < iMax ; ++i)
		{
			int x = i % diam - radius;
			int y = i / diam - radius;
			res.data[i] = (x*x + y*y) <= radius*radius;
		}
		return res;
	}

	float SquareCorrelation (const cv::Mat& ker, const cv::Mat& list)
	{
		float result = 0;
		for (int i=0; i<ker.rows*ker.cols; i++)
			result += list.data[i] * ker.data[i]; 
	    return result;
	}
}