#include "stdafx.h"

namespace imp
{
	cv::Mat DiskMatrix_8uc1 (unsigned radius)
	{
		unsigned diam = 2 * radius + 1;
		cv::Mat res = cv::Mat(diam, diam, CV_8UC1);
		for(unsigned iMax = diam*diam, i=0; i < iMax ; ++i)
		{
			int x = i % diam - radius;
			int y = i / diam - radius;
			res.at<uchar>(x,y) = static_cast<unsigned>(x*x + y*y) <= radius*radius;			
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

	void preprocess2DKernel( const cv::Mat& kernel, std::vector<cv::Point>& coords, std::vector<uchar>& coeffs )
	{
		int i, j, k, nz = cv::countNonZero(kernel), ktype = kernel.type();
		if(nz == 0)
			nz = 1;
		CV_Assert( ktype == CV_8U || ktype == CV_32S || ktype == CV_32F || ktype == CV_64F );
		coords.resize(nz);
		coeffs.resize(nz*cv::getElemSize(ktype));
		uchar* _coeffs = &coeffs[0];

		for( i = k = 0; i < kernel.rows; i++ )
		{
			const uchar* krow = kernel.data + kernel.step*i;
			for( j = 0; j < kernel.cols; j++ )
			{
				if( ktype == CV_8U )
				{
					uchar val = krow[j];
					if( val == 0 )
						continue;
					coords[k] = cv::Point(j,i);
					_coeffs[k++] = val;
				}
				else if( ktype == CV_32S )
				{
					int val = ((const int*)krow)[j];
					if( val == 0 )
						continue;
					coords[k] = cv::Point(j,i);
					((int*)_coeffs)[k++] = val;
				}
				else if( ktype == CV_32F )
				{
					float val = ((const float*)krow)[j];
					if( val == 0 )
						continue;
					coords[k] = cv::Point(j,i);
					((float*)_coeffs)[k++] = val;
				}
				else
				{
					double val = ((const double*)krow)[j];
					if( val == 0 )
						continue;
					coords[k] = cv::Point(j,i);
					((double*)_coeffs)[k++] = val;
				}
			}
		}
	}
}