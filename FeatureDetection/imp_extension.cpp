#include "stdafx.h"

namespace imp
{
	cv::Mat DiskMatrix_8uc1 (double radius)
	{
		assert(radius > 0);

		unsigned r  = static_cast<unsigned>(ROUND_VAL(radius)), d = 2 * r + 1;
		double rpow = radius + 0.5; rpow *= rpow;
		
		cv::Mat res = cv::Mat(d, d, CV_8UC1);
		for(unsigned imax = d*d, i=0; i < imax ; ++i)
		{
			int x = i % d - r, y = i / d - r;
			res.at<uchar>(x + r,y + r) = static_cast<unsigned>(x*x + y*y) <= rpow;			
		}
		return res;
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