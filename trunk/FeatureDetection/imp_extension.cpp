#include "stdafx.h"

#include "imp_extension.h"

namespace imp
{
	cv::Mat diskMatrix_8uc1 (double radius)
	{
		assert(radius > 0);

		unsigned r  = static_cast<unsigned>(ROUND_VAL(radius)), d = 2 * r + 1;
		double rpow = radius + 0.5; rpow *= rpow;
		
		cv::Mat res = cv::Mat(d, d, CV_8UC1);
		for(unsigned imax = d*d, i=0; i < imax ; ++i)
		{
			int x = i % d - r, y = i / d - r;
			res.at<uchar>(y + r,x + r) = (uchar)(static_cast<unsigned>(x*x + y*y) <= rpow);			
		}
		return res;
	}

	void nonMaxSupp3x3_8uc1(cv::Mat &src, cv::Mat &dst, bool preserveMaximaValues)
	{
		CV_Assert(src.type() == CV_8UC1 && src.dims == 2 && !src.empty());

		cv::Mat buf;
		uchar *buf_row  = NULL;

		bool inPlace;
		if (inPlace = src.data == dst.data)
		{
			buf = src.clone();
		}
		else
		{
			CV_Assert( dst.empty() || dst.refcount != NULL );

			buf = src;
			dst.create(src.size(), src.type());	
		}
		dst.setTo(cv::Scalar::all(0));

		cv::Mat skip(cv::Size(src.cols, 2), CV_8UC1, cv::Scalar(0));
		uchar *skip_cur = skip.ptr<uchar>(0), 
			  *skip_nxt = skip.ptr<uchar>(1); 
		
		int c, r, cmax = src.cols - 1, rmax = src.rows -1;
		for (r = 0; r < rmax; )
		{
			buf_row = buf.ptr<uchar>(++r);

			for(c = 0; c < cmax; )
			{
				if (skip_cur[++c]) continue;

				if (buf_row[c] <= buf_row[c + 1])
				{
					do { ++c; } while (c < cmax && buf_row[c] <= buf_row[c + 1]); 
					if (c == cmax) break;	
				}
				else if (buf_row[c] <= buf_row[c - 1]) continue;

				skip_cur[c + 1] = true;

				uchar *buf_row_temp = buf.ptr<uchar>(r + 1);
				if (buf_row[c] <= buf_row_temp[c - 1]) continue; /**/skip_nxt[c - 1] = true;
				if (buf_row[c] <= buf_row_temp[c	]) continue; /**/skip_nxt[c    ] = true;
				if (buf_row[c] <= buf_row_temp[c + 1]) continue; /**/skip_nxt[c + 1] = true;
				
				buf_row_temp = buf.ptr<uchar>(r - 1);
				if (buf_row[c] <= buf_row_temp[c - 1]) continue;
				if (buf_row[c] <= buf_row_temp[c    ]) continue;
				if (buf_row[c] <= buf_row_temp[c + 1]) continue;

				dst.at<uchar>(r, c) = preserveMaximaValues ? buf_row[c] : UCHAR_MAX;
			}

			swap(skip_cur, skip_nxt);
			memset(skip_nxt, 0, src.step);
		}
	}

	//void preprocess2DKernel( const cv::Mat& kernel, std::vector<cv::Point>& coords, std::vector<uchar>& coeffs )
	//{
	//	int i, j, k, nz = cv::countNonZero(kernel), ktype = kernel.type();
	//	if(nz == 0)
	//		nz = 1;
	//	CV_Assert( ktype == CV_8U || ktype == CV_32S || ktype == CV_32F || ktype == CV_64F );
	//	coords.resize(nz);
	//	coeffs.resize(nz*cv::getElemSize(ktype));
	//	uchar* _coeffs = &coeffs[0];
	//
	//	for( i = k = 0; i < kernel.rows; i++ )
	//	{
	//		const uchar* krow = kernel.data + kernel.step*i;
	//		for( j = 0; j < kernel.cols; j++ )
	//		{
	//			if( ktype == CV_8U )
	//			{
	//				uchar val = krow[j];
	//				if( val == 0 )
	//					continue;
	//				coords[k] = cv::Point(j,i);
	//				_coeffs[k++] = val;
	//			}
	//			else if( ktype == CV_32S )
	//			{
	//				int val = ((const int*)krow)[j];
	//				if( val == 0 )
	//					continue;
	//				coords[k] = cv::Point(j,i);
	//				((int*)_coeffs)[k++] = val;
	//			}
	//			else if( ktype == CV_32F )
	//			{
	//				float val = ((const float*)krow)[j];
	//				if( val == 0 )
	//					continue;
	//				coords[k] = cv::Point(j,i);
	//				((float*)_coeffs)[k++] = val;
	//			}
	//			else
	//			{
	//				double val = ((const double*)krow)[j];
	//				if( val == 0 )
	//					continue;
	//				coords[k] = cv::Point(j,i);
	//				((double*)_coeffs)[k++] = val;
	//			}
	//		}
	//	}
	//}
}