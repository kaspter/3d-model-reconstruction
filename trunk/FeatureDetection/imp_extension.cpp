#include "stdafx.h"

#include "imp_extension.h"

namespace imp
{
	// Produces a square matrix (2*radius + 1)x(2*radius + 1) filled by 1's in a disk form
	cv::Mat DiskMatrix_8uc1 (double radius)
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

	// Gives a local maxima mask (map) for an 8-bit image given
	cv::Mat NonMaxSupp3x3_8uc1(const cv::Mat &hcr)
	{
		CV_Assert(hcr.type() ==  CV_8UC1 && hcr.dims == 2);

		cv::Mat mask(hcr.size(), CV_8UC1, cv::Scalar(0));
		cv::Mat skip(cv::Size(hcr.cols, 2), CV_8UC1, cv::Scalar(0));
		 
		uchar *skip_cur = skip.data, *skip_nxt = skip_cur + skip.step; 
		uchar *hcr_row  = NULL;

		int c, r, cmax = hcr.cols - 1, rmax = hcr.rows -1;
		for (r = 0; r < rmax; )
		{
			++r; hcr_row = hcr.data + r * hcr.step;

			for(c = 0; c < cmax; )
			{
				++c;

				if (skip_cur[c]) continue;
				if (hcr_row[c] <= hcr_row[c + 1])
				{
					while (++c < cmax && hcr_row[c] <= hcr_row[c + 1]) { /* EMPTY */ }
					if (c == cmax) break;

					if (hcr_row[c] <= hcr_row[c - 1]) continue;
				}
				skip_cur[c + 1] = true;

				uchar *hcr_row_temp = hcr_row + hcr.step;
				if (hcr_row[c] <= hcr_row_temp[c - 1]) continue; /**/skip_nxt[c - 1] = true;
				if (hcr_row[c] <= hcr_row_temp[c]    ) continue; /**/skip_nxt[c    ] = true;
				if (hcr_row[c] <= hcr_row_temp[c + 1]) continue; /**/skip_nxt[c + 1] = true;
				hcr_row_temp = hcr_row - hcr.step;
				if (hcr_row[c] <= hcr_row_temp[c - 1]) continue;
				if (hcr_row[c] <= hcr_row_temp[c    ]) continue;
				if (hcr_row[c] <= hcr_row_temp[c + 1]) continue;

				mask.at<uchar>(r, c) = 0xFF;
			}

			swap(skip_cur, skip_nxt);
			memset(skip_nxt, 0, hcr.step);
		}

		return mask;
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