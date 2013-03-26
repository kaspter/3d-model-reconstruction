#pragma once 
#include "opencv2\core\core.hpp"

namespace imp
{
	cv::Mat DiskMatrix_8uc1 (unsigned radius);
	float SquareCorrelation (const cv::Mat& ker, const cv::Mat& list);	
	void preprocess2DKernel( const cv::Mat& kernel, std::vector<cv::Point>& coords, std::vector<uchar>& coeffs );
}