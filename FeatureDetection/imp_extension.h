#pragma once 
#include "opencv2\core\core.hpp"

namespace imp
{
	cv::Mat DiskMatrix (unsigned radius, int type);
	float SquareCorrelation (const cv::Mat& ker, const cv::Mat& list);	
}