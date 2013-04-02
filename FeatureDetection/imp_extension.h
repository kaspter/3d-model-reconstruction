#pragma once 

namespace imp
{
#define ROUND_VAL(x) (double(x) > 0.0 ? floor((x) + 0.5) : ceil((x) - 0.5))

	cv::Mat DiskMatrix_8uc1 (double radius);
	cv::Mat NonMaxSupp3x3_8uc1(const cv::Mat &hcr);
	//void preprocess2DKernel( const cv::Mat& kernel, std::vector<cv::Point>& coords, std::vector<uchar>& coeffs );

	template<typename SwapType> 
	inline void swap(SwapType &a, SwapType &b) { SwapType temp = a; a = b; b = temp; }

	template<typename _vt> 
	inline int sign(_vt val)				{ return val != 0 ? (val > 0 ? 1 : -1) : 0; }
	inline int sign(unsigned val)			{ return static_cast<int>(val > 0); }
	inline int sign(uchar val)				{ return static_cast<int>(val > 0); }
	inline int sign(ushort val)				{ return static_cast<int>(val > 0); }
	inline int sign(unsigned long val)		{ return static_cast<int>(val > 0); }
	inline int sign(unsigned long long val) { return static_cast<int>(val > 0); }
} 