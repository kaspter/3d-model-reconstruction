#pragma once 

namespace imp
{
#define ROUND_VAL(x) (double(x) > 0.0 ? floor((x) + 0.5) : ceil((x) - 0.5))

	// Produces a square matrix (2*radius + 1)x(2*radius + 1) filled by 1's in a disk form
	cv::Mat diskMatrix_8uc1 (double radius);
	 // Gives a local maxima mask (map) for an 8-bit image given
	void nonMaxSupp3x3_8uc1(cv::Mat &src, cv::Mat &dst, bool preserveMaximaValues = false);
	
	//void preprocess2DKernel( const cv::Mat& kernel, std::vector<cv::Point>& coords, std::vector<uchar>& coeffs );

	template<typename SwapType> 
	inline void swap(SwapType &a, SwapType &b) { SwapType temp = a; a = b; b = temp; }

	template<typename _vt> 
	inline int sign(_vt val) { return val != 0 ? (val > 0 ? 1 : -1) : 0; }

	template<> inline int sign<unsigned>			(unsigned			val) { return static_cast<int>(val > 0); }
	template<> inline int sign<uchar>				(uchar				val) { return static_cast<int>(val > 0); }
	template<> inline int sign<ushort>				(ushort				val) { return static_cast<int>(val > 0); }
	template<> inline int sign<unsigned long>		(unsigned long		val) { return static_cast<int>(val > 0); }
	template<> inline int sign<unsigned long long>	(unsigned long long val) { return static_cast<int>(val > 0); }
} 