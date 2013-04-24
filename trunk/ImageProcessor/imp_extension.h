#pragma once 

#define ROUND_VAL(x) (double(x) > 0.0 ? floor((x) + 0.5) : ceil((x) - 0.5))

namespace imp
{
	// Produces a square matrix (2*radius + 1)x(2*radius + 1) filled by 1's in a disk form
	cv::Mat diskMask(double radius);
	// Gives a local maxima mask (map) for a grayscale image given
	void nonMaxSuppression3x3(cv::Mat &src, cv::Mat &dst, bool preserveMaximaValues = false);
	// Gives graymap intensity occurence histogram for a grayscale image
	void discreteGraymapHistogram(cv::Mat &src, cv::OutputArray &dst);
	
	template<typename SwapType> 
	inline void swap(SwapType &a, SwapType &b) { SwapType temp = a; a = b; b = temp; }

	template<typename _vt> 
	inline int sign(_vt val) { return val != 0 ? (val > 0 ? 1 : -1) : 0; }

	template<> inline int sign<unsigned>			(unsigned			val) { return static_cast<int>(val > 0); }
	template<> inline int sign<uchar>				(uchar				val) { return static_cast<int>(val > 0); }
	template<> inline int sign<ushort>				(ushort				val) { return static_cast<int>(val > 0); }
	template<> inline int sign<unsigned long>		(unsigned long		val) { return static_cast<int>(val > 0); }
	template<> inline int sign<unsigned long long>	(unsigned long long val) { return static_cast<int>(val > 0); }

	inline bool isBitmap (const cv::Mat &mat) 
	{ 
		int depth = mat.depth();
		return mat.dims == 2 &&
			(depth == CV_8U || depth == CV_16U || depth == CV_32F);
	}
	inline bool isGraymap(const cv::Mat &mat) { return isBitmap(mat) && mat.channels() == 1; }
} 