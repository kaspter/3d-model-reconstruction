#pragma once 

#define ROUND_VAL(x) (double(x) > 0.0 ? floor((x) + 0.5) : ceil((x) - 0.5))

#define BEGIN_TIMER_SECTION(time_var)	\
	int64 time_var = cv::getTickCount()

#if defined WIN32 || defined _WIN32 || defined WINCE
#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#if defined WINCE
#define OutputDebugStringA OutputDebugString
#endif

#define END_TIMER_SECTION(time_var, logtext)										\
{																					\
	std::string debugstr(strlen(logtext) + 21, '\0');								\
	sprintf_s(&debugstr[0], debugstr.length(), "%s: %f\n", logtext,					\
					(cv::getTickCount() - (time_var)) / cv::getTickFrequency());	\
	OutputDebugStringA(debugstr.c_str());											\
}
#else
#include <iostream>
#define END_TIMER_SECTION(time_var, logtext)										\	
	std::cout << logtext << ": " << (cv::getTickCount() - time_var) / cv::getTickFrequency() << std::endl
#endif

#include "opencv2\core\core.hpp"
#include "opencv2\features2d\features2d.hpp"

namespace imp
{

	class PyramidAdapterHack : public cv::PyramidAdaptedFeatureDetector
	{
	public:
		using cv::PyramidAdaptedFeatureDetector::maxLevel;
		using cv::PyramidAdaptedFeatureDetector::detector;
	};

	// Produces a square matrix (2*radius + 1)x(2*radius + 1) filled by 1's in a disk form
	cv::Mat diskMask(double radius);
	// Gives a local maxima mask (map) for a grayscale image given
	void nonMaxSuppression3x3(const cv::Mat &src, cv::Mat &dst, bool preserveMaximaValues = false);
	// Gives graymap intensity occurence histogram for a grayscale image
	void discreteGraymapHistogram(const cv::Mat &src, cv::OutputArray &dst);
	
	template<typename _St> 
	inline void swap(_St &a, _St &b) { _St temp = a; a = b; b = temp; }

	template<typename _Vt> 
	inline int sign(_Vt val) { return val != 0 ? (val > 0 ? 1 : -1) : 0; }

	template<> inline int sign<unsigned>			(unsigned			val) { return static_cast<int>(val != 0); }
	template<> inline int sign<uchar>				(uchar				val) { return static_cast<int>(val != 0); }
	template<> inline int sign<ushort>				(ushort				val) { return static_cast<int>(val != 0); }
	template<> inline int sign<unsigned long>		(unsigned long		val) { return static_cast<int>(val != 0); }
	template<> inline int sign<unsigned long long>	(unsigned long long val) { return static_cast<int>(val != 0); }

	inline bool isBitmap (const cv::Mat &mat) 
	{ 
		int depth = mat.depth();
		return mat.dims == 2 &&
			(depth == CV_8U || depth == CV_16U || depth == CV_32F);
	}
	inline bool isGraymap(const cv::Mat &mat) { return isBitmap(mat) && mat.channels() == 1; }

} // namespace imp