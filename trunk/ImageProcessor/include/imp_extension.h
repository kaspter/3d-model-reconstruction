#pragma once 

#define IMP_BEGIN_TIMER_SECTION(tick_var)	\
	int64  tick_var = cv::getTickCount();

#if defined IMP_MACROS_EXTENDED

#if defined WIN32 || defined _WIN32 || defined WINCE
#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#define IMP_END_TIMER_SECTION(tick_var, logtext)											\
{																							\
	SIZE_T _text_amount = strlen(logtext) + 23;												\
	TCHAR* _debugstr	= new TCHAR[_text_amount];											\
	sprintf_s(_debugstr, _text_amount, "%s: %fs.\n", logtext,								\
					(cv::getTickCount() - (tick_var)) / cv::getTickFrequency());			\
	OutputDebugString(_debugstr);															\
	delete[] _debugstr;																		\
}

#else

#include <iostream>

#define IMP_END_TIMER_SECTION(tick_var, logtext)											\
{																							\
	double elapsed_time = (double(cv::getTickCount() - tick_var) / cv::getTickFrequency());	\
	std::cout << logtext << ": " << elapsed_time << "s." std::endl;							\
}

#endif

#else

#define IMP_END_TIMER_SECTION(tick_var, time_var)											\
	time_var = (double(cv::getTickCount() - tick_var) / cv::getTickFrequency());

#endif

#include <cmath>

#include "opencv2\core\core.hpp"
#include "opencv2\features2d\features2d.hpp"

namespace imp
{

	class PyramidAdapterHack : public cv::PyramidAdaptedFeatureDetector
	{
	public:
		PyramidAdapterHack( const cv::Ptr<cv::FeatureDetector>& detector, int maxLevel=2 )
			: PyramidAdaptedFeatureDetector(detector, maxLevel) { }

		using cv::PyramidAdaptedFeatureDetector::maxLevel;
		using cv::PyramidAdaptedFeatureDetector::detector;
	};

	// Produces a square matrix (2*radius + 1)x(2*radius + 1) filled by 1's in a disk form
	cv::Mat diskMask(double radius);
	// Gives a local maxima mask (map) for a grayscale image given
	size_t nonMaxSuppression3x3(const cv::Mat &src, cv::Mat &dst, bool preserveMaximaValues = false);
	// Gives graymap intensity occurence histogram for a grayscale image
	void discreteGraymapHistogram(const cv::Mat &src, cv::OutputArray dst);

	template<typename _Vt>
	inline cv::Point_<_Vt> operator/(const cv::Point_<_Vt> &pt, _Vt scalar) { return cv::Point_<_Vt>(pt.x / scalar, pt.y / scalar); }
	
	template<typename _Vt> 
	inline int sign(_Vt val) { return val != 0 ? (val > 0 ? 1 : -1) : 0; }

	template<> inline int sign<unsigned>			(unsigned			val) { return static_cast<int>(val != 0); }
	template<> inline int sign<uchar>				(uchar				val) { return static_cast<int>(val != 0); }
	template<> inline int sign<ushort>				(ushort				val) { return static_cast<int>(val != 0); }
	template<> inline int sign<unsigned long>		(unsigned long		val) { return static_cast<int>(val != 0); }
	template<> inline int sign<unsigned long long>	(unsigned long long val) { return static_cast<int>(val != 0); }

	// TODO: will be used no std::funcs in future
	inline double round (double x) { return x > 0.0  ? std::floor(x + 0.5)  : std::ceil(x - 0.5);  }
	inline float  round (float  x) { return x > 0.0F ? std::floor(x + 0.5F) : std::ceil(x - 0.5F); }

	inline bool isBitmap (const cv::Mat &mat) 
	{ 
		int depth = mat.depth();
		return mat.dims == 2 &&
			(depth == CV_8U || depth == CV_16U || depth == CV_32F);
	}
	inline bool isGraymap(const cv::Mat &mat) { return isBitmap(mat) && mat.channels() == 1; }

} // namespace imp