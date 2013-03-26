#pragma once 

namespace imp
{
#define ROUND_VAL(x) (double(x) > 0.0 ? floor((x) + 0.5) : ceil((x) - 0.5))

	cv::Mat DiskMatrix_8uc1 (double radius);
	void preprocess2DKernel( const cv::Mat& kernel, std::vector<cv::Point>& coords, std::vector<uchar>& coeffs );

	template<typename ST, typename DT> struct Cast
	{
		typedef ST type1;
		typedef DT rtype;

		DT operator()(ST val) const { return cv::saturate_cast<DT>(val); }
	};
}