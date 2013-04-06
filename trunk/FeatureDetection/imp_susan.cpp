
#include "stdafx.h"
#include "imp_susan.h"

namespace imp 
{
	// Image preprocessing section
	cv::Ptr<cv::BaseFilter> getSusanImageFilter(int srcType, int dstType, unsigned radius, double sigma, double t)
	{
		int sdepth = CV_MAT_DEPTH(srcType), ddepth = CV_MAT_DEPTH(dstType);

		CV_Assert( CV_MAT_CN(srcType) == CV_MAT_CN(dstType) && sdepth >= ddepth );

		if( sdepth == CV_8U && ddepth == CV_8U )
			return cv::Ptr<cv::BaseFilter>(
				new SUSANImageFilter<uchar, float, uchar>(radius, sigma, t));
		if( sdepth == CV_8U && ddepth == CV_16U )
			return cv::Ptr<cv::BaseFilter>(
				new SUSANImageFilter<uchar, float, ushort>(radius, sigma, t));
		if( sdepth == CV_8U && ddepth == CV_16S )
			return cv::Ptr<cv::BaseFilter>(
				new SUSANImageFilter<uchar, float, short>(radius, sigma, t));
		if( sdepth == CV_8U && ddepth == CV_32F )
			return cv::Ptr<cv::BaseFilter>(
				new SUSANImageFilter<uchar, float, float>(radius, sigma, t));
		if( sdepth == CV_8U && ddepth == CV_64F )
			return cv::Ptr<cv::BaseFilter>(
				new SUSANImageFilter<uchar, double, double>(radius, sigma, t));

		if( sdepth == CV_16U && ddepth == CV_16U )
			return cv::Ptr<cv::BaseFilter>(
				new SUSANImageFilter<ushort, float, ushort>(radius, sigma, t));
		if( sdepth == CV_16U && ddepth == CV_32F )
			return cv::Ptr<cv::BaseFilter>(
				new SUSANImageFilter<ushort, float, float>(radius, sigma, t));
		if( sdepth == CV_16U && ddepth == CV_64F )
			return cv::Ptr<cv::BaseFilter>(
				new SUSANImageFilter<ushort, double, double>(radius, sigma, t));

		if( sdepth == CV_16S && ddepth == CV_16S )
			return cv::Ptr<cv::BaseFilter>(
				new SUSANImageFilter<short, float, short>(radius, sigma, t));
		if( sdepth == CV_16S && ddepth == CV_32F )
			return cv::Ptr<cv::BaseFilter>(
				new SUSANImageFilter<short, float, float>(radius, sigma, t));
		if( sdepth == CV_16S && ddepth == CV_64F )
			return cv::Ptr<cv::BaseFilter>(
				new SUSANImageFilter<short, double, double>(radius, sigma, t));

		if( sdepth == CV_32F && ddepth == CV_32F )
			return cv::Ptr<cv::BaseFilter>(
				new SUSANImageFilter<float, float, float>(radius, sigma, t));
		if( sdepth == CV_64F && ddepth == CV_64F )
			return cv::Ptr<cv::BaseFilter>(
				new SUSANImageFilter<double, double, double>(radius, sigma, t));

		CV_Error_( CV_StsNotImplemented, 
			("Unsupported combination of source format (=%d), and destination format (=%d)", srcType, dstType) );

		return cv::Ptr<cv::BaseFilter>(NULL);
	}

	cv::Ptr<cv::FilterEngine> createSusanImageFilter(int srcType, int dstType, unsigned radius, double sigma, double t, 
		int rowBorderType, int columnBorderType, const cv::Scalar &borderValue)
	{
		srcType = CV_MAT_TYPE(srcType); dstType = CV_MAT_TYPE(dstType);
		
		cv::Ptr<cv::BaseFilter> _filter2D = getSusanImageFilter(srcType, dstType, radius, sigma, t);
		return cv::Ptr<cv::FilterEngine>(new cv::FilterEngine(
			_filter2D, cv::Ptr<cv::BaseRowFilter>(0), cv::Ptr<cv::BaseColumnFilter>(0), 
			srcType, dstType, srcType, rowBorderType, columnBorderType, borderValue)
			);
	}

	void SusanImagePrepare(const cv::Mat &src, cv::Mat &dst, int radius, double sigma, double t, int borderType)
	{
		CV_Assert( src.dims == 2 );
		if ( dst.dims != 2 || src.depth() > dst.depth() || src.channels() != dst.channels() || src.size() != dst.size() )
		{
			CV_Assert( dst.empty() || dst.refcount != NULL );
			dst.create(src.size(), src.type());
		}

		cv::Ptr<cv::FilterEngine> filter = createSusanImageFilter(src.type(), dst.type(), radius, sigma, t, borderType); 
		filter->apply(src, dst);
	}

	// Feature dtection section
	cv::Ptr<cv::BaseFilter> getSusanFeatureResponse(int srcType, int dstType, unsigned radius, double t, double g)
	{
		int cn = CV_MAT_CN(srcType), sdepth = CV_MAT_DEPTH(srcType), ddepth = CV_MAT_DEPTH(dstType);

		CV_Assert( cn == 1 && cn == CV_MAT_CN(dstType) && sdepth >= ddepth );

		if( sdepth == CV_8U && ddepth == CV_8U )
			return cv::Ptr<cv::BaseFilter>(
				new SUSANFeatureResponse<uchar, float, uchar>(radius, t, g));
		if( sdepth == CV_8U && ddepth == CV_16U )
			return cv::Ptr<cv::BaseFilter>(
				new SUSANFeatureResponse<uchar, float, ushort>(radius, t, g));
		if( sdepth == CV_8U && ddepth == CV_16S )
			return cv::Ptr<cv::BaseFilter>(
				new SUSANFeatureResponse<uchar, float, short>(radius, t, g));
		if( sdepth == CV_8U && ddepth == CV_32F )
			return cv::Ptr<cv::BaseFilter>(
				new SUSANFeatureResponse<uchar, float, float>(radius, t, g));
		if( sdepth == CV_8U && ddepth == CV_64F )
			return cv::Ptr<cv::BaseFilter>(
				new SUSANFeatureResponse<uchar, double, double>(radius, t, g));

		if( sdepth == CV_16U && ddepth == CV_16U )
			return cv::Ptr<cv::BaseFilter>(
				new SUSANFeatureResponse<ushort, float, ushort>(radius, t, g));
		if( sdepth == CV_16U && ddepth == CV_32F )
			return cv::Ptr<cv::BaseFilter>(
				new SUSANFeatureResponse<ushort, float, float>(radius, t, g));
		if( sdepth == CV_16U && ddepth == CV_64F )
			return cv::Ptr<cv::BaseFilter>(
				new SUSANFeatureResponse<ushort, double, double>(radius, t, g));

		if( sdepth == CV_16S && ddepth == CV_16S )
			return cv::Ptr<cv::BaseFilter>(
				new SUSANFeatureResponse<short, float, short>(radius, t, g));
		if( sdepth == CV_16S && ddepth == CV_32F )
			return cv::Ptr<cv::BaseFilter>(
				new SUSANFeatureResponse<short, float, float>(radius, t, g));
		if( sdepth == CV_16S && ddepth == CV_64F )
			return cv::Ptr<cv::BaseFilter>(
				new SUSANFeatureResponse<short, double, double>(radius, t, g));

		if( sdepth == CV_32F && ddepth == CV_32F )
			return cv::Ptr<cv::BaseFilter>(
				new SUSANFeatureResponse<float, float, float>(radius, t, g));
		if( sdepth == CV_64F && ddepth == CV_64F )
			return cv::Ptr<cv::BaseFilter>(
				new SUSANFeatureResponse<double, double, double>(radius, t, g));

		CV_Error_( CV_StsNotImplemented, 
			("Unsupported combination of source format (=%d), and destination format (=%d)", srcType, dstType) );

		return cv::Ptr<cv::BaseFilter>(NULL);
	}

	cv::Ptr<cv::FilterEngine> createSusanFeatureResponse(int srcType, int dstType, unsigned radius, double sigma, double t, 
		int rowBorderType, int columnBorderType, const cv::Scalar &borderValue)
	{
		srcType = CV_MAT_TYPE(srcType); dstType = CV_MAT_TYPE(dstType);
		
		cv::Ptr<cv::BaseFilter> _filter2D = getSusanFeatureResponse(srcType, dstType, radius, sigma, t);
		return cv::Ptr<cv::FilterEngine>(new cv::FilterEngine(
			_filter2D, cv::Ptr<cv::BaseRowFilter>(0), cv::Ptr<cv::BaseColumnFilter>(0), 
			srcType, dstType, srcType, rowBorderType, columnBorderType, borderValue)
			);
	}

	void SusanFeatureResponse(const cv::Mat &src, cv::Mat &dst, int radius, double t, double g, int borderType)
	{
		CV_Assert( src.channels() == 1 && src.dims == 2 && src.depth() == CV_8U );
		if ( dst.dims != 2 || src.channels() < dst.channels() || src.size() != dst.size() )
		{
			CV_Assert( dst.empty() || dst.refcount != NULL );
			dst.create(src.size(), src.type());
		}
			
		cv::Ptr<cv::FilterEngine> filter = createSusanFeatureResponse(src.type(), dst.type(), radius, t, g, borderType); 
		filter->apply(src, dst);
	}
}