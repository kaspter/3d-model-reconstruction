
#include "imp_susan.h"
#include "imp_susan_template.cpp"

namespace imp 
{
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	cv::Ptr<cv::BaseFilter> getSusanImageFilter(int srcType, int dstType, unsigned radius, double sigma, double t)
	{
		int sdepth = CV_MAT_DEPTH(srcType), ddepth = CV_MAT_DEPTH(dstType);

		CV_Assert( CV_MAT_CN(srcType) == CV_MAT_CN(dstType) && sdepth <= ddepth );

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

	void filterSusan(const cv::Mat &src, cv::Mat &dst, int radius, double sigma, double t, int borderType)
	{
		CV_Assert( isBitmap(src) );
		if ( dst.dims != 2 || src.depth() > dst.depth() || src.channels() != dst.channels() || src.size() != dst.size() )
		{
			CV_Assert( dst.empty() || dst.refcount != NULL );
			dst.create(src.size(), src.type());
		}

		cv::Ptr<cv::FilterEngine> filter = createSusanImageFilter(src.type(), dst.type(), radius, sigma, t, borderType); 
		filter->apply(src, dst);
	}


	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	cv::Ptr<cv::BaseFilter> getSusanFeatureResponse(int srcType, int dstType, unsigned radius, double t, double g)
	{
		int cn = CV_MAT_CN(srcType), sdepth = CV_MAT_DEPTH(srcType), ddepth = CV_MAT_DEPTH(dstType);

		CV_Assert( cn == 1 && cn == CV_MAT_CN(dstType) && sdepth <= ddepth );

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

	void cornerSusan(const cv::Mat &src, cv::Mat &dst, int radius, double t, double g, int borderType)
	{
		CV_Assert( isGraymap(src) );
		if ( dst.dims != 2 || src.channels() < dst.channels() || src.size() != dst.size() )
		{
			CV_Assert( dst.empty() || dst.refcount != NULL );
			dst.create(src.size(), src.type());
		}
			
		cv::Ptr<cv::FilterEngine> filter = createSusanFeatureResponse(src.type(), dst.type(), radius, t, g, borderType); 
		filter->apply(src, dst);
	}


	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	inline void SUSAN::set_radius(unsigned r) {	_radius = std::max(1U, r);	}
	inline void SUSAN::set_tparam(double   t) { _tparam = std::max(t, 1.0); }
	inline void SUSAN::set_gparam(double   g) { _gparam = g == -1.0 ? g : std::max(g, 0.0); }

	inline int SUSAN::reset_pass_counter() const { int rvalue = _pass_counter; _pass_counter = -1; return rvalue; }

	SUSAN::SUSAN (unsigned radius, double t, double g, bool prefilter, bool subpixel)
		: _prefilter(prefilter), _subpixel(subpixel)
	{
		set_radius(radius);
		set_tparam(t);
		set_gparam(g);

		reset_pass_counter();
	}


	float _gradientAngle3x3(const cv::Mat& image, int row, int col)
	{
		uchar w[3][3] = { 0x00 };
		for (int r = 0; r < 3; ++r)
		{
			int i = std::abs(row + r - 1); if (i >= image.rows) i -= image.rows;
			for (int c = 0; c < 3; ++c)
			{
				int j = std::abs(col + c - 1); if (j >= image.cols) j -= image.cols;
				w[r][c] = image.at<uchar>(i, j);
			}
		}

		int G[] = { // Scharr kernel convolution expanded
			3 * (w[2][0] - w[2][2] + w[0][0] - w[0][2]) + 10 * (w[1][0] - w[1][2]),
			3 * (w[2][0] + w[2][2] - w[0][0] - w[0][2]) + 10 * (w[0][1] - w[2][1])
		};

		return (G[0] == 0 && G[1] == 0) ? std::numeric_limits<float>::quiet_NaN()
			: static_cast<float>(180.0 * std::atan2(double(G[1]), double(G[0])) / CV_PI + (G[1] > 0 ? 0.0 : 360.0));		
	}
	void SUSAN::detectImpl ( const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, const cv::Mat& mask ) const
	{	
		CV_Assert(isBitmap(image));
		
		keypoints.clear();
		if (image.channels() > 3) return;

		cv::Mat src;
		if ( image.channels() == 3 )
			cv::cvtColor(image, src, cv::COLOR_BGR2GRAY); 
		else image.copyTo(src);
		
		if (_prefilter) filterSusan(src, src, _radius, _radius / 3.0, _tparam * 2);
		
		cv::Mat dst(src.size(), CV_32FC1);
		cornerSusan(src, dst, _radius, _tparam, _gparam);

		size_t corner_count = nonMaxSuppression3x3(dst, dst);
		if (corner_count == 0) return;
		
		++_pass_counter;
		
		cv::Mat corners(4, corner_count, CV_32FC1);
		cv::Point2f *corner		= reinterpret_cast<cv::Point2f*>(corners.ptr<float>(0));
		cv::Vec2f	*response	= reinterpret_cast<cv::Vec2f*>(corners.ptr<float>(2));

		corner_count = 0;
		for (int r=0; r<dst.rows; ++r)
		{
			float* row = dst.ptr<float>(r);
			for (int c=0; c<dst.cols; ++c)
			{
				if (row[c] > 0.0F)
				{
					new (corner + corner_count) cv::Point2f(
						static_cast<float>(c), 
						static_cast<float>(r)
					);
					new (response + corner_count) cv::Vec2f(
						 _gradientAngle3x3(src, r, c), row[c]);
					++corner_count;
				}

			}
		}
		dst.release();

		int ksize = 2 * _radius + 1;
		if (_subpixel) 
		{
			int win_radius = ksize + 4;
			win_radius = (src.rows < win_radius || src.cols < win_radius) 
				? std::max((std::min(src.rows, src.cols) - 5) / 2, 0) : _radius;

			if (!!win_radius)
			{
				cv::Point win_size(win_radius, win_radius);
				cv::Mat points(corner_count, 1, CV_32FC2, corners.data);
				cv::cornerSubPix(src, points, win_size, win_size / 3, cv::TermCriteria(
					cv::TermCriteria::COUNT + cv::TermCriteria::EPS, _radius * 10, std::numeric_limits<float>::epsilon()
					));
			}
		}

		keypoints.resize(corner_count);
		cv::KeyPoint* keypoint = &keypoints.front();
		for (size_t i = 0; i < corner_count; ++i)
		{
			new (keypoint + i) cv::KeyPoint(
				corner[i], static_cast<float>(ksize), response[i].val[0], response[i].val[1], _pass_counter, -1);
		}

		cv::KeyPointsFilter::runByPixelsMask(keypoints, mask);
	}

} // namespace imp