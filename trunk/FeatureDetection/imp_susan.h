#pragma once

namespace imp
{
	//class RadialCachedFilterBase : cv::BaseFilter
	//{
	//protected:
	//	// Filter direct parameters
	//	unsigned paramRadius;

	//	// Internal intermediate data structures
	//	std::vector<cv::Point>	_pt;		// Kernel top-left based relative coordinates.
	//	std::vector<uchar*>		_psrc;		// Source image pixel data element pointer set.
	//	std::vector<uchar>		_kval;		// Kernel values. Dynamic (as much elements as _pt.size() * channel_count).
	//	cv::Mat					_ctable;	// Filter exponent cache table. Is valuable for 8-bit integer SourceValueType only

	//public:
	//	inline RadialCachedFilterBase()
	//};

	// =================================== SUSAN =====================================	
	template<typename SourceValueType, typename KernelValueType, typename ResultValueType> 
	class SUSANImageFilter : public cv::BaseFilter
	{
		// Filter direct parameters
		unsigned paramRadius;
		double	 paramSigma;
		double	 paramT;

		// Internal intermediate data structures
		std::vector<cv::Point>		 _pt;		// Kernel top-left based relative coordinates.
		std::vector<uchar*>			 _psrc;		// Source image pixel data element pointer set.
		std::vector<KernelValueType> _kval;		// Kernel values. Dynamic (as much elements as _pt.size() * channel_count).
		cv::Mat						 _ctable;	// Filter exponent cache table. Is valuable for 8-bit integer SourceValueType only
		double						 _2sigma;	// 2nd power of paramSigma twice

		double _fexp_u8(const cv::Point &at, SourceValueType nucleus, SourceValueType value);
		double _fexp_xx(const cv::Point &at, SourceValueType nucleus, SourceValueType value);
				
	public:
		inline SUSANImageFilter (unsigned radius, double sigma, double t)
			: paramRadius(0), paramSigma(1.0), paramT(1.0)	{ init(radius, sigma, t); }

		void init(unsigned radius, double sigma = -1.0, double t = -1.0);
		void operator()(const uchar** src, uchar* dst, int dststep, int dstcount, int width, int cn);

		inline void reset() { _kval.clear(); }
	};
	cv::Ptr<cv::BaseFilter> getSusanImageFilter(int srcType, int dstType, unsigned radius, double sigma, double t);
	cv::Ptr<cv::FilterEngine> createSusanImageFilter(int srcType, int dstType, unsigned radius, double sigma, double t, 
		int rowBorderType = cv::BORDER_DEFAULT, int columnBorderType = -1, const cv::Scalar &borderValue = cv::Scalar());
	void filterSusan(const cv::Mat &src, cv::Mat &dst, int radius, double sigma, double t, int borderType = cv::BORDER_DEFAULT);

	template<typename SourceValueType, typename KernelValueType, typename ResultValueType = uchar> 
	class SUSANFeatureResponse : public cv::BaseFilter
	{
		typedef cv::Point_<KernelValueType> vec2d;

		// Filter direct parameters
		unsigned paramRadius;		
		double	 paramG;
		double	 paramT;		

		// Internal intermediate data structures
		std::vector<cv::Point>		 _pt;		// Kernel top-left based relative coordinates.
		std::vector<uchar*>			 _psrc;		// Source image pixel data element pointer set.
		std::vector<KernelValueType> _kval;		// Kernel values. Dynamic (as much elements as _pt.size() * channel_count).
		std::vector<double>			 _ctable;	// Filter exponent cache table. Is valuable for 8-bit integer SourceValueType only
		double						 _dist;		// Minimal distance the usan's centroid must be located at.

		double _fexp_u8(SourceValueType nucleus, SourceValueType value);
		double _fexp_xx(SourceValueType nucleus, SourceValueType value);
				
	public:
		inline SUSANFeatureResponse (unsigned radius, double t, double g)
			: paramRadius(0), paramG(1.0), paramT(1.0) { init(radius, t, g); }

		void init(unsigned radius, double t = -1.0, double g = -1.0);
		void operator()(const uchar** src, uchar* dst, int dststep, int dstcount, int width, int cn);

		inline void reset() { _kval.clear(); }
	};

	cv::Ptr<cv::BaseFilter> getSusanFeatureResponse(int srcType, int dstType, unsigned radius, double t, double g);
	cv::Ptr<cv::FilterEngine> createSusanFeatureResponse(int srcType, int dstType, unsigned radius, double t, double g, 
		int rowBorderType = cv::BORDER_DEFAULT, int columnBorderType = -1, const cv::Scalar &borderValue = cv::Scalar());
	void cornerSusan(const cv::Mat &src, cv::Mat &dst, int radius, double t, double g, int borderType = cv::BORDER_DEFAULT);

	class SUSAN : public cv::FeatureDetector
	{
		void detectImpl( const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, const cv::Mat& mask=cv::Mat() ) const;

		unsigned paramRadius;		
		double	 paramG;
		double	 paramT;

		bool	 preFilter;

	public:
		SUSAN ( unsigned radius = 3, double t = 64.0, double g = 18.5, bool useOwnFilter = false );
		cv::AlgorithmInfo* info() const;
	};
}

#include "imp_susan_template.cpp"