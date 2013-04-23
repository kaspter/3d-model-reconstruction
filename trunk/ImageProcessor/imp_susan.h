#pragma once

namespace imp
{
	cv::Ptr<cv::BaseFilter> getSusanImageFilter(int srcType, int dstType, unsigned radius, double sigma, double t);
	cv::Ptr<cv::FilterEngine> createSusanImageFilter(int srcType, int dstType, unsigned radius, double sigma, double t, 
		int rowBorderType = cv::BORDER_DEFAULT, int columnBorderType = -1, const cv::Scalar &borderValue = cv::Scalar());
	void filterSusan(const cv::Mat &src, cv::Mat &dst, int radius, double sigma, double t, int borderType = cv::BORDER_DEFAULT);

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
		SUSAN ( unsigned radius = 3, double t = 27.0, double g = -1.0, bool useOwnFilter = false );
		cv::AlgorithmInfo* info() const;
	};
}