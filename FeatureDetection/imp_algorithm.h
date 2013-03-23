namespace imp
{
//-------------------BASE-----------------------------
	class IImageFilter
	{
	public:
		virtual void Filtering(cv::Mat image) = 0;
		virtual ~IImageFilter() = 0;
	};


	class IFeatureDetector
	{
	public:
		virtual void Detection() = 0;
		virtual ~IFeatureDetector() = 0;
	};

//-------------------SUSAN-----------------------------
	class SUSANImageFilter : public IImageFilter
	{
	public:
		virtual void Filtering(cv::Mat image);
	};

	class SUSANFeatureDetector : public IFeatureDetector
	{};

}