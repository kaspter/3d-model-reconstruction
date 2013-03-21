namespace imp
{
	class IImageFilter
	{
	public:
		virtual void Filtering() = 0;
		virtual ~IImageFilter() = 0;
	};


	class IFeatureDetector
	{
	public:
		virtual void Detection() = 0;
		virtual ~IFeatureDetector() = 0;
	};
}