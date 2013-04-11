// StereoReconstruction.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"


class ImageFileDescriptor
{
private:
	cv::Mat		_image;
	std::string _imageFilePath;

public:
	ImageFileDescriptor(const std::string &imageFilePath) 
		: _imageFilePath(imageFilePath)
	{
		_image = cv::imread(_imageFilePath);
	}

	const cv::Mat		*operator->()   const { return &_image; }
	const cv::Mat		&operator*()	const { return _image; }
	const std::string	&name()			const { return _imageFilePath; }
};

int _tmain(int argc, _TCHAR* argv[])
{
	int retResult = 0;

	cv::CommandLineParser arguments(argc, argv, "{l| left||left channel image path string}{r|right||right channel image path string}");

	// Image set preparation step
	std::vector<ImageFileDescriptor> descriptors;
	descriptors.push_back(ImageFileDescriptor(arguments.get<std::string>("left")));
	descriptors.push_back(ImageFileDescriptor(arguments.get<std::string>("right")));

	// cv::Mat alternative image set representation needed by OpenCV interface
	std::vector<cv::Mat> images;

	bool terminate = false;
	for (auto i = descriptors.begin(); i != descriptors.end(); ++i)
	{
		if (terminate = (*i)->empty())
		{
			std::cerr << "Cannot locate an image at: "<< i->name() << "!" << std::endl;
			continue;
		}
		images.push_back(**i);
	}
	if (terminate) { retResult = -1; goto EXIT; }

	// Further code is divided by blocks because of using 'goto' statements 
	// Blockwise code structure prevents of the 'C2362' compiler error
	{ // Image features recognition and description block
		if (!cv::initModule_nonfree())
		{
			std::cerr << "Failed to init Non-Free Features2d module" << std::endl; goto EXIT;
		}

		std::string detector = "SIFT";
		cv::Ptr<cv::FeatureDetector> features = cv::FeatureDetector::create(detector);

		std::vector<std::vector<cv::KeyPoint>> keypoints(images.size());
		features->detect(images, keypoints);

	}

EXIT:
	std::cout << "Success! Hit any key to exit.";
	
	_gettchar();
	return retResult;
}

