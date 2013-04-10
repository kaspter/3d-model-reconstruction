// StereoReconstruction.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"


int _tmain(int argc, _TCHAR* argv[])
{
	int retResult = 0;

	cv::CommandLineParser arguments(argc, argv, "{l| left||left channel image path string}{r|right||right channel image path string}");

	cv::Mat imgLeft  = cv::imread(arguments.get<std::string>("left")),
			imgRight = cv::imread(arguments.get<std::string>("right"));

	if (imgLeft.data == NULL || imgRight.data == NULL)
	{
		std::cerr << "cannot locate image(s) specofied!";
		retResult = -1;
		goto EXIT;
	}



EXIT:
	_gettchar();
	return retResult;
}

