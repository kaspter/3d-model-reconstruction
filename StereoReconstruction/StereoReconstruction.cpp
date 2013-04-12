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

WNDPROC subclassWindowProc = NULL;
LRESULT CALLBACK presenterWindowProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	switch(message)
	{
	case WM_DESTROY:
		if (subclassWindowProc != NULL) 
			SetWindowLong(hWnd, GWL_WNDPROC, (LONG)subclassWindowProc);
		PostQuitMessage(0);
		return 0;
	}

	return subclassWindowProc != NULL 
		? subclassWindowProc(hWnd, message, wParam, lParam) 
		: DefWindowProc(hWnd, message, wParam, lParam);
}
DWORD _stdcall presenterThreadFunc(LPVOID threadParameter)
{
	std::string windowName = "Keypoint presenter window";

	cv::Mat *pmat = static_cast<cv::Mat*>(threadParameter);
	cv::imshow(windowName, *pmat);

	HWND hwnd			= static_cast<HWND>(cvGetWindowHandle(windowName.c_str()));
	subclassWindowProc	= (WNDPROC)SetWindowLong(hwnd, GWL_WNDPROC, (LONG)presenterWindowProc);
	
	// Code should be run at window creation time, which is impossible within the current situation
	//SetWindowLong(hwnd, GWL_STYLE, GetWindowLong(hwnd, GWL_STYLE) & ~WS_MAXIMIZEBOX);
	//SetWindowPos(hwnd, NULL, 0, 0, 0, 0, SWP_NOSIZE | SWP_NOMOVE | SWP_NOZORDER | SWP_FRAMECHANGED | SWP_DRAWFRAME);

	MSG msg;
	while (GetMessage(&msg, NULL, 0, 0))
	{
		TranslateMessage(&msg);
		DispatchMessage(&msg);
	}
	return msg.wParam;
}

int _tmain(int argc, _TCHAR* argv[])
{
	int retResult = 0;

	cv::CommandLineParser arguments(argc, argv, "{l| left||left channel image path string}{r|right||right channel image path string}");

	// Image set preparation step
	std::vector<ImageFileDescriptor> files;
	files.push_back(ImageFileDescriptor(arguments.get<std::string>("left")));
	files.push_back(ImageFileDescriptor(arguments.get<std::string>("right")));

	// cv::Mat alternative image set representation needed by OpenCV interface
	std::vector<cv::Mat> images;
	//std::vector<cv::Mat> output;
	cv::Mat	drawImg;

	bool terminate = false;
	for (int i = 0, imax = files.size(); i < imax; ++i)
	{
		if (terminate = files[i]->empty())
		{
			std::cerr << "Cannot locate an image at: "<< files[i].name() << "!" << std::endl;
			continue;
		}
		images.push_back(*files[i]);
		cv::cvtColor(images[i], images[i], CV_RGB2GRAY);
	}
	if (terminate) { retResult = -1; goto EXIT; }

	// Further code is divided by blocks because of using 'goto' statements 
	// Blockwise code structure prevents of the 'C2362' compiler error
	{
		// Image features recognition and description block
		if (!cv::initModule_nonfree())
		{
			std::cerr << "Failed to init Non-Free Features2d module" << std::endl; goto EXIT;
		}

		std::string algorithm_name = "SIFT";
		cv::Ptr<cv::Feature2D> algorithm = cv::Feature2D::create(algorithm_name);			

		std::vector<cv::Mat>					descriptors(files.size());
		std::vector<std::vector<cv::KeyPoint>>	keypoints(files.size());
		for (int i = 0, imax = files.size(); i < imax; ++i)
		{
			(*algorithm)(images[i], cv::Mat(), keypoints[i], descriptors[i]);
			//cv::drawKeypoints(images[i], keypoints[i], output[i], cv::Scalar(0x000000FF));
		}

		std::string matcher_name = "FlannBased";
		cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(matcher_name);

		std::vector<cv::DMatch> matches;
		matcher->match(descriptors[0],descriptors[1], matches);		
		
		std::vector<cv::Mat> points(files.size(), cv::Mat(matches.size(), 1, CV_32FC2));
		//cv::drawMatches(output[0],keypoints[0], output[1],keypoints[1], matches, drawImg);		
		for (int i=0, imax = matches.size(); i<imax; ++i) 
		{
			float* elemPtr = points[0].ptr<float>(i);
			elemPtr[0] = keypoints[0][matches[i].queryIdx].pt.x;
			elemPtr[1] = keypoints[0][matches[i].queryIdx].pt.y;

			elemPtr = points[1].ptr<float>(i);
			elemPtr[0] = keypoints[1][matches[i].trainIdx].pt.x;
			elemPtr[1] = keypoints[1][matches[i].trainIdx].pt.y;
		}
		cv::Mat fundamentalMatrix = cv::findFundamentalMat(points[0], points[1], cv::FM_RANSAC, 3, 0.99);

		std::cout<<"------------Fundamental matrix-----------------"<<std::endl;
		std::cout<<fundamentalMatrix<<std::endl;

		std::vector<cv::Mat> correspondentLines(files.size());
		cv::computeCorrespondEpilines(points[0], 1, fundamentalMatrix, correspondentLines[1]);
		cv::computeCorrespondEpilines(points[1], 2, fundamentalMatrix, correspondentLines[0]);		

		std::cout<<"------------Left Correspondent Lines-----------------"<<std::endl;
		std::cout<<correspondentLines[0]<<std::endl;
		std::cout<<"------------Right Correspondent Lines-----------------"<<std::endl;
		std::cout<<correspondentLines[1]<<std::endl;

		std::vector<cv::Mat> homography(files.size());
		cv::stereoRectifyUncalibrated(points[0], points[1], fundamentalMatrix, images[1].size(), homography[0], homography[1]);

		std::cout<<"------------Left Homography-----------------"<<std::endl;
		std::cout<<homography[0]<<std::endl;
		std::cout<<"------------Right Homography-----------------"<<std::endl;
		std::cout<<homography[1]<<std::endl;		

		float camerMatrixElements [] = {1, 0, images[0].cols/2, 
										0, 1, images[0].rows/2,
										0, 0, 1};
		cv::Mat cameraMatrix(3, 3, CV_64FC1, camerMatrixElements);
		std::vector<cv::Mat> remapData (files.size());

		cv::Mat R = cameraMatrix.inv() * homography[0] * cameraMatrix, temp( images[0].size(), images[0].type() );				
		cv::initUndistortRectifyMap(cameraMatrix, cv::Mat(1, 5, CV_32FC1, cv::Scalar::all(0)), R, cameraMatrix, images[0].size(), CV_32FC1, remapData[0], remapData[1]);			
		cv::remap(images[0], temp, remapData[0], remapData[1], cv::INTER_LINEAR);
		temp.copyTo(images[0]);

		R = cameraMatrix.inv() * homography[1] * cameraMatrix;				
		cv::initUndistortRectifyMap(cameraMatrix, cv::Mat(1, 5, CV_32FC1, cv::Scalar::all(0)), R, cameraMatrix, images[1].size(), CV_32FC1, remapData[0], remapData[1]);
		cv::remap(images[1], temp, remapData[0], remapData[1], cv::INTER_LINEAR);
		temp.copyTo(images[1]);
		temp.release();

		drawImg.create(images[0].rows, images[0].cols * 2, images[0].type());
		images[0].copyTo(drawImg(cv::Rect(0, 0, images[0].cols, images[0].rows)));
		images[1].copyTo(drawImg(cv::Rect(images[0].cols, 0, images[1].cols, images[1].rows)));
	}

	HANDLE threadPresenter = CreateThread(NULL, 0, presenterThreadFunc, &drawImg, 0x00, NULL);
	WaitForSingleObject(threadPresenter, INFINITE);
	CloseHandle(threadPresenter);

EXIT:
	std::cout << "Success! Hit any key to exit.";
	
	_gettchar();
	return retResult;
}

