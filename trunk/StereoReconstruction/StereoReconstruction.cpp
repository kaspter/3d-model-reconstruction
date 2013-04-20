// StereoReconstruction.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"


class ImageFileDescriptor
{
private:
	cv::Mat		_image;
	std::string _imageFilePath;

public:
	ImageFileDescriptor(const std::string &imageFilePath, int flags = 1) 
		: _imageFilePath(imageFilePath)
	{
		_image = cv::imread(_imageFilePath, flags);
	}

	const cv::Mat		*operator->()   const { return &_image; }
	const cv::Mat		&operator*()	const { return _image; }
	const std::string	&name()			const { return _imageFilePath; }
};

std::vector<std::vector<cv::Point2f>> matchedKeypointsCoords(const std::vector<std::vector<cv::KeyPoint>> &keypoints, 
	const std::vector<cv::DMatch> &matches, std::vector<uchar> &match_status = std::vector<uchar>())
{
	CV_Assert( keypoints.size() == 2 );
	
	bool	 notFiltered	= match_status.empty() || match_status.size() != matches.size();
	unsigned pointsCount	= 0;

	std::vector<std::vector<cv::Point2f>> points = std::vector<std::vector<cv::Point2f>>(2, std::vector<cv::Point2f>(matches.size()));
	for (int i=0, imax = matches.size(); i<imax; ++i) 
	{
		if ( notFiltered || match_status[i] != 0x00 )
		{
			const cv::DMatch &match = matches[i];
			points[0][pointsCount] = cv::Point2f(keypoints[0][match.queryIdx].pt.x, keypoints[0][match.queryIdx].pt.y);
			points[1][pointsCount] = cv::Point2f(keypoints[1][match.trainIdx].pt.x, keypoints[1][match.trainIdx].pt.y);
			++pointsCount;
		}
	}

	if ( !notFiltered )
	{
		points[0].resize(pointsCount);
		points[1].resize(pointsCount);
	}

	return points;
}

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
	
	// Code should be run at window creation time, which is impossible at the current situation
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
	files.push_back(ImageFileDescriptor(arguments.get<std::string>("left"),  CV_LOAD_IMAGE_GRAYSCALE));
	files.push_back(ImageFileDescriptor(arguments.get<std::string>("right"), CV_LOAD_IMAGE_GRAYSCALE));

	// cv::Mat alternative image set representation needed by OpenCV interface
	std::vector<cv::Mat> images;
	cv::Mat				 output;

	bool terminate = false;
	for (int i = 0, imax = files.size(); i < imax; ++i)
	{
		if (terminate = files[i]->empty())
		{
			std::cerr << "Cannot locate an image at: "<< files[i].name() << "!" << std::endl;
			continue;
		}
		images.push_back(*files[i]);
	}
	if (terminate) { retResult = -1; goto EXIT; }

	// Further code is divided by blocks because of using 'goto' statements 
	// Blockwise code structure prevents of the 'C2362' compiler error
	{	// Image features recognition and description block
		// NonFree OpenCV module initialization
		if (!cv::initModule_nonfree())
		{
			std::cerr << "Failed to init Non-Free Features2d module" << std::endl; 
			goto EXIT;
		}

		cv::Size pairSize = pairSize;
		if (pairSize != pairSize)
		{
			std::cerr << "Stereopair images are not equal by size! Reconstruction is impossible.";
			goto EXIT;
		}

		// TODO: Load camera intrinsic parameters here.

		//// Step 1: Image pair feature detection
		std::string detector_name = "SIFT";
		cv::Ptr<cv::Feature2D> algorithm = cv::Feature2D::create(detector_name);			

		std::vector<cv::Mat>					_descriptors(files.size());
		std::vector<std::vector<cv::KeyPoint>>	_keypoints  (files.size());
		(*algorithm)(images[0], cv::Mat(), _keypoints[0], _descriptors[0]);
		(*algorithm)(images[1], cv::Mat(), _keypoints[1], _descriptors[1]);
			//cv::drawKeypoints(images[i], _keypoints[i], _outpair[i], cv::Scalar(0x000000FF));

		//// Step 2: Image features correspondence tracking
		std::string matcher_name = "FlannBased";
		cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(matcher_name);

		std::vector<cv::DMatch> _matches;
		matcher->match(_descriptors[0],_descriptors[1], _matches);	

			//cv::drawMatches(_outpair[0],_keypoints[0], _outpair[1],_keypoints[1], _matches, output);
			//goto SHOW;
		
		//// Step 3: Fundamental matrix estimation (with intermediate data conversion)
		std::vector<std::vector<cv::Point2f>> points = matchedKeypointsCoords(_keypoints, _matches);

		double _val_dummy, _val_max;
		cv::minMaxIdx(points[0], &_val_dummy, &_val_max);
		std::vector<uchar> _match_status(_matches.size(), 0x00);
		cv::Mat fundamentalMatrix = cv::findFundamentalMat(points[0], points[1], cv::FM_RANSAC, 0.006 * _val_max, 0.99, _match_status);
		points = matchedKeypointsCoords(_keypoints, _matches, _match_status);
		_match_status.clear();

		std::cout<<"--------------- Fundamental matrix -----------------"<<std::endl;
		std::cout<<fundamentalMatrix<<std::endl;

		//// Step 4: Camera matrices estimation up to projection transformation
		// TODO: Improve fundamental matrix up to an essential matrix

		cv::Mat E; //= K.t() * F * K; 
		cv::SVD _fm_svd(E);

		double _singular_ratio = std::abs(_fm_svd.w.at<double>(0) / _fm_svd.w.at<double>(1));
		if (_singular_ratio > 1.0) _singular_ratio = 1.0 / _singular_ratio; // flip ratio to keep it [0,1]
		if (_singular_ratio < 0.7) 
		{
			std::cout << "singular values are too far apart" << std::endl;
			retResult = -1; goto EXIT;
		}

		std::vector<cv::Mat_<double>> _R(2, cv::Mat_<double>());
		std::vector<cv::Mat_<double>> _t(2, cv::Mat_<double>());

		cv::Matx33d W(0,-1, 0,	//HZ 9.13
					  1, 0, 0,
					  0, 0, 1);

		_R[0] = _fm_svd.u * cv::Mat(W)		* _fm_svd.vt;	//HZ 9.19
		_R[1] = _fm_svd.u * cv::Mat(W.t())	* _fm_svd.vt;	//HZ 9.19
		_t[0] =  _fm_svd.u.col(2); //u3
		_t[1] = -_fm_svd.u.col(2); //u3

		//// 
			//std::vector<cv::Mat> correspondentLines(files.size());
			//cv::computeCorrespondEpilines(points[0], 1, fundamentalMatrix, correspondentLines[1]);
			//cv::computeCorrespondEpilines(points[1], 2, fundamentalMatrix, correspondentLines[0]);		

			//std::cout<<"------------Left Correspondent Lines-----------------"<<std::endl;
			//std::cout<<correspondentLines[0]<<std::endl;
			//std::cout<<"------------Right Correspondent Lines-----------------"<<std::endl;
			//std::cout<<correspondentLines[1]<<std::endl;

			//std::vector<cv::Mat> homography(files.size());
			//cv::stereoRectifyUncalibrated(points[0], points[1], fundamentalMatrix, pairSize, homography[0], homography[1]);

			//std::cout<<"------------Left Homography-----------------"<<std::endl;
			//std::cout<<homography[0]<<std::endl;
			//std::cout<<"------------Right Homography-----------------"<<std::endl;
			//std::cout<<homography[1]<<std::endl;		

			//float camerMatrixElements [] = {1, 0, float(images[0].cols/2), 
			//								0, 1, float(images[0].rows/2),
			//								0, 0, 1};
			//cv::Mat cameraMatrix(3, 3, CV_64FC1, camerMatrixElements);
			//std::vector<cv::Mat> remapData (files.size());

			//cv::Mat R = cameraMatrix.inv() * homography[0] * cameraMatrix, temp( pairSize, images[0].type() );				
			//cv::initUndistortRectifyMap(cameraMatrix, cv::Mat(1, 5, CV_32FC1, cv::Scalar::all(0)), R, cameraMatrix, pairSize, CV_32FC1, remapData[0], remapData[1]);			
			//cv::remap(images[0], temp, remapData[0], remapData[1], cv::INTER_LINEAR);
			//temp.copyTo(images[0]);

			//R = cameraMatrix.inv() * homography[1] * cameraMatrix;				
			//cv::initUndistortRectifyMap(cameraMatrix, cv::Mat(1, 5, CV_32FC1, cv::Scalar::all(0)), R, cameraMatrix, pairSize, CV_32FC1, remapData[0], remapData[1]);
			//cv::remap(images[1], temp, remapData[0], remapData[1], cv::INTER_LINEAR);
			//temp.copyTo(images[1]);
			//temp.release();

			//output.create(images[0].rows, images[0].cols * 2, images[0].type());
			//images[0].copyTo(output(cv::Rect(0, 0, images[0].cols, images[0].rows)));
			//images[1].copyTo(output(cv::Rect(images[0].cols, 0, images[1].cols, images[1].rows)));
	}

//SHOW:
	HANDLE threadPresenter = CreateThread(NULL, 0, presenterThreadFunc, &output, 0x00, NULL);
	WaitForSingleObject(threadPresenter, INFINITE);
	CloseHandle(threadPresenter);

EXIT:
	std::cout << "Hit any key to exit...";
	_gettchar();
	return retResult;
}

