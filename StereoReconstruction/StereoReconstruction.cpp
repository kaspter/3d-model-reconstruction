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

template <class _pointT>
std::vector<std::vector<cv::Point_<_pointT>>> matchedKeypointsCoords(const std::vector<std::vector<cv::KeyPoint>> &keypoints, 
	const std::vector<cv::DMatch> &matches, std::vector<uchar> &match_status = std::vector<uchar>())
{
	CV_Assert( keypoints.size() == 2 );
	
	bool	 notFiltered	= match_status.empty() || match_status.size() != matches.size();
	unsigned pointsCount	= 0;

	std::vector<std::vector<cv::Point_<_pointT>>> points = std::vector<std::vector<cv::Point_<_pointT>>>(2, std::vector<cv::Point_<_pointT>>(matches.size()));
	for (int i=0, imax = matches.size(); i<imax; ++i) 
	{
		if ( notFiltered || match_status[i] != 0x00 )
		{
			const cv::DMatch &match = matches[i];
			points[0][pointsCount] = cv::Point_<_pointT>(keypoints[0][match.queryIdx].pt.x, keypoints[0][match.queryIdx].pt.y);
			points[1][pointsCount] = cv::Point_<_pointT>(keypoints[1][match.trainIdx].pt.x, keypoints[1][match.trainIdx].pt.y);
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

bool HZEssentialDecomposition(const cv::Mat &E, cv::Mat &R1, cv::Mat &R2, cv::Mat t1, cv::Mat t2)
{
	cv::SVD _fm_svd(E);

	double _singular_ratio = std::abs(_fm_svd.w.at<double>(0) / _fm_svd.w.at<double>(1));
	if (_singular_ratio > 1.0) _singular_ratio = 1.0 / _singular_ratio; // flip ratio to keep it [0,1]
	if (_singular_ratio < 0.7) return false;

	cv::Matx33d W(0,-1, 0,	//HZ 9.13
				  1, 0, 0,
				  0, 0, 1);

	R1 = _fm_svd.u * cv::Mat(W)		* _fm_svd.vt;	//HZ 9.19
	R2 = _fm_svd.u * cv::Mat(W.t())	* _fm_svd.vt;	//HZ 9.19
	
	t1 =  _fm_svd.u.col(2); //u3
	t2 = -t1;				//u3

	return true;
}

inline bool isCoherent(const cv::Mat &R)
{
	return std::abs(cv::determinant(R)) - 1 < 1e-07;
}

int _tmain(int argc, _TCHAR* argv[])
{
	int retResult = 0;

	cv::CommandLineParser arguments(argc, argv, "{l|left||}{r|right||}{1|||}");

	std::vector<ImageFileDescriptor> files;
	std::vector<cv::Mat>			 images;
	cv::Mat intrinsics, distortion, output;

	// Reading calibration data
	std::string		storageName;
	cv::FileStorage storage(storageName = arguments.get<std::string>("1"), cv::FileStorage::READ | cv::FileStorage::FORMAT_XML, "utf8");

	if (!storage.isOpened()) 
	{
		std::cerr << "Cannot read calibration data: file '" << storageName << "' does not exist." << std::endl;
		retResult = -1; goto EXIT;
	}

	storage["camera_intrinsics"] >> intrinsics;
	storage["camera_distortion"] >> distortion;

	if (intrinsics.empty() || intrinsics.size() != cv::Size(3,3) || intrinsics.type() != CV_64FC1
		|| distortion.empty() || distortion.size () != cv::Size(5,1) || distortion.type() != CV_64FC1)
	{
		std::cerr << "Cannot read calibration data: invalid contents of file '" << storageName << "'!" << std::endl;
		retResult = -1; goto EXIT;
	}

	// Image set preparation step
	files.push_back(ImageFileDescriptor(arguments.get<std::string>("left"),  CV_LOAD_IMAGE_GRAYSCALE));
	files.push_back(ImageFileDescriptor(arguments.get<std::string>("right"), CV_LOAD_IMAGE_GRAYSCALE));

	bool terminate = false;
	for (int i = 0, imax = files.size(); i < imax; ++i)
	{
		if (files[i]->empty())
		{
			std::cerr << "Cannot locate an image at: "<< files[i].name() << "!" << std::endl;

			terminate = true;
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

		cv::Size pairSize = images[0].size();
		if (pairSize != images[1].size())
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
		
		//// Step 3: Essential matrix estimation (with intermediate data conversion)
		std::vector<std::vector<cv::Point2d>> points = matchedKeypointsCoords<double>(_keypoints, _matches);

		double _val_dummy, _val_max;
		cv::minMaxIdx(points[0], &_val_dummy, &_val_max);

		double fx = static_cast<double>(std::max(pairSize.width, pairSize.height));
		intrinsics.at<double>(0,0) *= fx;
		intrinsics.at<double>(1,1) *= fx;
		intrinsics.at<double>(0,2) *= pairSize.width;
		intrinsics.at<double>(1,2) *= pairSize.height;

		std::vector<uchar> _match_status(_matches.size(), 0x00);
		cv::Mat	essential = intrinsics.t() * cv::findFundamentalMat(points[0], points[1], cv::FM_RANSAC, 0.006 * _val_max, 0.99, _match_status) * intrinsics;

		std::cout<<"Camera intrinsics are:"<<std::endl;
		std::cout<<intrinsics<<std::endl<<std::endl;

		std::cout<<"Essential matrix:"<<std::endl;
		std::cout<<essential<<std::endl;

		if (std::abs(cv::determinant(essential)) > 1e-07)
		{
			std::cerr << "Essential matrix determinant is not equal zero." << std::endl;
			retResult = -1; goto EXIT;
		}

		points = matchedKeypointsCoords<double>(_keypoints, _matches, _match_status);
		_match_status.clear();

		for (int i = 0, imax = points.size(); i < imax; ++i)
			cv::undistortPoints(points[i], points[i], intrinsics, distortion);

		//// Step 4: Camera matrices estimation up to projection transformation
		std::vector<cv::Mat_<double>> _R(2, cv::Mat_<double>(3,3));
		std::vector<cv::Mat_<double>> _t(2, cv::Mat_<double>(1,3));

		if (!HZEssentialDecomposition(essential, _R[0], _R[1], _t[0], _t[1]))
		{
			std::cerr << "Singular values are too far apart." << std::endl;
			retResult = -1; goto EXIT;
		}

		if (cv::determinant(_R[0]) + 1 < 1e-09 )
			HZEssentialDecomposition(-essential, _R[0], _R[1], _t[0], _t[1]);

		std::cout<<std::endl;

		std::vector<cv::Matx34d> camera(2, cv::Matx34d(
			1, 0, 0, /*I|0*/ 0, 
			0, 1, 0, /*I|0*/ 0, 
			0, 0, 1, /*I|0*/ 0
			));

		unsigned maxFrontalCount = 0;
		for (int r = 0; r < 2; ++r)
		{
			cv::Mat_<double> R = _R[r];
			if (!isCoherent(R)) continue;

			for (int t = 0; t < 2; ++t)
			{
				cv::Matx34d P_ = cv::Matx34d(
					R(0,0), R(0,1), R(0,2), _t[t](0),
					R(1,0), R(1,1), R(1,2), _t[t](1),
					R(2,0), R(2,1), R(2,2), _t[t](2)
					);

				cv::Mat spaceHomogeneous(points[0].size(), 1, CV_64FC4);
				cv::triangulatePoints(camera[0], P_, points[0], points[1], spaceHomogeneous);

				spaceHomogeneous = spaceHomogeneous.reshape(4, points[0].size());
				cv::Mat spaceEuclidian(points[0].size(), 1, CV_64FC3);
				for (int i = 0, imax = points[0].size(); i < imax; ++i)
				{
					cv::Vec4d point4d = spaceHomogeneous.at<cv::Vec4d>(i);
					double scale = point4d[3] != 0 ? 1 / point4d[3] : std::numeric_limits<double>::infinity();
					spaceEuclidian.at<cv::Point3d>(i) = cv::Point3d(
							point4d[0] * scale,
							point4d[1] * scale,
							point4d[2] * scale
						);
				}

				cv::Mat p(cv::Matx44d::eye());cv::Mat test(P_);
				cv::Mat(P_).copyTo(p(cv::Rect(0,0,4,3)));
				cv::perspectiveTransform(spaceEuclidian, spaceEuclidian, p);

				unsigned frontalCount = 0;
				for (int i = 0; i < spaceEuclidian.rows; ++i)
				{
					if (spaceEuclidian.at<cv::Point3d>(i).z > 0) frontalCount++;
				}
				if (frontalCount > maxFrontalCount)
				{
					maxFrontalCount = frontalCount;
					camera[1] = P_;
				}

			}
		}

			//std::vector<cv::Mat> correspondentLines(files.size());
			//cv::computeCorrespondEpilines(points[0], 1, F, correspondentLines[1]);
			//cv::computeCorrespondEpilines(points[1], 2, F, correspondentLines[0]);		

			//std::cout<<"------------Left Correspondent Lines-----------------"<<std::endl;
			//std::cout<<correspondentLines[0]<<std::endl;
			//std::cout<<"------------Right Correspondent Lines-----------------"<<std::endl;
			//std::cout<<correspondentLines[1]<<std::endl;

			//std::vector<cv::Mat> homography(files.size());
			//cv::stereoRectifyUncalibrated(points[0], points[1], F, pairSize, homography[0], homography[1]);

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

	HANDLE threadPresenter = CreateThread(NULL, 0, presenterThreadFunc, &output, 0x00, NULL);
	WaitForSingleObject(threadPresenter, INFINITE);
	CloseHandle(threadPresenter);

EXIT:
	std::cout << "Hit any key to exit...";
	_gettchar();
	return retResult;
}

