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

	std::vector<std::vector<cv::Point_<_pointT>>> points(2, std::vector<cv::Point_<_pointT>>(matches.size()));
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

inline cv::Point3f& operator/=(cv::Point3f &pt, int d) 
{
	pt.x /= d;
	pt.y /= d;
	pt.z /= d;

	return pt;
} 

bool HZEssentialDecomposition(cv::InputArray _E, cv::OutputArray R1, cv::OutputArray R2, cv::OutputArray t1, cv::OutputArray t2)
{
	cv::Mat E = _E.getMat();

	CV_Assert(E.depth() == CV_64F);

	int new_sz[] = { 3, 3 };
	cv::SVD _fm_svd(E.reshape(1, _countof(new_sz), new_sz));

	R1.create(E.size(), CV_64FC1);
	R2.create(E.size(), CV_64FC1);

	t1.create(3, 1, CV_64FC1);
	t2.create(3, 1, CV_64FC1);

	double _singular_ratio = std::abs(_fm_svd.w.at<double>(0) / _fm_svd.w.at<double>(1));
	if (_singular_ratio > 1.0) _singular_ratio = 1.0 / _singular_ratio; // flip ratio to keep it [0,1]
	if (_singular_ratio < 0.7) return false;

	cv::Matx33d W(0,-1, 0,	//HZ 9.13
				  1, 0, 0,
				  0, 0, 1);

	R1.getMat() = _fm_svd.u * cv::Mat(W)	 * _fm_svd.vt;	//HZ 9.19
	R2.getMat() = _fm_svd.u * cv::Mat(W.t()) * _fm_svd.vt;	//HZ 9.19
	
	_fm_svd.u.col(2).copyTo(t1.getMat());	//u3
	t2.getMat() = -_fm_svd.u.col(2);		//u3

	return true;
}
inline bool isCoherent(const cv::Mat &R)
{
	return std::abs(cv::determinant(R)) - 1.0 < 1e-07;
}

// Some presenter window supporting code is below
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
		if (!(cv::initModule_nonfree()
			&& cv::initModule_features2d()))
		{
			std::cerr << "Failed to init Non-Free & Features2d modules" << std::endl; 
			goto EXIT;
		}

		if (!imp::initModule())
		{
			std::cerr << "Failed to init ImageProcessor module" << std::endl; 
			goto EXIT;
		}

		cv::Size pairSize = images[0].size();
		if (pairSize != images[1].size())
		{
			std::cerr << "Stereopair images are not equal by size! Reconstruction is impossible.";
			goto EXIT;
		}

		// TODO: Test graymap histogram
		//std::vector<unsigned> image_histogram;
		//imp::discreteGraymapHistogram(images[0], image_histogram);

		//// Step 1: Image pair feature detection
		std::string tracker_name = "SIFT";
		cv::Ptr<cv::FeatureDetector>	 detector  = cv::FeatureDetector::create("PyramidSUSAN");
		cv::Ptr<cv::DescriptorExtractor> extractor = cv::DescriptorExtractor::create(tracker_name);

		reinterpret_cast<imp::PyramidAdapterHack*>(detector.obj)->detector->set("radius", 5);
		reinterpret_cast<imp::PyramidAdapterHack*>(detector.obj)->detector->set("tparam", 10.89);
		reinterpret_cast<imp::PyramidAdapterHack*>(detector.obj)->detector->set("gparam", 55.50);
		reinterpret_cast<imp::PyramidAdapterHack*>(detector.obj)->detector->set("prefilter", true);
		reinterpret_cast<imp::PyramidAdapterHack*>(detector.obj)->maxLevel = 16;

		std::vector<std::vector<cv::KeyPoint>> _keypoints(2);
		IMP_BEGIN_TIMER_SECTION(timer)
			detector->detect(images, _keypoints);	
		IMP_END_TIMER_SECTION(timer, "SUSAN coped in: ")
		detector.release();

		std::vector<cv::Mat> _descriptors(2);
		extractor->compute(images, _keypoints, _descriptors);

		//// Step 2: Image features correspondence tracking
		std::string matcher_name = "FlannBased";
		cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(matcher_name);

		std::vector<cv::DMatch> _matches;
		matcher->match(_descriptors[0],_descriptors[1], _matches);
		matcher.release();

		cv::drawMatches(images[0],_keypoints[0], images[1],_keypoints[1], _matches, output);
		//goto SHOW;
		
		//// Step 3: Essential matrix estimation (with intermediate data conversion)
		std::vector<std::vector<cv::Point2d>> points_matched = matchedKeypointsCoords<double>(_keypoints, _matches);

		double _val_max;
		cv::minMaxIdx(points_matched[0], NULL, &_val_max);

		double fx = static_cast<double>(std::max(pairSize.width, pairSize.height));
		intrinsics.at<double>(0,0) *= fx;
		intrinsics.at<double>(1,1) *= fx;
		intrinsics.at<double>(0,2) *= pairSize.width;
		intrinsics.at<double>(1,2) *= pairSize.height;

		intrinsics = cv::getOptimalNewCameraMatrix(intrinsics, distortion, pairSize, 0);
		cv::undistortPoints(points_matched[0], points_matched[0], intrinsics, distortion, cv::noArray(), intrinsics);	
		cv::undistortPoints(points_matched[1], points_matched[1], intrinsics, distortion, cv::noArray(), intrinsics);

		std::vector<uchar> _match_status(_matches.size(), 0x00);
		cv::Mat	essential = intrinsics.t() * cv::findFundamentalMat(points_matched[0], points_matched[1], cv::FM_RANSAC, 0.006 * _val_max, 0.99, _match_status) * intrinsics;

		std::cout<<"Camera intrinsics are:"<<std::endl;
		std::cout<<intrinsics<<std::endl<<std::endl;

		std::cout<<"Essential matrix:"<<std::endl;
		std::cout<<essential<<std::endl;

		if (!(std::abs(cv::determinant(essential)) < std::numeric_limits<float>::epsilon()))
		{
			std::cerr << "Essential matrix determinant is not equal zero." << std::endl;
			retResult = -1; goto EXIT;
		}

		//////////////////////////////////////////////////////////////////////////////////////////////////////
		// TODO: Filter points instead of the second matching alignment. Buggy behavior is HERE.
		// This code part is considered to be concluded within a particular funcrion. 
		std::vector<std::vector<cv::Point2d>> points_filtered(2, std::vector<cv::Point2d>(std::count_if(
			_match_status.begin(), _match_status.end(), [] (uchar status) -> bool { return !!status; }))
		);

		for (int v = 0; v < 2; ++v)
		{
			cv::Point2d *_points_src = &points_matched[v][0];
			cv::Point2d *_points_dst = &points_filtered[v][0];
			for (int i = 0, imax = points_matched[v].size(); i < imax; ++i)
			{
				if (_match_status[i]) *_points_dst++ = _points_src[i];
			}
		}
		//////////////////////////////////////////////////////////////////////////////////////////////////////

		// Considering the code above there is no need doing a point undistortion once more.
		//cv::undistortPoints(points_matched[0], points_matched[0], intrinsics, distortion, cv::noArray(), intrinsics);	
		//cv::undistortPoints(points_matched[1], points_matched[1], intrinsics, distortion, cv::noArray(), intrinsics);
		_match_status.clear();

		//// Step 4: Camera matrices estimation up to projection transformation
		std::vector<cv::Mat_<double>> _R(2), _t(2);

		if (!HZEssentialDecomposition(essential, _R[0], _R[1], _t[0], _t[1]))
		{
			std::cerr << "Singular values are too far apart." << std::endl;
			retResult = -1; goto EXIT;
		}

		if (cv::determinant(_R[0]) + 1 < std::numeric_limits<float>::epsilon()) // det(R1) == -1
			HZEssentialDecomposition(-essential, _R[0], _R[1], _t[0], _t[1]);

		double _ax = intrinsics.at<double>(0, 0);
		double _ay = intrinsics.at<double>(1, 1);
		std::vector<cv::Matx34d> camera(2, cv::Matx34d(
				_ax, 0.0, 0.0, /*I|0*/ 0.0, 
				0.0, _ay, 0.0, /*I|0*/ 0.0, 
				0.0, 0.0, 1.0, /*I|0*/ 0.0
			));

		cv::Mat opencvCloud;

		double maxFrontalPercentage = 0;
		for (int r = 0; r < 2; ++r)
		{
			cv::Mat_<double> R = _R[r];
			if (!isCoherent(R)) continue;

			for (int t = 0; t < 2; ++t)
			{
				cv::Mat _P = intrinsics * cv::Mat(cv::Matx34d(
						R(0,0), R(0,1), R(0,2), _t[t](0),
						R(1,0), R(1,1), R(1,2), _t[t](1),
						R(2,0), R(2,1), R(2,2), _t[t](2)
					));

				cv::Mat spaceHomogeneous;
				cv::triangulatePoints(camera[0], _P, points_filtered[0], points_filtered[1], spaceHomogeneous);

				spaceHomogeneous = spaceHomogeneous.reshape(4, points_filtered[0].size());
				cv::Mat spaceEuclidianProjected, spaceEuclidian(spaceHomogeneous.rows, 1, CV_64FC3);
				for (int i = 0, imax = points_filtered[0].size(); i < imax; ++i)
				{
					cv::Vec4d point4d = spaceHomogeneous.at<cv::Vec4d>(i);
					double scale = point4d[3] != 0 ? 1 / point4d[3] : std::numeric_limits<double>::infinity();
					spaceEuclidian.at<cv::Point3d>(i) = cv::Point3d(
							point4d[0] * scale,
							point4d[1] * scale,
							point4d[2] * scale
						);
				}

				cv::Mat p(cv::Matx44d::eye());
				cv::Mat(_P).copyTo(p(cv::Rect(0,0,4,3)));
				cv::perspectiveTransform(spaceEuclidian, spaceEuclidianProjected, p);

				unsigned frontalCount = 0;
				for (int i = 0; i < spaceEuclidianProjected.rows; ++i)
				{
					if (spaceEuclidianProjected.at<cv::Point3d>(i).z > 0) frontalCount++;
				}
				double frontalPercentage = 100.0 * frontalCount / spaceEuclidianProjected.rows;
				if (frontalPercentage > maxFrontalPercentage)
				{
					maxFrontalPercentage = frontalPercentage;
					
					camera[1] = _P;
					spaceEuclidian.convertTo(opencvCloud, CV_32F);
				}
			}
		}

		std::cout << std::endl;

		// Step 6: Model graph visualization
		if (!opencvCloud.empty())
		{
			boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud (
					new pcl::PointCloud<pcl::PointXYZ>(opencvCloud.cols, opencvCloud.rows, pcl::PointXYZ())
				);

			cv::Point3f average_point;
			for (int r = 0; r < opencvCloud.rows; ++r)
			{
				cv::Point3f* row = opencvCloud.ptr<cv::Point3f>(r);
				for (int c = 0; c < opencvCloud.cols; ++c)
				{
					memcpy((*cloud)(c, r).data, row + c, sizeof(cv::Point3f));
					average_point += *row;
					// DEBUG
					printf_s("{%3.2f;%3.2f;%3.2f} => {%3.2f;%3.2f;%3.2f}\n",
							row[c].x, row[c].y, row[c].z, (*cloud)(c, r).x, (*cloud)(c, r).y, (*cloud)(c, r).z);
					// DEBUG end
				}
			}
			average_point /= opencvCloud.size().area();
			
			boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
					new pcl::visualization::PCLVisualizer("3D model vertex cloud")
				);
			
			viewer->addPointCloud<pcl::PointXYZ>(cloud, "object cloud");
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.0, "object cloud");

			viewer->setBackgroundColor(0.275, 0.431, 0.824);
			viewer->addCoordinateSystem(std::min(
					std::min(std::min(std::abs(average_point.x), std::abs(average_point.y)), std::abs(average_point.z)),
				1.0F));

			double cam_shift = std::max(std::max(average_point.x, average_point.y), average_point.z) * 0.3;
			viewer->setCameraPosition(cam_shift, cam_shift, cam_shift, 0.0, 0.0, 0.0);

			while (!viewer->wasStopped())
			{
				viewer->spinOnce(100);
				boost::this_thread::sleep(boost::posix_time::microseconds(100000));
			}
		}
		//// Step 5: Image rectification
			//std::vector<cv::Mat> homography(2);
			//cv::stereoRectifyUncalibrated(points[0], points[1], F, pairSize, homography[0], homography[1]);

			//std::vector<cv::Mat> remapData(2);
			//cv::Mat tempRemapped;

			//cv::initUndistortRectifyMap(intrinsics, distortion, cv::Mat(), intrinsics, pairSize, CV_16SC2, remapData[0], remapData[1]);
			//
			//cv::remap(images[0], tempRemapped, remapData[0], remapData[1], cv::INTER_LINEAR);
			//tempRemapped.copyTo(images[0]);

			//cv::remap(images[1], tempRemapped, remapData[0], remapData[1], cv::INTER_LINEAR);
			//tempRemapped.copyTo(images[1]);

			//output.create(images[0].rows, images[0].cols * 2, images[0].type());
			//images[0].copyTo(output(cv::Rect(0, 0, images[0].cols, images[0].rows)));
			//images[1].copyTo(output(cv::Rect(images[0].cols, 0, images[1].cols, images[1].rows)));
	}


EXIT:
	HANDLE threadPresenter = CreateThread(NULL, 0, presenterThreadFunc, &output, 0x00, NULL);
	WaitForSingleObject(threadPresenter, INFINITE);
	CloseHandle(threadPresenter);

//EXIT:
	std::cout << "Hit any key to exit...";
	_gettchar();
	return retResult;
}

