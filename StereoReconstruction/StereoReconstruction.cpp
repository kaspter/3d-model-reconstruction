// StereoReconstruction.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

enum comparison_predicate_mode {
	LESS_THAN	 = 0x01,
	EQUAL		 = 0x02,
	GREATER_THAN = 0x04,

	LESS_THAN_OR_EQUAL	  = EQUAL | LESS_THAN,
	NOT_EQUAL			  = LESS_THAN | GREATER_THAN,
	GREATER_THAN_OR_EQUAL = GREATER_THAN | EQUAL
};

template<typename _Vt, int _mode>
class comparison_predicate
{
	_Vt _gauge;

	template <int _mode>
	bool compare(const _Vt &suspect) { throw std::exception("Bad comparison_predicate mode", _mode); }

	template<> bool compare< LESS_THAN             >(const _Vt &suspect) { return suspect <  _gauge; }
	template<> bool compare< LESS_THAN_OR_EQUAL    >(const _Vt &suspect) { return suspect <= _gauge; }
	template<> bool compare< EQUAL                 >(const _Vt &suspect) { return suspect == _gauge; }
	template<> bool compare< GREATER_THAN_OR_EQUAL >(const _Vt &suspect) { return suspect >= _gauge; }
	template<> bool compare< GREATER_THAN          >(const _Vt &suspect) { return suspect >  _gauge; }
	template<> bool compare< NOT_EQUAL             >(const _Vt &suspect) { return suspect != _gauge; }

public:
	comparison_predicate(_Vt  &gauge) : _gauge(gauge)	{ }
	comparison_predicate(_Vt &&gauge) : _gauge(std::forward<_Vt>(gauge)) { }

	comparison_predicate &operator=(_Vt &&gauge) { _gauge = std::move<_Vt>(gauge); }

	bool operator() (const _Vt &suspect) { return compare<_mode>(suspect); }
};

template <typename _Vt>
std::vector<std::vector<cv::Point_<_Vt>>> matchedKeypointsCoords(const std::vector<std::vector<cv::KeyPoint>> &keypoints, 
	const std::vector<cv::DMatch> &matches, const std::vector<uchar> &match_status = std::vector<uchar>())
{
	const uchar	*_match_status = match_status.empty() ? NULL : &match_status[0];
	CV_Assert( keypoints.size() == 2 && (_match_status == NULL || match_status.size() == matches.size()) );
	const cv::KeyPoint *_points_src[] = { &keypoints[0][0], &keypoints[1][0] };
	
	size_t inliers_count = _match_status == NULL ? matches.size() 
		: std::count_if(match_status.begin(), match_status.end(), comparison_predicate<uchar, NOT_EQUAL>(0x00));
	std::vector<std::vector<cv::Point_<_Vt>>> fundamental_inliers(2, std::vector<cv::Point_<_Vt>>(inliers_count));

	if (inliers_count != 0)
	{
		cv::Point_<_Vt>	*_points_dst[] = { &fundamental_inliers[0][0], &fundamental_inliers[1][0] };

		size_t current = 0;
		for (size_t i=0, imax = matches.size(); i<imax; ++i) 
		{
			if ( _match_status == NULL || _match_status[i] != 0x00 )
			{
				const cv::DMatch &match = matches[i];
				_points_dst[0][current] = cv::Point_<_Vt>(_points_src[0][match.queryIdx].pt.x, _points_src[0][match.queryIdx].pt.y);
				_points_dst[1][current] = cv::Point_<_Vt>(_points_src[1][match.trainIdx].pt.x, _points_src[1][match.trainIdx].pt.y);
				++current;
			}
		}

		fundamental_inliers[0].shrink_to_fit();
		fundamental_inliers[1].shrink_to_fit();
	}

	return fundamental_inliers;
}

template <typename _Vt>
std::vector<std::vector<cv::Point_<_Vt>>> filterPointsByStatus(
	const std::vector<std::vector<cv::Point_<_Vt>>> &fundamental_inliers, const std::vector<uchar> &status)
{
	CV_Assert( fundamental_inliers.size() == 2 && (fundamental_inliers[0].size() == status.size() && fundamental_inliers[1].size() == status.size()) );

	const uchar				*_status	   =   &status[0];
	const cv::Point_<_Vt>	*_points_src[] = { &fundamental_inliers[0][0], &fundamental_inliers[1][0] };

	size_t inliers_count = std::count_if(status.begin(), status.end(), comparison_predicate<uchar, NOT_EQUAL>(0x00));
	std::vector<std::vector<cv::Point_<_Vt>>> points_filtered(2, std::vector<cv::Point_<_Vt>>(inliers_count));
	
	if (inliers_count != 0)
	{
		cv::Point_<_Vt>	*_points_dst[] = { &points_filtered[0][0], &points_filtered[1][0] };

		size_t current = 0;
		for (int i = 0, imax = status.size(); i < imax; ++i)
		{
			if (_status[i]) 
			{
				_points_dst[0][current] = _points_src[0][i];
				_points_dst[1][current] = _points_src[1][i];
				++current;
			}
		}

		points_filtered[0].shrink_to_fit();
		points_filtered[1].shrink_to_fit();
	}

	return points_filtered;
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

template <typename _ElemT>
void pointsFromHomogeneous(cv::InputArray src, cv::OutputArray dst)
{
	if (src.empty()) return;

	cv::Mat _dst, _src = src.getMat();
	CV_Assert( _src.rows == 4 && _src.channels() == 1 && _src.elemSize1() == sizeof(_ElemT) );

	dst.create(_src.cols, 1, CV_MAKETYPE(_src.depth(), 3));
	_dst = dst.getMat();

	_ElemT *point_cliche[] = { 
		_src.ptr<_ElemT>(0),
		_src.ptr<_ElemT>(1),
		_src.ptr<_ElemT>(2),
		_src.ptr<_ElemT>(3)
	};

	for (int i = 0; i < _src.cols; ++i)
	{
		_ElemT scale = point_cliche[3][i] != _ElemT(0)
			? _ElemT(1) / point_cliche[3][i] : std::numeric_limits<_ElemT>::infinity();
		_dst.at<cv::Point3_<_ElemT>>(i) = cv::Point3_<_ElemT>(
				point_cliche[0][i] * scale,
				point_cliche[1][i] * scale,
				point_cliche[2][i] * scale
			);
	}
}

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

// Some presenter window supporting code is below
HANDLE	threadPresenter[2]	= { NULL };
WNDPROC subclassWindowProc	=	NULL;   // Such approach is not considered to be used simultaneously with multiple image-presenter windows
LRESULT CALLBACK matchPresenterWindowProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
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
DWORD CALLBACK matchPresenterThreadFunc(LPVOID threadParameter)
{
	_ASSERT(threadParameter != NULL);

	std::string windowName = "Keypoint presenter window";

	HWND hwnd = NULL;
	{ // Delete image in memory as soon as possible
		boost::shared_ptr<cv::Mat> image(static_cast<cv::Mat*>(threadParameter));

		cv::namedWindow(windowName);

		hwnd = static_cast<HWND>(cvGetWindowHandle(windowName.c_str()));
		if (hwnd == NULL)
		{
			MessageBox(GetConsoleWindow(), _T("Cannot start presenter window!"), _T("Error"), MB_OK | MB_ICONWARNING);
			return -1;
		}

		subclassWindowProc	= (WNDPROC)SetWindowLong(hwnd, GWL_WNDPROC, (LONG)matchPresenterWindowProc);

		HMONITOR monitor = MonitorFromWindow(hwnd, MONITOR_DEFAULTTOPRIMARY);
		MONITORINFO mi; mi.cbSize = sizeof(mi);
		if (GetMonitorInfo(monitor, &mi))
		{
			SIZE workAreaSize = { std::abs(mi.rcWork.right - mi.rcWork.left) * 0.8, std::abs(mi.rcWork.bottom - mi.rcWork.top) * 0.8 };
			if (image->cols > workAreaSize.cx || image->rows > workAreaSize.cy)
			{
				cv::Size newImageSize = image->cols > image->rows 
					? cv::Size(workAreaSize.cx, int(imp::round(workAreaSize.cx * (double(image->rows) / image->cols))))
					: cv::Size(int(imp::round(workAreaSize.cy * (double(image->cols) / image->rows))), workAreaSize.cy);

				cv::resize(*image, *image, newImageSize, 0, 0, cv::INTER_CUBIC);
			}
		}

		cv::imshow(windowName, *image);
	}


	MSG msg;
	while (GetMessage(&msg, NULL, 0, 0))
	{
		TranslateMessage(&msg);
		DispatchMessage(&msg);
	}
	return msg.wParam;
}
DWORD CALLBACK cloudPresenterThreadFunc(LPVOID threadParameter)
{
	_ASSERT(threadParameter != NULL);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
			new pcl::visualization::PCLVisualizer("3D model vertex cloud"));

	{ // Delete cloud-in-memory as soon as possible
		boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud(
			static_cast<pcl::PointCloud<pcl::PointXYZRGB>*>(threadParameter));

		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
		viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "object cloud"); cloud->clear();
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.0, "object cloud");
	}

	viewer->setBackgroundColor(0.231, 0.231, 0.231);
	viewer->addCoordinateSystem(0.1);
	viewer->initCameraParameters();

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(1000);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	return 0;
}
int _tmain(int argc, _TCHAR* argv[])
{
	int retResult = 0;

	cv::CommandLineParser arguments(argc, argv, "{l|left||}{r|right||}{1|||}");

	std::vector<ImageFileDescriptor> files;
	cv::Mat intrinsics, intrinsics_optimal, distortion;

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
	files.push_back(ImageFileDescriptor(arguments.get<std::string>("left"), cv::IMREAD_GRAYSCALE));
	files.push_back(ImageFileDescriptor(arguments.get<std::string>("right"), cv::IMREAD_GRAYSCALE));

	bool terminate = false;
	for (int i = 0, imax = files.size(); i < imax; ++i)
	{
		if (files[i]->empty())
		{
			std::cerr << "Cannot locate an image at: "<< files[i].name() << "!" << std::endl;

			terminate = true;
			continue;
		}
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
			retResult = -1; goto EXIT;
		}

		if (!imp::initModule())
		{
			std::cerr << "Failed to init ImageProcessor module" << std::endl; 
			retResult = -1; goto EXIT;
		}

		cv::Size pairSize = files[0]->size();
		if (pairSize != files[1]->size())
		{
			std::cerr << "Stereopair *files are not equal by size!" << std::endl;
			retResult = -1; goto EXIT;
		}

		// TODO: Test graymap histogram
		//std::vector<unsigned> image_histogram;
		//imp::discreteGraymapHistogram(*files[0], image_histogram);

		size_t count_matches = 0;
		std::vector<std::vector<cv::Point2d>> points_matched;
		//// Step 1: Image pair feature matches processing
		std::cout << "Image pair feature tracking..." << std::endl;
		{	
			std::vector<std::vector<cv::KeyPoint>>	_keypoints(2);
			std::vector<cv::Mat>					_descriptors(2);

			cv::Ptr<cv::Feature2D> sift = cv::Feature2D::create("SIFT");

			sift->set("contrastThreshold", 0.018);
			//sift->set("edgeThreshold", 105.6);

			double process_time;
			IMP_BEGIN_TIMER_SECTION(tick_pin);
			(*sift)(*files[0], cv::Mat(), _keypoints[0], _descriptors[0]);
			(*sift)(*files[1], cv::Mat(), _keypoints[1], _descriptors[1]);
			IMP_END_TIMER_SECTION(tick_pin, process_time);

			std::vector<cv::DMatch> _matches;
			cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("FlannBased");
			matcher->match(_descriptors[0],_descriptors[1], _matches);

			count_matches  = _matches.size();
			points_matched = matchedKeypointsCoords<double>(_keypoints, _matches);

			/////////////////////////////////////////////////////////////////////////////////////////////////////
			std::cout << "SIFT coped in: " << process_time << "s." << std::endl;
			std::cout << "Image " << files[0].name() << ": " << _keypoints[0].size() << " features." << std::endl;
			std::cout << "Image " << files[1].name() << ": " << _keypoints[1].size() << " features." << std::endl;
			std::cout << "With a total of " << _matches.size() << " matches" << std::endl;

			// Matched points visualization
			cv::Mat output;
			cv::drawMatches(*files[0], _keypoints[0], *files[1], _keypoints[1], _matches, output);
			threadPresenter[0] = CreateThread(NULL, 0, matchPresenterThreadFunc, new cv::Mat(output), 0x00, NULL);
			/////////////////////////////////////////////////////////////////////////////////////////////////////
		}
		std::cout << std::endl; 

		cv::Mat	essential;
		std::vector<std::vector<cv::Point2d>> fundamental_inliers;
		//// Step 2: Essential matrix estimation (with intermediate data conversion)
		std::cout << "Essential matrix estimaiton..." << std::endl;
		{
			double fx = static_cast<double>(std::max(pairSize.width, pairSize.height));
			intrinsics.at<double>(0,0) *= fx;
			intrinsics.at<double>(1,1) *= fx;
			intrinsics.at<double>(0,2) *= pairSize.width;
			intrinsics.at<double>(1,2) *= pairSize.height;

			intrinsics_optimal = cv::getOptimalNewCameraMatrix(intrinsics, distortion, pairSize, 0);
			cv::undistortPoints(points_matched[0], points_matched[0], intrinsics, distortion, cv::noArray(), intrinsics_optimal);	
			cv::undistortPoints(points_matched[1], points_matched[1], intrinsics, distortion, cv::noArray(), intrinsics_optimal);

			double _val_max;
			cv::minMaxIdx(points_matched[0], NULL, &_val_max);

			std::vector<uchar> _match_status(count_matches, 0x00);
			essential = intrinsics_optimal.t() * cv::findFundamentalMat(
				points_matched[0], points_matched[1], cv::FM_RANSAC, 0.006 * _val_max, 0.99, _match_status
			) * intrinsics_optimal;

			if (!(std::abs(cv::determinant(essential)) < std::numeric_limits<float>::epsilon()))
			{
				std::cerr << "Essential matrix determinant is not equal zero." << std::endl;
				retResult = -1; goto EXIT;
			}
			fundamental_inliers = filterPointsByStatus(points_matched, _match_status);
			
			//cv::correctMatches(essential, fundamental_inliers[0], fundamental_inliers[1], fundamental_inliers[0], fundamental_inliers[1]);

			double _inliers_percentage = 100.0 * fundamental_inliers[0].size() / count_matches;
			/////////////////////////////////////////////////////////////////////////////////////////////////////
			std::cout<<"Camera intrinsics are:"<<std::endl;
			std::cout<<intrinsics<<std::endl<<std::endl;

			std::cout<<"Essential matrix:"<<std::endl;
			std::cout<<essential<<std::endl;

			std::cout << "A total number of matched inliers: " << fundamental_inliers[0].size() 
				<< " (" << _inliers_percentage << "%)." << std::endl;
			/////////////////////////////////////////////////////////////////////////////////////////////////////
		}
		std::cout << std::endl;

		cv::Mat rotation, translation;
		//// Step 3: Camera matrices estimation up to projection transformation
		std::cout << "Camera matrices estimation..." << std::endl;
		{
			std::vector<cv::Mat_<double>> _R(2), _t(2);

			if (!HZEssentialDecomposition(essential, _R[0], _R[1], _t[0], _t[1]))
			{
				std::cerr << "Essential matrix singular values are too far apart." << std::endl;
				retResult = -1; goto EXIT;
			}

			if (cv::determinant(_R[0]) + 1 < std::numeric_limits<float>::epsilon()) // det(R1) == -1
				HZEssentialDecomposition(-essential, _R[0], _R[1], _t[0], _t[1]);

			cv::Mat _camera_right, _camera_left(cv::Matx34d::eye());
			intrinsics_optimal.copyTo(_camera_left(cv::Rect(cv::Point(), intrinsics_optimal.size())));
		
			std::cout << "Robust reconstruction estimation:" << std::endl;
			cv::Mat spaceHomogeneous, spaceEuclidian;

			double minReprojectionError = std::numeric_limits<double>::max(); 
			double maxFrontalPercentage = 0;
			for (int r = 0; r < 2; ++r)
			{
				cv::Mat_<double> R = _R[r];
				if (!isCoherent(R)) continue;

				for (int t = 0; t < 2; ++t)
				{
					cv::Mat _P = intrinsics_optimal * cv::Mat(cv::Matx34d(
							R(0,0), R(0,1), R(0,2), _t[t](0),
							R(1,0), R(1,1), R(1,2), _t[t](1),
							R(2,0), R(2,1), R(2,2), _t[t](2)
						));

					cv::triangulatePoints(_camera_left, _P, fundamental_inliers[0], fundamental_inliers[1], spaceHomogeneous);
					pointsFromHomogeneous<double>(spaceHomogeneous, spaceEuclidian);

					cv::Mat p(cv::Matx44d::eye());
					cv::Mat(_P).copyTo(p(cv::Rect(cv::Point(), _P.size())));
					cv::Mat spaceEuclidianReprojected;
					cv::perspectiveTransform(spaceEuclidian, spaceEuclidianReprojected, p);

					double errorReprojected	 = 0.0;
					double frontalPercentage = 0.0;
					for (int i = 0; i < spaceEuclidianReprojected.rows; ++i)
					{
						cv::Point3d point_reprojected = spaceEuclidianReprojected.at<cv::Point3d>(i);
						if (point_reprojected.z > 0) frontalPercentage++;

						errorReprojected += cv::norm(cv::Point2d(
							point_reprojected.x / point_reprojected.z, 
							point_reprojected.y / point_reprojected.z)
						- fundamental_inliers[1][i]);
					}

					errorReprojected /= spaceEuclidianReprojected.rows;
					frontalPercentage = 100.0 * frontalPercentage / spaceEuclidianReprojected.rows;

					std::cout << "Frontal percentage/Reprojection error: " 
						<< frontalPercentage << "/" << errorReprojected << ". ";

					if (errorReprojected < minReprojectionError 
						&& (errorReprojected / minReprojectionError 
							>= (maxFrontalPercentage - frontalPercentage) / frontalPercentage))
					{
						std::cout << (maxFrontalPercentage == 0.0 ? "Satisfy" : "Overrides") << "!";
						maxFrontalPercentage = frontalPercentage;

						rotation	= R;
						translation = _t[t];

						_camera_right = _P;

						if (frontalPercentage == 100.0) 
						{
							std::cout << " (Best fit break!)" << std::endl;
							goto BEST_FIT_BREAK;
						}
					}
					std::cout << std::endl;
				}
			}

			BEST_FIT_BREAK:
			/////////////////////////////////////////////////////////////////////////////////////////////////////
			std::cout << "Left camera matrix:  " << _camera_left  << std::endl;
			std::cout << "Right camera matrix: " << _camera_right << std::endl;
			/////////////////////////////////////////////////////////////////////////////////////////////////////
		}
		std::cout << std::endl;

		cv::Mat opencvCloud;
		{
			cv::Mat _depth_to_disparity;
			std::vector<cv::Mat> _image_rotation(2), _new_camera(2);
			cv::stereoRectify(intrinsics, distortion, intrinsics, distortion, pairSize, rotation, translation, 
				_image_rotation[0], _image_rotation[1], _new_camera[0], _new_camera[1], _depth_to_disparity, cv::CALIB_ZERO_DISPARITY, 0.0);

			std::vector<cv::Mat> _map(4);
			cv::initUndistortRectifyMap(intrinsics, distortion, _image_rotation[0], _new_camera[0], pairSize, CV_16SC2, _map[0], _map[2]);
			cv::initUndistortRectifyMap(intrinsics, distortion, _image_rotation[1], _new_camera[1], pairSize, CV_16SC2, _map[1], _map[3]);
			_image_rotation.clear();

			cv::remap(*files[0], *files[0], _map[0], _map[2], cv::INTER_CUBIC);
			cv::remap(*files[1], *files[1], _map[1], _map[3], cv::INTER_CUBIC);
			_map.clear();

			cv::StereoSGBM _matcher(0, 16, 5, 600, 2400, -1, 128);

			cv::Mat _disparity;
			_matcher(*files[0], *files[1], _disparity);

			cv::reprojectImageTo3D(_disparity, opencvCloud, _depth_to_disparity);
		}

		/////////////////////////////////////////////////////////////////////////////////////////////////////
		// Model graph visualization
		if (!opencvCloud.empty())
		{
			pcl::PointCloud<pcl::PointXYZRGB> *cloud 
				= new pcl::PointCloud<pcl::PointXYZRGB>(opencvCloud.cols, opencvCloud.rows, pcl::PointXYZRGB());
	
			//const uchar *rgb_source[2] = { NULL }; 
			for (int r = 0; r < opencvCloud.rows; ++r)
			{
				cv::Point3f* row = opencvCloud.ptr<cv::Point3f>(r);

				//rgb_source[0] = files[0]->ptr<uchar>(r);
				//rgb_source[1] = files[1]->ptr<uchar>(r); 
				for (int c = 0; c < opencvCloud.cols; ++c)
				{
					pcl::PointXYZRGB &point = (*cloud)(c, r);
					memcpy(point.data, row + c, sizeof(cv::Point3f));

					point.rgba	= 0xFF00FF00;
					//point.b		= (rgb_source[0][0] + rgb_source[1][0]) >> 1;
					//point.g		= (rgb_source[0][1] + rgb_source[1][1]) >> 1; 
					//point.r		= (rgb_source[0][2] + rgb_source[1][2]) >> 1;

					//rgb_source[0] += files[0]->elemSize();
					//rgb_source[1] += files[1]->elemSize();
				}
			}
			threadPresenter[1] = CreateThread(NULL, 0, cloudPresenterThreadFunc, cloud, 0x00, NULL);
		}
		/////////////////////////////////////////////////////////////////////////////////////////////////////
	}

EXIT:
	{
		size_t threadOffset = -1;
		DWORD  threadCount  = 0;
		for (size_t i = 0; i < _countof(threadPresenter); ++i)
		{
			if (threadPresenter[i] != NULL)
			{
				if (threadOffset == -1) threadOffset = i;
				++threadCount;
			}
		}
		WaitForMultipleObjects(threadCount, threadPresenter + std::max(threadOffset, size_t(0)), TRUE, INFINITE);
		for (size_t i = 0; i < threadCount; ++i) CloseHandle(threadPresenter[i]);
	}

	std::cout << std::endl << "Reconstruction " << ((!!retResult) ? "is aborted" : "complete") << "! Hit any key to exit...";
	_gettchar();

	return retResult;
}

