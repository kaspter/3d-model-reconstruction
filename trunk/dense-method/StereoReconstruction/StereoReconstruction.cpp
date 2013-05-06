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
					? cv::Size(workAreaSize.cy * (image->cols / image->rows), workAreaSize.cy)
					: cv::Size(workAreaSize.cx, workAreaSize.cx * (image->rows / image->cols));

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

		viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "object cloud"); cloud->clear();
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.3, "object cloud");
	}

	viewer->setBackgroundColor(0.231, 0.231, 0.231);
	viewer->addCoordinateSystem(0.15F);
	viewer->initCameraParameters();

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	return 0;
}
int _tmain(int argc, _TCHAR* argv[])
{
	int retResult = 0;

	cv::CommandLineParser arguments(argc, argv, "{l|left||}{r|right||}{1|||}");

	std::vector<ImageFileDescriptor> files;
	std::vector<cv::Mat>			 images;
	cv::Mat intrinsics, distortion;

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
	files.push_back(ImageFileDescriptor(arguments.get<std::string>("left")));
	files.push_back(ImageFileDescriptor(arguments.get<std::string>("right")));

	images.resize(files.size(), cv::Mat());

	bool terminate = false;
	for (int i = 0, imax = files.size(); i < imax; ++i)
	{
		if (files[i]->empty())
		{
			std::cerr << "Cannot locate an image at: "<< files[i].name() << "!" << std::endl;

			terminate = true;
			continue;
		}
		cv::cvtColor(*files[i], images[i], CV_RGB2GRAY); //images.push_back(*files[i]);
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

		cv::Size pairSize = images[0].size();
		if (pairSize != images[1].size())
		{
			std::cerr << "Stereopair images are not equal by size!" << std::endl;
			retResult = -1; goto EXIT;
		}

		// TODO: Test graymap histogram
		//std::vector<unsigned> image_histogram;
		//imp::discreteGraymapHistogram(images[0], image_histogram);

		size_t count_matches = 0;
		std::vector<std::vector<cv::Point2d>> points_matched;
		//// Step 1: Image pair feature matches processing
		std::cout << "Image pair feature tracking..." << std::endl;
		{	
			std::vector<std::vector<cv::KeyPoint>> _keypoints(2);

			cv::Ptr<cv::FeatureDetector> detector  = cv::FeatureDetector::create("PyramidSUSAN");

			reinterpret_cast<imp::PyramidAdapterHack*>(detector.obj)->detector->set("radius", 6);
			reinterpret_cast<imp::PyramidAdapterHack*>(detector.obj)->detector->set("tparam", 26.75);
			reinterpret_cast<imp::PyramidAdapterHack*>(detector.obj)->detector->set("gparam", 81.50);
			reinterpret_cast<imp::PyramidAdapterHack*>(detector.obj)->detector->set("prefilter", true);
			reinterpret_cast<imp::PyramidAdapterHack*>(detector.obj)->maxLevel = 6;

			double process_time;
			IMP_BEGIN_TIMER_SECTION(timer)
			detector->detect(images, _keypoints);	
			IMP_END_TIMER_SECTION(timer, process_time);

			cv::KeyPointsFilter::removeDuplicated(_keypoints[0]);
			cv::KeyPointsFilter::removeDuplicated(_keypoints[1]);

			std::vector<cv::Mat> _descriptors(2);

			cv::Ptr<cv::DescriptorExtractor> extractor = cv::DescriptorExtractor::create("SIFT");
			extractor->compute(images, _keypoints, _descriptors);

			std::vector<cv::DMatch> _matches;
			cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("FlannBased");
			matcher->match(_descriptors[0],_descriptors[1], _matches);

			count_matches  = _matches.size();
			points_matched = matchedKeypointsCoords<double>(_keypoints, _matches);

			/////////////////////////////////////////////////////////////////////////////////////////////////////
			std::cout << reinterpret_cast<imp::PyramidAdapterHack*>(detector.obj)->maxLevel 
				<< "-layer pyramid adapted SUSAN coped in: " << process_time << "s." << std::endl;
			std::cout << "Image " << files[0].name() << ": " << _keypoints[0].size() << " features." << std::endl;
			std::cout << "Image " << files[1].name() << ": " << _keypoints[1].size() << " features." << std::endl;
			std::cout << "With a total of " << _matches.size() << " matches" << std::endl;

			// Matched points visualization
			cv::Mat output;
			cv::drawMatches(*files[0],_keypoints[0], *files[1],_keypoints[1], _matches, output);
			threadPresenter[0] = CreateThread(NULL, 0, matchPresenterThreadFunc, new cv::Mat(output), 0x00, NULL);
			/////////////////////////////////////////////////////////////////////////////////////////////////////
		}
		std::cout << std::endl; 

		cv::Mat	essential;
		std::vector<std::vector<cv::Point2d>> fundamental_inliers;
		//// Step 2: Essential matrix estimation (with intermediate data conversion)
		std::cout << "Essential matrix estimaiton..." << std::endl;
		{
			double _val_max;
			cv::minMaxIdx(points_matched[0], NULL, &_val_max);

			double fx = static_cast<double>(std::max(pairSize.width, pairSize.height));
			intrinsics.at<double>(0,0) *= fx;
			intrinsics.at<double>(1,1) *= fx;
			intrinsics.at<double>(0,2) *= pairSize.width;
			intrinsics.at<double>(1,2) *= pairSize.height;

			//intrinsics = cv::getOptimalNewCameraMatrix(intrinsics, distortion, pairSize, 0);
			cv::undistortPoints(points_matched[0], points_matched[0], intrinsics, distortion, cv::noArray(), intrinsics);	
			cv::undistortPoints(points_matched[1], points_matched[1], intrinsics, distortion, cv::noArray(), intrinsics);

			std::vector<uchar> _match_status(count_matches, 0x00);
			essential = intrinsics.t() * cv::findFundamentalMat(
				points_matched[0], points_matched[1], cv::FM_RANSAC, 0.006 * _val_max, 0.99, _match_status
			) * intrinsics;

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

		cv::Mat opencvCloud;
		//// Step 4: Camera matrices estimation up to projection transformation
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

			cv::Mat camera_canonical(cv::Matx34d::eye());
			intrinsics.copyTo(camera_canonical(cv::Rect(cv::Point(), intrinsics.size())));
			std::vector<cv::Matx34d> camera(2, camera_canonical);
		
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
					cv::Mat _P = intrinsics * cv::Mat(cv::Matx34d(
							R(0,0), R(0,1), R(0,2), _t[t](0),
							R(1,0), R(1,1), R(1,2), _t[t](1),
							R(2,0), R(2,1), R(2,2), _t[t](2)
						));

					cv::triangulatePoints(camera[0], _P, fundamental_inliers[0], fundamental_inliers[1], spaceHomogeneous);
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

						spaceEuclidian.convertTo(opencvCloud, CV_32F);
						camera[1] = _P; // "Precious" right Camera

						if (frontalPercentage == 100.0) 
						{
							std::cout << " (Best fit break!)" << std::endl;
							goto BEST_FIT_BREAK;
						}
					}
					std::cout << std::endl;
				}
			}

			if (opencvCloud.empty())
			{
				std::cerr << "Failed due to rotation matrices incoherence." << std::endl;
				retResult = -1; goto EXIT;
			}

			BEST_FIT_BREAK:
			/////////////////////////////////////////////////////////////////////////////////////////////////////
			std::cout << "Left camera matrix:  " << camera[0] << std::endl;
			std::cout << "Right camera matrix: " << camera[1] << std::endl;
			/////////////////////////////////////////////////////////////////////////////////////////////////////
		}
		std::cout << std::endl;

		/////////////////////////////////////////////////////////////////////////////////////////////////////
		// Model graph visualization
		{
			_ASSERT( opencvCloud.size().area() == fundamental_inliers[0].size() 
				&& opencvCloud.size().area() == fundamental_inliers[1].size() );

			pcl::PointCloud<pcl::PointXYZRGB> *cloud 
				= new pcl::PointCloud<pcl::PointXYZRGB>(opencvCloud.cols, opencvCloud.rows, pcl::PointXYZRGB());

			if (opencvCloud.rows > opencvCloud.cols) opencvCloud = opencvCloud.t();

			cv::Point2d *points[] = { &fundamental_inliers[0][0], &fundamental_inliers[1][0] };
			
			const uchar *rgb_source[2] = { NULL }; 
			for (int r = 0; r < opencvCloud.rows; ++r)
			{
				cv::Point3f* row = opencvCloud.ptr<cv::Point3f>(r);
				for (int c = 0; c < opencvCloud.cols; ++c)
				{
					pcl::PointXYZRGB &point = (*cloud)(c, r);
					memcpy(point.data, row + c, sizeof(cv::Point3f));

					// TODO: Fix color extraction, because it works improperly.
					//size_t index_linear = r * opencvCloud.cols + c;
					//rgb_source[0] = files[0]->ptr<uchar>(imp::round(points[0][index_linear].y), imp::round(points[0][index_linear].x));
					//rgb_source[1] = files[1]->ptr<uchar>(imp::round(points[1][index_linear].y), imp::round(points[1][index_linear].x)); 

					//point.rgba	= 0xFF000000;
					//point.b		= (rgb_source[0][0] + rgb_source[1][0]) >> 1;
					//point.g		= (rgb_source[0][1] + rgb_source[1][1]) >> 1; 
					//point.r		= (rgb_source[0][2] + rgb_source[1][2]) >> 1;
				}
			}
			threadPresenter[1] = CreateThread(NULL, 0, cloudPresenterThreadFunc, cloud, 0x00, NULL);
		}
		/////////////////////////////////////////////////////////////////////////////////////////////////////
	}

EXIT:
	{
		size_t threadOffset = 0;
		DWORD  threadCount  = 0;
		for (int i = 0; i < _countof(threadPresenter); ++i)
		{
			if (threadPresenter[i] != NULL)
			{
				if (threadOffset == 0) threadOffset = i;
				++threadCount;
			}
		}
		WaitForMultipleObjects(threadCount, threadPresenter + threadOffset, TRUE, INFINITE);
		for (int i = 0; i < _countof(threadPresenter); ++i) CloseHandle(threadPresenter[i]);
	}

	std::cout << std::endl << "Reconstruction " << ((!!retResult) ? "is aborted" : "complete") << "! Hit any key to exit...";
	_gettchar();

	return retResult;
}
