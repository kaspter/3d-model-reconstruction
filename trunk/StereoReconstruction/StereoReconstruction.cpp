// StereoReconstruction.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

#include "main_task_atomic_funcs.h"

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

#include "OrphanTask.h"
extern DWORD CALLBACK imagePresenterThreadFunc(LPVOID);
extern DWORD CALLBACK cloudPresenterThreadFunc(LPVOID);

#include "CloudData.h"
int _tmain(int argc, _TCHAR* argv[])
{
	int retResult = 0;

	//cv::SparseMat test;
	//test.ptr(0, 0);

	cv::CommandLineParser arguments(argc, argv, "{l|left||}{r|right||}{1|||}");

	std::vector<cv::Mat>			 images;
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
	files.push_back(ImageFileDescriptor(arguments.get<std::string>("left")));
	files.push_back(ImageFileDescriptor(arguments.get<std::string>("right")));
	images.resize(files.size());

	bool terminate = false;
	for (int i = 0, imax = files.size(); i < imax; ++i)
	{
		if (files[i]->empty())
		{
			std::cerr << "Cannot locate an image at: "<< files[i].name() << "!" << std::endl;

			terminate = true;
			continue;
		}
		cv::cvtColor(*files[i], images[i], cv::COLOR_RGB2GRAY);
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
		//imp::discreteGraymapHistogram(*files[0], image_histogram);

		size_t count_matches = 0;
		std::vector<std::vector<cv::Point2d>> points_matched;
		//// Step 1: Image pair feature matches processing
		std::cout << "Image pair feature tracking..." << std::endl;
		{	
			std::vector<std::vector<cv::KeyPoint>> _keypoints(2);
			cv::Ptr<cv::FeatureDetector> detector	= cv::FeatureDetector::create("PyramidSUSAN");
			imp::PyramidAdapterHack*	 pyramid	= reinterpret_cast<imp::PyramidAdapterHack*>(
				dynamic_cast<cv::PyramidAdaptedFeatureDetector*>(detector.obj));

			if (!!pyramid)
			{
				pyramid->maxLevel = 6;
				pyramid->detector->set("radius", 6);
				pyramid->detector->set("tparam", 26.75);
				pyramid->detector->set("gparam", 81.50);
				pyramid->detector->set("prefilter", true);
				pyramid->detector->set("subpixel", true);
			}
			else
			{
				std::cerr << "SUSAN is not pyramid adapted" << std::endl;
				retResult = -1; goto EXIT;
			}


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
			std::cout << pyramid->maxLevel << "-layer pyramid adapted SUSAN coped in: " << process_time << "s." << std::endl;
			std::cout << "Image " << files[0].name() << ": " << _keypoints[0].size() << " features." << std::endl;
			std::cout << "Image " << files[1].name() << ": " << _keypoints[1].size() << " features." << std::endl;
			std::cout << "With a total of " << _matches.size() << " matches" << std::endl;

			// Matched points visualization
			cv::Mat output;
			cv::drawMatches(*files[0],_keypoints[0], *files[1],_keypoints[1], _matches, output);
			OrphanTask::submit(imagePresenterThreadFunc, new cv::Mat(output));
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

		cv::Mat opencvCloud;
		std::vector<cv::Matx34d> camera(2);
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
	
			std::cout << "Robust reconstruction estimation:" << std::endl;
			cv::Mat spaceHomogeneous, spaceEuclidian;

			{ // Canonical left camera estimation
				cv::Mat _camera_canonical(cv::Matx34d::eye());
				intrinsics_optimal.copyTo(_camera_canonical(cv::Rect(cv::Point(), intrinsics_optimal.size())));
				camera[0] = _camera_canonical;
			}

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
		if (!opencvCloud.empty())
		{
			_ASSERT( opencvCloud.size().area() == fundamental_inliers[0].size() 
				&& opencvCloud.size().area() == fundamental_inliers[1].size() );

			pcl::PointCloud<pcl::PointXYZRGB> *cloud 
				= new pcl::PointCloud<pcl::PointXYZRGB>(opencvCloud.cols, opencvCloud.rows, pcl::PointXYZRGB());

			if (opencvCloud.rows > opencvCloud.cols) opencvCloud = opencvCloud.t();

			for (int r = 0; r < opencvCloud.rows; ++r)
			{
				cv::Point3f* row = opencvCloud.ptr<cv::Point3f>(r);

				for (int c = 0; c < opencvCloud.cols; ++c)
				{
					pcl::PointXYZRGB &point = (*cloud)(c, r);
					memcpy(point.data, row + c, sizeof(cv::Point3f));

					point.rgba	= 0xFF00FF00;
				}
			}
			OrphanTask::submit(cloudPresenterThreadFunc, new CloudData(cloud, camera));
		}
		/////////////////////////////////////////////////////////////////////////////////////////////////////
	}

EXIT:
	OrphanTask::join(TRUE);

	std::cout << std::endl << "Reconstruction " << ((!!retResult) ? "is aborted" : "complete") << "! Hit any key to exit...";
	_gettch();

	return retResult;
}

