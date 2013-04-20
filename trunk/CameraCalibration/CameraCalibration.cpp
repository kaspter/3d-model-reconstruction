// CameraCalibration.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

bool PatternSize(const std::string &param_val, const std::string &delim_val, cv::Size &pattern_size)
{
	const char *delimeters = delim_val.c_str();
	char *context, *token = strtok_s(const_cast<char*>(param_val.c_str()), delimeters, &context);

	std::vector<int> values;
	while (token != NULL)
	{
		int value = atoi(token);
		if (value <= 0) return false;

		values.push_back(value);
		if (values.size() > 2) return false;

		token = strtok_s(NULL, delimeters, &context);
	}
	if (values.size() != 2) values.resize(2, values[0]);

	pattern_size = cv::Size(values[0], values[1]);
	return true;
}

int _tmain(int argc, _TCHAR* argv[])
{
	SetConsoleTitle("Chessboard camera calibration");

	int retResult = 0;

	SetLastError(0);

	{
		cv::CommandLineParser arguments(argc, argv, "{cbs||8,6|}{usz||10|}{dir|directory||}{out|output||}{1||^.+\\.jpe?g$|}");

		std::string appDir = arguments.get<std::string>("dir");
		if (!appDir.empty())
		{
			if (!SetCurrentDirectory(appDir.c_str())) std::cerr << appDir << " is not a valid path. Current directory was not changed." << std::endl;
		}
		appDir.resize(GetCurrentDirectory(0, NULL), 0x00);
		GetCurrentDirectory(appDir.size(), &appDir[0]);
		
		cv::Size pattern_size;
		float square_size;
		if (!PatternSize(arguments.get<std::string>("cbs"), ",;:", pattern_size))
		{
			std::cerr << "Bad pattern size specified." << std::endl;
			goto EXIT;
		}
		if ((square_size = arguments.get<float>("usz")) <= 0)
		{
			std::cerr << "Bad square size specified." << std::endl;
			goto EXIT;
		}

		std::cout << "Chessboard: " << pattern_size << " with " << square_size << "mm squares." << std::endl << std::endl;

		cv::Size                                initSize;
		std::vector<std::vector<cv::Point2f>>	imagePoints;

		try
		{
			std::regex regex(arguments.get<std::string>("1"));

			std::vector<std::string> file_set;

			WIN32_FIND_DATA ffData;
			HANDLE h = FindFirstFile(appDir.insert(appDir.length() - 1, "\\*.*").c_str(), &ffData);
			if( h != INVALID_HANDLE_VALUE )
			{
				std::cout << "Processing images..." << std::endl;
				do
				{
					if (std::regex_match(ffData.cFileName, regex))
					{
						cv::Mat image = cv::imread(ffData.cFileName, CV_LOAD_IMAGE_GRAYSCALE);
						if (image.empty())
						{
							std::cerr << "Cannot load '" << ffData.cFileName << "'! Skipped." << std::endl;
							continue;
						}

						std::vector<cv::Point2f> corners(pattern_size.area());
						if (!cv::findChessboardCorners(image, pattern_size, corners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS))
						{
							std::cerr << "Can't find chessboard corners at '" << ffData.cFileName << "'!" << std::endl;
							continue;
						}
						cv::cornerSubPix(image, corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

						if (initSize.area() == 0) initSize = image.size();
						imagePoints.push_back(corners);

						std::cout << imagePoints.size() << " '" << ffData.cFileName << "' (" << image.size().height << " x " << image.size().width << ")" << std::endl;
					}
				} while(FindNextFile(h,&ffData));
				std::cout << std::endl;
			}
			FindClose(h);
		}
		catch(std::regex_error &e)
		{
			std::cout << "Regex syntax problem. " << e.what() << std::endl;
			goto EXIT;
		}

		if (imagePoints.size() < 2)
		{
			std::cerr << "Insufficient ffData provided. Calibration aborted!" << std::endl;
			goto EXIT;
		}

		std::cout << "Calibrating camera..." << std::endl;

		std::vector<cv::Point3f> chessboardPoints(pattern_size.area());
		for (int r = 0, rmax = pattern_size.height; r < rmax; ++r)
		{
			cv::Point3f *row = &chessboardPoints[r * pattern_size.width];
			for (int c = 0, cmax = pattern_size.width; c < cmax; ++c)
				row[c] = cv::Point3f(c * square_size, r * square_size, 0);
		}
		std::vector<std::vector<cv::Point3f>> objectPoints(imagePoints.size(), chessboardPoints);

		cv::Mat K, d;
		std::vector<cv::Mat> r, t;

		double error = cv::calibrateCamera(objectPoints, imagePoints, initSize, K, d, r, t);

		double fx = K.at<double>(0,0);
		K.at<double>(0,0) = 1.0;
		K.at<double>(1,1) = K.at<double>(1,1) / fx;
		K.at<double>(0,2) /= initSize.width;
		K.at<double>(1,2) /= initSize.height;

		std::cout << "Final minimization error: " << error << std::endl;
		std::cout << "Camera intrinsics matrix:" << std::endl << K << std::endl;
		std::cout << "Camera distortion vector:" << std::endl << d << std::endl;

		std::string output = arguments.get<std::string>("out");
		if (!output.empty())
		{
			cv::FileStorage storage(output, cv::FileStorage::WRITE | cv::FileStorage::FORMAT_XML, "utf8");

			storage << "camera_intrinsics" << K << "camera_distortion" << d;
			storage.release();

			std::cout << "'" << output << "' was successfully saved!" << std::endl;
		}
		std::cout << std::endl;
	}

EXIT:
	std::cout << "Hit any key to exit...";
	_gettchar();
	return retResult;
}

