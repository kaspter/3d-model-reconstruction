// CameraCalibration.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "resource.h"

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

	pattern_size = cv::Size(values[1], values[0]);
	return true;
}

int _tmain(int argc, _TCHAR* argv[])
{
	int retResult = 0;

	SetLastError(0);

	{
		LPWSTR rcWideString = NULL;
		if (LoadStringW(GetModuleHandle(NULL), IDS_PARSER_TEMPLATE, (LPWSTR)&rcWideString, 0) == 0)
		{
			retResult = static_cast<int>(GetLastError());
			goto EXIT;
		}
		size_t rcWideStringLength = wcslen(rcWideString);

		std::string parserTemplate(rcWideStringLength, 0x00);
		if (WideCharToMultiByte(CP_ACP, WC_COMPOSITECHECK, rcWideString, rcWideStringLength, &parserTemplate[0], parserTemplate.size(), NULL, FALSE) == 0);
		{
			retResult = static_cast<int>(GetLastError());
			if (retResult != 0) goto EXIT;
		}

		cv::CommandLineParser arguments(argc, argv, parserTemplate.c_str());

		std::string appDir = arguments.get<std::string>("dir");
		if (!appDir.empty())
		{
			if (!SetCurrentDirectory(appDir.c_str())) std::cerr << appDir << " is not a valid path. Current directory was not changed." << std::endl;
		}
		appDir.resize(GetCurrentDirectory(0, NULL), 0x00);
		GetCurrentDirectory(appDir.size(), &appDir[0]);
		
		std::vector<std::vector<cv::Point2f>> opticalFlow;

		cv::Size chessboard_size;
		if (!PatternSize(arguments.get<std::string>("pat"), ",;:", chessboard_size))
		{
			std::cerr << "Bad pattern size specified." << std::endl;
			goto EXIT;
		}

		WIN32_FIND_DATA data;
		HANDLE h = FindFirstFile(appDir.insert(appDir.length() - 1, "\\*.*").c_str(), &data);
		if( h != INVALID_HANDLE_VALUE )
		{
			std::regex  regex(arguments.get<std::string>("1"));
			do
			{
				if (std::regex_match(data.cFileName, regex))
				{
					cv::Mat image = cv::imread(data.cFileName);
					if (image.data == NULL)
					{
						std::cerr << "Cannot load '" << data.cFileName << "'! Skipped." << std::endl;
						continue;
					}
					cv::cvtColor(image, image, cv::COLOR_RGB2GRAY);

					std::vector<cv::Point2f> corners(chessboard_size.area());
					if (!cv::findChessboardCorners(image, chessboard_size, corners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS))
					{
						std::cerr << "Can't find chessboard at '" << data.cFileName << "'!" << std::endl;
						continue;
					}
					cv::cornerSubPix(image, corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

					opticalFlow.push_back(corners);				
				}
			} while(FindNextFile(h,&data));
		}
		FindClose(h);

		// Process image data, compute both the camera intrinsics matrix and the distortion vector
		// Save visualize and, optionally, save the data estimated (depends on -out parameter)
	}

EXIT:
	std::cout << "Hit any key to exit.";
	_gettchar();
	return retResult;
}

