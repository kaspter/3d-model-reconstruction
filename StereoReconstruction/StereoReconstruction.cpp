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
	std::vector<cv::Mat> output;

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
	{
		// Image features recognition and description block
		if (!cv::initModule_nonfree())
		{
			std::cerr << "Failed to init Non-Free Features2d module" << std::endl; goto EXIT;
		}

		std::string algorithm_name = "SIFT";
		cv::Ptr<cv::Feature2D> algorithm = cv::Feature2D::create(algorithm_name);
		
		output.resize(files.size());

		std::vector<cv::Mat>					descriptors(files.size());
		std::vector<std::vector<cv::KeyPoint>>	keypoints(files.size());
		for (int i = 0, imax = files.size(); i < imax; ++i)
		{
			(*algorithm)(images[i], cv::Mat(), keypoints[i], descriptors[i]);
			cv::drawKeypoints(images[i], keypoints[i], output[i], cv::Scalar(0x000000FF));
		}


	}

	HANDLE threadPresenter = CreateThread(NULL, 0, presenterThreadFunc, &output[1], 0x00, NULL);
	WaitForSingleObject(threadPresenter, INFINITE);
	CloseHandle(threadPresenter);

EXIT:
	std::cout << "Success! Hit any key to exit.";
	
	_gettchar();
	return retResult;
}

