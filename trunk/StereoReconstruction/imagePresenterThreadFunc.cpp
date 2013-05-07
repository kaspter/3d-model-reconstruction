#include "stdafx.h"

namespace 
{
	WNDPROC subclassWindowProc	=	NULL;   // Such approach is not considered to be used simultaneously with multiple image-presenter windows
	LRESULT CALLBACK imagePresenterWindowProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
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
}

DWORD CALLBACK imagePresenterThreadFunc(LPVOID threadParameter)
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

		subclassWindowProc = (WNDPROC)SetWindowLong(hwnd, GWL_WNDPROC, (LONG)imagePresenterWindowProc);

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