// FeatureDetection.cpp : Defines the entry point for the application.
//

#include "stdafx.h"
#include "FeatureDetection.h"

// Global Variables:
HINSTANCE hInst;								// current instance
LPTSTR szTitle = _T("Feature Detection");		// The title bar text

HWND hcanvas, hstatus, htip;
TOOLINFO ti = { 0 };

// Different images (original and filteredImage grayscale)
cv::Mat originalImage, grayscaleImage, filteredImage;
cv::Mat *displayedImage = NULL;

#define GRAYSCALE_PAL_LEVELS ((USHORT)256)
BYTE bitmapInfoData[sizeof(BITMAPINFO) + sizeof(RGBQUAD) * GRAYSCALE_PAL_LEVELS] = { 0 };
BITMAPINFO &bmi = *reinterpret_cast<LPBITMAPINFO>(bitmapInfoData);

MINMAXINFO mmWnd = { 0 };
POINT imageCenter = { 0 };

UINT_PTR CALLBACK FileBrowserProc(HWND hdlg, UINT uiMsg, WPARAM wParam, LPARAM lParam)
{
	switch(uiMsg)
	{
	case WM_NOTIFY:
		{
			HWND hWnd = GetParent(hdlg);
			LPTSTRING lpOutString = reinterpret_cast<LPTSTRING>(reinterpret_cast<LPOFNOTIFY>(lParam)->lpOFN->lCustData);
			switch (reinterpret_cast<LPOFNOTIFY>(lParam)->hdr.code)
			{
			case CDN_INITDONE:
				if (lpOutString == NULL)
				{
					MessageBox(hWnd, _T("Something went wrong and caused out string pointer to be NULL."), _T("Open file dialog failed!"), MB_OK | MB_ICONWARNING);
					PostMessage(hWnd, WM_COMMAND, MAKEWPARAM(IDCANCEL, 0), (LPARAM)hWnd);
				}
				break;
			case CDN_FILEOK: 
				{
					int bufferSize = CommDlg_OpenSave_GetFilePath(hWnd, lpOutString->c_str(), lpOutString->length());
					if (bufferSize > static_cast<int>(lpOutString->length()))
					{
						lpOutString->resize(bufferSize);
						CommDlg_OpenSave_GetFilePath(hWnd, lpOutString->c_str(), lpOutString->length());
					}
				} break;
			}
			SetWindowLong(GetParent(hdlg), DWL_MSGRESULT, TRUE);
		} break;
	}

	return (UINT_PTR)FALSE;
}

TSTRING FileBrowserInit(HWND hwndOwner)
{
	TSTRING filePath = TSTRING();

	TCHAR localBuffer[4] = _T("\0");

	OPENFILENAME ofnData = {0};
	ofnData.lStructSize = sizeof(OPENFILENAME);
	ofnData.hwndOwner = hwndOwner;
	ofnData.lpstrFilter = _T("Image files\0*.BMP;*.DIB;*.JPG;*.JPEG;*.PNG;*.TIFF;*.TIF;\0"); ofnData.nFilterIndex = 1;
	ofnData.lpstrFile = localBuffer; ofnData.nMaxFile = _countof(localBuffer);
	ofnData.lpfnHook = FileBrowserProc;
	ofnData.lCustData = reinterpret_cast<LPARAM>(&filePath);
	ofnData.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST | OFN_LONGNAMES | OFN_DONTADDTORECENT | OFN_ENABLEHOOK | OFN_EXPLORER;
	ofnData.FlagsEx = OFN_EX_NOPLACESBAR;

	if (!GetOpenFileName(&ofnData)) return TSTRING(); 

	return filePath;
}

BOOL LoadImageSpecified(HWND hwnd, TSTRING & imageFileName)
{
	LPTSTRING errMsg = NULL;

#ifdef _UNICODE
	std::string mbFileName = std::string(imageFileName.length() + 1, '\0');
	size_t numOfCharsConverted;
	wcstombs_s(&numOfCharsConverted, &mbFileName[0], mbFileName.length(), imageFileName.c_str(), _TRUNCATE);
#else
	std::string mbFileName = imageFileName;
#endif

	cv::Mat rawImage = cv::imread(mbFileName);
	if (rawImage.data != NULL) 
	{
		// TODO: Process rawImage so it's width became aligned to sizeof(LONG)
		if (!originalImage.empty()) { delete[] originalImage.data; originalImage.release(); }
		ULONG alignedRowSize = ((rawImage.cols * rawImage.elemSize() - 1) & (~sizeof(DWORD) + 1)) + sizeof(DWORD);
		originalImage = cv::Mat(rawImage.rows, rawImage.cols, rawImage.type(), new BYTE[rawImage.rows * alignedRowSize], alignedRowSize);
		rawImage.copyTo(originalImage);

		/* Grayscale image preparation */
		cv::cvtColor(rawImage, rawImage, CV_BGR2GRAY); //rawImage = filteredImage;
		if (!grayscaleImage.empty()) { delete[] grayscaleImage.data; grayscaleImage.release(); }
		alignedRowSize = ((rawImage.cols * rawImage.elemSize() - 1) & (~sizeof(DWORD) + 1)) + sizeof(DWORD);
		grayscaleImage = cv::Mat(rawImage.rows, rawImage.cols, rawImage.type(), new BYTE[rawImage.rows * alignedRowSize], alignedRowSize);
		rawImage.copyTo(grayscaleImage);

		filteredImage.release();

		// Temporary.
		filteredImage = grayscaleImage;

		bmi.bmiHeader.biWidth = rawImage.cols;
		bmi.bmiHeader.biHeight = -rawImage.rows;

		RECT client, window, status;
		GetClientRect(hwnd, &client);
		GetWindowRect(hwnd, &window);
		GetWindowRect(GetDlgItem(hwnd, ID_DETECTION_STATUS), &status);

		cv::Size imageSize = rawImage.size();
		cv::Size frameSize = cv::Size(window.right - window.left - client.right + client.left, window.bottom - window.top - client.bottom + client.top) + imageSize;
		SetWindowPos(hwnd, NULL, 0, 0, frameSize.width, frameSize.height + status.bottom - status.top, SWP_NOMOVE | SWP_NOZORDER);
			
		GetClientRect(hwnd, &client);
		imageCenter.x = (client.right - client.left - imageSize.width) >> 1;
		imageCenter.y = (client.bottom + client.top - imageSize.height - status.bottom + status.top) >> 1;
		SetWindowPos(hcanvas, NULL, imageCenter.x, imageCenter.y, imageSize.width, imageSize.height, SWP_NOZORDER);
		ShowWindow(hcanvas, SW_SHOW);

		TSTRING windowText = TSTRING(_tcslen(szTitle) + imageFileName.length() + 4, _T('\0'));
		_stprintf_s(const_cast<LPTSTR>(windowText.c_str()), windowText.length(), _T("%s - %s"), szTitle, imageFileName.c_str());
		SetWindowText(hwnd, windowText.c_str());

		EnableMenuItem(GetMenu(hwnd), 1, MF_BYPOSITION | MF_ENABLED);
		PostMessage(hwnd, WM_COMMAND, MAKEWPARAM(ID_VIEW_ORIGINAL, 0), 0);
	}
	else
	{ 
		errMsg = new TSTRING(_T("OpenCV failed to open an image specified!"));
	}

	BOOL success = errMsg == NULL;
	if (!success) { MessageBox(hwnd, errMsg->c_str(), _T("Failed to load an image!"), MB_OK | MB_ICONWARNING); delete errMsg; }

	return success;
}

template <int _size>
VOID SetMenuGroupBoxChecked(HWND hwnd, int (&indexGroup)[_size], int checkedId)
{
	HMENU hMenu = GetMenu(hwnd);
	for (int i = 0; i < _size; ++i) 
	{
		if (checkedId != indexGroup[i]) CheckMenuItem(hMenu, indexGroup[i], MF_BYCOMMAND | MF_UNCHECKED);
	}
	CheckMenuItem(hMenu, checkedId, MF_BYCOMMAND | MF_CHECKED);
}

INT_PTR CALLBACK AboutBoxDlgProc(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam)
{
	UNREFERENCED_PARAMETER(lParam);
	switch (message)
	{
	case WM_INITDIALOG:
		return (INT_PTR)TRUE;

	case WM_COMMAND:
		if (LOWORD(wParam) == IDOK || LOWORD(wParam) == IDCANCEL)
		{
			EndDialog(hDlg, LOWORD(wParam));
			return (INT_PTR)TRUE;
		}
		break;
	}
	return (INT_PTR)FALSE;
}

BOOL OnCreateMain(HWND hwnd, LPCREATESTRUCT lpCreateStruct)
{
	// Status bar creation
	RECT client;
	GetClientRect(hwnd, &client);
	hcanvas = CreateWindow(CANVAS_WINDOW_CLASS, NULL, WS_CHILDWINDOW, 0, 0, client.right, client.top, hwnd, NULL, hInst, NULL);
	hstatus = CreateWindow(STATUSCLASSNAME, _T("No image loaded"), WS_CHILD | WS_VISIBLE | CCS_BOTTOM, 0, 0, 0, 0, hwnd, (HMENU)ID_DETECTION_STATUS, hInst, NULL); SendMessage(hstatus, SB_SIMPLE, 0, 0);
	htip	= CreateWindowEx(WS_EX_TOPMOST, TOOLTIPS_CLASS, NULL, WS_POPUP | TTS_NOPREFIX | TTS_ALWAYSTIP, CW_USEDEFAULT, CW_USEDEFAULT, CW_USEDEFAULT, CW_USEDEFAULT, hcanvas, NULL, hInst, NULL);

	ti.cbSize = TTTOOLINFO_V2_SIZE;
	ti.uFlags = TTF_IDISHWND | TTF_TRACK | TTF_ABSOLUTE;
	ti.uId = (UINT_PTR)hcanvas;
	ti.hwnd = hcanvas;
	ti.lpszText = LPSTR_TEXTCALLBACK;

	BOOL test = SendMessage(htip, TTM_ADDTOOL, 0, (LPARAM)&ti);

	// Internal business-logic structures initialization
	bmi.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
	bmi.bmiHeader.biPlanes = 1;
	bmi.bmiHeader.biCompression = BI_RGB;
	for (USHORT i = 0; i < GRAYSCALE_PAL_LEVELS; ++i) 
	{
		FillMemory(&bmi.bmiColors[i], sizeof(RGBQUAD) - 1, (BYTE)i);
	}

	SendMessage(hwnd, WM_GETMINMAXINFO, 0, (LPARAM)&mmWnd);

	return TRUE;
}
VOID OnCommandMain(HWND hwnd, int id, HWND hwndCtl, UINT codeNotify)
{
	BOOL switchMenuViewGroup = FALSE;
	// Parse the menu selections:
	switch (id)
	{
	case ID_FILE_OPEN:
		{
			TSTRING imagePath = FileBrowserInit(hwnd);
			if (!imagePath.empty())	LoadImageSpecified(hwnd, imagePath);
		} break;
	case IDM_EXIT:
		DestroyWindow(hwnd);
		break;

	case ID_VIEW_ORIGINAL: 
		switchMenuViewGroup = TRUE; 
		EnableMenuItem(GetMenu(hwnd), ID_VIEW_FILTERPARAMETERS, MF_BYCOMMAND | MF_DISABLED);
		displayedImage = &originalImage;
		break;
	case ID_VIEW_FILTERED: 
		switchMenuViewGroup = TRUE; 
		EnableMenuItem(GetMenu(hwnd), ID_VIEW_FILTERPARAMETERS, MF_BYCOMMAND | MF_ENABLED);
		displayedImage = &filteredImage;
		break;

	case IDM_ABOUT:
		DialogBox(hInst, MAKEINTRESOURCE(IDD_ABOUTBOX), hwnd, AboutBoxDlgProc);
		break;

	default:
		FORWARD_WM_COMMAND(hwnd, id, hwndCtl, codeNotify, DefWindowProc);
	}

	if (switchMenuViewGroup) 
	{
		int menuViewGroupSwitchboxes[] = { ID_VIEW_ORIGINAL, ID_VIEW_FILTERED };
		SetMenuGroupBoxChecked(hwnd, menuViewGroupSwitchboxes, id);

		// TODO: place image output function here or post redraw message
		bmi.bmiHeader.biBitCount = displayedImage->elemSize() * 8;

		RedrawWindow(hcanvas, NULL, NULL, RDW_ERASE | RDW_INVALIDATE);

		if (id != ID_VIEW_FILTERED) { /* TODO: Filter settings toolbox hiding operations code must be placed here */ }
	}
}
VOID OnDropFilesMain(HWND hwnd, HDROP hdrop)
{
	UINT fmax = DragQueryFile(hdrop, UINT(-1), NULL, 0);
	for (UINT f = 0; f < fmax; ++f)
	{
		UINT strLen = DragQueryFile(hdrop, f, NULL, 0);
		if (strLen++ != 0)
		{
			TSTRING fileName = TSTRING(strLen, _T('\0'));
			DragQueryFile(hdrop, f, const_cast<LPTSTR>(fileName.c_str()), fileName.length());
			if (LoadImageSpecified(hwnd, fileName)) return;
		}
		else
		{
			LPTSTR errText = _T("Drag'n'drop operation crashed with code: %u!");
			TSTRING errMsg = TSTRING(_tcslen(errText) + 11, _T('\0'));
			_stprintf_s(const_cast<LPTSTR>(errMsg.c_str()), errMsg.capacity(), errText, GetLastError());
			MessageBox(hwnd, errMsg.c_str(), _T("Drag and drop failure!"), MB_OK | MB_ICONWARNING);
			return;
		}
	}

	MessageBox(hwnd, _T("No files of dropped ones are images that can be loaded!"), _T("Warning!"), MB_OK | MB_ICONASTERISK);
}
VOID OnSizeMain(HWND hwnd, UINT state, int cx, int cy)
{
	SendMessage(GetDlgItem(hwnd, ID_DETECTION_STATUS), WM_SIZE, 0, 0);
	FORWARD_WM_SIZE(hwnd, state, cx, cy, DefWindowProc);
}
VOID OnDestroyMain(HWND hwnd)
{
	DestroyWindow(hcanvas);
	DestroyWindow(hstatus);
	DestroyWindow(htip);

	PostQuitMessage(0);
	FORWARD_WM_DESTROY(hwnd, DefWindowProc);
}
LRESULT CALLBACK MainWindowProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	switch (message)
	{
		HANDLE_MSG(hWnd, WM_CREATE, OnCreateMain);
		HANDLE_MSG(hWnd, WM_COMMAND, OnCommandMain);
		//HANDLE_MSG(hWnd, WM_NOTIFY, OnNotify);
		HANDLE_MSG(hWnd, WM_DROPFILES, OnDropFilesMain);
		HANDLE_MSG(hWnd, WM_SIZE, OnSizeMain);
		//HANDLE_MSG(hWnd, WM_MOUSEMOVE, OnMouseMove);
		//HANDLE_MSG(hWnd, WM_PAINT, OnPaint);

		//case WM_MOUSELEAVE:
		//	SendMessage(GetDlgItem(hWnd, ID_DETECTION_STATUS), SB_SETTEXT, MAKEWPARAM(0, 0), (LPARAM)NULL);
		//	break;

		HANDLE_MSG(hWnd, WM_DESTROY, OnDestroyMain);
	}

	return DefWindowProc(hWnd, message, wParam, lParam);
}

LRESULT OnNotifyCanvas(HWND hwnd, int nId, LPNMHDR lpNmhdr)
{
	switch (lpNmhdr->code)
	{
	case TTN_GETDISPINFO:
		{
			POINT cursorPos;
			GetCursorPos(&cursorPos);
			ScreenToClient(hwnd, &cursorPos);

			// 80 symbols MAX including '\0'
			_stprintf_s(((LPNMTTDISPINFO)lpNmhdr)->lpszText, 0x50, _T("%u @ %u;%u"), grayscaleImage.at<uchar>(cv::Point(cursorPos.x, cursorPos.y)), cursorPos.x, cursorPos.y);
		} break;
	}

	return FORWARD_WM_NOTIFY(hwnd, nId, lpNmhdr, DefWindowProc);
}
VOID OnMouseMoveCanvas(HWND hwnd, int x, int y, UINT keyFlags)
{
	TCHAR coordText[0x20] = { 0 };
	_stprintf_s(coordText, _T("%u @ %u;%u i/px"), grayscaleImage.at<uchar>(cv::Point(x, y)), x, y);
	SendMessage(GetDlgItem(GetParent(hwnd), ID_DETECTION_STATUS), SB_SETTEXT, MAKEWPARAM(0, 0), (LPARAM)coordText);

	if (ti.lParam != TRUE) 
	{
		SendMessage(htip, TTM_TRACKACTIVATE, FALSE, (LPARAM)&ti);
	}
	else
	{
		ti.lParam = FALSE;
	}

	TRACKMOUSEEVENT tme = { sizeof(TRACKMOUSEEVENT), TME_LEAVE | TME_HOVER, hwnd, HOVER_DEFAULT };
	TrackMouseEvent(&tme);
}
VOID OnMouseHoverCanvas(HWND hwnd, int x, int y, UINT keyFlags)
{	
	POINT absolute = { x + 10, y + 10 };
	ClientToScreen(hwnd, &absolute);
	SendMessage(htip, TTM_TRACKPOSITION, 0, MAKELPARAM(absolute.x, absolute.y));
	SendMessage(htip, TTM_TRACKACTIVATE, ti.lParam = TRUE, (LPARAM)&ti);
}
VOID OnMouseLeaveCanvas(HWND hwnd)
{
	SendMessage(GetDlgItem(GetParent(hwnd), ID_DETECTION_STATUS), SB_SETTEXT, MAKEWPARAM(0, 0), (LPARAM)NULL);
}
VOID OnPaintCanvas(HWND hwnd)
{
	if (displayedImage != NULL)
	{
		PAINTSTRUCT ps;
		HDC hdc = BeginPaint(hwnd, &ps);	
		SetDIBitsToDevice(hdc, ps.rcPaint.left, ps.rcPaint.top, ps.rcPaint.right - ps.rcPaint.left, ps.rcPaint.bottom - ps.rcPaint.top, 
			ps.rcPaint.left, ps.rcPaint.top, ps.rcPaint.top, ps.rcPaint.bottom, displayedImage->data, &bmi, DIB_RGB_COLORS);
		EndPaint(hwnd, &ps);
		return;
	}

	FORWARD_WM_PAINT(hwnd, DefWindowProc);
}
LRESULT CALLBACK CanvasWindowProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	switch (message)
	{
		HANDLE_MSG(hWnd, WM_NOTIFY, OnNotifyCanvas);
		HANDLE_MSG(hWnd, WM_MOUSEMOVE, OnMouseMoveCanvas);
		HANDLE_MSG(hWnd, WM_MOUSEHOVER, OnMouseHoverCanvas);
		HANDLE_MSG(hWnd, WM_MOUSELEAVE, OnMouseLeaveCanvas);
		HANDLE_MSG(hWnd, WM_PAINT, OnPaintCanvas);
	}

	return DefWindowProc(hWnd, message, wParam, lParam);
}

VOID WindowClassRegister(HINSTANCE hInstance)
{
	WNDCLASSEX wcex = { 0 };

	wcex.cbSize = sizeof(WNDCLASSEX);

	wcex.style			= CS_HREDRAW | CS_VREDRAW;
	wcex.lpfnWndProc	= MainWindowProc;
	wcex.hInstance		= hInstance;
	wcex.hIcon			= LoadIcon(hInstance, MAKEINTRESOURCE(IDI_FEATUREDETECTION));
	wcex.hCursor		= LoadCursor(NULL, IDC_ARROW);
	wcex.hbrBackground	= (HBRUSH)(COLOR_3DLIGHT);
	wcex.lpszMenuName	= MAKEINTRESOURCE(IDC_FEATUREDETECTION);
	wcex.lpszClassName	= MAIN_WINDOW_CLASS;
	wcex.hIconSm		= LoadIcon(wcex.hInstance, MAKEINTRESOURCE(IDI_SMALL));
	RegisterClassEx(&wcex);

	wcex.lpfnWndProc	= CanvasWindowProc;
	wcex.hIcon			= NULL;
	wcex.hCursor		= LoadCursor(NULL, IDC_CROSS);
	wcex.lpszMenuName	= NULL;
	wcex.lpszClassName	= CANVAS_WINDOW_CLASS;
	wcex.hIconSm		= NULL;
	RegisterClassEx(&wcex);
}
BOOL InitInstance(HINSTANCE hInstance, int nCmdShow)
{
   HWND hWnd;

   hInst = hInstance; // Store instance handle in our global variable

   hWnd = CreateWindowEx(WS_EX_ACCEPTFILES | WS_EX_STATICEDGE, MAIN_WINDOW_CLASS, szTitle, (WS_OVERLAPPEDWINDOW | WS_CLIPCHILDREN) & ~WS_THICKFRAME & ~WS_MAXIMIZEBOX,
      CW_USEDEFAULT, 0, 640, 480, NULL, NULL, hInstance, NULL);

   if (!hWnd)
   {
      return FALSE;
   }

   ShowWindow(hWnd, nCmdShow);
   UpdateWindow(hWnd);

   return TRUE;
}

int APIENTRY _tWinMain(_In_ HINSTANCE hInstance,
						 _In_opt_ HINSTANCE hPrevInstance,
						 _In_ LPTSTR    lpCmdLine,
						 _In_ int       nCmdShow)
{
	UNREFERENCED_PARAMETER(hPrevInstance);
	UNREFERENCED_PARAMETER(lpCmdLine);

	InitCommonControls();

	WindowClassRegister(hInstance);
	if (!InitInstance (hInstance, nCmdShow)) return FALSE;

	MSG msg;
	HACCEL hAccelTable = LoadAccelerators(hInstance, MAKEINTRESOURCE(IDC_FEATUREDETECTION));
	while (GetMessage(&msg, NULL, 0, 0))
	{
		if (!TranslateAccelerator(msg.hwnd, hAccelTable, &msg))
		{
			TranslateMessage(&msg);
			DispatchMessage(&msg);
		}
	}

	return (int) msg.wParam;
}
