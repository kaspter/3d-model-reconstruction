// FeatureDetection.cpp : Defines the entry point for the application.
//

#include "stdafx.h"
#include "FeatureDetection.h"

#define MAX_LOADSTRING 100

// Global Variables:
HINSTANCE hInst;								// current instance
TCHAR szTitle[MAX_LOADSTRING];					// The title bar text
TCHAR szWindowClass[MAX_LOADSTRING];			// the main window class name

// Different images (original and filtered grayscale)
cv::Mat source, filtered;
cv::Mat *imageShown = NULL;

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
	if (imageFileName.empty()) return FALSE;

	LPTSTRING errMsg = NULL;

#ifdef _UNICODE
	std::string mbFileName = std::string(imageFileName.length() + 1, '\0');
	size_t numOfCharsConverted;
	wcstombs_s(&numOfCharsConverted, &mbFileName[0], mbFileName.length(), imageFileName.c_str(), _TRUNCATE);
#else
	std::string mbFileName = imageFileName;
#endif

	cv::Mat loaded = cv::imread(mbFileName);
	if (loaded.data != NULL) 
	{
		source = loaded;
		cv::cvtColor(loaded, filtered, CV_BGR2GRAY);
	
		bmi.bmiHeader.biWidth = loaded.cols;
		bmi.bmiHeader.biHeight = -loaded.rows;

		RECT client, window;
		GetClientRect(hwnd, &client);
		GetWindowRect(hwnd, &window);

		cv::Size imageSize = loaded.size();
		cv::Size frameSize = cv::Size(window.right - window.left - client.right + client.left, window.bottom - window.top - client.bottom + client.top) + imageSize;
		SetWindowPos(hwnd, NULL, 0, 0, frameSize.width, frameSize.height, SWP_NOMOVE | SWP_NOZORDER);

		GetClientRect(hwnd, &client);
		imageCenter.x = (client.right - client.left - imageSize.width) >> 1;
		imageCenter.y = (client.bottom + client.top - imageSize.height) >> 1;

		TSTRING windowText = TSTRING(_tcslen(szTitle) + imageFileName.length() + 4, _T('\0'));
		_stprintf_s(const_cast<LPTSTR>(windowText.c_str()), windowText.length(), _T("%s - %s"), szTitle, imageFileName.c_str());
		SetWindowText(hwnd, windowText.c_str());

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

VOID DropResources() 
{ 
}

// Message handler for about box.
INT_PTR CALLBACK About(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam)
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


BOOL OnCreate(HWND hwnd, LPCREATESTRUCT lpCreateStruct)
{
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

void OnCommand(HWND hwnd, int id, HWND hwndCtl, UINT codeNotify)
{
	BOOL switchMenuViewGroup = FALSE;
	// Parse the menu selections:
	switch (id)
	{
	case ID_FILE_OPEN:
		LoadImageSpecified(hwnd, FileBrowserInit(hwnd));
		EnableMenuItem(GetMenu(hwnd), 1, MF_BYPOSITION | MF_ENABLED);
		break;
	case IDM_EXIT:
		DropResources();
		DestroyWindow(hwnd);
		break;

	case ID_VIEW_ORIGINAL: 
		switchMenuViewGroup = TRUE; 
		EnableMenuItem(GetMenu(hwnd), ID_VIEW_FILTERPARAMETERS, MF_BYCOMMAND | MF_DISABLED);
		imageShown = &source;
		break;
	case ID_VIEW_FILTERED: 
		switchMenuViewGroup = TRUE; 
		EnableMenuItem(GetMenu(hwnd), ID_VIEW_FILTERPARAMETERS, MF_BYCOMMAND | MF_ENABLED);
		imageShown = &filtered;
		break;

	case IDM_ABOUT:
		DialogBox(hInst, MAKEINTRESOURCE(IDD_ABOUTBOX), hwnd, About);
		break;

	default:
		FORWARD_WM_COMMAND(hwnd, id, hwndCtl, codeNotify, DefWindowProc);
	}

	if (switchMenuViewGroup) 
	{
		int menuViewGroupSwitchboxes[] = { ID_VIEW_ORIGINAL, ID_VIEW_FILTERED };
		SetMenuGroupBoxChecked(hwnd, menuViewGroupSwitchboxes, id);

		// TODO: place image output function here or post redraw message
		bmi.bmiHeader.biBitCount = imageShown->elemSize() * 8;

		RedrawWindow(hwnd, NULL, NULL, RDW_ERASE | RDW_INVALIDATE);

		if (id != ID_VIEW_FILTERED) { /* TODO: Filter settings toolbox hiding operations code must be placed here */ }
	}
}

void OnDropFiles(HWND hwnd, HDROP hdrop)
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

void OnPaint(HWND hwnd)
{
	if (imageShown != NULL)
	{
		PAINTSTRUCT ps;
		HDC hdc = BeginPaint(hwnd, &ps);	
		SIZE paintSize = { ps.rcPaint.right - ps.rcPaint.left, ps.rcPaint.bottom - ps.rcPaint.top };
		SetDIBitsToDevice(hdc, MAX(imageCenter.x, ps.rcPaint.left), MAX(imageCenter.y, ps.rcPaint.top), paintSize.cx, paintSize.cy, 
			ps.rcPaint.left, ps.rcPaint.top, ps.rcPaint.top, ps.rcPaint.bottom, imageShown->data, &bmi, DIB_RGB_COLORS);
		EndPaint(hwnd, &ps);
		return;
	}

	FORWARD_WM_PAINT(hwnd, DefWindowProc);
}

LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	switch (message)
	{
		HANDLE_MSG(hWnd, WM_CREATE, OnCreate);
		HANDLE_MSG(hWnd, WM_COMMAND, OnCommand);
		HANDLE_MSG(hWnd, WM_DROPFILES, OnDropFiles);
		HANDLE_MSG(hWnd, WM_PAINT, OnPaint);

		case WM_DESTROY:
			PostQuitMessage(0);
			break;		
	}

	return DefWindowProc(hWnd, message, wParam, lParam);
}

ATOM WindowClassRegister(HINSTANCE hInstance)
{
	WNDCLASSEX wcex;

	wcex.cbSize = sizeof(WNDCLASSEX);

	wcex.style			= CS_HREDRAW | CS_VREDRAW;
	wcex.lpfnWndProc	= WndProc;
	wcex.cbClsExtra		= 0;
	wcex.cbWndExtra		= 0;
	wcex.hInstance		= hInstance;
	wcex.hIcon			= LoadIcon(hInstance, MAKEINTRESOURCE(IDI_FEATUREDETECTION));
	wcex.hCursor		= LoadCursor(NULL, IDC_ARROW);
	wcex.hbrBackground	= (HBRUSH)(COLOR_WINDOW+1);
	wcex.lpszMenuName	= MAKEINTRESOURCE(IDC_FEATUREDETECTION);
	wcex.lpszClassName	= szWindowClass;
	wcex.hIconSm		= LoadIcon(wcex.hInstance, MAKEINTRESOURCE(IDI_SMALL));

	return RegisterClassEx(&wcex);
}

BOOL InitInstance(HINSTANCE hInstance, int nCmdShow)
{
   HWND hWnd;

   hInst = hInstance; // Store instance handle in our global variable

   hWnd = CreateWindowEx(WS_EX_ACCEPTFILES | WS_EX_STATICEDGE, szWindowClass, szTitle, WS_OVERLAPPEDWINDOW & ~WS_THICKFRAME & ~WS_MAXIMIZEBOX,
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

	_tsetlocale(LC_ALL, _T("Russian_rus.1251"));

 	// TODO: Place code here.
	MSG msg;
	HACCEL hAccelTable;

	// Initialize global strings
	LoadString(hInstance, IDS_APP_TITLE, szTitle, MAX_LOADSTRING);
	LoadString(hInstance, IDC_FEATUREDETECTION, szWindowClass, MAX_LOADSTRING);
	WindowClassRegister(hInstance);

	// Perform application initialization:
	if (!InitInstance (hInstance, nCmdShow)) return FALSE;

	hAccelTable = LoadAccelerators(hInstance, MAKEINTRESOURCE(IDC_FEATUREDETECTION));

	// Main message loop:
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
