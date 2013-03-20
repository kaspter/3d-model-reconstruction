// FeatureDetection.cpp : Defines the entry point for the application.
//

#include "stdafx.h"
#include "FeatureDetection.h"

#define MAX_LOADSTRING 100

// Global Variables:
HINSTANCE hInst;								// current instance
TCHAR szTitle[MAX_LOADSTRING];					// The title bar text
TCHAR szWindowClass[MAX_LOADSTRING];			// the main window class name

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
					MessageBox(hWnd, _T("Something went wrong and caused out string pointer to be NULL."), _T("Open file dialog failed!"), MB_OK);
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
	ofnData.lpstrFilter = _T("Image files\0*.BMP;*.JPG;*.JPEG;*.PNG;*.GIF;\0"); ofnData.nFilterIndex = 1;
	ofnData.lpstrFile = localBuffer; ofnData.nMaxFile = _countof(localBuffer);
	ofnData.lpfnHook = FileBrowserProc;
	ofnData.lCustData = reinterpret_cast<LPARAM>(&filePath);
	ofnData.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST | OFN_LONGNAMES | OFN_DONTADDTORECENT | OFN_ENABLEHOOK | OFN_EXPLORER;
	ofnData.FlagsEx = OFN_EX_NOPLACESBAR;

	if (!GetOpenFileName(&ofnData)) return TSTRING(); 

	return filePath;
}

BOOL CheckFilePathExtension(LPCTSTR extension)
{
	return FALSE;
}

VOID LoadImageSpecified(TSTRING & imageFileName)
{
}

VOID DisplayImageLoaded()
{
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
VOID SetMenuGroupBoxChecked(HWND hWnd, int (&indexGroup)[_size], int checkedId)
{
	HMENU hMenu = GetMenu(hWnd);
	for (int i = 0; i < _size; ++i) 
	{
		if (checkedId != indexGroup[i]) CheckMenuItem(hMenu, indexGroup[i], MF_BYCOMMAND | MF_UNCHECKED);
	}
	CheckMenuItem(hMenu, checkedId, MF_BYCOMMAND | MF_CHECKED);
}

//
//  FUNCTION: WndProc(HWND, UINT, WPARAM, LPARAM)
//
//  PURPOSE:  Processes messages for the main window.
//
//  WM_COMMAND	- process the application menu
//  WM_PAINT	- Paint the main window
//  WM_DESTROY	- post a quit message and return
//
//
LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	//PAINTSTRUCT ps;
	//HDC hdc;

	switch (message)
	{	
	case WM_COMMAND:
		{
			int wmId    = LOWORD(wParam);
			int wmEvent = HIWORD(wParam);

			BOOL switchMenuViewGroup = FALSE;
			// Parse the menu selections:
			switch (wmId)
			{
			case ID_FILE_OPEN:
				FileBrowserInit(hWnd);
				break;
			case IDM_EXIT:
				DropResources();
				DestroyWindow(hWnd);
				break;

			case ID_VIEW_ORIGINAL: 
				switchMenuViewGroup = TRUE; 
				EnableMenuItem(GetMenu(hWnd), ID_VIEW_FILTERPARAMETERS, MF_BYCOMMAND | MF_DISABLED);
				break;
			case ID_VIEW_FILTERED: 
				switchMenuViewGroup = TRUE; 
				EnableMenuItem(GetMenu(hWnd), ID_VIEW_FILTERPARAMETERS, MF_BYCOMMAND | MF_ENABLED);
				break;

			case IDM_ABOUT:
				DialogBox(hInst, MAKEINTRESOURCE(IDD_ABOUTBOX), hWnd, About);
				break;
			}

			if (switchMenuViewGroup) 
			{
				int menuViewGroupSwitchboxes[] = { ID_VIEW_ORIGINAL, ID_VIEW_FILTERED };
				SetMenuGroupBoxChecked(hWnd, menuViewGroupSwitchboxes, wmId);
			}
		} break;
	//case WM_PAINT:
	//	hdc = BeginPaint(hWnd, &ps);
	//	// TODO: Add any drawing code here...
	//	EndPaint(hWnd, &ps);
	//	break;
	case WM_DESTROY:
		PostQuitMessage(0);
		return 0;		
	}

	return DefWindowProc(hWnd, message, wParam, lParam);
}

//
//  FUNCTION: WindowClassRegister()
//
//  PURPOSE: Registers the window class.
//
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

//
//   FUNCTION: InitInstance(HINSTANCE, int)
//
//   PURPOSE: Saves instance handle and creates main window
//
//   COMMENTS:
//
//        In this function, we save the instance handle in a global variable and
//        create and display the main program window.
//
BOOL InitInstance(HINSTANCE hInstance, int nCmdShow)
{
   HWND hWnd;

   hInst = hInstance; // Store instance handle in our global variable

   hWnd = CreateWindow(szWindowClass, szTitle, WS_OVERLAPPEDWINDOW & ~WS_MAXIMIZEBOX,
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
