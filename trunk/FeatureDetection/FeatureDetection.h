#pragma once

#include "resource.h"
#include "windowsx.h"

#define MAIN_WINDOW_CLASS	_T("MainWindowClass")
#define CANVAS_WINDOW_CLASS _T("CanvasWIndowClass")

///* VOID Cls_OnMouseleave(HWND hwnd) */
//#define HANDLE_WM_MOUSELEAVE(hwnd, wParam, lParam, fn) \
//    ((fn)((hwnd)), 0L)
//#define FORWARD_WM_MOUSELEAVE(hwnd, lpCreateStruct, fn) \
//    (VOID)(DWORD)(fn)((hwnd), WM_MOUSELEAVE, 0L, 0L)

/* void Cls_OnMouseHover(HWND hwnd, int x, int y, UINT keyFlags) */
#define HANDLE_WM_MOUSEHOVER(hwnd, wParam, lParam, fn) \
    ((fn)((hwnd), (int)(short)LOWORD(lParam), (int)(short)HIWORD(lParam), (UINT)(wParam)), 0L)
#define FORWARD_WM_MOUSEHOVER(hwnd, x, y, keyFlags, fn) \
    (void)(fn)((hwnd), WM_MOUSEHOVER, (WPARAM)(UINT)(keyFlags), MAKELPARAM((x), (y)))

typedef std::basic_string<TCHAR> TSTRING, *LPTSTRING; 

#ifdef _UNICODE
#define TTTOOLINFO_V2_SIZE TTTOOLINFOW_V2_SIZE
#else
#define TTTOOLINFO_V2_SIZE TTTOOLINFOA_V2_SIZE
#endif

#define ID_DETECTION_STATUS 3999
#define ID_COORD_TOOLTIP    4000