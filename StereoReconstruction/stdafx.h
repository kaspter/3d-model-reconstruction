// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently, but
// are changed infrequently
//

#pragma once

#pragma comment(lib, "vtksys-gd.lib")
#pragma comment(lib, "vtkCommon-gd.lib")
#pragma comment(lib, "vtkFiltering-gd.lib")
#pragma comment(lib, "pcl_common_debug.lib")
#pragma comment(lib, "pcl_visualization_debug.lib")

#include "targetver.h"

#define WIN32_LEAN_AND_MEAN             // Exclude rarely-used stuff from Windows headers
// Windows Header Files:
#include <windows.h>

// C RunTime Header Files
#include <stdlib.h>
#include <malloc.h>
#include <memory.h>
#include <tchar.h>

// TODO: reference additional headers your program requires here
#include <iostream>
#include <algorithm>
#include <string>
#include <cmath>

#include "opencv2\core\core.hpp"
#include "opencv2\imgproc\imgproc.hpp"
#include "opencv2\nonfree\features2d.hpp"
#include "opencv2\nonfree\nonfree.hpp"
#include "opencv2\highgui\highgui.hpp"
#include "opencv2\calib3d\calib3d.hpp"

#include "boost\thread\thread.hpp"
#include "pcl\visualization\cloud_viewer.h"

// TODO: reference additional headers your program requires here
#include "imp_init.h"
#include "imp_susan.h"

#define  IMP_MACROS_EXTENDED
#include "imp_extension.h"
