// precomp.h : include file for standard system include files,
// or project specific include files that are used frequently, but
// are changed infrequently
//

#ifndef _IMP_PRECOMP_H_
#define _IMP_PRECOMP_H_

#pragma once

#if defined IMP_MACROS_EXTENDED
#	if defined WIN32_LEAN_AND_MEAN
#		include <Windows.h>
#	else
#		include <iostream>
#	endif
#endif

#include <vector>
#include <string>
#include <cmath>

#include "opencv2\core\core.hpp"
#include "opencv2\imgproc\imgproc.hpp"
#include "opencv2\nonfree\features2d.hpp"

#endif
