
#include "stdafx.h"

#include "imp_init.h"
#include "imp_susan.h"
#include "imp_private.h"

namespace imp
{
	IMP_INIT_ALGORITHM(SUSAN, "Feature2D.SUSAN",
		obj.info()->addParam(obj, "radius",		obj._radius, false, NULL, &SUSAN::_set_radius);
		obj.info()->addParam(obj, "tparam",		obj._tparam, false, NULL, &SUSAN::_set_tparam);
		obj.info()->addParam(obj, "gparam",		obj._gparam, false, NULL, &SUSAN::_set_gparam);
		obj.info()->addParam(obj, "prefilter",	obj._prefilter)
	);

	bool initModule ()
	{
		return !SUSAN_info_auto.name().empty();
	}
}