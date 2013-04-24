
#include "stdafx.h"

#include "imp_init.h"
#include "imp_susan.h"
#include "imp_private.h"

namespace imp
{
	class SusanManager : public cv::Algorithm
	{
		typedef _CallProxy<SUSAN, cv::Algorithm> MyProxy;
	public:
		void set_radius(unsigned r) { MyProxy::ref(this).set_radius(r); }
		void set_tparam(double   t) { MyProxy::ref(this).set_tparam(t); }
		void set_gparam(double   g) { MyProxy::ref(this).set_gparam(g); }
	};

	typedef void (cv::Algorithm::*uint_setter) (unsigned);
	typedef void (cv::Algorithm::*dbl_setter)  (double);


	IMP_INIT_ALGORITHM(SUSAN, "Feature2D.SUSAN",
		obj.info()->addParam(obj, "radius",		obj._radius, false, NULL, (uint_setter)&SusanManager::set_radius);
		obj.info()->addParam(obj, "tparam",		obj._tparam, false, NULL, (dbl_setter) &SusanManager::set_tparam);
		obj.info()->addParam(obj, "gparam",		obj._gparam, false, NULL, (dbl_setter) &SusanManager::set_gparam);
		obj.info()->addParam(obj, "prefilter",	obj._prefilter)
	);

	bool initModule ()
	{
		return !SUSAN_info_auto.name().empty();
	}
}