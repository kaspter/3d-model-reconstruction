
#include "stdafx.h"

#include "imp_init.h"
#include "imp_susan.h"
#include "imp_private.h"

namespace imp
{
	IMP_INIT_ALGORITHM(SUSAN, "SUSAN",
						obj.info()->addParam(obj, "paramRadius", obj.paramRadius);
						obj.info()->addParam(obj, "paramT", obj.paramT);
						obj.info()->addParam(obj, "paramG", obj.paramG);
						obj.info()->addParam(obj, "preFilter", obj.preFilter));

	bool initModule ()
	{
		cv::Ptr<cv::Algorithm> susan = createSUSAN_hidden();
		return susan->info() != NULL;
	}
}