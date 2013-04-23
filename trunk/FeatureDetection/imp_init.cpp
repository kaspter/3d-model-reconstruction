#include "stdafx.h"
#include "imp_init.h"

namespace imp
{
	IMP_INIT_ALGORITHM(SusanDetector, "SUSAN",
					  obj.info()->addParam(obj, "paramT", obj.paramT);
					  obj.info()->addParam(obj, "paramG", obj.paramG);
					  obj.info()->addParam(obj, "paramRadius", obj.paramRadius));

	bool initModule ()
	{
		cv::Ptr<cv::Algorithm> susan = createSusanDetector_hidden();
		return susan->info() != NULL;
	}
}